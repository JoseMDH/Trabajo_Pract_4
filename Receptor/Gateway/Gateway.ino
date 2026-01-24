/* ---------------------------------------------------------------------
 *  Gateway LoRa-Serial para Arduino MKR WAN 1310
 *  =============================================
 *  
 *  VERSIÓN SIMPLIFICADA con ACK bloqueante (polling)
 *  
 *  Este sketch convierte el Arduino en un gateway bidireccional:
 *  - Recibe mensajes LoRa y los envía por Serial a la Raspberry Pi
 *  - Recibe comandos por Serial y los transmite por LoRa con ACK
 *  
 *  Protocolo Serial:
 *  STX(0x02) | tipo(1) | topic_len(1) | topic | payload_len(1) | payload | ETX(0x03)
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

// =====================
// Configuración Serial
// =====================
#define SERIAL_PI Serial1
#define SERIAL_DEBUG Serial

// =====================
// Configuración LoRa
// =====================
const uint8_t localAddress = 0x05;       // Gateway
const uint8_t defaultDestination = 0x06; // Actuador
const uint8_t ACK_MARKER = 0xAC;

typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3};

LoRaConfig_t nodeConfig = {6, 10, 5, 2};  // BW=62.5kHz, SF=10, CR=4/5, TxPwr=2dBm

// =====================
// Protocolo Serial
// =====================
#define STX 0x02
#define ETX 0x03

#define MSG_TYPE_LORA_RX  'R'
#define MSG_TYPE_LORA_TX  'T'
#define MSG_TYPE_ACK      'A'
#define MSG_TYPE_NACK     'N'
#define MSG_TYPE_STATUS   'S'

#define MAX_TOPIC_LEN    64
#define MAX_PAYLOAD_LEN  128
#define SERIAL_BUFFER_SIZE 256

// =====================
// ACK Configuration - OPTIMIZADO
// =====================
const unsigned long ACK_TIMEOUT_MS = 2000;  // Timeout para esperar ACK (aumentado para SF10/BW62.5k)
const uint8_t MAX_ACK_RETRIES = 3;          // Máximo reintentos
const unsigned long TX_COOLDOWN_MS = 50;    // Cooldown entre TX

// Buffer para recepción serial
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];
uint16_t serialBufferIdx = 0;

// Estructura para mensajes parseados
typedef struct {
  uint8_t type;
  char topic[MAX_TOPIC_LEN + 1];
  uint8_t topicLen;
  uint8_t payload[MAX_PAYLOAD_LEN];
  uint8_t payloadLen;
} SerialMessage_t;

// =====================
// Variables de estado
// =====================
uint16_t msgCounter = 0;
unsigned long lastTxTime = 0;

// =====================
// Mapeo de topics LoRa -> MQTT
// =====================
const char* mapTopic(const char* inTopic) {
  if (strcmp(inTopic, "sensor/0") == 0) return "sensores/puerta";
  if (strcmp(inTopic, "sensor/1") == 0) return "sensores/luz";
  if (strcmp(inTopic, "sensor/2") == 0) return "sensores/temperatura";
  if (strcmp(inTopic, "sensor/3") == 0) return "sensores/humedad";
  if (strcmp(inTopic, "sensor/4") == 0) return "sensores/movimiento";
  if (strcmp(inTopic, "actuator/0") == 0) return "actuadores/led";
  if (strcmp(inTopic, "actuator/1") == 0) return "actuadores/servo";
  return inTopic;
}

// =====================
// Funciones de protocolo serial
// =====================

void sendSerialMessage(uint8_t msgType, const char* topic, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t topicLen = strlen(topic);
  SERIAL_PI.write(STX);
  SERIAL_PI.write(msgType);
  SERIAL_PI.write(topicLen);
  SERIAL_PI.write((const uint8_t*)topic, topicLen);
  SERIAL_PI.write(payloadLen);
  SERIAL_PI.write(payload, payloadLen);
  SERIAL_PI.write(ETX);
  SERIAL_PI.flush();
}

void sendAck() {
  uint8_t empty = 0;
  sendSerialMessage(MSG_TYPE_ACK, "", &empty, 0);
}

void sendNack(const char* reason) {
  sendSerialMessage(MSG_TYPE_NACK, "", (const uint8_t*)reason, strlen(reason));
}

void sendStatus(const char* status) {
  sendSerialMessage(MSG_TYPE_STATUS, "status", (const uint8_t*)status, strlen(status));
}

bool parseSerialMessage(SerialMessage_t* msg) {
  if (serialBufferIdx < 5) return false;
  
  uint16_t startIdx = 0;
  while (startIdx < serialBufferIdx && serialBuffer[startIdx] != STX) {
    startIdx++;
  }
  
  if (startIdx > 0) {
    memmove(serialBuffer, serialBuffer + startIdx, serialBufferIdx - startIdx);
    serialBufferIdx -= startIdx;
  }
  
  if (serialBufferIdx < 5) return false;
  
  msg->type = serialBuffer[1];
  msg->topicLen = serialBuffer[2];
  
  if (msg->topicLen > MAX_TOPIC_LEN) {
    serialBufferIdx = 0;
    return false;
  }
  
  uint16_t minLen = 4 + msg->topicLen + 1;
  if (serialBufferIdx < minLen) return false;
  
  msg->payloadLen = serialBuffer[3 + msg->topicLen];
  
  if (msg->payloadLen > MAX_PAYLOAD_LEN) {
    serialBufferIdx = 0;
    return false;
  }
  
  uint16_t totalLen = 5 + msg->topicLen + msg->payloadLen;
  
  if (serialBufferIdx < totalLen) return false;
  
  if (serialBuffer[totalLen - 1] != ETX) {
    memmove(serialBuffer, serialBuffer + 1, serialBufferIdx - 1);
    serialBufferIdx--;
    return false;
  }
  
  memcpy(msg->topic, serialBuffer + 3, msg->topicLen);
  msg->topic[msg->topicLen] = '\0';
  memcpy(msg->payload, serialBuffer + 4 + msg->topicLen, msg->payloadLen);
  
  memmove(serialBuffer, serialBuffer + totalLen, serialBufferIdx - totalLen);
  serialBufferIdx -= totalLen;
  
  return true;
}

// =====================
// Funciones LoRa SIMPLIFICADAS
// =====================

/**
 * Envía mensaje LoRa en modo RAW y espera ACK (BLOQUEANTE con polling)
 * Retorna: true si ACK recibido, false si timeout
 */
bool sendLoRaWithAck(uint8_t destination, const uint8_t* payload, uint8_t payloadLen) {
  uint16_t msgId = msgCounter++;
  
  for (uint8_t attempt = 0; attempt <= MAX_ACK_RETRIES; attempt++) {
    if (attempt > 0) {
      SERIAL_DEBUG.print("[RETRY] Intento ");
      SERIAL_DEBUG.print(attempt);
      SERIAL_DEBUG.print("/");
      SERIAL_DEBUG.println(MAX_ACK_RETRIES);
      delay(20);  // Pequeña pausa entre reintentos
    }
    
    // Enviar paquete (bloqueante)
    LoRa.beginPacket();
    LoRa.write(destination);
    LoRa.write(localAddress);
    LoRa.write((uint8_t)(msgId >> 8));
    LoRa.write((uint8_t)(msgId & 0xFF));
    LoRa.write(payloadLen);
    LoRa.write(payload, payloadLen);
    LoRa.endPacket();  // BLOQUEANTE - espera a que termine
    
    SERIAL_DEBUG.print("[TX] msgId=");
    SERIAL_DEBUG.print(msgId);
    SERIAL_DEBUG.print(" -> 0x");
    SERIAL_DEBUG.print(destination, HEX);
    SERIAL_DEBUG.print(" len=");
    SERIAL_DEBUG.println(payloadLen);
    
    // Entrar en modo recepción para ACK
    LoRa.receive();
    
    // Esperar ACK con polling (BLOQUEANTE)
    unsigned long startTime = millis();
    while (millis() - startTime < ACK_TIMEOUT_MS) {
      int packetSize = LoRa.parsePacket();
      if (packetSize > 0) {
        // Leer cabecera
        uint8_t recipient = LoRa.read();
        uint8_t sender = LoRa.read();
        uint16_t rxMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
        uint8_t rxLen = LoRa.read();
        
        // Verificar si es para nosotros
        if ((recipient & localAddress) != localAddress) {
          // Descartar
          while (LoRa.available()) LoRa.read();
          continue;
        }
        
        // Leer payload
        uint8_t rxBuf[16];
        uint8_t idx = 0;
        while (LoRa.available() && idx < sizeof(rxBuf)) {
          rxBuf[idx++] = LoRa.read();
        }
        
        // Verificar si es ACK
        if (rxLen >= 2 && rxBuf[0] == ACK_MARKER) {
          SERIAL_DEBUG.print("[ACK] Recibido de 0x");
          SERIAL_DEBUG.print(sender, HEX);
          SERIAL_DEBUG.print(" rxMsgId=");
          SERIAL_DEBUG.print(rxMsgId);
          SERIAL_DEBUG.print(" esperado=");
          SERIAL_DEBUG.println(msgId);
          
          if (sender == destination && rxMsgId == msgId) {
            SERIAL_DEBUG.println("[ACK] OK!");
            lastTxTime = millis();
            return true;
          }
        }
      }
      delay(5);  // Pequeña pausa para no saturar CPU
    }
    
    SERIAL_DEBUG.println("[TIMEOUT] ACK no recibido");
  }
  
  SERIAL_DEBUG.println("[FAIL] Agotados reintentos");
  lastTxTime = millis();
  return false;
}

/**
 * Verifica y procesa paquetes LoRa entrantes (modo polling)
 * Retorna true si se procesó un mensaje (no ACK)
 */
bool checkLoRaRx(char* topicOut, uint8_t* payloadOut, uint8_t* payloadLenOut) {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return false;
  
  // Leer cabecera
  uint8_t recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t msgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLen = LoRa.read();
  
  // Verificar destinatario
  if ((recipient & localAddress) != localAddress) {
    while (LoRa.available()) LoRa.read();
    return false;
  }
  
  // Leer datos
  uint8_t buffer[MAX_PAYLOAD_LEN + MAX_TOPIC_LEN + 1];
  uint8_t idx = 0;
  while (LoRa.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = LoRa.read();
  }
  
  // Ignorar ACKs (se procesan en sendLoRaWithAck)
  if (incomingLen >= 2 && buffer[0] == ACK_MARKER) {
    return false;
  }
  
  // Parsear topic y payload
  // Formato: topicLen(1) + topic(topicLen) + payload
  uint8_t topicLen = buffer[0];
  
  if (topicLen > MAX_TOPIC_LEN || topicLen >= idx) {
    return false;
  }
  
  // Copiar topic
  memcpy(topicOut, buffer + 1, topicLen);
  topicOut[topicLen] = '\0';
  
  // Copiar payload con metadatos
  uint8_t payloadStart = 1 + topicLen;
  uint8_t dataLen = (payloadStart < idx) ? (idx - payloadStart) : 0;
  
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  payloadOut[0] = sender;
  payloadOut[1] = (uint8_t)(-rssi);
  payloadOut[2] = (uint8_t)(snr + 128);
  if (dataLen > 0) {
    memcpy(payloadOut + 3, buffer + payloadStart, dataLen);
  }
  *payloadLenOut = dataLen + 3;
  
  SERIAL_DEBUG.print("[RX] topic=");
  SERIAL_DEBUG.print(topicOut);
  SERIAL_DEBUG.print(" from=0x");
  SERIAL_DEBUG.print(sender, HEX);
  SERIAL_DEBUG.print(" len=");
  SERIAL_DEBUG.println(*payloadLenOut);
  
  return true;
}

// =====================
// Setup
// =====================
void setup() {
  SERIAL_DEBUG.begin(115200);
  SERIAL_PI.begin(115200);
  
  delay(1000);
  
  SERIAL_DEBUG.println("Gateway LoRa-Serial (SIMPLIFICADO)");
  
  if (!init_PMIC()) {
    SERIAL_DEBUG.println("PMIC init failed");
  }
  
  if (!LoRa.begin(868E6)) {
    SERIAL_DEBUG.println("LoRa init failed!");
    sendStatus("LoRa init failed");
    while (true);
  }
  
  SERIAL_DEBUG.println("LoRa init OK");
  
  // Configurar parámetros LoRa
  LoRa.setSignalBandwidth(long(bandwidth_kHz[nodeConfig.bandwidth_index]));
  LoRa.setSpreadingFactor(nodeConfig.spreadingFactor);
  LoRa.setCodingRate4(nodeConfig.codingRate);
  LoRa.setTxPower(nodeConfig.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);
  
  // NO usar callbacks - modo polling simplificado
  LoRa.receive();
  
  SERIAL_DEBUG.println("Gateway listo (polling mode)!");
  sendStatus("Gateway ready");
}

// =====================
// Loop principal SIMPLIFICADO
// =====================
void loop() {
  // 1. Procesar mensajes LoRa entrantes (polling)
  char topic[MAX_TOPIC_LEN + 1];
  uint8_t payload[MAX_PAYLOAD_LEN + 3];
  uint8_t payloadLen;
  
  if (checkLoRaRx(topic, payload, &payloadLen)) {
    const char* mappedTopic = mapTopic(topic);
    sendSerialMessage(MSG_TYPE_LORA_RX, mappedTopic, payload, payloadLen);
  }
  
  // 2. Procesar datos del puerto serie (desde Raspberry Pi)
  while (SERIAL_PI.available()) {
    if (serialBufferIdx < SERIAL_BUFFER_SIZE) {
      serialBuffer[serialBufferIdx++] = SERIAL_PI.read();
    } else {
      memmove(serialBuffer, serialBuffer + 1, SERIAL_BUFFER_SIZE - 1);
      serialBuffer[SERIAL_BUFFER_SIZE - 1] = SERIAL_PI.read();
    }
  }
  
  // 3. Intentar parsear mensaje serial y transmitir
  SerialMessage_t msg;
  if (parseSerialMessage(&msg)) {
    if (msg.type == MSG_TYPE_LORA_TX) {
      // Extraer destino del topic si tiene formato "@XX/..."
      uint8_t destination = defaultDestination;
      
      if (msg.topicLen > 3 && msg.topic[0] == '@') {
        char hexAddr[3] = {msg.topic[1], msg.topic[2], '\0'};
        destination = (uint8_t)strtol(hexAddr, NULL, 16);
      }
      
      SERIAL_DEBUG.print("[CMD] TX -> 0x");
      SERIAL_DEBUG.print(destination, HEX);
      SERIAL_DEBUG.print(" payload: ");
      for (uint8_t i = 0; i < msg.payloadLen; i++) {
        SERIAL_DEBUG.print(msg.payload[i], HEX);
        SERIAL_DEBUG.print(" ");
      }
      SERIAL_DEBUG.println();
      
      // Enviar por LoRa con ACK (BLOQUEANTE)
      bool success = sendLoRaWithAck(destination, msg.payload, msg.payloadLen);
      
      if (success) {
        sendAck();
        SERIAL_DEBUG.println("[OK] Mensaje entregado");
      } else {
        sendNack("ACK timeout");
        SERIAL_DEBUG.println("[FAIL] No se pudo entregar");
      }
      
      // Volver a modo recepción
      LoRa.receive();
    }
  }
  
  // Pequeña pausa para no saturar CPU
  delay(1);
}
