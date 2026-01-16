/* ---------------------------------------------------------------------
 *  Gateway LoRa-Serial para Arduino MKR WAN 1310
 *  =============================================
 *  
 *  Este sketch convierte el Arduino en un gateway bidireccional:
 *  - Recibe mensajes LoRa y los envía por Serial a la Raspberry Pi
 *  - Recibe comandos por Serial y los transmite por LoRa
 *  
 *  Protocolo Serial:
 *  STX(0x02) | tipo(1) | topic_len(1) | topic | payload_len(1) | payload | ETX(0x03)
 *  
 *  Tipos de mensaje:
 *  - 'R' (0x52): Mensaje recibido de LoRa (Arduino -> Raspberry)
 *  - 'T' (0x54): Mensaje a transmitir por LoRa (Raspberry -> Arduino)
 *  - 'A' (0x41): ACK
 *  - 'N' (0x4E): NACK
 *  - 'S' (0x53): Estado del sistema
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

// =====================
// Configuración Serial
// =====================
// Usamos Serial1 para comunicación con Raspberry Pi (GPIO)
// Serial1 en MKR WAN 1310: Pin 13 (RX), Pin 14 (TX)
// Conectar: Arduino Pin 13 (RX) -> Raspberry GPIO14 (TX)
//           Arduino Pin 14 (TX) -> Raspberry GPIO15 (RX)
//           GND común
#define SERIAL_PI Serial1
#define SERIAL_DEBUG Serial  // USB para debug (opcional)

// =====================
// Configuración LoRa
// =====================
const uint8_t localAddress = 0x05;     // Dirección de este dispositivo (gateway)
const uint8_t defaultDestination = 0x06; // Dirección de destino por defecto

// Estructura para almacenar la configuración de la radio
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

#define MSG_TYPE_LORA_RX  'R'  // Mensaje recibido de LoRa
#define MSG_TYPE_LORA_TX  'T'  // Mensaje a transmitir por LoRa
#define MSG_TYPE_ACK      'A'  // Acknowledgment
#define MSG_TYPE_NACK     'N'  // Negative acknowledgment
#define MSG_TYPE_STATUS   'S'  // Estado del sistema

#define MAX_TOPIC_LEN    64
#define MAX_PAYLOAD_LEN  128
#define SERIAL_BUFFER_SIZE 256

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
volatile bool txDoneFlag = true;
volatile bool transmitting = false;
uint16_t msgCounter = 0;

// =====================
// Buffer para recepción LoRa (evitar I/O en ISR)
// =====================
typedef struct {
  bool pending;              // Hay mensaje pendiente de procesar
  char topic[MAX_TOPIC_LEN + 1];
  uint8_t payload[MAX_PAYLOAD_LEN + 3];  // sender + rssi + snr + datos
  uint8_t payloadLen;
} LoRaRxMessage_t;

volatile LoRaRxMessage_t loraRxMsg = {false, {0}, {0}, 0};

// =====================
// Mapeo de topics LoRa -> MQTT
// =====================
/**
 * Convierte topics internos a topics MQTT legibles
 */
const char* mapTopic(const char* inTopic) {
  if (strcmp(inTopic, "sensor/0") == 0) return "sensores/puerta";
  if (strcmp(inTopic, "sensor/1") == 0) return "sensores/luz";
  if (strcmp(inTopic, "sensor/2") == 0) return "sensores/temperatura";
  if (strcmp(inTopic, "sensor/3") == 0) return "sensores/humedad";
  if (strcmp(inTopic, "sensor/4") == 0) return "sensores/movimiento";
  if (strcmp(inTopic, "actuator/0") == 0) return "actuadores/led";
  if (strcmp(inTopic, "actuator/1") == 0) return "actuadores/servo";
  // Si no hay mapeo, devolver el original
  return inTopic;
}

// =====================
// Funciones de protocolo serial
// =====================

/**
 * Envía un mensaje por Serial1 en formato de trama (a la Raspberry Pi)
 */
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

/**
 * Envía un ACK por Serial
 */
void sendAck() {
  uint8_t empty = 0;
  sendSerialMessage(MSG_TYPE_ACK, "", &empty, 0);
}

/**
 * Envía un NACK por Serial
 */
void sendNack(const char* reason) {
  sendSerialMessage(MSG_TYPE_NACK, "", (const uint8_t*)reason, strlen(reason));
}

/**
 * Envía un mensaje de estado por Serial
 */
void sendStatus(const char* status) {
  sendSerialMessage(MSG_TYPE_STATUS, "status", (const uint8_t*)status, strlen(status));
}

/**
 * Parsea un mensaje del buffer serial
 * Retorna true si se parseó correctamente un mensaje completo
 */
bool parseSerialMessage(SerialMessage_t* msg) {
  if (serialBufferIdx < 5) return false;  // Mínimo: STX + tipo + topicLen + payloadLen + ETX
  
  // Buscar STX
  uint16_t startIdx = 0;
  while (startIdx < serialBufferIdx && serialBuffer[startIdx] != STX) {
    startIdx++;
  }
  
  if (startIdx > 0) {
    // Descartar bytes antes de STX
    memmove(serialBuffer, serialBuffer + startIdx, serialBufferIdx - startIdx);
    serialBufferIdx -= startIdx;
  }
  
  if (serialBufferIdx < 5) return false;
  
  // Verificar estructura
  msg->type = serialBuffer[1];
  msg->topicLen = serialBuffer[2];
  
  if (msg->topicLen > MAX_TOPIC_LEN) {
    // Topic demasiado largo, descartar
    serialBufferIdx = 0;
    return false;
  }
  
  uint16_t minLen = 4 + msg->topicLen + 1;  // STX + tipo + topicLen + topic + payloadLen + ETX
  if (serialBufferIdx < minLen) return false;
  
  msg->payloadLen = serialBuffer[3 + msg->topicLen];
  
  if (msg->payloadLen > MAX_PAYLOAD_LEN) {
    // Payload demasiado largo, descartar
    serialBufferIdx = 0;
    return false;
  }
  
  uint16_t totalLen = 5 + msg->topicLen + msg->payloadLen;  // STX + tipo + topicLen + topic + payloadLen + payload + ETX
  
  if (serialBufferIdx < totalLen) return false;
  
  // Verificar ETX
  if (serialBuffer[totalLen - 1] != ETX) {
    // Mensaje corrupto, buscar siguiente STX
    memmove(serialBuffer, serialBuffer + 1, serialBufferIdx - 1);
    serialBufferIdx--;
    return false;
  }
  
  // Copiar topic
  memcpy(msg->topic, serialBuffer + 3, msg->topicLen);
  msg->topic[msg->topicLen] = '\0';
  
  // Copiar payload
  memcpy(msg->payload, serialBuffer + 4 + msg->topicLen, msg->payloadLen);
  
  // Eliminar mensaje del buffer
  memmove(serialBuffer, serialBuffer + totalLen, serialBufferIdx - totalLen);
  serialBufferIdx -= totalLen;
  
  return true;
}

// =====================
// Funciones LoRa
// =====================

/**
 * Envía un mensaje por LoRa
 * El payload incluye: topic (null-terminated) + datos
 */
void sendLoRaMessage(uint8_t destination, const char* topic, const uint8_t* payload, uint8_t payloadLen) {
  while(!LoRa.beginPacket()) {
    delay(10);
  }
  
  // Cabecera del paquete LoRa
  LoRa.write(destination);                // Destinatario
  LoRa.write(localAddress);               // Remitente
  LoRa.write((uint8_t)(msgCounter >> 8)); // ID mensaje (MSB)
  LoRa.write((uint8_t)(msgCounter & 0xFF)); // ID mensaje (LSB)
  
  // Topic + payload
  uint8_t topicLen = strlen(topic);
  uint8_t totalLen = topicLen + 1 + payloadLen;  // topic + null + payload
  
  LoRa.write(totalLen);                   // Longitud total
  LoRa.write((const uint8_t*)topic, topicLen);  // Topic
  LoRa.write((uint8_t)'\0');              // Null terminator del topic
  LoRa.write(payload, payloadLen);        // Payload
  
  LoRa.endPacket(true);  // No bloqueante
  
  msgCounter++;
}

/**
 * Envía un mensaje por LoRa en modo RAW (solo payload, sin topic)
 * Usado para el actuador que espera: tipo(1) + valor(1)
 */
void sendLoRaMessageRaw(uint8_t destination, const uint8_t* payload, uint8_t payloadLen) {
  while(!LoRa.beginPacket()) {
    delay(10);
  }
  
  // Cabecera del paquete LoRa (mismo formato que el actuador espera)
  LoRa.write(destination);                // Destinatario
  LoRa.write(localAddress);               // Remitente
  LoRa.write((uint8_t)(msgCounter >> 8)); // ID mensaje (MSB)
  LoRa.write((uint8_t)(msgCounter & 0xFF)); // ID mensaje (LSB)
  LoRa.write(payloadLen);                 // Longitud del payload
  LoRa.write(payload, payloadLen);        // Payload directo (tipo + valor)
  
  LoRa.endPacket(true);  // No bloqueante
  
  SERIAL_DEBUG.print("LoRa TX raw -> 0x");
  SERIAL_DEBUG.print(destination, HEX);
  SERIAL_DEBUG.print(" len=");
  SERIAL_DEBUG.println(payloadLen);
  
  msgCounter++;
}

/**
 * Callback de recepción LoRa
 * NOTA: Este callback se ejecuta en contexto de interrupción.
 * NO hacer operaciones de I/O (Serial) aquí. Solo guardar datos en buffer.
 */
void onLoRaReceive(int packetSize) {
  if (transmitting && !txDoneFlag) txDoneFlag = true;
  if (packetSize == 0) return;
  
  // Si hay un mensaje pendiente sin procesar, descartar el nuevo
  if (loraRxMsg.pending) {
    while (LoRa.available()) LoRa.read();
    return;
  }
  
  // Leer cabecera
  uint8_t recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t msgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLen = LoRa.read();
  
  // Verificar destinatario
  if ((recipient & localAddress) != localAddress) {
    // Descartar bytes restantes
    while (LoRa.available()) LoRa.read();
    return;
  }
  
  // Leer datos
  uint8_t buffer[MAX_PAYLOAD_LEN + MAX_TOPIC_LEN + 1];
  uint8_t idx = 0;
  while (LoRa.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = LoRa.read();
  }
  
  if (idx != incomingLen) {
    // Error de longitud
    return;
  }
  
  // Separar topic y payload
  // Formato esperado: topicLen(1) + topic(topicLen) + payload
  uint8_t topicLen = buffer[0];
  
  // Validar topicLen
  if (topicLen > MAX_TOPIC_LEN || topicLen >= idx) {
    return;  // Topic inválido
  }
  
  // Copiar topic
  for (uint8_t i = 0; i < topicLen; i++) {
    ((char*)loraRxMsg.topic)[i] = buffer[1 + i];
  }
  ((char*)loraRxMsg.topic)[topicLen] = '\0';
  
  uint8_t payloadStart = 1 + topicLen;  // Después del topicLen + topic
  uint8_t payloadLen = (payloadStart < idx) ? (idx - payloadStart) : 0;
  
  // Obtener RSSI y SNR
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  // Construir payload: sender(1) + rssi(1) + snr(1) + payload
  ((uint8_t*)loraRxMsg.payload)[0] = sender;
  ((uint8_t*)loraRxMsg.payload)[1] = (uint8_t)(-rssi);  // Convertir a positivo
  ((uint8_t*)loraRxMsg.payload)[2] = (uint8_t)(snr + 128);  // Offset para negativos
  if (payloadLen > 0) {
    memcpy((uint8_t*)loraRxMsg.payload + 3, buffer + payloadStart, payloadLen);
  }
  loraRxMsg.payloadLen = payloadLen + 3;
  
  // Marcar mensaje como pendiente (esto lo procesará el loop principal)
  loraRxMsg.pending = true;
}

/**
 * Callback cuando termina la transmisión LoRa
 */
void onTxDone() {
  txDoneFlag = true;
}

// =====================
// Setup
// =====================
void setup() {
  // Serial para debug por USB (opcional)
  SERIAL_DEBUG.begin(115200);
  
  // Serial1 para comunicación con Raspberry Pi (GPIO)
  SERIAL_PI.begin(115200);
  
  // Esperar un poco para estabilizar
  delay(1000);
  
  SERIAL_DEBUG.println("Gateway LoRa-Serial iniciando...");
  
  // Inicializar PMIC
  if (!init_PMIC()) {
    SERIAL_DEBUG.println("PMIC init failed");
  }
  
  // Inicializar LoRa
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
  
  // Configurar callbacks
  LoRa.onReceive(onLoRaReceive);
  LoRa.onTxDone(onTxDone);
  
  // Activar recepción
  LoRa.receive();
  
  SERIAL_DEBUG.println("Gateway listo!");
  SERIAL_DEBUG.println("Serial1 -> Raspberry Pi GPIO");
  SERIAL_DEBUG.println("Serial  -> USB Debug");
  
  // Enviar estado inicial a la Raspberry
  sendStatus("Gateway ready");
}

// =====================
// Loop principal
// =====================
void loop() {
  // Procesar mensaje LoRa pendiente (recibido en ISR)
  if (loraRxMsg.pending) {
    // Copiar datos del buffer volatile a variables locales
    char topic[MAX_TOPIC_LEN + 1];
    uint8_t payload[MAX_PAYLOAD_LEN + 3];
    uint8_t payloadLen;
    
    noInterrupts();  // Sección crítica
    strcpy(topic, (const char*)loraRxMsg.topic);
    memcpy(payload, (const uint8_t*)loraRxMsg.payload, loraRxMsg.payloadLen);
    payloadLen = loraRxMsg.payloadLen;
    loraRxMsg.pending = false;  // Marcar como procesado
    interrupts();
    
    // Mapear topic a nombre legible
    const char* mappedTopic = mapTopic(topic);
    
    // Ahora es seguro hacer I/O serial
    sendSerialMessage(MSG_TYPE_LORA_RX, mappedTopic, payload, payloadLen);
    
    SERIAL_DEBUG.print("LoRa RX: topic=");
    SERIAL_DEBUG.print(topic);
    SERIAL_DEBUG.print(" -> ");
    SERIAL_DEBUG.print(mappedTopic);
    SERIAL_DEBUG.print(" len=");
    SERIAL_DEBUG.println(payloadLen);
  }
  
  // Procesar datos del puerto serie (desde Raspberry Pi)
  while (SERIAL_PI.available()) {
    if (serialBufferIdx < SERIAL_BUFFER_SIZE) {
      serialBuffer[serialBufferIdx++] = SERIAL_PI.read();
    } else {
      // Buffer lleno, descartar byte más antiguo
      memmove(serialBuffer, serialBuffer + 1, SERIAL_BUFFER_SIZE - 1);
      serialBuffer[SERIAL_BUFFER_SIZE - 1] = SERIAL_PI.read();
    }
  }
  
  // Intentar parsear mensaje
  SerialMessage_t msg;
  if (parseSerialMessage(&msg)) {
    if (msg.type == MSG_TYPE_LORA_TX) {
      // Transmitir por LoRa
      if (!transmitting) {
        transmitting = true;
        txDoneFlag = false;
        
        // Extraer destino del topic si tiene formato "@XX/..."
        uint8_t destination = defaultDestination;
        char* topicToSend = msg.topic;
        bool rawMode = false;  // Modo raw: enviar solo payload sin topic
        
        if (msg.topicLen > 3 && msg.topic[0] == '@') {
          char hexAddr[3] = {msg.topic[1], msg.topic[2], '\0'};
          destination = (uint8_t)strtol(hexAddr, NULL, 16);
          
          // Si el topic empieza con @XX/, es para el actuador -> modo raw
          // El actuador espera: tipo(1) + valor(1), sin topic
          rawMode = true;
          
          // Mover el puntero del topic después de "@XX/"
          if (msg.topicLen > 4 && msg.topic[3] == '/') {
            topicToSend = msg.topic + 4;
          } else {
            topicToSend = msg.topic + 3;
          }
          
          SERIAL_DEBUG.print("Destino: 0x");
          SERIAL_DEBUG.println(destination, HEX);
          SERIAL_DEBUG.print("Modo raw: ");
          SERIAL_DEBUG.println(rawMode ? "SI" : "NO");
        }
        
        if (rawMode) {
          // Modo raw: enviar solo el payload directamente (para actuador)
          SERIAL_DEBUG.print("Enviando raw a 0x");
          SERIAL_DEBUG.print(destination, HEX);
          SERIAL_DEBUG.print(": ");
          for (int i = 0; i < msg.payloadLen; i++) {
            SERIAL_DEBUG.print(msg.payload[i], HEX);
            SERIAL_DEBUG.print(" ");
          }
          SERIAL_DEBUG.println();
          
          sendLoRaMessageRaw(destination, msg.payload, msg.payloadLen);
        } else {
          // Modo normal: enviar topic + payload
          sendLoRaMessage(destination, topicToSend, msg.payload, msg.payloadLen);
        }
        
        sendAck();
      } else {
        sendNack("TX busy");
      }
    }
  }
  
  // Verificar si terminó la transmisión
  if (transmitting && txDoneFlag) {
    transmitting = false;
    LoRa.receive();  // Reactivar recepción
  }
}
