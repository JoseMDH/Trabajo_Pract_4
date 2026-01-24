#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

Servo miServo;

int pinServo = 7;
int pinLed   = 6;
int posicion = 10;
uint8_t localAddress = 0x06;
const uint8_t senderAddress = 0x05;
const uint8_t ACK_MARKER = 0xAC;

// =====================
// Configuración LoRa (igual que Gateway)
// =====================
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3};

LoRaConfig_t nodeConfig = {6, 10, 5, 2};  // BW=62.5kHz, SF=10, CR=4/5, TxPwr=2dBm

// Control de duplicados (último mensaje procesado)
uint16_t lastProcessedMsgId = 0xFFFF;

void sendAck(uint8_t dest, uint16_t msgId, uint8_t status) {
  // Enviar ACK inmediatamente (bloqueante)
  LoRa.beginPacket();
  LoRa.write(dest);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write((uint8_t)2);
  LoRa.write(ACK_MARKER);
  LoRa.write(status);
  LoRa.endPacket();  // Bloqueante para garantizar envío
  
  Serial.print("[ACK] Enviado msgId=");
  Serial.print(msgId);
  Serial.print(" status=");
  Serial.println(status);
}

void aplicarPuerta(uint8_t v) {
  if (v == 0) {
    posicion = 10;
    miServo.write(posicion);
    Serial.println("[PUERTA] CERRAR");
  } else if (v == 1 || v == 2 || v == 3) {
    posicion = 160;
    miServo.write(posicion);
    Serial.println("[PUERTA] ABRIR");
  } else {
    Serial.print("[PUERTA] Código desconocido: ");
    Serial.println(v);
  }
}

void aplicarLuz(uint8_t v) {
  if (v == 1) {
    digitalWrite(pinLed, HIGH);
    Serial.println("[LUZ] ENCENDER");
  } else if (v == 0) {
    digitalWrite(pinLed, LOW);
    Serial.println("[LUZ] APAGAR");
  } else {
    Serial.print("[LUZ] Código desconocido: ");
    Serial.println(v);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  delay(1000);
  
  Serial.println("\n=== ACTUADOR (POLLING MODE) ===");

  miServo.attach(pinServo);
  miServo.write(posicion);

  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  if (!LoRa.begin(868E6)) {
    Serial.println("ERROR: LoRa init failed!");
    while (1) {
      digitalWrite(pinLed, HIGH);
      delay(200);
      digitalWrite(pinLed, LOW);
      delay(200);
    }
  }
  
  // Configuración LoRa (igual que Gateway)
  LoRa.setSignalBandwidth(long(bandwidth_kHz[nodeConfig.bandwidth_index]));
  LoRa.setSpreadingFactor(nodeConfig.spreadingFactor);
  LoRa.setCodingRate4(nodeConfig.codingRate);
  LoRa.setTxPower(nodeConfig.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);
  
  Serial.print("Direccion: 0x");
  Serial.println(localAddress, HEX);
  Serial.println("Listo para recibir comandos");
}

void loop() {
  // Polling para recibir paquetes LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    delay(5);  // Pequeña pausa para no saturar CPU
    return;
  }

  // Leer cabecera
  uint8_t recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t msgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t msgLen = LoRa.read();

  Serial.println("\n--- PAQUETE ---");
  Serial.print("De: 0x");
  Serial.print(sender, HEX);
  Serial.print(" Para: 0x");
  Serial.print(recipient, HEX);
  Serial.print(" MsgId: ");
  Serial.print(msgId);
  Serial.print(" Len: ");
  Serial.println(msgLen);

  // Leer payload
  uint8_t rawPayload[64];
  uint8_t rawLen = 0;
  while (LoRa.available() && rawLen < 64) {
    rawPayload[rawLen++] = LoRa.read();
  }

  // Verificar destinatario
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("-> No es para mi");
    return;
  }

  // Verificar emisor autorizado
  if (sender != senderAddress) {
    Serial.print("-> Emisor no autorizado: 0x");
    Serial.println(sender, HEX);
    return;
  }

  // Verificar payload mínimo
  if (rawLen < 2) {
    Serial.println("-> Payload incompleto");
    sendAck(sender, msgId, 1);
    return;
  }

  uint8_t tipo = rawPayload[0];   // 0 = luz, 1 = puerta
  uint8_t valor = rawPayload[1];
  uint8_t ackStatus = 0;

  // Verificar si es mensaje duplicado
  bool isDuplicate = (msgId == lastProcessedMsgId);
  
  if (!isDuplicate) {
    // Procesar comando solo si no es duplicado
    Serial.print("-> Tipo=");
    Serial.print(tipo == 0 ? "LUZ" : tipo == 1 ? "PUERTA" : "??");
    Serial.print(" Valor=");
    Serial.println(valor);

    if (tipo == 0) {
      aplicarLuz(valor);
    } else if (tipo == 1) {
      aplicarPuerta(valor);
    } else {
      Serial.println("-> Tipo desconocido");
      ackStatus = 1;
    }
    
    lastProcessedMsgId = msgId;
  } else {
    Serial.println("-> Duplicado, solo ACK");
  }
  
  // SIEMPRE enviar ACK (incluso para duplicados)
  sendAck(sender, msgId, ackStatus);
}
