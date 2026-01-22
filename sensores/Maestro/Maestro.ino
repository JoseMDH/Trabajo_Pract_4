#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define LORA_LOCAL_ADDRESS      0x04
#define LORA_GATEWAY_ADDRESS    0x05
#define LORA_FREQUENCY          868E6
#define LORA_BW                 62.5E3
#define LORA_SF                 10
#define LORA_CR                 5
#define LORA_TP                 2
#define LORA_SYNC_WORD          0x12
#define LORA_PREAMBLE_LENGTH    8

#define LIGHT_THRESHOLD   500
#define DISTANCE_THRESHOLD 100

#define TOPIC_LIGHT    "sensor/1"
#define TOPIC_DISTANCE "sensor/0"

uint16_t loraMessageCounter = 0;
unsigned long lastLoraSendTime = 0;
unsigned long txStartTime = 0;
const unsigned long LORA_SEND_INTERVAL = 500;

volatile bool sendDistanceNext = false;

struct SensorMessage {
  uint8_t state;
  uint8_t remaining;
};

SensorMessage lightMsg = {0, 0};
SensorMessage distanceMsg = {0, 0};

uint8_t lastLightState = 0;
uint8_t lastDistanceState = 0;
uint16_t lastLightValue = 0;
uint16_t lastDistance1 = 0xFFFF;
uint16_t lastDistance2 = 0xFFFF;

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
constexpr uint8_t RX_PIN = 8;
constexpr uint8_t TX_PIN = 9;
SoftwareSerial Serial1(RX_PIN, TX_PIN);
#endif

bool lora_init() {
  if (!LoRa.begin(LORA_FREQUENCY)) return false;
  LoRa.setSignalBandwidth(long(LORA_BW));
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TP, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  return true;
}

void sendLoraPacket(const char* topic, uint8_t payload) {
  uint8_t topicLen = strlen(topic);
  uint8_t totalLen = 1 + topicLen + 1;

  LoRa.beginPacket();
  LoRa.write(LORA_GATEWAY_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write((uint8_t)(loraMessageCounter >> 8));
  LoRa.write((uint8_t)(loraMessageCounter & 0xFF));
  LoRa.write(totalLen);
  LoRa.write(topicLen);
  LoRa.write((const uint8_t*)topic, topicLen);
  LoRa.write(payload);

  if (LoRa.endPacket(false)) {
    loraMessageCounter++;
    txStartTime = millis();
    sendDistanceNext = !sendDistanceNext;
    Serial.println("[LoRa] Paquete enviado OK");
  } else {
    Serial.println("[LoRa] ERROR al enviar paquete");
  }
}

void processLoraQueue() {
  if (millis() - txStartTime < 200) return;

  if (sendDistanceNext) {
    if (distanceMsg.remaining > 0) {
      Serial.println("[LoRa] Enviando DISTANCIA...");
      uint8_t payload = distanceMsg.state ? '1' : '0';
      sendLoraPacket(TOPIC_DISTANCE, payload);
      Serial.print("[LoRa TX] ");
      Serial.print(TOPIC_DISTANCE);
      Serial.print(" = ");
      Serial.println((char)payload);
      distanceMsg.remaining--;
    } else {
      sendDistanceNext = false;
    }
  } else {
    if (lightMsg.remaining > 0) {
      Serial.println("[LoRa] Enviando LUZ...");
      uint8_t payload = lightMsg.state ? '1' : '0';
      sendLoraPacket(TOPIC_LIGHT, payload);
      Serial.print("[LoRa TX] ");
      Serial.print(TOPIC_LIGHT);
      Serial.print(" = ");
      Serial.println((char)payload);
      lightMsg.remaining--;
    } else {
      sendDistanceNext = true;
    }
  }
}

void parseSlaveMessage(uint8_t firstByte) {
  if ((firstByte & 0xF0) != 0x10) return;

  while (Serial1.available() < 4);

  uint8_t sensor_id = Serial1.read();
  uint8_t unit = Serial1.read();
  uint8_t hi = Serial1.read();
  uint8_t lo = Serial1.read();

  uint16_t measure = (hi << 8) | lo;

  Serial.print("[SLAVE] Medida sensor ");
  Serial.print(sensor_id);
  Serial.print(": ");
  Serial.println(measure);

  if (sensor_id == 2 || sensor_id == 3) {
    lastLightValue = measure;
    lastLightState = (measure < LIGHT_THRESHOLD) ? 1 : 0;
    Serial.print("  -> Estado luz: ");
    Serial.println(lastLightState ? "OSCURO (1)" : "ILUMINADO (0)");
  } else if (sensor_id == 0 || sensor_id == 1) {
    if (sensor_id == 0) lastDistance1 = measure;
    else lastDistance2 = measure;

    if (lastDistance1 < DISTANCE_THRESHOLD || lastDistance2 < DISTANCE_THRESHOLD)
      lastDistanceState = 1;
    else
      lastDistanceState = 0;

    Serial.print("  -> Estado distancia: ");
    Serial.println(lastDistanceState ? "OBJETO CERCA (1)" : "LIBRE (0)");
  }
}

void sendLoRaUpdates() {
  if (lastLightState != lightMsg.state) {
    uint8_t old = lightMsg.state;
    lightMsg.state = lastLightState;
    lightMsg.remaining = 3;
    Serial.print("[LoRa] Cambio LUZ: ");
    Serial.print(old ? '1' : '0');
    Serial.print(" -> ");
    Serial.println(lightMsg.state ? '1' : '0');
    Serial.print(" (raw: ");
    Serial.print(lastLightValue);
    Serial.println(")");

  }

  if (lastDistanceState != distanceMsg.state) {
    uint8_t old = distanceMsg.state;
    distanceMsg.state = lastDistanceState;
    distanceMsg.remaining = 3;
    Serial.print("[LoRa] Cambio DIST: ");
    Serial.print(old ? '1' : '0');
    Serial.print(" -> ");
    Serial.println(distanceMsg.state ? '1' : '0');
    Serial.print(" (d1: ");
    Serial.print(lastDistance1);
    Serial.print("cm, d2: ");
    Serial.print(lastDistance2);
    Serial.println("cm)");

  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial1.begin(9600);

  if (!lora_init()) {
    Serial.println("ERROR: LoRa init failed");
    while (1);
  }

  Serial.println("LoRa init OK");
  lastLoraSendTime = millis();
}

void loop() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();
    parseSlaveMessage(b);
  }

  unsigned long now = millis();
  if (now - lastLoraSendTime >= LORA_SEND_INTERVAL) {
    lastLoraSendTime = now;
    sendLoRaUpdates();
  }

  processLoraQueue();
}
