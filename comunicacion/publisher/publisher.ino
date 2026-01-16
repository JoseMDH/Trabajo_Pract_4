#include "lora_communication.h"

constexpr const uint32_t serial1_bauds = 9600;

// Tabla de topics (el índice es el topicId)
const char* topics[] = {
  "sensor/0",    // topicId = 0
  "sensor/1",    // topicId = 1
  "sensor/2",    // topicId = 2
  "sensor/3",    // topicId = 3
  "sensor/4",    // topicId = 4
  "actuator/0",  // topicId = 5
  "actuator/1",  // topicId = 6
  "status"       // topicId = 7
};
const uint8_t NUM_TOPICS = sizeof(topics) / sizeof(topics[0]);

/**
 * Envía un mensaje LoRa con formato compatible con el Gateway
 * Formato: topicLen(1) + topic(topicLen) + payload
 */
bool lora_send_with_topic(const char* topic, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t topicLen = strlen(topic);
  uint8_t totalLen = 1 + topicLen + payloadLen;  // topicLen + topic + payload
  
  uint8_t buffer[64];  // Buffer temporal
  if (totalLen > sizeof(buffer)) return false;
  
  buffer[0] = topicLen;
  memcpy(buffer + 1, topic, topicLen);
  memcpy(buffer + 1 + topicLen, payload, payloadLen);
  
  return lora_send(buffer, totalLen);
}

/**
 * Envía un mensaje usando topicId numérico
 */
bool lora_send_by_topic_id(uint8_t topicId, const uint8_t* payload, uint8_t payloadLen) {
  if (topicId >= NUM_TOPICS) {
    // TopicId inválido, usar topic genérico
    char genericTopic[16];
    snprintf(genericTopic, sizeof(genericTopic), "topic/%d", topicId);
    return lora_send_with_topic(genericTopic, payload, payloadLen);
  }
  return lora_send_with_topic(topics[topicId], payload, payloadLen);
}

void setup() {
  Serial.begin(115200);  // Debug
  
  if(!lora_init()) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa init OK");
  
  Serial1.begin(serial1_bauds);
}
uint8_t testValue = '0';
void loop(){
  // Leer respuestas del esclavo
  while (Serial1.available() > 0) {
    publish();
  }

  //test()
}

void test(){
  // Test: enviar mensajes de prueba cada 2 segundos
  static uint32_t lastSend = 0;

  
  if (millis() - lastSend > 2000) {
    uint8_t payload[1] = { testValue };
    lora_send_with_topic("sensor/1", payload, 1);
    
    Serial.print("TX: sensor/1 = ");
    Serial.println(testValue);
    if (testValue == '0'){
      testValue = '1';
    }else {
      testValue = '0';
    }
    lastSend = millis();
  }
}

void publish()
{
  if (Serial1.available() < 2) return;

  uint8_t topicId = Serial1.read();
  char payload = Serial1.read();

  uint8_t payloadData[1] = { payload };
  
  lora_send_by_topic_id(topicId, payloadData, 1);
  if (topicId==0){
    Serial.print("TX: topicId=");
  Serial.print(topicId);
  Serial.print(" payload=");
  Serial.println(payload); }
}
