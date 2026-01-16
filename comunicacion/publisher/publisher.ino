#include "lora_communication.h"

constexpr const uint32_t serial1_bauds = 9600;

// Tabla de topics (el índice es el topicId)
const char* topics[] = {
  "sensor/0",    // topicId = 0
  "sensor/1",    // topicId = 1
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
  publish();
}

// Byte de sincronización (debe coincidir con comunicacion.ino)
#define SYNC_BYTE 0xAA

// Parser robusto: estado para mantener sincronía
void publish() {
  static enum { WAIT_SYNC, READ_TOPIC, READ_PAYLOAD } state = WAIT_SYNC;
  static uint8_t topicId = 0;

  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();

    switch (state) {
      case WAIT_SYNC:
        if (b == SYNC_BYTE) {
          state = READ_TOPIC;
        } else {
          // Descartar basura
          Serial.print("Descartando: 0x");
          Serial.println(b, HEX);
        }
        break;

      case READ_TOPIC:
        topicId = b;
        if (topicId >= NUM_TOPICS) {
          Serial.print("Error topic invalido: ");
          Serial.println(topicId);
          state = WAIT_SYNC;
        } else {
          state = READ_PAYLOAD;
        }
        break;

      case READ_PAYLOAD: {
        uint8_t payload = b;
        // Si llega un SYNC aquí, perdimos bytes; reiniciar
        if (payload == SYNC_BYTE) {
          Serial.println("Desalineado: payload es SYNC");
          state = READ_TOPIC; // ya tenemos un SYNC implícito en b
          topicId = 0xFF;     // marcar inválido hasta recibir siguiente
          break;
        }

        // Convertir 0/1 numérico a '0'/'1'
        uint8_t payloadChar = (payload == 0) ? '0' : '1';
        uint8_t payloadData[1] = { payloadChar };

        lora_send_by_topic_id(topicId, payloadData, 1);

        Serial.print("TX: topic=");
        Serial.print(topics[topicId]);
        Serial.print(" raw=");
        Serial.print(payload);
        Serial.print(" char=");
        Serial.println((char)payloadChar);

        // Preparar para siguiente trama
        state = WAIT_SYNC;
        break;
      }
    }
  }
}
