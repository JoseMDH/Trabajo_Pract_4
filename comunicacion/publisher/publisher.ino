#include "lora_communication.h"


constexpr const uint32_t serial1_bauds = 9600;

void setup() {
  if(!lora_init()) {
    while (1);
  }
  Serial1.begin(serial1_bauds);
}

void loop(){
  // Leer respuestas del esclavo
  while (Serial1.available() > 0) {
    publish();
  }

  delay(1000);
  uint8_t data[2];
  data[0] = 0;
  data[1] = 1;
  lora_send(data,2);
  delay(1000);
  data[0] = 0;
  data[1] = 0;
  lora_send(data,2);
}

void publish()
{
  if (Serial1.available() < 2) return;

  uint8_t topicId = Serial1.read();
  uint8_t payload  = Serial1.read();

  uint8_t data[2] = {
    topicId,     // topic (ID)
    payload       // payload
  };

  lora_send(data, 2);
}
