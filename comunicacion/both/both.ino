#include "lora_communication.h"


constexpr const uint32_t serial1_bauds = 9600;

void setup() {
  if(!lora_init()) {
    while (1);
  }
  Serial1.begin(serial1_bauds);
  Wire.begin();
  pinMode(A1, INPUT);
}

void loop(){
  // Leer respuestas del esclavo
  static unsigned long lastUS = 0;
  static unsigned long lastPR = 0;

  unsigned long now = millis();

  // ---------- Ultrasonidos ----------
  if (now - lastUS >= DELAY) {
    lastUS = now;

    uint16_t us1 = readSRF02(SRF01_I2C_ADDRESS);
    delay(MINWAIT);
    uint16_t us2 = readSRF02(SRF02_I2C_ADDRESS);

    if (us1 != 0xFFFF && us2 != 0xFFFF) {
      uint8_t usPayload = ;

      publish(DISTANCE, checkUS(us1, us2));
    }
  }

  now = millis();

  // ---------- Sensor de luz ----------
  if (now - lastPR >= DELAY) {
    lastPR = now;

    uint8_t prPayload = ;

    pubish(LIGHT, checkPR());
  }

  delay(1000);
  uint8_t data[2];
  data[0] = 0;
  data[1] = 1;
  lora_send(data,2);
  delay(1000);
  data[1] = 0;
  lora_send(data,2);
}

void publish(uint8_t topicId,uint8_t payload){
  uint8_t data[2] = {
    topicId,     // topic (ID)
    payload       // payload
  };

  lora_send(data, 2);
}
