#include "sensors.ino"

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
constexpr uint8_t RX_PIN = 8;
constexpr uint8_t TX_PIN = 9;
SoftwareSerial Serial1(RX_PIN, TX_PIN);  // RX, TX
#endif

void setup() {
  Wire.begin();
  Serial1.begin(9600);
  pinMode(A1, INPUT);
}

void loop()
{
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
      uint8_t usPayload = checkUS(us1, us2);

      sendSensorPublish(DISTANCE, usPayload);
    }
  }

  now = millis();

  // ---------- Sensor de luz ----------
  if (now - lastPR >= DELAY) {
    lastPR = now;

    uint8_t prPayload = checkPR();

    sendSensorPublish(LIGHT, prPayload);
  }
}

