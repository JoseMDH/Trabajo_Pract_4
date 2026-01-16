#include <Wire.h>

#include "publish_format.h"

#define SRF01_I2C_ADDRESS byte((0xE0)>>1)
#define SRF02_I2C_ADDRESS byte((0xF2)>>1)

#define DELAY 200 // ms
#define MINWAIT 70 // ms
#define PR A1

#define SRF02_CMD_REG 0x00
#define SRF02_RANGE_HIGH 0x02
#define SRF02_RANGE_LOW 0x03

uint8_t checkUS(uint16_t us1, uint16_t us2){
  uint8_t result = LOW;
  if (us1 > USLimit || us2 > USLimit) result = HIGH;
  return result;
}

void sendSensorPublish(uint8_t sensorId, uint8_t flag)
{
  uint8_t msg[2];
  msg[0] = sensorId;
  msg[1] = flag;

  Serial1.write(msg, sizeof(msg));
}

uint16_t readSRF02(uint8_t address) {
  uint8_t cmd = 0x51;  // cm

  Wire.beginTransmission(address);
  Wire.write(SRF02_CMD_REG);
  Wire.write(cmd);
  Wire.endTransmission();

  delay(MINWAIT);

  Wire.beginTransmission(address);
  Wire.write(SRF02_RANGE_HIGH);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  
  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  return (high << 8) | low;
}



uint8_t checkPR(){
  uint8_t result = LOW;
  if (analogRead(PR)<=PRLimit) result = HIGH;
  return result;
}


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

