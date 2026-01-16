#include "publish_format.h"

#ifndef SENSORS
#define SENSORS

#define SRF01_I2C_ADDRESS byte((0xE0)>>1)
#define SRF02_I2C_ADDRESS byte((0xF2)>>1)

#define DELAY 200 // ms
#define MINWAIT 70 // ms
#define PR A1

uint8_t checkUS(uint16_t us1, uint16_t us2){
  uint8_t result = 0;

  if (us1 > USLimit) result |= 0x01;
  if (us2 > USLimit) result |= 0x02;

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

#endif


