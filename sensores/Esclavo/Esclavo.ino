#include <Wire.h>

#define SRF01_I2C_ADDRESS byte((0xE0)>>1)
#define SRF02_I2C_ADDRESS byte((0xF2)>>1)
#define SRF02_CMD_REG 0x00
#define SRF02_RANGE_HIGH 0x02

#define NUM_SENSORS 3

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
constexpr uint8_t RX_PIN = 8;
constexpr uint8_t TX_PIN = 9;
SoftwareSerial Serial1(RX_PIN, TX_PIN);
#endif

struct SensorConfig {
  uint8_t address;
  uint8_t unit;
  bool periodic;
  uint16_t periodMs;
  unsigned long lastShot;
  uint16_t lastMeasure;
  bool active;
};

SensorConfig sensors[NUM_SENSORS] = {
  {SRF01_I2C_ADDRESS, 1, false, 0, 0, 0, false},
  {SRF02_I2C_ADDRESS, 1, false, 0, 0, 0, false},
  {0xFF, 3, false, 0, 0, 0, true}
};

bool isI2CDeviceAvailable(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

uint16_t readSRF02(uint8_t address, uint8_t unit) {
  uint8_t cmd = 0x51;
  if (unit == 2) cmd = 0x50;
  if (unit == 0) cmd = 0x56;

  Wire.beginTransmission(address);
  Wire.write(SRF02_CMD_REG);
  Wire.write(cmd);
  Wire.endTransmission();

  delay(70);

  Wire.beginTransmission(address);
  Wire.write(SRF02_RANGE_HIGH);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;

  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  return (high << 8) | low;
}

void sendMeasurement(uint8_t sensor_id, uint8_t unit, uint16_t measure) {
  Serial1.write(0x10);
  Serial1.write(sensor_id);
  Serial1.write(unit);
  Serial1.write((measure >> 8) & 0xFF);
  Serial1.write(measure & 0xFF);
}

void setup() {
  Wire.begin();
  pinMode(A1, INPUT);
  Serial1.begin(9600);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i].address == 0xFF) {
      sensors[i].active = true;
    } else {
      sensors[i].active = isI2CDeviceAvailable(sensors[i].address);
    }
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i].active) {
      sensors[i].periodic = true;
      sensors[i].periodMs = 1000;
      sensors[i].lastShot = millis();

      if (sensors[i].address == 0xFF) {
        sensors[i].lastMeasure = analogRead(A1);
      } else {
        sensors[i].lastMeasure = readSRF02(sensors[i].address, sensors[i].unit);
      }

      sendMeasurement(i, sensors[i].unit, sensors[i].lastMeasure);
    }
  }
}

void loop() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    SensorConfig &s = sensors[i];

    if (s.periodic && s.active && (now - s.lastShot >= s.periodMs)) {
      s.lastShot = now;

      if (s.address == 0xFF) {
        s.lastMeasure = analogRead(A1);
      } else {
        s.lastMeasure = readSRF02(s.address, s.unit);
      }

      sendMeasurement(i, s.unit, s.lastMeasure);
    }
  }
}
