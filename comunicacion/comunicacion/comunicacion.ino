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

// Byte de sincronización para el protocolo serial
#define SYNC_BYTE 0xAA

// Retorna SENSOR_HIGH si algún sensor detecta objeto cerca (distancia < límite)
uint8_t checkUS(uint16_t us1, uint16_t us2) {
  if (us1 < USLimit || us2 < USLimit) {
    return SENSOR_HIGH;  // Objeto detectado cerca
  }
  return SENSOR_LOW;  // No hay objeto cerca
}

void sendSensorPublish(uint8_t sensorId, uint8_t flag)
{
  uint8_t msg[3];
  msg[0] = SYNC_BYTE;  // Byte de sincronización
  msg[1] = sensorId;
  msg[2] = flag;

  Serial1.write(msg, sizeof(msg));
  Serial1.flush();  // Asegurar que se envía todo
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



// Retorna SENSOR_HIGH si hay poca luz (valor ADC <= límite)
uint8_t checkPR() {
  uint16_t value = analogRead(PR);
  Serial.print("PR ADC: ");
  Serial.println(value);
  
  if (value <= PRLimit) {
    return SENSOR_HIGH;  // Poca luz
  }
  return SENSOR_LOW;  // Luz normal/alta
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
  Serial.begin(9600);
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
      
      Serial.print("US: ");
      Serial.print(us1);
      Serial.print("cm, ");
      Serial.print(us2);
      Serial.print("cm -> ");
      Serial.println(usPayload);

      sendSensorPublish(DISTANCE, usPayload);
    }
  }

  now = millis();

  // ---------- Sensor de luz ----------
  if (now - lastPR >= DELAY) {
    lastPR = now;

    uint8_t prPayload = checkPR();
    Serial.print("LIGHT payload: ");
    Serial.println(prPayload);
    
    sendSensorPublish(LIGHT, prPayload);
  }
}

