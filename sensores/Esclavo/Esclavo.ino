/* ----------------------------------------------------------------------
 *  Dispositivo esclavo con sensores + OLED
 * ---------------------------------------------------------------------- 
 */

 //José Manuel Díaz Hernández
 //Nicolás Rey Alonso

#include <Wire.h>

#define SRF01_I2C_ADDRESS byte((0xE0)>>1)
#define SRF02_I2C_ADDRESS byte((0xF2)>>1)
#define SRF02_CMD_REG 0x00
#define SRF02_RANGE_HIGH 0x02

// OLED display removed — using Serial for output

#define NUM_SENSORS 3

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
constexpr uint8_t RX_PIN = 8;
constexpr uint8_t TX_PIN = 9;
SoftwareSerial Serial1(RX_PIN, TX_PIN);  // RX, TX
#endif


struct SensorConfig {
  uint8_t address;
  uint8_t unit;
  uint16_t delayMs;
  bool periodic;
  uint16_t periodMs;
  unsigned long lastShot;
  uint16_t lastMeasure;
  bool active;
  char name[8];
};

SensorConfig sensors[NUM_SENSORS] = {
  {SRF01_I2C_ADDRESS, 1, 70, false, 0, 0, 0, false, "SRF01"},
  {SRF02_I2C_ADDRESS, 1, 70, false, 0, 0, 0, false, "SRF02"},
  {0xFF, 3, 70, false, 0, 0, 0, true, "LDR"} // sensor de luz (analógico)
};

bool isI2CDeviceAvailable(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

uint16_t readSRF02(uint8_t address, uint8_t unit) {
  uint8_t cmd = 0x51;  // cm
  if (unit == 2) cmd = 0x50;  // inches
  if (unit == 0) cmd = 0x56;  // microseconds

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

void sendResponse(uint8_t code, uint8_t* data = nullptr, uint8_t len = 0) {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial1.write(code);
  if (data && len > 0) {
    Serial1.write(data, len);
  }
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

// OLED removed — status shown via Serial

void handleCommand(uint8_t code) {
  uint8_t header = code & 0xF0;
  
  // HELP (0x00)
  if (code == 0x00) {
    sendResponse(0x00);
    return;
  }
  
  // US LIST (0x80)
  if (code == 0x80) {
    uint8_t data[1 + NUM_SENSORS * 2];
    data[0] = NUM_SENSORS;
    
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      data[1 + i * 2] = sensors[i].address;
      data[1 + i * 2 + 1] = sensors[i].active ? 0x01 : 0x00;
    }
    
    sendResponse(0x80, data, 1 + NUM_SENSORS * 2);
    return;
  }
  
  // UNIT (0xD0-0xDF)
  if (header == 0xD0) {
    uint8_t unit_bits = (code >> 2) & 0b11;
    
    while (Serial1.available() < 1);
    uint8_t sensor_id = Serial1.read();
    
    if (sensor_id >= NUM_SENSORS) {
      sendResponse(0xFF);
      return;
    }
    
    sensors[sensor_id].unit = unit_bits;
    sendResponse(0x00);
    return;
  }
  
  // RANGING (0xC0-0xCF)
  if (header == 0xC0) {
    uint8_t mode_bits = (code >> 2) & 0b11;
    
    while (Serial1.available() < 3);
    uint8_t sensor_id = Serial1.read();
    uint8_t period_low = Serial1.read();
    uint8_t period_high = Serial1.read();
    uint16_t period = (period_high << 8) | period_low;
    
    if (sensor_id >= NUM_SENSORS || !sensors[sensor_id].active) {
      sendResponse(0xFF);
      return;
    }
    
    SensorConfig &s = sensors[sensor_id];
    
    if (mode_bits == 0b00) {  // one-shot
      if (s.address == 0xFF) {
        s.lastMeasure = analogRead(A1);
      } else {
        s.lastMeasure = readSRF02(s.address, s.unit);
      }
      
      // Print reading to serial monitor
      Serial.print("Sensor ");
      Serial.print(sensor_id);
      Serial.print(" ");
      Serial.print(s.name);
      Serial.print(": ");
      if (s.lastMeasure == 0xFFFF) {
        Serial.println("ERR");
      } else {
        Serial.print(s.lastMeasure);
        const char* unitStr = (s.unit == 1) ? "cm" : (s.unit == 2) ? "in" : "us";
        Serial.print(" ");
        Serial.println(unitStr);
      }

      uint8_t data[4];
      data[0] = sensor_id;
      data[1] = s.unit;
      data[2] = (s.lastMeasure >> 8) & 0xFF;
      data[3] = s.lastMeasure & 0xFF;
      
      sendResponse(0x10, data, 4);
    }
    else if (mode_bits == 0b01) {  // off
      s.periodic = false;
      sendResponse(0x00);
    }
    else if (mode_bits == 0b10) {  // on

    if (period < 70) {
      sendResponse(0xFF);  // Período muy corto
      return;
    }
      s.periodic = true;
      s.periodMs = period;
      s.lastShot = millis();
      sendResponse(0x00);
    }
    return;
  }
  
  // DELAY (0xE0-0xEF)
  if (header == 0xE0) {
    while (Serial1.available() < 3);
    uint8_t sensor_id = Serial1.read();
    uint8_t delay_low = Serial1.read();
    uint8_t delay_high = Serial1.read();
    uint16_t delay_ms = (delay_high << 8) | delay_low;
    
    if (sensor_id >= NUM_SENSORS) {
      sendResponse(0xFF);
      return;
    }

    if (delay_ms < 70) {
      sendResponse(0xFF);  // Delay muy corto
      return;
    }
    
    sensors[sensor_id].delayMs = delay_ms;
    sendResponse(0x00);
    return;
  }
  
  // STATUS (0xF0-0xFF)
  if (header == 0xF0) {
    while (Serial1.available() < 1);
    uint8_t sensor_id = Serial1.read();
    
    if (sensor_id >= NUM_SENSORS) {
      sendResponse(0xFF);
      return;
    }
    
    SensorConfig &s = sensors[sensor_id];
    
    uint8_t data[5];
    data[0] = s.address;
    data[1] = (s.delayMs >> 8) & 0xFF;
    data[2] = s.delayMs & 0xFF;
    data[3] = s.unit;
    data[4] = s.periodic ? 0x00 : 0xFF;
    
    sendResponse(0x01, data, 5);
    return;
  }
  
  sendResponse(0xFF);
}

void setup() {
  Wire.begin();
  pinMode(A1, INPUT);
  Serial1.begin(9600);
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  delay(100);
  while (Serial1.available() > 0) {
    Serial1.read();
  }

  Serial.println("INICIANDO...");
  delay(1000);
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i].address == 0xFF) {
      sensors[i].active = true; // analog sensor LDR
    } else {
      sensors[i].active = isI2CDeviceAvailable(sensors[i].address);
    }
  }
  
  // updateOLEDStatus removed — show status via Serial
  Serial.println("Dispositivos I2C detectados:");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(" - "); Serial.print(sensors[i].name); Serial.print(": ");
    Serial.println(sensors[i].active ? "ON" : "OFF");
  }

  // Por defecto: activar mediciones periódicas y hacer una lectura inicial
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i].active) {
      sensors[i].periodic = true;
      sensors[i].periodMs = 1000; // 1s por defecto
      sensors[i].lastShot = millis();

      // lectura inicial (SRF02 I2C o LDR analógico)
      if (sensors[i].address == 0xFF) {
        sensors[i].lastMeasure = analogRead(A1);
      } else {
        sensors[i].lastMeasure = readSRF02(sensors[i].address, sensors[i].unit);
      }

      // imprimir por monitor serial
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(sensors[i].name);
      Serial.print(": ");
      if (sensors[i].lastMeasure == 0xFFFF) {
        Serial.println("ERR");
      } else {
        Serial.print(sensors[i].lastMeasure);
        const char* unitStr = (sensors[i].unit == 1) ? "cm" : (sensors[i].unit == 2) ? "in" : "us";
        Serial.print(" ");
        Serial.println(unitStr);
      }
    }
  }

  // actualizar OLED con las medidas iniciales

}

void loop() {
  if (Serial1.available()) {
    uint8_t code = Serial1.read();
    handleCommand(code);
  }

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
      
      // Print periodic reading to serial monitor
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(s.name);
      Serial.print(": ");
      if (s.lastMeasure == 0xFFFF) {
        Serial.println("ERR");
      } else {
        Serial.print(s.lastMeasure);
        const char* unitStr = (s.unit == 1) ? "cm" : (s.unit == 2) ? "in" : "us";
        Serial.print(" ");
        Serial.println(unitStr);
      }

      uint8_t data[4];
      data[0] = i;
      data[1] = s.unit;
      data[2] = (s.lastMeasure >> 8) & 0xFF;
      data[3] = s.lastMeasure & 0xFF;
      
      sendResponse(0x10, data, 4);
    }
  }
}
