/* ----------------------------------------------------------------------
 *  Supervisor - Envía comandos y recibe respuestas del dispositivo sensor
 *  Asignatura (GII-IC)
 * ---------------------------------------------------------------------- 
 */

 //José Manuel Díaz Hernández
 //Nicolás Rey Alonso


#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
constexpr uint8_t RX_PIN = 8;
constexpr uint8_t TX_PIN = 9;
SoftwareSerial Serial1(RX_PIN, TX_PIN);  // RX, TX
#endif



constexpr const uint32_t serial_monitor_bauds = 9600;
constexpr const uint32_t serial1_bauds = 9600;

String slaveBuffer = "";

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(serial_monitor_bauds);
  while (!Serial);
  delay(1000);

  Serial1.begin(serial1_bauds);

  Serial.println("=== SUPERVISOR INICIADO ===");
  Serial.println("Escribe 'help' para ver comandos disponibles");
}

void loop() {
  // Leer respuestas del esclavo
  while (Serial1.available() > 0) {
    uint8_t byte = Serial1.read();
    parseSlaveMessage(byte);
  }

  // Leer comandos del usuario
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseCommand(input);
  }
}

void parseCommand(String input) {
  input.trim();
  if (input.length() == 0) return;

  String tokens[5];
  int count = 0;
  int last = 0;

  for (int i = 0; i <= input.length(); i++) {
    if (input[i] == ' ' || i == input.length()) {
      tokens[count++] = input.substring(last, i);
      tokens[count - 1].trim();
      last = i + 1;
      if (count >= 5) break;
    }
  }

  // ===== HELP =====
  if (tokens[0] == "help") {
    Serial.println("\n[CMD] help");
    Serial.println("Comandos disponibles:");
    Serial.println("  help                              -> Muestra esta ayuda");
    Serial.println("  us                                -> Lista sensores disponibles");
    Serial.println("  us <id> one-shot                  -> Dispara una sola vez el sensor");
    Serial.println("  us <id> on <period_ms>            -> Activa disparo periódico");
    Serial.println("  us <id> off                       -> Detiene disparo periódico");
    Serial.println("  us <id> unit {inc|cm|ms}          -> Cambia la unidad de medida");
    Serial.println("  us <id> delay <ms>                -> Establece retardo entre disparos");
    Serial.println("  us <id> status                    -> Muestra configuración del sensor\n");

    uint8_t code = 0x00;
    Serial1.write(code);
    return;
  }

  // ===== US commands =====
  if (tokens[0] == "us") {
    if (count == 1) {
      Serial.println("[CMD] us - Listando sensores...");
      uint8_t code = 0x80;
      Serial1.write(code);
      return;
    }

    int srf = tokens[1].toInt();
    uint8_t sensor_id = srf & 0xFF;

    // ---- UNIT ----
    if (tokens[2] == "unit" && count >= 4) {
      uint8_t unit_bits = 0b00;
      if (tokens[3] == "ms") {
        unit_bits = 0b00;
      } else if (tokens[3] == "cm") {
        unit_bits = 0b01;
      } else if (tokens[3] == "inc") {
        unit_bits = 0b10;
      } else {
        Serial.print("[ERROR] Unidad inválida '");
        Serial.print(tokens[3]);
        Serial.println("'. Use: ms, cm, inc");
        return;
      }

      uint8_t header = 0b11010000 | (unit_bits << 2);

      Serial.print("[CMD] us ");
      Serial.print(srf);
      Serial.print(" unit ");
      Serial.println(tokens[3]);

      Serial1.write(header);
      Serial1.write(sensor_id);
      return;
    }

    // ---- one-shot / on / off ----
    if (tokens[2] == "one-shot" || tokens[2] == "on" || tokens[2] == "off") {
      uint8_t mode_bits = 0b00;
      if (tokens[2] == "one-shot") {
        mode_bits = 0b00;
      } else if (tokens[2] == "off") {
        mode_bits = 0b01;
      } else if (tokens[2] == "on") {
        mode_bits = 0b10;
      } else {
        Serial.print("[ERROR] Comando inválido '");
        Serial.print(tokens[2]);
        Serial.println("'. Use: one-shot, on, off");
        return;
      }

      uint16_t period = (tokens[2] == "on" && count >= 4) ? tokens[3].toInt() : 0;
      uint8_t header = 0b11000000 | (mode_bits << 2);

      Serial.print("[CMD] us ");
      Serial.print(srf);
      Serial.print(" ");
      Serial.print(tokens[2]);
      if (tokens[2] == "on") {
        Serial.print(" ");
        Serial.print(period);
        Serial.println(" ms");
      } else {
        Serial.println();
      }

      Serial1.write(header);
      Serial1.write(sensor_id);
      Serial1.write(period & 0xFF);
      Serial1.write(period >> 8);
      return;
    }

    // ---- delay ----
    if (tokens[2] == "delay" && count >= 4) {
      uint16_t delay_ms = tokens[3].toInt();
      uint8_t header = 0b11100000;

      Serial.print("[CMD] us ");
      Serial.print(srf);
      Serial.print(" delay ");
      Serial.print(delay_ms);
      Serial.println(" ms");

      Serial1.write(header);
      Serial1.write(sensor_id);
      Serial1.write(delay_ms & 0xFF);
      Serial1.write(delay_ms >> 8);
      return;
    }

    // ---- status ----
    if (tokens[2] == "status") {
      uint8_t header = 0b11110000;

      Serial.print("[CMD] us ");
      Serial.print(srf);
      Serial.println(" status");

      Serial1.write(header);
      Serial1.write(sensor_id);
      return;
    }

    Serial.println("[ERROR] Comando US no reconocido.");
  } else {
    Serial.println("[ERROR] Comando no reconocido. Escribe 'help' para ver comandos.");
  }
}

void parseSlaveMessage(uint8_t firstByte) {
  uint8_t header = firstByte & 0xF0;

  // Error (0xFF)
  if (firstByte == 0xFF) {
    Serial.println("[SLAVE] ERROR del dispositivo sensor");
    return;
  }

  // ACK (0x00)
  if (firstByte == 0x00) {
    Serial.println("[SLAVE] OK - Comando ejecutado correctamente");
    return;
  }

  // Lista de sensores (0x80)
  if (firstByte == 0x80) {
    while (Serial1.available() < 1)
      ;
    uint8_t numSensors = Serial1.read();

    Serial.println("[SLAVE] Sensores disponibles:");

    for (uint8_t i = 0; i < numSensors; i++) {
      while (Serial1.available() < 2)
        ;
      uint8_t addr = Serial1.read();
      uint8_t active = Serial1.read();

      Serial.print("  Sensor ");
      Serial.print(i);
      Serial.print(": I2C=0x");
      Serial.print(addr, HEX);
      Serial.print(" - ");
      Serial.println(active ? "ACTIVO" : "INACTIVO");
    }
    return;
  }

  // Medida (0x10 - header termina en ...10000)
  if ((firstByte & 0x0F) == 0x00 && (header == 0x10 || header == 0x00)) {
    while (Serial1.available() < 4)
      ;

    uint8_t sensor_id = Serial1.read();
    uint8_t unit_byte = Serial1.read();
    uint8_t measure_high = Serial1.read();
    uint8_t measure_low = Serial1.read();

    uint16_t measure = (measure_high << 8) | measure_low;

    String unidad = (unit_byte == 0) ? "ms" : (unit_byte == 1) ? "cm"
                                            : (unit_byte == 2) ? "inc"
                                                               : "unk";

    String sensorLabel = (sensor_id == 3) ? "Luz" : String(sensor_id);
    Serial.print("[SLAVE] Medida - ");
    Serial.print(sensorLabel);
    Serial.print(": ");
    Serial.print(measure);
    Serial.print(" ");
    Serial.println(unidad);
    return;
  }

  // Status (0x01)
  if (firstByte == 0x01) {
    while (Serial1.available() < 5)
      ;

    uint8_t i2c_addr = Serial1.read();
    uint8_t delay_high = Serial1.read();
    uint8_t delay_low = Serial1.read();
    uint8_t unit_byte = Serial1.read();
    uint8_t periodic_byte = Serial1.read();

    uint16_t delay_ms = (delay_high << 8) | delay_low;

    String unidad = (unit_byte == 0) ? "ms" : (unit_byte == 1) ? "cm"
                                            : (unit_byte == 2) ? "inc"
                                                               : "unk";

    String periodic = (periodic_byte == 0x00) ? "SI" : "NO";

    Serial.println("[SLAVE] Status del sensor:");
    Serial.print("  I2C Addr  : 0x");
    Serial.println(i2c_addr, HEX);
    Serial.print("  Delay     : ");
    Serial.print(delay_ms);
    Serial.println(" ms");
    Serial.print("  Unidad    : ");
    Serial.println(unidad);
    Serial.print("  Periódico : ");
    Serial.println(periodic);

    return;
  }

  Serial.print("[SLAVE] Código desconocido: 0x");
  Serial.println(firstByte, HEX);
}
