/* ----------------------------------------------------------------------
 *  Supervisor - Envía comandos y recibe respuestas del dispositivo sensor
 *  Asignatura (GII-IC)
 * ---------------------------------------------------------------------- 
 */

 //José Manuel Díaz Hernández
 //Nicolás Rey Alonso

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

// =====================
// Configuración LoRa
// =====================
#define LORA_LOCAL_ADDRESS      0x04
#define LORA_GATEWAY_ADDRESS    0x05
#define LORA_FREQUENCY          868E6
#define LORA_BW                 62.5E3
#define LORA_SF                 10
#define LORA_CR                 5
#define LORA_TP                 2
#define LORA_SYNC_WORD          0x12
#define LORA_PREAMBLE_LENGTH    8

// =====================
// Umbrales de sensores
// =====================
#define LIGHT_THRESHOLD   500   // Si luz < 500 -> 1 (oscuro)
#define DISTANCE_THRESHOLD 100  // Si distancia < 100cm -> 1 (objeto cerca)

// =====================
// Topics
// =====================
#define TOPIC_LIGHT    "sensor/1"   // -> sensores/luz en Gateway
#define TOPIC_DISTANCE "sensor/0"   // -> sensores/puerta en Gateway

// =====================
// Variables globales
// =====================
uint16_t loraMessageCounter = 0;
unsigned long lastLoraSendTime = 0;
unsigned long txStartTime = 0;
const unsigned long LORA_SEND_INTERVAL = 500;  // 500ms entre actualizaciones (más rápido)

// Flag para alternar entre sensores: false = luz, true = distancia
volatile bool sendDistanceNext = false;

// Mensaje pendiente para cada sensor (solo 1 por sensor)
struct SensorMessage {
  uint8_t payload;
  bool pending;
  uint8_t lastSentValue;  // Último valor enviado para detectar cambios
};

SensorMessage lightMsg = {'0', true, 0xFF};   // pending=true para forzar primer envío
SensorMessage distanceMsg = {'0', true, 0xFF}; // 0xFF = nunca enviado

// Últimos valores de sensores (arrancan en 0 para forzar el primer envío)
uint8_t lastLightState = 0;
uint8_t lastDistanceState = 0;
uint16_t lastLightValue = 0;
uint16_t lastDistance1 = 0xFFFF;   // Inicializar alto para evitar falsos positivos
uint16_t lastDistance2 = 0xFFFF;   // Inicializar alto para evitar falsos positivos

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
constexpr uint8_t RX_PIN = 8;
constexpr uint8_t TX_PIN = 9;
SoftwareSerial Serial1(RX_PIN, TX_PIN);  // RX, TX
#endif



constexpr const uint32_t serial_monitor_bauds = 9600;
constexpr const uint32_t serial1_bauds = 9600;

String slaveBuffer = "";

// =====================
// Funciones LoRa
// =====================
bool lora_init() {
  if (!LoRa.begin(LORA_FREQUENCY)) return false;
  LoRa.setSignalBandwidth(long(LORA_BW));
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TP, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  // No usar callback - modo bloqueante
  return true;
}

/**
 * Envía un mensaje LoRa (interno) - MODO BLOQUEANTE
 */
void sendLoraPacket(const char* topic, uint8_t payload) {
  uint8_t topicLen = strlen(topic);
  uint8_t totalLen = 1 + topicLen + 1;

  LoRa.beginPacket();
  LoRa.write(LORA_GATEWAY_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write((uint8_t)(loraMessageCounter >> 8));
  LoRa.write((uint8_t)(loraMessageCounter & 0xFF));
  LoRa.write(totalLen);
  LoRa.write(topicLen);
  LoRa.write((const uint8_t*)topic, topicLen);
  LoRa.write(payload);
  
  // Modo BLOQUEANTE (false) - espera a que termine el envío
  if (LoRa.endPacket(false)) {
    loraMessageCounter++;
    txStartTime = millis();
    // Alternar sensor después de enviar
    sendDistanceNext = !sendDistanceNext;
    Serial.println("[LoRa] Paquete enviado OK");
  } else {
    Serial.println("[LoRa] ERROR: endPacket failed");
  }
}

/**
 * Procesa las colas alternando entre luz y distancia
 */
void processLoraQueue() {
  // Cooldown entre envíos (mínimo 200ms)
  if (millis() - txStartTime < 200) return;

  // Debug: mostrar estado de colas cada 5 segundos
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    lastDebug = millis();
    Serial.print("[Debug] lightPending=");
    Serial.print(lightMsg.pending);
    Serial.print(" distPending=");
    Serial.print(distanceMsg.pending);
    Serial.print(" turno=");
    Serial.println(sendDistanceNext ? "DIST" : "LUZ");
  }

  if (sendDistanceNext) {
    // Turno de distancia (Sensor 0)
    if (distanceMsg.pending) {
      Serial.println("[LoRa] Enviando DISTANCIA...");
      sendLoraPacket(TOPIC_DISTANCE, distanceMsg.payload);
      Serial.print("[LoRa TX] ");
      Serial.print(TOPIC_DISTANCE);
      Serial.print(" = ");
      Serial.println((char)distanceMsg.payload);
      distanceMsg.lastSentValue = distanceMsg.payload;
      distanceMsg.pending = false;
    } else {
      // No hay nada pendiente, pasar turno
      sendDistanceNext = false;
    }
  } else {
    // Turno de luz (Sensor 1)
    if (lightMsg.pending) {
      Serial.println("[LoRa] Enviando LUZ...");
      sendLoraPacket(TOPIC_LIGHT, lightMsg.payload);
      Serial.print("[LoRa TX] ");
      Serial.print(TOPIC_LIGHT);
      Serial.print(" = ");
      Serial.println((char)lightMsg.payload);
      lightMsg.lastSentValue = lightMsg.payload;
      lightMsg.pending = false;
    } else {
      // No hay nada pendiente, pasar turno
      sendDistanceNext = true;
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(serial_monitor_bauds);
  while (!Serial);
  delay(1000);

  Serial1.begin(serial1_bauds);

  // Inicializar LoRa
  if (!lora_init()) {
    Serial.println("ERROR: LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa init OK");

  Serial.println("=== SUPERVISOR INICIADO ===");
  Serial.println("Escribe 'help' para ver comandos disponibles");
  Serial.println("Enviando datos por LoRa cada 500ms...");
  
  // Forzar primer envío inmediato
  lightMsg.pending = true;
  distanceMsg.pending = true;
  lastLoraSendTime = millis();
  Serial.println("[LoRa] Primer envio programado");
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

  // Enviar datos por LoRa cada segundo (si tenemos datos válidos)
  unsigned long now = millis();
  if (now - lastLoraSendTime >= LORA_SEND_INTERVAL) {
    lastLoraSendTime = now;
    sendLoRaUpdates();
  }

  // Bombear cola LoRa si el último TX ha terminado
  processLoraQueue();
}

/**
 * Actualiza los mensajes pendientes - SOLO si hay cambios o es primera vez
 * Esto evita saturar el canal con datos repetidos
 */
void sendLoRaUpdates() {
  uint8_t newLightPayload = (lastLightState == 1) ? '1' : '0';
  uint8_t newDistPayload = (lastDistanceState == 1) ? '1' : '0';
  
  // Solo enviar luz si cambió o es primera vez (lastSentValue == 0xFF)
  // Y NO hay ya un mensaje pendiente
  if (newLightPayload != lightMsg.lastSentValue && !lightMsg.pending) {
    lightMsg.payload = newLightPayload;
    lightMsg.pending = true;
    Serial.print("[LoRa] Cambio LUZ: ");
    Serial.print((char)lightMsg.lastSentValue);
    Serial.print(" -> ");
    Serial.print((char)newLightPayload);
    Serial.print(" (raw: ");
    Serial.print(lastLightValue);
    Serial.println(")");
  }

  // Solo enviar distancia si cambió o es primera vez
  // Y NO hay ya un mensaje pendiente
  if (newDistPayload != distanceMsg.lastSentValue && !distanceMsg.pending) {
    distanceMsg.payload = newDistPayload;
    distanceMsg.pending = true;
    Serial.print("[LoRa] Cambio DIST: ");
    Serial.print((char)distanceMsg.lastSentValue);
    Serial.print(" -> ");
    Serial.print((char)newDistPayload);
    Serial.print(" (d1: ");
    Serial.print(lastDistance1);
    Serial.print("cm, d2: ");
    Serial.print(lastDistance2);
    Serial.println("cm)");
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

    String sensorLabel = (sensor_id == 2 || sensor_id == 3) ? "Luz" : String(sensor_id);
    Serial.print("[SLAVE] Medida - ");
    Serial.print(sensorLabel);
    Serial.print(": ");
    Serial.print(measure);
    Serial.print(" ");
    Serial.println(unidad);
    
    // Procesar según tipo de sensor y aplicar umbrales
    if (sensor_id == 2 || sensor_id == 3) {
      // Sensor de luz (puede ser sensor_id 2 o 3 según configuración del esclavo)
      lastLightValue = measure;
      lastLightState = (measure < LIGHT_THRESHOLD) ? 1 : 0;
      Serial.print("  -> Estado luz: ");
      Serial.println(lastLightState ? "OSCURO (1)" : "ILUMINADO (0)");
    } else if (sensor_id == 0 || sensor_id == 1) {
      // Sensores de distancia (ultrasonidos)
      if (sensor_id == 0) {
        lastDistance1 = measure;
      } else {
        lastDistance2 = measure;
      }
      // Actualizar estado: 1 si alguno detecta objeto cerca
      if (lastDistance1 < DISTANCE_THRESHOLD || lastDistance2 < DISTANCE_THRESHOLD) {
        lastDistanceState = 1;
      } else {
        lastDistanceState = 0;
      }
      Serial.print("  -> Estado distancia: ");
      Serial.println(lastDistanceState ? "OBJETO CERCA (1)" : "LIBRE (0)");
    }
    
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
