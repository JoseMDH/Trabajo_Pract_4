#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

Servo miServo;

int pinServo = 7;
int pinLed   = 6;
int posicion = 10;
uint8_t localAddress = 0x06;
const uint8_t senderAddress = 0x05;

// =====================
// Configuración LoRa (igual que Gateway)
// =====================
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3};

LoRaConfig_t nodeConfig = {6, 10, 5, 2};  // BW=62.5kHz, SF=10, CR=4/5, TxPwr=2dBm

void aplicarPuerta(uint8_t v) {
  if (v == 0) {          // 00 -> cerrar
    posicion = 10;
    miServo.write(posicion);
    Serial.println("Puerta -> CERRAR (00)");
  } else if (v == 1 || v == 2 || v == 3) {
    posicion = 160;
    miServo.write(posicion);
    Serial.print("Puerta -> ABRIR/MANTENER (");
    Serial.print(v);
    Serial.println(")");
  } else {
    Serial.print("Código puerta desconocido: ");
    Serial.println(v);
  }
}

void aplicarLuz(uint8_t v) {
  if (v == 1) {
    digitalWrite(pinLed, HIGH);
    Serial.println("Luz -> ENCENDER (1)");
  } else if (v == 0) {
    digitalWrite(pinLed, LOW);
    Serial.println("Luz -> APAGAR (0)");
  } else {
    Serial.print("Código luz desconocido: ");
    Serial.println(v);
  }
}

void onReceive(int packetSize);  // prototipo arriba

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }  // Esperar a que Serial esté listo
  delay(1000);  // Dar tiempo extra para inicialización
  
  Serial.println("\n\n=== INICIANDO ACTUADOR ===");

  miServo.attach(pinServo);
  miServo.write(posicion);

  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);
  Serial.println("LED y Servo configurados OK");

  Serial.println("Inicializando LoRa...");
  if (!LoRa.begin(868E6)) {
    Serial.println("ERROR: No se pudo inicializar LoRa!");
    Serial.println("Verifica las conexiones del modulo LoRa");
    while (1) {
      // Parpadear LED para indicar error
      digitalWrite(pinLed, HIGH);
      delay(200);
      digitalWrite(pinLed, LOW);
      delay(200);
    }
  }
  
  // Configuración LoRa (igual que Gateway)
  LoRa.setSignalBandwidth(long(bandwidth_kHz[nodeConfig.bandwidth_index]));
  LoRa.setSpreadingFactor(nodeConfig.spreadingFactor);
  LoRa.setCodingRate4(nodeConfig.codingRate);
  LoRa.setTxPower(nodeConfig.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);
  
  // Activar callback de recepción
  LoRa.onReceive(onReceive);
  LoRa.receive();
  
  Serial.println("Actuador listo (tipo: 0=luz, 1=puerta)");
  Serial.print("Direccion local: 0x");
  Serial.println(localAddress, HEX);
  Serial.print("Config LoRa: BW=");
  Serial.print(bandwidth_kHz[nodeConfig.bandwidth_index]/1000);
  Serial.print("kHz, SF=");
  Serial.print(nodeConfig.spreadingFactor);
  Serial.print(", CR=4/");
  Serial.print(nodeConfig.codingRate);
  Serial.print(", TxPwr=");
  Serial.print(nodeConfig.txPower);
  Serial.println("dBm");
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  Serial.println("\n========== PAQUETE RECIBIDO ==========");
  Serial.print("Tamaño del paquete: ");
  Serial.println(packetSize);
  Serial.print("RSSI: ");
  Serial.print(LoRa.packetRssi());
  Serial.println(" dBm");
  Serial.print("SNR: ");
  Serial.print(LoRa.packetSnr());
  Serial.println(" dB");

  uint8_t recipient = LoRa.read();
  uint8_t sender    = LoRa.read();
  uint16_t msgID    = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t  msgLen   = LoRa.read();

  Serial.print("Destinatario: 0x");
  Serial.println(recipient, HEX);
  Serial.print("Remitente: 0x");
  Serial.println(sender, HEX);
  Serial.print("ID Mensaje: ");
  Serial.println(msgID);
  Serial.print("Longitud payload: ");
  Serial.println(msgLen);

  // Leer todo el payload restante para mostrarlo
  uint8_t rawPayload[64];
  uint8_t rawLen = 0;
  while (LoRa.available() && rawLen < 64) {
    rawPayload[rawLen++] = LoRa.read();
  }
  
  Serial.print("Payload raw (hex): ");
  for (int i = 0; i < rawLen; i++) {
    if (rawPayload[i] < 0x10) Serial.print("0");
    Serial.print(rawPayload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Payload raw (dec): ");
  for (int i = 0; i < rawLen; i++) {
    Serial.print(rawPayload[i]);
    Serial.print(" ");
  }
  Serial.println();

  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("-> IGNORADO: No es para mi");
    Serial.println("=======================================\n");
    LoRa.receive();  // Volver a escuchar
    return;
  }

  // Solo aceptar si el emisor es 0x05
  if (sender != senderAddress) {
    Serial.print("-> IGNORADO: Emisor no autorizado 0x");
    Serial.println(sender, HEX);
    Serial.println("=======================================\n");
    LoRa.receive();  // Volver a escuchar
    return;
  }

  if (rawLen < 2) {
    Serial.println("-> ERROR: Faltan bytes de tipo/valor");
    Serial.println("=======================================\n");
    LoRa.receive();  // Volver a escuchar
    return;
  }

  uint8_t tipo  = rawPayload[0];   // 0 = luz, 1 = puerta
  uint8_t valor = rawPayload[1];

  Serial.print("-> Tipo=");
  Serial.print(tipo);
  Serial.print(" (");
  Serial.print(tipo == 0 ? "luz" : tipo == 1 ? "puerta" : "desconocido");
  Serial.print("), Valor=");
  Serial.println(valor);

  if (tipo == 0) {
    aplicarLuz(valor);
  } else if (tipo == 1) {
    aplicarPuerta(valor);
  } else {
    Serial.println("-> Tipo desconocido, ignorando");
  }
  
  Serial.println("=======================================\n");
  
  // IMPORTANTE: Volver a activar modo recepción
  LoRa.receive();
}

void loop() {
  LoRa.receive();
}
