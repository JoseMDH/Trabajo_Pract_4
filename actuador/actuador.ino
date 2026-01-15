#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

Servo miServo;

int pinServo = 7;
int pinLed   = 6;
int posicion = 10;
uint8_t localAddress = 0x06;

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

  miServo.attach(pinServo);
  miServo.write(posicion);

  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  if (!LoRa.begin(868E6)) {
  Serial.println("Error LoRa");
  while (1);
  }
  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("Actuador listo (tipo: 0=luz, 1=puerta)");
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t recipient = LoRa.read();
  uint8_t sender    = LoRa.read();
  uint16_t msgID    = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t  msgLen   = LoRa.read();

  if (recipient != localAddress && recipient != 0xFF) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  if (LoRa.available() < 2) {
    Serial.println("Faltan bytes de tipo/valor");
    while (LoRa.available()) LoRa.read();
    return;
  }

  uint8_t tipo  = LoRa.read();   // 0 = luz, 1 = puerta
  uint8_t valor = LoRa.read();

  Serial.print("Tipo=");
  Serial.print(tipo);
  Serial.print(" valor=");
  Serial.println(valor);

  if (tipo == 0) {
    aplicarLuz(valor);
  } else if (tipo == 1) {
    aplicarPuerta(valor);
  } else {
    Serial.println("Tipo desconocido");
  }

  while (LoRa.available()) LoRa.read();
}

void loop() {
}
