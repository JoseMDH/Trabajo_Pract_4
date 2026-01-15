#include <Servo.h>

Servo miServo;

int pinServo = 7;   // Servo
int pinLed   = 6;   // LED en D6
int posicion = 90;

String comando = "";

void setup() {
  Serial.begin(9600);

  miServo.attach(pinServo);
  miServo.write(posicion);

  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  Serial.println("Sistema iniciado");
  Serial.println("Comandos:");
  Serial.println("  o   -> abrir servo");
  Serial.println("  c   -> cerrar servo");
  Serial.println("  on  -> encender LED");
  Serial.println("  off -> apagar LED");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Fin de línea → procesar comando
    if (c == '\n' || c == '\r') {
      comando.trim();   // limpiar espacios

      if (comando == "o" || comando == "O") {
        posicion = 160;
        miServo.write(posicion);
        Serial.println("Servo abierto");
      }
      else if (comando == "c" || comando == "C") {
        posicion = 10;
        miServo.write(posicion);
        Serial.println("Servo cerrado");
      }
      else if (comando == "on") {
        digitalWrite(pinLed, HIGH);
        Serial.println("LED encendido");
      }
      else if (comando == "off") {
        digitalWrite(pinLed, LOW);
        Serial.println("LED apagado");
      }
      else {
        Serial.print("Comando desconocido: ");
        Serial.println(comando);
      }

      comando = "";  // limpiar buffer
    }
    else {
      comando += c;  // acumular caracteres
    }
  }
}
