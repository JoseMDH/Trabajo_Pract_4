#include <Servo.h>

Servo miServo;  // Crear objeto servo
int pinServo = 7;  // Pin donde está conectado el servo
int posicion = 90;  // Posición inicial (centro)

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serial
  miServo.attach(pinServo);  // Asignar el servo al pin
  miServo.write(posicion);  // Posición inicial
  
  Serial.println("Control de Servo iniciado");
  Serial.println("Escribe 'r' para girar a la derecha");
  Serial.println("Escribe 'l' para girar a la izquierda");
}

void loop() {
  if (Serial.available() > 0) {  // Si hay datos disponibles
    char comando = Serial.read();  // Leer el caracter
    
    if (comando == 'o' || comando == 'O') {
      posicion = 160;  // open door
      miServo.write(posicion);
      Serial.println("Girando a la DERECHA (180°)");
    }
    else if (comando == 'c' || comando == 'C') {
      posicion = 10o;  // Close door
      miServo.write(posicion);
      Serial.println("Girando a la IZQUIERDA (0°)");
    }
  }
}