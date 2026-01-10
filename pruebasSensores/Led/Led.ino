int ledPin = 12; // Pin digital donde conectas el LED
String inputString = ""; // Variable para guardar lo que escribes

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Inicia comunicación serie a 9600 baudios
  Serial.println("Escribe 'on' para encender el LED y 'off' para apagarlo");
}

void loop() {
  // Leer datos del puerto serie
  while (Serial.available() > 0) {
    char c = Serial.read();  // Leer un carácter
    if (c == '\n' || c == '\r') {  // Si presionaste Enter
      inputString.trim();   // Eliminar espacios
      if (inputString == "on") {
        digitalWrite(ledPin, HIGH);
        Serial.println("LED encendido");
      } else if (inputString == "off") {
        digitalWrite(ledPin, LOW);
        Serial.println("LED apagado");
      } else {
        Serial.println("Comando no reconocido");
      }
      inputString = ""; // Vaciar la cadena para el siguiente comando
    } else {
      inputString += c; // Agregar carácter a la cadena
    }
  }
}
