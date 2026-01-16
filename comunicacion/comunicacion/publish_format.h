#ifndef PUBLISH
#define PUBLISH

#define TYPE 0b0011

/*
Serial format.
[0] topicId (0=LIGHT, 1=DISTANCE)
[1] payload flag (0=LOW, 1=HIGH)
*/

#define LIGHT 1
#define DISTANCE 0

#define USLimit 100 //centimeters
#define PRLimit 500 // valor ADC

// Valores numéricos para el payload
#define SENSOR_LOW   0   // us: distancia larga (no hay objeto) / luz: alta
#define SENSOR_HIGH  1   // us: distancia corta (hay objeto) / luz: baja

#endif