#ifndef PUBLISH
#define PUBLISH

#define TYPE 0b0011

/*
Serial format.
[0] topicId
[1] payload flag
*/

#define LIGHT 0
#define DISTANCE 1

#define USLimit 100 //centimeters
#define PRLimit 500 // lumens

#define LOW '0' // us long distance / nothing, high light
#define HIGH '1' // us short distance , low light

// us payload: 2 bits = 00,01,10,11
// light payload: 1 bit = 0,1
// topics: LUZ = 0, DISTANCIA = 1
#endif