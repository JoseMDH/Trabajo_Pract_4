#ifndef MQTTPUBLISH
#define MQTTPUBLISH

#define TYPE 0b0011

typedef {
  uint8_t headerLow = 0b00000011;
  uint8_t remainingLength = 4;
  uint8_t topicLengthLow = 1;
  uint8_t topicLengthHigh = 0;
  uint8_t topic;
  uint8_t payload;
} PublishMessage

#define LOW 0b0 // us long distance / nothing, high light
#define HIGH 0b1 // us short distance , low light

// us payload: 2 bits = 00,01,10,11
// light payload: 1 bit = 0,1
// topics: LUZ = 0, DISTANCIA = 1
#endif