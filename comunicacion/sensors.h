#ifndef SENSORS
#define SENSORS

#define US1 0xE0
#define US1 0xE2
#define PR A1

uint16_t readPR();
uint16_t readUS(uint8_t dir);

#endif