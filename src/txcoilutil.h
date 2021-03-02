#include <Arduino.h>

#define TX_PIN 32

void tx_start(uint16_t freq);
void tx_stop();
void getXY(int32_t &xC, int32_t &yC);
uint16_t searchResonanceFreq(uint16_t startFreq, uint16_t endFreq);