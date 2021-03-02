#include <Arduino.h>
#include "SPI.h"
#include "rxcoilutil.h"

#define CLK 14
#define MISO 12
#define SS 15



IRAM_ATTR uint16_t getRXValue()
{
  digitalWrite(SS, HIGH);
  usleep(3);
  digitalWrite(SS, LOW);
  return SPI.transfer16(0x00); // / 65535.0 * 5 - 2.5;
}

void rx_init()
{
  pinMode(SS, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CLK, OUTPUT);

  SPI.begin(CLK, MISO, -1, SS);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(20000000L);
}