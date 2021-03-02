#include <Arduino.h>
#include <driver/adc.h>
#include "compensator.h"


float getUpitValue()
{
  int val = adc1_get_raw(ADC1_CHANNEL_5);
  return (val / 4096.0) * 3.3 * (3.3 + 18) / 3.3;
}