#include <Arduino.h>
#include "voltageregulator.h"



void X9C_init()
{
  pinMode(CS, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(UD, OUTPUT);
  digitalWrite(CS, HIGH); // X9C в режиме низкого потребления
  digitalWrite(INC, HIGH);
  digitalWrite(UD, HIGH);
}

void setResistance(int percent)
{
  // Понижаем сопротивление до 0%:
  digitalWrite(UD, LOW); // выбираем понижение
  digitalWrite(CS, LOW); // выбираем потенциометр X9C
  for (int i = 0; i < 100; i++)
  { // т.к. потенциометр имеет 100 доступных позиций
    digitalWrite(INC, LOW);
    delayMicroseconds(1);
    digitalWrite(INC, HIGH);
    delayMicroseconds(1);
  }

  // Поднимаем сопротивление до нужного:
  digitalWrite(UD, HIGH);
  for (int i = 0; i < percent; i++)
  {
    digitalWrite(INC, LOW);
    delayMicroseconds(1);
    digitalWrite(INC, HIGH);
    delayMicroseconds(1);
  }

  digitalWrite(CS, HIGH); /* запоминаем значение 
  и выходим из режима настройки */
}
