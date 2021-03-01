#include <Arduino.h>
#include "txcoilutil.h"

volatile uint32_t txFreq;       //herz
volatile double  quaterPeriod; //micros

bool txHigh;

hw_timer_t *txTimer = NULL;
portMUX_TYPE txTimerMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t getQuaterPeriod()
{
    return quaterPeriod;
}

void IRAM_ATTR onTxTimer()
{
    portENTER_CRITICAL_ISR(&txTimerMux);
    if (txHigh)
    {
        digitalWrite(TX_PIN, HIGH);
    }
    else
    {
        digitalWrite(TX_PIN, LOW);
    }
    txHigh = !txHigh;
    portEXIT_CRITICAL_ISR(&txTimerMux);

    // if (quadrant==5)
    // {
    //   quadrant = 1;
    // }
    // if (quadrant==1)
    // {
    //   usleep(3);

    // }
    // if (quadrant==2)
    // {
    //   cX+=getRXValue();
    //   xCounter++;
    // }
    // if (quadrant==3)
    // {
    //   usleep(3);

    // }
    // if (quadrant==4)
    // {
    //   cY+=getRXValue();
    //   yCounter++;
    // }
    // quadrant++;

    // if (yCounter==100)
    // {
    //   tx_stop();
    // }

    //   portENTER_CRITICAL(&timerMux);
    //   cX=0;
    //   cY=0;
    //  for (int i = 0; i <= 80; i++)
    //    {
    //      digitalWrite(TX_PIN, HIGH);
    //      txLow = false;
    //      p0 = getRXValue();
    //      usleep(23);

    //      p90 = getRXValue();
    //      usleep(23);

    //      digitalWrite(TX_PIN, LOW);
    //      txLow = true;
    //      p180 = getRXValue();
    //      usleep(23);

    //      p270 = (uint32_t)getRXValue();
    //      usleep(23);

    //      if (i>30)
    //      {
    //        cX += (p180 - p0);
    //        cY += (p270 - p90);
    //      }
    //    }
    //    tx_stop();
    //   portEXIT_CRITICAL(&timerMux);
}


void tx_start(uint16_t freq)
{
    txFreq = freq;
    quaterPeriod = (1.0 / (freq) / 4) * 1000000L;

    Serial.println("quater period time " + String(quaterPeriod));

    txHigh = false;
    digitalWrite(TX_PIN, LOW);

    txTimer = timerBegin(0, 8, true);
    timerAttachInterrupt(txTimer, &onTxTimer, true);
    timerAlarmWrite(txTimer, quaterPeriod*10*2, true); 
    timerAlarmEnable(txTimer);

    delayMicroseconds(quaterPeriod * 4 * 50);
}

void tx_stop()
{
    txHigh = false;
    digitalWrite(TX_PIN, LOW);
    timerAlarmDisable(txTimer);
}

uint16_t searchResonanceFreq(uint16_t startFreq, uint16_t endFreq)
{
    return 0;
}


void init()
{
 
}