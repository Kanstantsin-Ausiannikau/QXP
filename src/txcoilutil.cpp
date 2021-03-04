#include <Arduino.h>
#include "driver/i2s.h"
#include <driver/adc.h>
#include "txcoilutil.h"
#include "rxcoilutil.h"

volatile uint32_t txFreq;     //herz
volatile double quaterPeriod; //micros

volatile int32_t x, y;
volatile int16_t counter;
volatile bool txFlag;
volatile uint8_t quadrant;

volatile int32_t p0;
volatile int32_t p90;
volatile int32_t p180;
volatile int32_t p270;

bool txHigh;

hw_timer_t *txTimer = NULL;
portMUX_TYPE txTimerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *compensatorTimer = NULL;
portMUX_TYPE compensatorTimerMux = portMUX_INITIALIZER_UNLOCKED;


uint8_t buffer[36];

uint32_t getQuaterPeriod()
{
    return quaterPeriod;
}

void getXY(int32_t &xC, int32_t &yC)
{
    x = 0;
    y = 0;
    counter = 0;
    txFlag = true;

    delayMicroseconds((uint32_t)quaterPeriod * 5 * 50);

    xC = x;
    yC = y;
}

void IRAM_ATTR onTxTimer()
{
    portENTER_CRITICAL_ISR(&txTimerMux);
    if (quadrant == 5)
        quadrant = 1;

    if (quadrant == 1)
    {
        if (txFlag)
        {
            p0 = getRXValue();
        }
        digitalWrite(TX_PIN, HIGH);
    }
    if (quadrant == 2)
    {
        if (txFlag)
        {
            p90 = getRXValue();
        }
    }
    if (quadrant == 3)
    {
        if (txFlag)
        {
            p180 = getRXValue();
        }

        digitalWrite(TX_PIN, LOW);
    }
    if (quadrant == 4)
    {
        if (txFlag)
        {
            p270 = getRXValue();
        }
        counter++;
    }
    quadrant++;

    if (txFlag)
    {
        x += (p180 - p0);
        y += (p270 - p90);
        counter++;
    }

    if (counter == 50)
    {
        x = x / 50;
        y = y / 50;
        counter = 0;
        txFlag = false;
    }
    portEXIT_CRITICAL_ISR(&txTimerMux);
}

void IRAM_ATTR onCompensatorTimer()
{
    portENTER_CRITICAL_ISR(&compensatorTimerMux);
    portEXIT_CRITICAL_ISR(&compensatorTimerMux);
}

void tx_start(uint16_t freq)
{
    txFreq = freq;
    quaterPeriod = (1.0 / (freq) / 4) * 1000000L;

    txFlag = false;
    quadrant = 1;

    //Serial.println("quater period time " + String(quaterPeriod));

    txHigh = false;
    digitalWrite(TX_PIN, LOW);

    txTimer = timerBegin(0, 8, true);
    timerAttachInterrupt(txTimer, &onTxTimer, true);
    timerAlarmWrite(txTimer, quaterPeriod * 10, true);
    timerAlarmEnable(txTimer);

    int i2s_num = 0;   // I2S port number
    i2s_config_t i2s_config =
    {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = freq*36,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,   // Default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 36,
    .use_apll = false
    };

    i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);   // Pin 25

    for(int i=0;i<36;i++)
    {
       buffer[i] = 200+3*sin(i*10*3.14/180);
    }

    compensatorTimer = timerBegin(0, 8, true);
    timerAttachInterrupt(compensatorTimer, &onCompensatorTimer, true);
    timerAlarmWrite(compensatorTimer, (quaterPeriod*4)/36 * 10, true);
    timerAlarmEnable(compensatorTimer);

    delayMicroseconds(quaterPeriod * 4 * 50);

}

void tx_stop()
{
    txHigh = false;
    digitalWrite(TX_PIN, LOW);
    timerAlarmDisable(txTimer);
}

uint16_t searchResonanceFreq(uint16_t startFreq, uint16_t endFreq, uint16_t step)
{
  int16_t findedFreq = startFreq;
  int32_t freqCurrent = 0;

  for (int f = startFreq; f < endFreq; f = f + step)
  {
    tx_start(f);
    int32_t currentSum = 0;
    for (int i = 0; i < 5000; i++)
    {
        currentSum += getTXCurrent();
    }
    tx_stop();

    if (currentSum>freqCurrent)
    {
        freqCurrent = currentSum;
        findedFreq = f;
    }
  }
    return findedFreq;
}

void tx_init()
{
    //TX current ADC setup
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_0);
}

IRAM_ATTR uint16_t getTXCurrent()
{
    return adc1_get_raw(ADC1_CHANNEL_7);
}