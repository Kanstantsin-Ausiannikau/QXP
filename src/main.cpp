#include <Arduino.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include "SPI.h"
#include "BluetoothSerial.h"
#include "txcoilutil.h"
#include "rxcoilutil.h"
#include "voltageregulator.h"
#include "compensator.h"

//#define TX_PIN 32
//#define Upit 27

#define LED_PIN 22
#define UCOMP_PIN 25


// void onTimer();
// void onTimer2();

// void tx_start();
// void tx_stop();

float getUpitValue();

// hw_timer_t *timer = NULL;

// hw_timer_t *timer2 = NULL;

// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

BluetoothSerial ESP_BT;
int incoming;


void setup()
{
  rx_init();
  tx_init();
  X9C_init();

   //TXCurrent ADC setup
  adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_0);

  delayMicroseconds(1000);

  Serial.begin(115200);                   // Запускаем последовательный монитор со скоростью 9600
  ESP_BT.begin("QXP detector");           // Задаем имя вашего устройства Bluetooth
  Serial.println("QXP is Ready to Pair"); // По готовности сообщаем, что устройство готово к сопряжению

  pinMode(TX_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  dac_output_enable(DAC_CHANNEL_1); //компенсация - 2.5 вольта для средней точки
  dac_output_voltage(DAC_CHANNEL_1, 200);

  //tx_start(8928);
  //tx_start(9165);

  
  uint16_t freq =  searchResonanceFreq(4000, 16000, 200);
  Serial.println("Freq iteration 1 = "+ String(freq));

  freq =  searchResonanceFreq(freq-100, freq+100, 5);
  Serial.println("Freq iteration 2 = "+ String(freq));

  tx_start(freq);

  delay(1000);
}



// void IRAM_ATTR onTimer2()
// {
//   portENTER_CRITICAL_ISR(&timer2Mux);

//   // double vdi;
//   // double amp;
//    //tx_start();
//    usleep(100000);

//   // vdi = atan2(cX, cY) * 57;
//   // amp = sqrt((double)cX * (double)cX / 10000 + (double)cY * (double)cY / 10000);

//   // portEXIT_CRITICAL_ISR(&timer2Mux);

//    Serial.print(cX/50);
//    Serial.print(";");
//    Serial.println(cY / 50);
//   // Serial.print(vdi);
//   // Serial.print(";");
//   // Serial.println(amp);
//   // Serial.print("time = ");

// }

// void IRAM_ATTR onTimer()
// {
//   // portENTER_CRITICAL_ISR(&timerMux);

//   // if (quadrant==5)
//   // {
//   //   quadrant = 1;
//   // }
//   // if (quadrant==1)
//   // {
//   //   usleep(3);
//   //   digitalWrite(TX_PIN, HIGH);
//   //   txLow = false;
//   // }
//   // if (quadrant==2)
//   // {
//   //   cX+=getRXValue();
//   //   xCounter++;
//   // }
//   // if (quadrant==3)
//   // {
//   //   usleep(3);
//   //   txLow = true;
//   //   digitalWrite(TX_PIN, LOW);
//   // }
//   // if (quadrant==4)
//   // {
//   //   cY+=getRXValue();
//   //   yCounter++;
//   // }
//   // quadrant++;

//   // if (yCounter==100)
//   // {
//   //   tx_stop();
//   // }
//   // portEXIT_CRITICAL_ISR(&timerMux);
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
// }

void BT_getData()
{
  if (ESP_BT.available()) // Проверяем, не получили ли мы что-либо от Bluetooth модуля
  {
    incoming = ESP_BT.read(); // Читаем, что получили
    Serial.print("Received:");
    Serial.println(incoming);

    if (incoming == 49) // Если значение равно единице, включаем светодиод
    {
      //digitalWrite(LED_BUILTIN, HIGH);
      ESP_BT.println("LED turned ON");
    }

    if (incoming == 48) // Если значение равно нулю, выключаем светодиод
    {
      //digitalWrite(LED_BUILTIN, LOW);
      ESP_BT.println("LED turned OFF");
    }
  }
}

void loop()
{

  int32_t dx=0;
  int32_t dy=0;
  getXY(dx,dy);
  Serial.println(String(dx) + ";" + String(dy));
  //Serial.println(atan2((float)dx/dy)*57);



  //for (int i=0;i<500;i++)
  //{
  //Serial.println(currentSum);
  //}

  delay(500);
}


