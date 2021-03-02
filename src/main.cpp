#include <Arduino.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include "SPI.h"
#include "BluetoothSerial.h"
#include "txcoilutil.h"
#include "rxcoilutil.h"

//#define TX_PIN 32
#define Upit 27

#define LED_PIN 22
#define UCOMP_PIN 25

#define CS 4
#define INC 17
#define UD 16

// void onTimer();
// void onTimer2();

void X9C_init();
// void tx_start();
// void tx_stop();
void setResistance(int percent);
float getUpitValue();
uint16_t getTXCurrent();
IRAM_ATTR void getData(uint16_t data[100]);

int64_t tx_upfront_time;

int rx_current_value = 0;
int rx_previous_value = 0;

int64_t rx_current_value_time = 0;
int64_t rx_previous_value_time = 0;

// hw_timer_t *timer = NULL;

// hw_timer_t *timer2 = NULL;

// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

int64_t period_time;

bool txLow;

int rx_index = 0;

BluetoothSerial ESP_BT;
int incoming;

//byte quadrant;

int32_t volatile cX;
int32_t volatile cY;

void setup()
{
  rx_init();

  tx_init();

   //TXCurrent ADC setup
  adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_0);

  X9C_init();

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

float getUpitValue()
{
  int val = adc1_get_raw(ADC1_CHANNEL_5);
  return (val / 4096.0) * 3.3 * (3.3 + 18) / 3.3;
}
