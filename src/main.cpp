#include <Arduino.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include "SPI.h"
#include "BluetoothSerial.h"

#define TX_PIN 32
#define Upit 27
//#define TXCURRENT_PIN 26

#define CLK 14
#define MISO 12
#define SS 15

#define LED_PIN 22
#define UCOMP_PIN 25

#define CS 4
#define INC 17
#define UD 16

void onTimer();
void onTimer2();
void RTC_init();
void X9C_init();
void tx_start();
void tx_stop();
void setResistance(int percent);
float getUpitValue();
uint16_t getTXCurrent();
IRAM_ATTR void getData(uint16_t data[100]);
IRAM_ATTR uint16_t getRXValue();

float coil_freq = 8200;

//float period = 1.0 / 8200;
int64_t halfPeriodTime = (int64_t)(1.0 / (coil_freq * 2) * 1000000); //in us

int64_t tx_upfront_time;
//int64_t rx_upfront_time;

int rx_current_value = 0;
int rx_previous_value = 0;

int64_t rx_current_value_time = 0;
int64_t rx_previous_value_time = 0;

hw_timer_t *timer = NULL;

hw_timer_t *timer2 = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

int64_t period_time;

bool txLow;

int rx_index = 0;

BluetoothSerial ESP_BT;
int incoming;

byte quadrant;

int32_t volatile cX;
int32_t volatile cY;

int32_t p0;
int32_t p90;
int32_t p180;
int32_t p270;

void setup()
{
  quadrant = 1;
  cX = 0;
  cY = 0;


  //Upit ADC setup
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

  //TXCurrent ADC setup
  adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_0);

  X9C_init();

  delayMicroseconds(1000);

  Serial.begin(115200);                   // Запускаем последовательный монитор со скоростью 9600
  ESP_BT.begin("QXP detector");           // Задаем имя вашего устройства Bluetooth
  Serial.println("QXP is Ready to Pair"); // По готовности сообщаем, что устройство готово к сопряжению

  Serial.println("Booting");
  Serial.println(coil_freq);

  pinMode(TX_PIN, OUTPUT);
  //pinMode(TXCURRENT_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  dac_output_enable(DAC_CHANNEL_1); //компенсация - 2.5 вольта для средней точки
  dac_output_voltage(DAC_CHANNEL_1, 200);

  RTC_init();

  timer = timerBegin(0, 1, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 4800 / 4 * 10 *100, true);

  timer2 = timerBegin(1, 1, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 10000000, true);

  timerAlarmEnable(timer2);
}

void tx_start()
{
  quadrant = 1;

  timerAlarmEnable(timer);
}
void tx_stop()

{
  txLow = true;
  digitalWrite(TX_PIN, LOW);
  timerAlarmDisable(timer);
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

void RTC_init()
{
  pinMode(SS, OUTPUT);
  pinMode(MISO, INPUT);
  //pinMode(MOSI, OUTPUT);
  pinMode(CLK, OUTPUT);

  SPI.begin(CLK, MISO, -1, SS);
  //SPI.setBitOrder(MSBFIRST);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(20000000L);
  //SPI.setFrequency(5000L);
}

void IRAM_ATTR onTimer2()
{
  portENTER_CRITICAL_ISR(&timer2Mux);

  // double vdi;
  // double amp;
   tx_start();
   usleep(100000);

  // vdi = atan2(cX, cY) * 57;
  // amp = sqrt((double)cX * (double)cX / 10000 + (double)cY * (double)cY / 10000);

  // portEXIT_CRITICAL_ISR(&timer2Mux);

   Serial.print(cX/50);
   Serial.print(";");
   Serial.println(cY / 50);
  // Serial.print(vdi);
  // Serial.print(";");
  // Serial.println(amp);
  // Serial.print("time = ");

}

void IRAM_ATTR onTimer()
{
  // portENTER_CRITICAL_ISR(&timerMux);

  // if (quadrant==5)
  // {
  //   quadrant = 1;
  // }
  // if (quadrant==1)
  // {
  //   usleep(3);
  //   digitalWrite(TX_PIN, HIGH);
  //   txLow = false;
  // }
  // if (quadrant==2)
  // {
  //   cX+=getRXValue();
  //   xCounter++;
  // }
  // if (quadrant==3)
  // {
  //   usleep(3);
  //   txLow = true;
  //   digitalWrite(TX_PIN, LOW);
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
  // portEXIT_CRITICAL_ISR(&timerMux);
  portENTER_CRITICAL(&timerMux);
  cX=0;
  cY=0;
 for (int i = 0; i <= 80; i++)
   {
     digitalWrite(TX_PIN, HIGH);
     txLow = false;
     p0 = getRXValue();
     usleep(23);
     
     p90 = getRXValue();
     usleep(23);

     digitalWrite(TX_PIN, LOW);
     txLow = true;
     p180 = getRXValue();
     usleep(23);

     p270 = (uint32_t)getRXValue();
     usleep(23);

     if (i>30)
     {
       cX += (p180 - p0);
       cY += (p270 - p90);
     }
   }
   tx_stop();
  portEXIT_CRITICAL(&timerMux);
}

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

IRAM_ATTR uint16_t getRXValue()
{
  digitalWrite(SS, HIGH);
  usleep(3);
  digitalWrite(SS, LOW);
  return SPI.transfer16(0x00); // / 65535.0 * 5 - 2.5;
}



void loop()
{

  // long start = esp_timer_get_time();

  // float s =0;
  // for (int i=1;i<=1000000;i++)
  // {
  //   s+=atan2(1,i)*57;
  // }

  //Serial.println((long)esp_timer_get_time() - start);
}

float getUpitValue()
{
  int val = adc1_get_raw(ADC1_CHANNEL_5);
  return (val / 4096.0) * 3.3 * (3.3 + 18) / 3.3;
}

IRAM_ATTR void getData(uint16_t data[100])
{

  for (int i = 0; i < 100; i++)
  {
    data[i] = getTXCurrent();
  }
}

IRAM_ATTR uint16_t getTXCurrent()
{

  //  return analogRead(TXCURRENT_PIN);

  int read_raw;
  esp_err_t r = adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_12Bit, &read_raw);

  if (r == ESP_OK)
  {
    return read_raw;
  }
  else
  {
    return -1;
  }
}