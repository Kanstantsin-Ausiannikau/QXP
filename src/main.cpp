#include <Arduino.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include "SPI.h"
#include "BluetoothSerial.h"

#define TX_PIN 32
#define Upit 27
#define TXCURRENT_PIN 33

#define CLK 14
#define MISO 12
#define SS 15

#define LED_PIN 22
#define UCOMP_PIN 25

#define CS 4
#define INC 17
#define UD 16

void onTimer();
void RTC_init();
void X9C_init();
void tx_start();
void tx_stop();
void setResistance(int percent);
float getUpitValue();
float getTXCurrent();

float coil_freq = 8200;

float period = 1.0 / 8200;
int64_t halfPeriodTime = (int64_t)(1 / (coil_freq * 2) * 1000000); //in us

int64_t tx_upfront_time;
int64_t rx_upfront_time;

int rx_current_value = 0;
int rx_previous_value = 0;

int64_t rx_current_value_time = 0;
int64_t rx_previous_value_time = 0;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

bool txLow;

int rx_data[100];
int tx_data[100];
int64_t rx_time[100];
int64_t tx_time[100];

int rx_index = 0;

bool write_flag = false;

BluetoothSerial ESP_BT;
int incoming;

void setup()
{

  //Upit ADC setup
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

  //TXCurrent ADC setup
  adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_11);

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
  timerAlarmWrite(timer, 4480 / 2, true);

  tx_start();

  delay(100);
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

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (txLow)
  {
    txLow = false;
    digitalWrite(TX_PIN, HIGH);
    tx_upfront_time = esp_timer_get_time();
  }
  else
  {
    txLow = true;
    digitalWrite(TX_PIN, LOW);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

void loop()
{
  for (int i = 0; i <= 100; i += 1)
  {
    //setResistance(i);
    //Serial.println(i);
    //delay(500);
  }

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

  sleep(1);

  for (int i = 0; i < 100; i++)
  {
    rx_time[i] = esp_timer_get_time();
    rx_data[i] = (txLow) ? 0 : 65536;
    digitalWrite(SS, HIGH);
    usleep(3);
    digitalWrite(SS, LOW);
    tx_data[i] = SPI.transfer16(0x00);
    tx_time[i] = esp_timer_get_time();
  }

  Serial.println((int)(tx_time[99] - tx_time[0]));

  Serial.println("tx___");

  for (int i = 0; i < 100; i++)
  {
    Serial.println(tx_data[i]);
  }

  Serial.println("rx_data___");

  for (int i = 0; i < 100; i++)
  {
    Serial.println(rx_data[i]);
  }

  Serial.println("tx_time___");
  for (int i = 0; i < 100; i++)
  {
    Serial.println((long)tx_time[i]);
  }

  Serial.println("rx_time___");
  for (int i = 0; i < 100; i++)
  {
    Serial.println((long)rx_time[i]);
  }

  printf("Upit - %f\n", getUpitValue());
  printf("TXcurrent - %f\n", getTXCurrent());


  Serial.println("txCurrent___");
  for (int i = 0; i < 100; i++)
  {
    Serial.println(getTXCurrent());
  }
  sleep(10);
  //   int c = analogRead(TXCURRENT_PIN);
}

void tx_start()
{
  txLow = true;
  digitalWrite(TX_PIN, LOW);
  timerAlarmEnable(timer);
}
void tx_stop()

{
  txLow = true;
  digitalWrite(TX_PIN, LOW);
  timerAlarmDisable(timer);
}

float getUpitValue()
{
  int val = adc1_get_raw(ADC1_CHANNEL_5);
  return (val / 4096.0) * 3.3 * (3.3 + 18) / 3.3;
}

float getTXCurrent()
{
  int read_raw;
  esp_err_t r = adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_12Bit, &read_raw);

  
  if (r == ESP_OK)
  {
    return (float)read_raw;
      //return analogRead(TXCURRENT_PIN);
  }
  else
  {
    return -1;
  }
}