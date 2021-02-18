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
int measure_mid_point_voltage();

float coil_freq = 8200;

float period = 1.0/8200;
int64_t halfPeriodTime = (int64_t)(1 / (coil_freq * 2) * 1000000); //in us

int mid_voltage = 0;

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

  X9C_init();

  delayMicroseconds(1000);


  Serial.begin(115200); // Запускаем последовательный монитор со скоростью 9600
  ESP_BT.begin("QXP detector"); // Задаем имя вашего устройства Bluetooth
  Serial.println("QXP is Ready to Pair");  // По готовности сообщаем, что устройство готово к сопряжению
 
  //Serial.begin(115200);
  Serial.println("Booting");
  Serial.println(coil_freq);


  pinMode(TX_PIN, OUTPUT);


  //pinMode(UCOMP_PIN, OUTPUT);
  //(UCOMP_PIN, LOW);

  pinMode(TX_PIN, OUTPUT);

  //pinMode(POWER_LEVEL_PIN, INPUT);
  pinMode(TXCURRENT_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  dac_output_enable(DAC_CHANNEL_1); //компенсация - 2.5 вольта для средней точки
  dac_output_voltage(DAC_CHANNEL_1, 200);

  RTC_init();

  timer = timerBegin(0, 1, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 4480 / 2, true);

  mid_voltage = measure_mid_point_voltage();
  tx_start();

  delay(100);
  
}

void X9C_init()
{
  pinMode(CS, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(UD, OUTPUT);
  digitalWrite(CS, HIGH);  // X9C в режиме низкого потребления
  digitalWrite(INC, HIGH); 
  digitalWrite(UD, HIGH);

   digitalWrite(UD, LOW); // выбираем понижение
   digitalWrite(CS, LOW); // выбираем потенциометр X9C
   for (int i=0; i<100; i++) { // т.к. потенциометр имеет 100 доступных позиций
     digitalWrite(INC, LOW);
     delayMicroseconds(1);
     digitalWrite(INC, HIGH);
     delayMicroseconds(1);
   }

     digitalWrite(UD, HIGH);
     for (int i=0; i<90; i++) {
       digitalWrite(INC, LOW);
       delayMicroseconds(1);
       digitalWrite(INC, HIGH);
       delayMicroseconds(1);
    }

   digitalWrite(CS, HIGH);

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

  // if (write_flag)
  // {
  //   rx_data[rx_index] = (txLow)?0:100;
  //   rx_time[rx_index] = esp_timer_get_time();
  //   rx_index++;
  //   if (rx_index>99)
  //   {
  //     rx_index=0;
  //   }

  // }

  portEXIT_CRITICAL_ISR(&timerMux);
}

void loop()
{

  if (ESP_BT.available()) // Проверяем, не получили ли мы что-либо от Bluetooth модуля
  {
    incoming = ESP_BT.read(); // Читаем, что получили
    Serial.print("Received:"); Serial.println(incoming);
 
    if (incoming == 49)  // Если значение равно единице, включаем светодиод
        {
        //digitalWrite(LED_BUILTIN, HIGH);
        ESP_BT.println("LED turned ON");
        }
 
    if (incoming == 48)  // Если значение равно нулю, выключаем светодиод
        {
        //digitalWrite(LED_BUILTIN, LOW);
        ESP_BT.println("LED turned OFF");
        }     
  }


  
  // rx_previous_value_time = esp_timer_get_time();
  // rx_previous_value = SPI.transfer16(0);
  // rx_current_value_time = esp_timer_get_time();
  // rx_current_value = SPI.transfer16(0);

  // while (rx_previous_value < mid_voltage && rx_current_value > mid_voltage)
  // {
  //   rx_previous_value = rx_current_value;
  //   rx_previous_value_time = rx_current_value_time;
  //   rx_current_value_time = esp_timer_get_time();
  //   rx_current_value = SPI.transfer16(0);
  // }

  // int time_interval = (int)(rx_previous_value_time + (rx_current_value_time - rx_previous_value_time) / 2 - tx_upfront_time);

  // Serial.println(time_interval);

  //Serial.println((long)rx_previous_value_time);
  //Serial.println((long)rx_previous_value_time);

  // sleep(2);

  // uint data[100];

  //int64_t t = esp_timer_get_time();

//tx_stop();

//-------------------------------------------------------------------------------------------------------------------------------------------------
  sleep(1);

   for (int i = 0; i < 100; i++)
   {
     rx_time[i] = esp_timer_get_time();
     rx_data[i] = (txLow)?0:65000;
    digitalWrite(SS, HIGH);
    usleep(3);
    digitalWrite(SS, LOW);
     tx_data[i] = SPI.transfer16(0x00);
     tx_time[i] = esp_timer_get_time();
  }


   Serial.println((int)(tx_time[99]-tx_time[0]));

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

    // int read_raw[100];
    // adc2_config_channel_atten( ADC2_CHANNEL_9, ADC_ATTEN_0db );

    // for(int i=0;i<100;i++)
    // {
    //     esp_err_t r = adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_12Bit, &read_raw[i]);
    // }

    // int read_raw[100];
    // adc2_config_channel_atten( ADC2_CHANNEL_7, ADC_ATTEN_0db );

    // for(int i=0;i<100;i++)
    // {
    //     esp_err_t r = adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_12Bit, &read_raw[i]);
    // }

  //   Serial.println("Upit");
  //   for (int i = 0; i < 100; i++)
  //  {
  //    Serial.println(read_raw[i]);
  //  }


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

    int read_raw;
    adc2_config_channel_atten( ADC2_CHANNEL_9, ADC_ATTEN_11db );

     esp_err_t r = adc2_get_raw( ADC2_CHANNEL_9, ADC_WIDTH_12Bit, &read_raw);
     if ( r == ESP_OK ) {
         printf("Current - %d\n", read_raw );
     } else if ( r == ESP_ERR_TIMEOUT ) {
         printf("ADC2 used by Wi-Fi.\n");
     }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_11);
    int val = adc1_get_raw(ADC1_CHANNEL_5);
    printf("Upit - %d\n", val);
  
   sleep(10);

//-------------------------------------------------------------------------------------------------------------------------------------------------

  // t = t - esp_timer_get_time();

  // Serial.println((long)t);
  // Serial.println("___");

  // for (int i = 0; i < 100; i++)
  // {
  //   Serial.println(data[i]);
  // }

  // Serial.print("Mid point - ");
  // tx_stop();
  // int mid_voltage = measure_mid_point_voltage();
  // Serial.println(mid_voltage);
  // tx_start();

  //delay(100);

  // digitalWrite(SS, LOW);
  // //delayMicroseconds(4);
  // int sample = SPI.transfer16(0x00);
  // digitalWrite(SS,HIGH);
  // Serial.println(sample);
  // delayMicroseconds(100000);

  // Serial.println("___");
  // Serial.println(millis());
  // int current = 0;
  // for(int i=0;i<1000;i++)
  // {
  //   int c = analogRead(TXCURRENT_PIN);
  //   if (current<c)
  //   {
  //     current = c;
  //   }
  // }
  // Serial.println(current);

  // Serial.println(millis());

  // sleep(1);

  // digitalWrite(TX_PIN, HIGH);
  // delayMicroseconds(periodTime);

  // digitalWrite(TX_PIN, LOW);
  // delayMicroseconds(periodTime);

  // put your main code here, to run repeatedly:

  //  long t = esp_timer_get_time();

  //  long data[100];

  //  for (int i = 0; i < 100; i++)
  //  {
  //   data[i] = analogRead(TXCURRENT_PIN);
  //  }

  //  t = t - esp_timer_get_time();
  //  Serial.println(t);
  //  Serial.println("___");

  //  for (int i = 0; i < 100; i++)
  //  {
  //    Serial.println(data[i]);
  //  }

  //  sleep(2);
  

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

int measure_mid_point_voltage()
{
  int s = 0;
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(SS, HIGH);
    usleep(3);
    digitalWrite(SS, LOW);
    //s = s + SPI.transfer16(0x00);
  }

  return (int)(s / 100);
}
