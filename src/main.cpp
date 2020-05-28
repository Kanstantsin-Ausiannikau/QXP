#include <Arduino.h>
#include <driver/dac.h>
//#include <SPI.h>

#define TX_PIN 32
#define POWER_LEVEL_PIN 12
#define TXCURRENT_PIN 33
#define CLK 14
#define MISO 12
#define MOSI 13
#define SS 15
#define LED_PIN 22

void onTimer();
//void RTC_init();

float coil_freq = 8900;

uint32_t halfPeriodTime = (uint32_t)(1 / (coil_freq*25*2) * 1000000*1000); //in steps

uint64_t sys_timer;
uint64_t current_period_timer;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

bool txLow = true;

void setup()
{

  delayMicroseconds(1000);

  sys_timer = 0LL;
  pinMode(CLK, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(MISO, INPUT);




  Serial.begin(115200);
  Serial.println("Booting");
  Serial.println("Period time - " + halfPeriodTime);

  pinMode(TX_PIN, OUTPUT);

  pinMode(POWER_LEVEL_PIN, INPUT);
  pinMode(TXCURRENT_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 100);

  //RTC_init();

  current_period_timer = sys_timer+halfPeriodTime;
  digitalWrite(TX_PIN, LOW);
  timer = timerBegin(0, 1, true);
  timerAttachInterrupt(timer, &onTimer, true);
  //timerAlarmWrite(timer, 4480/2, true);
  timerAlarmWrite(timer, 1, true);
  timerAlarmEnable(timer);

  //480
}

// void RTC_init()
// {
//   pinMode(SS, OUTPUT);
//   pinMode(MISO, INPUT);
//   pinMode(MOSI, OUTPUT);
//   pinMode(CLK, OUTPUT);

  
//   SPI.begin(CLK, MISO, MOSI, SS);
//   SPI.setBitOrder(MSBFIRST);
//   SPI.setDataMode(SPI_MODE3);
//   SPI.setFrequency(20000000L);
// }

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  // if (txLow)
  // {
  //   txLow = false;
  //   digitalWrite(TX_PIN, HIGH);
  // }
  // else 
  // {
  //   txLow = true;
  //   digitalWrite(TX_PIN, LOW);
  // }

  if (current_period_timer<sys_timer&&txLow==true)
  {
    txLow = false;
    digitalWrite(TX_PIN, HIGH);
    current_period_timer = sys_timer+halfPeriodTime;
  }
  if (current_period_timer<sys_timer&&txLow==false)
  {
    txLow = true;
    digitalWrite(TX_PIN, LOW);
    current_period_timer = sys_timer+halfPeriodTime;    
  }

  sys_timer++;

  //one step of sys_timer - 25ns

  portEXIT_CRITICAL_ISR(&timerMux);
}

void loop()
{

  // sleep(2);

  // uint data[100];

  // long t = esp_timer_get_time();

  // for (int i=0;i<100;i++)
  // {
  // digitalWrite(SS, HIGH);
  // usleep(3);
  // digitalWrite(SS,LOW);
  // data[i]=SPI.transfer16(0x00);
  // }

  // t = t - esp_timer_get_time();

  // Serial.println(t);
  // Serial.println("___");

  //   for (int i=0;i<100;i++)
  //   {
  //     Serial.println(data[i]);
  //   }

  // sleep(20);

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
