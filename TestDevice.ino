#include <Arduino.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <InterpolationLib.h>

#define ACT_LED 4
#define STAT_LED 13
#define PWM_FAN_CTRL 6
#define CH1_PWM_CTRL 9
#define CH2_PWM_CTRL 11

#define NTC_CH1 0
#define NTC_CH2 1

#define DUTY_100_PERC 255
#define DUTY_50_PERC 127
#define DUTY_0_PERC 0

//Offsets defines for the analog channels for NTC.
//Real Arduino: 212.008281 
//China Arduino: 204.391217
#define NTC_CH1_OFFSET 212.008281
#define NTC_CH2_OFFSET 212.008281

boolean IdleState = true;
boolean IdleFlag = false;
unsigned long CyclickTask_counter10ms = 0u;
const long Counter10msDelayTime = 10u;
unsigned int CounterActLed = 0u;
unsigned int CounterStatLed = 0u;
unsigned int CounterDebugInfo = 0u;
const int CounterActLedRef = 100u;
const int CounterStatLedRef = 50u;
const int CounterDebugInfoRef = 100u;

//NTC Raw Values from the ADC
unsigned int NtcCh1AdcRaw = 0u;
unsigned int NtcCh2AdcRaw = 0u;

unsigned int tempout_NTC_1 = 0u;
unsigned int tempout_NTC_2 = 0u;

//Control values for the PWM Out
unsigned int PwmOutFan = 0u;
unsigned int PwmOutCh1 = 0u;
unsigned int PwmOutCh2 = 0u;

//PWM Channels PID configuration
#define NUM_CHANNELS 2
const int CoeffProp = 2;
const float CoeffIntegral = 0.01;
const int CoeffDerivat = 5;
const int deltaTime = 10;
struct ChannelData 
{
  unsigned char ChId;
  unsigned int tempAdcRawValue;
  unsigned int tempValue;
  float error;
  float integral;
  float derivative;
  float lastError;
  int heatingPower;
  unsigned char pwmOutPin;
  unsigned char setPoint; //values from 0 to 255
};

//Interpolation points:
  const unsigned char numValues = 13;
  double xValues_Voltage[13] = { 0.11644697, 0.221562458, 
                                 0.401090967, 0.685617124,
                                 1.097381647,1.62022035,
                                 2.204293965, 2.786757329,
                                 3.316969617, 3.74925015, 
                                 4.085968783, 4.336513443, 
                                 4.517528009 }; // Points from the NTC curve
                                 
  double yValues_Celsius[13] = {-40, -30,
                                -20, -10,
                                  0, 10,
                                 20, 30, 
                                 40, 50, 
                                 60, 70, 
                                 80 };//List for temperature range  
                                 
//Channels data:                               
ChannelData channels[NUM_CHANNELS];

//Variables for the HC06 handling:
String cmd="";
String cmd_last="";

SoftwareSerial hc06(2,3);
void setup()
{
  //init functions
  
  Serial.begin(9600);
  hc06.begin(9600);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, HIGH);
  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, HIGH);

 //Init the PWM outputs:
  pinMode(PWM_FAN_CTRL, OUTPUT);
  analogWrite(PWM_FAN_CTRL, DUTY_100_PERC);
  pinMode(CH1_PWM_CTRL, OUTPUT);
  analogWrite(CH1_PWM_CTRL, DUTY_100_PERC);
  pinMode(CH2_PWM_CTRL, OUTPUT);
  analogWrite(CH2_PWM_CTRL, DUTY_100_PERC);

 for (int i = 0; i < NUM_CHANNELS; i++) 
 {
    ChannelData& channel = channels[i];
    if (0u == i)
    {
      channel.ChId = 1u;
      channel.tempValue = 0u;
      channel.error = 0u;
      channel.integral = 0u;
      channel.derivative =0u;
      channel.lastError = 0u;
      channel.heatingPower = 0u;
      channel.pwmOutPin = CH1_PWM_CTRL;
      channel.setPoint = 125u; //values from 0 to 255
    }
    if (1u == i)
    {
      channel.ChId = 2u;
      channel.tempValue = 0u;
      channel.error = 0u;
      channel.integral = 0u;
      channel.derivative =0u;
      channel.lastError = 0u;
      channel.heatingPower = 0u;
      channel.pwmOutPin = CH2_PWM_CTRL;
      channel.setPoint = 125u; //values from 0 to 255  
    }

 }

  //Print the debug menu interface welcome message
  Serial.print("\n -------------------------------------------");
  Serial.print("\n|       Welcome to the Free Masonry|");
  Serial.print("\n -------------------------------------------");
}

void loop()
{
  if(millis() - CyclickTask_counter10ms >= Counter10msDelayTime)
  {
    CyclickTask_counter10ms = millis();
    ActLedCyclic();
    AdcReadCyclic();
    NtcDataCyclic();
    PidFuncCyclic();
    PwmDataCyclic();
    DebugDataCyclic();
    ComInfoSendCyclic();
    IoDriverCyclic();
    // Put here your Cyclick task on Counter10msDelayTime
  }
}

void ActLedCyclic()
{
 if(CounterStatLed >= CounterStatLedRef)
  {
    CounterStatLed = 0u;
    digitalWrite(STAT_LED, !digitalRead(STAT_LED));
  }
  else
  {
    CounterStatLed++;
  }

   if(CounterActLed >= CounterActLedRef)
  {
    CounterActLed = 0u;
    digitalWrite(ACT_LED, !digitalRead(ACT_LED));
  }
  else
  {
    CounterActLed++;
  }
}

void AdcReadCyclic()
{
    NtcCh1AdcRaw = analogRead(NTC_CH1);
    NtcCh1AdcRaw = (NtcCh1AdcRaw/(NTC_CH1_OFFSET));

    NtcCh2AdcRaw = analogRead(NTC_CH2);
    NtcCh2AdcRaw = (NtcCh2AdcRaw/(NTC_CH2_OFFSET));

  for (int i = 0; i < NUM_CHANNELS; i++) 
     {
      ChannelData& channel = channels[i];
      if (0u == i)
      {
       channel.tempAdcRawValue = NtcCh1AdcRaw;
      }
      if (1u == i)
      {
       channel.tempAdcRawValue = NtcCh2AdcRaw;
      }
     }
}

void NtcDataCyclic()
{
   //Interpolation
   tempout_NTC_1 = Interpolation::Linear(xValues_Voltage, yValues_Celsius, numValues, NtcCh1AdcRaw, true);
   tempout_NTC_2 = Interpolation::Linear(xValues_Voltage, yValues_Celsius, numValues, NtcCh2AdcRaw, true);
   

     for (int i = 0; i < NUM_CHANNELS; i++) 
     {
      ChannelData& channel = channels[i];
      if (0u == i)
      {
       channel.tempAdcRawValue = 
       channel.tempValue = tempout_NTC_1;
      }
      if (1u == i)
      {
       channel.tempValue = tempout_NTC_2;
      }
     }
}

void PidFuncCyclic()
{
//PID Regulator
 for (int i = 0; i < NUM_CHANNELS; i++) 
 {
    ChannelData& channel = channels[i];
    channel.error = channel.setPoint - channel.tempValue;
    channel.integral = (channel.integral += (channel.error * deltaTime)); //deltaTime is10ms - calculating the integral in one row
    channel.derivative = ((channel.error - channel.lastError) / deltaTime); //deltaTime is 10ms - calculating the derivat in onee row
    channel.heatingPower = CoeffProp * channel.error + CoeffIntegral * channel.integral + CoeffDerivat * channel.derivative;
    channel.lastError = channel.error;
 }
}

void PwmDataCyclic()
{
//PWM Fan Update value
PwmOutFan = NtcCh1AdcRaw/4;

//Set the Heater PWM
for (int i = 0; i < NUM_CHANNELS; i++) 
 {
    ChannelData& channel = channels[i];
    analogWrite(channel.pwmOutPin, channel.heatingPower);
 }

//Set the Fan speed;
analogWrite(PWM_FAN_CTRL, PwmOutFan);
}

void DebugDataCyclic()
{
//DebugDataCyclic

 if(CounterDebugInfo >= CounterDebugInfoRef)
  {
    CounterDebugInfo = 0u;
    Serial.print("\n -------------------------------------------");
    Serial.print("\n|       DEBUG INFO                         |");
    Serial.print("\n -------------------------------------------");
    Serial.print("\n BLUETOOTH RECEIVED INFO:");
    Serial.println(cmd);
    
   /* Serial.print("\n NTC_CH1:");
    Serial.print(NtcCh1AdcRaw);
    Serial.print("\n NTC_CH2:");
    Serial.print(NtcCh2AdcRaw);
    Serial.print("\n PWM OUT FAN:");
    Serial.print(PwmOutFan);
    Serial.print("\n PWM OUT CH1:");
    Serial.print(PwmOutCh1);
    Serial.print("\n PWM OUT CH2:");
    Serial.print(PwmOutCh2); */

    
   for (int i = 0; i < NUM_CHANNELS; i++) 
   {
      ChannelData& channel = channels[i];
        Serial.print("\n -------------------------------------------");
        Serial.print("\n CHANNEL ID:");
        Serial.print(channel.ChId);
        Serial.print("\n TEMP RAW VALUE [V]:");
        Serial.print(channel.tempAdcRawValue);
        Serial.print("\n TEMP VALUE [DEG. C]:");
        Serial.print(channel.tempValue);
        Serial.print("\n ERROR:");
        Serial.print(channel.error);
        Serial.print("\n INTEGRAL:");
        Serial.print(channel.integral);
        Serial.print("\n DERIVATIVE:");
        Serial.print(channel.derivative);
        Serial.print("\n LAST ERROR:");
        Serial.print(channel.lastError);
        Serial.print("\n HEATING POWER [W]:");
        Serial.print(channel.heatingPower);
        Serial.print("\n PWM CONTROL PIN:");
        Serial.print(channel.pwmOutPin);
        Serial.print("\n SET POINT [DEG C]:");
        Serial.print(channel.setPoint);
        Serial.print("\n -------------------------------------------");   
   }
   
   
  }
  else
  {
    CounterDebugInfo++;
  }
}

void ComInfoSendCyclic()
{
//Bluetooth
   while(hc06.available()>0)
   {
    cmd+=(char)hc06.read();
   }  
}

void IoDriverCyclic()
{
//IO Control
}
