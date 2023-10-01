#include <Arduino.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>


#define ACT_LED 4
#define LCD_LED 5

#define LIN_RX 2
#define LIN_TX 3
#define LIN_EN 4

#define KL15_EN 6
#define BTN_CH0 8
#define BTN_CH1 9
#define BTN_CH2 10

#define LIN_SUP_SENS 5
#define UBAT_SUP_SENS 4
#define LIN_SUP_SENS_FACTOR 11
#define UBAT_SUP_SENS_FACTOR 11


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
unsigned int LinSupSensAdcRaw = 0u;
unsigned int UbatSupSensAdcRaw = 0u;
float LinSupSensAdcScaled = 0;
float UbatSupSensAdcScaled = 0;
unsigned long TimeStamp = 0;
unsigned int incomingByte = 0u;
unsigned char ShowFlag = true;


//BUTTON Cyclick Task init
int WhichScreen = 0;                       // This variable stores the current Screen number
boolean flag = true;                       // Check if the button is pressed
boolean flagPlus = true;
boolean flagMinus = true;
const int buttonPinSelect = BTN_CH0;             // the number of the pushbuttonSelect pin
const int buttonPinPlus = BTN_CH1;               // the number of the pushbuttonPlus pin
const int buttonPinMinus = BTN_CH2;             // the number of the pushbuttonMinus pin
int buttonStateSelect;                     // the current reading from the input pin
int lastButtonStateSelect = HIGH;           // the previous reading from the input pin
int buttonStatePlus;                       // the current reading from the input pin
int lastButtonStatePlus = HIGH;
int buttonStateMinus;                       // the current reading from the input pin
int lastButtonStateMinus = HIGH;
unsigned long lastDebounceTime = 0;        // the last time the output pin was toggled
unsigned long debounceDelay = 5;           // the debounce time; increase if the output flickers
unsigned long lastDebounceTimePlus = 0;        // the last time the output pin was toggled
unsigned long debounceDelayPlus = 5;           // the debounce time; increase if the output flickers
unsigned long lastDebounceTimeMinus = 0;        // the last time the output pin was toggled
unsigned long debounceDelayMinus = 5;

//LIN Serial init
SoftwareSerial LinSerial(2,3);

//I2C LCD Initialisation
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
unsigned char LcdRefreshTimer = 0;
unsigned char LcdRefreshTimerDelay = 10;
int temperatureDisplay = 0;
unsigned char diagRefreshTimerDelay = 50;
unsigned char diagRefreshTimer = 0;

void setup()
{
  //init functions
  Serial.begin(9600);
  LinSerial.begin(9600);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, HIGH);
  pinMode(LCD_LED, OUTPUT);
  digitalWrite(LCD_LED, HIGH);

  pinMode(buttonPinSelect, INPUT_PULLUP); // Read input from button selectz
  pinMode(buttonPinPlus, INPUT_PULLUP); // Read input from button Plus
  pinMode(buttonPinMinus, INPUT_PULLUP); // Read input from button Minus

  // Init the LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("INITIALISATION...");

  //Print the debug menu interface welcome message
  Serial.print("\n -------------------------------------------");
  Serial.print("\n|       Welcome to the Free Masonry");
  Serial.print("\n -------------------------------------------");
}

void loop()
{
  if(millis() - CyclickTask_counter10ms >= Counter10msDelayTime)
  {
    CyclickTask_counter10ms = millis();
    ActLedCyclic();
    AdcReadCyclic();
    IoDriverCyclic();
    Lcd_CyclickTaks();
    ButtonSelect();
    ButtonPlus();
    ButtonMinus();
    //Diag_CyclickTask();
    ReadInputSerialCyclick();
    // Put here your Cyclick task on Counter10msDelayTime
  }
}

void ActLedCyclic()
{
 if(CounterStatLed >= CounterStatLedRef)
  {
    CounterStatLed = 0u;
    digitalWrite(LCD_LED, !digitalRead(LCD_LED));
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
    LinSupSensAdcRaw = (analogRead(LIN_SUP_SENS)*5.015/1024);
    LinSupSensAdcScaled = (LinSupSensAdcRaw*(LIN_SUP_SENS_FACTOR));

    UbatSupSensAdcRaw = (analogRead(UBAT_SUP_SENS)*5.015/1024);
    UbatSupSensAdcScaled = (UbatSupSensAdcRaw*(UBAT_SUP_SENS_FACTOR));
}


void IoDriverCyclic()
{
//IO Control
}

// LCD function---------------------------------------------
 
void Lcd_CyclickTaks()
{
  LcdRefreshTimer++;
  if(LcdRefreshTimer>LcdRefreshTimerDelay)
  {
    LcdRefreshTimer=0;
     if(true == flag)
     {
       //WhichScreen++;
       flag = false;
     }
  switch(WhichScreen)
  {
    case 0:          // ENTER HOMESCREEN
    {
      
      if( true == ShowFlag)
      {
        ShowFlag = false;
        firstScreen();
      }

      if(true == flagMinus)        // ButtonMinus();
      {
       WhichScreen = 1;
       flagMinus = false;  
       ShowFlag = true;
      }

    }
    break;
   
    case 1:          // TEST STEP 1 - KM
    {
      
      if( true == ShowFlag)
      {
        ShowFlag = false;
        LinSendSpeed10Km();
        secondScreen();
      }
      
      if(true == flagMinus)        // ButtonMinus();
     {
      WhichScreen = 2; // TEST OK - go to next test
      flagMinus = false;  
      ShowFlag = true;
     }
     else if(true == flagPlus)    // ButtonPlus();
     {
      WhichScreen = 3; // TEST NOK - go to FAIL STATE
      flagPlus = false;
      ShowFlag = true;
     }    
    }
    break;
   
    case 2:          // ENTER VOLTAGE MENU        
    {
      if( true == ShowFlag)
      {
        ShowFlag = false;
      LinSendSpeed45Km();
      thirdScreen();
      }
 
      if(true == flagMinus)        // ButtonMinus();
      {
       WhichScreen = 2; // TEST OK - go to next test
       flagMinus = false;
       ShowFlag = true;  
      }
      else if(true == flagPlus)    // ButtonPlus();
      {
       WhichScreen = 3; // TEST NOK - go to FAIL STATE
       flagPlus = false;
       ShowFlag = true;
      }
    }
    break;
   
    case 3:          // FAIL MENU
    {
      if( true == ShowFlag)
      {
       ShowFlag = false;
       FailScreen();
      }

       if(true == flagMinus)        // ButtonMinus();
       { 
        WhichScreen = 0; // Main
        flagMinus = false;
        ShowFlag = true;  
       }
    }
    break;  
   
  }
 
 }
}

// Button Minus function ---------------------------------------------
void ButtonMinus()
{

    int readingMinus = digitalRead(buttonPinMinus);
    if (readingMinus != lastButtonStateMinus)
    {
      // reset the debouncing timer
      lastDebounceTimeMinus = debounceDelayMinus;
    }

   if ((lastDebounceTimeMinus--) <= 0)
   {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingMinus != buttonStateMinus)
    {
      buttonStateMinus = readingMinus;

      // only toggle the LED if the new button state is HIGH
      if (buttonStateMinus == LOW)
      {
        flagMinus = true;              
      }
      else
      {
        flagMinus = false;
      }
    }
    else
    {
      flagMinus = false;
    }
   }
 
   lastButtonStateMinus = readingMinus;
}

//Button Select function ------------------------------------------
void ButtonSelect()
{
    // BEGIN of the switch debouncing code
    int readingSelect = digitalRead(buttonPinSelect);

   
    if (readingSelect != lastButtonStateSelect)
    {
      // reset the debouncing timer
      lastDebounceTime = debounceDelay;
    }

   if ((lastDebounceTime--) <= 0)
   {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingSelect != buttonStateSelect)
    {
      buttonStateSelect = readingSelect;

      // only toggle the LED if the new button state is HIGH
      if (buttonStateSelect == LOW)
      {
        flag = true;      
      }
      else
      {
        flag = false;
      }
    }
    else
    {
      flag = false;
    }
   }
   
   lastButtonStateSelect = readingSelect;
}

// Button Plus function ---------------------------------------------
void ButtonPlus()
{
    int readingPlus = digitalRead(buttonPinPlus);
    if (readingPlus != lastButtonStatePlus)
    {
      // reset the debouncing timer
      lastDebounceTimePlus = debounceDelayPlus;
    }  

   if ((lastDebounceTimePlus--) <= 0)
   {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingPlus != buttonStatePlus)
    {
      buttonStatePlus = readingPlus;

      // only toggle the LED if the new button state is HIGH
      if (buttonStatePlus == LOW)
      {
        flagPlus = true;
           
      }
      else
      {
        flagPlus = false;
      }
    }
    else
    {
      flagPlus = false;
    }
   }
   
     lastButtonStatePlus = readingPlus;
}


//=======================================================
//Homescreen LCD refresh

void firstScreen()
  {
   lcd.clear();
   lcd.setCursor(4,0); // Column, line
   lcd.print("TEST START?");
   Serial.print("\nTEST START?");
   lcd.setCursor(8,1);
   lcd.print("Y/N");
   Serial.print("\nY/N");
  }

void secondScreen()
  {
   lcd.clear();
   lcd.setCursor(0,0); // Column, line
   lcd.print("KM TEST - 10KM?");
   Serial.print("\nKM TEST - 10KM?");
   lcd.setCursor(0,1);
   lcd.print("Y/N");
   Serial.print("\nY/N");
  }

void thirdScreen()
  {
   lcd.clear();
   lcd.setCursor(0,0); // Column, line
   lcd.print("KM TEST - 45KM?");
   Serial.print("\nKM TEST - 45KM?");
   lcd.setCursor(0,1);
   lcd.print("Y/N");
   Serial.print("\nY/N");
  }

  //Calibration LCD refresh
void fourthScreen()
  {
   lcd.clear();
   lcd.setCursor(0,0); // Column, line
   lcd.print("Calibration");
   lcd.setCursor(0,1);
   //lcd.print(calibration_const);
   lcd.setCursor(5,1);
   lcd.print("RPM = ");
   //lcd.print(rpm);
  }

//Voltage LCD refresh
void FifthScreen()
  {
   lcd.clear();
   lcd.setCursor(0,0); // Column, line
   lcd.print("Voltage Sensor");
   lcd.setCursor(0,1);
   lcd.print("Voltage = ");
   lcd.print(UbatSupSensAdcScaled );
   lcd.print(" V" );
  }

  //Voltage LCD refresh
void FailScreen()
  {
   lcd.clear();
   lcd.setCursor(0,0); // Column, line
   lcd.print("FAIL!");
   Serial.print("\nFAIL!");
   lcd.setCursor(0,1);
   lcd.print("GO TO MAIN? - Y/N");
   Serial.print("\nGO TO MAIN? - Y/N");
  }

 
// DIAGNOSTIC REALTIME FUNCTION-------------------------------------------------

void Diag_CyclickTask()
{
  diagRefreshTimer++;
  if(diagRefreshTimer>diagRefreshTimerDelay)
  {
    diagRefreshTimer=0;
 
    Serial.print("\n -------------------------------------------");
    Serial.print("\n|       DEBUG INFO                         |");
    Serial.print("\n -------------------------------------------");
    Serial.print("\n LIN SUP: ");
    Serial.print(LinSupSensAdcScaled);
    Serial.print(" V");
    Serial.print("\n LIN SUP RAW: ");
    Serial.print(LinSupSensAdcRaw);
    Serial.print("\n Ubat: ");
    Serial.print(UbatSupSensAdcScaled);
    Serial.print(" V");
    Serial.print("\n Ubat RAW: ");
    Serial.print(UbatSupSensAdcRaw);
  
    //Print the time stamp::
    Serial.print("Time: ");
    Serial.print(TimeStamp);
    Serial.println(" ms");

    //Print the State value:
    Serial.print("State value = ");
    Serial.println(WhichScreen);

    //Print Btn Select state:
    Serial.print("Select btn state = ");
    Serial.println(flag);

    //Print Btn plus state:
    Serial.print("Select btn plus = ");
    Serial.println(flagPlus);

   
    //Print Btn plus state:
    Serial.print("Select btn plus = ");
    Serial.println(flagMinus);
   
    //Print the State 1:
    if (WhichScreen == 0)
    {
        Serial.println("MainScreen:");
        Serial.print("RPM = ");
        //Serial.println(rpm);      
    }

    //Print the State 2:
    if (WhichScreen == 1)
    {
        Serial.println("Temp Sensor");
        Serial.println("Menu");      
    }

    //Print the State 3:
    if (WhichScreen == 2)
    {
        Serial.println("Voltage Sensor");
        Serial.println("Menu");      
    }

    //Print the State 4:
    if (WhichScreen == 3)
    {
        Serial.println("Calibration");
        Serial.println("Menu");      
    }
   
  }     
}

void LinSendSpeed10Km()
{
  //Send 10 km comand:
  LinSerial.print("10km");
}

void LinSendSpeed45Km ()
{
  //Send 45 km comand:
  LinSerial.print("10km");
}

void LinSendLightsOn ()
{
  //Send comand Lights ON:
  LinSerial.print("LightsON");
}

void ReadInputSerialCyclick()
{
    if (Serial.available() > 0) 
    {
     // read the incoming byte:
     incomingByte = Serial.read();
     if('y' == incomingByte)
     {
       flagMinus = true;
     }
     else if ('n' == incomingByte)
     {
       flagPlus = true;
     }
    }
}
