#include <Arduino.h>
#include <Arduino.h>
#include "WiFi.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "secrets.h"
#include "splash.h"
#include "Button2.h"
#include "ThingSpeak.h"
//#include "deque"

#define CHANNEL_ID 1759092                 // Chris// Ahmed:1713470                 // Channel ID on Thingspeak
#define CHANNEL_API_KEY "KBZPVDC83RELSMFS" // Chris //Ahmed: "K7G57B13MZEUEGW7" // API Read Key from Thingspeak

WiFiClient client; // this will be used by Thingspeak Library to make HTTP request

int counter = 0; // To know exactly how much Data have been sent

// Variablen für Buttonbefehle
uint8_t Buttoncount1=0; //Test der Buttonfunktionen mit hilfe von Interrupts
uint8_t Buttoncount2=0; 
bool click1=false;
bool click2=false;
bool doubleclick1=false;
bool doubleclick2=false;
uint8_t movcolum=0;
uint8_t movrow=0;


float measurementvalue[18];//Ausgabe der Drehmessung


//Wifi Konfiguration
#define WIFI_NETWORK "Chris1"    // WIFI Name
#define WIFI_PASSWORD "tqor8485" // WIFI Password
#define WIFI_TIMEOUT 20000       // in milliseconds

#define BUTTON_1 0 
#define BUTTON_2 35

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

/*
uint8_t btn1 = 0;
uint8_t btn2 = 35;
*/

/////////////////////////
// The Code Below is From Prof
/////////////////////////
TFT_eSPI tft;
VL53L1X sensor[5];
uint8_t sensorPin[5] = {17, 2, 15, 13, 12};             // XSHUT pins at ESP
uint8_t sensorAddr[5] = {0x51, 0x52, 0x53, 0x54, 0x55}; // new addresses for sensors

#define ADC_EN 14 // ADC_EN is the ADC detection enable port
#define ADC_PIN 34

int vref = 1100;

#define SENSOR_COUNT 5
#define VALUE_COUNT 74

const int Roboter_ID=5;

// deque<int> values[SENSOR_COUNT];


// Initialfunktion Sensoren
void initToFs()
{
    tft.setTextColor(TFT_YELLOW);

    tft.println("Initialisiere Sensoren");
    for (int i = 0; i < 5; i++)
    {
        pinMode(sensorPin[i], OUTPUT);
        digitalWrite(sensorPin[i], LOW); // shut down sensor i
    }

    Serial.println("init ToF Sensors...");

    for (int i = 0; i < 5; i++)
    {
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");

        digitalWrite(sensorPin[i], HIGH); // wake up next sensor
        sensor[i].setTimeout(500);
        if (!sensor[i].init())
        {
            tft.setTextColor(TFT_RED);
            tft.println("Sensor failed!");
            Serial.println("failed!");
        }
        else
        {
            sensor[i].setAddress(sensorAddr[i]); // set new address
            tft.setTextColor(TFT_GREEN);
            tft.println("initialized!");
            Serial.print("initialized, new address, and ");

            // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
            // You can change these settings to adjust the performance of the sensor, but
            // the minimum timing budget is 20 ms for short distance mode and 33 ms for
            // medium and long distance modes. See the VL53L1X datasheet for more
            // information on range and timing limits.
            sensor[i].setDistanceMode(VL53L1X::Long);
            sensor[i].setMeasurementTimingBudget(50000);

            // Start continuous readings at a rate of one measurement every 50 ms (the
            // inter-measurement period). This period should be at least as long as the
            // timing budget.
            sensor[i].startContinuous(50);
            Serial.println("started.");
        }
    }
}
/////////////////////////
// The Code Above is From Prof
/////////////////////////
// Initialfunktion display
void display_init()
{
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);
}
// Initialfunktion Wifi
void connectToWiFi() // Function to connect to wifi
{
    tft.println("Connecting to WiFi");
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT)
    {
        Serial.print(".");
        delay(100);
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(" Failed!");
        // take action
        tft.setTextColor(TFT_RED);
        tft.println("Failed");
    }
    else
    {
        Serial.print("Connected!");
        tft.setTextColor(TFT_GREEN);
        tft.println("Connected!");
        Serial.println(WiFi.localIP());
    }
}
//Button2_ handler
void handler1(Button2 &btn)
{
    switch (btn.getType())
    {
    case single_click:
        
        break;
    case double_click:
        Serial.print("double ");
        break;
    case triple_click:
        Serial.print("triple ");
        break;
    case long_click:
        Serial.print("long");
        break;
    }
    Serial.print("click1");
    Serial.print(" (");
    Serial.print(btn.getNumberOfClicks());
    Serial.println(")");
    click1=true;
}

void handler2(Button2 &btn)
{
    switch (btn.getType())
    {
    case single_click:
        break;
    case double_click:
        Serial.print("double ");
        break;
    case triple_click:
        Serial.print("triple ");
        break;
    case long_click:
        Serial.print("long");
        break;
    }
    Serial.print("click2");
    Serial.print(" (");
    Serial.print(btn.getNumberOfClicks());
    Serial.println(")");
    click2=true;
}

void handler3(Button2 &btn)
{
    switch (btn.getType())
    {
    case single_click:
        break;
    case double_click:
        Serial.print("double ");
        break;
    case triple_click:
        Serial.print("triple ");
        break;
    case long_click:
        Serial.print("long");
        break;
    }
    Serial.print("double_click1");
    Serial.print(" (");
    Serial.print(btn.getNumberOfClicks());
    Serial.println(")");
    doubleclick1=true;
}
void handler4(Button2 &btn)
{
    switch (btn.getType())
    {
    case single_click:
        break;
    case double_click:
        Serial.print("double ");
        break;
    case triple_click:
        Serial.print("triple ");
        break;
    case long_click:
        Serial.print("long");
        break;
    }
    Serial.print("double_click2");
    Serial.print(" (");
    Serial.print(btn.getNumberOfClicks());
    Serial.println(")");
    doubleclick2=true;
}


// Lesen der Sensorwerte
float sensorRead(){
  float saveValue[50];
   for (int n = 0; n < 10; n++)
    {
      

        for (int i = 0; i < 5; i++)
        {
            // Seriele Schnettstelle übertragung
            sensor[i].read();
            Serial.print("Sensor");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(sensor[i].ranging_data.range_mm);
            Serial.print("\t");

            // Display Übertragung der Werte
            /*
            tft.setCursor(0, i * 25 + 75);
            tft.fillRect(0, i * 25 + 75, 125, 10, TFT_BLACK);
            tft.setTextColor(TFT_GREEN);
            tft.print("Sensor ");
            tft.print(i + 1);
            tft.print(" misst:");
            tft.print(sensor[i].ranging_data.range_mm);
            */
            // Speichern von Messdaten zum Mitteln

            saveValue[i + n * 5] = sensor[i].ranging_data.range_mm;
        }

        Serial.println("");
    }

    float average[5]={0,0,0,0,0};

    for (int i = 0; i < 5; i++)
    {
        for (int n = 0; n < 10; n++)
        {
            average[i] += saveValue[i + n * 5];
        }
        average[i] = average[i] / 10;
        /*
        tft.setCursor(0, i * 25 + 85);
        tft.fillRect(0, i * 25 + 85, 125, 10, TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.print("Mittelwert ");
        tft.print(i + 1);
        tft.print(" :");
        tft.print(average[i]);
        */
    }
    return average[3];
}

void soloSensorOutput(){
  float saveValue[50];
   for (int n = 0; n < 10; n++)
    {
      

        for (int i = 0; i < 5; i++)
        {
            // Seriele Schnettstelle übertragung
            sensor[i].read();
            Serial.print("Sensor");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(sensor[i].ranging_data.range_mm);
            Serial.print("\t");

            // Display Übertragung der Werte
            
            tft.setCursor(0, i * 25 + 75);
            tft.fillRect(0, i * 25 + 75, 125, 10, TFT_BLACK);
            tft.setTextColor(TFT_GREEN);
            tft.print("Sensor ");
            tft.print(i + 1);
            tft.print(" misst:");
            tft.print(sensor[i].ranging_data.range_mm);
            
            // Speichern von Messdaten zum Mitteln

            saveValue[i + n * 5] = sensor[i].ranging_data.range_mm;
        }

        Serial.println("");
    }

    float average[5]={0,0,0,0,0};

    for (int i = 0; i < 5; i++)
    {
        for (int n = 0; n < 10; n++)
        {
            average[i] += saveValue[i + n * 5];
        }
        average[i] = average[i] / 10;
        
        tft.setCursor(0, i * 25 + 85);
        tft.fillRect(0, i * 25 + 85, 125, 10, TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.print("Mittelwert ");
        tft.print(i + 1);
        tft.print(" :");
        tft.print(average[i]);
        
    }
    
}


// Funktionen zum Senden von Befehlen 
void Robotturn20(){
    Wire.beginTransmission(Roboter_ID);
    Wire.write('1');
    Wire.endTransmission();
}

void Robotturn90(){
  Wire.beginTransmission(Roboter_ID);
  Wire.write('2');
  Wire.endTransmission();
}
// Variable des Sensors 3 an Roboter senden (die zwei, weil von 0 aus gezählt wird)
void Robotsend3(){
    char value[5];
    Wire.beginTransmission(Roboter_ID);
    Wire.write(sprintf(value, "%04d",sensor[2].ranging_data.range_mm));
    Wire.endTransmission();
}

// Funktion zum Aufsetzen eines Auswahlfeldes 
void movefield(){
  //tft.fillRect(0,0,250,250,TFT_BLACK);
  uint8_t colorChangeColum=0;
  uint8_t colorChangeRow=0;

  for(uint8_t rows=0;rows<4;rows++){
    for(uint8_t colum=0;colum<4;colum++){
      
      tft.drawRect(35*colum,37.5*rows+90,30,30,TFT_GOLD);



      //Felderstellung mit unterschiedlichen Farben
      /*
      colorChangeColum=colum%2;
      colorChangeRow=rows%2;
      if(colorChangeColum==0&&colorChangeRow==0){
        tft.drawRect(35*colum,37.5*rows+90,30,30,TFT_GOLD);

      }
      if(colorChangeColum==1&&colorChangeRow==0){
        tft.drawRect(35*colum,37.5*rows+90,30,30,TFT_BLUE);
      }
      if(colorChangeColum==0&&colorChangeRow==1){
        tft.drawRect(35*colum,37.5*rows+90,30,30,TFT_GREEN);
      }
      if(colorChangeColum==1&&colorChangeRow==1){
        tft.drawRect(35*colum,37.5*rows+90,30,30,TFT_RED);
      }
      */
    }
  }
}
// movfield 
// Bewegliches Feld für Auswahl der Fahrposition entwerfen.
void movecount(uint8_t counts){
  movefield();
  movcolum=counts;
  if(counts>3){
    counts=0;
    movcolum=counts;
    movrow++;
  }
  if(movrow>3){
    movrow=0;
    
  }
  tft.fillRect(35*movcolum,37.5*movrow+90,30,30,TFT_GOLD);
  
  if(movcolum==0){
    tft.fillRect(35*3,37.5*3+90,30,30,TFT_BLACK);
    tft.drawRect(35*3,37.5*3+90,30,30,TFT_GOLD);
  }
  if(movcolum>0&&movcolum<=3){
    tft.fillRect(35*(movcolum-1),37.5*movrow+90,30,30,TFT_BLACK);
    tft.drawRect(35*(movcolum-1),37.5*movrow+90,30,30,TFT_GOLD);
  }

  
}
// Reset beim Verlassen des Feldes 
void movreset(){
  movcolum=0;
  movrow=0;
  Buttoncount2=0;
}



void displayReset(){
  tft.fillRect(0,0,250,250,TFT_BLACK);
}

void measurement(){
  
  uint8_t measurecount=0;
  measurementvalue[measurecount];
  for(measurecount;measurecount<18;measurecount++){
    measurementvalue[measurecount]=sensorRead();
    delay(100);
    Robotturn20();
    delay(100);
    if(measurecount<9){
    tft.setCursor(0,25*(measurecount+1));
    tft.print(measurementvalue[measurecount]);
    }
    if(measurecount>=9){
      tft.setCursor(50,25*(measurecount+1-9));
      tft.print(measurementvalue[measurecount]);

    }
  }

}


void setup() {
 
    // put your setup code here, to run once:
    Serial.begin(115200); // start the serial
    display_init();
    connectToWiFi(); // connect to WIFI

    
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C
    initToFs();
    uint8_t saveValue[50]; // Speicher

  
    while (Wire.available())
    {
        char c = Wire.read();
        Serial.print(c);
    }

    Serial.println("");

    delay(500);

    
  /*
    pinMode(btn1, PULLDOWN);
    pinMode(btn2, PULLDOWN);
*/

  btn1.begin(BUTTON_1,INPUT_PULLUP);
  btn2.begin(BUTTON_2,INPUT_PULLUP);

  btn1.setClickHandler(handler1);
  btn2.setClickHandler(handler2);
  btn1.setDoubleClickHandler(handler3);
  btn2.setDoubleClickHandler(handler4);

  btn1.setDebounceTime(20);
  btn2.setDebounceTime(20);
    ThingSpeak.begin(client); // to initilize the thingspeak library
}

void loop() {
  
  btn1.loop();
  btn2.loop();
  
  if(click1==true){
    Robotturn20();
    Buttoncount1++;
    movreset();
    displayReset();
    tft.setCursor(0,0);
    
    tft.setTextColor(TFT_GREEN);
    tft.println("SoloMeasurement ");
    tft.print("Counts: ");
    tft.print(Buttoncount1);
    soloSensorOutput();
    click1=false;
  }
  if(click2==true){
    Robotturn90();
    displayReset();

    
    movecount(Buttoncount2);
    if(Buttoncount2>3){
      Buttoncount2=0;
    }
    Buttoncount2++;
    tft.setCursor(0,0);
    //tft.fillRect(0,220,125,10,TFT_BLACK);
    tft.setTextColor(TFT_BLUE);
    tft.print("SelectMode, Counts:");
    tft.print(Buttoncount2);
    
    click2=false;
    
  }
  if(doubleclick1==true){
    doubleclick1=false;
    displayReset();
    movreset();
    tft.setCursor(0,0);
    tft.setTextColor(TFT_YELLOW);
    tft.print("Measurmode");
    measurement();
    
  
    tft.print(Buttoncount2);
    
    
    
  }
  if(doubleclick2==true){
    doubleclick2=false;
    displayReset();
    movreset();
    tft.setCursor(0,0);
    tft.setTextColor(TFT_COLMOD);
    tft.print("ShowField");

    movefield();
  }



}