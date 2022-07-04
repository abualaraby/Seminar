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

#define CHANNEL_ID 1713470                 // Chris: 1759092 // Ahmed:1713470                 // Channel ID on Thingspeak
#define CHANNEL_API_KEY "K7G57B13MZEUEGW7" // Chris: KBZPVDC83RELSMFS // Ahmed: "K7G57B13MZEUEGW7" // API Read Key from Thingspeak

WiFiClient client; // this will be used by Thingspeak Library to make HTTP request

int counter = 0; // To know exactly how much Data have been sent

// Variablen für Buttonbefehle
uint8_t Buttoncount1 = 0; // Test der Buttonfunktionen mit hilfe von Interrupts
uint8_t Buttoncount2 = 0;
bool click1 = false;
bool click2 = false;
bool doubleclick1 = false;
bool doubleclick2 = false;
bool longclick1 = false;
bool longclick2 = false;

uint8_t menu = 0;

uint8_t movcolum = 0;
uint8_t movrow = 0;

int position = 0;             // Variable zum Definieren der Position
float measurementvalue15[24]; // Ausgabe der Drehmessung
float measurementvalue5[72];

float xValues5[72];
float yValues5[72];
float xValues15[24];
float yValues15[24];

float longSideX = 0;
float longSideY = 0;
float shortSideX = 0;
float shortSideY = 0;

// Wifi Konfiguration
#define WIFI_NETWORK "Drei"        // WIFI Name
#define WIFI_PASSWORD "Howyoudoin" // WIFI Password
#define WIFI_TIMEOUT 10000         // 20000       // in milliseconds

#define BUTTON_1 0
#define BUTTON_2 35

Button2 btn1(BUTTON_1); // Linke Taste
Button2 btn2(BUTTON_2); // Rechte Taste

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

const int Roboter_ID = 5;

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
      sensor[i].setDistanceMode(VL53L1X::Short);
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
// Button2_ handler
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
  click1 = true;
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
  click2 = true;
}

void doubleClickHandler1(Button2 &btn)
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
  doubleclick1 = true;
}
void doubleClickHandler2(Button2 &btn)
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
  doubleclick2 = true;
}

void longClickHandler1(Button2 &btn)
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
  longclick1 = true;
}

void longClickHandler2(Button2 &btn)
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
  longclick2 = true;
}

// Lesen der Sensorwerte
float sensorRead()
{
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

  float average[5] = {0, 0, 0, 0, 0};

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

  return average[2];
}

float sensor3Read()
{
  float saveValue[10];
  float average = 0;
  // saveValue[0] = sensor[2].ranging_data.range_mm;
  for (int i = 0; i < 10; i++)
  {
    saveValue[i] = sensor[2].read();
    average += saveValue[i];
  }
  average /= 10;
  return average;
}

void soloSensorOutput()
{
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

  float average[5] = {0, 0, 0, 0, 0};

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
// '10' wird nicht richtig übertragen, weil der Wert dann über12000 ist/ Empfangen => keine Aktion
void Robotturn20()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(1);
  Wire.endTransmission();
}

void Robotturn90()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(2);
  Wire.endTransmission();
}

void Robotturn15()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(3);
  Wire.endTransmission();
}
void Robotturn5()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(4);
  Wire.endTransmission();
}

// Roboter vor Fahren lassen
void RobotForward()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(5);
  Wire.endTransmission();
}
void turnLeft()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(6);
  Wire.endTransmission();
}
void RobotStop()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(7);
  Wire.endTransmission();
}
// Variable des Sensors 3 an Roboter senden (die zwei, weil von 0 aus gezählt wird)
void Robotsend3()
{
  char value[5];
  Wire.beginTransmission(Roboter_ID);
  Wire.write(sprintf(value, "%04d", sensor[2].ranging_data.range_mm));
  Wire.endTransmission();
}

// Funktion zum Aufsetzen eines Auswahlfeldes
void movefield()
{
  // tft.fillRect(0,0,250,250,TFT_BLACK);
  uint8_t colorChangeColum = 0;
  uint8_t colorChangeRow = 0;

  for (uint8_t rows = 0; rows < 4; rows++)
  {
    for (uint8_t colum = 0; colum < 4; colum++)
    {

      tft.drawRect(35 * colum, 37.5 * rows + 90, 30, 30, TFT_GOLD);

      // Felderstellung mit unterschiedlichen Farben
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
void movecount(uint8_t counts)
{
  movefield();
  movcolum = counts;
  if (counts > 3)
  {
    counts = 0;
    movcolum = counts;
    movrow++;
  }
  if (movrow > 3)
  {
    movrow = 0;
  }
  tft.fillRect(35 * movcolum, 37.5 * movrow + 90, 30, 30, TFT_GOLD);

  if (movcolum == 0)
  {
    tft.fillRect(35 * 3, 37.5 * 3 + 90, 30, 30, TFT_BLACK);
    tft.drawRect(35 * 3, 37.5 * 3 + 90, 30, 30, TFT_GOLD);
  }
  if (movcolum > 0 && movcolum <= 3)
  {
    tft.fillRect(35 * (movcolum - 1), 37.5 * movrow + 90, 30, 30, TFT_BLACK);
    tft.drawRect(35 * (movcolum - 1), 37.5 * movrow + 90, 30, 30, TFT_GOLD);
  }
}
// Reset beim Verlassen des Feldes
void movreset()
{
  movcolum = 0;
  movrow = 0;
  Buttoncount2 = 0;
}
// Display Reseten(komplett Schwarz färben)
void displayReset()
{
  tft.fillRect(0, 0, 250, 250, TFT_BLACK);
}

// Messen in 15° Schritten
void measurement15()
{

  for (position = 0; position < 24; position++)
  { // Bei 20° Schritten position <18, bei 15°Schritten position<24
    delay(100);
    measurementvalue15[position] = sensorRead();
    Serial.println(measurementvalue15[position]);
    delay(100);
    Robotturn15(); // Roboterbewegung der Zahlenwert gibt die Gradzahl an.

    delay(100);
    if (position < 15)
    {
      tft.setCursor(0, 15 * (position + 1));
      tft.setTextColor(TFT_GOLD);
      tft.print(measurementvalue15[position]);
    }

    if (position >= 15 && position < 30)
    {
      tft.setCursor(40, 15 * (position + 1 - 15));
      tft.setTextColor(TFT_GREEN);
      tft.print(measurementvalue15[position]);
    }

    else
    {
      tft.setCursor(80, 15 * (position + 1 - 32));
      tft.setTextColor(TFT_RED);
      tft.print(measurementvalue15[position]);
    }
  }

  if (position >= 24)
  {
    position = 0;
  }
  delay(100);
  measurementvalue15[24] = sensorRead();
  delay(100);
  Serial.println(measurementvalue15[24]);
  tft.setCursor(80, 0); // 15*(24+1-18)
  tft.setTextColor(TFT_BLUE);
  tft.print(measurementvalue15[24]);
}

// Aufnehmen von Messdaten in 5° Schritten
void measurement5()
{
  // measurementvalue5[position] = sensor3Read();

  for (position = 0; position < 72; position++)
  { // Bei 5° Schritten position <72, bei 15°Schritten position<24
    // delay(100);
    measurementvalue5[position] = sensor3Read();
    Serial.println(measurementvalue5[position]);
    // delay(100);
    Robotturn5(); // Roboterbewegung der Zahlenwert gibt die Gradzahl an.

    // delay(100);
    if (position < 15)
    {
      tft.setCursor(0, 15 * (position + 1));
      tft.setTextColor(TFT_GOLD);
      tft.print(measurementvalue5[position]);
    }

    if (position >= 15 && position < 30)
    {
      tft.setCursor(40, 15 * (position + 1 - 15));
      tft.setTextColor(TFT_GREEN);
      tft.print(measurementvalue5[position]);
    }

    if (position >= 30 && position < 45)
    {
      tft.setCursor(80, 15 * (position + 1 - 30));
      tft.setTextColor(TFT_RED);
      tft.print(measurementvalue5[position]);
    }
    if (position == 45)
    {
      tft.fillRect(40, 0, 250, 250, TFT_BLACK);
    }
    if (position >= 45 && position < 60)
    {

      tft.setCursor(40, 15 * (position + 1 - 45));
      tft.setTextColor(TFT_GREEN);
      tft.print(measurementvalue5[position]);
    }

    if (position >= 60 && position < 75)
    {
      tft.setCursor(80, 15 * (position + 1 - 60));
      tft.setTextColor(TFT_RED);
      tft.print(measurementvalue5[position]);
    }

    if (position >= 85 && position < 100)
    {
      tft.setCursor(120, 15 * (position + 1 - 60));
      tft.setTextColor(TFT_BLUE);
      tft.print(measurementvalue5[position]);
    }
  }

  if (position >= 72)
  {
    position = 0;
  }
  // delay(100);
  // measurementvalue5[72] = sensor3Read();
  // delay(100);
  // Serial.println(measurementvalue5[72]);
  tft.setCursor(80, 0); // 80,25*(24+1-18)
  tft.setTextColor(TFT_BLUE);
  tft.print(measurementvalue5[72]);
}

// Umrechnen der Werte in X Koordinaten (5° Schritte)
void convertMeasuermentX5()
{
  displayReset();
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_BROWN);
  tft.print("xWerte");
  for (position = 0; position < 72; position++)
  {
    xValues5[position] = (cosf(5 * position * DEG_TO_RAD)) * measurementvalue5[position];
    /* War zur Ausgabe der x-Werte um Sie auf Ihre Richtigkeit zu kontrollieren kann später komplett entfernt werden
        if (position < 15)
        {

          tft.setCursor(0, 15 * (position + 1));
          if (position == 0)
          {
            tft.setTextColor(TFT_BLUE);
          }
          else
          {
            tft.setTextColor(TFT_GOLD);
          }
          tft.print(xValues5[position]);
        }

        if (position >= 15 && position < 30)
        {

          tft.setCursor(40, 15 * (position + 1 - 15));
          tft.setTextColor(TFT_GREEN);
          tft.print(xValues5[position]);
        }

        if (position >= 30 && position < 45)
        {
          tft.setCursor(80, 15 * (position + 1 - 30));
          tft.setTextColor(TFT_RED);
          tft.print(xValues5[position]);
        }
        if (position == 45)
        {
          tft.fillRect(40, 0, 250, 250, TFT_BLACK);
        }
        if (position >= 45 && position < 60)
        {

          tft.setCursor(40, 15 * (position + 1 - 45));
          tft.setTextColor(TFT_GREEN);
          tft.print(xValues5[position]);
        }

        if (position >= 60 && position < 75)
        {

          tft.setCursor(80, 15 * (position + 1 - 60));
          if (position == 71)
          {
            tft.setTextColor(TFT_SKYBLUE);
          }
          else
          {
            tft.setTextColor(TFT_RED);
          }

          tft.print(xValues5[position]);
        }

        if (position >= 85 && position < 100)
        {
          tft.setCursor(120, 15 * (position + 1 - 60));
          tft.setTextColor(TFT_BLUE);
          tft.print(xValues5[position]);
        }
        */
  }

  delay(100);
}

// Umrechnen der Daten in Y Koordinaten ( 5° Schritte)
void convertMeasurementY5()
{
  for (position = 0; position < 72; position++)
  {
    yValues5[position] = sinf(5 * position * DEG_TO_RAD) * measurementvalue5[position];
    /*
       if (position < 15)
    {

      tft.setCursor(0, 15 * (position + 1));
      if (position == 0)
      {
        tft.setTextColor(TFT_BLUE);
      }
      else
      {
        tft.setTextColor(TFT_GOLD);
      }
      tft.print(xValues5[position]);
    }

    if (position >= 15 && position < 30)
    {

      tft.setCursor(40, 15 * (position + 1 - 15));
      tft.setTextColor(TFT_GREEN);
      tft.print(xValues5[position]);
    }

    if (position >= 30 && position < 45)
    {
      tft.setCursor(80, 15 * (position + 1 - 30));
      tft.setTextColor(TFT_RED);
      tft.print(xValues5[position]);
    }
    if (position == 45)
    {
      tft.fillRect(40, 0, 250, 250, TFT_BLACK);
    }
    if (position >= 45 && position < 60)
    {

      tft.setCursor(40, 15 * (position + 1 - 45));
      tft.setTextColor(TFT_GREEN);
      tft.print(xValues5[position]);
    }

    if (position >= 60 && position < 75)
    {

      tft.setCursor(80, 15 * (position + 1 - 60));
      if (position == 71)
      {
        tft.setTextColor(TFT_SKYBLUE);
      }
      else
      {
        tft.setTextColor(TFT_RED);
      }

      tft.print(xValues5[position]);
    }

    if (position >= 85 && position < 100)
    {
      tft.setCursor(120, 15 * (position + 1 - 60));
      tft.setTextColor(TFT_BLUE);
      tft.print(xValues5[position]);
    }
    */
  }
}

// Zeichnen einer Karte aufgrund von Messdaten
void mapDrawing()
{
  measurement5();
  convertMeasuermentX5();
  convertMeasurementY5();
  displayReset();
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_GOLD);
  tft.print("Mapdrawing");
  tft.drawRect(0, 20, 130, 220, TFT_RED);

  int roundValueX = 0;
  int roundValueY = 0;
  tft.drawPixel(60, 140, TFT_ORANGE);
  for (uint8_t i = 0; i < 72; i++)
  {
    roundValueX = xValues5[i] / 10;
    roundValueY = yValues5[i] / 10;

    if (i < 12)
    {
      tft.drawPixel(65 - (roundValueY), 140 - (roundValueX), TFT_BLUE);
    }
    if (i >= 12 && i < 24)
    {
      tft.drawPixel(65 - (roundValueY), 140 - (roundValueX), TFT_RED);
    }
    if (i > 24 && i < 36)
    {
      tft.drawPixel(65 - (roundValueY), 140 - (roundValueX), TFT_GOLD);
    }
    if (i >= 36 && i < 48)
    {
      tft.drawPixel(65 - (roundValueY), 140 - (roundValueX), TFT_GREEN);
    }
    if (i >= 48 && i < 60)
    {
      tft.drawPixel(65 - (roundValueY), 140 - (roundValueX), TFT_WHITE);
    }
    if (i >= 60 && i < 100)
    {
      tft.drawPixel(65 - (roundValueY), 140 - (roundValueX), TFT_SKYBLUE);
    }
  }
}
// Roboter aufgrund der Sensordaten 1 cm vor Fahren lassen.
void Drive10()
{
  sensorRead();
  displayReset();
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_GREEN);
  tft.println("10cm Forwärtsfahren");
  int now = sensorRead();

  int drive = now - 2;
  tft.setTextColor(TFT_RED);
  tft.print("Ziel: ");
  tft.print(drive);
  RobotForward();
  tft.setTextColor(TFT_GOLD);
  while (now > drive)
  {
    tft.fillRect(0, 50, 100, 100, TFT_BLACK);
    tft.setCursor(0, 50);
    tft.print("Momentan: ");
    tft.print(now);
    now = sensorRead();
  }
  RobotStop();
  tft.setCursor(0, 70);
  tft.print("Endvalue: ");
  tft.print(now);
}

/**
 * Funktion zum Hochladen der X und Y Koordinaten der Karte auf ThingSpeak.
 * Es werden die umgewandelten Korrdinaten in jeweils zwei Strings
 * augeteilt, da ThingSpeak nur eine Länge von max. 255 Zeichen erlaubt.
 * Am Ende werden die Daten ueber die ersrten vier Felder hochgeladen.
 */
void uploadMap5()
{
  String Xcoords[] = {"", ""};
  String Ycoords[] = {"", ""};

  for (int j = 0; j < 2; j++)
  {
    bool isFirst = true;
    for (int i = 0; i < 36; i++)
    {
      if (!isFirst)
      {
        Xcoords[j] += ";";
        Ycoords[j] += ";";
      }

      Xcoords[j] += (int)xValues5[i + (36 * j)];
      Ycoords[j] += (int)yValues5[i + (36 * j)];
      isFirst = false;
    }
  }

  ThingSpeak.setField(1, Xcoords[0]);
  ThingSpeak.setField(2, Xcoords[1]);
  ThingSpeak.setField(3, Ycoords[0]);
  ThingSpeak.setField(4, Ycoords[1]);
  ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);
}

void setup()
{

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

  btn1.begin(BUTTON_1, INPUT_PULLUP);
  btn2.begin(BUTTON_2, INPUT_PULLUP);

  btn1.setClickHandler(handler1);
  btn2.setClickHandler(handler2);
  btn1.setDoubleClickHandler(doubleClickHandler1);
  btn2.setDoubleClickHandler(doubleClickHandler2);
  btn1.setLongClickDetectedHandler(longClickHandler1);
  btn2.setLongClickDetectedHandler(longClickHandler2);

  btn1.setDebounceTime(20);
  btn2.setDebounceTime(20);
  ThingSpeak.begin(client); // to initilize the thingspeak library
}

void loop()
{

  btn1.loop();
  btn2.loop();

  if (click1 == true)
  {
    Buttoncount1++;
    movreset();
    displayReset();
    tft.setCursor(0, 0);

    tft.setTextColor(TFT_GREEN);
    tft.println("SoloMeasurement ");
    tft.print("Counts: ");
    tft.print(Buttoncount1);
    // RobotForward();
    mapDrawing();
    //  soloSensorOutput();
    uploadMap5();
    Serial.println("Done uploading map");
    click1 = false;
  }
  if (click2 == true)
  {
    // Robotturn90();
    displayReset();
    /* Erstellen des Auswahlfeldes und bewegen des Feldes
    movecount(Buttoncount2);
    if (Buttoncount2 > 3)
    {
      Buttoncount2 = 0;
    }
    */
    // RobotStop();
    Drive10();
    /*
    Buttoncount2++;
    tft.setCursor(80, 0);
    // tft.fillRect(0,220,125,10,TFT_BLACK);
    tft.setTextColor(TFT_BLUE);
    tft.print("SelectMode, Counts:");
    tft.print(Buttoncount2);
  */
    click2 = false;
  }
  if (doubleclick1 == true)
  {
    doubleclick1 = false;
    displayReset();
    movreset();
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_YELLOW);
    tft.print("Measurmode 15");
    measurement15();

    tft.print(Buttoncount2);
  }
  if (doubleclick2 == true)
  {
    doubleclick2 = false;
    displayReset();
    movreset();
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_COLMOD);
    tft.print("Measuremode 5");

    measurement5();
  }
}