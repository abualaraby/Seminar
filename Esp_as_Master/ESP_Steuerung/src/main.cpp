#include <Arduino.h>
#include <Arduino.h>
#include "WiFi.h"     // Für Wlan-Verbindung
#include <TFT_eSPI.h> // Für den ESP32 und das Bildschirm
#include <SPI.h>
#include <Wire.h>
#include "VL53L1X.h" // Für die Sensoren
#include "secrets.h"
#include "splash.h"
#include "Button2.h"
#include "ThingSpeak.h" // Für die Verbindung mit ThingSpeak

#define CHANNEL_ID 1713470                 // Chris: 1759092 // Ahmed:1713470                 // Channel ID on Thingspeak
#define CHANNEL_API_KEY "K7G57B13MZEUEGW7" // Chris: KBZPVDC83RELSMFS // Ahmed: "K7G57B13MZEUEGW7" // API Read Key from Thingspeak

WiFiClient client; // this will be used by Thingspeak Library to make HTTP request

int counter = 0; // To know exactly how much Data have been sent

// Variablen für Buttonbefehle
uint8_t Buttoncount1 = 0; // Test der Buttonfunktionen mit hilfe von Interrupts
uint8_t Buttoncount2 = 0;
bool Menu = true; // Variable zum aktivieren und deaktivieren des Menüs
bool click1 = false;
bool click2 = false;
bool doubleclick1 = false;
bool doubleclick2 = false;
bool longclick1 = false;
bool longclick2 = false;

uint8_t menu = 0;

uint8_t movcolum = 0;
uint8_t movrow = 0;

uint8_t measurementCount = 0;
int position = measurementCount * 71; // Variable zum Definieren der Position
int safePosition = 0;
float measurementvalue15[24]; // Ausgabe der Drehmessung
float measurementvalue5[72];

float xValues5[72]; // Variable zum speichern der Messwerte x
float yValues5[72]; // Variable zum speichern der Messwerte y
float xValues15[24];
float yValues15[24];

String Xcoords[] = {"", "", "", ""}; // Variable zum Speichern Die X-Koordinate in einem Stringkette
String Ycoords[] = {"", "", "", ""}; // Variable zum Speichern Die Y-Koordinate in einem Stringkette

float xDrive = 0;
float yDrive = 0;
float angleDrive = 0;
uint8_t angleCount = 0;
int realAngleDrive = 0;
float polarDrive = 0;
uint16_t realpolarDrive = 0;
float safeRealpolarDrive=0;

float longSideX = 0;
float longSideY = 0;
float shortSideX = 0;
float shortSideY = 0;

uint8_t xPosition = 140;
uint8_t yPosition = 65;
uint8_t cursorX = 65;
uint8_t cursorY = 140;
bool mapIsOpen = false;

// Wifi Konfiguration
#define WIFI_NETWORK "Drei"    //"Drei"        // WIFI Name
#define WIFI_PASSWORD "Howyoudoin" //"Howyoudoin" // WIFI Password
#define WIFI_TIMEOUT 10000       // 20000       // in milliseconds

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
  WiFi.mode(WIFI_STA); // STA = Station (Wird für Router genutzt), AP = Access Point (Wird mit AP und Handys genutzt)
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
/*
void Robotturn90()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(2);
  Wire.endTransmission();
}
*/

void Robotturn5()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(1);
  Wire.endTransmission();
  delay(100);
}


void Robotdrive1()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(2);
  Wire.endTransmission();
  delay(500);
}

void Robotdrive2()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(3);
  Wire.endTransmission();
  delay(1000);
}
void Robotdrive5()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(4);
  Wire.endTransmission();
  delay(2000);
}
void Robotdrive10()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(5);
  Wire.endTransmission();
  delay(4000);
}
void Robotdrive20()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(6);
  Wire.endTransmission();
  delay(8000);
}

void Robotdrive50()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(7);
  Wire.endTransmission();
  delay(10000);
}

void Robotdrive100()
{
  Wire.beginTransmission(Roboter_ID);
  Wire.write(8);
  Wire.endTransmission();
  delay(100000);
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
    // Robotturn15(); // Roboterbewegung der Zahlenwert gibt die Gradzahl an.

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
    // delay(50);
    measurementvalue5[position] = sensor3Read();
    Serial.println(measurementvalue5[position]);
    // delay(50);
    Robotturn5(); // Roboterbewegung der Zahlenwert gibt die Gradzahl an.
  }

  if (position < 72)
  {
    position = 0;
  }

  // tft.setCursor(80, 0); // 80,25*(24+1-18)
  // tft.setTextColor(TFT_BLUE);
  // tft.print(measurementvalue5[72]);
}

// Umrechnen der Werte in X Koordinaten (5° Schritte)
void convertMeasuermentX5()
{

  // tft.setCursor(0, 0);
  // tft.setTextColor(TFT_BROWN);
  // tft.print("xWerte");

  for (position = 0; position < 72; position++)
  {
    xValues5[position] = (cosf(5 * position * DEG_TO_RAD)) * measurementvalue5[position];
  }

  // delay(100);
}

// Umrechnen der Daten in Y Koordinaten ( 5° Schritte)
void convertMeasurementY5()
{

  for (position =0; position < 72 ; position++)
  {
    yValues5[position] = sinf(5 * position * DEG_TO_RAD) * measurementvalue5[position];
  }
}
// Funktion zum zurücksetzen des Cursors
void resetCursor()
{
  cursorX = 65; // Verschiebung nach unten zum Nullpunkt der x-Achse
  cursorY = 140; // Verschiebung nach rechts zum Nullpunkt der y-Achse
  xDrive = 0;
  yDrive = 0;
  angleDrive = 0;
  angleCount = 0;
  realAngleDrive = 0;
  polarDrive = 0;
}
// Zeichnen einer Karte aufgrund von Messdaten (1 Messung)
void mapDrawing()
{
  // Max x = 134; Max y=239
  displayReset();
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_GOLD);
  tft.print("Mapdrawing");
  tft.drawRect(0, 20, 130, 220, TFT_RED);

  int roundValueX = 0;
  int roundValueY = 0;
  tft.drawCircle(65, 140, 3, TFT_RED);

  for (uint8_t i = 0 ; i < 72 ; i++)
  {
    roundValueX = xValues5[i] / 10;
    roundValueY = yValues5[i] / 10;

    tft.drawPixel(yPosition - (roundValueY), xPosition - (roundValueX), TFT_WHITE);
    resetCursor();
  }
}

void mapPosition()
{
  xPosition = xDrive / 10;
  yPosition = yDrive / 10;
  angleDrive;
}

// Nächste Messung

// Funktionen zum setzen eines Cursors auf dem Bildschirm, um später den Roboter dort
// hin fahren zu lassen.
void mapCursorMoveX()
{
  cursorX++;
  if (cursorX == 139)
  {
    cursorX = 0;
    tft.drawPixel(cursorX, cursorY, TFT_GOLD);
    tft.drawPixel(138, cursorY, TFT_BLACK);
  }
  else
  {
    tft.drawPixel(cursorX+1, cursorY, TFT_GOLD);
    tft.drawPixel(cursorX  -2, cursorY, TFT_BLACK);
  }
}
void mapCursorMoveY()
{
  cursorY--;
  if (cursorY == 20)
  {
    cursorY = 239;
    tft.drawPixel(cursorX, cursorY, TFT_GOLD);
    tft.drawPixel(cursorX, 21, TFT_GOLD);
  }
  else
  {
    tft.drawPixel(cursorX, cursorY-1, TFT_GOLD);
    tft.drawPixel(cursorX, cursorY +2, TFT_BLACK);
  }
}

// Konvertierung der Cursor Position um den Roboter dort hin fahren zu lassen.
void convertCursorPosition()
{
  xDrive = (cursorY - 140) * -10;    //10
  yDrive = (cursorX - 65) * -10;    //-10

  angleDrive = atan(-yDrive / xDrive) * RAD_TO_DEG;
  
  if(angleDrive>0){
    angleCount=(360-angleDrive)/5;
  }
  
  if(angleDrive=0){
    angleCount=0;
  }

  if(angleDrive<0){
    angleCount=(angleDrive)/5;
  }
  
 
  realAngleDrive = 5 * angleCount;

  polarDrive = sqrt(xDrive * xDrive + yDrive * yDrive);
  realpolarDrive = polarDrive / 10;
  safeRealpolarDrive=realpolarDrive;

  displayReset();
  tft.setCursor(0, 10);
  tft.setTextColor(TFT_GREEN);
  tft.printf("x Wert: %f ", yDrive);

  tft.setCursor(0, 20);
  tft.printf("y Wert: %f", -xDrive);

  tft.setCursor(0, 30);
  tft.printf("Winkel: %f ", angleDrive);

  tft.setCursor(0, 40);
  tft.printf("Umdrehungssprünge= %d", angleCount);

  tft.setCursor(0, 50);
  tft.printf("Wahrer Fahrwinkel= %d", realAngleDrive);

  tft.setCursor(0, 70);
  tft.printf("Polarentfernung in");
  tft.setCursor(0, 80);
  tft.printf("mm: %f", polarDrive);

  tft.setCursor(0, 100);
  tft.printf("Wahre Farhentfernung ");
  tft.setCursor(0, 110);
  tft.printf("in cm: %d", realpolarDrive);
}

void driveCursorPosition()
{

  uint8_t Drive100 = 0;
  uint8_t Drive50 = 0;
  uint8_t Drive20 = 0;
  uint8_t Drive10 = 0;
  uint8_t Drive5 = 0;
  uint8_t Drive2 = 0;
  uint8_t Drive1 = 0;
  int moduloValue100=0;
  int moduloValue50=0;
  int moduloValue20=0;
  int moduloValue10=0;
  int moduloValue5=0;
  int moduloValue2=0;
  int moduloValue1=0;

  for (int i = 0; i < angleCount; i++)
  {
    Robotturn5();
  }

  Drive100= realpolarDrive/100;
  moduloValue100=realpolarDrive%100;

  Drive50=moduloValue100/50;
  moduloValue50=moduloValue100 % 50;

  Drive20=moduloValue50/20;
  moduloValue20=moduloValue50 % 20;

  Drive10=moduloValue20 /10;
  moduloValue10= moduloValue20 % 10;

  Drive5=moduloValue10/5;
  moduloValue5=moduloValue10 % 5;

  Drive2=moduloValue5/2;
  moduloValue2=moduloValue5 % 2;

  Drive1=moduloValue2;

  if(Drive100>=1){
    for(int i=0;i<Drive100;i++){
      Robotdrive100();
    }
  }
  if(Drive50>=1){
    for(int i=0;i<Drive50;i++){
      Robotdrive50();
    }
  }
  if(Drive20>=1){
    for(int i=0;i<=Drive20;i++){
      Robotdrive20();
    }
  }
  if(Drive10>=1){
    for(int i=0;i<Drive10;i++){
      Robotdrive10();
    }
  }
  if(Drive5>=1){
    for(int i=0;i<Drive5;i++){
      Robotdrive5();
    }
  }
  if(Drive2>=1){
    for(int i=0;i<Drive2;i++){
      Robotdrive2();
    }
  }
  if(Drive1>=1){
    for(int i=0;i<Drive1;i++){
      Robotdrive1();
    }
  }
/*
  for(int i=0;i<72-angleCount;i++){
    Robotturn5();
  }
*/


}

/**
 * Funktion zum Hochladen der X und Y Koordinaten der Karte auf ThingSpeak.
 * Es werden die umgewandelten Koordinaten in jeweils zwei Strings
 * augeteillt, da ThingSpeak nur eine Länge von max. 255 Zeichen erlaubt.
 * Am Ende werden die Daten ueber die ersten vier Felder hochgeladen.
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

/**
 * @brief
 *Funktion zum Hinzufügen der ausgefahrenen Distanz zu den Daten erstellt.
 *Die Daten bei der X-Achse werden ins Format {500|362;785;3215;315;……} ausgebildet,
 *wobei der Wert vor dem Zeichen | die Distanz ist und die Werte nach dem gleichen Zeichen
 *sind die Werte vom Sensor mit einem Semikolon ; für trennen der Werten auseinander.
 * @param isFirst
 */

void buildData(bool isFirst)

{
  int offset = 2;
  if (isFirst)
  {
    Xcoords[0] += realpolarDrive;
    Xcoords[0] += "|";
    Ycoords[0] += realAngleDrive;
    Ycoords[0] += "|";
    offset = 0;
  }

  for (int j = 0; j < 2; j++)
  {
    bool isFirst = true;
    for (int i = 0; i < 36; i++)
    {
      if (!isFirst)
      {
        Xcoords[j + offset] += ";";
        Ycoords[j + offset] += ";";
      }

      Xcoords[j + offset] += (int)xValues5[i + (36 * j)];
      Ycoords[j + offset] += (int)yValues5[i + (36 * j)];
      isFirst = false;
    }
  }
}

/**
 * @brief
 * Funktion zum Hochladen der X und Y Koordinaten der Karte auf ThingSpeak.
 * Es werden die umgewandelten Koordinaten in jeweils Vier Strings
 * augeteillt, da ThingSpeak nur eine Länge von max. 255 Zeichen erlaubt.
 * Am Ende werden die Daten ueber die ersten Acht Felder hochgeladen .
 * (Feld 1&2 für erste X Werte von der ersten Messung, 3&4 für zweite X Werte von der 2ten Messung).
 * (Feld 5&6 für erste Y Werte von der ersten Messung, 7&8 für zweite Y Werte von der 2ten Messung).
 */
void uploadMap()
{
  // Uploading map data
  for (int i = 0; i < 4; i++)
  {
    ThingSpeak.setField(i + 1, Xcoords[i]);
    ThingSpeak.setField(i + 5, Ycoords[i]);
  }
  ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);

  // Resetting string arrays for next use
  for (int i = 0; i < 4; i++)
  {
    Xcoords[i] = "";
    Ycoords[i] = "";
  }
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
  btn1.setLongClickTime(3000);
  btn2.setLongClickTime(3000);
  ThingSpeak.begin(client); // to initilize the thingspeak library
}

void loop()
{

  btn1.loop();
  btn2.loop();

  if (Menu == true)
  {
    Menu = false;
    displayReset();
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE);
    // Start Menü
    tft.println("Menu");
    tft.println("");
    tft.setTextColor(TFT_GOLD);
    // Aktion linker Knopfdurck Cursor nach unten führen nach Messung
    tft.println("Einzel Klick links = ");

    tft.println("Cursor nach oben");
    tft.println("");
    tft.setTextColor(TFT_GREEN);
    // Aktion Recher Knopfdruck Cursor nach unten führen nach Messung
    tft.println("Einzel Klick rechts =");
    tft.println("");
    tft.println("Cursor nach rechts");
    tft.println("");

    tft.setTextColor(TFT_GOLD);
    // Aktion linker Doppelknopfdruck
    tft.println("Doppelklick Links  = ");
    tft.println("");
    tft.println("Doppelte Rundummessung");
    tft.println("");
    tft.println("Mit Bildschirmausgabe und upload");

    tft.println("");
    tft.setTextColor(TFT_GREEN);
    // Aktion rechter Doppelknopfdruck 30 cm nach vorne fahren
    tft.println("Doppelklick Rechts = ");
    tft.println("");
    tft.println("50 cm nach vorne");
    tft.println("fahren ohne");
    tft.println("ohne Sensordaten");
    tft.println("");
    tft.setTextColor(TFT_GOLD);
    // Aktion Links lange halten zurück zum Menü
    tft.println("Langer Klick links = ");
    tft.println("");
    tft.println("Back to Menu");
    tft.println("");
    // Aktion Rechts lange halten Fahrt zum Cursor Punkt.
    tft.setTextColor(TFT_GREEN);
    tft.println("Langer Klick Rechts = ");
    tft.println("");
    tft.println("Zielposition auswäheln und fahren");
  }

  if (click1 == true)
  {

    click1 = false;
    if (mapIsOpen == true)
    {
      mapCursorMoveY();
    }
    else
    {
      
      tft.println("");
      tft.setTextColor(TFT_RED);
      tft.println("Error");
    }
  }

  if (click2 == true)
  {
    click2 = false;
    if (mapIsOpen == true)
    {
      mapCursorMoveX();
    }
    else
    {
     
      tft.setTextColor(TFT_RED);
      tft.println("Error");
    }
  }

  if (doubleclick1 == true)
  {
    doubleclick1 = false;
    if (mapIsOpen == true)
    {
      mapCursorMoveX();
    }
    else
    {
      Buttoncount1++;
      movreset();
      displayReset();
      tft.setCursor(0, 0);

      tft.setTextColor(TFT_GREEN);
      tft.println("SoloMeasurement ");
      tft.print("Counts: ");
      tft.print(Buttoncount1);

      measurement5();
      convertMeasuermentX5();
      convertMeasurementY5();
      mapDrawing();

      /*
      buildData(true, 500);
      delay(2000);
      Robotdrive50();
      delay(5000);

      measurement5();
      convertMeasuermentX5();
      convertMeasurementY5();
      mapDrawing();

      buildData(false);
      uploadMap();
      */
      tft.setCursor(75, 0);
      tft.setTextColor(TFT_GREEN);
      tft.print("done");
      Serial.println("Done uploading map");

      mapIsOpen = true;
    }
  }
  if (doubleclick2 == true)
  {
    doubleclick2 = false;
    // if (mapIsOpen == true)
    // {
    //   mapCursorMoveY();
    // }
    // else
    // {
    displayReset();
    // convertCursorPosition();
    delay(50);
   

    tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);
    tft.println("Robot is Driving");

    mapIsOpen = false;
    //}
  }
  if (longclick1 == true)
  {
    Menu = true;
    longclick1 = false;
    mapIsOpen = false;
  }
  if (longclick2 == true)
  {
    convertCursorPosition();
    buildData(true);
    
    delay(3000);
    driveCursorPosition();

    delay(5000);
    measurement5();
    convertMeasuermentX5();
    convertMeasurementY5();
    mapDrawing();
    buildData(false);
    uploadMap();
    longclick2 = false;
    doubleclick2 = false;
    // mapIsOpen = false;
  }
}