#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "secrets.h"
#include "splash.h"
#include "Button2.h"

TFT_eSPI tft;
VL53L1X sensor[5];
uint8_t sensorPin[5] = {17, 2, 15, 13, 12}; // XSHUT pins at ESP
uint8_t sensorAddr[5] = {0x51, 0x52, 0x53, 0x54, 0x55}; // new addresses for sensors

#define BUTTON_1            35
#define BUTTON_2            0

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34

int vref = 1100;

const int Roboter_ID = 5;

void initToFs () {
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPin[i], OUTPUT);
    digitalWrite(sensorPin[i], LOW); // shut down sensor i
  }

  Serial.println("init ToF Sensors...");

  for (int i = 0; i < 5 ; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");

    digitalWrite(sensorPin[i], HIGH); // wake up next sensor
    sensor[i].setTimeout(500);
    if (!sensor[i].init())
    {
      Serial.println("failed!");
    }
    else
    {
      sensor[i].setAddress(sensorAddr[i]); // set new address

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


void drehung45() {
  Wire.beginTransmission(Roboter_ID);
  Wire.write('a');
  Wire.endTransmission();
}

void drehung90() {
  Wire.beginTransmission(Roboter_ID);
  Wire.write('b');
  Wire.endTransmission();
}

void fahren() {
  Wire.beginTransmission(Roboter_ID);
  Wire.write('c');
  Wire.endTransmission();
}

void fahrenStrecke() {
  Wire.beginTransmission(Roboter_ID);
  int a = 0;
  a = sensor[2].ranging_data.range_mm; // Sensorwert (Strecke) auslesen
  char zahl[3];
  sprintf(zahl, "%03d", a); //Sensorwert in 3 stellige Zeichenkette
  tft.print(zahl);
  tft.print(zahl[0]);
  tft.print(zahl[1]);
  tft.print(zahl[2]);

  //  char Str[4] = {};
  //  Str[0] = zahl[0];
  //  Str[1] = zahl[1];
  //  Str[2] = zahl[2];
  //  Str[3] = ':';
  Wire.write('#');

  Wire.write(zahl[0]);
  Wire.write(zahl[1]);
  Wire.write(zahl[2]);
  //  Wire.write(zahl);
  Wire.write('#');
  Wire.endTransmission();

}

bool BUTTON1 = false;
void IRAM_ATTR toggleButton1() {
  BUTTON1 = true;
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 135, 240, splash);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  delay (1000);

  initToFs();

  Serial.print("req ID from 3pi+...");
  Wire.requestFrom(5, 10); // read ID from robot
  Serial.print("done. ID: ");

  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }

  Serial.println("");

  delay(5000);


  pinMode(BUTTON_1 , INPUT);
  attachInterrupt(BUTTON_1 , toggleButton1, FALLING);

}

float strecke[9]; // 9, da der 360 Grad (=anfangswert) dabei ist
int i = 0;
void anzeigeStrecke() {
  sensor[2].read();
  tft.print(" Sensor:");
  tft.print(sensor[2].ranging_data.range_mm);
  strecke[i] = sensor[2].ranging_data.range_mm;
  i++;
}


int a = 0;
void loop()
{

  if (a < 1) {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE);
    tft.print(" Start ");
    a++;
  }

  btn1.loop();
  btn2.loop();

  //  for (int i = 0; i<5; i++) {
  //    sensor[i].read();
  //    Serial.print("Sensor");
  //    Serial.print(i+1);
  //    Serial.print(": ");
  //    Serial.print(sensor[i].ranging_data.range_mm);
  //    Serial.print("\t");
  //  }
  //  Serial.println("");
  //  Serial.print("\tstatus: ");
  //  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  //  Serial.print("\tpeak signal: ");
  //  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  //  Serial.print("\tambient: ");
  //  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);
  //  Serial.println();
  //  displayDistance();
  //  showVoltage();


  // Werte aus Sensore auslesen
  //float strecke;
  //  if (btn1.wasPressed() )
  //  {
  //      sensor[2].read();
  //      tft.print("Sensor:");
  //      tft.print(sensor[2].ranging_data.range_mm);
  //      // immer nur ein Wert gegebenenfalls Array benutzen und dann mit schleife bestücken
  //      strecke = sensor[2].ranging_data.range_mm;
  //
  //    }


  // fahren();

  int winkel = 0;
  //  if (btn1.isPressed() )
  if (BUTTON1 = true)
  {
    anzeigeStrecke(); // Strecke abrufen, ausgeben 0 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 45 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 90 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 135 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 180 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 225 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 270 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 315 Grad
    drehung45();      // drehung
    anzeigeStrecke(); // Strecke abrufen, ausgeben 360 Grad (anfangswert)

    BUTTON1 = false;

    float xStrecke[9];       // 9, da der 360 Grad (=anfangswert) dabei ist
    float yStrecke[winkel];  // 9, da der 360 Grad (=anfangswert) dabei ist
    // Umrechnen der Daten in X Koordinaten
    // x = cos(Winkel)*Strecke
    for (winkel = 0; winkel < 8; winkel++) { //winkel entspricht den 45Grad positionen, wodurch ich von 0-360 jede durchgehe und den xStrecken wert berechne
      xStrecke[winkel] = cosf(45 * winkel * DEG_TO_RAD) * strecke[winkel];
      tft.print(" xStrecke:");
      tft.print(xStrecke[winkel]);
    }

    // Umrechnen der Daten in Y Koordinaten
    // y = sin( Winkel )*Strecke
    for (int winkel = 0; winkel < 8; winkel++) { //winkel entspricht den 45Grad positionen, wodurch ich von 0-360 jede durchgehe und den yStrecken wert berechne
      yStrecke[winkel] =  sinf(45 * winkel * DEG_TO_RAD) * strecke[winkel];
      tft.print(" yStrecke:");
      tft.print(yStrecke[winkel]);
    }
    delay(20000);

    ////Algorytmus entscheidung gerade aus fahren oder rechts fahren
    //// könnte noch mit rückwärts fahren und links fahren erweitert werden
    //if(strecke[0] > strecke[2]){ // 0Grad größer 90Grad
    //  fahrenStrecke();
    //}else if(strecke[0] < strecke[2])){ // 0Grad kleiner 90Grad
    //  drehung90();
    //  fahrenStrecke();
    //}


  }

}

void displayDistance()
{
  tft.setTextDatum(MC_DATUM);  // middle centered

  String distanceString =  String(sensor[0].ranging_data.range_mm);
  distanceString = String(distanceString + "mm");

  tft.fillRect(0, 0, 135, 200 - (sensor[0].ranging_data.range_mm / 10), TFT_BLACK);
  tft.fillRect(0, 200 - (sensor[0].ranging_data.range_mm / 10), 135, 200, TFT_SKYBLUE);

  if (sensor[0].ranging_data.range_status == VL53L1X::RangeValid)
  {
    tft.setTextColor(TFT_SKYBLUE);
  }
  else
  {
    tft.setTextColor(TFT_RED);
  }
  tft.fillRect(0, 200, 135, 240, TFT_BLACK);
  tft.drawString(distanceString, 80, 220, 4);
}

void showVoltage()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000) {
    timeStamp = millis();
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    String voltage = "Voltage :" + String(battery_voltage) + "V";
    Serial.println(voltage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage,  tft.width() / 2, tft.height() / 2 );
  }
}
