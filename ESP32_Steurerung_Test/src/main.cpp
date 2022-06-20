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
int Buttoncount=0; //Test der Buttonfunktionen mit hilfe von Interrupts 

#define WIFI_NETWORK "Chris1"    // WIFI Name
#define WIFI_PASSWORD "tqor8485" // WIFI Password
#define WIFI_TIMEOUT 20000       // in milliseconds


/*  Definition für Button2.h
Im folgenden wird mit Interrupts mit esp32-hal-gpio gearbeitet.
#define BUTTON_1 35
#define BUTTON_2 0

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);
*/

uint8_t btn1 = 0;
uint8_t btn2 = 35;


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

void display_init()
{
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);
}

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

/* Gehört zur Bibiliothek Button2.h
void handler(Button2 &btn)
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
    Serial.print("click");
    Serial.print(" (");
    Serial.print(btn.getNumberOfClicks());
    Serial.println(")");
}
*/
/*
String sensorToString(int index)
{
    String output = "";
    bool isFirst = true;

    for (int n : values[index])
    {
        if (!isFirst)
            output += ";";

        output += n;
        isFirst = false;
    }

    return output;
}
*/
/////////////////////////
// The Code Below is From Prof
/////////////////////////
/* void displayDistance()
{
    tft.setTextDatum(MC_DATUM); // middle centered

    String distanceString = String(sensor[0].ranging_data.range_mm);
    distanceString = String(distanceString + "mm");

    tft.fillRect(0, 0, 135, 200 - (sensor[2].ranging_data.range_mm / 10), TFT_BLACK);
    tft.fillRect(0, 200 - (sensor[2].ranging_data.range_mm / 10), 135, 200, TFT_SKYBLUE);

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
} */

/* void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000)
    {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage, tft.width() / 2, tft.height() / 2);
    }
} */
/////////////////////////
// The Code Above is From Prof
/////////////////////////

//Sollen später ein Stoppen oder Programmwechsel ermöglichen 
/*
void UpCounter()
{
    Buttoncount++;
    tft.fillRect(0, 200, 125, 10, TFT_BLACK);
    tft.setCursor(0, 200);
    tft.setTextColor(TFT_GREEN);
    tft.print("Counter: ");
    tft.print(Buttoncount);
    while (!digitalRead(btn1))
    ;
}
//Down Counter Zählt aus irgendeinem Grund doppelt. 
void DownCounter()
{
    Buttoncount--;
    tft.fillRect(0, 200, 125, 10, TFT_BLACK);
    tft.setCursor(0, 200);
    tft.setTextColor(TFT_RED);
    tft.print("Counter: ");
    tft.print(Buttoncount);
    while (!digitalRead(btn2))
    ;
}
*/
// Drehbefehl an Roboter senden 
void Robotturn(){
    Wire.beginTransmission(Roboter_ID);
    Wire.write('1');
    Wire.endTransmission();
}
// Variable des Sensors 3 an Roboter senden (die zwei, weil von 0 aus gezählt wird)
void Robotsend3(){
    char value[5];
    Wire.beginTransmission(Roboter_ID);
    Wire.write(sprintf(value, "%04d",sensor[2].ranging_data.range_mm));
    Wire.endTransmission();
}

void setup()
{

    // put your setup code here, to run once:
    Serial.begin(115200); // start the serial
    display_init();
    connectToWiFi(); // connect to WIFI

    /////////////////////////
    // The Code Below is From Prof
    /////////////////////////
    /*
        tft.init();
        tft.setRotation(0);
        tft.fillScreen(TFT_WHITE);
        tft.setSwapBytes(true);
        tft.pushImage(0, 0, 135, 240, splash);
      */
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C
    initToFs();
    uint8_t saveValue[50]; // Speicher

    /*     Serial.print("req ID from 3pi+...");
        Wire.requestFrom(5, 10); // read ID from robot
        Serial.print("done. ID: "); */

    while (Wire.available())
    {
        char c = Wire.read();
        Serial.print(c);
    }

    Serial.println("");

    delay(500);

    /////////////////////////
    // The Code Above is From Prof
    /////////////////////////
    /*
    btn1.setDebounceTime(20);
    btn2.setDebounceTime(20);

    btn1.begin(BUTTON_1);
    btn2.begin(BUTTON_2);

    btn1.setClickHandler(handler);

      btn1.setLongClickHandler(handler);
      btn1.setDoubleClickHandler(handler);
      btn1.setTripleClickHandler(handler);

    btn2.setClickHandler(handler);

    btn2.setLongClickHandler(handler);
    btn2.setDoubleClickHandler(handler);
    btn2.setTripleClickHandler(handler);
    nach mehreren Versuchen sehe ich diese Bibiliothek als nicht geeignet an.
*/
    pinMode(btn1, PULLDOWN);
    pinMode(btn2, PULLDOWN);

    //attachInterrupt(digitalPinToInterrupt(btn1), UpCounter, HIGH);
    //attachInterrupt(digitalPinToInterrupt(btn2), DownCounter, HIGH);

    ThingSpeak.begin(client); // to initilize the thingspeak library
}

void loop()
{

    float saveValue[50];
    

    /////////////////////////
    // The Code Below is From Prof
    /////////////////////////
    // tft.fillRect(0,200,125,10,TFT_GREEN); //Testfeld um Bildschirm zu lokalisieren.
    /* Programm hängt sich so auf
    btn1.loop();
    btn2.loop();
    */
    /* Verwendung von Button2.h Bibiliothek scheint ungeeignet zu sein, da kein Interruptmode
    besteht oder man auf die Knopfdrücke warten muss bis etwas passiert. Abfrage über Interrupts
    wurde in der Bibilitohek nicht gefunden. Dadurch lässt sich kein Echtzeitfähiges Programm hiermit
    schreiben. Es wird folgend auf die esp32-hal-gpio.h bibiliothek zurückgegriffen.
    btn1.waitForClick();
    btn2.waitForClick();

     if(btn1.wasPressed()){
         counter++;
         tft.fillRect(0,200,125,10,TFT_BLACK);
         tft.setCursor(0,200);

         tft.print("Counter: ");
         tft.print(counter);
         Serial.print("Counter: ");
         Serial.print(counter);
         Serial.print("\t");

     }

     if(btn2.wasPressed()){
         counter--;
         tft.fillRect(0,200,125,10,TFT_BLACK);
         tft.setCursor(0,200);
         tft.print("Counter: ");
         tft.print(counter);
         Serial.print("Counter: ");
         Serial.print(counter);
         Serial.print("\t");


     }
     */

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

    // Messdaten Mitteln

    //  Serial.print("\tstatus: ");
    //  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
    //  Serial.print("\tpeak signal: ");
    //  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
    //  Serial.print("\tambient: ");
    //  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);
    //  Serial.println();
    //   displayDistance();
    //  showVoltage();

    /////////////////////////
    // The Code Above is From Prof
    /////////////////////////

    /*
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        for (int n : values[i])
        {
           Serial.print(n);
           Serial.print(" ;");
        }

        Serial.println("");
    }
    */
    /*
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            int newValue = sensor[i].read();

            if (values[i].size() >= VALUE_COUNT)
                values[i].pop_front();

            values[i].push_back(newValue);
        }
    */
    /*
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            ThingSpeak.setField(i + 1, sensorToString(i));
        }

        counter++; // we will increment the counter

        ThingSpeak.setField(7, counter);     // we will Add to the counter Channel
        ThingSpeak.setField(6, WiFi.RSSI()); // we will Add to the WiFi Channel

        ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY); // to push all of the data to thingspeak
    */
    /*
            ThingSpeak.setField(1, "");
           ThingSpeak.setField(2, "");
           ThingSpeak.setField(3, "");
           ThingSpeak.setField(4, "");
           ThingSpeak.setField(5, "");
           */

    ThingSpeak.setField(1, sensor[0].ranging_data.range_mm); // we will Add to the Sensor Außenlinkst to Channel 1
    ThingSpeak.setField(2, sensor[1].ranging_data.range_mm); // we will Add to the Sensor Innenlinkst to Channel 2
    ThingSpeak.setField(3, sensor[2].ranging_data.range_mm); // we will Add to the Sensor in der Mitte to Channel 3
    ThingSpeak.setField(4, sensor[3].ranging_data.range_mm); // we will Add to the Sensor InnenRechts to Channel 4
    ThingSpeak.setField(5, sensor[4].ranging_data.range_mm); // we will Add to the Sensor AußenRechts to Channel 5
    ThingSpeak.setField(7, counter);                         // we will Add to the counter Channel
    ThingSpeak.setField(6, WiFi.RSSI());                     // we will Add to the WiFi Channel
    ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);     // to push all of the data to thingspeak

    delay(500); // thingSpeak will allow one new data point every 15 seconds

    //Senden eines Signals zum Roboter
    if(digitalRead(btn1)==HIGH){
        Serial.print("BUTTON PRESSED");
        Robotturn();
        while(digitalRead(btn1)==HIGH);
    }
    //Senden der Sensordaten von Sensor 3 an Roboter. Muss später noch abgeändert werden um 
    //den Mittelwert zu senden. Kann sein, dass hierfür keine Funktion geschrieben werden kann,
    //da der Mittelwert erst in der Loop ermittelt und initialisiert wird. Muss man dementsprechend 
    //anpassen, indem man den Wert übergibt oder das Übertragen direkt hier hin schreibt. 
    Robotsend3();
    delay(100); //Delay kann später weggenommen werden ist jetzt bloß zum testen eingebracht.

}
