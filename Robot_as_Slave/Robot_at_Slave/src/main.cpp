#include <Arduino.h>

// Code von Herr Lübbers Start
#include <Pololu3piPlus32U4.h>
#include <Wire.h>
#include <Pololu3piPlus32U4IMU.h>
#include <PololuMenu.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
BumpSensors bumpSensors;
Buzzer buzzer;
Motors motors;
IMU imu;
// Code von Herr Lübbers Ende
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
uint8_t count = 0;
LCD display;

int16_t xValue = imu.g.x;
int16_t yValue = imu.g.y;
// Code von Herr Lübbers Start
const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";
const char tetris[] PROGMEM = "!T240 L8 MS >e<eb>c>d>e16>d16>cba<aa>c>e<a>d>cb<g+b>c>d<e>e<e>c<aarae<bc d>d<d>f>ad16d16>g>f>e<cr>c>e<g>d>cb<g+b>c>d<e>e<e>c<aa<aa";
const char starwars[] PROGMEM = "! T140 o4 v12 l4 ar16ar16ar16 f5r24>c12r32ar16 f5r24>c12r24 a2";
const char supermario[] PROGMEM = "v12 L16 o5 eererce8g8r8<g8r8";

bool sendID = true; // if true, output is ID of robot, if flase, output is sensor data

// Code von Herr Lübbers Ende
const int RoboterAdr = 5;

int test = 0;
uint8_t DriveTest=0;
bool StopTest;
bool wireReadCounter = 0;
bool wireFunktionCheck = 0;

// Fahren Internet
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long Period = 20;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

bool requestDrive = false;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0351;

float Sl = 0.0F;
float Sr = 0.0F;

void checkEncoders(float howFar)
{

  while (((Sl + Sr) / 2) < howFar)
  {
    
    if (requestDrive == true)
    {
      currentMillis = millis();
      if (currentMillis > prevMillis + Period)
      {
        countsLeft += encoders.getCountsAndResetLeft();
        countsRight += encoders.getCountsAndResetRight();

        Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
        Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

        int wheelSpeed = 75;

        if (Sr < 60 || Sl < 60)
        {
          if (Sr > 50 || Sl > 50)
          {
            wheelSpeed = 75 * ((30 - Sr) / 10);
            if (wheelSpeed < 40)
            {
              wheelSpeed = 40;
            }
          }

          motors.setSpeeds(wheelSpeed-5, wheelSpeed);         // Roboter komplett
          //motors.setSpeeds(wheelSpeed, wheelSpeed);         // Roboter 1 Sensor 
        }

        prevLeft = countsLeft;
        prevRight = countsRight;

        prevMillis = currentMillis;
      }
    }
    if(requestDrive==false){
      requestDrive=true;
      prevMillis=millis();
      prevLeft=0;
      prevRight=0;

    }
  }
  Sl = 0;
  Sr = 0;
  countsLeft = 0;
  countsRight = 0;
  motors.setSpeeds(0, 0);
  requestDrive=false;
}

// Fahrtrichtungen Geschwindigkeitsfunktionen
// Setzen wir hier zuvor fest, um weniger Arbeit zu haben.

void driveForward()
{
  motors.setSpeeds(43, 50); // stand 46,50 komplett ohne gewichte
}
void driveBackward()
{
  motors.setSpeeds(-50, -50);
}
void driveStop()
{
  motors.setSpeeds(0, 0);
}
void driveTurnleft()
{
  motors.setSpeeds(-50, 50);
}
void driveTurnright()
{

  motors.setSpeeds(150, -150);
}

void driveturn5()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;

  char buf[4];

  // Anfangsreset
  encCountsLeft = encoders.getCountsAndResetLeft();
  encCountsRight = encoders.getCountsAndResetRight();
  encCountsLeft = 0;
  encCountsRight = 0;

  // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl
  // encCountsLeft >-240 || encCountsRight <240 => 90°
  // encCountsLeft >-40  || encCountsRight <40  => 20°
  // while (encCountsLeft<1000||encCountsRight>-1000)


  while (((encCountsRight - encCountsLeft) / 2) < 8.85)


  {

    driveTurnleft();

    encCountsLeft += encoders.getCountsAndResetLeft();

    if (encCountsLeft < 0)
    {
      driveStop();
    }
    if (encCountsRight > 50)
    {
      driveStop();
    }

    encCountsRight += encoders.getCountsAndResetRight();
    /*
    if (encCountsRight < 0)
    {
      encCountsRight += 10000;
    }
    if (encCountsRight > 9999)
    {
      encCountsRight -= 10000;
    }
    */
    if (encoders.checkErrorLeft() == true || encoders.checkErrorRight() == true)
    {
      driveStop();
      encoders.init();
    }
  }
  driveStop();
  encCountsLeft = 0;
  encCountsRight = 0;
  // delay(50);
}

// Drehung des Roboters um 90° nach links, wenn man vor einem Hindernis steht.
void driveturn90()
{

  uint16_t encCountsLeft = 0, encCountsRight = 0;

  char buf[4];

  // Anfangsreset
  encCountsLeft = encoders.getCountsAndResetLeft();
  encCountsRight = encoders.getCountsAndResetRight();
  encCountsLeft = 0;
  encCountsRight = 0;

  // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl
  // encCountsLeft >-240 || encCountsRight <240 => 90°
  // encCountsLeft >-40  || encCountsRight <40  => 20°

  while (encCountsLeft > -240 || encCountsRight < 240)
  {

    driveTurnleft();

    encCountsLeft += encoders.getCountsAndResetLeft();
    if (encCountsLeft < 0)
    {
      encCountsLeft += 1000;
    }
    if (encCountsRight > 999)
    {
      encCountsLeft -= 1000;
    }

    encCountsRight += encoders.getCountsAndResetRight();
    if (encCountsRight < 0)
    {
      encCountsRight += 1000;
    }
    if (encCountsRight > 999)
    {
      encCountsRight -= 1000;
    }
  }
  driveStop();
  encCountsLeft = 0;
  encCountsRight = 0;
  delay(1000);
}

// Forwärtsfahren mit Encoder und übermittelten Werten.
void driveForwar(float howFar)
{
  float encCountsLeft = 0, encCountsRight = 0;
  encoders.init();

  char buf[4];

  // Anfangsreset
  encCountsLeft = encoders.getCountsAndResetLeft();
  encCountsRight = encoders.getCountsAndResetRight();
  encCountsLeft = 0;
  encCountsRight = 0;

  // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl
  // encCountsLeft >-240 || encCountsRight <240 => 90°
  // encCountsLeft >-40  || encCountsRight <40  => 20°
  if (wireFunktionCheck == wireReadCounter)
  {
    while (encCountsLeft < howFar || encCountsRight < howFar)
    {
      float faktor;
      if (howFar <= 10)
      {
        faktor = 0.77;
      }
      if (howFar > 10 && howFar < 30)
      {
        faktor = 0.8;
      }
      if (howFar >= 30 && howFar <= 110)
      {
        faktor = 1.05;
      }
      if (howFar > 120 && howFar <= 250)
      {
        faktor = 1.1;
      }
      if (howFar > 250)
      {
        faktor = 1.1;
      }

      driveForward();
      // Umrechnung des Encoder Wertes in mm
      encCountsLeft += encoders.getCountsAndResetLeft() / (3.1415 * faktor);

      if (encCountsLeft < 0)
      {
        encCountsLeft += 10000;
      }
      if (encCountsRight > 9999)
      {
        encCountsLeft -= 10000;
      }
      // Umrechnung des Encoder Wertes in mm
      encCountsRight += encoders.getCountsAndResetRight() / (3.1415 * faktor);
      if (encCountsRight < 0)
      {
        encCountsRight += 10000;
      }
      if (encCountsRight > 9999)
      {
        encCountsRight -= 10000;
      }

      bumpSensors.read();
      if (bumpSensors.rightIsPressed() || bumpSensors.leftIsPressed())
      {
        driveStop();
        return;
      }
    }
  }

  // driveForward();
  // delay(1000);
  driveStop();
  encCountsLeft = 0;
  encCountsRight = 0;
  delay(1000);
}

// Recieve- und RequesEvents zum erhalten und übermitteln von Daten.
void receiveEvent(int howMany)
{

  while (Wire.available())
  {
    char rxChar = Wire.read();

    // '10' wird nicht richtig übertragen / Empfangen => keine Aktion

    if (rxChar == 1)
    {
      driveturn5();
    }
    if (rxChar == 2)
    {
      //1cm Fahren
      DriveTest=1;
    }
    if (rxChar == 3)
    {
      //2cm Fahren
      DriveTest=2;
    }
    if (rxChar == 4)
    {
      //5cm Fahren
      DriveTest=3;
    }
   
    if (rxChar == 5)
    {
      //10cm Fahren 
      DriveTest=4;
    }
    if (rxChar == 6)
    {
      //20cm Fahren
      DriveTest = 5;
    }
    if(rxChar==7){
      //50cm Fahren
      DriveTest=6;
    }
    if(rxChar==8){
      //100cm Fahren
      DriveTest=7;
    }
    // rxChar=0;
    /*
    else{

      Serial.println(rxChar);
      // value[charCount++]=rxChar;
      driveForwar(value);
      charCount++; //Zum springen im Array um den Wert zu schreiben


    }
    */
  }
}

// SendeEvent noch nicht eingefügt, um eine Rückgabe zu machen.
void requestEvent()
{
}

// Initialisierung der Funktionen
void setup()
{

  Serial.begin(115200);
  delay(2000);
  Serial.println("");
  Serial.println("3pi+ I2C slave v1.0");
  Serial.println("h_da (c) 2022");
  Serial.println("--------------------");
  // I2C-Adresszuweisung: Slave 5
  Serial.print("starting I2C slave address 0x05...");
  Wire.begin(RoboterAdr);
  Wire.setClock(400000); // use 400 kHz I2C
  Serial.println("done.");

  // Handler für das I2C-Empfangsereignis festlegen
  Wire.onReceive(receiveEvent); // Zum Empfangen
  Wire.onRequest(requestEvent); // Zum Senden

  Serial.print("calibrating bump sensors...");
  bumpSensors.calibrate();
  Serial.println("done.");
  Serial.print("waiting for I2C master");
  // Code von Herr Lübbers Ende
  bumpSensors.calibrate();

  encoders.init();

  // imu.init();
}

void loop()
{
  if(DriveTest==2){
    //1cm Fahren
    checkEncoders(3);
    DriveTest=0;
  }
  if(DriveTest==3){
    //2cm Fahren
    checkEncoders(0.5);
    DriveTest=0;

  }
  if(DriveTest==4){
    //5cm Fahren
    checkEncoders(5);
    DriveTest=0;
  }

  if (DriveTest == 5)
  {
    //10 cm Fahren
    
    checkEncoders(9);

    DriveTest = 0;
    // driveForwar(300); // Hiermit fährt der Roboter nach vorne in mm
  }
  if(DriveTest==6){
    //20cm Fahren
    checkEncoders(18);
    DriveTest=0;

  }
  if(DriveTest==7){
    //50cm Fahren
    checkEncoders(53);
    DriveTest=0;
  }

  if(DriveTest==8){
    //Entfernung =100cm 
    checkEncoders(100);
    DriveTest=0;
  }

  

  // Forwärtsfahren über Sensoren (War nicht sehr gut)
  /*
  while(DriveTest==1){
    DriveTest=0;
    driveForward();
  }
  driveStop();
  */
}