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

bool sendID = true; // if true, output is ID of robot, if flase, output is sensor data

// Code von Herr Lübbers Ende 
const int RoboterAdr=5;

int test=0;
bool DriveTest;
bool StopTest;

// Fahrtrichtungen Geschwindigkeitsfunktionen
// Setzen wir hier zuvor fest, um weniger Arbeit zu haben. 
void driveForward()
{
  motors.setSpeeds(100, 100);
}
void driveBackward()
{
  motors.setSpeeds(-100, -100);
}
void driveStop()
{
  motors.setSpeeds(0, 0);
}
void driveTurnleft()
{
  motors.setSpeeds(-50,50);
}
void driveTurnright()
{
  motors.setSpeeds(150, -150);
}

// 20° Drehung des Roboters zum erfassen der Werte
void driveturn20()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  char buf[4];
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  if(encoders.checkErrorLeft()||encoders.checkErrorRight()){
    return;
  }
  
 // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl 
 //encCountsLeft >-240 || encCountsRight <240 => 90°
 //encCountsLeft >-40  || encCountsRight <40  => 20° 

  while (encCountsLeft>-40||encCountsRight<40)
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
  encCountsLeft=0;
  encCountsRight=0;
  delay(1000);
}

// 15° Drehung des Roboters zum erfassen der Werte
void driveturn15()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  while (encCountsLeft>-30||encCountsRight<30)
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
  encCountsLeft=0;
  encCountsRight=0;
  delay(1000);
}

// 5° Drehung des Roboters zum erfassen der Werte
void driveturn5()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  while(((encCountsRight-encCountsLeft)/2)<9)
  {
    driveTurnleft();
    
    encCountsLeft += encoders.getCountsAndResetLeft();
    if (encCountsLeft < 0)
    {
      encCountsLeft += 10000;
    }
    if (encCountsRight > 9999)
    {
      encCountsLeft -= 10000;
    }

    encCountsRight += encoders.getCountsAndResetRight();
    if (encCountsRight < 0)
    {
      encCountsRight += 10000;
    }
    if (encCountsRight > 9999)
    {
      encCountsRight -= 10000;
    }

  }
  driveStop();
  encCountsLeft=0;
  encCountsRight=0;
  delay(1000);
}

// Drehung des Roboters um 90° nach links, wenn man vor einem Hindernis steht. 
void driveturn90()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  while (encCountsLeft>-240||encCountsRight<240)
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
  encCountsLeft=0;
  encCountsRight=0;
  delay(1000);
}

//Forwärtsfahren mit Encoder und übermittelten Werten.
void driveForwar(float howFar){
  float encCountsLeft = 0, encCountsRight = 0;

  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  while (encCountsLeft<howFar||encCountsRight<howFar)
  {
    float faktor;
    if(howFar<=10){
      faktor=0.77;
    }
    if(howFar>10&&howFar<30){
      faktor=0.8;
    }
    if(howFar>=30&&howFar<=110){
      faktor=1.05;
    }
    if(howFar>120&&howFar<=250){
      faktor=1.1;
    }
    if(howFar>250){
      faktor=1.12;
    }
   
    driveForward();
    // Umrechnung des Encoder Wertes in mm 
    encCountsLeft += encoders.getCountsAndResetLeft()/(3.1415*faktor);

    if (encCountsLeft < 0)
    {
      encCountsLeft += 10000;
    }
    if (encCountsRight > 9999)
    {
      encCountsLeft -= 10000;
    }
    //Umrechnung des Encoder Wertes in mm 
    encCountsRight += encoders.getCountsAndResetRight()/(3.1415*faktor);
    if (encCountsRight < 0)
    {
      encCountsRight += 10000;
    }
    if (encCountsRight > 9999)
    {
      encCountsRight -= 10000;
    }
    
  }

  //driveForward();
  //delay(1000);
  driveStop();
  encCountsLeft=0;
  encCountsRight=0;
  delay(1000);
}

int a1 =0;
char wert[4];

// Recieve- und RequesEvents zum erhalten und übermitteln von Daten. 
void receiveEvent(int howMany){
  int charcount = 0;

  while(1 < Wire.available()){
        
    char rxChar = Wire.read();
    // '10' wird nicht richtig übertragen / Empfangen => keine Aktion
//    if(rxChar==1){
//      driveturn20();
//    }
//    else if(rxChar==2){
//      driveturn90();
//    }
//    else if(rxChar==3){
//      driveturn15();
//    }
//    else if(rxChar==4){
//      driveturn5(); 
//    }
//    else if(rxChar==5){
//      DriveTest=1;
//    }
//    else if(rxChar==6){
//      driveTurnleft();
//    }
//    else if(rxChar==7){
//      DriveTest=0;
//    }
//    else 
    if(rxChar == '#'){  //Einführung der Fahrübertragung
      DriveTest=1;
    }
    else {
      DriveTest = 1;
      wert[charcount] = rxChar;
      charcount++;
//    a1 = atoi(wert);
    }
  }
}

//SendeEvent noch nicht eingefügt, um eine Rückgabe zu machen.
void requestEvent(){

}


//Initialisierung der Funktionen
void setup() {
  
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
  Wire.onReceive(receiveEvent);//Zum Empfangen
  Wire.onRequest(requestEvent);//Zum Senden

  Serial.print("calibrating bump sensors...");
  bumpSensors.calibrate();
  Serial.println("done.");
  Serial.print("waiting for I2C master");
  // Code von Herr Lübbers Ende
  bumpSensors.calibrate();
  
  encoders.init();

}

void loop() {
  //Vorwärts fahren über Encoder
  while(DriveTest==1){
    DriveTest=0;
   // driveForwar(3 * 10); //Hiermit fährt der Roboter nach vorne in mm 
    driveForwar( a1 ); //Hiermit fährt der Roboter nach vorne und bleibt 2cm vorm hindernis stehen
    Serial.print(wert);
    a1 = atoi(wert);
    Serial.print(a1);
    driveForwar( a1 );
  }
  
}
