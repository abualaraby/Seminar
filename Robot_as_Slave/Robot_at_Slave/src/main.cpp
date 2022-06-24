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
const int RoboterAdr=5;

int test=0;


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

// 20° Drehung des Roboters zum erfassen der Werte
void driveturn15()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  char buf[4];
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  

 // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl 
 //encCountsLeft >-240 || encCountsRight <240 => 90°
 //encCountsLeft >-40  || encCountsRight <40  => 20° 

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

void driveturn5()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  char buf[4];
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  

 // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl 
 //encCountsLeft >-240 || encCountsRight <240 => 90°
 //encCountsLeft >-40  || encCountsRight <40  => 20° 
 //while (encCountsLeft<1000||encCountsRight>-1000)
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
  
  char buf[4];
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  

 // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl 
 //encCountsLeft >-240 || encCountsRight <240 => 90°
 //encCountsLeft >-40  || encCountsRight <40  => 20° 

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
void driveForwar(int howFar){
   uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  char buf[4];
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  

 // encCountsLeft und encCountsRight wert abändern für andere Grad Zahl 
 //encCountsLeft >-240 || encCountsRight <240 => 90°
 //encCountsLeft >-40  || encCountsRight <40  => 20° 

  while (encCountsLeft>howFar||encCountsRight>howFar)
  {
    
    driveForward();
    
   
    
  }
  driveStop();
  encCountsLeft=0;
  encCountsRight=0;
  delay(1000);
}


// Recieve- und RequesEvents zum erhalten und übermitteln von Daten. 
void receiveEvent(int howMany){
  int charCount=0;
  int value=100;
  while(Wire.available()){
    char rxChar=Wire.read();
    // '10' wird nicht richtig übertragen / Empfangen => keine Aktion
    if(rxChar=='1'){
      driveturn20();
    
    }
    if(rxChar=='2'){
      driveturn90();
    }
    if(rxChar=='5'){
      driveturn15();
     
    }
    if(rxChar=='7'){
      driveturn5();
      
    }
    
    else{
      Serial.println(rxChar);
      // value[charCount++]=rxChar;
      driveForwar(value);
      charCount++; //Zum springen im Array um den Wert zu schreiben
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

  // imu.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}