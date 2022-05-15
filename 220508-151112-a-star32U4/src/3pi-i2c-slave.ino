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

void setup()
{
  display.gotoXY(0, 1);
  display.print(F("Init"));
  Serial.begin(115200);
  delay(2000);
  Serial.println("");
  Serial.println("3pi+ I2C slave v1.0");
  Serial.println("h_da (c) 2022");
  Serial.println("--------------------");
  // I2C-Adresszuweisung: Slave 5
  Serial.print("starting I2C slave address 0x05...");
  Wire.begin(5);
  Wire.setClock(400000); // use 400 kHz I2C
  Serial.println("done.");

  // Handler für das I2C-Empfangsereignis festlegen
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.print("calibrating bump sensors...");
  bumpSensors.calibrate();
  Serial.println("done.");
  Serial.print("waiting for I2C master");
  // Code von Herr Lübbers Ende
  bumpSensors.calibrate();
  display.clear();
  display.gotoXY(0, 0);
  display.print("Press");
  display.gotoXY(0, 1);
  display.print(F("Button"));
  // encoders.init();
  // imu.init();
}
// Fahrtrichtungen Geschwindigkeitsfunktionen
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
  motors.setSpeeds(-50, 50);
}
void driveTurnright()
{
  motors.setSpeeds(100, -100);
}

// Roboter über Bump Sensoren fahren lassen.
void bumpDrive()
{

  unsigned long stopDelay = 100;
  unsigned long turnDelay = 500;
  unsigned long backwardDelay = 200;
  unsigned long countTurnDelay = 1000;
  unsigned long countBackwardDelay = 200;

  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Bump"));
  display.gotoXY(0, 1);
  display.print(F("Drive"));
  driveForward();
  // imu.configureForTurnSensing();
  bumpSensors.read();

  if (count >= 5)
  {
    count = 0;
    driveBackward();
    delay(countBackwardDelay);
    driveTurnright();
    delay(countTurnDelay);
    // driveStop();
    // delay(stopDelay);
    driveForward();
  }
  if (bumpSensors.leftIsPressed())
  { /*
     if (bumpSensors.leftIsPressed())
     {
       */
    driveBackward();
    delay(backwardDelay);
    driveTurnright();
    delay(turnDelay);
    /*
    imu.read();
    angle=imu.g.x;
    if(angle==90){
      driveStop();
      delay(20);
      driveForward();
    }
    */
    // driveStop();
    // delay(stopDelay);
    driveForward();
    count++;
    /*}
    else{
      driveForward();
    }*/
  }

  if (bumpSensors.rightIsPressed())
  { /*
     if (bumpSensors.rightIsPressed())
     {
       */
    driveBackward();
    delay(backwardDelay);
    // driveStop();
    // delay(stopDelay);
    driveTurnleft();
    delay(turnDelay);
    /*
    imu.read();
    angle=imu.g.x;
    if(angle==-90){
      driveStop();
      delay(20);
      driveForward();
    }

    driveStop();
    delay(stopDelay);
    */
    driveForward();
    count++;
    /*}
    else{
      driveForward();
    }*/
  }

  if (bumpSensors.rightIsPressed() && bumpSensors.leftIsPressed())
  {
    /*
   if (bumpSensors.rightIsPressed() && bumpSensors.leftIsPressed())
    {
      */
    driveBackward();
    delay(backwardDelay);
    driveTurnleft();
    delay(turnDelay);
    /*imu.read();
    angle=imu.g.x;
     if(angle==90){
      driveStop();
      delay(20);
      driveForward();
    }
    */
    /*
    driveStop();
    delay(stopDelay);
    */
    driveForward();
    count++;
    /*
    else{
      driveForward();
    }
    */
  }
}
void mainDisplay()
{
  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Press"));
  display.gotoXY(0, 1);
  display.print(F("Button"));
}
// Klasse Drehen für Drehen auf bestimmte Winkel gedacht.
void turning()
{
  display.clear();
  display.gotoXY(0, 1);
  display.print(F("turning"));
  unsigned long turningDelay = 100;
  driveTurnleft();
  delay(turningDelay);
  driveStop();
}

void initIntertialSensors()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
}

void encoderTurn()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  uint16_t lastUpdateTime = millis() - 100;
  char buf[4];
  driveTurnleft();
  while (!buttonC.getSingleDebouncedRelease())
  {
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
    // Zeitliches Update.
    // if((uint16_t)(millis()-lastUpdateTime)>5){
    lastUpdateTime = millis();
    display.clear();
    display.gotoXY(0, 0);
    sprintf(buf, "%03d", encCountsLeft);
    display.print(buf);
    display.gotoXY(0, 1);
    sprintf(buf, "%03d", encCountsRight);
    display.print(buf);
    display.gotoXY(5, 0);
    display.print("L");
    display.gotoXY(5, 1);
    display.print("R");
    // mit Updatrhythmus 5ms
    // 90° Drehwinkel
    // Turning Speeds (-100, 100)
    // 185 zu kurz 5 Sprünge bis über 360°
    // 200 zu kurz 5 Sprünge bis über 360°
    // 220 trifft manchmal, aber nicht dauerhaft
    // 225 trifft öfter aber nicht dauerhaft.
    // Turning Speeds(-75,75)
    // 225 zu kurz
    // 235 zu weit drüber
    // 230 leicht zu kurz bei mehreren Umdrehungen
    // 232 leicht drunter über die Zeit erhöht sich
    // der Abstand
    // 233 leicht drüber, bleibt aber mehr im Rahmen.
    //  Genausetes über den Modus
    // Turning Speeds(-50,50)
    // 233 drunter und wird über die Zeit mehr
    // 235 drunter und wird über die Zeit mehr
    // 237 kommt gut hin. Versuch mit 236
    // 236 wird von mal zu mal immer kleiner.
    // Wenn die Messung, wie weit gedreht wurde
    // hiermit stattfindet, eignet sich Speed-50,50
    // und 237 am besten.
    // Leichter Drift in eine Richtung
    // 20° Drehwinkel
    // 30 zu gering.
    // 50 zu groß
    // 240 auf beiden Seiten für 90° Winkel
    // mit Geschwindigkeit -50,50 und keinem
    // Updateintervall.
    if (encCountsLeft < -240 || encCountsRight > 240)
    {
      driveStop();
      encCountsLeft = 0;
      encCountsRight = 0;
      delay(5000);

      driveTurnleft();
    }
    else
    {
      driveTurnleft();
    }
  }
}
//}

void driveturn90()
{
  uint16_t encCountsLeft = 0, encCountsRight = 0;
  
  char buf[4];
  
  //Anfangsreset 
  encCountsLeft=encoders.getCountsAndResetLeft();
  encCountsRight=encoders.getCountsAndResetRight();
  encCountsLeft=0;
  encCountsRight=0;

  driveStop();


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
}

void bumpDrive90(){
  unsigned long stopDelay = 500;
  unsigned long turnDelay = 500;
  unsigned long backwardDelay = 50;
  unsigned long countTurnDelay = 1000;
  unsigned long countBackwardDelay = 200;
  
  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Bump"));
  display.gotoXY(0, 1);
  display.print(F("Drive"));
  driveForward();
  // imu.configureForTurnSensing();
  //bumpSensors.calibrate();

  while(!buttonC.getSingleDebouncedPress()){
    bumpSensors.read();
    driveForward();
    
    if(bumpSensors.leftIsPressed()){
      driveBackward();
      //delay(backwardDelay);
      driveturn90();
      delay(turnDelay);
      driveForward();
      return;
    }
    if(bumpSensors.rightIsPressed()){
       driveBackward();
      //delay(backwardDelay);
      driveturn90();
      delay(turnDelay);
      driveForward();
      return;
    }
    if(bumpSensors.rightIsPressed()&&bumpSensors.leftIsPressed()){
      driveBackward();
      //delay(stopDelay);
      driveturn90();
      delay(turnDelay);
      driveForward();
      return;
    }
   
  }
}

void loop()
{
  Serial.print(".");
  delay(1000);
  bool stateButtonA = 0;
  bool stateButtonB = 0;
  bool stateButtonC = 0;
  mainDisplay();

  if (buttonB.getSingleDebouncedRelease())
  {
    stateButtonB = 1;
  }
  if (buttonA.getSingleDebouncedRelease())
  {
    stateButtonA = 1;
  }
  if (buttonC.getSingleDebouncedRelease())
  {
    stateButtonA = 0;
    stateButtonB = 0;
    mainDisplay();
  }
  while (stateButtonA == 1)
  {
    bumpDrive90();

    if (buttonC.getSingleDebouncedPress())
    {
      stateButtonA = 0;
      motors.setSpeeds(0, 0);
      mainDisplay();
    }
  }
  while (stateButtonB == 1)
  {
    encoderTurn();
    if (buttonC.getSingleDebouncedRelease())
    {
      stateButtonB = 0;
      mainDisplay();
    }
  }
}

// Code von Herr Lübbers Start
void receiveEvent(int howMany)
{
  Serial.println("");
  while (Wire.available())
  {
    Serial.print("I2C command received: ");
    uint8_t command = Wire.read();
    if (command == 1)
    { // set motor speeds, 3 bytes, format: [1, left, right]
      uint8_t motorL = Wire.read();
      uint8_t motorR = Wire.read();
      motors.setSpeeds(motorL, motorR);
      Serial.println("motor.");
    }
    if (command == 0)
    { // stop robot, 1 byte, format: [0]
      motors.setSpeeds(0, 0);
      Serial.println("stop.");
    }
    if (command == 10)
    { // play music: 1 byte, format: [10]
      buzzer.playFromProgramSpace(starwars);
      Serial.println("sound.");
    }
  }
  Serial.print("waiting for I2C master");
}

void requestEvent()
{ // upon request send out all sensor data
  Serial.println("");
  Serial.print("I2C request received: ");

  if (sendID)
  { // first output is robot ID, all others are sensor data
    Wire.write("3pi+ I2Cslave v1.0");
    sendID = false;
    Serial.println("ID sent.");
  }
  else
  {
    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    bumpSensors.read();

    uint8_t data = 0; // 0 = no bump sensor pressed

    if (bumpSensors.leftIsPressed())
    {
      data = data & 0xF0; // 4 msb set to 1
    }
    if (bumpSensors.rightIsPressed())
    {
      data = data & 0x0F; // 4 lsb set to 1
    }
    Wire.write(data);
    Serial.println("data sent.");
  }
  Serial.print("waiting for I2C master");
}
// Code von Herr Lübbers Ende