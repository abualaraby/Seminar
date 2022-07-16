#include <Pololu3piPlus32U4.h>
#include <Wire.h>
#include <Pololu3piPlus32U4IMU.h>
#include <PololuMenu.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
BumpSensors bumpSensors;
Buzzer buzzer;
Motors motors;

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";
const char tetris[] PROGMEM = "!T240 L8 MS >e<eb>c>d>e16>d16>cba<aa>c>e<a>d>cb<g+b>c>d<e>e<e>c<aarae<bc d>d<d>f>ad16d16>g>f>e<cr>c>e<g>d>cb<g+b>c>d<e>e<e>c<aa<aa";
const char starwars[] PROGMEM = "! T140 o4 v12 l4 ar16ar16ar16 f5r24>c12r32ar16 f5r24>c12r24 a2";
const char supermario[] PROGMEM =  "v12 L16 o5 eererce8g8r8<g8r8";

bool sendID = true; // if true, output is ID of robot, if flase, output is sensor data

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("");
  Serial.println("3pi+ I2C slave v1.0");
  Serial.println("h_da (c) 2022");
  Serial.println("--------------------");
  //I2C-Adresszuweisung: Slave 5
  Serial.print("starting I2C slave address 0x05...");
  Wire.begin(5);
  Wire.setClock(400000); // use 400 kHz I2C
  Serial.println("done.");

  //Handler für das I2C-Empfangsereignis festlegen
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.print("calibrating bump sensors...");
  bumpSensors.calibrate();
  Serial.println("done.");
  Serial.print("waiting for I2C master");
}

void loop() {

}

bool fahrenGesendeteStrecke = false;
int a1 = 0;
char wert[5];
void receiveEvent(int howMany) {

  int charcount = 0;

  Serial.println("");
  while (Wire.available())
  {
    //    Serial.print("I2C command received: ");
    //    uint8_t command = Wire.read();
    //    if (command == 1) { // set motor speeds, 3 bytes, format: [1, left, right]
    //      uint8_t motorL = Wire.read();
    //      uint8_t motorR = Wire.read();
    //      motors.setSpeeds(motorL, motorR);
    //      Serial.println("motor.");
    //    }
    //    if (command == 0) { // stop robot, 1 byte, format: [0]
    //      motors.setSpeeds(0,0);
    //      Serial.println("stop.");
    //    }
    //    if (command == 10) { // play music: 1 byte, format: [10]
    //      buzzer.playFromProgramSpace(starwars);
    //      Serial.println("sound.");
    //    }
    char rxChar = Wire.read();
    if (rxChar == 'a') { // wenn a gesendet wird drehung

      // drehung 45 Grad Drehung links fahren
      uint16_t encoderlinks = 0;
      uint16_t encoderrechts = 0;
      encoderlinks = encoders.getCountsAndResetLeft();
      encoderrechts = encoders.getCountsAndResetRight();

      while ( ( encoderrechts - encoderlinks) < 242)
      {
        motors.setSpeeds(-80, 80);
        delay(200);
      }
      motors.setSpeeds(0, 0);
    }

    if (rxChar == 'b') { // wenn a gesendet wird drehung

      // drehung 90 Grad Drehung links fahren
      uint16_t encoderlinks = 0;
      uint16_t encoderrechts = 0;
      encoderlinks = encoders.getCountsAndResetLeft();
      encoderrechts = encoders.getCountsAndResetRight();

      while ( ( encoderrechts - encoderlinks) < 480)
      {
        motors.setSpeeds(-80, 80);
        delay(200);
      }
      motors.setSpeeds(0, 0);
    }

    if (rxChar == 'c') { // vorwärtsfahren bis bumpSensors ausgelöst

      while ( (bumpSensors.leftIsPressed() && bumpSensors.rightIsPressed()) == false  )
      {
        bumpSensors.read();
        motors.setSpeeds(100, 100);
      }
      motors.setSpeeds(0, 0);
    }

    if (rxChar == '#') { //Einführung der Fahrübertragung
      fahrenGesendeteStrecke = true;
    }
    else {
      wert[charcount] = rxChar;
      charcount++;
      //    a1 = atoi(wert);
      //      if(rxChar == ':')){
      //        return;
      //      }
    }


  }

  if (fahrenGesendeteStrecke == true) {
    Serial.print(wert);
    char wertStr[2];
    for (int i = 0; i < 10; i++)
    {
      if (wert[i] == ':') {
        return;
      } else {
        wertStr[i] = wert[i];
      }

    }
    a1 = atoi(wertStr);

    //    // hier müsste jetzt die gesendete Stecke gefahren werde
    //    // funktioniert noch nicht richtig!
    //    uint16_t encoderlinks = 0;
    //    uint16_t encoderrechts = 0;
    //    encoderlinks += encoders.getCountsAndResetLeft();
    //    encoderrechts += encoders.getCountsAndResetRight();
    //    // Umfang in mm 2*pi*Radius entspricht 2*3.14159265359*17
    //
    //    while ( ( (encoderrechts/(2*3.14159265359*17) )- encoderlinks/(2*3.14159265359*17)) < (a1*2) )
    //    {
    //      motors.setSpeeds(100, 100);
    //      //delay(200);
    //    }
    //    motors.setSpeeds(0, 0);
  }

  //Serial.print("waiting for I2C master");
}

void requestEvent() { // upon request send out all sensor data
  Serial.println("");
  Serial.print("I2C request received: ");

  if (sendID) { // first output is robot ID, all others are sensor data
    Wire.write("3pi+ I2Cslave v1.0");
    sendID = false;
    Serial.println("ID sent.");
  }
  else {
    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    bumpSensors.read();

    uint8_t data = 0; // 0 = no bump sensor pressed

    if (bumpSensors.leftIsPressed()) {
      data = data & 0xF0; // 4 msb set to 1
    }
    if (bumpSensors.rightIsPressed()) {
      data = data & 0x0F; // 4 lsb set to 1
    }
    Wire.write(data);
    Serial.println("data sent.");
  }
  Serial.print("waiting for I2C master");
}
