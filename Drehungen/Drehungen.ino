

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;
ButtonA buttonB;
ButtonC buttonC;

// LCD display;
OLED display;

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

char report[80];

void setup()
{
  // Uncomment the following lines for the Hyper edition; its wheels
  // spin in the opposite direction relative to the encoders.
  // encoders.flipEncoders(true);
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);
}

void loop()
{
  static uint8_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;

  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    if (encoders.checkErrorLeft())
    {
      // An error occurred on the left encoder channel.
      // Display it for the next 10 iterations and also beep.
      displayErrorLeftCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorLeft);
    }

    if (encoders.checkErrorRight())
    {
      // An error occurred on the left encoder channel.
      // Display for the next 10 iterations and also beep.
      displayErrorRightCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorRight);
    }

    // Update the screen with encoder counts and error info.
    display.noAutoDisplay();
    display.clear();
    display.print(countsLeft);
    display.gotoXY(0, 1);
    display.print(countsRight);
    if (displayErrorLeftCountdown)
    {
      // Show an exclamation point on the first line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 0);
      display.print('!');
      displayErrorLeftCountdown--;
    }
    if (displayErrorRightCountdown)
    {
      // Show an exclamation point on the second line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 1);
      display.print('!');
      displayErrorRightCountdown--;
    }
    display.display();

    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("%6d %6d %1d %1d"),
        countsLeft, countsRight, errorLeft, errorRight);
    Serial.println(report);
  }

  if (buttonA.isPressed())
  {
    // drehung
    display.print('drehen');

    //motors.setSpeeds(400, 400);
    for(int i = 0; i < 10; i++)
    {
    motors.setSpeeds(-80, 80);
    delay(10);
    }
    
  }
  else if (buttonB.isPressed())
  {

    // drehung 90 Grad links
    uint16_t encoderlinks = 0;
    uint16_t encoderrechts = 0;
    encoderlinks = encoders.getCountsAndResetLeft();
    encoderrechts = encoders.getCountsAndResetRight();

    while( ( encoderrechts - encoderlinks) > 480)
    {
    motors.setSpeeds(-80, 80);
    delay(200);
    }
    
// Umrechnung in andere Grad zahlen sind aber nicht allle genau

//    // drehung 45 Grad links
//    uint16_t encoderlinks = 0;
//    uint16_t encoderrechts = 0;
//    encoderlinks = encoders.getCountsAndResetLeft();
//    encoderrechts = encoders.getCountsAndResetRight();
//
//    while( ( encoderrechts - encoderlinks) > 120)
//    {
//    motors.setSpeeds(-80, 80);
//    delay(200);    

//    // drehung 180 Grad links
//    uint16_t encoderlinks = 0;
//    uint16_t encoderrechts = 0;
//    encoderlinks = encoders.getCountsAndResetLeft();
//    encoderrechts = encoders.getCountsAndResetRight();
//
//    while( ( encoderrechts - encoderlinks) > 960)
//    {
//    motors.setSpeeds(-80, 80);
//    delay(200);

//    // drehung 360 Grad links
//    uint16_t encoderlinks = 0;
//    uint16_t encoderrechts = 0;
//    encoderlinks = encoders.getCountsAndResetLeft();
//    encoderrechts = encoders.getCountsAndResetRight();
//
//    while( ( encoderrechts - encoderlinks) > 1950)
//    {
//    motors.setSpeeds(-80, 80);
//    delay(200);



//    //180 Grad drehung
//    display.clear();
//    display.print("DrehenLinks");
//    for (int i = 0; i < 20; i++)
//    {
//      motors.setSpeeds(-100, 100);
//      delay(20);
//    }

    
  }
    else if (buttonC.isPressed())
  {
    //rückwärts fahren, solange taste C gedrückt
    motors.setSpeeds(-400, -400);
  }
  else
  {
    motors.setSpeeds(0, 0);
  }
}
