
// This program shows how to read the encoders on the 3pi+ 32U4.
// The encoders can tell you how far, and in which direction each
// motor has turned.
//
// You can press button A on the 3pi+ to drive both motors
// forward at full speed.  You can press button C to drive both
// motors in reverse at full speed.
//
// Encoder counts are printed to the LCD/OLED screen and to the
// serial monitor.
//
// On the screen, the top line shows the counts from the left
// encoder, and the bottom line shows the counts from the right
// encoder.  Encoder errors should not happen, but if one does
// happen then the buzzer will beep and an exclamation mark will
// appear temporarily on the screen.
//
// In the serial monitor, the first and second numbers represent
// counts from the left and right encoders, respectively.  The
// third and fourth numbers represent errors from the left and
// right encoders, respectively.

#include <Arduino.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <Wire.h>

using namespace Pololu3piPlus32U4;
LCD display;

Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
BumpSensors bumpSensors;
Motors motors;
Encoders encoders;
IMU imu;

// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;


// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}



void setup()
{
  // Uncomment the following lines for the Hyper edition; its wheels
  // spin in the opposite direction relative to the encoders.
  // encoders.flipEncoders(true);
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
}

void loop()
{

  //ledYellow(false);
  //ledRed(false);

  if (buttonA.getSingleDebouncedRelease()) // Rechtsrum Drehen
  {
    display.clear();
    //display.print("DrehenLinks");

    for (int i = 0; i < 2000; i++)
    {
      motors.flipLeftMotor(-100);
      motors.flipRightMotor(100);
      display.gotoXY(1, 0);
      display.print(encoders.getCountsLeft());
      display.gotoXY(1, 1);
      display.print(encoders.getCountsRight());
      motors.setSpeeds(-100, 100);
      //if(encoders.getCountsRight() <= -450 && encoders.getCountsLeft() >= 450){
        if((encoders.getCountsLeft() - encoders.getCountsRight()) >= 1900){
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
        break;
      }
      delay(1);
    }
  }
  else if (buttonB.getSingleDebouncedRelease()) // Linksrum Drehen
  {

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));
  }
  display.clear();

  
  }
  else if (buttonC.getSingleDebouncedRelease()) // Fahre vorwärts, auser bumpSensor aktiv, dann stehen bleiben
  {
    display.clear();
    display.print("Fahren");

    for (int i = 0; i < 40; i++)
    {
      bumpSensors.read();
      motors.setSpeeds(100, 100);

      // Linker BumpSensor
      if (bumpSensors.leftIsPressed())
      {
        // Left bump sensor was just pressed.
        ledYellow(bumpSensors.leftIsPressed());
        buzzer.play("a32");
        display.gotoXY(0, 1);
        display.print('L');
        motors.setSpeeds(0, 0);
        break;
      }

      if (bumpSensors.rightIsPressed())
      {
        // Right bump sensor was just pressed.
        ledRed(bumpSensors.rightIsPressed());
        buzzer.play("e32");
        display.gotoXY(7, 0);
        display.print('R');
        motors.setSpeeds(0, 1);
        break;
      }

      delay(200);
    }
  }
  else
  {
    motors.setSpeeds(0, 0);
  }
}
