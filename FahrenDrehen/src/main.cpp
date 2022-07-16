#include <Arduino.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>



using namespace Pololu3piPlus32U4;
LCD display;

Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
BumpSensors bumpSensors;
Motors motors;
Encoders encoders;

void setup()
{
  // put your setup code here, to run once:
  bumpSensors.calibrate();

  display.clear();
  display.gotoXY(0, 0);
  display.print("Start");
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Lampen aus
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
    display.print("DrehenRechts");
    for (int i = 0; i < 40; i++)
    {
      motors.setSpeeds(100, -100);
      encoders.getCountsRight();
      display.gotoXY(1, 1);
      display.print(encoders.getCountsRight());
      delay(20);
    }
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
        motors.setSpeeds(0, 0);
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