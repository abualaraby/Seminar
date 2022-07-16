#include <Arduino.h>
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;
BumpSensors bumpSensors;
Buzzer buzzer;

OLED display;

void setup()
{
  bumpSensors.calibrate();
  display.clear();
  display.gotoXY(0, 1);
  display.print("Bump me!");
}

void loop()
{
  bumpSensors.read();

  if (bumpSensors.leftChanged())
  {
    ledYellow(bumpSensors.leftIsPressed());
    if (bumpSensors.leftIsPressed())
    {
      // linker bump Sensor ist getrückt (Ton und anzeigen)
      buzzer.play("a32");
      display.gotoXY(0, 0);
      display.print('L');
    }
    else
    {
      // linker bump Sensor ist nicht mehr getrückt (Ton und anzeigen weg)
      buzzer.play("b32");
      display.gotoXY(0, 0);
      display.print(' ');
    }
  }

  if (bumpSensors.rightChanged())
  {
    ledRed(bumpSensors.rightIsPressed());
    if (bumpSensors.rightIsPressed())
    {
      // rechter bump Sensor ist getrückt (Ton und anzeigen)
      buzzer.play("e32");
      display.gotoXY(7, 0);
      display.print('R');
    }
    else
    {
      // rechter bump Sensor ist nicht mehr getrückt (Ton und anzeigen weg)
      buzzer.play("f32");
      display.gotoXY(7, 0);
      display.print(' ');
    }
  }


}
