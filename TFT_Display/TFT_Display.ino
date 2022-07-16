#include <TFT_eSPI.h> 
#include <SPI.h>

//#include "Button2.h"
//
//#define BUTTON_1            35
//#define BUTTON_2            0
//
//Button2 btn1(BUTTON_1);
//Button2 btn2(BUTTON_2);



TFT_eSPI tft;

void setup(void) {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  tft.setSwapBytes(true);
}

void loop() {


    // The standard ADAFruit font still works as before
    tft.setTextColor(TFT_BLACK);
    tft.setCursor (12, 5);
    tft.print("Original ADAfruit font!");

//    tft.setTextColor(TFT_BLACK, TFT_BLACK); // Do not plot the background colour

tft.setTextColor(TFT_SKYBLUE);

tft.drawString("---",37,72,4);
tft.setCursor (13, 1);
//tft.setTextColor(TFT_BLUE);
tft.setTextColor(TFT_RED);
tft.drawString("Test",10,1,4);

}
