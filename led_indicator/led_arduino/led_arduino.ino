#include <Adafruit_NeoPixel.h>

#define PIN 9         //pin for led matrix data output
#define NUMPIXELS 61  // number of pixel in array


Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int ledColor = 0;        //variable for color mode: 0=OFF, 1=RED, 2=GREEN_FLASH, 3=BLUE
long LEDFlashTimer = 0;  // timer for green LED
long runTime = 0;

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(50);

  pixels.begin();  //initiate neopixel
}

void loop() {


  while (Serial.available() > 0) {
    if (Serial.read() == '#') {  //start byte

      ledColor = Serial.parseInt();  //variable int sets LED color

      if (Serial.read() == '%') {  //end byte
        Serial.print("#"+String(ledColor)+"%");



        if (ledColor == 0) {
          displayOFF();  //LED OFF
        } else if (ledColor == 1) {
          displayRED();  //LED SOLID RED
        } else if (ledColor == 2) {
          LEDFlashTimer = millis();  //reset flashtimer to current time
          displayGREENpulse();       //LED FLASHING GREEN @0.5Hz
        } else if (ledColor == 3) {
          displayBLUE();  //LED SOLID BLUE
        }
      }
    }
  }
  runTime = millis();  //set equal to current time
  if (ledColor == 2) {  //Flashing sequence for green LED
    if ((runTime - LEDFlashTimer) <= 1000) {
      displayGREENpulse();  //GREEN LED ON
    } else if ((runTime - LEDFlashTimer) <= 2000) {
      displayOFF();  //LED OFF
    } else if ((runTime - LEDFlashTimer) > 2000) {
      LEDFlashTimer = runTime;  //reset flash timer
    }
  }
  delay(10);
}

void displayRED() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
  }
}

void displayGREENpulse() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    pixels.show();
  }
}

void displayBLUE() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    pixels.show();
  }
}
void displayOFF() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
}