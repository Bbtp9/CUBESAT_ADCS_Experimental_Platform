#include <Adafruit_NeoPixel.h>

#define LED_PIN 8
#define NUMPIXELS 1

Adafruit_NeoPixel pixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {

  pixel.begin();

  // Turcoaz
  pixel.setPixelColor(0, pixel.Color(0, 255, 180));

  pixel.show();
}

void loop() {
}