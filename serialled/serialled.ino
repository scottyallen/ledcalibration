#include "FastLED.h"
#define NUM_LEDS 49
#define DATA_PIN 3
CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<UCS1903B, DATA_PIN, RGB>(leds, NUM_LEDS);
  Serial.begin(115200);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void loop() {
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    int separator = line.indexOf(' ');
    int ledNum = line.substring(0, separator).toInt();
    int ledVal = line.substring(separator).toInt();
    //Serial.println(ledNum);
    //Serial.println(ledVal);
    //Serial.println();

    //for (int i = 0; i < NUM_LEDS; i++) {
      //leds[i] = CRGB::Black;
    //}
    leds[ledNum] = CHSV(255, 0, ledVal);
    FastLED.show();
  }
}
