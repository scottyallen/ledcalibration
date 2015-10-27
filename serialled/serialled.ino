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
    char line[4];
    int length = Serial.readBytesUntil('\n', line, 4);
    if (length != 4) {
//      Serial.println("bad frame - skipping");
      continue;
    }
    int ledNum = line[0];
    int hue = line[1];
    int saturation = line[2];
    int value = line[3];
//    Serial.print("led: ");
//    Serial.print(ledNum);
//    Serial.print(" hue: ");
//    Serial.print(hue);
//    Serial.print(" saturation: ");
//    Serial.print(saturation);
//    Serial.print(" value: ");
//    Serial.print(value);
//    Serial.println();
    
    leds[ledNum] = CHSV(hue, saturation, value);
    FastLED.show();
  }
}
