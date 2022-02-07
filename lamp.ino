#include "FastLED.h"

// LED specific
#define DATA_PIN 5
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define DEFAULT_BRIGHTNESS 255

#define NUM_LEDS 1

CRGB leds[NUM_LEDS];

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting");

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  FastLED.setBrightness(DEFAULT_BRIGHTNESS);
}

void loop()
{
  Serial.println("loop");

  leds[0] = CRGB::White;
  FastLED.show();
  delay(300);

  leds[0] = CRGB::Red;
  FastLED.show();
  delay(300);
}
