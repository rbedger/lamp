#include "FastLED.h"

// LED specific
#define DATA_PIN 5
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define DEFAULT_BRIGHTNESS 255

#define NUM_LEDS 11

CRGB leds[NUM_LEDS];
int counter = 0;
CRGB colors[NUM_LEDS] = {
    CRGB::AliceBlue,
    CRGB::Amethyst,
    CRGB::AntiqueWhite,
    CRGB::Aqua,
    CRGB::Aquamarine,
    CRGB::Azure,
    CRGB::Beige,
    CRGB::Bisque,
    CRGB::Black,
    CRGB::BlanchedAlmond,
    CRGB::Blue,
};

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

  for (int i =0; i < NUM_LEDS; i++) {
    leds[i] = colors[(counter + i) % NUM_LEDS];
  }

  FastLED.show();
  delay(300);

  if (++counter >= NUM_LEDS)
  {
    counter = 0;
  }
}
