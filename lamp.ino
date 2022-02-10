#include "FastLED.h"

#define MODE_PIN 0
#define DATA_PIN 5
#define NUM_LEDS 11
#define ANALOG_THRESHOLD 1000
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS_LOOPS 3

CRGB leds[NUM_LEDS];

enum Mode
{
  Tungsten40 = 0,
  CarbonArc = 1,
  DirectSunlight = 2,
  Rainbow = 3,
};

enum OnOff
{
  ON,
  OFF,
};

uint8_t modePressLoopCount = 0;
OnOff prevBtnState = OnOff::OFF;
Mode curMode = Mode::CarbonArc;

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting");

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  FastLED.setBrightness(100);
}

void loop()
{
  readMode();

  displayMode();

  delay(300);
}

void readMode()
{
  OnOff curBtnState = analogRead(MODE_PIN) < ANALOG_THRESHOLD
                          ? OnOff::OFF
                          : OnOff::ON;

  if (curBtnState == OnOff::OFF)
  {
    // only change mode when the button is quick-pressed.
    // i.e. transitioning to OFF from a brightness change should not change mode.
    if (prevBtnState == OnOff::ON && modePressLoopCount <= MIN_BRIGHTNESS_LOOPS)
    {
      curMode = (Mode)((((int)curMode) + 1) % 4);

      Serial.print("Mode switched to ");
      Serial.println(curMode);
    }

    modePressLoopCount = 0;
  }
  else
  {
    // if the button has been held down long enough, start changing the brightness in increments
    if (++modePressLoopCount > 3)
    {
      FastLED.setBrightness((FastLED.getBrightness() + 25) % MAX_BRIGHTNESS);

      Serial.print("Changed brightness to: ");
      Serial.println(FastLED.getBrightness());
    }
  }

  prevBtnState = curBtnState;
}

void displayMode()
{
  switch (curMode)
  {
  case Mode::Tungsten40:
    FastLED.setTemperature(ColorTemperature::Tungsten40W);
    fill_solid(leds, NUM_LEDS, CRGB::White);
    break;
  case Mode::CarbonArc:
    FastLED.setTemperature(ColorTemperature::CarbonArc);
    fill_solid(leds, NUM_LEDS, CRGB::White);
    break;
  case Mode::DirectSunlight:
    FastLED.setTemperature(ColorTemperature::DirectSunlight);
    fill_solid(leds, NUM_LEDS, CRGB::White);
    break;
  case Mode::Rainbow:
    FastLED.setTemperature(ColorTemperature::DirectSunlight);
    fill_rainbow(leds, NUM_LEDS, 0, 255 / NUM_LEDS);
    break;
  }

  FastLED.show();
}
