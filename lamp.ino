#include "FastLED.h"

#define MODE_PIN 0
#define DATA_PIN 5
#define NUM_LEDS 11
#define ANALOG_THRESHOLD 1000
#define MAX_BRIGHTNESS 255

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

int modePressLoopCount = 0;
OnOff previousModeButtonValue = OnOff::ON;
Mode currentMode = Mode::CarbonArc;

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting");

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  FastLED.setBrightness(100);
}

void loop()
{
  OnOff modeButtonValue = readMode();

  switch (currentMode)
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

  previousModeButtonValue = modeButtonValue;

  FastLED.show();
  delay(300);
}

OnOff readMode()
{
  const int requiredBrightnessLoops = 3;

  OnOff modeButtonValue = analogRead(MODE_PIN) < ANALOG_THRESHOLD
                              ? OnOff::OFF
                              : OnOff::ON;

  if (modeButtonValue == OnOff::OFF)
  {
    if (previousModeButtonValue == OnOff::ON && modePressLoopCount <= requiredBrightnessLoops)
    {
      currentMode = (Mode)((((int)currentMode) + 1) % 4);
      Serial.print("Mode switched to ");
      Serial.println(currentMode);
    }

    modePressLoopCount = 0;
  }
  else if (modeButtonValue == OnOff::ON)
  {
    // if the button has been held down long enough, start changing the brightness in increments
    if (++modePressLoopCount > 3)
    {
      FastLED.setBrightness((FastLED.getBrightness() + 25) % MAX_BRIGHTNESS);
      Serial.print("Changed brightness to: ");
      Serial.println(FastLED.getBrightness());
    }
  }

  return modeButtonValue;
}
