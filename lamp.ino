#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
// Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                              BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                              BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper *err)
{
  Serial.println(err);
  while (1)
    ;
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t *data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

#include "FastLED.h"

#define MODE_PIN 5
#define DATA_PIN 5
#define NUM_LEDS 11
#define ANALOG_THRESHOLD 1000
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS_LOOPS 3

CRGB leds[NUM_LEDS];

enum Mode
{
  AllWhite = 0,
  xRainbow = 1,
  xSlowFade = 2,
  xSolidColor = 3
};

enum OnOff
{
  ON,
  OFF,
};

uint8_t modePressLoopCount = 0;
unsigned long lastModePressMillis = 0;

OnOff prevBtnState = OnOff::OFF;
Mode curMode = Mode::xRainbow;

CRGB solidColor = CRGB::Red;

OnOff prevBTLEState = OnOff::OFF;

void setup()
{
  while (!Serial)
    ; // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println("Booting");

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  FastLED.setBrightness(200);

  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE)
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset())
    {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false); // debug info is a little annoying after this point!
}

void loop()
{
  if (prevBTLEState == OnOff::OFF && ble.isConnected())
  {
    prevBTLEState = OnOff::ON;

    Serial.println(F("******************************"));

    // LED Activity command is only supported from 0.6.6
    if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
    {
      // Change Mode LED Activity
      Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    }

    // Set Bluefruit to DATA mode
    Serial.println(F("Switching to DATA mode!"));
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("******************************"));
  }
  readMode();

  displayMode();

  handleBluetooth();
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
      const unsigned long now = millis();
      const unsigned long elapsedMillis = now - lastModePressMillis;
      Serial.print("Elapsed: ");
      Serial.println(elapsedMillis);

      // debounce brightness change a bit
      if (elapsedMillis > 175)
      {
        FastLED.setBrightness((FastLED.getBrightness() + 25) % MAX_BRIGHTNESS);

        Serial.print("Changed brightness to: ");
        Serial.println(FastLED.getBrightness());

        lastModePressMillis = now;
      }
    }
  }

  prevBtnState = curBtnState;
}

void displayMode()
{
  switch (curMode)
  {
  case Mode::AllWhite:
    fill_solid(leds, NUM_LEDS, CRGB::White);
    break;
  case Mode::xRainbow:
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(random8(), 255, 255);
    }
    break;
  case Mode::xSlowFade:
    displaySlowFade();
    break;
  case Mode::xSolidColor:
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = solidColor;
    }
    break;
  }

  FastLED.show();
}

CRGB slowFadeColors[NUM_LEDS] = {
    CRGB::Red,
    CRGB::Violet,
    CRGB::OrangeRed,
    CRGB::Yellow,
    CRGB::Blue,
    CRGB::YellowGreen,
    CRGB::Teal,
    CRGB::Green,
    CRGB::Orange,
    CRGB::BlueViolet,
    CRGB::Indigo,
};

void displaySlowFade()
{
  const int factor = 1000;
  const unsigned long iteration = (factor + millis()) / factor;

  Serial.print("Iteration: ");
  Serial.println(iteration);

  for (int i = 0; i < NUM_LEDS; i++)
  {
    int currentColorIndex = (i + iteration) / NUM_LEDS;
    CRGB currentColor = slowFadeColors[currentColorIndex];
    CRGB nextColor = slowFadeColors[(currentColorIndex + 1) % NUM_LEDS];

    float blendAmount = (iteration % 100) / (float)100;
    CRGB blended = blend(currentColor, nextColor, blendAmount);

    if (i == 0)
    {
      Serial.println(currentColorIndex);
      Serial.println(blendAmount);
    }

    leds[i] = blended;
  }

  delay(1);
}

void handleBluetooth()
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0)
    return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C')
  {
    curMode = Mode::xSolidColor;

    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];

    Serial.print("RGB #");
    if (red < 0x10)
      Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10)
      Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10)
      Serial.print("0");
    Serial.println(blue, HEX);

    solidColor = CRGB(red, green, blue);
  }
}
