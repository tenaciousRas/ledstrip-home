/*
  Created By:
  Free Beachler
  Longevity Software LLC
  Terawatt Industries 2015
  :::
  Conveyed to you under the Apache 2.0 license, because we LOVE open source software and hardware.  Enjoy!
  :::
  Description:
  :::
  This sketch provides a programmable wireless controller for a LED strip(s) using an Arduino and Bluetooth (or other serial).  Commands can be sent to a BT module
  attached to an Arduino to program the mode of the LED strip.  This can be used for home lighting strips, for example.  Programmable feature values are stored in EEPROM 
  so the LED strip starts in the last-known-state when it's turned on.
  
  This code supports multiple types of LED strips.  Initially WS2801,WS2811/12 strips are supported, through support graciously provided by the AdafruitWS2801 library.  Support
  for PWM-based single-color or RGB strips will be added.
  
  Programmable features include color, brightness, fade mode, animations, and more.
  
  Different features are available for addressable and non-addresable strips.
  :::
  Sparkfun WS2801 LED Strip :::
  You will need to connect 5V/Gnd from the Arduino (USB power seems to be sufficient).
  
  For the data pins, please pay attention to the arrow printed on the strip. You will need to connect to
  the end that is the begining of the arrows (data connection)--->
  
  If you have a 4-pin connection:
  Blue = 5V
  Red = SDI
  Green = CKI
  Black = GND
  
  If you have a split 5-pin connection:
  2-pin Red+Black = 5V/GND
  Green = CKI
  Red = SDI
  :::
 */
#include <EEPROM.h>
#include <SPI.h>
#include "Obi.h"
#include "SoftwareSerial.h"
#include "Adafruit_WS2801.h"
#include "ledstrip_home.h"

#define _VERSION 4  // it's probably more like v.100
#define DEBUG_MODE 1

Obi btModule = Obi(4, 6);

// LED Strip setup
// Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
#define NUM_LEDS 32
Adafruit_WS2801 strip = Adafruit_WS2801(NUM_LEDS);

#define OFF 0x000000
#define RED 0xFF0000
#define BLUE 0x0000FF
#define GREEN 0x00FF00
#define YELLOW 0xFFFF00
#define CYAN 0x00FFFF
#define VIOLET 0xFF00FF
#define WHITE 0xFFFFFF
#define DIM_RED 0x330000
#define DIM_BLUE 0x000033
#define DIM_GREEN 0x003300
#define DIM_YELLOW 0x333300
#define DIM_CYAN 0x003333
#define DIM_VIOLET 0x330033
#define DIM_WHITE 0x333333
#define TWYELLOW 0xFFEE00
#define TWBLUE 0x00A5D5

/**
 * display mode values:
 * 0: solid color
 * 1: fade terawatt colors
 * 2: fade random colors
 * 10: fade two colors
 * 11: fade three colors
 * 12: two alternating colors
 * 13: terawatt alternating colors
 * 14: three alternating colors
 * 15: two random alternating colors
 * 20: rainbow
 * 21: rainbow cycle
 * 22: random candy
 * 30: cylon
 */
#define DEFAULT_MULTI_COLOR_HOLD_TIME 5000;  // time to hold colors when showing multiple colors
#define DEFAULT_DISP_MODE 20;
#define DEFAULT_LED_FADE_MODE 2;
#define INITIAL_MULTI_COLOR_ALT_STATE 0;
#define DEFAULT_LED_STRIP_BRIGHTNESS 1.0;

led_strip_disp_state stripState;

unsigned long multiColorNextColorTime;
unsigned long ledColor[NUM_LEDS] = {TWYELLOW, TWBLUE, OFF};

#define LED_FADE_STEPS 16;
#define LED_FADE_STEP_DELAY_MS 50;  // microsecs between fade steps
unsigned long ledColorOld[NUM_LEDS] = {TWYELLOW, TWBLUE, OFF};  // color before fade
unsigned long ledColorFadeTo[NUM_LEDS] = {TWYELLOW, TWBLUE, OFF};    // fade-to color
unsigned long ledFadeStepTime;  // time to next fade step
int ledFadeStepIndex;  // color distance divided by LED_FADE_STEPS
double ledFadeStep[NUM_LEDS][3];
uint8_t  rainbowStepIndex = 0;

int eepromAddyStripState = 4;  // eeprom addy to store strip state

void setup() {
  initStripState();
  // set the data rate for bt device
  Serial.begin(9600);
#ifdef DEBUG_MODE
  Serial.print("startup\n");
#endif
  // send init data
  btModule.registerFunction(setLEDStripColor, 'c');
  btModule.registerFunction(setDispMode, 'm');
  btModule.registerFunction(setBright, 'b');
  btModule.registerFunction(setLedFadeMode, 'f');
  btModule.registerFunction(setLedFadeTimeInterval, 'i');
  btModule.registerFunction(setMultiColorHoldTime, 't');
  btModule.begin(9600);
  btModule.send("arduino led controller connected via bluetooth.\n");
#ifdef DEBUG_MODE
  Serial.print("configured bt\n");
#endif
  // read eeprom
  readStripState(&stripState);
  // setup LEDs
  setDispModeColors(stripState.dispMode);
  strip.begin();
  strip.show();
}

void loop() {
  btModule.receive(); // receive BT events
  if ((stripState.ledFadeMode == 0)
        || (stripState.ledFadeMode == 1 && !stripState.fading)) {
    renderPixels();
  }
  if (stripState.fading) {
    doFade();
    return;
  }
  switch(stripState.dispMode) {
    case 0:
      solidOneColor();
      break;
    case 1:
      solidTwoColors();
      break;
    case 2:
      solidTwoColors();
      break;
    case 10:
      solidTwoColors();
      break;
    case 11:
      solidThreeColors();
      break;
    case 12:
      alternatingTwoColors();
      break;
    case 13:
      alternatingTwoColors();
      break;
    case 14:
      alternatingThreeColors();
      break;
    case 15:
      alternatingTwoRandomColors();
      break;
    case 20:
      rainbow(1000);
      break;
    case 21:
      rainbowCycle(1000);
      break;
    case 22:
      randomCandy();
      break;
    case 30:
      break;
    default:
      break;
  }
}

void initStripState() {
  stripState.fading = false;
  stripState.dispMode = DEFAULT_DISP_MODE;
  stripState.ledFadeMode = DEFAULT_LED_FADE_MODE;
  stripState.multiColorAltState = 0;
  stripState.ledModeColor[0] = TWYELLOW;
  stripState.ledModeColor[1] = TWBLUE;
  stripState.ledModeColor[2] = OFF;
  stripState.multiColorHoldTime = DEFAULT_MULTI_COLOR_HOLD_TIME;
  stripState.fadeTimeInterval = LED_FADE_STEP_DELAY_MS;
  stripState.ledStripBrightness = DEFAULT_LED_STRIP_BRIGHTNESS;
}

void readStripState(led_strip_disp_state* ret) {
  unsigned int offset = eepromAddyStripState + sizeof(int);  // store int at addy 0
  byte bData;
  ret->fading = false;
  ret->multiColorAltState = INITIAL_MULTI_COLOR_ALT_STATE;
  bData = EEPROM.read(offset++);
  ret->dispMode = (uint8_t) bData;
  bData = EEPROM.read(offset++);
  ret->ledFadeMode = (uint8_t) bData;
  unsigned long color;
  for (int i = 0; i < 3; i++) {
    color = 0;
    for (int j = 3; j >= 0; j--) {
      bData = EEPROM.read(offset++);
      color += (bData << (8 * j));
    }
    ret->ledModeColor[i] = color;
  }
  unsigned long holdTime;
  holdTime = 0;
  for (int j = 3; j >= 0; j--) {
    bData = EEPROM.read(offset++);
    holdTime += (bData << (8 * j));
  }
  ret->multiColorHoldTime = holdTime;
  unsigned int storedLedStripBrightness;
  storedLedStripBrightness = 0;
  for (int j = 1; j >= 0; j--) {
    bData = EEPROM.read(offset++);
    storedLedStripBrightness += (bData << (8 * j));
  }
  ret->ledStripBrightness = (float) storedLedStripBrightness / (float) 1024;
#ifdef DEBUG_MODE
  Serial.print("multiColorHoldTime = ");
  Serial.print(ret->multiColorHoldTime);
  Serial.print("\nread eeprom done\n");
#endif
}

void putStripState(led_strip_disp_state* lsds) {
  led_strip_disp_state storedState;
  readStripState(&storedState);
  unsigned int offset = eepromAddyStripState + sizeof(int);  // store int at addy 0
  if ((unsigned int) storedState.dispMode != (unsigned int) lsds->dispMode) {
    EEPROM.write(offset, (byte) lsds->dispMode);
  }
  offset++;
  if (storedState.ledFadeMode != lsds->ledFadeMode) {
    EEPROM.write(offset, (byte) lsds->ledFadeMode);
  }
  offset++;
  for (int i = 0; i < 3; i++) {
    if (storedState.ledModeColor[i] != lsds->ledModeColor[i]) {
        for (int j = 3; j >= 0; j--) {
          EEPROM.write(offset++, (byte) ((lsds->ledModeColor[i] >> (8 * j)) & 0xFF));
        }
    } else {
      offset += 4;
    }
  }
  if (storedState.multiColorHoldTime != lsds->multiColorHoldTime) {
    for (int j = 3; j >= 0; j--) {
      EEPROM.write(offset++, (byte) ((lsds->multiColorHoldTime >> (8 * j)) & 0xFF));
    }
  } else {
    offset += 4;
  }
  if (storedState.ledStripBrightness != lsds->ledStripBrightness) {
    // store an int
    unsigned int lsb = lsds->ledStripBrightness * 1024;
    for (int j = 1; j >= 0; j--) {
      EEPROM.write(offset++, (byte) ((lsb >> (8 * j)) & 0xFF));
    }
  } else {
    offset += 2;
  }
#ifdef DEBUG_MODE
  Serial.print("write eeprom done\n");
#endif
}

void setDispModeColors(int mode) {
  switch(mode) {
    case 0:
      break;
    case 1:
      stripState.ledModeColor[0] = TWYELLOW;
      stripState.ledModeColor[1] = TWBLUE;
      break;
    case 2:
      stripState.ledModeColor[0] = random(0xFFFFFF);
      stripState.ledModeColor[1] = random(0xFFFFFF);
      break;
    case 10:
      break;
    case 11:
      break;
    case 12:
      break;
    case 13:
      stripState.ledModeColor[0] = TWYELLOW;
      stripState.ledModeColor[1] = TWBLUE;
      break;
    case 14:
      break;
    case 15:
      stripState.ledModeColor[0] = random(0xFFFFFF);
      stripState.ledModeColor[1] = random(0xFFFFFF);
      break;
    case 20:
      rainbowStepIndex = 0;
      break;
    case 21:
      rainbowStepIndex = 0;
      break;
    case 22:
      rainbowStepIndex = 0;
      break;
    case 30:
      break;
    default:
      break;
  }
  putStripState(&stripState);
}

void setLEDStripColor(byte flag, byte numOfValues) {
  char data[40];
  btModule.getString(data);
  // expect to always have two Ints from Android app
  uint8_t b1[2];
  uint8_t b2[38];
  b1[0] = data[0];
  b1[1] = '\0';
  int a = 2;
  for(;a < 40; a++){
    b2[a - 2] = data[a];
    if (b2[a - 2] == 0) {
      break;
    }
  }
  int colorIndex = atoi((char *) b1);
  unsigned long color = atol((char *) b2);
  color = (color << 8) >> 8;
  stripState.ledModeColor[colorIndex] = color;
  putStripState(&stripState);
#ifdef DEBUG_MODE
Serial.print("colorIndex = ");
Serial.print(colorIndex);
Serial.print(",");
Serial.print("color = ");
Serial.print(color);
Serial.print("\n");
Serial.println(data);
Serial.print("\n");
#endif
}

void setDispMode(byte flag, byte numOfValues) {
  char data[8];
  int bData;
  bData = btModule.getInt();
  stripState.dispMode = bData;
  setDispModeColors(stripState.dispMode);
  stripState.multiColorAltState = INITIAL_MULTI_COLOR_ALT_STATE;
  stripState.fading = false;
  putStripState(&stripState);
#ifdef DEBUG_MODE
Serial.print("dispMode = ");
Serial.print((int) bData);
Serial.print(", ");
Serial.print(stripState.dispMode);
Serial.print("\n");
#endif
}

void setBright(byte flag, byte numOfValues) {
  int bData;
  bData = btModule.getInt();
  if (1024 < bData) {
    bData = 1024;
  }
  stripState.ledStripBrightness = (float) bData / (float) 1024.0;
  putStripState(&stripState);
#ifdef DEBUG_MODE
Serial.print("ledStripBrightness = ");
Serial.print(bData);
//Serial.print(", ");
//Serial.print((float) stripState.ledStripBrightness);
Serial.print("\n");
#endif
}

void setLedFadeTimeInterval(byte flag, byte numOfValues) {
  unsigned long bData;
  bData = btModule.getLong();
  stripState.fadeTimeInterval = bData;
  putStripState(&stripState);
}

void setMultiColorHoldTime(byte flag, byte numOfValues) {
  unsigned long bData;
  bData = btModule.getInt();
  stripState.multiColorHoldTime = bData;
  putStripState(&stripState);
}

void setLedFadeMode(byte flag, byte numOfValues) {
  unsigned int bData;
  bData = btModule.getInt();
  if (11 < stripState.dispMode) {
    // only for solid colors
    return;
  }
  if (1 < bData) {
    bData = 0;
  }
  stripState.ledFadeMode = bData;
  putStripState(&stripState);
#ifdef DEBUG_MODE
Serial.print("ledFadeMode = ");
Serial.print(stripState.ledFadeMode);
Serial.print("\n");
#endif
}

void refreshLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    ledColor[i] = 0xFF0000;
  }
  colorWipe(50);
  for (int i = 0; i < NUM_LEDS; i++) {
    ledColor[i] = 0xFF00;
  }
  colorWipe(50);
  for (int i = 0; i < NUM_LEDS; i++) {
    ledColor[i] = 0xFF;
  }
  colorWipe(50);
}

void turnOffLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    ledColor[i] = 0x00;
  }
  colorWipe(10);
}

void white(float brightness) {
  if (brightness > 1.0) {
    brightness = 1.0;
  }
  unsigned int whiteness = (int) ((float) 0xFFFFFF * (float) brightness);
  for (int i = 0; i < NUM_LEDS; i++) {
    ledColor[i] = whiteness;
  }
  colorWipe(10);
}

void solidMultiColor(int numMultiColors) {
  if (multiColorNextColorTime - millis() > stripState.multiColorHoldTime) {
    multiColorNextColorTime = millis() + stripState.multiColorHoldTime;
    for (int x = (NUM_LEDS - 1) ; x >= 0 ; x--) {
      ledColorFadeTo[x] = stripState.ledModeColor[stripState.multiColorAltState];
    }
#ifdef DEBUG_MODE
Serial.print("multiColorAltState,ledModeColor[multiColorAltState] = ");
Serial.print(stripState.multiColorAltState);
Serial.print(", ");
Serial.print(stripState.ledModeColor[stripState.multiColorAltState]);
Serial.print(", multiColorNextTime,currentMillis = ");
Serial.print(multiColorNextColorTime);
Serial.print(", ");
Serial.print(millis());
Serial.print("\n");
#endif
    stripState.multiColorAltState = (stripState.multiColorAltState + 1) % numMultiColors;
    startFade();
  }
}

void alternatingMultiColor(int numMultiColors) {
  for (int x = (NUM_LEDS - 1) ; x >= 0 ; x -= numMultiColors) {
    for (int y = 0; y < numMultiColors; y++) {
      ledColorFadeTo[x - y] = stripState.ledModeColor[y];
    }
  }
  startFade();
}

void solidOneColor() {
  solidMultiColor(1);
}

void solidTwoColors() {
  solidMultiColor(2);
}

void solidThreeColors() {
  solidMultiColor(3);
}

void alternatingTwoColors() {
  alternatingMultiColor(2);
}

void alternatingTwoRandomColors() {
  alternatingMultiColor(2);
}

void alternatingThreeColors() {
  alternatingMultiColor(3);
}

void startFade() {
  stripState.fading = true;
  unsigned int ledFadeSteps = LED_FADE_STEPS;
  long colorDist;
  multiColorNextColorTime += (stripState.fadeTimeInterval * ledFadeSteps);
  switch (stripState.ledFadeMode) {
    case 0:
      ledFadeStepTime = 0;
      ledFadeStepIndex = 0;
      for (int x = (NUM_LEDS - 1) ; x >= 0 ; x--) {
        ledColorOld[x] = ledColor[x];
        colorDist = ledColorFadeTo[x] - ledColorOld[x];
        ledFadeStep[x][0] = (double) (colorDist >> 16) / (double) ledFadeSteps;
        ledFadeStep[x][1] = (double) ((colorDist >> 8) & 0xFF) / (double) ledFadeSteps;
        ledFadeStep[x][2] = (double) (colorDist & 0xFF) / (double) ledFadeSteps;
        if (0 > colorDist) {
          ledFadeStep[x][1] *= -1;
          ledFadeStep[x][2] *= -1;
        }
      }
/*
#ifdef DEBUG_MODE
Serial.print("startFade -- ");
Serial.print("ledColorOld[x] = ");
Serial.print(ledColorOld[x]);
Serial.print(" | ");
Serial.print("ledColorFadeTo[x] = ");
Serial.print(ledColorFadeTo[x]);
Serial.print(" | ");
Serial.print("ledFadeStep[x] = ");
Serial.print(colorDist & 0xFF);
Serial.print(",");
Serial.print(ledFadeStep[x][0]);
Serial.print(",");
Serial.print(ledFadeStep[x][1]);
Serial.print(",");
Serial.print(ledFadeStep[x][2]);
Serial.print("\n");
#endif
*/
      break;
    case 1:
    default:
      for (int x = (NUM_LEDS - 1) ; x >= 0 ; x--) {
        ledColorOld[x] = ledColor[x];
        ledColor[x] = ledColorFadeTo[x];
      }
      break;
  }
}

void doFade() {
  if (!stripState.fading) { return; }
  unsigned int ledFadeSteps = LED_FADE_STEPS;
  switch (stripState.ledFadeMode) {
    case 0:
      int x, y;
      if (ledFadeStepIndex >= ledFadeSteps) {
        // finished, set end color
        for (x = (NUM_LEDS - 1) ; x >= 0 ; x--) {
          ledColor[x] = ledColorFadeTo[x];
        }
        stripState.fading = false;
      }
      if (ledFadeStepTime - millis() > stripState.fadeTimeInterval) {
        unsigned long newColor;
        for (x = (NUM_LEDS - 1) ; x >= 0 ; x--) {
          newColor = ledColorOld[x];
          for (y = 2; y >= 0; y--) {
            if (0 > ledFadeStep[x][y]) {
              newColor -= (long) ((long) ((double) ledFadeStepIndex * -ledFadeStep[x][y]) << (y * 8));
            } else {
              newColor += (long) ((unsigned long) ((double) ledFadeStepIndex * ledFadeStep[x][y]) << (y * 8));
            }
          }
          ledColor[x] = newColor;
        }
        ledFadeStepIndex++;
        ledFadeStepTime = millis() + stripState.fadeTimeInterval;
/*
#ifdef DEBUG_MODE
Serial.print("ledFadeStepIndex = ");
Serial.print(ledFadeStepIndex);
Serial.print(" | ");
Serial.print("ledFadeStep[0][0] = ");
Serial.print(ledFadeStep[0][0]);
Serial.print(" | ");
Serial.print("ledColorOld[0] = ");
Serial.print(ledColorOld[0]);
Serial.print(" | ");
Serial.print("ledColor[0] = ");
Serial.print(ledColor[0]);
Serial.print("\n");
#endif
*/
      }
      break;
    case 1:
    default:
      colorWipe(ledFadeSteps * stripState.fadeTimeInterval);
      stripState.fading = false;
      break;
  }
}

/**
  * Throws random colors down the strip array.
  */
void randomCandy(void) {
  if (multiColorNextColorTime - millis() > stripState.multiColorHoldTime) {
    //get new RGB color
    unsigned long new_color = random(0xFFFFFF);
    // move old color down chain
    for (int x = (NUM_LEDS - 1) ; x > 0 ; x--) {
      ledColor[x] = ledColor[x - 1];
    }
    // set new led color
    ledColor[0] = new_color;
    multiColorNextColorTime = millis() + stripState.multiColorHoldTime;
  }
}

void rainbow(uint8_t wait) {
  if (multiColorNextColorTime - millis() > wait) {
    int i;

    if (256 < rainbowStepIndex) {
      // 3 cycles of all 256 colors in the wheel
      rainbowStepIndex = 0;
    }
    for (i=0; i < strip.numPixels(); i++) {
      ledColor[i] = wheel((i + rainbowStepIndex) % 255);
    }
    rainbowStepIndex++;
    multiColorNextColorTime = millis() + wait;
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(uint8_t wait) {
  if (multiColorNextColorTime - millis() > wait) {
    int i;
    if (rainbowStepIndex > 256 * 5) {
      rainbowStepIndex = 0;
    }
    for (i=0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      ledColor[i] =  wheel(((i * 256 / strip.numPixels()) + rainbowStepIndex) % 256);
    }
    rainbowStepIndex++;
    multiColorNextColorTime = millis() + wait;
  }
}

// fill the dots one after the other with set colors
// good for testing purposes
void colorWipe(uint8_t wait) {
  int i;
  for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, ledColor[i]);
      strip.show();
      delay(wait);
  }
}

void renderPixels() {
  int i;
  float brightness = stripState.ledStripBrightness;
  unsigned long ledColorFadeToChannels[3];
  ledColorFadeToChannels[0] = ((float) ((ledColor[i] >> 16) & 0xFF) * brightness);
  ledColorFadeToChannels[1] = ((float) ((ledColor[i] >> 8) & 0xFF) * brightness);
  ledColorFadeToChannels[2] = ((float) (ledColor[i] & 0xFF) * brightness);
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, ledColorFadeToChannels[0] << 16 
        | ledColorFadeToChannels[1] << 8 
        | ledColorFadeToChannels[2]);
    //strip.setPixelColor(i, ledColor[i]);
  }
  strip.show();
}

/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t rgbColor(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t wheel(byte wheelPos)
{
  if (wheelPos < 85) {
   return rgbColor(wheelPos * 3, 255 - wheelPos * 3, 0);
  } else if (wheelPos < 170) {
   wheelPos -= 85;
   return rgbColor(255 - wheelPos * 3, 0, wheelPos * 3);
  } else {
   wheelPos -= 170; 
   return rgbColor(0, wheelPos * 3, 255 - wheelPos * 3);
  }
}

