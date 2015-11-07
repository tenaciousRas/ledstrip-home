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
  :::
  This fork only supports all-white or single-color LED strips and drives them with PWM.
  
 */
#include <EEPROM.h>
#include "SoftwareSerial.h"
#include "ledstrip_home.h"
#define OBI_RX 4
#define OBI_TX 6
#include "Obi.h"

#define _VERSION 5  // it's probably more like v.100
//#define DEBUG_MODE 1

// comment out if no POT setup
#define HAS_POT 1

#define STRIP_TYPE_PWM  10
// LED Strip setup
// set number of LED strips
#define NUM_STRIPS 1
// Set number color of PWM STRIPs
#define NUM_LEDS_PWM_WHITE_STRIP  1  // single color PWM

uint8_t pwmStripPins[NUM_STRIPS][1] = {{9}};
uint8_t stripNumPixels[NUM_STRIPS] = {1};

#define OFF 0x00
#define WHITE 0xFF

// POT setup
#ifdef HAS_POT
#define POT_PIN_OUT  2
#define POT_PIN_IN  A3
#endif

#define DEFAULT_LED_FADE_MODE 2;
#define DEFAULT_LED_STRIP_BRIGHTNESS 1.0;

led_strip_disp_state stripState[NUM_STRIPS];
uint8_t remoteControlStripIndex = 0;

#define LED_FADE_STEPS 16;
#define LED_FADE_STEP_DELAY_MS 50;  // microsecs between fade steps
float ledBrightness[NUM_STRIPS][1] = { 
  {0.1} 
  };  // color before fade
float ledBrightnessOld[NUM_STRIPS][1] = { 
  {0.0} 
  };  // color before fade
float ledBrightnessFadeTo[NUM_STRIPS][1] = { 
  {1.0}
  };    // fade-to color
unsigned long ledFadeStepTime[NUM_STRIPS];  // time to next fade step
int ledFadeStepIndex[NUM_STRIPS];  // color distance divided by LED_FADE_STEPS
double ledFadeStep[NUM_STRIPS][1][1];  // 1 pixel, 1 color

int eepromAddyStripState = 4;  // eeprom addy to store strip state

Obi btModule = Obi(OBI_RX, OBI_TX);

uint8_t brightnessSetByDeviceId = 0;  // 0 for serial/bluetooth; 1 for analog pot
#ifdef HAS_POT
float potNewReadingTolerance = 0.1;
uint32_t potReading = 0;
#endif

void setup() {
  // set the data rate for bt device
//  Serial.begin(9600);
#ifdef DEBUG_MODE
  Serial.print(F("startup\n"));
#endif
  // send init data
  btModule.registerFunction(btSetRemoteControlStripIndex, 'n');
  btModule.registerFunction(btSetBright, 'b');
  btModule.registerFunction(btSetLedFadeMode, 'f');
  btModule.registerFunction(btSetLedFadeTimeInterval, 'i');
  btModule.registerFunction(btQueryInfo, 'q');
  btModule.begin(9600);
  btModule.send("arduino led controller connected via bluetooth.\n");
#ifdef DEBUG_MODE
  Serial.print(F("configured bt\n"));
#endif
  for (int i = 0; i < NUM_STRIPS; i++) {
    initStripState(i);
    // read eeprom
    readStripState(&stripState[i]);
  }
#ifdef HAS_POT
  digitalWrite(POT_PIN_OUT, HIGH);
#endif
}

void loop() {
  btModule.receive(); // receive BT events
#ifdef HAS_POT
  readPotBrightness();
#endif
  for (int i = 0; i < NUM_STRIPS; i++) {
    if (stripState[i].fading) {
      doFade(i);
      renderPixels(i);
      continue;
    }
  }
}

void initStripState(uint8_t stripNum) {
  stripState[stripNum].fading = false;
  stripState[stripNum].ledFadeMode = DEFAULT_LED_FADE_MODE;
  stripState[stripNum].fadeTimeInterval = LED_FADE_STEP_DELAY_MS;
  stripState[stripNum].ledStripBrightness = DEFAULT_LED_STRIP_BRIGHTNESS;
}

void readPotBrightness() {
#ifdef HAS_POT
  uint32_t newReading = 0;
  newReading = analogRead(POT_PIN_IN);
  uint32_t oldReadingTolerance = newReading * potNewReadingTolerance;
  if (newReading > potReading + oldReadingTolerance
      || newReading < potReading - oldReadingTolerance) {
    potReading = newReading;
    for (int i = 0; i < NUM_STRIPS; i++) {
      stripState[i].ledStripBrightness = potReading / 4;
      putStripState(&stripState[i]);
    }
    brightnessSetByDeviceId = 1;
#ifdef DEBUG_MODE
Serial.print(F("ledStripBrightness[0] = "));
Serial.print(bData);
Serial.print(F(", "));
Serial.print((float) stripState[0].ledStripBrightness);
Serial.print(F("\n"));
#endif
  }
#endif
}

void readStripState(led_strip_disp_state* ret) {
  unsigned int offset = eepromAddyStripState + sizeof(int);  // store int at addy 0
  byte bData;
  ret->fading = false;
  bData = EEPROM.read(offset++);
  ret->ledFadeMode = (uint8_t) bData;
  unsigned long fadeTimeInterval;
  for (int i = 0; i < 3; i++) {
    fadeTimeInterval = 0;
    for (int j = 3; j >= 0; j--) {
      bData = EEPROM.read(offset++);
      fadeTimeInterval += (bData << (8 * j));
    }
    ret->fadeTimeInterval = fadeTimeInterval;
  }
  unsigned int storedLedStripBrightness;
  storedLedStripBrightness = 0;
  for (int j = 1; j >= 0; j--) {
    bData = EEPROM.read(offset++);
    storedLedStripBrightness += (bData << (8 * j));
  }
  ret->ledStripBrightness = (float) storedLedStripBrightness / (float) 1024;
#ifdef DEBUG_MODE
  Serial.print(F("storedLedStripBrightness = "));
  Serial.print((ret->ledStripBrightness));
  Serial.print(F("\nread eeprom done\n"));
#endif
}

void putStripState(led_strip_disp_state* lsds) {
  led_strip_disp_state storedState;
  readStripState(&storedState);
  unsigned int offset = eepromAddyStripState + sizeof(int);  // store int at addy 0
  if (storedState.ledFadeMode != lsds->ledFadeMode) {
    EEPROM.write(offset, (byte) lsds->ledFadeMode);
  }
  offset++;
  if (storedState.fadeTimeInterval != lsds->fadeTimeInterval) {
    for (int j = 3; j >= 0; j--) {
      EEPROM.write(offset++, (byte) ((lsds->fadeTimeInterval >> (8 * j)) & 0xFF));
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
  Serial.print(F("write eeprom done\n"));
#endif
}

void btQueryInfo(byte flag, byte numOfValues) {
  btModule.send("{ firmware : 'ledstrip_home_by_f_beachler'");
  btModule.send(", ver : '");
  btModule.send(_VERSION);
  btModule.send("', numStrips : ");
  btModule.send(NUM_STRIPS);
  btModule.send(", hasPot : ");
#ifdef HAS_POT
  btModule.send(1);
  btModule.send(", potReading : ");
  btModule.send(potReading);
#elif
  btModule.send(0);
#endif
  btModule.send(", brightnessSetByDeviceId : ");
  btModule.send(brightnessSetByDeviceId);
  btModule.send(", remoteControlStripIndex : ");
  btModule.send(remoteControlStripIndex);
  btModule.send(", strips : [\n");
  for (int i = 0; i < NUM_STRIPS; i++) {
    btModule.send((i > 0 ? ", { " : " { "));
    btModule.send("stripType : ");
    btModule.send(STRIP_TYPE_PWM);
    btModule.send(", stripNumPixels : ");
    btModule.send(stripNumPixels[i]);
    btModule.send(", ledFadeMode : ");
    btModule.send(stripState[i].ledFadeMode);
    btModule.send(", fadeTimeInterval : ");
    btModule.send(stripState[i].fadeTimeInterval);
    btModule.send(", ledStripBrightness : ");
    btModule.send(stripState[i].ledStripBrightness);
    btModule.send(" }\n");
  }
  btModule.send("]\n");
  btModule.send("}\n");
}

void btSetRemoteControlStripIndex(byte flag, byte numOfValues) {
  int bData;
  bData = btModule.getInt();
  if (0 > bData) {
    bData = 0;
  }
  if (bData > NUM_STRIPS - 1) {
    bData = NUM_STRIPS - 1;
  }
  remoteControlStripIndex = bData;
  btModule.send("OK\n");
#ifdef DEBUG_MODE
Serial.print(F("remoteControlStripIndex = "));
Serial.print(remoteControlStripIndex);
#endif
}

void btSetBright(byte flag, byte numOfValues) {
  int bData;
  bData = btModule.getInt();
  if (255 < bData) {
    bData = 255;
  }
  stripState[remoteControlStripIndex].ledStripBrightness = (float) bData / (float) 255.0;
  putStripState(&stripState[remoteControlStripIndex]);
  brightnessSetByDeviceId = 0;
  btModule.send("OK\n");
#ifdef DEBUG_MODE
Serial.print(F("ledStripBrightness = "));
Serial.print(bData);
Serial.print(F(", "));
Serial.print((float) stripState[remoteControlStripIndex].ledStripBrightness);
Serial.print(F("\n"));
#endif
}

void btSetLedFadeTimeInterval(byte flag, byte numOfValues) {
  unsigned long bData;
  bData = btModule.getLong();
  stripState[remoteControlStripIndex].fadeTimeInterval = bData;
  putStripState(&stripState[remoteControlStripIndex]);
  btModule.send("OK\n");
}

void btSetLedFadeMode(byte flag, byte numOfValues) {
  unsigned int bData;
  bData = btModule.getInt();
  if (1 < bData) {
    bData = 0;
  }
  stripState[remoteControlStripIndex].ledFadeMode = bData;
  putStripState(&stripState[remoteControlStripIndex]);
  btModule.send("OK\n");
#ifdef DEBUG_MODE
Serial.print(F("ledFadeMode = "));
Serial.print(stripState[remoteControlStripIndex].ledFadeMode);
Serial.print(F("\n"));
#endif
}

void refreshLEDs(uint8_t stripNum) {
    initStripState(stripNum);
    readStripState(&stripState[stripNum]);
}

void turnOffLEDs(uint8_t stripNum) {
  stripState[stripNum].ledStripBrightness = 0.0;
}

void white(uint8_t stripNum) {
  stripState[stripNum].ledStripBrightness = 1.0;
}

void startFade(uint8_t stripNum) {
  stripState[stripNum].fading = true;
  unsigned int ledFadeSteps = LED_FADE_STEPS;
  float brightDist;
  float  rLedBrightness, rLedBrightnessOld;
  switch (stripState[stripNum].ledFadeMode) {
    case 0:
    default:
      ledFadeStepTime[stripNum] = micros() - stripState[stripNum].fadeTimeInterval + 1;
      ledFadeStepIndex[stripNum] = 0;
      for (int x = (stripNumPixels[stripNum] - 1); x >= 0 ; x--) {
        rLedBrightness = ledBrightness[stripNum][x];
        ledBrightnessOld[stripNum][x] = rLedBrightness;
        rLedBrightnessOld = rLedBrightness;
        brightDist = ledBrightnessFadeTo[stripNum][x] - rLedBrightnessOld;
        ledFadeStep[stripNum][x][0] = (float) brightDist / (float) ledFadeSteps;
#ifdef DEBUG_MODE
Serial.print(F("startFade for stripNum -- "));
Serial.print(stripNum);
Serial.print(F("; ledBrightnessOld[x] = "));
Serial.print(ledBrightnessOld[stripNum][x]);
Serial.print(F(" | "));
Serial.print(F("ledBrightnessFadeTo[x] = "));
Serial.print(ledBrightnessFadeTo[stripNum][x]);
Serial.print(F(" | "));
Serial.print(F("brightDist = "));
Serial.print(brightDist & 0xFF);
Serial.print(F(","));
Serial.print(ledFadeStep[stripNum][x][0]);
Serial.print(F("\n"));
#endif
      }
      break;
  }
}

void doFade(uint8_t stripNum) {
  if (!stripState[stripNum].fading) { return; }
  uint16_t ledFadeSteps = LED_FADE_STEPS;
  uint32_t  rLedBrightness, rLedBrightnessOld;
  switch (stripState[stripNum].ledFadeMode) {
    default:
    case 0:
      int x, y;
      if ((uint16_t) ledFadeStepIndex[stripNum] >= ledFadeSteps) {
        // finished, set end color
        for (x = (stripNumPixels[stripNum] - 1) ; x >= 0 ; x--) {
          ledBrightness[stripNum][x] = ledBrightnessFadeTo[stripNum][x];
        }
        stripState[stripNum].fading = false;
        break;
      }
      if (ledFadeStepTime[stripNum] - micros() > stripState[stripNum].fadeTimeInterval) {
        uint32_t newBrightness;
        for (x = (stripNumPixels[stripNum] - 1) ; x >= 0 ; x--) {
          newBrightness = ledBrightnessOld[stripNum][x];
          for (y = 2; y >= 0; y--) {
            if (0 > ledFadeStep[stripNum][x][y]) {
              newBrightness -= (long) ((long) ((double) ledFadeStepIndex[stripNum] * -ledFadeStep[stripNum][x][y]) << (y * 8));
            } else {
              newBrightness += (long) ((unsigned long) ((double) ledFadeStepIndex[stripNum] * ledFadeStep[stripNum][x][y]) << (y * 8));
            }
          }
          ledBrightness[stripNum][x] = newBrightness;
        }
        ledFadeStepIndex[stripNum]++;
        ledFadeStepTime[stripNum] = millis() + stripState[stripNum].fadeTimeInterval;
/*
#ifdef DEBUG_MODE
Serial.print(F("ledFadeStepIndex = "));
Serial.print(ledFadeStepIndex);
Serial.print(F(" | "));
Serial.print(F("ledFadeStep[0][0] = "));
Serial.print(ledFadeStep[0][0]);
Serial.print(F(" | "));
Serial.print(F("ledBrightnessOld[0] = "));
Serial.print(ledBrightnessOld[0]);
Serial.print(F(" | "));
Serial.print(F("ledBrightness[0] = "));
Serial.print(ledBrightness[0]);
Serial.print(F("\n"));
#endif
*/
      }
      break;
  }
}

/*
 * Render strip.
 */
void renderPixels(uint8_t stripNum) {
  float brightness = stripState[stripNum].ledStripBrightness;
  analogWrite(pwmStripPins[stripNum][0], (int) ((float) (0xFF) * brightness) & 0xFF);
}

