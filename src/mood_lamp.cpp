/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <WS2812FX.h>
//#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN    3    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 16

#define LED_COUNT 16
#define LED_PIN 3

#define AVERAGE_BUF_LEN 100
#define SENSE_SAMPLES         100
#define EDGE_SAMPLE_INTERVAL  200
#define CALIBRATION_INTERVAL  60000
#define CALIBRATION_TIME      500
#define INITIAL_CALIBRATION   1000

#define OFF_TIME              3000

#define FALLING_EDGE            0
#define RISING_EDGE             1
#define NO_CHANGE               2

#define OFF                     2
#define TOUCH                   1
#define TOUCH_REMOVED           0

#define RESET_EFFECTS           1

CapacitiveSensor   cs = CapacitiveSensor(6,9);        // 10 megohm resistor between pins 4 & 8, pin 8 is sensor pin, add wire, foil
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);


unsigned int get_touch_value() {
  static unsigned int edge = 2;
  static bool level[4] = { 0, 0 , 0, 0};
  unsigned long sense;
  unsigned long threshold_high = 0;
  unsigned long threshold_low = 0;

  static unsigned long previousMillisEdge = 0;
  unsigned long currentMillis;

  threshold_high = 250;
  threshold_low =   60;

  // default edge
  edge = NO_CHANGE;

  // sample edge every EDGE_SAMPLE_INTERVAL
  currentMillis = millis();
  if (currentMillis - previousMillisEdge >= EDGE_SAMPLE_INTERVAL) {
    previousMillisEdge = currentMillis;

    sense = cs.capacitiveSensor(128);

    // rotate input data buffer
    for (int i=3; i > 0; i--) {
      level[i] = level[i-1];
    }

    // get "logic" level
    if (sense >  threshold_high) {
      level[0] = 1;
    } else if (sense < threshold_low){
      level[0] = 0;
    }

    // detect edge
    if ((level[3] == 0) && (level[2] == 0) && level[1] == 1 && level[0] == 1) {
      edge = RISING_EDGE;
    } else if ((level[3] == 1) && (level[2] == 1) && (level[1] == 0) && (level[0] == 0)) {
      edge = FALLING_EDGE;
    } else {
      edge = NO_CHANGE;
    }
  }
  return edge;
}

void next_effect (unsigned char reset) {

  static unsigned int effect_cnt = 3;

  if (reset == RESET_EFFECTS) {
      effect_cnt=3;
  } else {
    if (effect_cnt == 3) {
      effect_cnt=0;
    } else {
      effect_cnt++;
    }
  }

  switch (effect_cnt) {
    case 0:
      // smoke white
      ws2812fx.setBrightness(255);
      ws2812fx.setColor(255,255,255);
      ws2812fx.setMode(FX_MODE_STATIC);
      break;

    case 1:
      ws2812fx.setBrightness(255);
      ws2812fx.setSpeed(1);
      ws2812fx.setMode(FX_MODE_RAINBOW);
      break;

    case 2:
      ws2812fx.setBrightness(255);
      ws2812fx.setColor(226,88,34);
      ws2812fx.setSpeed(150);
      ws2812fx.setMode(FX_MODE_FIRE_FLICKER_SOFT);
      break;

    case 3:
      ws2812fx.setBrightness(128);
      ws2812fx.setColor(255,163,67);
      break;
  }
}

void setup() {

  Serial.begin(115200);

  ws2812fx.init();
  ws2812fx.setBrightness(128);
  ws2812fx.setColor(255,163,67);
  ws2812fx.start();

}

void loop()
{
    static unsigned long timeOn = 0;
    static unsigned char state = TOUCH_REMOVED;
    unsigned int edge;

    edge = get_touch_value();

    if (edge == RISING_EDGE) {
      //Serial.print("---rising edge ---");
      if (state == OFF) {
        next_effect(RESET_EFFECTS);
      } else {
        next_effect(0);
      }
      state = TOUCH;
      timeOn = millis();

    } else if (edge == FALLING_EDGE) {
      if (state != OFF)
        state = TOUCH_REMOVED;
    }

    if ((state == TOUCH) & (millis() - timeOn > OFF_TIME)) {
      ws2812fx.setColor(0,0,0);
      state = OFF;
    }

    ws2812fx.service();

}
