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
#define EDGE_SAMPLE_INTERVAL  50
#define CALIBRATION_INTERVAL  60000
#define CALIBRATION_TIME      500
#define INITIAL_CALIBRATION   1000

#define FALLING_EDGE            0
#define RISING_EDGE             1
#define NO_CHANGE               2

struct average {
  unsigned long cap_buffer[AVERAGE_BUF_LEN];
  unsigned long cur;
  unsigned long min;
  unsigned long max;
};
static struct average average;
static unsigned long max_sense =0;

static int touched = 0;

CapacitiveSensor   cs = CapacitiveSensor(6,9);        // 10 megohm resistor between pins 4 & 8, pin 8 is sensor pin, add wire, foil
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

// placeholder function in case we need to do something with sample value
static __inline__ long do_sample () {
  unsigned long sense;
  sense =  cs.capacitiveSensor(SENSE_SAMPLES);
  if (sense > max_sense) {
    max_sense = sense;
  }

  return sense;
}

void do_average( ) {
  unsigned int buf_index;
  unsigned int items_in_buffer;
  unsigned long long accumulated_average;

  unsigned long previousMillis = millis();
  unsigned long currentMillis = previousMillis;

  buf_index = 0;
  items_in_buffer = 0;

  // reset average stats
  average.min = 0xFFFFFFFF;
  average.max = 0;

  while ((currentMillis - previousMillis < CALIBRATION_TIME)) {
    currentMillis = millis();

    // catch overflow
    if (currentMillis < previousMillis) {
      Serial1.print("------ OVERFLOW -------");
      previousMillis = currentMillis;
      break;
    }

    // Calculate index in ring buffer
    if (buf_index == (AVERAGE_BUF_LEN-1) ) {
      buf_index=0;
    } else {
      buf_index++;
    }
    // get new average from values in ring buffer
    average.cap_buffer[buf_index] = do_sample();

    // count items in buffer to make sure we only average items recorded
    if (items_in_buffer < AVERAGE_BUF_LEN) {
      items_in_buffer++;
    }
  }

  // calculate average
  accumulated_average = 0;
  for (unsigned int i=0; i<=items_in_buffer; i++) {
    accumulated_average += average.cap_buffer[i];
  }
  average.cur = (unsigned long) (accumulated_average / items_in_buffer);

  // record min and max values
  if (average.cur > average.max) {
    average.max = average.cur;
  } else if (average.cur < average.min) {
    average.min = average.cur;
  }

}

unsigned int get_touch_value() {
  static unsigned int edge = 2;
  static bool level[4] = { 0, 0 , 0, 0};
  unsigned long sense;
  unsigned long threshold_high = 0;
  unsigned long threshold_low = 0;

  static unsigned long previousMillisEdge = 0;
  unsigned long currentMillis;

  // try to get sensible initial default thresholds
  if (average.max > 100) {
    threshold_high = average.max + 500;
    threshold_low = average.max + 100;
  } else {
    threshold_high = average.max * 10;
    threshold_low = average.max * 2;
  }

  // default edge
  edge = NO_CHANGE;

  // sample edge every EDGE_SAMPLE_INTERVAL
  currentMillis = millis();
  if (currentMillis - previousMillisEdge >= EDGE_SAMPLE_INTERVAL) {
    previousMillisEdge = currentMillis;

    sense = do_sample();

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

void change_effect (unsigned int edge ) {

  static unsigned int effect_cnt = 0;

  if (edge == RISING_EDGE) {
    if (effect_cnt == 5) {
        effect_cnt=0;
    } else {
      effect_cnt++;
    }
  }

  ws2812fx.setMode(FX_MODE_STATIC);
  switch (effect_cnt) {
    case 0:
      ws2812fx.setColor(255,0,0);
      break;
    case 1:
      ws2812fx.setColor(0,255,0);
      break;
    case 2:
      ws2812fx.setColor(0,0,255);
      break;
    case 3:
      ws2812fx.setColor(255,255,0);
      break;
    case 4:
      ws2812fx.setColor(0, 255,255);
      break;
    case 5:
      ws2812fx.setColor(255,255,255);
      break;
  }
}

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);

  // initialize buffers and structs
  for (int i=0; i<AVERAGE_BUF_LEN; i++){
    average.cap_buffer[i] = do_sample();
  }
  average.cur = 0;
  average.max = 0;
  average.min = 0xFFFFFFFF;

  ws2812fx.init();
  ws2812fx.setBrightness(255);
  ws2812fx.setSpeed(200);
  ws2812fx.setColor(244,217,66);
  ws2812fx.start();

  do_average();
}

void loop()
{

    static unsigned long previousMillisCalibration = 0;
    // static unsigned long previousMillisPrint = 0;
    unsigned long currentMillis = millis();
    unsigned int edge;

    if (currentMillis - previousMillisCalibration >= CALIBRATION_INTERVAL) {
      previousMillisCalibration = currentMillis;
      if (touched == 0) {
        do_average();
        return;
      }
    }

    edge = get_touch_value();

    if (edge == RISING_EDGE) {
      touched = 1;
      //Serial1.print("---rising edge ---");
    } else if (edge == FALLING_EDGE) {
      touched = 0;
      //Serial1.print("---falling edge ---");
    }

    change_effect(edge);
    ws2812fx.service();

}
