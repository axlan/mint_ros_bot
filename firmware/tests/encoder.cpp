#include <Arduino.h>

static constexpr int PIN_LED = 2;

static constexpr int PIN_LEFT_FWR = 21;
static constexpr int PIN_LEFT_BAK = 19;

static constexpr int PIN_LEFT_ENC = 13;


#define ENCODER_SENSOR_RING_MILLIS 0
#define ENCODER_SENSOR_TIMEOUT_MILLIS       400L // Timeout for encoder ticks if motor is running

volatile unsigned long LastEncoderInterruptMillis; // used internal for debouncing and lock/timeout detection
// Do not move it!!! It must be after AverageSpeedIsValid and is required for resetSpeedValues()
volatile unsigned long EncoderInterruptDeltaMillis; // Used to get speed

/*
  * Distance optocoupler impulse counter. It is reset at startGoDistanceCount if motor was stopped.
  * Both values are incremented at each encoder interrupt and reset at startGoDistanceMillimeter().
  */
volatile unsigned int EncoderCount; // 11 mm for a 220 mm Wheel and 20 encoder slots reset at startGoDistanceMillimeter
volatile unsigned int EncoderCountForSynchronize; // count used and modified by function

volatile bool SensorValuesHaveChanged; // true if encoder count and derived encoder speed, or one of the TMU data have changed

void IRAM_ATTR handleEncoderInterrupt() {
        EncoderCount++;
        EncoderCountForSynchronize++;
        SensorValuesHaveChanged = true;
    // digitalWrite(PIN_LED, LOW);
    // long tMillis = millis();
    // unsigned long tDeltaMillis = tMillis - LastEncoderInterruptMillis;
    // if (tDeltaMillis <= ENCODER_SENSOR_RING_MILLIS) {
    //     // assume signal is ringing and do nothing
    // } else {
    //     LastEncoderInterruptMillis = tMillis;

    //     if (tDeltaMillis < ENCODER_SENSOR_TIMEOUT_MILLIS) {
    //         EncoderInterruptDeltaMillis = tDeltaMillis;
    //     } else {
    //         // timeout
    //         EncoderInterruptDeltaMillis = 0;
    //     }

    //     EncoderCount++;
    //     EncoderCountForSynchronize++;
    //     SensorValuesHaveChanged = true;
    // }
}

void setup()
{
  Serial.begin(115200);

  EncoderCount = 0;
  EncoderCountForSynchronize = 0;
  EncoderInterruptDeltaMillis = 0;
  LastEncoderInterruptMillis = millis() - ENCODER_SENSOR_RING_MILLIS - 1; // Set to a sensible value to avoid initial timeout
  SensorValuesHaveChanged = false;

  pinMode(PIN_LEFT_FWR, OUTPUT);
  analogWrite(PIN_LEFT_FWR, 64);
  pinMode(PIN_LEFT_BAK, OUTPUT);
  analogWrite(PIN_LEFT_BAK, 0);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_LEFT_ENC, INPUT_PULLUP);
  attachInterrupt(PIN_LEFT_ENC, handleEncoderInterrupt, FALLING);
  Serial.println("Started");
}

void loop()
{
  if (SensorValuesHaveChanged) {
    auto tmp = EncoderCount;
    SensorValuesHaveChanged = false;
    Serial.println(EncoderCount);
  }
  delay(100);
}

// Diameter = 1.9in -> circumference = 5.96902604182
// Traveled 3Ft -> 25646
// 36in/C =  6.03113468559 revs
// 25646 / 6.03113468559 - > 4252.26782968 ticks per rev

