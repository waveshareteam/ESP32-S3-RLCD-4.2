#pragma once

// Board sensors: SHTC3 temperature/humidity, PCF85063 real-time clock and the
// battery voltage divider.  Everything here is called from the UI task only
// (it owns the I2C bus).

#include <Arduino.h>
#include <time.h>

struct IndoorReading {
  bool valid = false;
  float tempC = 0;     // corrected for self-heating (INDOOR_TEMP_OFFSET_C)
  float rh = 0;        // corrected to match
  float rawTempC = 0;  // as measured
};

struct BatteryReading {
  bool present = false;
  float volts = 0;     // corrected by the battery_calibration setting
  float rawVolts = 0;  // as the ADC and the divider gave it, before that
  int percent = 0;
};

// Starts the I2C bus.  Returns true when the SHTC3 answered.
bool sensorsBegin();

// One blocking measurement (about 15 ms).  Returns false on a bus or CRC error.
bool readIndoor(IndoorReading &out);

// Average of several ADC samples, converted through the 1:3 divider and multiplied by the
// battery_calibration setting (everything else works with the corrected voltage).
BatteryReading readBattery();

// Hardware RTC, kept in UTC.  rtcReadUtc() is false when the clock has never
// been set or lost power (the chip's oscillator-stop flag).
bool rtcReadUtc(time_t *utc);
bool rtcWriteUtc(time_t utc);

// The two audio chips on the same I2C bus, which the clock does not use (an ES8311 codec and an ES7210
// microphone ADC): told to power down, with what Waveshare's audio example writes when it closes them.
// Returns which of them answered.  Call it once, after sensorsBegin().
enum : int { AUDIO_CODEC_ANSWERED = 1, AUDIO_MIC_ADC_ANSWERED = 2 };
int audioChipsStandby();
