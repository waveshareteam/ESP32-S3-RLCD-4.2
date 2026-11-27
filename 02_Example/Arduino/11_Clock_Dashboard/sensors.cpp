#include "sensors.h"

#include <Wire.h>

#include "app_settings.h"
#include "calc.h"
#include "config.h"

// ---------------------------------------------------------------------------
// SHTC3
// ---------------------------------------------------------------------------
static const uint16_t SHTC3_WAKEUP = 0x3517;
static const uint16_t SHTC3_SLEEP = 0xB098;
static const uint16_t SHTC3_MEASURE_T_FIRST = 0x7866;  // normal power, no clock stretching

static bool shtc3Write(uint16_t cmd) {
  Wire.beginTransmission(I2C_ADDR_SHTC3);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  return Wire.endTransmission() == 0;
}

bool sensorsBegin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.setTimeOut(50);
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);
  IndoorReading probe;
  return readIndoor(probe);
}

bool readIndoor(IndoorReading &out) {
  out.valid = false;
  if (!shtc3Write(SHTC3_WAKEUP)) return false;
  delayMicroseconds(300);  // wake-up time, datasheet 240 us
  if (!shtc3Write(SHTC3_MEASURE_T_FIRST)) {
    shtc3Write(SHTC3_SLEEP);
    return false;
  }
  delay(15);  // measurement takes up to 12.1 ms

  uint8_t buf[6];
  bool ok = Wire.requestFrom((uint8_t)I2C_ADDR_SHTC3, (size_t)6) == 6;
  if (ok) {
    for (int i = 0; i < 6; i++) buf[i] = (uint8_t)Wire.read();
  }
  shtc3Write(SHTC3_SLEEP);  // back to ~0.3 uA
  if (!ok) return false;

  float t, rh;
  if (!calc::shtc3Decode(buf, &t, &rh)) return false;
  out.rawTempC = t;
  out.tempC = t + g_cfg.indoorOffsetC;
  out.rh = calc::humidityAtTemperature(rh, t, out.tempC);
  out.valid = true;
  return true;
}

// ---------------------------------------------------------------------------
// The audio chips, which the clock does not use
// ---------------------------------------------------------------------------
static bool writeRegisters(uint8_t address, const uint8_t (*pairs)[2], size_t count) {
  for (size_t i = 0; i < count; i++) {
    Wire.beginTransmission(address);
    Wire.write(pairs[i][0]);
    Wire.write(pairs[i][1]);
    if (Wire.endTransmission() != 0) return false;
  }
  return true;
}

int audioChipsStandby() {
  // Register and value, in the order of es8311_suspend() and es7210_stop() in Waveshare's audio example for
  // this board (07_Audio_Test/src/ExternLib/esp_codec_dev/device/...): what that library writes when it
  // closes the chips.  Volumes to zero, the analog parts and the microphone bias off, the clocks off.
  static const uint8_t kEs8311[][2] = {{0x32, 0x00}, {0x17, 0x00}, {0x0E, 0xFF}, {0x12, 0x02}, {0x14, 0x00},
                                       {0x0D, 0xFA}, {0x15, 0x00}, {0x02, 0x10}, {0x00, 0x00}, {0x00, 0x1F},
                                       {0x01, 0x30}, {0x01, 0x00}, {0x45, 0x00}, {0x0D, 0xFC}, {0x02, 0x00}};
  static const uint8_t kEs7210[][2] = {{0x47, 0xFF}, {0x48, 0xFF}, {0x49, 0xFF}, {0x4A, 0xFF}, {0x4B, 0xFF},
                                       {0x4C, 0xFF}, {0x40, 0xC0}, {0x01, 0x7F}, {0x06, 0x07}};
  int answered = 0;
  if (writeRegisters(I2C_ADDR_ES8311, kEs8311, sizeof kEs8311 / sizeof kEs8311[0])) answered |= AUDIO_CODEC_ANSWERED;
  if (writeRegisters(I2C_ADDR_ES7210, kEs7210, sizeof kEs7210 / sizeof kEs7210[0])) answered |= AUDIO_MIC_ADC_ANSWERED;
  return answered;
}

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
BatteryReading readBattery() {
  BatteryReading r;
  uint32_t sum = 0;
  const int kSamples = 16;
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);  // also needed when this is the very first call at boot
  for (int i = 0; i < kSamples; i++) {
    sum += analogReadMilliVolts(PIN_BATTERY_ADC);  // factory-calibrated millivolts
    delayMicroseconds(200);
  }
  r.rawVolts = (sum / (float)kSamples) / 1000.0f * BATTERY_DIVIDER;
  r.volts = r.rawVolts * g_cfg.batteryCalibration;
  r.present = r.rawVolts >= 2.5f;  // a floating / unpopulated pin reads far below any real cell
  r.percent = r.present ? calc::batteryPercent(r.volts) : 0;
  return r;
}

// ---------------------------------------------------------------------------
// PCF85063 RTC.  Registers 0x04..0x0A hold sec, min, hour, day, weekday, month,
// year as BCD; bit 7 of the seconds register is the oscillator-stop flag.
// ---------------------------------------------------------------------------
static const uint8_t RTC_REG_SECONDS = 0x04;

static inline uint8_t bcdToDec(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static inline uint8_t decToBcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

bool rtcReadUtc(time_t *utc) {
  Wire.beginTransmission(I2C_ADDR_PCF85063);
  Wire.write(RTC_REG_SECONDS);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)I2C_ADDR_PCF85063, (size_t)7) != 7) return false;
  uint8_t b[7];
  for (int i = 0; i < 7; i++) b[i] = (uint8_t)Wire.read();

  if (b[0] & 0x80) return false;  // oscillator stopped: contents are not trustworthy

  int sec = bcdToDec(b[0] & 0x7F);
  int min = bcdToDec(b[1] & 0x7F);
  int hour = bcdToDec(b[2] & 0x3F);
  int day = bcdToDec(b[3] & 0x3F);
  int mon = bcdToDec(b[5] & 0x1F);
  int year = 2000 + bcdToDec(b[6]);
  if (sec > 59 || min > 59 || hour > 23 || day < 1 || day > 31 || mon < 1 || mon > 12) return false;
  if (year < 2024) return false;  // never set, or reset to the chip default (2000)

  *utc = (time_t)calc::epochFromUtc(year, mon, day, hour, min, sec);
  return true;
}

bool rtcWriteUtc(time_t utc) {
  struct tm t;
  gmtime_r(&utc, &t);
  Wire.beginTransmission(I2C_ADDR_PCF85063);
  Wire.write(RTC_REG_SECONDS);
  Wire.write(decToBcd(t.tm_sec) & 0x7F);  // clears the oscillator-stop flag
  Wire.write(decToBcd(t.tm_min));
  Wire.write(decToBcd(t.tm_hour));
  Wire.write(decToBcd(t.tm_mday));
  Wire.write((uint8_t)t.tm_wday);
  Wire.write(decToBcd(t.tm_mon + 1));
  Wire.write(decToBcd(t.tm_year % 100));
  return Wire.endTransmission() == 0;
}
