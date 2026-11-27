#pragma once

// When to switch the clock off before it flattens the cell.  A bare LiPo with no protection
// board is ruined by being run to zero; this board has a protection chip as a last resort, but
// that cuts off abruptly and a lot lower, so the software stops first.
//
//   * Guard: fed with each battery reading while running on the battery.  The reading has to stay
//     under the cut-off for a minute (and for at least three readings), so a WiFi burst that drags
//     the voltage down for a moment never trips it.
//   * bootDecision(): asked first thing after every power-up or wake-up, before the display or
//     WiFi are started.  After a low-battery shutdown the clock only starts again once the cell is
//     back above kRestartV.  That is well above the cut-off on purpose: a cell that has just been
//     relieved of the load springs back by a hundred millivolts or more, so restarting at the
//     cut-off would boot, sag, shut down and repeat.  Charging lifts it past kRestartV; waiting
//     does not.
//
// Header-only and free of Arduino dependencies so tools/tests can check it on a PC.

#include <stdint.h>

namespace lowbat {

const float kMinCutoffV = 3.10f;        // limits of the setting (see settings.cpp)
const float kMaxCutoffV = 3.60f;
const float kRestartV = 3.70f;          // restart only above this after a low-battery shutdown
const float kHysteresisV = 0.02f;       // a reading must come this far above the cut-off to cancel a trip
const uint32_t kHoldSec = 60;           // ... and under it for this long to trip
const int kMinReadings = 3;
const uint32_t kSleepCheckSec = 300;    // while shut down: look at the battery this often

class Guard {
 public:
  void configure(bool enabled, float cutoffV) {
    enabled_ = enabled;
    cutoff_ = cutoffV < kMinCutoffV ? kMinCutoffV : (cutoffV > kMaxCutoffV ? kMaxCutoffV : cutoffV);
    reset();
  }

  void reset() {
    below_ = false;
    count_ = 0;
  }

  // One reading of the battery voltage.  onBattery is false while charging or when the battery
  // is not what the clock runs on (the guard then stands down).  True means: shut down now.
  bool feed(uint32_t nowSec, float volts, bool onBattery) {
    if (!enabled_ || !onBattery || !(volts > 0.5f)) {
      reset();
      return false;
    }
    if (volts < cutoff_) {
      if (!below_) {
        below_ = true;
        belowSince_ = nowSec;
        count_ = 0;
      }
      count_++;
    } else if (volts >= cutoff_ + kHysteresisV) {
      reset();
      return false;
    }
    return below_ && count_ >= kMinReadings && (nowSec - belowSince_) >= kHoldSec;
  }

  float cutoffV() const { return cutoff_; }
  bool enabled() const { return enabled_; }
  bool tripping() const { return below_; }  // under the cut-off right now (the clock may warn)

 private:
  bool enabled_ = true;
  float cutoff_ = 3.30f;
  bool below_ = false;
  uint32_t belowSince_ = 0;
  int count_ = 0;
};

enum class BootAction : uint8_t { RUN, STAY_ASLEEP };

// volts: the battery reading taken first thing; shutdownFlag: the previous run ended with a
// low-battery shutdown (kept in RTC memory or flash).
inline BootAction bootDecision(bool enabled, float volts, float cutoffV, bool shutdownFlag) {
  if (!enabled || !(volts > 0.5f)) return BootAction::RUN;  // off, or no reading to go by
  if (shutdownFlag) return volts >= kRestartV ? BootAction::RUN : BootAction::STAY_ASLEEP;
  return volts < cutoffV ? BootAction::STAY_ASLEEP : BootAction::RUN;
}

}  // namespace lowbat
