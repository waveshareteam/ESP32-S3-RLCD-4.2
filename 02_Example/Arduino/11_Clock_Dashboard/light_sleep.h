#pragma once

// Light sleep between the frames (light_sleep = on).
//
// With the radio off the clock has little to do in a second: it draws the coming second, sends it just before the
// tick and waits for the next one.  Without this, the wait is spent idling awake, in naps of a few milliseconds, and
// at 20 MHz the chip alone then draws some 9 to 12 mA (Espressif gives 13.2 mA at 40 MHz and nothing lower).  In light
// sleep it draws about a quarter of a milliamp and keeps its memory and the settings of its peripherals (the display
// keeps its picture by itself).  So when nothing needs the chip awake, the loop sleeps until the next thing it has to
// do instead of napping.  The Arduino core has no automatic light sleep (CONFIG_PM_ENABLE is not set), so the loop
// asks for each one itself.
//
// What keeps it awake (decide()):
//   * the radio, on or about to be: WiFi cannot sleep like this (wifi_mode = always never sleeps);
//   * the USB console: in light sleep the chip lets go of its USB port, so a computer on it would lose the clock
//     every second, and one plugged in later would not find it (console = auto or off shut the console down);
//   * a button that is down, or was pressed in the last 8 seconds (clockpolicy::kPokeMs), or a change of one that
//     the interrupts recorded and the loop has not taken yet: the loop times the gestures, and whoever pressed is
//     waiting for the answer;
//   * anything else that keeps the loop busy: a redraw that is due, the cube, a restart;
//   * slow work that is due (the sensors, the battery, a line of the power log) and can run now; when it has to
//     wait for its part of the second, the sleep ends where that part begins;
//   * a sleep the SDK refused less than kRetryMs ago (every try costs the calibration);
//   * the next step of the frame planner (frame_plan.h, FramePlanner::idleUs()), or anything else that is due
//     (a banner to take down), closer than kMinSleepUs.
// The sleep ends kGuardUs (plus 1/128 of its length) before the first of those, so the loop finds things as they
// would have been without it.  KEY and BOOT wake it at once (a low level on either pin).
//
// Keeping time.  The chip's timer stands still while it sleeps, and the SDK puts it forward afterwards by the time
// the RTC's slow clock counted: an RC oscillator inside the chip, calibrated against the crystal before every sleep.
// The seconds still come from the system clock, as before, and are drawn and sent on time; but for the time asleep
// that clock is only as good as the RC oscillator, which is why the hourly network time matters (drift.h measures
// what is left between two syncs; without WiFi nothing corrects it).  The SDK also takes the time of day afresh from
// the RC count at every wake-up, the time awake since the last wake-up included, which a long time awake (an hour of
// music on the Now Playing page) would turn into a jump.  WallFix puts that part back, so the time awake is counted
// by the crystal as before.
//
// Pure logic with no Arduino dependency: the decision, the steps of one nap against a small interface (napOnce();
// power.cpp has the one with the real SDK calls), the correction of the time of day, the share of the time asleep
// and the row of the Power and settings page.  tools/tests/test_radio.cpp runs all of it on a PC.

#include <stdint.h>
#include <stdio.h>

namespace lightsleep {

const int32_t kNever = INT32_MAX;       // "nothing due" for the times of Inputs
const int32_t kMinSleepUs = 20000;      // a shorter sleep is not worth the way in and out (the SDK calibrates for ~4 ms)
const int32_t kGuardUs = 5000;          // ... and every sleep ends this much early, plus 1/128 of its length
const int32_t kMaxSleepUs = 2000000;    // never longer than this (the frames wake it every second anyway)
const int64_t kWallStepUs = 2000;       // the time of day is corrected once the corrections add up to this
const uint32_t kRetryMs = 1000;         // after the SDK refused a sleep, none is tried for this long

// Why the loop did, or did not, sleep.
enum Why : uint8_t {
  ASLEEP = 0,  // it sleeps
  OFF,         // light_sleep = off
  RADIO,       // the radio is on, or about to be
  CONSOLE,     // the USB console runs
  BUTTON,      // a button is down, or was pressed a moment ago
  BUSY,        // the loop has something else to do (a redraw, the cube, a restart)
  WORK,        // slow work is due and can run now
  SOON,        // the next thing is too close to be worth a sleep
  REFUSED,     // the SDK would not sleep (the last try came back with an error)
};

// What the loop knows when it would otherwise nap.  The times are from now, in microseconds.
struct Inputs {
  bool enabled = false;        // light_sleep = on
  bool radio = false;          // netRadioNeedsFastClock()
  bool console = false;        // the USB console runs (g_consoleOn)
  bool button = false;         // KEY or BOOT is down (the pins, the click detectors), or a change of one waits in the record
  bool poked = false;          // a button was pressed in the last kPokeMs
  bool busy = false;           // a redraw is due, the cube is up, a restart is asked for
  bool backOff = false;        // the SDK refused a sleep less than kRetryMs ago
  int32_t frameInUs = 0;       // until the frame planner's next step (FramePlanner::idleUs())
  int32_t workInUs = kNever;   // until due slow work can run: 0 = now, kNever = none is due
  int32_t otherInUs = kNever;  // until anything else is due (a banner to take down), kNever = nothing
};

struct Decision {
  bool sleep = false;
  int32_t us = 0;  // how long, when it sleeps
  Why why = OFF;
};

inline Decision decide(const Inputs &in) {
  Decision d;
  if (!in.enabled) return d;  // (why = OFF)
  if (in.radio) {
    d.why = RADIO;
  } else if (in.console) {
    d.why = CONSOLE;
  } else if (in.button || in.poked) {
    d.why = BUTTON;
  } else if (in.busy) {
    d.why = BUSY;
  } else if (in.backOff) {
    d.why = REFUSED;
  } else if (in.workInUs <= 0) {
    d.why = WORK;
  } else {
    int64_t ahead = in.frameInUs;
    if (in.workInUs < ahead) ahead = in.workInUs;
    if (in.otherInUs < ahead) ahead = in.otherInUs;
    if (ahead > kMaxSleepUs) ahead = kMaxSleepUs;
    const int64_t us = ahead - kGuardUs - ahead / 128;
    if (us < kMinSleepUs) {
      d.why = SOON;
    } else {
      d.sleep = true;
      d.us = (int32_t)us;
      d.why = ASLEEP;
    }
  }
  return d;
}

// The time of day after a nap.  `wallBefore` and `monoBefore` are the time of day and the chip's timer just before,
// `wallAfter` and `monoAfter` just after, all in microseconds.  The timer was put forward by the time asleep, so the
// time of day should have moved by as much; what the SDK made of it differs from that by the RC oscillator's error
// over the time awake since the last wake-up.  add() returns what to add to the time of day now: 0 until the
// differences add up to kWallStepUs (setting the time of day costs a few microseconds, which would add up if it were
// done on every wake-up), then all of it.
class WallFix {
 public:
  int64_t add(int64_t wallBefore, int64_t monoBefore, int64_t wallAfter, int64_t monoAfter) {
    pending_ += (wallBefore + (monoAfter - monoBefore)) - wallAfter;
    if (pending_ < kWallStepUs && pending_ > -kWallStepUs) return 0;
    const int64_t fix = pending_;
    pending_ = 0;
    fixes_++;
    const int64_t mag = fix < 0 ? -fix : fix;
    if (mag > largestUs_) largestUs_ = mag;
    return fix;
  }
  int64_t pendingUs() const { return pending_; }
  uint32_t fixes() const { return fixes_; }
  int64_t largestUs() const { return largestUs_; }

 private:
  int64_t pending_ = 0;
  uint32_t fixes_ = 0;
  int64_t largestUs_ = 0;
};

// One nap, as it went.
struct Nap {
  bool slept = false;     // it slept and came back
  bool byButton = false;  // ... woken by KEY or BOOT rather than by the timer
  bool pressed = false;   // a button went down just before: no sleep at all (the loop answers it)
  int err = 0;            // what the SDK said when it would not sleep (0 = it did)
  int64_t us = 0;         // how long it took by the chip's timer: asleep, and on the way in and out
  int64_t wallFixUs = 0;  // what was added to the time of day afterwards (WallFix)
};

// The steps of one nap of `us` microseconds, against `hw`:
//   wakeOnButtons(true)   KEY and BOOT become the way out (a low level); their interrupts are held off meanwhile,
//                         as the wake-up level would call them without end
//   buttonDown()          either is down now, or a change of one was recorded and not yet taken by the loop (read
//                         after the line above: from there on a press wakes the chip)
//   sleep(us)             the light sleep, ended by the timer or a button: 0, or the SDK's error when it would not
//   wokeByButton()        it was a button that ended it
//   wakeOnButtons(false)  as they were: the interrupts back, and a button that is down by then recorded as pressed
//   monoUs(), wallUs()    the chip's timer (it includes the time asleep) and the time of day
//   shiftWallUs(d)        adds d to the time of day
// (`fix` is a WallFix; the tests also hand it wrong ones, to show that their checks can fail)
template <class Hw, class Fix>
Nap napOnce(Hw &hw, Fix &fix, int32_t us) {
  Nap n;
  hw.wakeOnButtons(true);
  if (hw.buttonDown()) {
    hw.wakeOnButtons(false);
    n.pressed = true;
    return n;
  }
  const int64_t wall0 = hw.wallUs(), mono0 = hw.monoUs();
  n.err = hw.sleep(us);
  const int64_t mono1 = hw.monoUs(), wall1 = hw.wallUs();
  n.slept = n.err == 0;
  n.byButton = n.slept && hw.wokeByButton();
  hw.wakeOnButtons(false);
  n.us = mono1 - mono0;
  if (n.slept) {  // (a sleep the SDK refused leaves the time of day alone)
    n.wallFixUs = fix.add(wall0, mono0, wall1, mono1);
    if (n.wallFixUs != 0) hw.shiftWallUs(n.wallFixUs);
  }
  return n;
}

// The share of the time spent asleep, since the loop first asked (the Power and settings page, the power log).
class Stats {
 public:
  void begin(int64_t nowUs) {
    if (begun_) return;
    begun_ = true;
    sinceUs_ = nowUs;
  }
  void add(const Nap &n) {
    if (n.slept) {
      asleepUs_ += n.us > 0 ? n.us : 0;
      naps_++;
      if (n.byButton) buttonWakes_++;
    } else if (n.err != 0) {
      refused_++;
    }
  }
  // Tenths of a percent, 0 to 1000.
  uint16_t permille(int64_t nowUs) const {
    const int64_t span = nowUs - sinceUs_;
    if (!begun_ || span <= 0) return 0;
    const int64_t p = asleepUs_ * 1000 / span;
    return (uint16_t)(p > 1000 ? 1000 : (p < 0 ? 0 : p));
  }
  uint32_t naps() const { return naps_; }
  uint32_t buttonWakes() const { return buttonWakes_; }
  uint32_t refused() const { return refused_; }

 private:
  bool begun_ = false;
  int64_t sinceUs_ = 0, asleepUs_ = 0;
  uint32_t naps_ = 0, buttonWakes_ = 0, refused_ = 0;
};

// Why it is awake right now, for the Power and settings page; "" when that is just the way of a second (it sleeps).
inline const char *awakeText(Why why) {
  switch (why) {
    case RADIO: return "radio on";
    case CONSOLE: return "console on";
    case BUTTON: return "button pressed";
    case BUSY: return "busy";
    case REFUSED: return "SDK refused";
    default: return "";  // ASLEEP, SOON, WORK: it sleeps as much as there is; OFF has no row
  }
}

const size_t kRow = 43;  // an info page shows 42 characters of a value

// The Sleep row of the Power and settings page: "78.3 % of the time", and why not now when that is worth saying
// ("78.3 % of the time (now: radio on)").
inline void formatRow(uint16_t permille, Why why, char out[kRow]) {
  if (permille > 1000) permille = 1000;
  const char *now = awakeText(why);
  snprintf(out, kRow, "%u.%u %% of the time%s%s%s", (unsigned)(permille / 10), (unsigned)(permille % 10), now[0] ? " (now: " : "",
           now, now[0] ? ")" : "");
}

}  // namespace lightsleep
