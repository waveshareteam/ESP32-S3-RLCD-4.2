#pragma once

// display_power = low: the ST7305 drives the panel in its low power mode (LPM), refreshing it once a second, and
// 8 times a second for the seconds after a button press, in place of its high power mode (HPM) at 25.5 times a second.
//
// From the datasheet (Sitronix ST7305, version 0.2, 2021/04; Waveshare hosts a copy):
//  * 38h and 39h switch the panel drive to HPM and LPM.  After a reset the chip is in HPM.
//  * FRCTRL (B2h): bit 4 (HFRA) is the frame rate in HPM, bits 2-0 (LFRA) the one in LPM: 0.25, 0.5, 1, 2, 4 or
//    8 Hz.  With OSCSET (D8h) = 0x80, which the board's init sends, HFRA 0 is 25.5 Hz.  Waveshare's init sends
//    B2h = 0x02: 25.5 Hz in HPM, 1 Hz in LPM (which it never enters).  kFrameRate is that value again.
//  * Its tearing-effect timings have the panel reading the frame memory in both modes: 30 ms of every 31.2 at 32 Hz
//    in HPM, 990.5 ms of every 1000 at 1 Hz in LPM.
//  * Section 7.11 (a figure) gives the way from one mode to the other: HPM => LPM is the source voltages for LPM
//    (C1h, C2h, C4h, C5h, then C9h), 20 ms, 39h, 100 ms; LPM => HPM is 38h, 300 ms, the voltages for HPM, 20 ms.  The
//    board's init writes the same values into all four voltage sets and selects the first (C9h = 0), so here the
//    voltages "for LPM" are the same ones sent again.
//  * Sitronix's own sample init ends in LPM.  Nothing in the datasheet gives the current in either mode, or says
//    when a new B2h takes effect while the panel is in LPM.
//
// What follows from that and is not written there: a frame is on the glass when the panel next refreshes its rows,
// 0 to 39 ms after the transfer in HPM and 0 to 1000 ms after it at 1 Hz.  The clock times its frames for the middle
// of that (frame_plan.h), so in LPM the seconds change up to about 500 ms early or late instead of 20.  Up to 1.13
// LPM ran at 8 Hz, the fastest it goes, with the seconds within about 63 ms, then at 4 Hz, within about 125 ms, and
// then at 2 Hz, within about 250 ms.  1 Hz, the slowest rate with a refresh for every second, is chosen for the
// current: drawn as a straight line through the only two published figures (README: "The display's low power mode")
// the controller draws about 0.08 mA for each refresh a second, so 1 Hz would be some 0.08 mA less than 2 at the rail.
// It has a cost that 2 Hz has not.  The refresh is timed by the panel's own oscillator and the seconds by the
// crystal, and at 1 Hz there is one refresh for each second, so the two drift past each other.  If the panel's second
// is longer than the crystal's, now and then a second is never shown (the seconds go from 17 to 19): one every 1 / x
// seconds for an oscillator x slow, so one every 50 s at 2 %.  If it is shorter, now and then a second stays on the
// glass for two refreshes.  At 2 Hz every second is in the frame memory for two refreshes, so none can be lost (the
// dropped-second model in tools/tests shows both).  The datasheet gives no tolerance for the oscillator, and does not
// say whether a transfer starts a refresh afresh (which would end the drift), so only the board can say how often it
// happens: the battery loop's probes.md has the probes for it (round 10, the panel's own second; round 4, whether a
// command starts the scan afresh).
// So that a press is still answered quickly, for kBusyMs after a button the rate is 8 Hz and the frames are timed for
// it; it goes back to 1 Hz once that time is over, at a moment the frame that waits has the time for the longer lead
// (mayRest()).  (With light sleep the CPU clock stays fast for good, clock_policy.h, and the 8 Hz still lasts only
// these seconds: the two have their own times.)
// None of it is measured, and nobody has seen 1 Hz on the board.  Whether the picture keeps its contrast at 1 Hz,
// whether a second is ever missed, whether a change of the rate shows at the next refresh and leaves no mark, and how
// much current it saves, has to be seen on the board.
//
// Pure logic with no Arduino dependency: ST7305_U8g2 sends the steps, tools/tests checks them and the timing.

#include <stddef.h>
#include <stdint.h>

#include "frame_plan.h"

namespace dispower {

constexpr uint8_t kFrameRate = 0x02;      // FRCTRL (B2h): HFRA 0 = 25.5 Hz in HPM (as before), LFRA 2 = 1 Hz in LPM
constexpr uint8_t kBusyFrameRate = 0x05;  // ... the same HFRA, LFRA 5 = 8 Hz: the seconds after a button press
constexpr int32_t kHpmFrameUs = 39216;    // one refresh in HPM: 1 / 25.5 Hz
constexpr int32_t kLpmFrameUs = 1000000;  // ... in LPM: 1 / 1 Hz
constexpr int32_t kLpmBusyFrameUs = 125000;  // ... in LPM after a press: 1 / 8 Hz
constexpr uint32_t kBusyMs = 8000;  // after a button press the faster rate lasts this long (and then to a moment with
                                    // the time for the slower one, mayRest())

// For a clock that has switched itself off (the battery-empty screen, then deep sleep for as long as the cell is low,
// which can be days): LPM at its slowest, whatever display_power says.  The picture does not change again until a
// wake-up starts the clock afresh, and fullInit() then sets kFrameRate back, so the refresh only has to keep it there.
constexpr uint8_t kRestFrameRate = 0x00;  // FRCTRL (B2h): HFRA 0 as in kFrameRate, LFRA 0 = 0.25 Hz in LPM

// The rate in LPM, at rest or after a press (`busy`): B2h, and one refresh.
inline uint8_t frameRate(bool busy) { return busy ? kBusyFrameRate : kFrameRate; }
inline int32_t lpmFrameUs(bool busy) { return busy ? kLpmBusyFrameUs : kLpmFrameUs; }

// How much later a frame is on the glass in LPM than in HPM, on average: half the one refresh less half the other.
constexpr int32_t kExtraLatencyUs = (kLpmFrameUs - kHpmFrameUs) / 2;          // at rest
constexpr int32_t kBusyExtraLatencyUs = (kLpmBusyFrameUs - kHpmFrameUs) / 2;  // after a press

// The time from the end of a transfer to the frame being on the glass, on average, that the frame planner works
// with: `hpmUs` (DISPLAY_LATENCY_MS) in HPM, that and the longer refresh in LPM, the one at rest or after a press.
inline int32_t latencyUs(bool low, int32_t hpmUs, bool busy) {
  return low ? hpmUs + (busy ? kBusyExtraLatencyUs : kExtraLatencyUs) : hpmUs;
}
inline int32_t latencyUs(bool low, int32_t hpmUs) { return latencyUs(low, hpmUs, false); }  // (at rest)

// Does B2h have to be written for the rate at rest or after a press (`busy`), on a panel whose B2h holds `current`?
// Only in LPM (`inLpm`): in HPM the rate of LPM does nothing, and a switch into LPM is followed by this.  38h and 39h
// leave B2h as it is (setRestMode() relies on that too), so what was written last is what the panel has.
inline bool rateNeedsWrite(bool inLpm, uint8_t current, bool busy) { return inLpm && current != frameRate(busy); }

// How much earlier a frame is sent once the rate goes back from the one after a press to the one at rest: 437.5 ms.
constexpr int32_t kRestLeadGrowUs = kExtraLatencyUs - kBusyExtraLatencyUs;

// Back to the rate at rest: once the kBusyMs after a press are over (`poked` false), with a frame prepared
// (FramePlanner::prepared()), and only while the wait before that frame's send is more than kRestLeadGrowUs and
// FramePlanner::kSpinUs.  `idleUs` is FramePlanner::idleUs(), which is that wait and FramePlanner::kWakeLagUs (the
// light sleep is aimed that far past the send), so the lag is taken off again here.  By the longer lead its send then
// still has more than kSpinUs to go, so it goes out on time.  (Without light sleep the CPU clock may come down at the
// same moment, which lengthens the send's own part of the lead: serviceClock() runs first, so `idleUs` has that
// already.)  A quiet planner, the rule up to 2 Hz, is not enough: quiet() promises more than kQuietUs (300 ms) before
// the send by the busy lead, and 1 Hz takes 437.5 ms more, so the frame could be due in the past and late.  With light
// sleep the moment is the first wake after the seconds of the press, straight after its draw (the send about 955 ms
// off by the busy lead, 520 by the one at rest).  With no frame prepared the draw that comes next might not be over in
// time at the 20 MHz idle clock (without light sleep).
inline bool mayRest(bool busyNow, bool poked, bool prepared, int32_t idleUs) {
  return busyNow && !poked && prepared && idleUs > kRestLeadGrowUs + FramePlanner::kSpinUs + FramePlanner::kWakeLagUs;
}

// One step of a switch between the modes: a command without parameters, or the source voltages (VOLTAGES, which
// the driver sends with their parameters), then a wait.
constexpr uint8_t VOLTAGES = 0;
struct Step {
  uint8_t cmd;
  uint16_t waitMs;
};
constexpr size_t kMaxSteps = 2;

// The datasheet's sequence (7.11) into LPM (`toLow`) or out of it.  Returns the number of steps.
inline size_t switchSteps(bool toLow, Step out[kMaxSteps]) {
  if (toLow) {
    out[0] = {VOLTAGES, 20};
    out[1] = {0x39, 100};
  } else {
    out[0] = {0x38, 300};
    out[1] = {VOLTAGES, 20};
  }
  return 2;
}

}  // namespace dispower
