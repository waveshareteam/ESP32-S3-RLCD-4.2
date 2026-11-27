#pragma once

// display_power = low: the ST7305 drives the panel in its low power mode (LPM), refreshing it display_low_hz times a
// second at rest (1, 2, 4 or 8; 2 unless set), and 8 times a second for the seconds after a button press, in place of
// its high power mode (HPM) at 25.5 times a second.
//
// From the datasheet (Sitronix ST7305, version 0.2, 2021/04; Waveshare hosts a copy):
//  * 38h and 39h switch the panel drive to HPM and LPM.  After a reset the chip is in HPM.
//  * FRCTRL (B2h): bit 4 (HFRA) is the frame rate in HPM, bits 2-0 (LFRA) the one in LPM: 0.25, 0.5, 1, 2, 4 or
//    8 Hz.  With OSCSET (D8h) = 0x80, which the board's init sends, HFRA 0 is 25.5 Hz.  Waveshare's init sends
//    B2h = 0x02: 25.5 Hz in HPM, 1 Hz in LPM (which it never enters).  kFrameRate, what the clock's init sends, has
//    the default rate in LPM instead, 2 Hz (0x03); setLowRate() writes the one in force.
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
// 0 to 39 ms after the transfer in HPM and 0 to one refresh after it in LPM, 0 to 500 ms at 2 Hz.  The clock times
// its frames for the middle of that (frame_plan.h), so in LPM the seconds change up to half a refresh early or late
// instead of 20 ms: about 250 ms at 2 Hz, 500 at 1, 125 at 4 and 63 at 8.  Up to 1.13 LPM ran at 8 Hz, the fastest
// it goes.  The rate at rest is a setting so that the rates can be compared on the board with a meter: drawn as a
// straight line through the only two published figures (README: "The display's low power mode") the controller draws
// about 0.08 mA for each refresh a second, so 4 Hz would be some 0.32 mA less than 8 at the rail, 2 Hz 0.16 less
// than 4, and 1 Hz 0.08 less than 2.
// 2 Hz is the default because 1 Hz has a cost the others have not.  The refresh is timed by the panel's own
// oscillator and the seconds by the crystal, and at 1 Hz there is one refresh for each second, so the two drift past
// each other.  If the panel's second is longer than the crystal's, now and then a second is never shown (the seconds
// go from 17 to 19): one every 1 / x seconds for an oscillator x slow, so one every 50 s at 2 %.  If it is shorter,
// now and then a second stays on the glass for two refreshes.  At 2 Hz and faster every second is in the frame memory
// for two refreshes or more, so none can be lost (the dropped-second model in tools/tests shows both).  The datasheet
// gives no tolerance for the oscillator, and does not say whether a transfer starts a refresh afresh (which would end
// the drift), so only the board can say how often it happens (TESTING.md, item 38, counts them at 1 Hz).
// So that a press is still answered quickly, for kBusyMs after a button the rate is 8 Hz and the frames are timed for
// it; it goes back to the rate at rest once that time is over, at a moment the frame that waits has the time for the
// longer lead (mayRest()).  (With light sleep the CPU clock stays fast for good, clock_policy.h, and the 8 Hz still
// lasts only these seconds: the two have their own times.)  At display_low_hz = 8 the two rates are one.
// None of it is measured, and nobody has seen the low power mode on the board at any rate.  Whether the picture keeps
// its contrast, whether a second is ever missed at 1 Hz, whether a change of the rate shows at the next refresh and
// leaves no mark, and how much current each rate saves, has to be seen on the board.
//
// Pure logic with no Arduino dependency: ST7305_U8g2 sends the steps, tools/tests checks them and the timing.

#include <stddef.h>
#include <stdint.h>

#include "frame_plan.h"

namespace dispower {

// display_low_hz: the rate in LPM at rest, 1, 2, 4 or 8 Hz (LFRA 2 to 5).  0.25 and 0.5 Hz (LFRA 0 and 1) are not on
// offer: with fewer refreshes than seconds, most seconds would never be shown.
constexpr int kLowHzDefault = 2;  // (DISPLAY_LOW_HZ in config.h; kFrameRate, kLpmFrameUs and the overloads without a rate are for it)
constexpr bool lowHzValid(int hz) { return hz == 1 || hz == 2 || hz == 4 || hz == 8; }
constexpr int checkedLowHz(int hz) { return lowHzValid(hz) ? hz : kLowHzDefault; }  // (a rate not on offer is the default)

constexpr uint8_t kBusyFrameRate = 0x05;     // FRCTRL (B2h): HFRA 0 = 25.5 Hz in HPM (as before), LFRA 5 = 8 Hz: the seconds after a press
constexpr int32_t kHpmFrameUs = 39216;       // one refresh in HPM: 1 / 25.5 Hz
constexpr int32_t kLpmBusyFrameUs = 125000;  // ... in LPM after a press: 1 / 8 Hz
constexpr uint32_t kBusyMs = 8000;  // after a button press the faster rate lasts this long (and then to a moment with
                                    // the time for the slower one, mayRest())

// The rate in LPM, at rest (display_low_hz, `lowHz`) or after a press (`busy`): B2h (the same HFRA, and the LFRA of
// the rate: 1 Hz is 2, 2 Hz 3, 4 Hz 4, 8 Hz 5), and one refresh.
constexpr uint8_t frameRate(bool busy, int lowHz) {
  return busy ? kBusyFrameRate
              : (uint8_t)(checkedLowHz(lowHz) == 1 ? 2 : checkedLowHz(lowHz) == 2 ? 3 : checkedLowHz(lowHz) == 4 ? 4 : 5);
}
constexpr int32_t lpmFrameUs(bool busy, int lowHz) { return busy ? kLpmBusyFrameUs : 1000000 / checkedLowHz(lowHz); }
constexpr uint8_t frameRate(bool busy) { return frameRate(busy, kLowHzDefault); }
constexpr int32_t lpmFrameUs(bool busy) { return lpmFrameUs(busy, kLowHzDefault); }

constexpr uint8_t kFrameRate = frameRate(false);    // 0x03, what fullInit() sends: 25.5 Hz in HPM, 2 Hz in LPM
constexpr int32_t kLpmFrameUs = lpmFrameUs(false);  // one refresh in LPM at the default rate: 1 / 2 Hz

// For a clock that has switched itself off (the battery-empty screen, then deep sleep for as long as the cell is low,
// which can be days): LPM at its slowest, whatever display_power says.  The picture does not change again until a
// wake-up starts the clock afresh, and fullInit() then sets kFrameRate back, so the refresh only has to keep it there.
constexpr uint8_t kRestFrameRate = 0x00;  // FRCTRL (B2h): HFRA 0 as in kFrameRate, LFRA 0 = 0.25 Hz in LPM

// How much later a frame is on the glass in LPM than in HPM, on average: half the one refresh less half the other.
constexpr int32_t extraLatencyUs(bool busy, int lowHz) { return (lpmFrameUs(busy, lowHz) - kHpmFrameUs) / 2; }
constexpr int32_t kExtraLatencyUs = extraLatencyUs(false, kLowHzDefault);     // at rest, at the default rate: 230.4 ms
constexpr int32_t kBusyExtraLatencyUs = extraLatencyUs(true, kLowHzDefault);  // after a press: 42.9 ms

// The time from the end of a transfer to the frame being on the glass, on average, that the frame planner works
// with: `hpmUs` (DISPLAY_LATENCY_MS) in HPM, that and the longer refresh in LPM, the one at rest (`lowHz`) or after a
// press.
inline int32_t latencyUs(bool low, int32_t hpmUs, bool busy, int lowHz) { return low ? hpmUs + extraLatencyUs(busy, lowHz) : hpmUs; }
inline int32_t latencyUs(bool low, int32_t hpmUs, bool busy) { return latencyUs(low, hpmUs, busy, kLowHzDefault); }
inline int32_t latencyUs(bool low, int32_t hpmUs) { return latencyUs(low, hpmUs, false); }  // (at rest, at the default rate)

// Does B2h have to be written for the rate at rest (`lowHz`) or after a press (`busy`), on a panel whose B2h holds
// `current`?  Only in LPM (`inLpm`): in HPM the rate of LPM does nothing, and a switch into LPM is followed by this.
// 38h and 39h leave B2h as it is (setRestMode() relies on that too), so what was written last is what the panel has.
inline bool rateNeedsWrite(bool inLpm, uint8_t current, bool busy, int lowHz) { return inLpm && current != frameRate(busy, lowHz); }
inline bool rateNeedsWrite(bool inLpm, uint8_t current, bool busy) { return rateNeedsWrite(inLpm, current, busy, kLowHzDefault); }

// Is the panel (in LPM, B2h holding `current`) at the rate for after a press, and is that another one than the rate
// at rest?  At display_low_hz = 8 the two are one, and there is nothing to go back to.
inline bool atBusyRate(bool inLpm, uint8_t current, int lowHz) {
  return inLpm && current == kBusyFrameRate && frameRate(false, lowHz) != kBusyFrameRate;
}

// How much earlier a frame is sent once the rate goes back from the one after a press to the one at rest: the
// difference of the two latencies, 187.5 ms at 2 Hz, 437.5 at 1, 62.5 at 4 and none at 8.
constexpr int32_t restLeadGrowUs(int lowHz) { return extraLatencyUs(false, lowHz) - extraLatencyUs(true, lowHz); }
constexpr int32_t kRestLeadGrowUs = restLeadGrowUs(kLowHzDefault);

// Back to the rate at rest (`lowHz`): once the kBusyMs after a press are over (`poked` false), with a frame prepared
// (FramePlanner::prepared()), and only while the wait before that frame's send is more than restLeadGrowUs() for the
// rate in force and FramePlanner::kSpinUs.  `idleUs` is FramePlanner::idleUs(), which is that wait and
// FramePlanner::kWakeLagUs (the light sleep is aimed that far past the send), so the lag is taken off again here.  By
// the longer lead its send then still has more than kSpinUs to go, so it goes out on time.  (Without light sleep the
// CPU clock may come down at the same moment, which lengthens the send's own part of the lead: serviceClock() runs
// first, so `idleUs` has that already.)  A quiet planner, the rule while the rate at rest was 2 Hz, would do at 2 Hz
// (quiet() promises more than kQuietUs, 300 ms, before the send by the busy lead, and 2 Hz takes 187.5 of it) but not
// at 1 Hz, which takes 437.5 ms: the frame could be due in the past and late.  With light sleep the moment is the first
// wake after the seconds of the press, straight after its draw (the send about 955 ms off by the busy lead, 768 by the
// lead at 2 Hz and 518 by the one at 1 Hz).  With no frame prepared the draw that comes next might not be over in
// time at the 20 MHz idle clock (without light sleep).
inline bool mayRest(bool busyNow, bool poked, bool prepared, int32_t idleUs, int lowHz) {
  return busyNow && !poked && prepared && idleUs > restLeadGrowUs(lowHz) + FramePlanner::kSpinUs + FramePlanner::kWakeLagUs;
}
inline bool mayRest(bool busyNow, bool poked, bool prepared, int32_t idleUs) {
  return mayRest(busyNow, poked, prepared, idleUs, kLowHzDefault);
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
