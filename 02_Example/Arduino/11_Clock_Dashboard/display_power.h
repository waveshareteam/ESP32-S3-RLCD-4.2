#pragma once

// display_power = low: the ST7305 drives the panel in its low power mode (LPM), refreshing it 8 times a second, in
// place of its high power mode (HPM) at 25.5 times a second.
//
// From the datasheet (Sitronix ST7305, version 0.2, 2021/04; Waveshare hosts a copy):
//  * 38h and 39h switch the panel drive to HPM and LPM.  After a reset the chip is in HPM.
//  * FRCTRL (B2h): bit 4 (HFRA) is the frame rate in HPM, bits 2-0 (LFRA) the one in LPM: 0.25, 0.5, 1, 2, 4 or
//    8 Hz.  With OSCSET (D8h) = 0x80, which the board's init sends, HFRA 0 is 25.5 Hz.  Waveshare's init sends
//    B2h = 0x02: 25.5 Hz in HPM, 1 Hz in LPM (which it never enters).
//  * Its tearing-effect timings have the panel reading the frame memory in both modes: 30 ms of every 31.2 at 32 Hz
//    in HPM, 990.5 ms of every 1000 at 1 Hz in LPM.
//  * Section 7.11 (a figure) gives the way from one mode to the other: HPM => LPM is the source voltages for LPM
//    (C1h, C2h, C4h, C5h, then C9h), 20 ms, 39h, 100 ms; LPM => HPM is 38h, 300 ms, the voltages for HPM, 20 ms.  The
//    board's init writes the same values into all four voltage sets and selects the first (C9h = 0), so here the
//    voltages "for LPM" are the same ones sent again.
//  * Sitronix's own sample init ends in LPM.  Nothing in the datasheet gives the current in either mode.
//
// What follows from that and is not written there: a frame is on the glass when the panel next refreshes its rows,
// 0 to 39 ms after the transfer in HPM and 0 to 125 ms after it at 8 Hz.  The clock times its frames for the middle
// of that (frame_plan.h), so in LPM the seconds change up to about 63 ms early or late instead of 20.  8 Hz is the
// fastest LPM goes; 4 Hz would make that 125 ms.  Whether the picture keeps its contrast at 8 Hz, and how much
// current it saves, has to be seen on the board.
//
// Pure logic with no Arduino dependency: ST7305_U8g2 sends the steps, tools/tests checks them and the timing.

#include <stddef.h>
#include <stdint.h>

namespace dispower {

constexpr uint8_t kFrameRate = 0x05;      // FRCTRL (B2h): HFRA 0 = 25.5 Hz in HPM (as before), LFRA 5 = 8 Hz in LPM
constexpr int32_t kHpmFrameUs = 39216;    // one refresh in HPM: 1 / 25.5 Hz
constexpr int32_t kLpmFrameUs = 125000;   // ... in LPM: 1 / 8 Hz

// For a clock that has switched itself off (the battery-empty screen, then deep sleep for as long as the cell is low,
// which can be days): LPM at its slowest, whatever display_power says.  The picture does not change again until a
// wake-up starts the clock afresh, and fullInit() then sets kFrameRate back, so the refresh only has to keep it there.
constexpr uint8_t kRestFrameRate = 0x00;  // FRCTRL (B2h): HFRA 0 as in kFrameRate, LFRA 0 = 0.25 Hz in LPM

// How much later a frame is on the glass in LPM than in HPM, on average: half the one refresh less half the other.
constexpr int32_t kExtraLatencyUs = (kLpmFrameUs - kHpmFrameUs) / 2;

// The time from the end of a transfer to the frame being on the glass, on average, that the frame planner works
// with: `hpmUs` (DISPLAY_LATENCY_MS) in HPM, that and the longer refresh in LPM.
inline int32_t latencyUs(bool low, int32_t hpmUs) { return low ? hpmUs + kExtraLatencyUs : hpmUs; }

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
