#pragma once

// Which CPU clock the clock runs at: the working clock (cpu_mhz) while something needs it, the idle clock
// (cpu_idle_mhz) when nothing does.  What needs speed:
//   * the radio: WiFi works from 80 MHz up, so the clock is raised before it is switched on and lowered
//     after it is off;
//   * a computer on the USB port: below 80 MHz the clock of some peripherals follows the CPU, and the USB
//     console may stop, which would look like a crash when someone is watching the log;
//   * a button press, for a few seconds: a frame takes four times as long to draw at 20 MHz as at 80, and
//     the person pressing wants the answer quickly.
//
// Pure logic with no Arduino dependency; the UI loop (which owns the display, and so the SPI bus the core
// has to be told about) applies it, see powerSetCpuMhz().
//
// The radio and the clock are in two tasks: the network task wants the radio, the UI loop sets the clock.  So
// the network task does not look at the clock and go (the loop may have decided a moment before to lower it, and
// lower it just after; or be on its way into a light sleep, which stops both cores): it asks, and waits for the
// loop to answer that it has seen the request and the clock is where it will stay while the radio needs it
// (radioMayStart(); net_task.cpp has the two sides, radio_plan.h the wait, ClockWait).

#include <stdint.h>

namespace clockpolicy {

const uint32_t kPokeMs = 8000;  // after a button press the clock stays fast this long
const int kRadioMinMhz = 80;    // WiFi works from this clock up

struct Needs {
  bool radio = false;    // the radio is on or about to be
  bool usbHost = false;  // a computer is on the USB port
  bool poked = false;    // a button was pressed a moment ago
};

// The clock to run at, in MHz.  No idle clock (0), or one that is not below the working clock, means "no change".
inline int target(const Needs &n, int workMhz, int idleMhz) {
  if (idleMhz <= 0 || idleMhz >= workMhz) return workMhz;
  return (n.radio || n.usbHost || n.poked) ? workMhz : idleMhz;
}

// May the clock change now?  Up at once (something is waiting for it); down only when the frame planner is
// quiet (frame_plan.h): no frame is being drawn, and the send of the one that waits is far enough off for it to
// go out with the send cost of the new clock.
inline bool mayChange(int currentMhz, int targetMhz, bool quiet) {
  if (targetMhz == currentMhz) return false;
  return targetMhz > currentMhz || quiet;
}

// May the UI loop tell the radio that it can come on?  Once a time round, after the clock was set: `radio` is what
// the network task asked for, read in the same pass before the clock was set (so `targetMhz` was worked out with
// it).  Yes when the clock is where WiFi works and where it stays for as long as the radio needs it: at the clock
// the policy wants, or at the one it is at because the core refused that (`refusedMhz`, 0 = none).  Not while it
// is still to be raised: from 80 to 240 MHz, say, which would change the clock under a radio that is starting.
inline bool radioMayStart(bool radio, int currentMhz, int targetMhz, int refusedMhz) {
  return radio && currentMhz >= kRadioMinMhz && (currentMhz == targetMhz || (refusedMhz != 0 && targetMhz == refusedMhz));
}

}  // namespace clockpolicy
