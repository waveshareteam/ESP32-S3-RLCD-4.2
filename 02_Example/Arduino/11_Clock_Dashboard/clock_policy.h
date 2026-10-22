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

#include <stdint.h>

namespace clockpolicy {

const uint32_t kPokeMs = 8000;  // after a button press the clock stays fast this long

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

// May the clock change now?  Up at once (something is waiting for it); down only between frames, when
// nothing is drawn or waiting to be sent, so that no frame is planned with the old costs.
inline bool mayChange(int currentMhz, int targetMhz, bool quiet) {
  if (targetMhz == currentMhz) return false;
  return targetMhz > currentMhz || quiet;
}

}  // namespace clockpolicy
