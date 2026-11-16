#pragma once

// When the USB serial console (the log and the commands) is shut down: "console = on | auto | off".
//
//   on    never.
//   auto  when no computer has been on the USB port for kNoHostMs, counted from start-up: a clock on a battery
//         or a charger drops the console ten seconds after it started, one on a computer keeps it until the
//         cable is pulled.
//   off   at once, which is the end of start-up (the first time round the loop).
//
// Once shut down it stays down until the next restart: with the USB transceiver off there is no way to see a
// computer arrive.  A restart with KEY held down keeps the console for that run whatever the setting (`keep`),
// so a clock set to "off" can still be talked to and flashed without an SD card.
//
// Pure logic with no Arduino dependency; the sketch does the shutting down (log.cpp).

#include <stdint.h>

namespace consolepolicy {

enum Mode : uint8_t { ON = 0, AUTO, OFF };  // the values of the "console" setting (ConsoleMode in settings.h)

const uint32_t kNoHostMs = 10000;

class Watch {
 public:
  // Call every time round the loop while the console runs.  `hostNow`: a computer is on the USB port right
  // now.  True: shut the console down now.
  bool shouldStop(int mode, bool keep, bool hostNow, uint32_t nowMs) {
    if (!started_ || hostNow) {  // the count starts with the first call, and again whenever a computer is seen
      started_ = true;
      lastHostMs_ = nowMs;
    }
    if (keep || mode == ON) return false;
    if (mode == OFF) return true;
    return (uint32_t)(nowMs - lastHostMs_) >= kNoHostMs;
  }

 private:
  bool started_ = false;
  uint32_t lastHostMs_ = 0;
};

}  // namespace consolepolicy
