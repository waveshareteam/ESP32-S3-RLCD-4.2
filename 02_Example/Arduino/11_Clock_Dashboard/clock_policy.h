#pragma once

// Which CPU clock the clock runs at: the working clock (cpu_mhz) while something needs it, the idle clock
// (cpu_idle_mhz) when nothing does.  What needs speed:
//   * the radio: WiFi works from 80 MHz up, so the clock is raised before it is switched on and lowered
//     after it is off;
//   * a computer on the USB port: below 80 MHz the clock of some peripherals follows the CPU, and the USB
//     console may stop, which would look like a crash when someone is watching the log;
//   * a button press, for a few seconds: a frame takes four times as long to draw at 20 MHz as at 80, and
//     the person pressing wants the answer quickly;
//   * light sleep between the frames, all the time (light_sleep = on, and the console shut down, as it keeps the
//     chip awake for as long as it runs: Needs::sleeps).  The idle clock saves only while the chip idles awake.
//     Asleep it draws about a quarter of a milliamp whatever the clock, and what it does in a second (the send, the
//     draw, the slow work) takes a quarter of the time at 80 MHz that it takes at 20, so it is back asleep sooner: a
//     frame is some 180 ms awake at 20 MHz at about 14 mA, against 45 ms at about 28 mA at 80 and the rest of the time
//     asleep, so about half the charge.  (Idle, the chip draws 22 mA at 80 MHz by the datasheet, and 12.5 at 20 by
//     the difference the owner's meter showed between the two; what working adds is the datasheet's.  Not measured
//     with light sleep.)  So with light sleep the idle clock is used only while the console runs (the chip never
//     sleeps then), and not at all once it is shut down.
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

// After a button press the clock stays fast for pokeMs(): kPokeAwakeMs for a chip that stays awake between the frames
// (light_sleep = off, or the console on), as it always has, and kPokeMs for one that sleeps between them.  For that
// one the clock does not go down again at all: target() keeps the working clock by Needs::sleeps alone, before the
// first press and a day after the last one as well, so with `sleeps` kPokeMs means "no end".  It is a day because
// that is how the models of the loop read it (tools/tests/test_radio.cpp, and the battery bench, which take the time
// the clock stays fast after a press from kPokeMs); millis() compares right across it (up to 24.8 days).
const uint32_t kPokeAwakeMs = 8000;
const uint32_t kPokeMs = 86400000;
inline uint32_t pokeMs(bool sleeps) { return sleeps ? kPokeMs : kPokeAwakeMs; }
const int kRadioMinMhz = 80;  // WiFi works from this clock up

struct Needs {
  bool radio = false;    // the radio is on or about to be
  bool usbHost = false;  // a computer is on the USB port
  bool poked = false;    // a button was pressed a moment ago (in the last pokeMs())
  bool sleeps = false;   // light_sleep = on and the console is off: the chip sleeps between the frames
};

// The clock to run at, in MHz.  No idle clock (0), or one that is not below the working clock, means "no change".
inline int target(const Needs &n, int workMhz, int idleMhz) {
  if (idleMhz <= 0 || idleMhz >= workMhz) return workMhz;
  return (n.radio || n.usbHost || n.poked || n.sleeps) ? workMhz : idleMhz;
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
