#pragma once

// When to draw a second and when to send it to the panel.
//
// Drawing a frame takes tens of milliseconds (four times as long at the 20 MHz idle
// clock) and the time varies (cache misses, the WiFi stack taking the memory bus, a
// flash write on the other core), while sending it is short.  So every second is drawn
// into the display buffer a second ahead, straight after the one before it has been
// sent, and sent just before it starts: the draw may run long by most of a second
// without the digits changing late, and the send and the draw are one stretch of work,
// so with light sleep (light_sleep.h) the chip wakes once a second, for the send
// (idleUs()).  The panel refreshes itself at 25.5 Hz, so a frame shows up
// 0-39 ms after it has been sent (with display_power = low 0 to one refresh at its
// display_low_hz, 0-500 ms at 2 Hz, and 0-125 ms at its 8 Hz after a button press,
// display_power.h); the send is timed to end `latencyUs` before the second starts, which
// puts the average error at zero.  (After a light sleep it goes out a few milliseconds
// later on purpose: kWakeLagUs.)
//
// What the frame shows besides the time (the weather, the sensors, the battery) is
// taken when it is drawn, so up to a second before it is on the glass; whatever has to
// show at once forces a redraw.
//
// Slow work (the sensors, the battery, a line of the power log) is kept away from the
// sends: it may run while the planner is quiet (quiet()), from the end of a draw until
// kQuietUs before the send of the frame just drawn, whichever second that is for.  So
// with light sleep it runs in the wake of the draw, and needs no wake of its own.
//
// Pure logic with no Arduino dependency: the UI loop asks next() whenever it wakes up
// and does what it says; tools/tests drives it with simulated clocks and stalls.

#include <stdint.h>
#include <time.h>

class FramePlanner {
 public:
  enum Action : uint8_t {
    IDLE,      // nothing to do yet, and the send is far off (quiet()): slow work may run
    DRAW_NOW,  // draw `sec`, then send it at once (something has to be on screen now)
    PREPARE,   // draw `sec` into the buffer, but do not send it yet
    SEND,      // wait `waitUs` (at most kSpinUs), then send the prepared frame
    SLEEP,     // the prepared frame is due in `waitUs`, too soon for slow work: sleep a little and ask again
  };
  struct Step {
    Action action = IDLE;
    time_t sec = 0;
    int32_t waitUs = 0;
  };

  static constexpr int32_t kSpinUs = 5000;      // waits shorter than this are spun, longer ones slept
  // Light sleep is aimed this far past the send's moment (idleUs()), so that a wake-up is never early and nothing is
  // spun before the send: the sleep's guard (0.2 % of it, 1.9 ms of a sleep of 950 ms, light_sleep.h), the RC clock's
  // 0.2 % and the millisecond a wake-up may come early add up to 4.8 ms in the simulation.
  static constexpr int32_t kWakeLagUs = 5000;
  static constexpr int32_t kQuietUs = 300000;   // slow work only with more than this left before the send

  explicit FramePlanner(int32_t latencyUs) : latencyUs_(latencyUs) {}

  // `sec` / `usec`: the clock now.  `force`: something has to be on the glass immediately
  // (a key press, a toast, a page change).
  Step next(time_t sec, int32_t usec, bool force) {
    const int32_t usToNext = 1000000 - usec;
    Step st;
    nowSec_ = sec;  // (for quiet(usec))
    nowUsec_ = usec;

    // A frame prepared for a second that is over, or for one more than two ahead, means the clock was stepped (the
    // hourly time sync) or the loop stalled for a long while: it is dropped, and the second that it is now is drawn at
    // once (below), so an old one is not sent first.  (A frame for the second that has just begun is sent, late.)
    if (prepared_ && (preparedSec_ < sec || preparedSec_ > sec + 2)) prepared_ = false;

    if (force) {
      // A frame drawn and sent now is on the glass `cost` and `latencyUs_` from now.  Close to a tick
      // there would be no time left afterwards to prepare the next second the usual way (a draw, and
      // a send that ends `latencyUs_` before the tick), so show the second that is about to start
      // instead: early beats a late second.  The window is 2 * cost + latencyUs_ before the tick,
      // 110 ms at 80 MHz in the high power mode, and with display_power = low at 2 Hz about 340 ms at
      // 80 MHz (the clock with light sleep) and 610 at 20 (590 and 860 at 1 Hz), but the longer
      // latency moves the scheduled sends just as far, so on average such a frame is at most `cost`
      // early whatever the latency (45 ms at 80 MHz, 180 at 20), and the
      // refresh's own spread comes on top (up to 20 ms either way at 25.5 Hz, 250 at 2 Hz).  (A press
      // has the clock at 80 MHz and the panel at 8 Hz: 153 ms, and up to 63 ms either way.)  Nor does
      // it go back to the second before the coming one once that has been sent: after a switch to a
      // shorter latency (a press with display_power = low) it can have gone out by the longer lead,
      // before the window that is in force now.
      const int32_t cost = (int32_t)(drawCostUs_ + sendCostUs_);
      st.action = DRAW_NOW;
      st.sec = usToNext <= 2 * cost + latencyUs_ || lastSentSec_ == sec + 1 ? sec + 1 : sec;
      return st;
    }
    if (prepared_) {
      const int32_t waitUs = untilSendUs(sec, usec);  // 0 or less: due (its second may have begun)
      if (waitUs <= kSpinUs) {
        st.action = SEND;
        st.waitUs = waitUs > 0 ? waitUs : 0;
      } else {
        st.action = quiet(sec, usec) ? IDLE : SLEEP;
        st.waitUs = waitUs;
      }
      return st;
    }
    if (lastSentSec_ == sec || lastSentSec_ == sec + 1) {  // the one after the second on the glass, drawn at once
      st.action = PREPARE;
      st.sec = lastSentSec_ + 1;
      return st;
    }
    st.action = DRAW_NOW;  // behind (start-up, a long stall, a step of the clock): catch up at once
    st.sec = sec;
    return st;
  }

  // How long the loop may sleep (light_sleep.h) before it asks again, if nothing is forced meanwhile: until kWakeLagUs
  // past the send's moment of the prepared frame; 0 when it asks for something now (with no frame prepared it always
  // does, PREPARE or DRAW_NOW, and with one it asks for the send once the wait is down to kSpinUs).  next() asks for
  // nothing but IDLE or SLEEP until kSpinUs before the send's moment, then for the SEND, with no wait at all once the
  // moment has come.  So a sleep that wakes up to kWakeLagUs early (in the simulation none of up to a second wakes
  // earlier) sends at once, late on purpose by what is left of kWakeLagUs, or by more if it wakes late (light_sleep.h
  // says how much that can be); one that wakes earlier still (a sleep of over a second, or a board that differs from
  // the simulation) is spun into the send (SEND) or naps a little more first (SLEEP), as when the sleep was aimed at
  // the send itself.
  int32_t idleUs(time_t sec, int32_t usec) const {
    if (!prepared_ || preparedSec_ < sec || preparedSec_ > sec + 2) return 0;  // (a frame next() drops counts as none)
    const int32_t waitUs = untilSendUs(sec, usec);
    return waitUs > kSpinUs ? waitUs + kWakeLagUs : 0;  // (from kSpinUs on next() asks for the send)
  }

  // May slow work (I2C sensors, battery, a line of the power log) run now, `sec` / `usec` being the clock?  Never
  // close to a send: with a frame prepared, only while its send is more than kQuietUs away, for whichever second it
  // is.  With display_power = low the send begins earlier by half a refresh (at 2 Hz about 265 ms before the tick at
  // 80 MHz, the clock with light sleep, and 310 at the 20 MHz idle clock; at 1 Hz 515 and 560), so the draw after it
  // ends before the tick, with its own send most of a second off: slow work that is due runs straight after it, in the
  // same wake.  (This used to take a prepared frame to go out in this second, so slow work waited for the tick there,
  // and with light sleep that was a sleep of its own every 5 s.  In the high power mode the draw ends after the tick,
  // and the two rules agree.)  A frame prepared for a second that is over, or for one more than two ahead (next() drops
  // it), is never quiet; with none prepared, the tick is what counts.  With display_power = low that leaves slow work
  // about 655 ms of every second at 80 MHz, at 2 Hz from some 220 ms before the tick to 435 ms after it, and 520 ms at
  // 20 MHz, from some 130 ms before the tick to 390 ms after it (as long at any rate, and earlier in the second the
  // slower the rate).
  bool quiet(time_t sec, int32_t usec) const {
    if (!prepared_) return 1000000 - usec > kQuietUs;
    if (preparedSec_ < sec || preparedSec_ > sec + 2) return false;  // (which keeps untilSendUs() well in range)
    return untilSendUs(sec, usec) > kQuietUs;
  }
  // The same, for the second the last next() was asked in, or the one after it once `usec` has gone back past the
  // tick since: right while no more than a second goes by between the two, as in a loop that asks next() every time
  // round; after a longer stall it is a second out.  (The firmware passes the second itself.)
  bool quiet(int32_t usec) const { return quiet(usec < nowUsec_ ? nowSec_ + 1 : nowSec_, usec); }

  // The caller reports what it did.  `scheduled`: the frame is meant to go out at its precise
  // moment (as opposed to a forced redraw), so its timing is worth recording.
  void drew(time_t sec, uint32_t costUs, bool scheduled) {
    drawCostUs_ = (drawCostUs_ * 3 + clampCost(costUs)) / 4;
    prepared_ = true;
    preparedSec_ = sec;
    preparedScheduled_ = scheduled;
  }
  void sent(uint32_t costUs) {
    sendCostUs_ = (sendCostUs_ * 3 + clampCost(costUs)) / 4;
    lastSentSec_ = preparedSec_;
    prepared_ = false;
  }
  // Something else was drawn into the display buffer (the cube): the frame prepared there is gone.
  void discard() { prepared_ = false; }

  // The CPU clock changed from `fromMhz` to `toMhz` (below 80 MHz everything takes longer: 30 ms to draw
  // at 80 MHz is 120 ms at 20): what a draw and a send are expected to cost scales the other way round, so a
  // frame that is prepared already goes out with the right lead, and the next one is drawn with the right cost.
  // The ceiling on one measurement (a stall must not move the schedule) scales with it.
  void clockChanged(uint32_t fromMhz, uint32_t toMhz) {
    if (fromMhz == 0 || toMhz == 0 || fromMhz == toMhz) return;
    drawCostUs_ = scaleCost(drawCostUs_, fromMhz, toMhz);
    sendCostUs_ = scaleCost(sendCostUs_, fromMhz, toMhz);
    mhz_ = toMhz;
  }
  uint32_t mhz() const { return mhz_; }

  bool prepared() const { return prepared_; }
  time_t preparedSec() const { return preparedSec_; }
  bool preparedScheduled() const { return preparedScheduled_; }
  time_t lastSentSec() const { return lastSentSec_; }
  uint32_t drawCostUs() const { return drawCostUs_; }
  uint32_t sendCostUs() const { return sendCostUs_; }
  int32_t latencyUs() const { return latencyUs_; }
  // The panel was switched to another refresh rate (display_power.h).  A frame that is prepared already goes out
  // by the new figure.
  void setLatencyUs(int32_t us) { latencyUs_ = us; }

 private:
  // How far ahead of its second a frame is sent: the send, and the panel's time to the glass.
  int32_t sendLeadUs() const { return (int32_t)sendCostUs_ + latencyUs_; }
  // From now until the prepared frame is to be sent (negative once that is past).  Its second is the coming one, or
  // the one after it (a draw that ends before the tick, as with display_power = low), or the one that has begun (it is
  // late).
  int32_t untilSendUs(time_t sec, int32_t usec) const {
    return (int32_t)(preparedSec_ - sec - 1) * 1000000 + (1000000 - usec) - sendLeadUs();
  }

  // One freak measurement (a stall in the middle of a draw) must not move the schedule for long.  (The
  // ceiling is for the clocks of 80 MHz and up; below that a normal draw takes longer, and so does the ceiling.)
  uint32_t clampCost(uint32_t us) const {
    const uint32_t cap = mhz_ < 80 ? (uint32_t)((uint64_t)150000 * 80 / mhz_) : 150000;
    return us > cap ? cap : us;
  }
  uint32_t scaleCost(uint32_t us, uint32_t fromMhz, uint32_t toMhz) const {
    const uint64_t v = (uint64_t)us * fromMhz / toMhz;
    const uint32_t cap = toMhz < 80 ? (uint32_t)((uint64_t)150000 * 80 / toMhz) : 150000;
    return v > cap ? cap : (uint32_t)v;
  }

  int32_t latencyUs_;
  time_t nowSec_ = 0;  // the clock at the last next()
  int32_t nowUsec_ = 0;
  bool prepared_ = false;
  bool preparedScheduled_ = false;
  time_t preparedSec_ = 0;
  time_t lastSentSec_ = 0;
  uint32_t drawCostUs_ = 30000;
  uint32_t sendCostUs_ = 15000;
  uint32_t mhz_ = 80;  // the CPU clock the costs are for
};
