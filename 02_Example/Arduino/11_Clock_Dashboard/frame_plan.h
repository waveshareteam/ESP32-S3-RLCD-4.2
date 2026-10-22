#pragma once

// When to draw the coming second and when to send it to the panel.
//
// Drawing a frame takes tens of milliseconds and the time varies (cache misses, the
// WiFi stack taking the memory bus, a flash write on the other core), while sending
// it is short.  So the coming second is drawn into the display buffer early, with
// room to spare, and sent just before it starts: the draw may run long without the
// digits changing late.  The panel refreshes itself at 25.5 Hz, so a frame shows
// up 0-39 ms after it has been sent (0-125 ms at the 8 Hz of display_power = low,
// display_power.h); the send is timed to end `latencyUs` before the second starts,
// which puts the average error at zero.
//
// Pure logic with no Arduino dependency: the UI loop asks next() whenever it wakes up
// and does what it says; tools/tests drives it with simulated clocks and stalls.

#include <stdint.h>
#include <time.h>

class FramePlanner {
 public:
  enum Action : uint8_t {
    IDLE,      // nothing to do yet; slow work may run if quiet() allows
    DRAW_NOW,  // draw `sec`, then send it at once (something has to be on screen now)
    PREPARE,   // draw `sec` into the buffer, but do not send it yet
    SEND,      // wait `waitUs` (at most kSpinUs), then send the prepared frame
    SLEEP,     // the prepared frame is not due for `waitUs`: sleep a little and ask again
  };
  struct Step {
    Action action = IDLE;
    time_t sec = 0;
    int32_t waitUs = 0;
  };

  static constexpr int32_t kSpinUs = 5000;     // waits shorter than this are spun, longer ones slept
  static constexpr int32_t kMarginUs = 60000;  // spare time between a frame being drawn and being due
  static constexpr int32_t kQuietUs = 300000;  // slow work only with more than this left before the tick

  explicit FramePlanner(int32_t latencyUs) : latencyUs_(latencyUs) {}

  // `sec` / `usec`: the clock now.  `force`: something has to be on the glass immediately
  // (a key press, a toast, a page change).
  Step next(time_t sec, int32_t usec, bool force) {
    const int32_t usToNext = 1000000 - usec;
    const int32_t sendLead = (int32_t)sendCostUs_ + latencyUs_;
    const int32_t drawLead = sendLead + (int32_t)drawCostUs_ + kMarginUs;
    Step st;

    if (prepared_ && preparedSec_ > sec + 1) prepared_ = false;  // the clock went backwards: start over

    if (force) {
      // A frame drawn and sent now is on the glass `cost` from now.  Close to a tick there would
      // be no time left afterwards to prepare the next second the usual way, so show the second
      // that is about to start instead (up to about 65 ms early beats a late second).
      const int32_t cost = (int32_t)(drawCostUs_ + sendCostUs_);
      st.action = DRAW_NOW;
      st.sec = usToNext <= 2 * cost + latencyUs_ ? sec + 1 : sec;
      return st;
    }
    if (prepared_) {
      const int32_t waitUs = preparedSec_ > sec ? usToNext - sendLead : 0;  // already due if its second has begun
      st.waitUs = waitUs > 0 ? waitUs : 0;
      st.action = waitUs > kSpinUs ? SLEEP : SEND;
      return st;
    }
    if (lastSentSec_ != sec && lastSentSec_ != sec + 1) {  // behind (start-up, a long stall): catch up at once
      st.action = DRAW_NOW;
      st.sec = sec;
      return st;
    }
    if (usToNext <= drawLead && lastSentSec_ != sec + 1) {
      st.action = PREPARE;
      st.sec = sec + 1;
      return st;
    }
    return st;  // IDLE
  }

  // How long the loop may leave the planner alone (light_sleep.h): the time until next() asks for more than a
  // nap, PREPARE or SEND, if nothing is forced meanwhile.  0 when it asks for something now.
  int32_t idleUs(time_t sec, int32_t usec) const {
    const int32_t usToNext = 1000000 - usec;
    const int32_t sendLead = (int32_t)sendCostUs_ + latencyUs_;
    const int32_t drawLead = sendLead + (int32_t)drawCostUs_ + kMarginUs;
    if (prepared_) {
      if (preparedSec_ <= sec || preparedSec_ > sec + 1) return 0;  // due, or the clock went backwards (next() starts over)
      const int32_t waitUs = usToNext - sendLead;
      return waitUs > kSpinUs ? waitUs - kSpinUs : 0;  // SEND, once the wait is down to what is spun
    }
    if (lastSentSec_ == sec + 1) {  // the coming second is out already: the one after it is prepared in the next second
      const int32_t into = 1000000 - drawLead;
      return usToNext + (into > 0 ? into : 0);
    }
    if (lastSentSec_ != sec) return 0;  // behind: DRAW_NOW
    return usToNext > drawLead ? usToNext - drawLead : 0;
  }

  // May slow work (I2C sensors, battery) run now?  Never close to a tick.
  bool quiet(int32_t usec) const { return !prepared_ && 1000000 - usec > kQuietUs; }

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

  // The CPU clock changed from `fromMhz` to `toMhz` (below 80 MHz everything takes longer: 30 ms to draw
  // at 80 MHz is 120 ms at 20): what a draw and a send are expected to cost scales the other way round, so
  // the first frame at the new clock is planned with the right lead.  The ceiling on one measurement
  // (a stall must not move the schedule) scales with it.
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
  bool prepared_ = false;
  bool preparedScheduled_ = false;
  time_t preparedSec_ = 0;
  time_t lastSentSec_ = 0;
  uint32_t drawCostUs_ = 30000;
  uint32_t sendCostUs_ = 15000;
  uint32_t mhz_ = 80;  // the CPU clock the costs are for
};
