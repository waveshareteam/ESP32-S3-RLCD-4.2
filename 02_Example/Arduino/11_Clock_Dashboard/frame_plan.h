#pragma once

// When to draw a second and when to send it to the panel.
//
// Drawing a frame takes tens of milliseconds (four times as long at the 20 MHz idle
// clock) and the time varies (cache misses, the WiFi stack taking the memory bus, a
// flash write on the other core), while sending it is short.  So every second is drawn
// into the display buffer a second ahead, straight after the one before it has been
// sent, and sent just before it starts: the draw may run long by most of a second
// without the digits changing late, and the send and the draw are one stretch of work,
// so with light sleep (light_sleep.h) the chip wakes once a second.  The panel
// refreshes itself at 25.5 Hz, so a frame shows up 0-39 ms after it has been sent
// (0-125 ms at the 8 Hz of display_power = low, display_power.h); the send is timed to
// end `latencyUs` before the second starts, which puts the average error at zero.
//
// What the frame shows besides the time (the weather, the sensors, the battery) is
// taken when it is drawn, so up to a second before it is on the glass; whatever has to
// show at once forces a redraw.
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

  static constexpr int32_t kSpinUs = 5000;     // waits shorter than this are spun, longer ones slept
  static constexpr int32_t kQuietUs = 300000;  // slow work only with more than this left before the send

  explicit FramePlanner(int32_t latencyUs) : latencyUs_(latencyUs) {}

  // `sec` / `usec`: the clock now.  `force`: something has to be on the glass immediately
  // (a key press, a toast, a page change).
  Step next(time_t sec, int32_t usec, bool force) {
    const int32_t usToNext = 1000000 - usec;
    Step st;

    // A frame prepared for a second that is over, or for one more than two ahead, means the clock was stepped (the
    // hourly time sync) or the loop stalled for a long while: it is dropped, and the second that it is now is drawn at
    // once (below), so an old one is not sent first.  (A frame for the second that has just begun is sent, late.)
    if (prepared_ && (preparedSec_ < sec || preparedSec_ > sec + 2)) prepared_ = false;

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
      const int32_t waitUs = untilSendUs(sec, usec);  // 0 or less: due (its second may have begun)
      if (waitUs <= kSpinUs) {
        st.action = SEND;
        st.waitUs = waitUs > 0 ? waitUs : 0;
      } else {
        st.action = quiet(usec) ? IDLE : SLEEP;
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

  // How long the loop may leave the planner alone (light_sleep.h): the time until next() asks for more than IDLE or
  // SLEEP, if nothing is forced meanwhile.  That is the send of the prepared frame, once the wait is down to what is
  // spun; 0 when it asks for something now (with no frame prepared it always does: PREPARE or DRAW_NOW).
  int32_t idleUs(time_t sec, int32_t usec) const {
    if (!prepared_ || preparedSec_ < sec || preparedSec_ > sec + 2) return 0;  // (a frame next() drops counts as none)
    const int32_t waitUs = untilSendUs(sec, usec);
    return waitUs > kSpinUs ? waitUs - kSpinUs : 0;
  }

  // May slow work (I2C sensors, battery, a line of the power log) run now?  Never close to a send.  A prepared
  // frame is taken to go out in this second, as it does once its draw is over; at 80 MHz the draw can end just
  // before the tick, a second and more before its send, and then slow work waits for the tick.  With no frame
  // prepared, the tick is what counts.
  bool quiet(int32_t usec) const {
    const int32_t left = 1000000 - usec;
    return prepared_ ? left - sendLeadUs() > kQuietUs : left > kQuietUs;
  }

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
  // the one after it (a draw at 80 MHz can end before the tick), or the one that has begun (it is late).
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
  bool prepared_ = false;
  bool preparedScheduled_ = false;
  time_t preparedSec_ = 0;
  time_t lastSentSec_ = 0;
  uint32_t drawCostUs_ = 30000;
  uint32_t sendCostUs_ = 15000;
  uint32_t mhz_ = 80;  // the CPU clock the costs are for
};
