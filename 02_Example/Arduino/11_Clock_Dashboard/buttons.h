#pragma once

// Debounced multi-click / long-press detector for one active-low button.
// Pure logic with no Arduino dependency (tools/tests drives it with synthetic
// timestamps).  Feed update() the raw pressed/not-pressed level every few
// milliseconds; it returns a gesture once one is complete:
//
//   CLICK_1 / CLICK_2 / CLICK_3   that many clicks, reported once the button has
//                                 been idle for `multiClickGapMs` (3+ clicks = CLICK_3)
//   CLICK_LONG                    held for `longPressMs`, reported while still held

#include <stdint.h>

enum ClickEvent : uint8_t {
  CLICK_NONE = 0,
  CLICK_1 = 1,
  CLICK_2 = 2,
  CLICK_3 = 3,
  CLICK_LONG = 4
};

class ClickDetector {
 public:
  ClickDetector(uint32_t multiClickGapMs, uint32_t longPressMs, uint32_t debounceMs = 25)
      : gapMs_(multiClickGapMs), longMs_(longPressMs), debounceMs_(debounceMs) {}

  ClickEvent update(bool pressed, uint32_t nowMs) {
    // time does not run backwards (a level read just after an interrupt recorded a later change, see edge())
    if (seen_ && (int32_t)(nowMs - lastMs_) < 0) nowMs = lastMs_;
    seen_ = true;
    lastMs_ = nowMs;
    // debounce: accept a level once it has been steady for debounceMs_
    if (pressed != raw_) {
      raw_ = pressed;
      rawSinceMs_ = nowMs;
    }
    if (raw_ != stable_ && (uint32_t)(nowMs - rawSinceMs_) >= debounceMs_) {
      stable_ = raw_;
      if (stable_) {  // pressed
        pressStartMs_ = nowMs;
        longFired_ = false;
      } else if (longFired_) {  // released after a long press: nothing more to report
        clicks_ = 0;
      } else {  // released after a short press: count it
        if (clicks_ < 255) clicks_++;
        lastReleaseMs_ = nowMs;
      }
    }

    if (stable_ && !longFired_ && (uint32_t)(nowMs - pressStartMs_) >= longMs_) {
      longFired_ = true;
      clicks_ = 0;
      return CLICK_LONG;
    }
    if (!stable_ && clicks_ > 0 && (uint32_t)(nowMs - lastReleaseMs_) >= gapMs_) {
      uint8_t c = clicks_;
      clicks_ = 0;
      return c >= 3 ? CLICK_3 : (c == 2 ? CLICK_2 : CLICK_1);
    }
    return CLICK_NONE;
  }

  // A change of the level that happened at `atMs`, recorded by an interrupt while the loop was too busy to
  // look (a slow frame at the idle clock): time passes with the level as it was, and then it changes.  So a
  // tap that began and ended between two looks still counts, with its real length.
  ClickEvent edge(bool pressed, uint32_t atMs) {
    const ClickEvent before = update(raw_, atMs);
    const ClickEvent after = update(pressed, atMs);
    return before != CLICK_NONE ? before : after;
  }

  // The debounced state of the button, and for how long it has been down (0 when it is up), by the time
  // of the last update.
  bool isDown() const { return stable_; }
  uint32_t heldMs() const { return stable_ ? (uint32_t)(lastMs_ - pressStartMs_) : 0; }

 private:
  uint32_t gapMs_, longMs_, debounceMs_;
  bool seen_ = false;
  uint32_t lastMs_ = 0;
  bool raw_ = false;
  bool stable_ = false;
  bool longFired_ = false;
  uint8_t clicks_ = 0;
  uint32_t rawSinceMs_ = 0;
  uint32_t pressStartMs_ = 0;
  uint32_t lastReleaseMs_ = 0;
};

// A very long hold that acts when the button is let go: "armed" once it has been down for `holdMs` (the moment
// to say so on the screen), "fired" when it then comes up.  The clock restarts on this, and it must not restart
// with the button still down: KEY held while the clock starts means something else.  Feed it the debounced
// state of a ClickDetector (isDown(), heldMs()).
class HoldRelease {
 public:
  enum Event : uint8_t { NONE = 0, ARMED, FIRED };

  explicit HoldRelease(uint32_t holdMs) : holdMs_(holdMs) {}

  Event update(bool down, uint32_t heldMs) {
    if (down) {
      if (armed_ || heldMs < holdMs_) return NONE;
      armed_ = true;
      return ARMED;
    }
    if (!armed_) return NONE;
    armed_ = false;
    return FIRED;
  }

  bool armed() const { return armed_; }
  // The hold has come to mean something else (a second button joined it): letting go will not fire.
  void cancel() { armed_ = false; }

 private:
  uint32_t holdMs_;
  bool armed_ = false;
};

// Two buttons held down together for `holdMs`: "fired" once, while both are still down.  From the moment
// both are down until both are up again, and for `quietMs` after that, the buttons' own gestures must not
// count (muted()): each one's long press comes due long before the two together do, a hold that acts on
// letting go would act, and two buttons let go are not two clicks.  The quiet time is for the click that a
// detector reports a moment after its button came up.  Feed it the debounced states of the two
// ClickDetectors (isDown()).
class ChordHold {
 public:
  enum Event : uint8_t { NONE = 0, FIRED };

  ChordHold(uint32_t holdMs, uint32_t quietMs) : holdMs_(holdMs), quietMs_(quietMs) {}

  Event update(bool aDown, bool bDown, uint32_t nowMs) {
    const bool both = aDown && bDown;
    if (both && !both_) {  // the two together, from now: one that was let go and pressed again starts the count afresh
      sinceMs_ = nowMs;
      fired_ = false;
    }
    both_ = both;
    if (both) engaged_ = true;
    if (engaged_ && !aDown && !bDown) {  // both are up again
      engaged_ = false;
      quiet_ = true;
      quietSinceMs_ = nowMs;
    }
    if (quiet_ && (uint32_t)(nowMs - quietSinceMs_) >= quietMs_) quiet_ = false;
    if (both && !fired_ && (uint32_t)(nowMs - sinceMs_) >= holdMs_) {
      fired_ = true;
      return FIRED;
    }
    return NONE;
  }

  bool muted() const { return engaged_ || quiet_; }

 private:
  uint32_t holdMs_, quietMs_;
  bool both_ = false;     // the two are down together right now
  bool fired_ = false;    // ... and that has been reported
  bool engaged_ = false;  // they were, and not both have come up since
  bool quiet_ = false;    // both came up a moment ago
  uint32_t sinceMs_ = 0, quietSinceMs_ = 0;
};
