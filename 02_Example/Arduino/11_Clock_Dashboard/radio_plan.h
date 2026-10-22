#pragma once

// When the radio is on, in "wifi_mode = sync" (and the CPU clock policy that goes with it).
//
// In sync mode the radio is off between "sessions".  A session starts when something needs the
// network (the time, the place, the weather, a look at Spotify, a key press), lasts while there
// is something to do, and ends a moment after the last thing was done.  The network task works out
// what is due and tells the planner; the planner says whether the radio should be on, and keeps
// the books on what went wrong:
//
//   * A network that is there but cannot be joined (a wrong password, a router that refuses), or
//     work that never gets done, is a failure: the next session waits 1, 2, 5, 10 and then 15
//     minutes, so a clock that is locked out does not hammer the router all day.
//   * A network that is not there is not a failure.  The WiFi stack says so within a few seconds
//     ("no such network"), the session ends there and then, and the clock looks again every 2
//     minutes for the first ten and every 5 from then on, for as long as it takes: a clock taken
//     to work finds the phone's hotspot soon after it is switched on.
//
// A key press starts a session whatever the wait (the person wants an answer) and holds the radio
// for 45 seconds; one press buys one attempt.  Any button press makes a clock that found no
// network look again at once.
//
// Pure logic with no Arduino dependency: the times are passed in, tools/tests drives it with a
// simulated day.

#include <stdint.h>

namespace radioplan {

const uint32_t kConnectTimeoutMs = 45000;  // a session that is not online by now is given up (main 20 s, backup 20 s, and a margin)
const uint32_t kSettleMs = 2500;           // the radio stays on this long after the last thing was done (sockets close, a command is confirmed)
const uint32_t kDemandMs = 45000;          // a key press holds the radio for this long (follow-up presses, the confirming poll)
const uint32_t kSessionMaxMs = 180000;     // work that is still "due" after this long, with nothing holding the radio, ends the session as a failure
const int kBackoffSteps = 5;
const uint32_t kBackoffMs[kBackoffSteps] = {60000UL, 120000UL, 300000UL, 600000UL, 900000UL};  // after the 1st, 2nd, ... failed session in a row

// None of the clock's networks is in range: it looks again after kAwayQuickMs for the first kAwayQuickLooks
// looks (a router that is restarting is back within a few minutes), and every kAwaySlowMs from then on.  A look
// is a scan for each network, two or three seconds each, so even the slow rate costs about a hundredth of
// what a radio left on would.
const uint32_t kAwayQuickMs = 120000UL;
const int kAwayQuickLooks = 5;
const uint32_t kAwaySlowMs = 300000UL;
inline uint32_t awayAfterMs(int looks) { return looks <= kAwayQuickLooks ? kAwayQuickMs : kAwaySlowMs; }

// How many "no such network" answers it takes, per network, before the clock believes it: two when the last
// session worked (one scan can miss a router that is there), one while it is away already (a miss then only
// costs the wait until the next look).
const int kAwayScansFirst = 2;
const int kAwayScansAgain = 1;
// An answer that comes sooner than this after an attempt began belongs to the attempt before it: a scan of
// every channel takes well over a second.
const uint32_t kMinScanMs = 700;

// A time still to come, kept as millis(): 0 means "none".  A time that has passed must be put back to 0 (or
// kept fresh with keepDue()), because 32 bits of milliseconds wrap: one that lies more than 24.8 days back
// looks like one that is still to come.
inline bool pending(uint32_t nowMs, uint32_t untilMs) { return untilMs != 0 && (int32_t)(untilMs - nowMs) > 0; }
// The times a person's key press holds the radio (`netWake()` records `demandUntil(now)`).
inline uint32_t demandUntil(uint32_t nowMs) { return (nowMs + kDemandMs) | 1u; }
inline bool demandActive(uint32_t nowMs, uint32_t untilMs) { return pending(nowMs, untilMs); }
// How long to wait before trying a piece of work again (the weather) after it failed `streak` times in a
// row while the radio was joined: 1, 2, 5, 10 minutes and then every 15.
inline uint32_t retryAfterMs(int streak) {
  const int step = streak < 1 ? 0 : (streak > kBackoffSteps ? kBackoffSteps - 1 : streak - 1);
  return kBackoffMs[step];
}

// A session that is going on anyway also asks for the network time when it is within kNtpEarlyMs of being
// due, and the time only starts a session of its own when it is kNtpGraceMs overdue: the sessions come every
// quarter of an hour for the weather, and a separate session just for the time would cost more than a
// few minutes early or late.
const uint32_t kNtpEarlyMs = 10UL * 60UL * 1000UL;
const uint32_t kNtpGraceMs = 5UL * 60UL * 1000UL;

// Is a time (millis) due?  Wrap-safe for times up to 24.8 days back; see keepDue() for longer.
inline bool due(uint32_t nowMs, uint32_t atMs) { return (int32_t)(nowMs - atMs) >= 0; }
// Work that has been due for a long time (the network was away for weeks) must stay due: call this every time
// round the loop and a time more than 12 days back is moved up to 12 days back, well short of the wrap.
const uint32_t kStaleMs = 0x40000000UL;
inline void keepDue(uint32_t nowMs, uint32_t *atMs) {
  if ((int32_t)(nowMs - *atMs) > (int32_t)kStaleMs) *atMs = nowMs - kStaleMs;
}

// What Spotify needs of the radio.  A command that waits and the Now Playing page hold it in any case.  Music
// (playing, or paused a few minutes ago) holds it only with spotify_live = on: then the clock asks the player
// every ten seconds, as it does with the radio always on.  With spotify_live = off the radio goes off between
// looks and the next look is due when the track on screen should be over, see trackLookAfterMs().
struct SpotifyNeeds {
  bool commandWaiting = false;
  bool pageOpen = false;
  bool playing = false;
  bool pausedRecently = false;
  bool live = true;
};
inline bool spotifyHolds(const SpotifyNeeds &n) {
  return n.commandWaiting || n.pageOpen || (n.live && (n.playing || n.pausedRecently));
}
// spotify_live = off: how long after a look at the player the next one is due, for a track of `durationMs` that
// is `progressMs` in: when it should be over, and a moment more because Spotify needs one to move on.  A track
// that is over already by Spotify's own figures gets just that moment.
const uint32_t kTrackEndSlackMs = 1500;
inline uint32_t trackLookAfterMs(uint32_t durationMs, uint32_t progressMs) {
  return (durationMs > progressMs ? durationMs - progressMs : 0) + kTrackEndSlackMs;
}

// What the network task knows, every time round its loop.
struct Inputs {
  bool online = false;       // associated, with an address
  bool workDue = false;      // the time, the place, the weather or a look at Spotify is due
  bool holdSpotify = false;  // music is playing, a command waits, it was playing a minute ago, the Now Playing page is open
  bool demand = false;       // a key press (or a refresh) asked for the radio and its time has not run out
  uint32_t demandSeq = 1;    // ... and which press that was: it counts up with every press
  bool absent = false;       // the WiFi stack has looked for each of the clock's networks and found none (Search::noneInRange)
};

// One session's search for the clock's networks.  The network task tries one network at a time and passes on
// what the WiFi stack has answered since the attempt began: how often "no such network" (a scan of every
// channel found nothing by that name), how often anything else (a wrong password, a router that refuses, a
// connection that broke).
class Search {
 public:
  // The radio came on.  `scansNeeded`: kAwayScansFirst or kAwayScansAgain.
  void begin(bool hasMain, bool hasBackup, int scansNeeded) {
    has_[0] = hasMain;
    has_[1] = hasBackup;
    need_ = scansNeeded < 1 ? 1 : scansNeeded;
    forget();
  }
  // A connection came up, or broke: what was learnt before says nothing about now.
  void forget() {
    absent_[0] = absent_[1] = false;
    sawOther_ = false;
  }
  // The answers to the attempt on `net` (0 = the main network, 1 = the backup) so far.  True, once, when it
  // turns out that this network is not there: the next one can be tried at once instead of waiting the
  // attempt out.
  bool note(int net, unsigned notFound, unsigned other) {
    if (other > 0) sawOther_ = true;
    if (net < 0 || net > 1 || absent_[net] || notFound < (unsigned)need_) return false;
    absent_[net] = true;
    return true;
  }
  // None of the clock's networks is there, and nothing else went wrong on the way: the session can end.
  bool noneInRange() const {
    return (has_[0] || has_[1]) && !sawOther_ && (!has_[0] || absent_[0]) && (!has_[1] || absent_[1]);
  }

 private:
  bool has_[2] = {false, false};
  bool absent_[2] = {false, false};
  bool sawOther_ = false;
  int need_ = kAwayScansFirst;
};

class Planner {
 public:
  enum State : uint8_t { OFF, CONNECTING, ONLINE };
  // How the last session ended: everything done, none of the networks in range, or given up (the network
  // would not let the clock in, or work never finished).
  enum Outcome : uint8_t { NONE, DONE, AWAY, FAILED };

  // `sync` false: the radio is always on and the planner only mirrors it.  `connectTimeoutMs`: how long a
  // session may take to join a network before it is given up (long enough to try both networks if there
  // are two, see kConnectTimeoutMs).
  explicit Planner(bool sync = true, uint32_t connectTimeoutMs = kConnectTimeoutMs) : sync_(sync), connectTimeoutMs_(connectTimeoutMs) {
    if (!sync) state_ = CONNECTING;
  }

  // Should the radio be on?  Call it every time round the loop.
  bool update(uint32_t nowMs, const Inputs &in) {
    accountOnTime(nowMs);
    if (!sync_) {
      state_ = in.online ? ONLINE : CONNECTING;
      return true;
    }
    // a key press buys one attempt: the one a session was given up on does not start the next
    const bool demand = in.demand && in.demandSeq != spentSeq_;
    const bool wants = in.workDue || in.holdSpotify || demand;
    switch (state_) {
      case OFF:
        if (retryAtMs_ != 0 && due(nowMs, retryAtMs_)) retryAtMs_ = 0;  // the wait is over (and a time long past must not come round again)
        if (wants && (retryAtMs_ == 0 || demand)) {                      // (a key press does not wait)
          state_ = CONNECTING;
          sessionStartMs_ = nowMs;
          askedFor_ = demand || nudged_;
          nudged_ = false;
        }
        break;
      case CONNECTING:
        if (in.online) {
          state_ = ONLINE;
          onlineSinceMs_ = nowMs;
          lastActiveMs_ = nowMs;
          failures_ = 0;
          awayLooks_ = 0;
        } else if (in.absent) {
          end(nowMs, in, AWAY);
        } else if ((uint32_t)(nowMs - sessionStartMs_) >= connectTimeoutMs_) {
          end(nowMs, in, FAILED);
        }
        break;
      case ONLINE:
        if (!in.online) {  // dropped: join again (the time allowed starts over)
          state_ = CONNECTING;
          sessionStartMs_ = nowMs;
          break;
        }
        if (wants) lastActiveMs_ = nowMs;
        if (!in.holdSpotify && !demand && (uint32_t)(nowMs - sessionStartMs_) >= kSessionMaxMs && in.workDue) {
          end(nowMs, in, FAILED);  // work that never gets done must not keep the radio on for ever
        } else if ((uint32_t)(nowMs - lastActiveMs_) >= kSettleMs) {
          state_ = OFF;  // everything is done
          retryAtMs_ = 0;
          outcome_ = DONE;
          sessions_++;
        }
        break;
    }
    return state_ != OFF;
  }

  // Someone is using the clock (any button): if it found none of its networks last time, it looks again now
  // instead of waiting for the next look.  Does nothing otherwise.
  void lookNow() {
    if (!sync_ || state_ != OFF || outcome_ != AWAY) return;
    retryAtMs_ = 0;
    nudged_ = true;
  }

  State state() const { return state_; }
  bool radioWanted() const { return state_ != OFF; }

  // True once after a session ended without the work done (AWAY or FAILED, see lastOutcome()).
  bool takeGaveUp() {
    const bool g = gaveUp_;
    gaveUp_ = false;
    return g;
  }
  Outcome lastOutcome() const { return outcome_; }
  // When the next session may start although something is due (0 = now); only means something while OFF.
  uint32_t retryAtMs() const { return retryAtMs_; }
  int failures() const { return failures_; }    // failed sessions in a row
  int awayLooks() const { return awayLooks_; }  // looks in a row that found none of the networks
  uint32_t sessions() const { return sessions_; }

  // Milliseconds the radio was on (CONNECTING and ONLINE) up to `nowMs`.
  uint32_t radioOnMs(uint32_t nowMs) {
    accountOnTime(nowMs);
    return onMs_;
  }

 private:
  void end(uint32_t nowMs, const Inputs &in, Outcome why) {
    uint32_t wait;
    if (why == AWAY) {
      // a look someone asked for (a button) is extra: it does not use up one of the quick looks
      if (awayLooks_ == 0 || (!askedFor_ && awayLooks_ < 1000)) awayLooks_++;
      wait = awayAfterMs(awayLooks_);
    } else {
      if (failures_ < 1000) failures_++;
      wait = retryAfterMs(failures_);
    }
    retryAtMs_ = nowMs + wait;
    if (retryAtMs_ == 0) retryAtMs_ = 1;
    if (in.demand) spentSeq_ = in.demandSeq;  // that key press has had its attempt
    state_ = OFF;
    outcome_ = why;
    gaveUp_ = true;
  }
  void accountOnTime(uint32_t nowMs) {
    if (started_ && state_ != OFF) onMs_ += (uint32_t)(nowMs - lastTickMs_);
    lastTickMs_ = nowMs;
    started_ = true;
  }

  bool sync_;
  uint32_t connectTimeoutMs_;
  State state_ = OFF;
  Outcome outcome_ = NONE;
  uint32_t sessionStartMs_ = 0, onlineSinceMs_ = 0, lastActiveMs_ = 0;
  uint32_t retryAtMs_ = 0;
  uint32_t spentSeq_ = 0;
  bool nudged_ = false, askedFor_ = false;
  bool gaveUp_ = false;
  int failures_ = 0;
  int awayLooks_ = 0;
  uint32_t sessions_ = 0;
  uint32_t onMs_ = 0, lastTickMs_ = 0;
  bool started_ = false;
};

}  // namespace radioplan
