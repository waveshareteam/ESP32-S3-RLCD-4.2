// Host-side tests of the power logic that is not the settings: when the radio is on in "wifi_mode = sync"
// (radio_plan.h), which CPU clock the clock runs at (clock_policy.h), when the console is shut down
// (console_policy.h), the picker across radio sessions (wifi_pick.h), the frame planner after a clock change
// (frame_plan.h), the light sleep between the frames (light_sleep.h), and the radio's wait for the CPU clock with
// the figures that say why sessions went wrong (clock_policy.h, radio_plan.h).  Built and run by run_tests.sh.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

#include "check.h"
#include "clock_policy.h"
#include "console_policy.h"
#include "display_power.h"
#include "frame_plan.h"
#include "light_sleep.h"
#include "radio_plan.h"
#include "settings.h"
#include "wifi_pick.h"

using namespace radioplan;

namespace {

// A planner driven by hand: `in` is what the network task would report.
struct Drive {
  Planner p;
  Inputs in;
  uint32_t t = 100000;
  explicit Drive(bool sync = true, uint32_t timeout = kConnectTimeoutMs) : p(sync, timeout) {}
  bool step(uint32_t ms) {
    t += ms;
    return p.update(t, in);
  }
  // steps of `ms` until the radio is on (true) or `limitMs` has passed
  bool untilOn(uint32_t limitMs, uint32_t ms = 1000) {
    const uint32_t t0 = t;
    bool on = false;
    while (!on && t - t0 < limitMs) on = step(ms);
    return on;
  }
  bool untilOff(uint32_t limitMs, uint32_t ms = 1000) {
    const uint32_t t0 = t;
    bool on = true;
    while (on && t - t0 < limitMs) on = step(ms);
    return !on;
  }
};

void testSteps() {
  section("radio plan: one session at a time");
  {  // nothing to do: the radio stays off
    Drive d;
    for (int i = 0; i < 100; i++) CHECK(!d.step(1000));
    CHECK(d.p.state() == Planner::OFF && d.p.sessions() == 0 && d.p.radioOnMs(d.t) == 0);
    CHECK(d.p.lastOutcome() == Planner::NONE);
  }
  {  // always mode: the radio is on whatever it is asked
    Drive d(false);
    CHECK(d.step(10) && d.p.state() == Planner::CONNECTING);
    d.in.online = true;
    CHECK(d.step(10) && d.p.state() == Planner::ONLINE);
    d.in.online = false;
    CHECK(d.step(10) && d.p.state() == Planner::CONNECTING);
    d.in.absent = true;  // ... and "no such network" does not switch it off: that is sync mode's business
    for (int i = 0; i < 100; i++) CHECK(d.step(1000));
    CHECK(!d.p.takeGaveUp() && d.p.lastOutcome() == Planner::NONE);
    d.p.lookNow();
    CHECK(d.step(10));
  }
  {  // a session: starts at once, goes online, ends 2.5 s after the work is done
    Drive d;
    d.in.workDue = true;
    CHECK(d.step(100) && d.p.state() == Planner::CONNECTING);
    for (int i = 0; i < 30; i++) CHECK(d.step(100));  // 3 s of association
    d.in.online = true;
    CHECK(d.step(100) && d.p.state() == Planner::ONLINE);
    for (int i = 0; i < 50; i++) CHECK(d.step(100));  // 5 s of work
    d.in.workDue = false;
    const uint32_t tDone = d.t;
    bool on = true;
    while (on && d.t - tDone < 10000) on = d.step(100);
    CHECK(!on && d.p.state() == Planner::OFF);
    CHECK(d.t - tDone >= kSettleMs && d.t - tDone <= kSettleMs + 200);  // the settle time, no more, no less
    CHECK(d.p.sessions() == 1 && d.p.failures() == 0 && d.p.retryAtMs() == 0);
    CHECK(d.p.lastOutcome() == Planner::DONE && !d.p.takeGaveUp());
    CHECK(d.p.radioOnMs(d.t) > 8000 && d.p.radioOnMs(d.t) < 12000);
  }
  {  // a session holds while Spotify holds it, then settles
    Drive d;
    d.in.holdSpotify = true;
    d.step(100);
    d.in.online = true;
    for (int i = 0; i < 6000; i++) CHECK(d.step(100));  // ten minutes of music
    d.in.holdSpotify = false;
    const uint32_t t0 = d.t;
    bool on = true;
    while (on && d.t - t0 < 10000) on = d.step(100);
    CHECK(!on && d.t - t0 >= kSettleMs && d.t - t0 <= kSettleMs + 200);
  }
  {  // a dropped connection is joined again, with a fresh allowance of time
    Drive d;
    d.in.workDue = true;
    d.step(100);
    d.in.online = true;
    d.step(100);
    d.in.online = false;
    CHECK(d.step(100) && d.p.state() == Planner::CONNECTING);
    for (uint32_t waited = 0; waited < kConnectTimeoutMs - 1000; waited += 1000) CHECK(d.step(1000));
    CHECK(d.p.state() == Planner::CONNECTING && !d.p.takeGaveUp());
    bool on = true;
    for (int i = 0; i < 3 && on; i++) on = d.step(1000);
    CHECK(!on && d.p.takeGaveUp() && d.p.failures() == 1 && d.p.lastOutcome() == Planner::FAILED);
  }
}

void testFailures() {
  section("radio plan: a network that cannot be joined");
  Drive d;
  d.in.workDue = true;  // the weather is due and stays due: nothing ever gets done
  const uint32_t expectBackoff[] = {60000, 120000, 300000, 600000, 900000, 900000, 900000};
  for (int round = 0; round < 7; round++) {
    bool on = false;
    const uint32_t t0 = d.t;
    while (!on && d.t - t0 < 2000000) on = d.step(1000);  // (the first session starts at once, the others after the back-off)
    CHECK(on);
    const uint32_t tOn = d.t;
    while (on && d.t - tOn < 120000) on = d.step(1000);
    CHECK(!on && d.p.takeGaveUp() && d.p.lastOutcome() == Planner::FAILED);
    CHECK(d.t - tOn >= kConnectTimeoutMs - 1000 && d.t - tOn <= kConnectTimeoutMs + 1000);
    CHECK(d.p.failures() == round + 1 && d.p.awayLooks() == 0);
    CHECK(d.p.retryAtMs() - d.t >= expectBackoff[round] - 1000 && d.p.retryAtMs() - d.t <= expectBackoff[round] + 1000);
    const uint32_t retryAt = d.p.retryAtMs();
    while ((int32_t)(retryAt - d.t) > 1500) CHECK(!d.step(1000));  // it stays off until then
  }
  {  // a key press starts a session in the middle of the back-off (the person wants an answer), and the count goes on
    Drive e;
    e.in.workDue = true;
    e.step(1000);
    bool on = true;
    while (on) on = e.step(1000);  // the first session failed
    CHECK(e.p.failures() == 1);
    e.step(5000);
    CHECK(!e.step(100));  // 5 s into a minute's back-off
    e.in.demand = true;
    CHECK(e.step(100) && e.p.state() == Planner::CONNECTING);
    e.in.demand = false;
    on = true;
    while (on) on = e.step(1000);  // it fails too: two in a row, so the back-off is two minutes
    CHECK(e.p.failures() == 2);
    CHECK(e.p.retryAtMs() - e.t >= 119000 && e.p.retryAtMs() - e.t <= 121000);
    CHECK(!e.step(30000));  // and it is off again, with the work still due
  }
  {  // one press buys one attempt: when that is given up, the rest of the 45 seconds does not start another
     // (a clock with one network gives up after 25 s; the press would otherwise run the radio twice)
    Drive e(true, 25000);
    e.in.workDue = true;
    e.step(1000);
    CHECK(e.untilOff(60000) && e.p.failures() == 1);
    e.step(5000);
    e.in.demand = true;
    e.in.demandSeq = 7;
    CHECK(e.step(100) && e.p.state() == Planner::CONNECTING);
    CHECK(e.untilOff(60000, 100) && e.p.failures() == 2);  // given up after 25 s, the press still "active" for 20 more
    for (int i = 0; i < 190; i++) CHECK(!e.step(100));     // ... and nothing starts in those
    e.in.demandSeq = 8;                                     // a new press does
    CHECK(e.step(100) && e.p.state() == Planner::CONNECTING);
  }
  {  // joining resets the count
    Drive e;
    e.in.workDue = true;
    e.step(1000);
    bool on = true;
    while (on) on = e.step(1000);
    CHECK(e.p.failures() == 1);
    while (!e.step(1000)) {
    }
    e.in.online = true;
    e.step(100);
    CHECK(e.p.failures() == 0);
  }
  {  // work that never gets done, with a joined radio, is a failure after three minutes, not a radio that stays on for ever
    Drive e;
    e.in.workDue = true;
    e.step(100);
    e.in.online = true;
    bool on = true;
    const uint32_t t0 = e.t;
    while (on && e.t - t0 < 400000) on = e.step(1000);
    CHECK(!on && e.p.takeGaveUp() && e.p.lastOutcome() == Planner::FAILED);
    CHECK(e.t - t0 >= kSessionMaxMs - 1000 && e.t - t0 <= kSessionMaxMs + 1000);
  }
  {  // ... but not while music holds it
    Drive e;
    e.in.workDue = true;
    e.in.holdSpotify = true;
    e.step(100);
    e.in.online = true;
    for (uint32_t i = 0; i < 400; i++) CHECK(e.step(1000));
  }
}

// One session's search for the networks: what "no such network" from the WiFi stack adds up to.
void testSearch() {
  section("radio plan: is the network there at all?");
  {  // one network, coming from a session that worked: two answers before it is believed
    Search s;
    s.begin(true, false, kAwayScansFirst);
    CHECK(!s.noneInRange());
    CHECK(!s.note(0, 0, 0) && !s.noneInRange());
    CHECK(!s.note(0, 1, 0) && !s.noneInRange());  // one scan can miss a router that is there
    CHECK(s.note(0, 2, 0) && s.noneInRange());
    CHECK(!s.note(0, 3, 0) && s.noneInRange());   // said once: the caller moves on once, not every time round
  }
  {  // away already: the first answer will do
    Search s;
    s.begin(true, false, kAwayScansAgain);
    CHECK(s.note(0, 1, 0) && s.noneInRange());
  }
  {  // two networks: both have to be missing
    Search s;
    s.begin(true, true, kAwayScansAgain);
    CHECK(s.note(0, 1, 0) && !s.noneInRange());   // the main one is not there: on to the backup at once
    CHECK(!s.note(1, 0, 0) && !s.noneInRange());
    CHECK(s.note(1, 1, 0) && s.noneInRange());
  }
  {  // a network that answers anything else (a wrong password) is there: the session is not "away"
    Search s;
    s.begin(true, true, kAwayScansAgain);
    CHECK(!s.note(0, 0, 1) && !s.noneInRange());
    CHECK(s.note(1, 1, 0));                       // the backup is missing ...
    CHECK(!s.noneInRange());                      // ... but the main one refused: that is a failure, with its back-off
    CHECK(!s.note(0, 5, 0) || !s.noneInRange());  // and it stays one for this session
    CHECK(!s.noneInRange());
  }
  {  // a mixed bag on one network (not found, then a refusal): not away
    Search s;
    s.begin(true, false, kAwayScansFirst);
    CHECK(!s.note(0, 1, 1) && !s.noneInRange());
    s.note(0, 2, 1);
    CHECK(!s.noneInRange());
  }
  {  // a connection came up or broke: start over
    Search s;
    s.begin(true, false, kAwayScansAgain);
    s.note(0, 0, 3);
    s.forget();
    CHECK(s.note(0, 1, 0) && s.noneInRange());
    s.forget();
    CHECK(!s.noneInRange());
  }
  {  // no network set at all is not "away" (the network task never gets this far), and a bad index is ignored
    Search s;
    s.begin(false, false, kAwayScansAgain);
    CHECK(!s.noneInRange());
    CHECK(!s.note(-1, 9, 0) && !s.note(2, 9, 0) && !s.noneInRange());
    s.begin(false, true, 0);  // (a count below one is one)
    CHECK(s.note(1, 1, 0) && s.noneInRange());
  }
  CHECK(kAwayScansFirst == 2 && kAwayScansAgain == 1 && kMinScanMs >= 500 && kMinScanMs < 1300);  // (a scan of 11 channels takes 1.3 s at the least)
}

void testAway() {
  section("radio plan: none of the networks in range");
  {  // not a failure: the session ends as soon as the answer is in, and the looks come every 2 minutes for the
     // first five, then every 5 minutes, for as long as it takes
    Drive d;
    d.in.workDue = true;
    const uint32_t expectWait[] = {120000, 120000, 120000, 120000, 120000, 300000, 300000, 300000, 300000};
    for (int look = 0; look < 9; look++) {
      CHECK(d.untilOn(400000, 100));
      const uint32_t tOn = d.t;
      for (int i = 0; i < 24; i++) CHECK(d.step(100));  // 2.4 s: the scan
      d.in.absent = true;
      CHECK(!d.step(100) && d.p.takeGaveUp() && d.p.lastOutcome() == Planner::AWAY);
      d.in.absent = false;
      CHECK(d.t - tOn <= 2600);
      CHECK(d.p.failures() == 0 && d.p.awayLooks() == look + 1);  // the failure count is not touched
      const uint32_t wait = d.p.retryAtMs() - d.t;
      if (wait != expectWait[look]) printf("  look %d: wait %u s\n", look + 1, (unsigned)(wait / 1000));
      CHECK(wait == expectWait[look]);
      const uint32_t retryAt = d.p.retryAtMs();
      while ((int32_t)(retryAt - d.t) > 150) CHECK(!d.step(100));  // off until then
    }
    // the network is back: joined at the next look, and the counts start over
    CHECK(d.untilOn(400000, 100));
    d.in.online = true;
    d.step(100);
    CHECK(d.p.state() == Planner::ONLINE && d.p.awayLooks() == 0);
    d.in.workDue = false;
    CHECK(d.untilOff(10000, 100) && d.p.lastOutcome() == Planner::DONE && d.p.retryAtMs() == 0);
  }
  CHECK(awayAfterMs(1) == kAwayQuickMs && awayAfterMs(kAwayQuickLooks) == kAwayQuickMs && awayAfterMs(kAwayQuickLooks + 1) == kAwaySlowMs &&
        awayAfterMs(1000) == kAwaySlowMs);
  CHECK(kAwaySlowMs < kBackoffMs[kBackoffSteps - 1]);  // the point of it: a network that is not there is looked for more often than one that refuses

  // (helper: one look that finds nothing; the radio is on for 2.5 s)
  auto lookAndMiss = [](Drive &d) {
    for (int i = 0; i < 24; i++) d.step(100);
    d.in.absent = true;
    const bool off = !d.step(100);
    d.in.absent = false;
    return off && d.p.takeGaveUp() && d.p.lastOutcome() == Planner::AWAY;
  };
  {  // a key press while away: one look at once, and only one for that press; the next press looks again
    Drive d;
    d.in.workDue = true;
    CHECK(d.step(100) && lookAndMiss(d));
    CHECK(!d.step(10000));  // ten seconds into the two minutes
    d.in.demand = true;
    d.in.demandSeq = 2;
    CHECK(d.step(100) && d.p.state() == Planner::CONNECTING);
    CHECK(lookAndMiss(d));
    for (int i = 0; i < 400; i++) CHECK(!d.step(100));  // the press is still "active" for 40 s: no loop of looks
    CHECK(d.p.awayLooks() == 1);                         // a look someone asked for is extra: the quick ones are not used up
    d.in.demandSeq = 3;
    CHECK(d.step(100) && lookAndMiss(d));
    CHECK(d.p.awayLooks() == 1);
    d.in.demand = false;
    // ... and when the hotspot is on at last, the press that finds it holds the radio as a press does
    d.in.demand = true;
    d.in.demandSeq = 4;
    CHECK(d.step(100));
    d.in.online = true;
    d.in.workDue = false;
    for (int i = 0; i < 300; i++) CHECK(d.step(100));  // 30 s on, held by the press
    d.in.demand = false;
    CHECK(d.untilOff(5000, 100) && d.p.lastOutcome() == Planner::DONE);
  }
  {  // the first look of an absence counts even when a key press started it (the next looks then take one answer)
    Drive d;
    d.in.demand = true;
    d.in.demandSeq = 2;
    CHECK(d.step(100) && lookAndMiss(d) && d.p.awayLooks() == 1);
  }
  {  // any button (BOOT has no business with the network, but it means someone is there): look now
    Drive d;
    d.in.workDue = true;
    CHECK(d.step(100) && lookAndMiss(d));
    CHECK(!d.step(30000));
    d.p.lookNow();
    CHECK(d.step(100) && d.p.state() == Planner::CONNECTING);
    d.p.lookNow();  // while a look is under way: nothing
    CHECK(lookAndMiss(d) && d.p.awayLooks() == 1);
    CHECK(d.p.retryAtMs() - d.t == kAwayQuickMs);  // and the looks by the clock go on from there
    CHECK(!d.step(60000));
  }
  {  // ... but only then: it does not cut a back-off short (the network is there and refuses), and it starts
     // nothing when all is well
    Drive d;
    d.in.workDue = true;
    d.step(100);
    CHECK(d.untilOff(60000) && d.p.lastOutcome() == Planner::FAILED);
    const uint32_t retryAt = d.p.retryAtMs();
    d.p.lookNow();
    CHECK(d.p.retryAtMs() == retryAt && !d.step(1000));
    Drive fine;
    fine.p.lookNow();
    for (int i = 0; i < 50; i++) CHECK(!fine.step(1000));
  }
  {  // away and failures keep separate books: a refusal after five looks waits a minute, not a quarter of an hour
    Drive d;
    d.in.workDue = true;
    for (int i = 0; i < 6; i++) {
      CHECK(d.untilOn(400000, 100));
      CHECK(lookAndMiss(d));
    }
    CHECK(d.p.awayLooks() == 6 && d.p.failures() == 0);
    CHECK(d.untilOn(400000, 100));
    CHECK(d.untilOff(60000, 100) && d.p.lastOutcome() == Planner::FAILED && d.p.failures() == 1);
    CHECK(d.p.retryAtMs() - d.t == kBackoffMs[0]);
  }
  {  // a connection that breaks and then finds the network gone (the router was switched off): away, in the same session
    Drive d;
    d.in.workDue = true;
    d.step(100);
    d.in.online = true;
    d.step(100);
    d.in.online = false;
    CHECK(d.step(100) && d.p.state() == Planner::CONNECTING);
    CHECK(lookAndMiss(d) && d.p.failures() == 0);
  }
}

// 32 bits of milliseconds wrap after 49.7 days, and a time more than 24.8 days back looks like one still to come.
void testWrap() {
  section("radio plan: after weeks of running");
  CHECK(!pending(1000, 0) && pending(1000, 1001) && !pending(1000, 1000) && !pending(1000, 999));
  CHECK(!pending(0x90000000u, 0) && !pending(0xFFFFFFFFu, 0));  // 0 is "none" at every age, not a time 25 days ahead
  CHECK(pending(0xFFFFFFF0u, 0x10u));                 // across the wrap: 32 ms to go
  CHECK(pending(1000, 1000 + 0x7FFFFFFFu));           // (the far end of what can be told)
  CHECK(demandUntil(5) != 0 && demandUntil(0u - kDemandMs) != 0);  // never the "none" value
  CHECK(demandActive(5, demandUntil(5)) && !demandActive(5 + kDemandMs + 2, demandUntil(5)));
  CHECK(due(1000, 1000) && due(1001, 1000) && !due(999, 1000) && due(5, 0xFFFFFFF0u));
  {  // keepDue: work that is due stays due, however long ago that was
    uint32_t at = 1000;
    uint32_t now = 1000;
    int flips = 0;
    for (uint64_t step = 0; step < 70ULL * 24 * 3600 * 1000; step += 3600000ULL) {  // 70 days, looked at every hour
      now = (uint32_t)(1000 + step);
      keepDue(now, &at);
      if (!due(now, at)) flips++;
    }
    CHECK(flips == 0);
    uint32_t bare = 1000;  // without it: "not due" for 24.8 of those days
    int bareFlips = 0;
    for (uint64_t step = 0; step < 70ULL * 24 * 3600 * 1000; step += 3600000ULL) {
      if (!due((uint32_t)(1000 + step), bare)) bareFlips++;
    }
    CHECK(bareFlips > 24 * 20);
    uint32_t ahead = now + 60000;  // a time still to come is left alone
    keepDue(now, &ahead);
    CHECK(ahead == now + 60000);
  }
  // the planner starts sessions at every age: at the start, past 2^31 ms (24.8 days) and across 2^32 (49.7 days)
  for (uint32_t start : {100000u, 0x7FFFF000u, 0x80000100u, 0xC0000000u, 0xFFFF0000u}) {
    Drive d;
    d.t = start;
    int sessions = 0;
    for (int n = 0; n < 12; n++) {  // twelve sessions ten minutes apart: two hours, which crosses the boundary in question
      d.in.workDue = true;
      const bool on = d.step(100);
      if (!on) printf("  no session at t = 0x%08x (start 0x%08x)\n", (unsigned)d.t, (unsigned)start);
      CHECK(on);
      d.in.online = true;
      d.step(3000);
      d.in.workDue = false;
      d.in.online = true;
      if (d.untilOff(10000, 100)) sessions++;
      d.in.online = false;
      for (int i = 0; i < 600; i++) d.step(1000);
    }
    CHECK(sessions == 12 && d.p.sessions() == 12);
  }
  {  // a wait that ran out long ago does not come round again: a give-up, then nothing due for 30 days, then work
    Drive d;
    d.in.workDue = true;
    d.step(100);
    CHECK(d.untilOff(60000));
    d.in.workDue = false;
    for (int day = 0; day < 30; day++) {
      for (int i = 0; i < 24; i++) CHECK(!d.step(3600000));
    }
    CHECK(d.p.retryAtMs() == 0);
    d.in.workDue = true;
    CHECK(d.step(100));
  }
}

void testSpotifyRules() {
  section("radio plan: what Spotify needs of the radio");
  SpotifyNeeds n;
  CHECK(!spotifyHolds(n));
  n.playing = true;
  CHECK(spotifyHolds(n));  // live (the default): music holds the radio
  n.live = false;
  CHECK(!spotifyHolds(n));  // spotify_live = off: it does not
  n.pausedRecently = true;
  CHECK(!spotifyHolds(n));
  n.live = true;
  n.playing = false;
  CHECK(spotifyHolds(n));  // paused a moment ago, live
  for (bool live : {false, true}) {  // a command that waits and the Now Playing page hold it either way
    SpotifyNeeds c;
    c.live = live;
    c.commandWaiting = true;
    CHECK(spotifyHolds(c));
    c.commandWaiting = false;
    c.pageOpen = true;
    CHECK(spotifyHolds(c));
  }
  // the next look: when the track should be over, and a moment more
  CHECK(trackLookAfterMs(210000, 0) == 210000 + kTrackEndSlackMs);
  CHECK(trackLookAfterMs(210000, 200000) == 10000 + kTrackEndSlackMs);
  CHECK(trackLookAfterMs(210000, 210000) == kTrackEndSlackMs);  // over already by Spotify's own figures: just the moment
  CHECK(trackLookAfterMs(210000, 250000) == kTrackEndSlackMs);
  CHECK(kTrackEndSlackMs < kSettleMs);  // a look that finds the track on its last second is repeated within the same session
}

// A model of the network task's rules in sync mode (net_task.cpp): what is due, how long each thing takes,
// what the WiFi stack answers and what a joined radio does about it.  One pass every 100 ms.
enum NetState { NET_NOT_SET, NET_UP, NET_ABSENT, NET_REFUSING };

struct NetDay {
  Planner p;
  wifipick::Picker pick;
  Search search;
  NetState net[2] = {NET_UP, NET_NOT_SET};  // the main network and the backup
  uint32_t connectMs = 3000;  // a network that is up is joined this long after the attempt on it began
  uint32_t scanMs = 2400;     // one that is not answers "no such network" (or refuses) this long after, and again every so long
  bool internetUp = true;
  bool spotify = false;  // configured and linked
  bool live = true;      // spotify_live
  bool playing = false;  // the phone is playing
  uint32_t trackMs = 210000;
  uint32_t t = 1000;
  // the session
  bool radio = false, online = false;
  uint32_t sessionStart = 0, lastBegin = 0, answerAt = 0, busyUntil = 0;
  int attemptNet = -1, joinedNet = -1;
  unsigned notFound = 0, other = 0;
  // the network task's timers
  bool ntpEver = false;
  uint32_t ntpAt = 0, weatherAt = 0, ntpRetryAt = 0;
  int weatherFails = 0;
  bool peekDone = false;
  uint32_t demandUntilMs = 0, demandSeq = 1;
  // the player, and what the clock believes of it
  uint32_t trackStart = 0, trackNo = 0;
  bool seenPlaying = false;
  uint32_t seenTrackNo = 0, lookAt = 0, nextPollAt = 0;
  // what happened
  uint32_t weatherFetches = 0, ntpSyncs = 0, peeks = 0, looks = 0, awayEnds = 0, failedEnds = 0;
  uint32_t lastWeather = 0, lastNtp = 0, maxWeatherGap = 0, maxNtpGap = 0;
  uint32_t longestSession = 0, staleMs = 0, staleRun = 0, maxStaleRun = 0;
  explicit NetDay(uint32_t timeout = 25000) : p(true, timeout) { pick.configure(true, false); }

  void setNets(NetState mainNet, NetState backup) {
    net[0] = mainNet;
    net[1] = backup;
    pick.configure(mainNet != NET_NOT_SET, backup != NET_NOT_SET);
  }
  void press() {
    demandSeq++;
    demandUntilMs = demandUntil(t);
  }
  void play(bool on) {
    playing = on;
    trackStart = t;
    trackNo++;
  }
  void skip() {
    trackStart = t;
    trackNo++;
  }
  // wakes the radio on its own only when it is overdue by the grace time
  bool ntpDue() const { return (!ntpEver || due(t, ntpAt + kNtpGraceMs)) && due(t, ntpRetryAt); }
  // asked in a session that is going on anyway: due within ten minutes counts
  bool ntpNear() const { return (!ntpEver || due(t, ntpAt - kNtpEarlyMs)) && due(t, ntpRetryAt); }
  bool weatherDue() const { return due(t, weatherAt); }
  bool peekDue() const { return spotify && !peekDone; }
  bool lookDue() const { return spotify && !live && seenPlaying && lookAt != 0 && due(t, lookAt); }

  void poll() {  // a look at the player
    seenPlaying = playing;
    seenTrackNo = trackNo;
    lookAt = playing ? ((t + trackLookAfterMs(trackMs, t - trackStart)) | 1u) : 0;
    nextPollAt = t + 10000;
  }

  void tick() {
    t += 100;
    if (playing && (uint32_t)(t - trackStart) >= trackMs) {  // the next track begins
      trackStart += trackMs;
      trackNo++;
    }
    if (seenPlaying && (seenTrackNo != trackNo || !playing)) {  // the screen shows something that is no longer so
      staleMs += 100;
      staleRun += 100;
      if (staleRun > maxStaleRun) maxStaleRun = staleRun;
    } else {
      staleRun = 0;
    }

    // what the WiFi stack does with the attempt under way
    if (radio && !online && attemptNet >= 0) {
      const NetState s = net[attemptNet];
      if (s == NET_UP && (uint32_t)(t - lastBegin) >= connectMs) {
        online = true;
        joinedNet = attemptNet;
        pick.joined(attemptNet ? wifipick::NET_BACKUP : wifipick::NET_MAIN, t);
        search.forget();
      } else if (s != NET_UP && due(t, answerAt)) {
        if (s == NET_ABSENT) notFound++;
        else other++;
        answerAt = t + scanMs;
      }
    }
    if (radio && online && net[joinedNet] != NET_UP) {  // the connection broke: the stack tries the same network again
      online = false;
      pick.lost();
      search.forget();
      lastBegin = t;
      notFound = other = 0;
      answerAt = t + scanMs;
    }

    Inputs in;
    in.online = radio && online;
    SpotifyNeeds needs;
    needs.playing = seenPlaying;
    needs.live = live;
    in.workDue = ntpDue() || weatherDue() || peekDue() || lookDue();
    in.holdSpotify = spotify && spotifyHolds(needs);
    in.demand = demandActive(t, demandUntilMs);
    in.demandSeq = demandSeq;
    in.absent = radio && search.noneInRange();
    const bool was = radio;
    radio = p.update(t, in);
    if (radio && !was) {
      sessionStart = t;
      peekDone = false;
      nextPollAt = t;
      search.begin(net[0] != NET_NOT_SET, net[1] != NET_NOT_SET, p.awayLooks() == 0 ? kAwayScansFirst : kAwayScansAgain);
      attemptNet = -1;
      lastBegin = 0;
      online = false;
    }
    if (!radio && was) {
      if ((uint32_t)(t - sessionStart) > longestSession) longestSession = t - sessionStart;
      pick.radioOff();
      online = false;
      if (p.takeGaveUp()) {
        if (p.lastOutcome() == Planner::AWAY) awayEnds++;
        else failedEnds++;
        seenPlaying = false;  // the network is gone: a track nobody can follow is taken off the screen
      }
    }
    if (!radio) return;

    if (!online) {  // the attempts: one network at a time, the other one at once when this one is not there
      const bool moveOn = attemptNet >= 0 && search.note(attemptNet, notFound, other) && !search.noneInRange();
      if (lastBegin == 0 || (uint32_t)(t - lastBegin) > 20000 || moveOn) {
        attemptNet = pick.nextAttempt() == wifipick::NET_BACKUP ? 1 : 0;
        lastBegin = t ? t : 1;
        notFound = other = 0;
        answerAt = t + scanMs;
      }
      return;
    }
    // what the task does once joined: the work of one loop, in the order of net_task.cpp
    if (t >= busyUntil) {
      if (ntpNear()) {
        busyUntil = t + 400;
        if (internetUp) {
          ntpEver = true;
          ntpAt = t + 3600000UL;
          if (lastNtp && t - lastNtp > maxNtpGap) maxNtpGap = t - lastNtp;
          lastNtp = t;
          ntpSyncs++;
        } else {  // no reply: wait twenty seconds, try again in five minutes
          busyUntil = t + 20000;
          ntpRetryAt = t + 20000 + 300000;
        }
      } else if (weatherDue()) {
        busyUntil = t + 2500;
        if (internetUp) {
          weatherAt = t + 15 * 60000UL;
          weatherFails = 0;
          if (lastWeather && t - lastWeather > maxWeatherGap) maxWeatherGap = t - lastWeather;
          lastWeather = t;
          weatherFetches++;
        } else {
          weatherAt = t + retryAfterMs(++weatherFails);  // gently: the radio is dear
        }
      } else if (peekDue()) {  // the session's first look at the player (TLS and all: three seconds)
        busyUntil = t + 3000;
        peekDone = true;
        peeks++;
        poll();
      } else if (spotify && due(t, nextPollAt) && (seenPlaying || lookDue())) {  // the ten second poll while it plays and the radio is on
        busyUntil = t + 500;
        looks++;
        poll();
      }
    }
  }
  void run(uint32_t ms) {
    const uint32_t end = t + ms;
    while ((int32_t)(end - t) > 0) tick();
  }
};

void testDay() {
  section("radio plan: a simulated day");
  {  // a quiet day with a good network: a session every quarter of an hour, the radio on for a sliver of it
    NetDay d(kConnectTimeoutMs);
    d.run(24 * 3600000UL);
    const double duty = d.p.radioOnMs(d.t) / (24.0 * 3600000.0);
    if (duty > 0.015) printf("  radio on %.2f %% of the day\n", duty * 100);
    printf("  (a quiet day: radio on %.2f %%, %u sessions, longest %.1f s)\n", duty * 100, (unsigned)d.p.sessions(), d.longestSession / 1000.0);
    CHECK(duty < 0.015);
    CHECK(d.weatherFetches >= 94 && d.weatherFetches <= 98);
    CHECK(d.ntpSyncs >= 23 && d.ntpSyncs <= 26);  // about every hour, in a session that is going on anyway
    CHECK(d.p.sessions() >= 94 && d.p.sessions() <= 104);  // the time sync does not need sessions of its own
    CHECK(d.maxWeatherGap <= 15 * 60000UL + 15000);  // never later than a quarter of an hour and a few seconds
    CHECK(d.maxNtpGap <= 3600000UL + kNtpGraceMs + 30000);  // an hour, and a few minutes when no session comes by
    CHECK(d.longestSession <= 20000);  // a session is seconds, not minutes
    CHECK(d.p.failures() == 0 && d.awayEnds == 0 && d.failedEnds == 0);
  }
  {  // with Spotify linked, each session also looks at it once: a few more seconds, nothing more
    NetDay d(kConnectTimeoutMs);
    d.spotify = true;
    d.run(24 * 3600000UL);
    const double duty = d.p.radioOnMs(d.t) / (24.0 * 3600000.0);
    if (duty >= 0.025 || d.peeks < 94 || d.peeks > 125) printf("  radio on %.2f %%, %u peeks, %u sessions\n", duty * 100, (unsigned)d.peeks, (unsigned)d.p.sessions());
    printf("  (with Spotify linked: radio on %.2f %%, %u sessions)\n", duty * 100, (unsigned)d.p.sessions());
    CHECK(duty < 0.025 && d.peeks >= 94 && d.peeks <= 125);  // (a few more sessions than weather fetches: the hourly time sync on its own)
    CHECK(d.longestSession <= 25000);
  }
  {  // three hours with a router that is there and refuses the clock (a changed password): a try after 1, 2, 5,
     // 10 minutes and then every quarter of an hour, the radio on for the time it takes to give up each time;
     // and the first good session within a quarter of an hour of the password being put right
    NetDay d;  // (a clock with one network: 25 s is long enough to give up)
    d.run(2 * 3600000UL);
    d.net[0] = NET_REFUSING;
    const uint32_t onBefore = d.p.radioOnMs(d.t);
    d.run(3 * 3600000UL);
    const uint32_t onOutage = d.p.radioOnMs(d.t) - onBefore;
    const uint32_t tries = (uint32_t)d.p.failures();
    printf("  (three hours with a router that refuses: %u failed sessions, radio on %.1f %% of the time)\n", (unsigned)tries, onOutage / (3 * 36000.0));
    if (tries < 14 || tries > 18) printf("  %u failed sessions in three hours\n", (unsigned)tries);
    CHECK(tries >= 14 && tries <= 18);
    CHECK(d.awayEnds == 0 && d.failedEnds == tries);
    CHECK(onOutage <= (tries + 1) * 26000UL);
    CHECK(onOutage < 0.04 * 3 * 3600000.0);
    const uint32_t weatherBefore = d.weatherFetches;
    d.net[0] = NET_UP;
    const uint32_t back = d.t;
    while (d.weatherFetches == weatherBefore && d.t - back < 20 * 60000UL) d.tick();
    CHECK(d.weatherFetches > weatherBefore && d.t - back <= 15 * 60000UL + 30000);
    CHECK(d.p.failures() == 0);
  }
  {  // the router is there, the internet is not: the weather and the time are retried gently, nothing stays on
    NetDay d(kConnectTimeoutMs);
    d.run(3600000UL);
    d.internetUp = false;
    const uint32_t onBefore = d.p.radioOnMs(d.t);
    d.run(2 * 3600000UL);
    const double on = (double)(d.p.radioOnMs(d.t) - onBefore) / (2 * 3600000.0);
    if (on > 0.15) printf("  radio on %.1f %% with no internet\n", on * 100);
    printf("  (router up, no internet: radio on %.1f %%)\n", on * 100);
    CHECK(on < 0.05);  // retried after 1, 2, 5, 10 minutes and then every 15 (and the time every 5)
    CHECK(d.longestSession <= 60000);
  }
  {  // key presses: each gets a joined radio in seconds, and the radio goes off again some 45 s after the last
    NetDay d(kConnectTimeoutMs);
    d.run(3600000UL);
    for (int press = 0; press < 12; press++) {
      d.run(7 * 60000UL + press * 13000UL);  // (some of them close to the quarter of an hour sessions)
      const uint32_t t0 = d.t;
      d.press();
      bool joined = false;
      while (!joined && d.t - t0 < 20000) {
        d.tick();
        joined = d.online;
      }
      CHECK(joined && d.t - t0 <= d.connectMs + 800);
      d.run(kDemandMs - 3000 - (d.t - t0));
      CHECK(d.online);  // still held
      d.run(3000 + kSettleMs + 2000);
      CHECK(!d.online || d.t - t0 < kDemandMs + 6000);
    }
  }
  {  // music: the radio stays on for as long as it plays and goes off a few seconds after
    NetDay d(kConnectTimeoutMs);
    d.spotify = true;
    d.run(3600000UL);
    d.play(true);
    d.run(16 * 60000UL);  // (the next session finds it playing)
    d.run(2 * 3600000UL);
    CHECK(d.radio && d.online);
    CHECK(d.maxStaleRun <= 12000);  // the strip is never more than a poll behind
    d.play(false);
    const uint32_t t0 = d.t;
    while (d.radio && d.t - t0 < 30000) d.tick();
    CHECK(!d.radio && d.t - t0 <= 10000 + kSettleMs + 3500);  // (the next ten second poll sees that it stopped)
    const uint32_t onAfterMusic = d.p.radioOnMs(d.t);
    d.run(4 * 3600000UL);
    CHECK(d.p.radioOnMs(d.t) - onAfterMusic < 0.03 * 4 * 3600000.0);
  }
}

// What the new rule is for: a clock that is somewhere its network is not.
void testAwayDays() {
  section("radio plan: a clock away from its network");
  {  // taken to work for eight hours, one network: a look every 2 minutes at first, then every 5, each a scan long;
     // the radio is on for a fraction of what the old "45 seconds, then back off" cost, and home is found within
     // 5 minutes of coming back
    NetDay d;
    d.run(2 * 3600000UL);
    d.net[0] = NET_ABSENT;
    const uint32_t onBefore = d.p.radioOnMs(d.t);
    d.run(8 * 3600000UL);
    const uint32_t onAway = d.p.radioOnMs(d.t) - onBefore;
    printf("  (eight hours away, one network: %u looks, radio on %.2f %% of the time, longest session %.1f s)\n", (unsigned)d.awayEnds,
           onAway / (8 * 36000.0), d.longestSession / 1000.0);
    CHECK(d.failedEnds == 0 && d.p.failures() == 0);
    CHECK(d.awayEnds >= 95 && d.awayEnds <= 104);  // 5 quick ones, then one every 5 minutes (and the 2.5 s each takes)
    CHECK(onAway < 0.012 * 8 * 3600000.0);         // about one percent
    CHECK(d.longestSession <= 20000);              // (the longest was a good session before it left)
    const uint32_t weatherBefore = d.weatherFetches;
    d.net[0] = NET_UP;
    const uint32_t back = d.t;
    while (d.weatherFetches == weatherBefore && d.t - back < 20 * 60000UL) d.tick();
    CHECK(d.weatherFetches > weatherBefore && d.t - back <= kAwaySlowMs + 10000);
    CHECK(d.p.awayLooks() == 0);
  }
  {  // the first look of an absence asks twice (4.9 s), the later ones once (2.5 s)
    NetDay d;
    d.run(3600000UL);
    d.net[0] = NET_ABSENT;
    uint32_t lengths[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      while (!d.radio) d.tick();
      const uint32_t t0 = d.t;
      while (d.radio) d.tick();
      lengths[i] = d.t - t0;
    }
    CHECK(lengths[0] >= 4700 && lengths[0] <= 5200);
    CHECK(lengths[1] >= 2300 && lengths[1] <= 2800 && lengths[2] >= 2300 && lengths[2] <= 2800);
  }
  {  // with a backup network set, both are asked for, one after the other: twice the scan, still seconds
    NetDay d(kConnectTimeoutMs);
    d.setNets(NET_UP, NET_ABSENT);
    d.run(3600000UL);
    d.net[0] = NET_ABSENT;
    const uint32_t onBefore = d.p.radioOnMs(d.t);
    d.run(8 * 3600000UL);
    const uint32_t onAway = d.p.radioOnMs(d.t) - onBefore;
    printf("  (eight hours away, two networks: %u looks, radio on %.2f %% of the time)\n", (unsigned)d.awayEnds, onAway / (8 * 36000.0));
    CHECK(d.failedEnds == 0 && d.awayEnds >= 93 && d.awayEnds <= 104);
    CHECK(onAway < 0.022 * 8 * 3600000.0);
    // at work the phone's hotspot (the backup) is switched on: found at the next look, no button needed
    d.net[1] = NET_UP;
    const uint32_t on = d.t;
    while (!d.online && d.t - on < 20 * 60000UL) d.tick();
    CHECK(d.online && d.joinedNet == 1 && d.t - on <= kAwaySlowMs + 2 * d.scanMs + d.connectMs + 1000);
  }
  {  // ... and with a button, at once: KEY, or any button (the nudge), a few seconds after the hotspot is up
    for (int how = 0; how < 2; how++) {
      NetDay d(kConnectTimeoutMs);
      d.setNets(NET_ABSENT, NET_ABSENT);
      d.run(3600000UL);  // an hour away already: the looks are 5 minutes apart
      CHECK(d.awayEnds >= 10 && !d.radio);
      while (d.radio || d.p.retryAtMs() - d.t < 200000) d.tick();  // just after a look: the next is minutes off
      d.net[1] = NET_UP;
      const uint32_t on = d.t;
      if (how == 0) d.press();
      else d.p.lookNow();
      while (!d.online && d.t - on < 60000) d.tick();
      if (!d.online) printf("  not joined after a %s\n", how == 0 ? "key press" : "nudge");
      CHECK(d.online && d.joinedNet == 1 && d.t - on <= d.scanMs + d.connectMs + 1500);  // the main one is asked for first, then the hotspot
    }
  }
  {  // a key press where there is no network: one look of a few seconds, not 45 seconds of radio
    NetDay d;
    d.net[0] = NET_ABSENT;
    d.run(3600000UL);
    while (d.radio) d.tick();
    const uint32_t onBefore = d.p.radioOnMs(d.t);
    const uint32_t looksBefore = d.awayEnds;
    d.press();
    d.run(60000);
    const uint32_t cost = d.p.radioOnMs(d.t) - onBefore;
    CHECK(d.awayEnds == looksBefore + 1 || d.awayEnds == looksBefore + 2);  // (the press's look, and perhaps the clock's own, due in that minute)
    CHECK(cost <= 2 * 2800);
  }
  {  // the router restarts (a session finds it gone, and it comes back three minutes later): no failure is
     // counted, and the clock is on it again within two minutes of its return
    NetDay d;
    d.run(3600000UL);
    d.net[0] = NET_ABSENT;
    while (d.awayEnds == 0 && d.t < 3 * 3600000UL) d.tick();  // (the next session notices)
    CHECK(d.awayEnds == 1 && d.p.failures() == 0);
    d.run(3 * 60000UL);
    d.net[0] = NET_UP;
    const uint32_t back = d.t;
    while (!d.online && d.t - back < 10 * 60000UL) d.tick();
    CHECK(d.online && d.t - back <= kAwayQuickMs + d.connectMs + 500);
    CHECK(d.failedEnds == 0);
  }
  {  // gone in the middle of a session (the router loses power while the weather is fetched): the stack's own
     // reconnect hears "no such network" and the session ends as away, not after 25 seconds as a failure
    NetDay d;
    d.run(3600000UL);
    while (!d.online) d.tick();
    d.net[0] = NET_ABSENT;
    const uint32_t t0 = d.t;
    while (d.radio && d.t - t0 < 60000) d.tick();
    CHECK(!d.radio && d.t - t0 <= 2 * d.scanMs + 600 && d.awayEnds == 1 && d.failedEnds == 0);
  }
}

// spotify_live = off: the radio does not stay on for the music.
void testTrackMode() {
  section("radio plan: music with spotify_live = off");
  uint32_t onLive = 0, onTrack = 0;
  for (int mode = 0; mode < 2; mode++) {
    NetDay d(kConnectTimeoutMs);
    d.spotify = true;
    d.live = mode == 0;
    d.run(3600000UL);
    d.play(true);
    d.run(16 * 60000UL);  // (a session comes by and finds it playing)
    CHECK(d.seenPlaying);
    const uint32_t onBefore = d.p.radioOnMs(d.t);
    const uint32_t sessionsBefore = d.p.sessions();
    const uint32_t staleBefore = d.staleMs;
    d.maxStaleRun = 0;
    d.run(2 * 3600000UL);  // two hours of music, tracks of three and a half minutes
    const uint32_t on = d.p.radioOnMs(d.t) - onBefore;
    const uint32_t sessions = d.p.sessions() - sessionsBefore;
    if (mode == 0) {
      onLive = on;
      printf("  (two hours of music, live: radio on %.0f %%, the strip at most %.1f s behind)\n", on / 72000.0, d.maxStaleRun / 1000.0);
      CHECK(on > 0.99 * 7200000.0 && d.maxStaleRun <= 12000);
    } else {
      onTrack = on;
      printf("  (two hours of music, spotify_live = off: radio on %.1f %%, %u sessions, the strip at most %.1f s behind, %.1f %% of the time)\n",
             on / 72000.0, (unsigned)sessions, d.maxStaleRun / 1000.0, (d.staleMs - staleBefore) / 72000.0);
      CHECK(sessions >= 34 && sessions <= 46);  // one a track (34 of them), and the weather's when it does not fall together with one
      CHECK(on < 0.07 * 7200000.0);
      CHECK(d.maxStaleRun <= kTrackEndSlackMs + d.connectMs + 3000 + 1000);  // the next title a few seconds after the track changed
      CHECK(d.longestSession <= 25000);
      // the music stops (the phone is paused): the look at the end of the track sees it, and the looks stop
      d.play(false);
      d.run(d.trackMs + 20000);
      CHECK(!d.seenPlaying);
      const uint32_t s2 = d.p.sessions();
      d.run(3600000UL);
      CHECK(d.p.sessions() - s2 <= 6);  // back to the weather's four an hour (and the hourly time sync)
    }
  }
  CHECK(onTrack * 10 < onLive);  // what it is for

  {  // a track skipped on the phone shows late, and not later than the old one would have run
    NetDay d(kConnectTimeoutMs);
    d.spotify = true;
    d.live = false;
    d.run(3600000UL);
    d.play(true);
    d.run(16 * 60000UL);
    // 20 s into a track, the radio off, and no weather or time session in the next minutes (it would look too)
    while ((uint32_t)(d.t - d.trackStart) < 20000 || (uint32_t)(d.t - d.trackStart) > 30000 || d.radio ||
           (int32_t)(d.weatherAt - d.t) < 400000 || (int32_t)(d.ntpAt - d.t) < 400000) {
      d.tick();
    }
    d.maxStaleRun = 0;
    d.skip();
    d.run(d.trackMs + 30000);
    CHECK(d.maxStaleRun >= 150000);  // (it does show late: the rest of the track that was skipped.  That is the price.)
    CHECK(d.maxStaleRun <= d.trackMs + 10000);
    CHECK(d.seenTrackNo == d.trackNo || d.radio);
  }
  {  // ... and never later than the next session that comes by for the weather: every session looks at the player
    NetDay d(kConnectTimeoutMs);
    d.spotify = true;
    d.live = false;
    d.trackMs = 3600000;  // an hour-long episode
    d.run(3600000UL);
    d.play(true);
    d.run(16 * 60000UL);
    while (d.radio) d.tick();
    d.maxStaleRun = 0;
    d.skip();
    d.run(40 * 60000UL);
    CHECK(d.maxStaleRun <= 15 * 60000UL + 15000);
  }
  {  // KEY is live either way: a press brings the radio up and the player is looked at straight away
    NetDay d(kConnectTimeoutMs);
    d.spotify = true;
    d.live = false;
    d.run(3600000UL);
    d.play(true);
    d.run(16 * 60000UL);
    while (d.radio) d.tick();
    while ((uint32_t)(d.t - d.trackStart) < 20000 || (uint32_t)(d.t - d.trackStart) > 30000 || d.radio) d.tick();
    d.skip();
    d.press();
    const uint32_t t0 = d.t;
    while (d.seenTrackNo != d.trackNo && d.t - t0 < 30000) d.tick();
    CHECK(d.seenTrackNo == d.trackNo && d.t - t0 <= d.connectMs + 3000 + 1000);
  }
  {  // the network goes away while music plays: the track is taken off the screen at the first look that fails
    NetDay d;
    d.spotify = true;
    d.live = false;
    d.run(3600000UL);
    d.play(true);
    d.run(16 * 60000UL);
    while (d.radio) d.tick();
    CHECK(d.seenPlaying);
    d.net[0] = NET_ABSENT;
    d.run(d.trackMs + 30000);
    CHECK(!d.seenPlaying && d.awayEnds >= 1);
  }
}

void testClockPolicy() {
  section("CPU clock policy");
  using namespace clockpolicy;
  Needs none, radio, usb, poke;
  radio.radio = true;
  usb.usbHost = true;
  poke.poked = true;
  CHECK(target(none, 80, 20) == 20);
  CHECK(target(radio, 80, 20) == 80 && target(usb, 80, 20) == 80 && target(poke, 80, 20) == 80);
  Needs all;
  all.radio = all.usbHost = all.poked = true;
  CHECK(target(all, 80, 20) == 80);
  // no idle clock, or one that is not below the working clock: always the working clock
  CHECK(target(none, 80, 0) == 80 && target(none, 80, 80) == 80 && target(none, 80, 160) == 80);
  CHECK(target(none, 240, 80) == 80 && target(none, 160, 40) == 40);
  // up at once, down only when quiet, nothing to do when it is there already
  CHECK(mayChange(20, 80, false) && mayChange(20, 80, true));
  CHECK(!mayChange(80, 20, false) && mayChange(80, 20, true));
  CHECK(!mayChange(80, 80, true) && !mayChange(20, 20, false));

  // the settings give the idle clock: never above the working one, and 20 MHz is the lowest on offer
  Settings st;
  st.cpuIdle = CPUIDLE_20;
  CHECK(target(none, st.cpuMhz(), st.cpuIdleMhz()) == 20);
  st.cpuIdle = CPUIDLE_40;
  CHECK(target(none, st.cpuMhz(), st.cpuIdleMhz()) == 40);
  st.cpuIdle = CPUIDLE_80;  // = cpu_mhz = 80: nothing to do
  CHECK(target(none, st.cpuMhz(), st.cpuIdleMhz()) == 80);
  int lowest = 1000;
  for (int v = 0; v < 8; v++) {
    Settings any;
    any.cpuSpeed = CPU_240;
    any.cpuIdle = (uint8_t)v;
    if (any.cpuIdleMhz() > 0 && any.cpuIdleMhz() < lowest) lowest = any.cpuIdleMhz();
  }
  CHECK(lowest == 20);
}

void testConsolePolicy() {
  section("console policy");
  using namespace consolepolicy;
  CHECK((int)CONSOLE_ON == (int)ON && (int)CONSOLE_AUTO == (int)AUTO && (int)CONSOLE_OFF == (int)OFF);  // the setting's numbers are the policy's
  {  // on: never, with or without a computer
    Watch w;
    for (uint32_t t = 0; t < 3600000; t += 1000) CHECK(!w.shouldStop(ON, false, (t / 60000) % 2 == 0, t));
  }
  {  // off: at once, a computer on the port or not
    Watch w;
    CHECK(w.shouldStop(OFF, false, true, 3000));
    Watch v;
    CHECK(v.shouldStop(OFF, false, false, 3000));
  }
  {  // auto, no computer: ten seconds after the first call, not before
    Watch w;
    uint32_t t = 2500;  // (the loop starts a few seconds after power-on)
    bool stop = false;
    while (!stop && t < 60000) {
      stop = w.shouldStop(AUTO, false, false, t);
      if (!stop) t += 4;
    }
    CHECK(stop && t - 2500 >= kNoHostMs && t - 2500 <= kNoHostMs + 8);
  }
  {  // auto, on a computer: it stays for as long as the computer is seen, and goes ten seconds after the cable is pulled
    Watch w;
    uint32_t t = 2500;
    for (; t < 3600000; t += 100) CHECK(!w.shouldStop(AUTO, false, true, t));
    const uint32_t pulled = t - 100;  // the last time the computer was seen
    bool stop = false;
    while (!stop && t - pulled < 60000) {
      t += 100;
      stop = w.shouldStop(AUTO, false, false, t);
    }
    CHECK(stop && t - pulled >= kNoHostMs && t - pulled <= kNoHostMs + 200);
  }
  {  // auto: a computer that shows up within the ten seconds (the port takes a moment after power-on) keeps it, and
     // short gaps (the port being reset by an upload) do not count up
    Watch w;
    uint32_t t = 1000;
    for (; t < 9000; t += 100) CHECK(!w.shouldStop(AUTO, false, false, t));
    for (; t < 20000; t += 100) CHECK(!w.shouldStop(AUTO, false, true, t));
    for (int round = 0; round < 20; round++) {
      for (int i = 0; i < 80; i++, t += 100) CHECK(!w.shouldStop(AUTO, false, false, t));  // 8 s without
      for (int i = 0; i < 10; i++, t += 100) CHECK(!w.shouldStop(AUTO, false, true, t));   // 1 s with
    }
  }
  {  // KEY held at start-up keeps the console whatever the setting, for the whole run
    for (int mode : {(int)ON, (int)AUTO, (int)OFF}) {
      Watch w;
      for (uint32_t t = 0; t < 600000; t += 500) CHECK(!w.shouldStop(mode, true, false, t));
    }
  }
  {  // across the wrap of millis()
    Watch w;
    uint32_t t = 0xFFFFF000u;
    CHECK(!w.shouldStop(AUTO, false, true, t));
    bool stop = false;
    uint32_t waited = 0;
    while (!stop && waited < 60000) {
      waited += 100;
      stop = w.shouldStop(AUTO, false, false, t + waited);
    }
    CHECK(stop && waited >= kNoHostMs && waited <= kNoHostMs + 100);
  }
}

void testPicker() {
  section("radio plan: the picker across sessions");
  using namespace wifipick;
  Picker pk;
  pk.configure(true, true);
  CHECK(pk.nextAttempt() == NET_MAIN);
  pk.joined(NET_MAIN, 1000);
  pk.radioOff();
  CHECK(pk.nextAttempt() == NET_MAIN && pk.current() == NET_MAIN);  // on the main network: back to it
  // on the backup: the next session starts there, not with a failed attempt on the main one
  Picker bk;
  bk.configure(true, true);
  bk.nextAttempt();
  bk.nextAttempt();
  bk.joined(NET_BACKUP, 5000);
  CHECK(bk.onBackup());
  bk.radioOff();
  CHECK(bk.nextAttempt() == NET_BACKUP && bk.onBackup());
  CHECK(bk.nextAttempt() == NET_MAIN);  // if that fails, the other one, as always
  // ... and the look for the main network goes on by the clock
  bk.joined(NET_BACKUP, 10000);
  bk.radioOff();
  CHECK(!bk.timeToLookForMain(10000 + 5 * 60000UL) && bk.timeToLookForMain(10000 + 10 * 60000UL));
  // no backup: nothing changes
  Picker one;
  one.configure(true, false);
  one.nextAttempt();
  one.joined(NET_MAIN, 1);
  one.radioOff();
  CHECK(one.nextAttempt() == NET_MAIN);
}

// After the clock changes the frame planner expects the costs of the new clock.  A model of the UI loop at
// 80 MHz and at 20, where a frame takes four times as long, and a switch between them in the middle of the run.
void testFramePlanClock() {
  section("frame plan: after a clock change");
  FramePlanner p(20000);
  CHECK(p.mhz() == 80 && p.drawCostUs() == 30000 && p.sendCostUs() == 15000);
  p.clockChanged(80, 20);
  CHECK(p.mhz() == 20 && p.drawCostUs() == 120000 && p.sendCostUs() == 60000);
  p.clockChanged(20, 80);
  CHECK(p.drawCostUs() == 30000 && p.sendCostUs() == 15000);
  p.clockChanged(80, 80);  // no change
  CHECK(p.drawCostUs() == 30000);
  p.clockChanged(0, 20);   // nonsense is ignored
  p.clockChanged(80, 0);
  CHECK(p.drawCostUs() == 30000 && p.mhz() == 80);
  p.clockChanged(80, 10);
  CHECK(p.drawCostUs() == 240000 && p.sendCostUs() == 120000);  // more than the 150 ms ceiling of the fast clocks

  // a long stall must not stretch the plan, at 20 MHz as at 80: a measurement is capped at 150 ms * 80 / MHz
  FramePlanner q(20000);
  q.drew(100, 400000, true);
  CHECK(q.drawCostUs() == (30000u * 3 + 150000u) / 4);
  FramePlanner slow(20000);
  slow.clockChanged(80, 20);
  for (int i = 0; i < 40; i++) slow.drew(100 + i, 5000000, true);  // always stalled: the ceiling is 600 ms here
  CHECK(slow.drawCostUs() > 590000u && slow.drawCostUs() <= 600000u);

  // the first frame after a switch is planned with the new costs: a simulated second-by-second loop, the
  // clock dropping from 80 to 20 MHz at second 50 and coming back at second 150, with draws and sends that
  // take 4 times as long while it is low; a frame is late if it goes out after its second has begun
  FramePlanner pl(20000);
  int64_t t = 0;
  int lateFrames = 0, frames = 0;
  int mhz = 80;
  auto cost = [&](int64_t us) { return mhz == 80 ? us : us * 4; };
  time_t lastSent = 0;
  time_t prepared = 0;
  bool havePrepared = false;
  const int64_t end = 300 * 1000000LL;
  while (t < end) {
    const time_t sec = (time_t)(t / 1000000);
    const int sec2 = (int)sec;
    if ((sec2 == 50 && mhz == 80 && !pl.prepared()) || (sec2 == 150 && mhz == 20 && !pl.prepared())) {
      const int to = mhz == 80 ? 20 : 80;
      pl.clockChanged((uint32_t)mhz, (uint32_t)to);
      mhz = to;
    }
    const FramePlanner::Step st = pl.next(sec, (int32_t)(t % 1000000), false);
    switch (st.action) {
      case FramePlanner::DRAW_NOW:
        t += cost(30000);
        pl.drew(st.sec, (uint32_t)cost(30000), false);
        t += cost(15000);
        pl.sent((uint32_t)cost(15000));
        lastSent = st.sec;
        break;
      case FramePlanner::PREPARE:
        t += cost(30000);
        pl.drew(st.sec, (uint32_t)cost(30000), true);
        prepared = st.sec;
        havePrepared = true;
        break;
      case FramePlanner::SEND:
        t += st.waitUs;
        t += cost(15000);
        if (havePrepared && sec2 > 5) {
          frames++;
          // on the glass 20 ms (the panel) after it was sent: late if that is after its second began
          if ((t + 20000) > (int64_t)prepared * 1000000LL + 30000) lateFrames++;  // 30 ms of grace
        }
        pl.sent((uint32_t)cost(15000));
        lastSent = prepared;
        break;
      case FramePlanner::SLEEP:
        t += 4000;
        break;
      default:
        t += 4000;
        break;
    }
  }
  (void)lastSent;
  printf("  (%d of %d frames late around the two clock changes)\n", lateFrames, frames);
  CHECK(frames > 250);
  CHECK(lateFrames <= 2);  // a frame or two at the moment of a switch is what it costs; the rest is on time

  // After 1.12 a frame is prepared for most of a second, so the clock mostly changes while one waits to be sent: up at
  // once, down only when the planner is quiet (clock_policy.h).  The send cost is what scales for that frame; it still
  // goes out on time, and so does every one after it.  The clock wants the other speed every 3.7 s, so the switches
  // land all over the second.
  {
    FramePlanner q(20000);
    int64_t tq = 0;
    int m = 80, switches = 0, whilePrepared = 0, late = 0, sent = 0;
    int64_t worst = 0;
    auto costAt = [&](int64_t us) { return m == 80 ? us : us * 4; };
    while (tq < end) {
      const time_t sec = (time_t)(tq / 1000000);
      const int32_t usec = (int32_t)(tq % 1000000);
      const int want = (tq / 3700000) % 2 ? 20 : 80;
      if (clockpolicy::mayChange(m, want, q.quiet(usec))) {
        whilePrepared += q.prepared();
        q.clockChanged((uint32_t)m, (uint32_t)want);
        m = want;
        switches++;
      }
      const FramePlanner::Step st = q.next(sec, usec, false);
      switch (st.action) {
        case FramePlanner::DRAW_NOW:
          tq += costAt(30000);
          q.drew(st.sec, (uint32_t)costAt(30000), false);
          tq += costAt(15000);
          q.sent((uint32_t)costAt(15000));
          break;
        case FramePlanner::PREPARE:
          tq += costAt(30000);
          q.drew(st.sec, (uint32_t)costAt(30000), true);
          break;
        case FramePlanner::SEND: {
          tq += st.waitUs + costAt(15000);
          const int64_t err = tq + 20000 - (int64_t)q.preparedSec() * 1000000LL;  // on the glass less the start of its second
          if (q.preparedScheduled() && sec > 5) {
            sent++;
            if (err > 30000) late++;
            if (err > worst) worst = err;
            if (-err > worst) worst = -err;
          }
          q.sent((uint32_t)costAt(15000));
          break;
        }
        default:
          tq += 4000;
          break;
      }
    }
    printf("  %d clock changes, %d of them with a frame waiting: %d of %d frames late, the worst %.1f ms off\n", switches, whilePrepared,
           late, sent, worst / 1000.0);
    CHECK(sent > 280 && switches > 70 && whilePrepared > switches * 9 / 10);
    CHECK(late == 0 && worst <= 5000);
  }
}

// ---------------------------------------------------------------------------
// Light sleep (light_sleep.h)
// ---------------------------------------------------------------------------

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 7) {}
  double uni() {
    s = s * 6364136223846793005ull + 1442695040888963407ull;
    return (double)(s >> 11) / 9007199254740992.0;
  }
};

void splitUs(int64_t us, time_t *sec, int32_t *usec) {
  *sec = (time_t)(us / 1000000);
  *usec = (int32_t)(us % 1000000);
}

// FramePlanner::idleUs(): for as long as it says, next() asks for nothing but naps, and once that time is up it asks
// for a step.  The planner is driven the way the sketch drives it, at `mhz` (a frame costs 80 / mhz times as much),
// and probed at every turn of the loop.  Returns the number of probes where `idle` (the planner's own, or a wrong one
// that shows this can fail) is not borne out.  `latencyUs`: the planner's time to the glass (display_power.h); with
// `switchHalfway` it changes to the other mode's figure half-way through, as a switch of display_power would.
template <class Idle>
int idleMisses(Idle idle, int mhz, uint64_t seed, int *probesOut = nullptr, int32_t latencyUs = 20000, bool switchHalfway = false) {
  FramePlanner p(latencyUs);
  if (mhz != 80) p.clockChanged(80, (uint32_t)mhz);
  Rng r(seed);
  const double f = 80.0 / mhz;
  int64_t t = 1700000000LL * 1000000LL + 345678;
  int misses = 0, probes = 0;
  auto probe = [&](int64_t at, bool wantStep) {
    time_t s;
    int32_t u;
    splitUs(at, &s, &u);
    FramePlanner q = p;  // (next() is asked of a copy: the probe must not change the plan)
    const FramePlanner::Action a = q.next(s, u, false).action;
    const bool step = a != FramePlanner::IDLE && a != FramePlanner::SLEEP;
    probes++;
    if (step != wantStep) misses++;
  };
  for (int turn = 0; turn < 6000; turn++) {
    if (switchHalfway && turn == 3000) p.setLatencyUs(p.latencyUs() == 20000 ? dispower::latencyUs(true, 20000) : 20000);
    time_t sec;
    int32_t usec;
    splitUs(t, &sec, &usec);
    const int32_t id = idle(p, sec, usec);
    if (id > 0) {
      for (int32_t d : {0, id / 3, id / 2, id - 1}) probe(t + d, false);  // naps only, until then ...
    }
    probe(t + (id > 0 ? id : 0), true);  // ... and a step once it is up
    const FramePlanner::Step st = p.next(sec, usec, false);
    auto draw = [&](time_t s, bool scheduled) {
      const int64_t c = (int64_t)(30000 * f * (0.9 + 0.2 * r.uni()));
      t += c;
      p.drew(s, (uint32_t)c, scheduled);
    };
    auto send = [&]() {
      const int64_t c = (int64_t)(15000 * f * (0.9 + 0.2 * r.uni()));
      t += c;
      p.sent((uint32_t)c);
    };
    switch (st.action) {
      case FramePlanner::DRAW_NOW:
        draw(st.sec, false);
        send();
        break;
      case FramePlanner::PREPARE:
        draw(st.sec, true);
        break;
      case FramePlanner::SEND:
        t += st.waitUs;
        send();
        break;
      default:  // a nap of 1 to 4 ms, or now and then a sleep of up to the whole idle time (as light sleep does)
        t += r.uni() < 0.2 && id > 2000 ? (int64_t)(id * r.uni()) : 1000 + (int64_t)(3000 * r.uni());
        break;
    }
  }
  if (probesOut) *probesOut = probes;
  return misses;
}

void testFramePlanIdle() {
  section("frame plan: how long the loop may sleep");
  auto real = [](const FramePlanner &p, time_t s, int32_t u) { return p.idleUs(s, u); };
  for (int mhz : {80, 40, 20}) {
    int probes = 0;
    const int misses = idleMisses(real, mhz, 11 + (uint64_t)mhz, &probes);
    if (misses) printf("  %d MHz: %d of %d probes miss\n", mhz, misses, probes);
    CHECK(misses == 0 && probes > 20000);
  }
  for (bool halfway : {false, true}) {  // display_power = low: 43 ms more to the glass, from the start or from half-way
    int probes = 0;
    const int misses = idleMisses(real, 20, 5, &probes, dispower::latencyUs(true, 20000), halfway);
    if (misses) printf("  display_power = low%s: %d of %d probes miss\n", halfway ? ", switched half-way" : "", misses, probes);
    CHECK(misses == 0 && probes > 20000);
  }
  {  // ... and at 80 MHz, where the draw is then over before the tick, so the frame waits for the second after next
    int probes = 0;
    const int misses = idleMisses(real, 80, 6, &probes, dispower::latencyUs(true, 20000));
    if (misses) printf("  display_power = low, 80 MHz: %d of %d probes miss\n", misses, probes);
    CHECK(misses == 0 && probes > 20000);
  }
  // the probes can fail: four ways of getting it wrong, each caught.  (Up to 1.12 the planner waited before it drew the
  // next second, and two of these were about that wait; the next second is drawn at once now, so they are about the
  // wait before the send.)
  auto noSpin = [](const FramePlanner &p, time_t s, int32_t u) {  // the send that is spun counted as idle time
    const int32_t id = p.idleUs(s, u);
    return p.prepared() && id > 0 ? id + FramePlanner::kSpinUs : id;
  };
  auto waitToDraw = [](const FramePlanner &p, time_t s, int32_t u) {  // the wait before the draw that there was up to 1.12
    if (!p.prepared() && (p.lastSentSec() == s || p.lastSentSec() == s + 1)) {
      const int32_t lead = (int32_t)(p.sendCostUs() + p.drawCostUs()) + p.latencyUs() + 60000;
      const int32_t until = (p.lastSentSec() == s + 1 ? 1000000 : 0) + 1000000 - u - lead;
      return until > 0 ? until : 0;
    }
    return p.idleUs(s, u);
  };
  auto comingOnly = [](const FramePlanner &p, time_t s, int32_t u) {  // a frame two seconds ahead taken for the coming one
    if (p.prepared() && p.preparedSec() == s + 2) {
      const int32_t w = 1000000 - u - (int32_t)p.sendCostUs() - p.latencyUs();
      return w > FramePlanner::kSpinUs ? w - FramePlanner::kSpinUs : 0;
    }
    return p.idleUs(s, u);
  };
  auto fixedCost = [](const FramePlanner &p, time_t s, int32_t u) {  // the send cost of 80 MHz, whatever the clock
    if (p.prepared() && p.preparedSec() >= s && p.preparedSec() <= s + 2) {
      const int32_t w = (int32_t)(p.preparedSec() - s - 1) * 1000000 + 1000000 - u - 15000 - p.latencyUs();
      return w > FramePlanner::kSpinUs ? w - FramePlanner::kSpinUs : 0;
    }
    return p.idleUs(s, u);
  };
  CHECK(idleMisses(noSpin, 80, 3) > 100);
  CHECK(idleMisses(waitToDraw, 20, 3) > 100);
  CHECK(idleMisses(comingOnly, 80, 3, nullptr, dispower::latencyUs(true, 20000)) > 100);  // (there the draw is over before the tick)
  CHECK(idleMisses(fixedCost, 20, 3) > 100);
  // (and the clock going backwards: a frame prepared for a second that is far ahead is due for a fresh start now)
  FramePlanner b(20000);
  b.drew(2000, 30000, true);
  CHECK(b.idleUs(1000, 500000) == 0);
}

void testLightSleepDecide() {
  section("light sleep: when, and for how long");
  using namespace lightsleep;
  using lightsleep::Inputs;  // (not radioplan's)
  Inputs base;
  base.enabled = true;
  base.frameInUs = 700000;
  {
    const Decision d = decide(base);  // nothing in the way: up to the next frame step, less the guard
    CHECK(d.sleep && d.why == ASLEEP && d.us == 700000 - kGuardUs - 700000 / 128);
  }
  // what keeps it awake, one at a time, and the reason it gives
  struct Case {
    const char *what;
    Inputs in;
    Why why;
  };
  std::vector<Case> cases;
  auto with = [&](const char *what, Why why, void (*set)(Inputs &)) {
    Inputs in = base;
    set(in);
    cases.push_back({what, in, why});
  };
  with("off", OFF, [](Inputs &i) { i.enabled = false; });
  with("radio", RADIO, [](Inputs &i) { i.radio = true; });
  with("console", CONSOLE, [](Inputs &i) { i.console = true; });
  with("button down", BUTTON, [](Inputs &i) { i.button = true; });
  with("poke window", BUTTON, [](Inputs &i) { i.poked = true; });
  with("busy", BUSY, [](Inputs &i) { i.busy = true; });
  with("refused a moment ago", REFUSED, [](Inputs &i) { i.backOff = true; });
  with("work that can run now", WORK, [](Inputs &i) { i.workInUs = 0; });
  with("frame step now", SOON, [](Inputs &i) { i.frameInUs = 0; });
  with("frame step close", SOON, [](Inputs &i) { i.frameInUs = kMinSleepUs + kGuardUs; });
  with("banner due now", SOON, [](Inputs &i) { i.otherInUs = 0; });
  with("banner due soon", SOON, [](Inputs &i) { i.otherInUs = 10000; });
  with("work's part of the second close", SOON, [](Inputs &i) { i.workInUs = 15000; });
  for (const Case &c : cases) {
    const Decision d = decide(c.in);
    if (d.sleep || d.why != c.why) printf("  %s: sleep %d, why %d (wanted %d)\n", c.what, d.sleep, (int)d.why, (int)c.why);
    CHECK(!d.sleep && d.why == c.why);
  }
  // when there are several, the first of: off, radio, console, button, busy, refused, work, soon
  {
    Inputs all = base;
    all.radio = all.console = all.button = all.poked = all.busy = all.backOff = true;
    all.workInUs = 0;
    CHECK(decide(all).why == RADIO);
    all.radio = false;
    CHECK(decide(all).why == CONSOLE);
    all.console = false;
    CHECK(decide(all).why == BUTTON);
    all.button = all.poked = false;
    CHECK(decide(all).why == BUSY);
    all.busy = false;
    CHECK(decide(all).why == REFUSED);
    all.backOff = false;
    CHECK(decide(all).why == WORK);
    all.enabled = false;
    all.radio = true;
    CHECK(decide(all).why == OFF);
  }
  // the earliest of the three times counts; the guard is taken off it; never longer than kMaxSleepUs
  {
    Inputs in = base;
    in.workInUs = 400000;
    CHECK(decide(in).us == 400000 - kGuardUs - 400000 / 128);
    in.otherInUs = 300000;
    CHECK(decide(in).us == 300000 - kGuardUs - 300000 / 128);
    in.frameInUs = 250000;
    CHECK(decide(in).us == 250000 - kGuardUs - 250000 / 128);
    Inputs far = base;
    far.frameInUs = 1900000000;  // (the planner never says more than about two seconds, but)
    CHECK(decide(far).us == kMaxSleepUs - kGuardUs - kMaxSleepUs / 128);
    Inputs none = base;  // no work due, no banner: those two do not shorten it
    none.workInUs = kNever;
    none.otherInUs = kNever;
    CHECK(decide(none).us == decide(base).us);
  }
  // the shortest sleep there is, to the microsecond
  {
    Inputs in = base;
    int32_t first = -1;
    for (int32_t ahead = 0; ahead < 60000 && first < 0; ahead++) {
      in.frameInUs = ahead;
      const Decision d = decide(in);
      if (d.sleep) first = ahead;
    }
    in.frameInUs = first;
    CHECK(first > kMinSleepUs + kGuardUs && decide(in).us == kMinSleepUs);
    in.frameInUs = first - 1;
    CHECK(!decide(in).sleep);
  }
  // wifi off and wifi_mode = sync differ only in whether the radio is ever on; always mode never sleeps
  {
    Inputs wifiOff = base, syncBetween = base, syncSession = base;
    syncSession.radio = true;
    CHECK(decide(wifiOff).sleep && decide(syncBetween).sleep && !decide(syncSession).sleep);
  }
  // the row of the Power and settings page: whole for every reason and every share, and saying why when it is worth it
  {
    char row[kRow];
    int whole = 0, cut = 0;
    for (int why = ASLEEP; why <= REFUSED; why++) {
      for (int p : {0, 1, 9, 10, 785, 999, 1000, 1001, 65535}) {
        formatRow((uint16_t)p, (Why)why, row);
        const size_t n = strlen(row);
        n + 1 < kRow && row[n - 1] != '(' ? whole++ : cut++;
        CHECK(strstr(row, "% of the time") != nullptr);
      }
    }
    CHECK(cut == 0 && whole == 9 * 9);
    formatRow(785, ASLEEP, row);
    CHECK_STR(row, "78.5 % of the time");
    formatRow(785, SOON, row);
    CHECK_STR(row, "78.5 % of the time");
    formatRow(785, WORK, row);
    CHECK_STR(row, "78.5 % of the time");
    formatRow(0, RADIO, row);
    CHECK_STR(row, "0.0 % of the time (now: radio on)");
    formatRow(1000, BUTTON, row);
    CHECK_STR(row, "100.0 % of the time (now: button pressed)");
    formatRow(5000, CONSOLE, row);  // (more than all of it cannot be)
    CHECK_STR(row, "100.0 % of the time (now: console on)");
    formatRow(312, REFUSED, row);
    CHECK_STR(row, "31.2 % of the time (now: SDK refused)");
  }
}

// A stand-in for the chip: it writes down the calls of a nap, in order, as letters.
struct FakeChip {
  std::string calls;
  bool down = false;          // a button is down when asked
  int err = 0;                // what sleep() returns
  bool byButton = false;      // ... and what ended it
  bool armed = false;
  int64_t mono = 5000000, wall = 1700000000LL * 1000000LL;
  int64_t sdkJumpUs = 0;      // what the SDK adds to the time of day at the next wake-up (the RC count of the time awake)
  int64_t sleptUs = 0;
  void wakeOnButtons(bool on) {
    calls += on ? "A" : "a";
    armed = on;
  }
  bool buttonDown() {
    calls += "D";
    return down;
  }
  int sleep(int32_t us) {
    calls += armed ? "S" : "S!";  // (a sleep with the buttons not armed would be one that only the timer can end)
    if (err) return err;
    if (byButton) us /= 2;  // (a press ends it half way)
    mono += us;
    wall += us + sdkJumpUs;
    sdkJumpUs = 0;
    sleptUs += us;
    return 0;
  }
  bool wokeByButton() {
    calls += "B";
    return byButton;
  }
  int64_t monoUs() { return mono; }
  int64_t wallUs() { return wall; }
  void shiftWallUs(int64_t d) {
    calls += "W";
    wall += d;
  }
  void awake(int64_t us) {  // time passes with the chip awake
    mono += us;
    wall += us;
  }
};

void testNapSteps() {
  section("light sleep: the steps of a nap");
  using namespace lightsleep;
  {  // the usual one: buttons armed, a look at them, the sleep, what ended it, the buttons back
    FakeChip c;
    WallFix fix;
    const Nap n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADSBa");
    CHECK(n.slept && !n.byButton && !n.pressed && n.err == 0 && n.us == 600000 && n.wallFixUs == 0 && c.sleptUs == 600000);
  }
  {  // ended by a button
    FakeChip c;
    c.byButton = true;
    WallFix fix;
    const Nap n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADSBa" && n.slept && n.byButton && n.us == 300000);
  }
  {  // a button went down just before: no sleep, the buttons as they were, nothing else touched
    FakeChip c;
    c.down = true;
    WallFix fix;
    const int64_t wall = c.wall, mono = c.mono;
    const Nap n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADa" && !n.slept && n.pressed && n.err == 0 && n.us == 0 && c.wall == wall && c.mono == mono && !c.armed);
  }
  {  // the SDK refuses: the buttons back, the time of day left alone (the SDK does not touch it then either)
    FakeChip c;
    c.err = 0x103;
    c.sdkJumpUs = 50000;
    WallFix fix;
    const Nap n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADSa" && !n.slept && n.err == 0x103 && !c.armed && fix.pendingUs() == 0);
  }
  {  // the time of day: what the SDK added beyond the time asleep is taken off again once it adds up to kWallStepUs
    FakeChip c;
    WallFix fix;
    c.sdkJumpUs = 1500;
    Nap n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADSBa" && n.wallFixUs == 0 && fix.pendingUs() == -1500);
    c.calls.clear();
    c.sdkJumpUs = 700;
    n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADSBaW" && n.wallFixUs == -2200 && fix.pendingUs() == 0 && fix.fixes() == 1 && fix.largestUs() == 2200);
    CHECK(c.wall - c.mono == 1700000000LL * 1000000LL - 5000000);  // back in step with the timer
    c.calls.clear();
    c.sdkJumpUs = -40000;  // a big one the other way (an hour awake, with the RC clock slow): at once
    n = napOnce(c, fix, 600000);
    CHECK(c.calls == "ADSBaW" && n.wallFixUs == 40000 && fix.largestUs() == 40000 && c.wall - c.mono == 1700000000LL * 1000000LL - 5000000);
  }
  // the share of the time asleep
  {
    Stats st;
    CHECK(st.permille(1000000) == 0);
    st.begin(1000000);
    st.begin(5000000);  // (only the first counts)
    Nap a;
    a.slept = true;
    a.us = 785000;
    st.add(a);
    CHECK(st.permille(2000000) == 785 && st.naps() == 1);
    Nap b = a;
    b.byButton = true;
    b.us = 2000000;
    st.add(b);
    CHECK(st.permille(4000000) == 928 && st.buttonWakes() == 1);  // 2.785 of 3 s
    Nap refused;
    refused.err = 0x103;
    st.add(refused);
    Nap pressed;
    pressed.pressed = true;
    st.add(pressed);
    CHECK(st.refused() == 1 && st.naps() == 2);
    CHECK(st.permille(1000000) == 0 && st.permille(3000000) == 1000);  // (never more than all of it, nor less than none)
  }
}

// What the time of day comes to over a day of naps, with the SDK taking it afresh from the RC count at every wake-up
// (the RC oscillator `ppm` off over the time awake) and a fix of the kind given.  Returns the worst error against the
// true time, in microseconds.  Sleeps are 700 ms, awake 300 ms, and twice an hour the clock stays awake for minutes.
template <class Fix>
int64_t wallError(Fix &fix, double ppm, uint64_t seed) {
  FakeChip c;
  Rng r(seed);
  const int64_t offset = c.wall - c.mono;  // the true time of day is the timer plus this
  int64_t worst = 0, awakeUs = 0;
  for (int i = 0; i < 86400; i++) {
    int64_t awake = 250000 + (int64_t)(100000 * r.uni());
    if (i % 1800 == 900) awake += (int64_t)(600 * 1000000LL * r.uni());  // ten minutes of music, a session, a long press
    c.awake(awake);
    awakeUs += awake;
    c.sdkJumpUs = (int64_t)(awakeUs * ppm / 1e6);  // the RC's count of the time awake since the last wake-up, off by ppm
    awakeUs = 0;
    lightsleep::napOnce(c, fix, 700000);
    const int64_t err = (c.wall - c.mono) - offset;
    if ((err < 0 ? -err : err) > worst) worst = err < 0 ? -err : err;
  }
  return worst;
}

// Two wrong fixes, to show that the check below can fail: none at all (the SDK alone), and one that drops what is too
// small to be worth a correction instead of keeping it until it adds up.
struct NoFix {
  int64_t add(int64_t, int64_t, int64_t, int64_t) { return 0; }
};
struct DropSmall {
  int64_t add(int64_t w0, int64_t m0, int64_t w1, int64_t m1) {
    const int64_t d = (w0 + (m1 - m0)) - w1;
    return d >= lightsleep::kWallStepUs || d <= -lightsleep::kWallStepUs ? d : 0;
  }
};

void testWallFix() {
  section("light sleep: the time of day");
  for (double ppm : {300.0, -300.0, 5000.0, -50.0}) {
    lightsleep::WallFix fix;
    const int64_t worst = wallError(fix, ppm, 5);
    printf("  RC %+.0f ppm over the time awake: worst %.2f ms off in a day with the fix, %u corrections\n", ppm, worst / 1000.0,
           (unsigned)fix.fixes());
    CHECK(worst <= lightsleep::kWallStepUs);
    NoFix none;
    DropSmall drop;
    const int64_t without = wallError(none, ppm, 5), dropped = wallError(drop, ppm, 5);
    if (ppm == 300.0) printf("  ... without it %.0f ms, dropping the small ones %.0f ms\n", without / 1000.0, dropped / 1000.0);
    CHECK(without > 10 * lightsleep::kWallStepUs);  // (the check can fail)
    if (ppm != 5000.0) CHECK(dropped > 10 * lightsleep::kWallStepUs);  // (5000 ppm: every one is big enough, so dropping costs little)
  }
}

// The schedule the frame planner had up to 1.12, for the negative controls: the next second was drawn only when its
// draw had to begin (the send, the draw and 60 ms to spare before its tick), and nothing was quiet while a frame
// waited, so a second had two sleeps, one until the draw and a short one between the draw and the send.  (FramePlanner
// with that wait put back in front of it.)
struct DrawLate : FramePlanner {
  explicit DrawLate(int32_t latencyUs) : FramePlanner(latencyUs) {}
  // How long until the next second is to be drawn, while it waits for that (0: now); -1 when it does not
  int32_t untilDrawUs(time_t sec, int32_t usec) const {
    if (prepared() || (lastSentSec() != sec && lastSentSec() != sec + 1)) return -1;
    const int32_t lead = (int32_t)(sendCostUs() + drawCostUs()) + latencyUs() + 60000;
    const int32_t until = (lastSentSec() == sec + 1 ? 1000000 : 0) + 1000000 - usec - lead;
    return until > 0 ? until : 0;
  }
  Step next(time_t sec, int32_t usec, bool force) {
    if (!force && untilDrawUs(sec, usec) > 0) return Step();  // IDLE
    return FramePlanner::next(sec, usec, force);
  }
  int32_t idleUs(time_t sec, int32_t usec) const {
    const int32_t until = untilDrawUs(sec, usec);
    return until >= 0 ? until : FramePlanner::idleUs(sec, usec);
  }
  bool quiet(int32_t usec) const { return !prepared() && 1000000 - usec > kQuietUs; }
};

// The UI loop with light sleep, as the sketch runs it (11_Clock_Dashboard.ino, loop() and lightSleepStep()): the frame
// planner, the slow work in its part of the second, a line of the power log every ten minutes, a banner, button
// presses, radio sessions with the clock at 80 MHz, and the idle clock in between.  A frame costs 30 ms to draw and
// 15 to send at 80 MHz, 80 / MHz times as much below.  A light sleep takes 5 ms of the chip awake on the way (the SDK
// calibrates the RC clock for some 4 ms, and takes that off the time asked for), and wakes up to 1 ms early or 2 ms
// late, give or take 0.2 % of its length.  Stalls (a flash write, the radio stack: work of the other core) come at
// random moments while the chip is awake; while it sleeps the other core sleeps too.  `Plan` is the frame planner, or
// for the negative controls the schedule it had up to 1.12 (DrawLate, below).
struct SleepLoop {
  bool lightSleep = true;
  int idleMhz = 20;
  int32_t latencyUs = 20000;   // the planner's time to the glass (display_power.h)
  double stallEverySec = 0;    // stalls of 20 to 300 ms, about this often in the time awake (0: none)
  double slowDrawShare = 0;    // this share of the scheduled draws is held up by a stall of 20 to 300 ms (0: none)
  double pressEverySec = 600;  // a button press about this often
  // what came of it
  int frames = 0, late = 0, presses = 0, logLines = 0, sleeps = 0;
  int64_t worstLateUs = 0, worstBatteryGapUs = 0, worstLogLateUs = 0;
  int64_t worstBannerUs = 0;        // from the moment a banner should come down to the redraw, the worst ...
  int64_t worstBannerIdleUs = -1;   // ... of those that came down with no button pressed for a while (light sleep in play)
  int64_t worstPressUs = 0;         // from a press to the loop taking it, the worst of all
  int64_t worstPressAsleepUs = -1;  // ... of those that came while the chip slept (-1: none did)
  int64_t asleepUs = 0, workUs = 0, idleUs = 0, totalUs = 0;
  double chipMa = 0;  // the chip's average current by the datasheet: working, awake and idle, or in light sleep

  template <class Plan = FramePlanner>
  void run(int seconds, uint64_t seed) {
    Rng r(seed);
    Plan p(latencyUs);
    int mhz = 80;
    int64_t t = 1700000000LL * 1000000LL + 250000;
    const int64_t start = t, end = t + (int64_t)seconds * 1000000;
    int64_t nextStall = stallEverySec > 0 ? t + (int64_t)(stallEverySec * 1e6 * r.uni() * 2) : INT64_MAX;
    int64_t lastBattery = 0, lastSensor = 0, nextLog = t + 10 * 1000000LL, pokeUntil = 0, bannerUntil = t + 6000000;
    int64_t nextPress = t + 40 * 1000000LL;  // (the first one soon, so that every run has some)
    bool force = true, pressWokeIt = false;
    // ESP32-S3 datasheet v2.2, table 5-9 (radio off, peripheral clocks off): idle 22.0 / 13.2 mA and one core working
    // 28.4 / 16.2 mA at 80 / 40 MHz; 20 MHz is drawn out along the same line (Espressif gives nothing below 40)
    auto mA = [](int m, bool working) {
      const double at40 = working ? 16.2 : 13.2, perMhz = working ? (28.4 - 16.2) / 40 : (22.0 - 13.2) / 40;
      return at40 + (m - 40) * perMhz;
    };
    double charge = 0;  // mA x us
    enum Kind { WORKING, IDLE, ASLEEP };
    auto spend = [&](int64_t us, Kind kind) {
      if (us <= 0) return;
      // A stall freezes the loop (a flash write, the radio stack) wherever it happens while the chip is awake.  (Up to
      // 1.12 one that fell due in a sleep came at its start and made it end that much late, as if the time asleep were
      // awake: that put a stall at every sleep's start with a chance that grew with the sleep, which is not how it
      // goes on the chip, where only the few milliseconds of the way in are exposed, and those are spent WORKING.)
      while (kind != ASLEEP && t + us >= nextStall) {
        const int64_t s = (int64_t)((20 + 280 * r.uni()) * 1000);
        workUs += s;
        charge += (double)s * mA(mhz, true);
        t += s;
        nextStall = t + (int64_t)(stallEverySec * 1e6 * r.uni() * 2);
      }
      if (kind == ASLEEP && nextStall != INT64_MAX) nextStall += us;  // (no stalls while asleep)
      t += us;
      if (kind == WORKING) {
        workUs += us;
        charge += (double)us * mA(mhz, true);
      } else if (kind == IDLE) {
        idleUs += us;
        charge += (double)us * mA(mhz, false);
      } else {
        asleepUs += us;
        charge += (double)us * 0.24;
      }
    };
    auto cost = [&](double us80) { return (int64_t)(us80 * 80.0 / mhz * (0.9 + 0.2 * r.uni())); };
    auto radioOn = [&](int64_t at) { return ((at - start) / 1000000) % 900 < 6; };  // a 6 s session every 15 min
    while (t < end) {
      if (t >= nextPress) {  // a press (recorded by interrupt whenever it came): the loop takes it now
        const int64_t after = t - nextPress;
        if (after > worstPressUs) worstPressUs = after;
        if (pressWokeIt && after > worstPressAsleepUs) worstPressAsleepUs = after;
        pressWokeIt = false;
        presses++;
        pokeUntil = t + 8000000;
        force = true;
        bannerUntil = t + 2200000;
        nextPress = t + (int64_t)(pressEverySec * (0.5 + r.uni()) * 1e6);
      }
      if (bannerUntil != 0 && t >= bannerUntil) {  // the banner comes down: a redraw
        if (t - bannerUntil > worstBannerUs) worstBannerUs = t - bannerUntil;
        if (t >= pokeUntil && t - bannerUntil > worstBannerIdleUs) worstBannerIdleUs = t - bannerUntil;
        bannerUntil = 0;
        force = true;
      }
      // the CPU clock (clock_policy.h): up at once, down only between frames
      const int want = radioOn(t) || t < pokeUntil ? 80 : idleMhz;
      time_t sec;
      int32_t usec;
      splitUs(t, &sec, &usec);
      if (want != mhz && (want > mhz || p.quiet(usec))) {
        p.clockChanged((uint32_t)mhz, (uint32_t)want);
        mhz = want;
      }
      const FramePlanner::Step st = p.next(sec, usec, force);
      auto send = [&](bool scheduled) {
        const int64_t c = cost(15000);
        spend(c, WORKING);
        if (scheduled) {  // (recordLateness())
          frames++;
          const int64_t lateUs = (t - (int64_t)p.preparedSec() * 1000000) + p.latencyUs();
          if (lateUs > 30000) late++;
          if (lateUs > worstLateUs) worstLateUs = lateUs;
        }
        p.sent((uint32_t)c);
        force = false;
      };
      bool napTime = false;
      switch (st.action) {
        case FramePlanner::DRAW_NOW: {
          const int64_t c = cost(30000);
          spend(c, WORKING);
          p.drew(st.sec, (uint32_t)c, false);
          send(false);
          break;
        }
        case FramePlanner::PREPARE: {
          int64_t c = cost(30000);
          if (slowDrawShare > 0 && r.uni() < slowDrawShare) c += (int64_t)((20 + 280 * r.uni()) * 1000);  // (a stall in the draw)
          spend(c, WORKING);
          p.drew(st.sec, (uint32_t)c, true);
          break;
        }
        case FramePlanner::SEND:
          spend(st.waitUs, WORKING);  // (spun)
          send(p.preparedScheduled());
          break;
        case FramePlanner::SLEEP:
          napTime = true;
          break;
        default:
          if (p.quiet(usec)) {  // readSlowSensors()
            if (lastBattery == 0 || t - lastBattery >= 5000000) {
              if (lastBattery && t - lastBattery > worstBatteryGapUs) worstBatteryGapUs = t - lastBattery;
              spend(cost(3000), WORKING);
              lastBattery = t;
            }
            if (lastSensor == 0 || t - lastSensor >= 10000000) {
              spend(15000, IDLE);  // (the SHTC3 measures while the loop waits)
              spend(cost(2000), WORKING);
              lastSensor = t;
            }
          }
          splitUs(t, &sec, &usec);
          if (t >= nextLog && usec <= 200000) {  // servicePowerLog()
            if (t - nextLog > worstLogLateUs) worstLogLateUs = t - nextLog;
            spend(150000, WORKING);
            logLines++;
            nextLog += 600 * 1000000LL;
          }
          napTime = true;
          break;
      }
      if (!napTime) continue;
      bool slept = false;
      if (lightSleep) {  // lightSleepStep(), and slowWorkInUs()
        splitUs(t, &sec, &usec);
        lightsleep::Inputs in;
        in.enabled = true;
        in.radio = radioOn(t);
        in.button = t >= nextPress;  // a change in the record that the loop has not taken yet
        in.poked = t < pokeUntil;
        in.busy = force;
        in.frameInUs = p.idleUs(sec, usec);
        const int32_t toTick = 1000000 - usec;
        if (lastBattery == 0 || t - lastBattery >= 5000000 || lastSensor == 0 || t - lastSensor >= 10000000) {
          in.workInUs = p.quiet(usec) ? 0 : toTick;
        }
        if (t >= nextLog) {
          const int32_t at = usec <= 200000 ? 0 : toTick;
          if (at < in.workInUs) in.workInUs = at;
        }
        if (bannerUntil) in.otherInUs = bannerUntil <= t ? 0 : (int32_t)std::min<int64_t>(bannerUntil - t, lightsleep::kMaxSleepUs);
        const lightsleep::Decision d = lightsleep::decide(in);
        if (d.sleep) {
          sleeps++;
          spend(5000, WORKING);  // the calibration, the way in and out
          int64_t us = d.us - 5000 + (int64_t)(d.us * 0.002 * (2 * r.uni() - 1)) + (int64_t)(3000 * r.uni()) - 1000;
          if (t + us > nextPress) {  // KEY ends it: awake a millisecond after the press
            us = nextPress + 1000 - t;
            pressWokeIt = true;
          }
          spend(us, ASLEEP);
          slept = true;
        }
      }
      if (!slept) spend(st.action == FramePlanner::SLEEP && st.waitUs <= 12000 ? 1000 + (int64_t)(r.uni() * 1000) : 4000 + (int64_t)(r.uni() * 1000), IDLE);
    }
    totalUs = t - start;
    chipMa = charge / (double)totalUs;
  }
};

void testLightSleepLoop() {
  section("light sleep: a simulated UI loop");
  const int kSeconds = 3 * 3600;
  for (int mhz : {20, 40, 80}) {
    SleepLoop off, on, late;  // (late: the schedule up to 1.12, DrawLate)
    off.lightSleep = false;
    off.idleMhz = on.idleMhz = late.idleMhz = mhz;
    off.run(kSeconds, 21);
    on.run(kSeconds, 21);
    late.run<DrawLate>(kSeconds, 21);
    const double perSec = on.sleeps / (double)kSeconds, perSecLate = late.sleeps / (double)kSeconds;
    printf("  idle clock %d MHz: asleep %.1f %% of the time, working %.1f %%, awake and idle %.1f %%; the chip %.1f mA without, %.1f mA with"
           " (datasheet model)\n",
           mhz, 100.0 * on.asleepUs / on.totalUs, 100.0 * on.workUs / on.totalUs, 100.0 * on.idleUs / on.totalUs, off.chipMa, on.chipMa);
    printf("    %.2f sleeps a second (%.2f with the draw left until just before the send, as up to 1.12, and %.2f mA)\n", perSec, perSecLate,
           late.chipMa);
    printf("    frames late: %d of %d with (worst %.1f ms), %d of %d without (worst %.1f ms); battery read %.1f s apart at worst; log lines"
           " %.2f s late at worst; presses taken %.1f ms after at worst, %.1f ms when it slept; banner down %.1f ms late at worst, %.1f ms"
           " with no button pressed\n",
           on.late, on.frames, on.worstLateUs / 1000.0, off.late, off.frames, off.worstLateUs / 1000.0, on.worstBatteryGapUs / 1e6,
           on.worstLogLateUs / 1e6, on.worstPressUs / 1000.0, on.worstPressAsleepUs / 1000.0, on.worstBannerUs / 1000.0,
           on.worstBannerIdleUs / 1000.0);
    CHECK(on.frames > kSeconds * 9 / 10 && off.frames > kSeconds * 9 / 10);
    CHECK(on.late <= off.late);
    CHECK(on.worstLateUs <= off.worstLateUs + 5000);
    CHECK(off.asleepUs == 0 && off.sleeps == 0);
    // one sleep a second after 1.12 (the send and the draw are one stretch awake), where there were two (up to 1.12 this
    // checked for more than one a second); the time with the radio on or a button pressed has none
    CHECK(perSec > 0.95 && perSec < 1.05);
    CHECK(perSecLate > 1.8);  // (the old schedule: the check can fail)
    CHECK(on.chipMa < late.chipMa);
    CHECK(on.asleepUs > (mhz == 20 ? 0.70 : 0.85) * on.totalUs);  // (the model's own figures; the board's are unknown)
    CHECK(on.worstBatteryGapUs < 7000000);  // every 5 s, and a second or two late at worst
    CHECK(on.logLines >= kSeconds / 600 - 1 && on.worstLogLateUs < 1000000);  // (each line within a second of being due)
    CHECK(on.presses > 10 && on.worstPressAsleepUs >= 0 && on.worstPressAsleepUs < 5000);  // KEY ends a sleep at once ...
    CHECK(on.worstPressUs < 45000 * 1.1 * 80 / mhz + 10000);  // ... and otherwise a press waits for a frame at most
    CHECK(on.worstBannerIdleUs >= 0 && on.worstBannerIdleUs < 6000);  // a banner comes down on time (the sleep ends before)
    CHECK(on.chipMa < off.chipMa * 0.5);
  }
  {  // stalls of 20 to 300 ms every 8 s awake on average: no more late frames than without light sleep, and a press
     // every half a minute: taken as soon as without it
    SleepLoop off, on;
    off.lightSleep = false;
    off.stallEverySec = on.stallEverySec = 8;
    off.pressEverySec = on.pressEverySec = 30;
    off.run(3600, 9);
    on.run(3600, 9);
    printf("  stalls every 8 s awake, a press every 30 s: %d of %d frames late with light sleep (worst %.0f ms), %d of %d without (worst %.0f ms);"
           " presses taken %.1f ms after at worst with, %.1f ms without; asleep %.1f %%\n",
           on.late, on.frames, on.worstLateUs / 1000.0, off.late, off.frames, off.worstLateUs / 1000.0, on.worstPressUs / 1000.0,
           off.worstPressUs / 1000.0, 100.0 * on.asleepUs / on.totalUs);
    CHECK(on.late <= off.late + 2);
    CHECK(on.worstPressUs <= off.worstPressUs + 5000);
    // and against the schedule up to 1.12, over three hours: a stall in the draw no longer makes its second late, only
    // one in the send, or in the few milliseconds awake before it
    int lateNew = 0, lateOld = 0, framesNew = 0;
    for (uint64_t seed : {9, 10, 11}) {
      SleepLoop a, b;
      a.stallEverySec = b.stallEverySec = 8;
      a.pressEverySec = b.pressEverySec = 30;
      a.run(3600, seed);
      b.run<DrawLate>(3600, seed);
      lateNew += a.late;
      lateOld += b.late;
      framesNew += a.frames;
    }
    printf("    over three hours: %d of %d frames late, %d with the draw left until just before the send (up to 1.12)\n", lateNew, framesNew,
           lateOld);
    CHECK(lateNew < lateOld);
  }
  {  // one draw in ten held up by a stall of 20 to 300 ms, at the idle clock of 20 MHz and with light sleep, over three hours:
     // drawn a second ahead, no second is late; drawn just in time (up to 1.12), many are
    SleepLoop on, late;
    on.slowDrawShare = late.slowDrawShare = 0.1;
    on.run(kSeconds, 33);
    late.run<DrawLate>(kSeconds, 33);
    printf("  one draw in ten held up by 20 to 300 ms: %d of %d frames late (worst %.1f ms), %d of %d drawn just in time (worst %.0f ms);"
           " battery read %.1f s apart at worst, log lines %.2f s late at worst\n",
           on.late, on.frames, on.worstLateUs / 1000.0, late.late, late.frames, late.worstLateUs / 1000.0, on.worstBatteryGapUs / 1e6,
           on.worstLogLateUs / 1e6);
    CHECK(on.frames > kSeconds * 9 / 10);
    CHECK(on.late == 0 && on.worstLateUs <= 30000);
    CHECK(late.late > 100);  // (the check can fail)
    // the slow work still runs: the battery every 5 s, and a line of the log (which starts only early in a second, after
    // the draw after 1.12: one held up past that waits a second)
    CHECK(on.worstBatteryGapUs <= 10000000 && on.logLines >= kSeconds / 600 - 1 && on.worstLogLateUs < 2000000);
  }
  {  // display_power = low: every frame goes out 43 ms earlier in its second (the panel takes that much longer); no more
     // late frames for it, and the chip sleeps about as much
    SleepLoop hpm, lpm;
    lpm.latencyUs = dispower::latencyUs(true, 20000);
    hpm.run(kSeconds, 21);
    lpm.run(kSeconds, 21);
    printf("  display_power = low, idle clock 20 MHz: asleep %.1f %% (%.1f %% with normal), frames late %d of %d (%d of %d), worst %.1f ms"
           " (%.1f ms)\n",
           100.0 * lpm.asleepUs / lpm.totalUs, 100.0 * hpm.asleepUs / hpm.totalUs, lpm.late, lpm.frames, hpm.late, hpm.frames,
           lpm.worstLateUs / 1000.0, hpm.worstLateUs / 1000.0);
    CHECK(lpm.frames > kSeconds * 9 / 10);
    CHECK(lpm.late <= hpm.late && lpm.worstLateUs <= hpm.worstLateUs + 5000);
    CHECK(lpm.asleepUs > 0.70 * lpm.totalUs && lpm.asleepUs > hpm.asleepUs - 0.02 * hpm.totalUs);
  }
}

// ---------------------------------------------------------------------------
// The radio and the CPU clock (1.13)
// ---------------------------------------------------------------------------

void testClockAnswer() {
  section("radio and clock: when the UI loop may say the clock is up");
  using namespace clockpolicy;
  CHECK(kRadioMinMhz == 80);
  CHECK(radioMayStart(true, 80, 80, 0) && radioMayStart(true, 240, 240, 0) && radioMayStart(true, 160, 160, 0));
  CHECK(!radioMayStart(false, 80, 80, 0));    // nothing asked (or asked after the loop looked): no answer
  CHECK(!radioMayStart(true, 20, 80, 0));     // still to be raised
  CHECK(!radioMayStart(true, 40, 40, 0));     // where it stays, but WiFi does not work there
  CHECK(!radioMayStart(true, 80, 240, 0));    // up enough, but about to change under the radio
  CHECK(radioMayStart(true, 80, 240, 240));   // ... unless the core refused 240: then 80 is where it stays
  CHECK(!radioMayStart(true, 20, 80, 80));    // a refused 80 MHz is no answer
  CHECK(!radioMayStart(true, 20, 20, 20));
  // what target() gives with the radio asking is always an answer (when the core takes it), whatever the settings
  for (int work : {80, 160, 240})
    for (int idle : {0, 20, 40, 80, 160}) {
      Needs n;
      n.radio = true;
      for (int poked = 0; poked < 2; poked++) {
        n.poked = poked != 0;
        const int want = target(n, work, idle);
        CHECK(want == work && radioMayStart(true, want, want, 0));
        Needs off = n;
        off.radio = false;
        CHECK(!radioMayStart(false, target(off, work, idle), target(off, work, idle), 0));
      }
    }
}

void testClockWait() {
  section("radio and clock: the network task's wait");
  using radioplan::ClockWait;
  ClockWait w;
  CHECK(w.waits() == 0 && w.longestMs() == 0 && w.putOff() == 0);
  // at the idle clock: it waits until the loop has answered and the clock is up, then goes
  w.begin(1000, 20);
  CHECK(w.step(1000, false, 20, false) == ClockWait::WAIT);
  CHECK(w.step(1060, false, 80, false) == ClockWait::WAIT);  // the clock is up, but the loop has not said so: not yet
  CHECK(w.step(1120, true, 80, false) == ClockWait::GO);
  CHECK(w.waits() == 1 && w.longestMs() == 120 && w.putOff() == 0);
  // already fast (a button was pressed): it still waits for the answer, and is not counted as a wait for the clock
  w.begin(5000, 80);
  CHECK(w.step(5000, false, 80, false) == ClockWait::WAIT);
  CHECK(w.step(5010, true, 80, false) == ClockWait::GO);
  CHECK(w.waits() == 1 && w.longestMs() == 120);
  // an answer while the clock reads below 80 MHz is not taken (belt and braces)
  w.begin(9000, 20);
  CHECK(w.step(9100, true, 20, false) == ClockWait::WAIT);
  CHECK(w.step(9200, true, 80, false) == ClockWait::GO && w.longestMs() == 200 && w.waits() == 2);
  // no answer: it waits kClockWaitMs and no longer, and does not go: the start is put off
  w.begin(20000, 20);
  for (uint32_t t = 20000; t < 20000 + radioplan::kClockWaitMs; t += 10) {
    if (w.step(t, false, 20, false) != ClockWait::WAIT) {
      printf("  gave up after %u ms\n", (unsigned)(t - 20000));
      CHECK(false);
      break;
    }
  }
  CHECK(w.step(20000 + radioplan::kClockWaitMs, false, 20, false) == ClockWait::PUT_OFF);
  CHECK(w.putOff() == 1 && w.waits() == 3 && w.longestMs() == radioplan::kClockWaitMs);
  w.begin(30000, 80);  // (the loop held up with the clock at 80: put off too, and the clock was not low)
  CHECK(w.step(30000 + radioplan::kClockWaitMs, false, 80, false) == ClockWait::PUT_OFF && w.putOff() == 2 && w.waits() == 3);
  // on the way to the deep sleep of the shutdown: no radio, nothing counted
  w.begin(40000, 20);
  CHECK(w.step(40010, false, 20, true) == ClockWait::STOP && w.step(40020, true, 80, true) == ClockWait::STOP);
  CHECK(w.putOff() == 2 && w.waits() == 3);
  // across the wrap of millis()
  w.begin(0xFFFFFF00u, 20);
  CHECK(w.step(0x00000100u, false, 20, false) == ClockWait::WAIT);
  CHECK(w.step(0x00000200u, true, 80, false) == ClockWait::GO && w.longestMs() == radioplan::kClockWaitMs);
  // the wait is longer than the longest pass of the loop seen on the board (a frame 3.7 s late) and than a light sleep,
  // and short against the time a session has to join
  CHECK(radioplan::kClockWaitMs > 3700 && radioplan::kClockWaitMs > (uint32_t)lightsleep::kMaxSleepUs / 1000);
  CHECK(radioplan::kClockWaitMs + radioplan::kClockHoldMs < radioplan::kBackoffMs[0]);
}

void testPostpone() {
  section("radio plan: a session put off for the clock");
  {  // put off: no outcome, nothing held against the network, the radio stays off for the hold and comes on after it
    Drive d;
    d.in.workDue = true;
    CHECK(d.step(100) && d.p.state() == Planner::CONNECTING);
    d.p.postpone(d.t, kClockHoldMs);
    CHECK(d.p.state() == Planner::OFF && d.p.lastOutcome() == Planner::NONE && !d.p.takeGaveUp());
    CHECK(d.p.failures() == 0 && d.p.failedSessions() == 0 && d.p.awaySessions() == 0 && d.p.sessions() == 0);
    CHECK(d.p.holdUntilMs() != 0 && d.p.retryAtMs() == 0);
    const uint32_t t0 = d.t;
    bool on = false;
    while (!on && d.t - t0 < 60000) on = d.step(100);
    CHECK(on && d.t - t0 >= kClockHoldMs && d.t - t0 <= kClockHoldMs + 200);
    CHECK(d.p.holdUntilMs() == 0);
  }
  {  // a key press waits too (it is the clock that is not ready), and keeps its attempt: when the hold is over, the
     // session it asked for still comes, and the press is spent only by the session that is given up
    Drive d(true, 25000);
    d.in.demand = true;
    d.in.demandSeq = 5;
    CHECK(d.step(100));
    d.p.postpone(d.t, kClockHoldMs);
    for (int i = 0; i < 90; i++) CHECK(!d.step(100));  // 9 s: still held
    bool on = false;
    for (int i = 0; i < 20 && !on; i++) on = d.step(100);
    CHECK(on && d.p.state() == Planner::CONNECTING);
    CHECK(d.untilOff(60000, 100) && d.p.lastOutcome() == Planner::FAILED && d.p.failedSessions() == 1);
    for (int i = 0; i < 100; i++) CHECK(!d.step(100));  // the press had its attempt
  }
  {  // a look someone asked for stays one when it is put off: it does not use up a quick look
    Drive d;
    d.in.workDue = true;
    CHECK(d.step(100));
    for (int i = 0; i < 24; i++) d.step(100);
    d.in.absent = true;
    CHECK(!d.step(100) && d.p.takeGaveUp() && d.p.awayLooks() == 1 && d.p.awaySessions() == 1);
    d.in.absent = false;
    CHECK(!d.step(10000));
    d.p.lookNow();
    CHECK(d.step(100));
    d.p.postpone(d.t, kClockHoldMs);
    CHECK(d.p.lastOutcome() == Planner::AWAY);  // (what the Info page says stays as it was)
    bool on = false;
    for (int i = 0; i < 200 && !on; i++) on = d.step(100);
    CHECK(on);
    for (int i = 0; i < 24; i++) d.step(100);
    d.in.absent = true;
    CHECK(!d.step(100) && d.p.awayLooks() == 1 && d.p.awaySessions() == 2);
  }
  {  // only a session that is starting can be put off: not one that is off, online, or in always mode
    Drive d;
    d.p.postpone(d.t, kClockHoldMs);
    CHECK(d.p.holdUntilMs() == 0);
    d.in.workDue = true;
    d.step(100);
    d.in.online = true;
    d.step(100);
    d.p.postpone(d.t, kClockHoldMs);
    CHECK(d.p.state() == Planner::ONLINE && d.p.holdUntilMs() == 0);
    Drive a(false);
    a.step(100);
    a.p.postpone(a.t, kClockHoldMs);
    CHECK(a.p.state() == Planner::CONNECTING && a.step(100));
  }
  {  // the time to join counts from when the radio came on: a session that waited 4 s for the clock still has its
     // 25 s; and the wait was no time on the air
    for (int told = 0; told < 2; told++) {
      Drive d(true, 25000);
      d.in.workDue = true;
      CHECK(d.step(100));
      const uint32_t asked = d.t;
      d.t += 4000;  // (the network task waited for the clock; the planner was not asked meanwhile)
      if (told) d.p.radioOn(d.t);
      const uint32_t radioAt = d.t;
      bool on = true;
      while (on && d.t - asked < 60000) on = d.step(100);
      CHECK(!on && d.p.lastOutcome() == Planner::FAILED);
      if (told) {
        CHECK(d.t - radioAt >= 25000 && d.t - radioAt <= 25200);
        CHECK(d.p.radioOnMs(d.t) >= 25000 && d.p.radioOnMs(d.t) <= 25200);
      } else {  // (and without it, which is how it was: 4 s shorter, and the wait counted as on)
        CHECK(d.t - asked >= 25000 && d.t - asked <= 25200);
        CHECK(d.p.radioOnMs(d.t) >= 25000 && d.p.radioOnMs(d.t) <= 25200);
      }
    }
    Drive d;  // (and it only means something for a session that is starting)
    d.p.radioOn(d.t + 5000);
    CHECK(d.p.state() == Planner::OFF && !d.step(100) && d.p.radioOnMs(d.t) == 0);
    Drive j;  // ... not for one that is joined: work that never gets done still ends it 3 minutes after it began
    j.in.workDue = true;
    j.step(100);
    const uint32_t began = j.t;
    j.in.online = true;
    for (int i = 0; i < 1000; i++) j.step(100);
    j.p.radioOn(j.t);
    bool on = true;
    while (on && j.t - began < 400000) on = j.step(1000);
    CHECK(!on && j.p.lastOutcome() == Planner::FAILED && j.t - began >= kSessionMaxMs && j.t - began <= kSessionMaxMs + 1000);
  }
  {  // the totals since the start: failed and away sessions are counted apart, and a good session clears neither
    Drive d;
    d.in.workDue = true;
    for (int i = 0; i < 3; i++) {
      CHECK(d.untilOn(1000000, 100));
      CHECK(d.untilOff(60000, 100) && d.p.lastOutcome() == Planner::FAILED);
    }
    for (int i = 0; i < 2; i++) {
      CHECK(d.untilOn(1000000, 100));
      for (int k = 0; k < 24; k++) d.step(100);
      d.in.absent = true;
      CHECK(!d.step(100) && d.p.lastOutcome() == Planner::AWAY);
      d.in.absent = false;
    }
    CHECK(d.untilOn(1000000, 100));
    d.in.online = true;
    d.step(100);
    d.in.workDue = false;
    CHECK(d.untilOff(10000, 100) && d.p.lastOutcome() == Planner::DONE);
    CHECK(d.p.failedSessions() == 3 && d.p.awaySessions() == 2 && d.p.sessions() == 1 && d.p.failures() == 0 && d.p.awayLooks() == 0);
  }
}

// The UI loop (serviceClock() and lightSleepStep() in the .ino) and the network task's start of the radio (startRadio() in
// net_task.cpp), side by side, by events in microseconds.  A pass of the loop reads the radio's request and sets the
// clock, where a switch takes effect kSwitchUs after the loop decided on it (the core's own work), answers the
// request (not with `oldRule`), and then sleeps, naps, draws a frame or is held up.  A light sleep stops both tasks:
// the network task looks again 10 ms of ticks later, and ticks stand still while the chip sleeps.
//
// `oldRule`: the network task does as 1.12 did: it looks at the clock every 10 ms and starts the radio once it reads
// 80 MHz, or after 300 looks (3 s) regardless.
struct Tandem {
  static constexpr int64_t kNever = INT64_MAX;
  static constexpr int64_t kSwitchUs = 300;   // from the loop's decision to lower the clock to the clock being lower
  static constexpr int64_t kEntryUs = 3000;   // the SDK's way into a light sleep, while the other core still runs
  bool oldRule = false;
  int workMhz = 80, idleMhz = 20;
  bool lightSleep = false;
  int64_t sessionUs = 5000000;
  // the chip, and what the tasks share
  int64_t now = 0;
  int mhz = 80;
  bool asleep = false;
  bool needsFast = false;
  uint32_t asked = 0, answered = 0;
  int64_t pokeUntil = 0;
  // the UI loop
  enum Ui { UI_PASS, UI_SWITCH, UI_ENTER, UI_SLEEP, UI_BUSY } ui = UI_PASS;
  int64_t uiAt = 0;
  bool radioSeen = false;
  uint32_t askSeen = 0;
  int target = 80, pending = 0;
  int64_t stallUs = 0;  // the next pass is held up this long
  Rng rng{1};
  // the scenarios: the network task asks in the middle of the next switch down, of the next way into a sleep, or
  // just after the next stall began
  bool askInSwitch = false, askInEntry = false, askInStall = false;
  // the random run
  bool randomAsks = false;
  double stallChance = 0, pokeChance = 0;
  int64_t longestStallUs = 4500000;
  // the network task
  enum Net { NET_IDLE, NET_WAIT, NET_ON } net = NET_IDLE;
  int64_t netAt = kNever, askAt = kNever;
  bool netTicks = false;  // netAt is a wait in ticks (a light sleep puts it off)
  uint32_t myAsk = 0;
  int looks = 0;
  radioplan::ClockWait wait;
  // what was seen
  int asks = 0, starts = 0, startsLow = 0, putOff = 0, changesWhileOn = 0, stalls = 0;
  int64_t lowOnUs = 0, asleepOnUs = 0;

  explicit Tandem(bool old, uint64_t seed = 1) : oldRule(old), rng(seed) {}
  static uint32_t ms(int64_t us) { return (uint32_t)(us / 1000); }
  int violations() const { return startsLow + changesWhileOn + (lowOnUs > 0) + (asleepOnUs > 0); }

  void run(int64_t forUs) {
    const int64_t end = now + forUs;
    for (;;) {
      const int64_t netNext = asleep ? kNever : std::min(netAt, askAt);
      int64_t next = std::min(std::min(uiAt, netNext), end);
      if (next < now) next = now;  // (what fell due while the chip slept happens as it wakes)
      if (net == NET_ON) {  // what the radio had under it until then
        if (mhz < 80) lowOnUs += next - now;
        if (asleep) asleepOnUs += next - now;
      }
      now = next;
      if (uiAt <= now) {
        uiStep();
      } else if (!asleep && askAt <= now) {
        ask();
      } else if (!asleep && netAt <= now) {
        netStep();
      } else if (now >= end) {
        return;
      }
    }
  }

  void uiStep() {
    switch (ui) {
      case UI_PASS: {  // serviceClock(): the request's number first, then the request, then the clock
        if (randomAsks && rng.uni() < pokeChance) pokeUntil = now + (int64_t)clockpolicy::kPokeMs * 1000;
        askSeen = asked;
        radioSeen = needsFast;
        clockpolicy::Needs n;
        n.radio = radioSeen;
        n.poked = now < pokeUntil;
        target = clockpolicy::target(n, workMhz, idleMhz);
        if (clockpolicy::mayChange(mhz, target, true)) {
          pending = target;
          ui = UI_SWITCH;
          uiAt = now + kSwitchUs;
          if (askInSwitch && target < mhz) {
            askInSwitch = false;
            askAt = now + kSwitchUs / 2;
          }
          return;
        }
        finishPass();
        return;
      }
      case UI_SWITCH:
        if (net == NET_ON) changesWhileOn++;
        mhz = pending;
        finishPass();
        return;
      case UI_ENTER: {
        asleep = true;
        ui = UI_SLEEP;
        const int64_t len = 100000 + (int64_t)(800000 * rng.uni());
        uiAt = now + len;
        if (netTicks && netAt != kNever) netAt += len;
        return;
      }
      case UI_SLEEP:
        asleep = false;
        ui = UI_PASS;
        return;
      case UI_BUSY:
        ui = UI_PASS;
        return;
    }
  }
  void finishPass() {
    if (!oldRule && clockpolicy::radioMayStart(radioSeen, mhz, target, 0)) answered = askSeen;
    ui = UI_BUSY;
    if (stallUs == 0 && randomAsks && rng.uni() < stallChance) stallUs = 200000 + (int64_t)((longestStallUs - 200000) * rng.uni());
    if (stallUs > 0) {  // held up: a slow line of the power log, a frame 3.7 s late
      uiAt = now + stallUs;
      stallUs = 0;
      stalls++;
      if (askInStall) {
        askInStall = false;
        askAt = now + 1000;
      }
      return;
    }
    // lightSleepStep(): the request is read again; with it, or a button a moment ago, no sleep
    if (lightSleep && !needsFast && now >= pokeUntil && rng.uni() < 0.5) {
      ui = UI_ENTER;
      uiAt = now + kEntryUs;
      if (askInEntry) {
        askInEntry = false;
        askAt = now + kEntryUs / 2;
      }
      return;
    }
    uiAt = now + (rng.uni() < 0.3 ? (int64_t)30000 * 80 / mhz : 4000);  // a frame (four times as long at 20 MHz), or a nap
  }

  void ask() {  // startRadio(): the request, and the first look at once
    askAt = kNever;
    asks++;
    needsFast = true;
    myAsk = ++asked;
    looks = 0;
    wait.begin(ms(now), mhz);
    net = NET_WAIT;
    netStep();
  }
  void netStep() {
    if (net == NET_WAIT) {
      if (oldRule) {
        if (mhz >= 80 || looks >= 300) {
          start();
          return;
        }
        looks++;
      } else {
        const radioplan::ClockWait::Step s = wait.step(ms(now), answered == myAsk, mhz, false);
        if (s == radioplan::ClockWait::GO) {
          start();
          return;
        }
        if (s == radioplan::ClockWait::PUT_OFF) {  // the session is put off; the radio asks again after the hold
          putOff++;
          needsFast = false;
          net = NET_IDLE;
          netAt = kNever;
          askAt = now + (int64_t)radioplan::kClockHoldMs * 1000;
          return;
        }
      }
      netAt = now + 10000;
      netTicks = true;
      return;
    }
    if (net == NET_ON) {  // the session is over: the radio goes off, and then the clock may come down
      net = NET_IDLE;
      needsFast = false;
      netAt = kNever;
      if (randomAsks) askAt = now + 2000000 + (int64_t)(60000000 * rng.uni());
    }
  }
  void start() {
    starts++;
    if (mhz < 80) startsLow++;
    net = NET_ON;
    netAt = now + sessionUs;
    netTicks = false;  // (the session's time is by millis(), which a light sleep does not stop)
  }
};

void testRadioClockTasks() {
  section("radio and clock: the UI loop and the network task side by side");
  auto report = [](const char *what, const Tandem &t) {
    printf("  (%s, %s: %d starts, %d below 80 MHz, %.1f ms below 80 MHz and %.1f ms asleep with the radio on, %d clock changes under it, "
           "%d put off, the longest wait %u ms)\n",
           what, t.oldRule ? "the old rule" : "now", t.starts, t.startsLow, t.lowOnUs / 1000.0, t.asleepOnUs / 1000.0, t.changesWhileOn,
           t.putOff, (unsigned)t.wait.longestMs());
  };
  for (int old = 0; old < 2; old++) {
    {  // the loop is held up for 3.7 s (the longest seen on the board) just as the radio asks
      Tandem t(old != 0);
      t.mhz = 20;
      t.run(2000000);
      t.stallUs = 3700000;
      t.askInStall = true;
      t.run(12000000);
      report("held up 3.7 s", t);
      CHECK(t.asks == 1 && t.starts == 1);
      if (old) {
        CHECK(t.startsLow == 1);  // what 1.12 did: the radio came on at 20 MHz after 3 s
      } else {
        CHECK(t.startsLow == 0 && t.lowOnUs == 0 && t.putOff == 0);
        CHECK(t.wait.longestMs() >= 3650 && t.wait.longestMs() <= 3900 && t.wait.waits() == 1);
      }
    }
    {  // held up for 6 s: put off, not started at 20 MHz; the next try comes after the hold and goes
      Tandem t(old != 0);
      t.mhz = 20;
      t.run(2000000);
      t.stallUs = 6000000;
      t.askInStall = true;
      t.run(30000000);
      report("held up 6 s", t);
      if (old) {
        CHECK(t.startsLow == 1);
      } else {
        CHECK(t.putOff == 1 && t.wait.putOff() == 1 && t.asks == 2 && t.starts == 1 && t.startsLow == 0 && t.lowOnUs == 0);
      }
    }
    {  // the clock comes down (a button's 8 s are over) in the very moment the radio asks: the loop read "no radio" a
       // moment before
      Tandem t(old != 0);
      t.mhz = 20;
      t.run(1000000);
      t.pokeUntil = t.now + 8000000;
      t.run(1000000);
      CHECK(t.mhz == 80);
      t.askInSwitch = true;
      t.run(15000000);
      report("the clock comes down as the radio asks", t);
      CHECK(t.asks == 1 && t.starts == 1);
      if (old) {
        CHECK(t.changesWhileOn >= 1 && t.lowOnUs > 0);  // started at 80, then lowered under it (and raised again)
      } else {
        CHECK(t.startsLow == 0 && t.changesWhileOn == 0 && t.lowOnUs == 0);
      }
    }
    {  // no idle clock, light sleep on: the chip is on its way into a sleep as the radio asks
      Tandem t(old != 0);
      t.idleMhz = 0;
      t.lightSleep = true;
      t.run(1000000);
      t.askInEntry = true;
      t.run(15000000);
      report("a light sleep begins as the radio asks", t);
      CHECK(t.asks == 1 && t.starts == 1 && t.startsLow == 0);
      if (old) {
        CHECK(t.asleepOnUs > 0);  // the radio came on, and the chip went to sleep under it
      } else {
        CHECK(t.asleepOnUs == 0);
      }
    }
    {  // cpu_mhz = 240 with cpu_idle_mhz = 80: the clock is up enough at once, and still to be raised
      Tandem t(old != 0);
      t.workMhz = 240;
      t.idleMhz = 80;
      t.run(1000000);
      CHECK(t.mhz == 80);
      t.askAt = t.now + 1234;
      t.run(10000000);
      report("240 MHz with an idle clock of 80", t);
      CHECK(t.asks == 1 && t.starts == 1 && t.startsLow == 0);
      CHECK(t.changesWhileOn == (old ? 1 : 0));  // (1.12: the radio started at 80 and the clock went to 240 under it)
    }
  }
  // the same rules over many hours: sessions every few seconds to a minute, presses, stalls up to 4.5 s, light sleep on
  // and off, every idle clock.  Now: never a radio below 80 MHz, asleep, or under a clock change, and nothing put off
  // (no stall is long enough).  The old rule, on the very same days, runs into each of the cases above.
  int oldViolations = 0;
  for (int idle : {20, 40, 0})
    for (int sleep = 0; sleep < 2; sleep++)
      for (int old = 0; old < 2; old++) {
        Tandem t(old != 0, 77 + (uint64_t)idle * 3 + (uint64_t)sleep);
        t.idleMhz = idle;
        t.lightSleep = sleep != 0;
        t.randomAsks = true;
        t.stallChance = 0.0005;
        t.pokeChance = 0.0003;
        t.mhz = idle > 0 ? idle : 80;
        t.askAt = 3000000;
        t.run(6LL * 3600 * 1000000);
        if (old) {
          oldViolations += t.violations();
          continue;
        }
        if (t.violations() || t.putOff || idle == 20) {
          char what[48];
          snprintf(what, sizeof what, "6 h, idle %d MHz, light sleep %s", idle, sleep ? "on" : "off");
          report(what, t);
        }
        CHECK(t.starts > 300 && t.stalls > 10);
        CHECK(t.violations() == 0 && t.putOff == 0);
        CHECK(t.wait.longestMs() < radioplan::kClockWaitMs);
        CHECK(t.asks == t.starts || t.asks == t.starts + 1);  // (the last may still be waiting)
      }
  printf("  (the old rule on the same days: %d starts below 80 MHz, under a clock change, or with the clock low or asleep)\n", oldViolations);
  CHECK(oldViolations > 0);
}

void testTroubleRow() {
  section("radio: the second Radio row of the Power and settings page");
  using radioplan::troubleRow;
  char row[radioplan::kTroubleRow];
  CHECK(!troubleRow(0, 0, 0, 0, row) && row[0] == 0);  // all is well: no row at all
  CHECK(troubleRow(0, 0, 0, 201, row));
  CHECK_STR(row, "0 failed, 0 away, WiFi said 201");
  CHECK(troubleRow(2, 1, 0, 201, row));
  CHECK_STR(row, "2 failed, 1 away, WiFi said 201");
  CHECK(troubleRow(2, 1, 1, 201, row));
  CHECK_STR(row, "2 failed, 1 away, 1 put off, WiFi said 201");
  CHECK(strlen(row) == radioplan::kTroubleRow - 1);  // (exactly as wide as the page: the README's example)
  CHECK(troubleRow(3, 0, 0, 0, row));
  CHECK_STR(row, "3 failed, 0 away");
  CHECK(troubleRow(0, 0, 2, 0, row));
  CHECK_STR(row, "0 failed, 0 away, 2 put off");
  CHECK(troubleRow(12, 288, 3, 201, row));
  CHECK_STR(row, "12 failed, 288 away, 3 put off, WiFi 201");  // a little less said, all of it there
  CHECK(troubleRow(10000, 4294967295u, 9999, 15, row));
  CHECK_STR(row, "9999+ failed, 9999+ away, 9999 put off");
  // whatever the figures, the row fits (42 characters), and every count in it is the one given (or 9999+)
  const uint32_t counts[] = {0, 1, 9, 10, 99, 100, 999, 1000, 9999, 10000, 4294967295u};
  int rows = 0, widest = 0, reasonsShown = 0;
  for (uint32_t f : counts)
    for (uint32_t a : counts)
      for (uint32_t p : counts)
        for (unsigned reason : {0u, 2u, 15u, 201u, 255u}) {
          const bool any = troubleRow(f, a, p, (uint8_t)reason, row);
          if (!any) {
            CHECK(f == 0 && a == 0 && p == 0 && reason == 0);
            continue;
          }
          rows++;
          const int len = (int)strlen(row);
          if (len > widest) widest = len;
          CHECK(len < (int)radioplan::kTroubleRow);
          auto shown = [](uint32_t v) { return v > 9999 ? std::string("9999+") : std::to_string(v); };
          CHECK(std::string(row).find(shown(f) + " failed, " + shown(a) + " away") == 0);
          CHECK((p > 0) == (std::string(row).find(", " + shown(p) + " put off") != std::string::npos));
          const size_t at = std::string(row).find("WiFi");
          if (at != std::string::npos) {
            reasonsShown++;
            CHECK(reason != 0 && std::string(row).substr(std::string(row).rfind(' ') + 1) == std::to_string(reason));
          } else if (reason != 0) {  // left out only when even ", WiFi 201" would not fit
            CHECK(shown(f).size() + 9 + shown(a).size() + 5 + (p ? 2 + shown(p).size() + 8 : 0) + 7 + std::to_string(reason).size() > 42);
          }
        }
  printf("  (%d rows, the widest %d characters, the reason given in %d)\n", rows, widest, reasonsShown);
  CHECK(widest == 42 && reasonsShown > rows / 2);
  {  // the check can fail: the first form, as it is, does not fit with the largest figures
    char big[96];
    const int n = snprintf(big, sizeof big, "%s failed, %s away, %s put off, WiFi said %u", "9999+", "9999+", "9999+", 212u);
    CHECK(n >= (int)radioplan::kTroubleRow);
  }
  // the words for a reason, for the serial console
  CHECK_STR(radioplan::reasonText(201), "no such network");
  CHECK_STR(radioplan::reasonText(15), "key exchange timed out");
  CHECK_STR(radioplan::reasonText(0), "none yet");
  CHECK_STR(radioplan::reasonText(77), "");
}

}  // namespace

int main() {
  testSteps();
  testFailures();
  testSearch();
  testAway();
  testWrap();
  testSpotifyRules();
  testDay();
  testAwayDays();
  testTrackMode();
  testClockPolicy();
  testConsolePolicy();
  testPicker();
  testFramePlanClock();
  testFramePlanIdle();
  testLightSleepDecide();
  testNapSteps();
  testWallFix();
  testLightSleepLoop();
  testClockAnswer();
  testClockWait();
  testPostpone();
  testRadioClockTasks();
  testTroubleRow();
  printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
