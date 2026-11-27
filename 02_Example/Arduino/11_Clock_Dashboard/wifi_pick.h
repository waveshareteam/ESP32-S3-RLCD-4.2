#pragma once

// Which WiFi network the clock tries next, when it has a backup.  Pure logic (no WiFi calls; the
// times are passed in), so tools/tests covers it on a PC.
//
//   * The main network always comes first: at start-up, and after a connection was lost.
//   * When an attempt runs out of time the other network is tried, and so on, alternating.
//   * While the clock sits on the backup it looks for the main network every 10 minutes (the sketch
//     scans for it) and goes back as soon as it is in range, so it does not stay on, say, a phone's
//     hotspot for days.  If going back does not work out (the main network is in range but cannot be
//     joined) the clock ends up on the backup again and the looks come further apart: 20 minutes,
//     40, then every hour.  Joining the main network starts over at 10 minutes.

#include <stdint.h>
#include <string.h>

namespace wifipick {

enum Net : int8_t { NET_NONE = -1, NET_MAIN = 0, NET_BACKUP = 1 };

const uint32_t kLookFirstMs = 10UL * 60UL * 1000UL;
const uint32_t kLookMaxMs = 60UL * 60UL * 1000UL;

// Which of the two a connection is, going by the name the WiFi stack reports (it may have
// reconnected by itself): the main network if the name matches, the backup if that one does.
inline Net classify(const char *joined, const char *mainSsid, const char *backupSsid) {
  if (mainSsid[0] && !strcmp(joined, mainSsid)) return NET_MAIN;
  if (backupSsid[0] && !strcmp(joined, backupSsid)) return NET_BACKUP;
  return mainSsid[0] ? NET_MAIN : NET_BACKUP;  // a name we do not know: say what we have
}

class Picker {
 public:
  // Which names are set; a backup equal to the main network counts as none (Settings::hasBackupWifi).
  void configure(bool hasMain, bool hasBackup) {
    hasMain_ = hasMain;
    hasBackup_ = hasBackup;
    current_ = NET_NONE;
    lastTried_ = NET_NONE;
    lookAtMs_ = 0;
    failedReturns_ = 0;
    returning_ = false;
  }

  // The network for the next association attempt; NET_NONE if no name is set.
  Net nextAttempt() {
    Net n = NET_NONE;
    if (hasMain_ && hasBackup_) {
      n = lastTried_ == NET_MAIN ? NET_BACKUP : NET_MAIN;
    } else if (hasMain_) {
      n = NET_MAIN;
    } else if (hasBackup_) {
      n = NET_BACKUP;
    }
    lastTried_ = n;
    return n;
  }

  // The connection is up on `net`.
  void joined(Net net, uint32_t nowMs) {
    current_ = net;
    if (net == NET_BACKUP) {
      if (returning_ && failedReturns_ < 8) failedReturns_++;  // we set out for the main network and are back here
      lookAtMs_ = nowMs + lookInterval();
    } else if (net == NET_MAIN) {
      failedReturns_ = 0;
    }
    returning_ = false;
  }

  // The connection dropped (or was dropped on purpose): the next attempt starts with the main network.
  void lost() {
    current_ = NET_NONE;
    lastTried_ = NET_NONE;
  }

  // The radio was switched off on purpose (wifi_mode = sync): the network joined is still the network to
  // go back to, so the next attempt starts with it, not with the main one when the clock is on the backup
  // (that would cost a failed attempt of 20 seconds at every session).  The backup's looks for the main
  // network go on by the clock.
  void radioOff() {
    lastTried_ = current_ == NET_BACKUP ? NET_MAIN : NET_NONE;
  }

  Net current() const { return current_; }
  bool onBackup() const { return current_ == NET_BACKUP; }

  // On the backup, and long enough since it was joined (or since the last look)?
  bool timeToLookForMain(uint32_t nowMs) const {
    return hasMain_ && current_ == NET_BACKUP && (int32_t)(nowMs - lookAtMs_) >= 0;
  }

  // The sketch looked: `mainSeen` says the main network is in range.  Then the clock leaves the backup
  // and the next attempt is the main network.
  void lookedForMain(uint32_t nowMs, bool mainSeen) {
    lookAtMs_ = nowMs + kLookFirstMs;  // (until the next join sets the real interval)
    if (mainSeen) returning_ = true;
  }

  uint32_t lookInterval() const {
    uint32_t t = kLookFirstMs;
    for (int i = 0; i < failedReturns_ && t < kLookMaxMs; i++) t *= 2;
    return t > kLookMaxMs ? kLookMaxMs : t;
  }

 private:
  bool hasMain_ = false, hasBackup_ = false;
  Net current_ = NET_NONE;
  Net lastTried_ = NET_NONE;
  uint32_t lookAtMs_ = 0;
  int failedReturns_ = 0;
  bool returning_ = false;
};

}  // namespace wifipick
