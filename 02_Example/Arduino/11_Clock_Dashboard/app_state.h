#pragma once

// State shared between the network task (core 0) and the UI loop (core 1).
// Writers take a StateLock for the few microseconds a copy needs; the UI takes
// one snapshot per frame.

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <time.h>

#include "app_model.h"

// wifi_mode = sync: how the last radio session ended, if not well.
enum RadioTrouble : uint8_t {
  RADIO_FINE = 0,
  RADIO_AWAY,         // none of the clock's networks is in range: it looks again every few minutes
  RADIO_CANNOT_JOIN   // a network is there but the clock could not get in (or get anything done): it backs off
};

struct SharedState {
  // --- network & time -------------------------------------------------------
  bool wifiUp = false;
  bool wifiBackup = false;  // the connection is the backup network, not the main one
  int rssi = -100;
  char ssid[33] = "";
  char ip[16] = "";
  bool timeTrusted = false;  // system clock is believed correct (NTP synced, or valid RTC)
  bool ntpSynced = false;    // NTP has completed at least once since boot
  time_t lastNtpSyncUtc = 0;
  char status[56] = "";      // transient progress / error text for the status bar

  // --- the radio (wifi_mode = sync) -------------------------------------------
  bool radioSync = false;           // sync mode is on
  bool radioAsleep = false;         // ... and the radio is off at the moment
  uint8_t radioTrouble = 0;         // how the last session ended if not well: RADIO_FINE, RADIO_AWAY, RADIO_CANNOT_JOIN
  int32_t radioWakeInSec = -1;      // seconds until the next session by the clock, -1 = not known
  uint16_t radioOnPermille = 0;     // the share of the time since start that the radio was on, in thousandths
  uint32_t radioSessions = 0;

  // --- clock accuracy (drift.h) ----------------------------------------------
  bool driftKnown = false;   // a figure exists: measured now, or kept in flash from an earlier run
  bool driftLive = false;    // ... measured during this run
  float driftPpm = 0;        // positive: the clock gains
  float driftErrPpm = 0;
  float driftHours = 0;      // the history the figure rests on

  // --- location & zone ------------------------------------------------------
  bool locationKnown = false;
  double lat = 0;
  double lon = 0;
  char place[40] = "";
  char tzName[48] = "";   // IANA name, e.g. "Australia/Perth"
  char tzPosix[72] = "";  // POSIX rule in effect, e.g. "AWST-8"

  // --- weather --------------------------------------------------------------
  WeatherData weather;
  uint32_t weatherFetchedMs = 0;  // millis() of the last good fetch

  // --- Spotify (progressMs is the value at spotifySampledMs) ----------------
  SpotifyInfo spotify;
  uint32_t spotifySampledMs = 0;

  // --- one-shot notices from background tasks, shown by the UI as a toast ---
  char notice[40] = "";
  int noticeIcon = 0;
  uint32_t noticeSeq = 0;
};

extern SharedState g_state;

void stateInit();

class StateLock {
 public:
  StateLock();
  ~StateLock();
};

// printf-style transient status line ("" clears it).
void stateSetStatus(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Queue a toast for the UI.  `icon` is a UiToastIcon.
void stateNotice(const char *text, int icon);

// A flash write (NVS) stops both cores from running code out of flash for a few
// milliseconds, now and then for a few tens of them.  Call this first so the write
// lands in the middle of a second, away from the moment the UI sends the next frame.
// Blocks the caller for at most a second.
void waitForQuietPhase();
