#pragma once

// The clock's settings: what they are, how a hand-written settings file turns into
// them, how they are written back out as an example file, and how each one is turned
// into text for flash.  Pure logic with no Arduino dependency (the sketch keeps the
// values in flash and reads the file from the SD card), so tools/tests covers all of
// it on a PC.
//
// A setting has three possible sources, later ones winning:
//     built-in default (config.h / secrets.h)  <  saved in flash  <  the SD card file
// `userSet` records which settings came from the last two, so the file can say
// "forget that one" (an empty value) and the defaults come back.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum UnitSystem : uint8_t { UNITS_METRIC = 0, UNITS_IMPERIAL };
enum TimeFormat : uint8_t { TIME_24H = 0, TIME_12H };
enum DateFormat : uint8_t {
  DATE_ISO = 0,     // 2026-10-04
  DATE_DMY,         // 04/10/2026
  DATE_MDY,         // 10/04/2026
  DATE_DMY_DOT,     // 04.10.2026
  DATE_D_MON_Y,     // 4 Oct 2026
  DATE_MON_D_Y,     // Oct 4, 2026
  DATE_FORMAT_COUNT
};
enum BatteryMode : uint8_t { BATTERY_AUTO = 0, BATTERY_NONE };
enum WifiSave : uint8_t { WIFISAVE_NORMAL = 0, WIFISAVE_MAX };
enum WifiMode : uint8_t { WIFIMODE_ALWAYS = 0, WIFIMODE_SYNC };  // radio connected all the time, or only for the syncs (radio_plan.h)
enum CpuSpeed : uint8_t { CPU_80 = 0, CPU_160, CPU_240 };
enum CpuIdle : uint8_t { CPUIDLE_OFF = 0, CPUIDLE_80, CPUIDLE_40, CPUIDLE_20 };  // the clock while the radio is off
// The USB serial console: always, only while a computer is on the USB port, or never (console_policy.h)
enum ConsoleMode : uint8_t { CONSOLE_ON = 0, CONSOLE_AUTO, CONSOLE_OFF };
// How the display's controller drives the panel: its high power mode (25.5 Hz) or its low one (8 Hz; display_power.h)
enum DisplayPower : uint8_t { DISPPOWER_NORMAL = 0, DISPPOWER_LOW };

struct Settings {
  // --- WiFi ---------------------------------------------------------------------
  bool wifi = true;
  char wifiSsid[33] = "";
  char wifiPassword[65] = "";
  char wifiBackupSsid[33] = "";      // tried when the main network cannot be joined; "" = no backup
  char wifiBackupPassword[65] = "";
  char hostname[32] = "rlcd-clock";
  char ntpServer[48] = "";  // "" = the built-in list
  uint8_t wifiPowerSave = WIFISAVE_NORMAL;  // what the Arduino core does anyway; "max" is untried and can delay the time sync
  uint8_t wifiMode = WIFIMODE_ALWAYS;       // "sync": the radio is off between syncs (untried on the board)
  // --- units and formats --------------------------------------------------------
  uint8_t units = UNITS_METRIC;
  uint8_t timeFormat = TIME_24H;
  uint8_t dateFormat = DATE_ISO;
  bool showWeek = true;  // ISO week number and day of the year after the weekday
  // --- location -----------------------------------------------------------------
  double latitude = 0;
  double longitude = 0;
  char locationLabel[40] = "";
  char location[64] = "";  // a place name to look up, used when there are no coordinates
  char timezone[72] = "";  // IANA name or POSIX rule; "" = follow the location
  // --- Spotify ------------------------------------------------------------------
  bool spotify = true;
  char spotifyClientId[40] = "";
  bool spotifyLive = true;  // wifi_mode = sync: the radio stays on while music plays; off = it looks once a track (untried on the board)
  // --- sensors and battery ------------------------------------------------------
  float indoorOffsetC = -4.0f;
  uint8_t battery = BATTERY_AUTO;
  int32_t batteryCapacityMah = 0;
  float batteryCalibration = 1.0f;  // multiplies the measured battery voltage (real / shown); 1 = none
  bool lowBatteryShutdown = true;
  float batteryCutoffV = 3.30f;
  // --- power --------------------------------------------------------------------
  uint8_t cpuSpeed = CPU_80;
  uint8_t cpuIdle = CPUIDLE_OFF;  // the CPU clock while the radio is off; off = no change (untried on the board)
  uint8_t console = CONSOLE_ON;   // the USB serial console; auto and off shut it down (untried on the board)
  bool powerLog = false;          // a line of readings in the clock's flash every ten minutes (power_log.h; untried on the board)
  bool lightSleep = false;        // light sleep between the frames while nothing needs the chip (light_sleep.h; untried on the board)
  uint8_t displayPower = DISPPOWER_NORMAL;  // the panel's refresh: low = the controller's low power mode (display_power.h; untried on the board)
  // --- weather ------------------------------------------------------------------
  int32_t weatherIntervalMin = 15;

  // Bit i is set when setting i (in table order, see settingKey()) came from flash or
  // the SD card rather than the built-in defaults.  (Not kept anywhere: the flash holds each setting under its
  // own name, and the bit is set again as it is read back.)
  uint64_t userSet = 0;

  bool hasMainWifi() const { return wifiSsid[0] != 0; }
  // A backup that is the same network as the main one is no backup.
  bool hasBackupWifi() const { return wifiBackupSsid[0] != 0 && strcmp(wifiBackupSsid, wifiSsid) != 0; }
  bool imperial() const { return units == UNITS_IMPERIAL; }
  bool time12h() const { return timeFormat == TIME_12H; }
  bool hasLatLon() const { return latitude != 0 || longitude != 0; }
  int cpuMhz() const { return cpuSpeed == CPU_240 ? 240 : (cpuSpeed == CPU_160 ? 160 : 80); }
  // The clock while the radio is off, or 0 for "the same as cpuMhz()".  Never above cpuMhz().  (40 MHz is the
  // lowest clock Espressif gives figures for and 20 the lowest the Arduino core lists for this chip; 10 is left
  // out: a frame would take a quarter of a second to draw, which leaves little of each second to save in.)
  int cpuIdleMhz() const {
    const int mhz = cpuIdle == CPUIDLE_80 ? 80 : cpuIdle == CPUIDLE_40 ? 40 : cpuIdle == CPUIDLE_20 ? 20 : 0;
    return mhz > 0 && mhz < cpuMhz() ? mhz : 0;
  }
  bool syncMode() const { return wifi && wifiMode == WIFIMODE_SYNC; }
  bool hasBattery() const { return battery != BATTERY_NONE; }
};

// ---------------------------------------------------------------------------
// The settings table
// ---------------------------------------------------------------------------
size_t settingCount();
const char *settingKey(size_t i);      // as written in the file: "wifi_ssid"
const char *settingNvsKey(size_t i);   // short name for flash (15 characters at most)
bool settingIsSecret(size_t i);        // passwords and ids: never echoed
bool settingIsUserSet(const Settings &s, size_t i);

// The setting as text for flash: booleans "on"/"off", enums by name, strings as they are.
// Returns false if it does not fit.
bool formatSetting(const Settings &s, size_t i, char *out, size_t cap);

// Applies text previously produced by formatSetting() (flash load), or the build's defaults: the
// text is the value as it is, with none of the file's quoting and comment rules, but it has to pass
// the same checks.  A value that does not (a newer firmware changed the rules) is ignored, and `err`
// (if given) says why.  Marks it user-set.
bool applyStoredSetting(Settings &s, size_t i, const char *value, char *err = nullptr, size_t errCap = 0);

// ---------------------------------------------------------------------------
// The settings file
// ---------------------------------------------------------------------------
const int kConfigMaxIssues = 8;

struct ConfigIssue {
  int line = 0;
  char text[96] = "";
};

struct ConfigReport {
  int lines = 0;      // lines in the file
  int changed = 0;    // settings that now differ from what they were
  int unchanged = 0;  // ... set to what they already were
  int cleared = 0;    // ... forgotten (empty value), so the default is back
  int unknown = 0;    // names the clock does not know
  int bad = 0;        // known names with a value that does not fit
  bool resetAll = false;
  int issueCount = 0;  // the first few problems, for the screen
  ConfigIssue issues[kConfigMaxIssues];

  int problems() const { return unknown + bad; }
  int touched() const { return changed + cleared + (resetAll ? 1 : 0); }
};

// Reads a whole settings file and applies every active line to `s`.  `defaults` is what a
// forgotten setting reverts to.  Tolerant of the things people do: UTF-8 / UTF-16 byte-order
// marks, CR or CRLF line ends, spaces around "=" (or ":"), quoted or bare values (straight
// or curly quotes), upper case, "-" or spaces in names, a comma as the decimal mark, a unit
// after a number, N/S/E/W after a coordinate, and #-comments after bare values.
ConfigReport applyConfigText(Settings &s, const Settings &defaults, const char *text, size_t len);

// One "name = value" on its own, with the same tolerance.  An empty value forgets the
// setting.  On failure `err` says why (if given).
enum class ApplyResult { Changed, Unchanged, Cleared, UnknownKey, BadValue };
ApplyResult applySetting(Settings &s, const Settings &defaults, const char *key, const char *value,
                         char *err = nullptr, size_t errCap = 0);

// One line like "7 changed, 1 unchanged, 2 problems" for the screen.
void formatConfigSummary(const ConfigReport &r, char *out, size_t cap);

// What the banner says about the settings file of an SD card, after a start that read one (a banner holds
// 39 characters).  Returns true if it is a warning.
//   carried        settings that the start before this one changed and saved without being able to say so:
//                  it went on to install a firmware from the same card and restarted.  This start reads the
//                  same file, finds nothing left to change, and would report that the card did nothing.
//   justInstalled  this firmware came from the card at the last start, which read the same file.  If
//                  nothing was carried over (the firmware that ran then did not keep a count yet), whether
//                  the file changed anything is not known, so "none new" is left unsaid.
// (Here, and not in settings.cpp, so that the UI preview can measure the texts it makes.)
const char *const kSdNoteNvsKey = "sdnote";  // where `carried` is kept in flash, next to the settings: no setting may use this name
inline bool formatSdBanner(const ConfigReport &r, bool savedToFlash, int carried, bool justInstalled, char *out, size_t cap) {
  if (carried < 0) carried = 0;
  const int changed = r.touched() + carried;
  if (r.touched() > 0 && !savedToFlash) {  // in force for this run, gone after the next restart
    snprintf(out, cap, "%d changed, NOT saved to flash", r.touched());
    return true;
  }
  if (r.problems() > 0) {
    snprintf(out, cap, "SD: %d changed, %d problem%s", changed, r.problems(), r.problems() == 1 ? "" : "s");
    return true;
  }
  if (changed > 0) {
    snprintf(out, cap, "SD settings: %d changed", changed);
  } else if (r.unchanged == 0) {  // every line is a comment: the file the clock wrote, as it wrote it
    snprintf(out, cap, "SD settings: the file sets nothing");
  } else if (justInstalled) {
    snprintf(out, cap, "SD settings: %d in force", r.unchanged);
  } else {
    snprintf(out, cap, "SD settings: %d in force, none new", r.unchanged);
  }
  return false;
}

// The file the clock writes to a card that has none: every setting commented out with its
// current value (never passwords or ids) and an explanation.  `driftNote` (may be empty)
// is the measured clock drift, put into the note about running without WiFi.
// Returns the length, or 0 if `cap` is too small.  `crlf` writes Windows line ends (the clock
// does, for the sake of Notepad on the other side of the card); the parser reads either.
size_t renderExampleConfig(const Settings &current, const char *driftNote, char *out, size_t cap, bool crlf = false);
