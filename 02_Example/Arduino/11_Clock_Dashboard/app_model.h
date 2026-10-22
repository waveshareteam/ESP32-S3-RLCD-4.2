#pragma once

// Plain data shared between the network task, the UI loop and the renderer.
// No Arduino headers here on purpose: ui.cpp and the host-side preview tool
// (tools/ui_preview) compile against this file too.

#include <stdint.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Weather (Open-Meteo)
// ---------------------------------------------------------------------------
struct WeatherDay {
  uint8_t code = 0;      // WMO weather interpretation code
  float tmax = 0;        // degrees C
  float tmin = 0;        // degrees C
  uint8_t rainPct = 0;   // max precipitation probability, percent
  uint8_t weekday = 0;   // 0 = Sunday ... 6 = Saturday
};

struct WeatherData {
  bool valid = false;
  uint32_t ageSec = 0;     // filled in when a snapshot is taken for the UI
  float temp = 0;          // degrees C
  float feels = 0;         // degrees C
  float windKmh = 0;
  uint8_t humidity = 0;    // percent
  uint8_t code = 0;        // WMO code
  bool isDay = true;
  WeatherDay day[3];       // [0] today, [1] tomorrow, [2] the day after
  char sunrise[6] = "";    // local "HH:MM"
  char sunset[6] = "";
  float uvMax = 0;
};

// ---------------------------------------------------------------------------
// Spotify
// ---------------------------------------------------------------------------
enum SpotifyStatus : uint8_t {
  SPOTIFY_DISABLED = 0,   // no client id configured
  SPOTIFY_NEEDS_LINK,     // client id set but no refresh token yet
  SPOTIFY_IDLE,           // linked, nothing playing / no active device
  SPOTIFY_PAUSED,
  SPOTIFY_PLAYING,
  SPOTIFY_ERROR           // linked, but the last request failed (see message)
};

struct SpotifyInfo {
  SpotifyStatus status = SPOTIFY_DISABLED;
  char title[96] = "";
  char artist[96] = "";    // artists joined with ", " (or the show name for podcasts)
  char album[80] = "";
  char device[40] = "";
  uint32_t durationMs = 0;
  uint32_t progressMs = 0;  // already extrapolated to "now" when handed to the UI
  int8_t volume = -1;       // percent, -1 = unknown
  bool shuffle = false;
  char message[56] = "";    // short hint or error, e.g. "Premium required"
  char linkUrl[48] = "";    // where to open the linking page (set while it is being served)
  int16_t linkDaysLeft = -1;  // days until Spotify expires the link (6 months); -1 = unknown
};

// ---------------------------------------------------------------------------
// Everything the renderer needs for one frame
// ---------------------------------------------------------------------------
enum UiPage : uint8_t {
  PAGE_DASHBOARD = 0,
  PAGE_NOW_PLAYING = 1,
  PAGE_INFO = 2,    // system: network, time, weather, sensors
  PAGE_POWER = 3,   // battery, power, clock accuracy and the settings that are in force
  PAGE_LEGEND = 4,  // what the symbols and the button clicks mean
  PAGE_COUNT = 5
};

// What the battery is doing; shown as a small icon beside the gauge (nothing for UNKNOWN).
enum UiCharge : uint8_t {
  CHARGE_UNKNOWN = 0,
  CHARGE_DISCHARGING,  // running on the battery
  CHARGE_CHARGING,
  CHARGE_FULL          // charged and held there
};

#define UI_INFO_LINES 14  // lines on an info page (PAGE_INFO or PAGE_POWER)
#define UI_INFO_LINE_LEN 54

// The banner when KEY has been held long enough for a restart (it happens when the button is let go), and
// the one that follows.  Here because the layout check in tools/ui_preview measures the first one: it is
// the longest banner the clock shows.
#define UI_TOAST_RESTART_HOLD "Let go of KEY to restart (reads SD)"
#define UI_TOAST_RESTARTING "Restarting..."

// The screen shown at start-up while the firmware is updated from the SD card (uiDrawFirmwareUpdate).
enum UiFwKind : uint8_t {
  FW_BUSY = 0,  // checking or writing
  FW_DONE,      // installed, restarting
  FW_PROBLEM    // not installed; the clock carries on with the firmware it has
};
struct UiFwScreen {
  UiFwKind kind = FW_BUSY;
  char what[64] = "";        // the line in larger type: what is happening, or what went wrong
  char detail[64] = "";      // the file, or a hint
  int percent = -1;          // 0..100 draws a progress bar; -1 none
  bool keepPowered = false;  // "do not switch off or take the card out"
};

struct UiModel {
  // time -------------------------------------------------------------------
  bool timeValid = false;
  struct tm local = {};       // local time to display
  float secFrac = 0;          // 0..1 into the current second (sweeping second hand)
  int utcOffsetMin = 0;
  char tzAbbrev[8] = "";

  // indoor sensor ----------------------------------------------------------
  bool indoorValid = false;
  float indoorC = 0;
  float indoorRh = 0;

  // battery ----------------------------------------------------------------
  bool batPresent = false;
  float batVolts = 0;
  int batPercent = 0;
  bool batLow = false;        // below the warning threshold (and not charging)
  bool batBlinkOn = false;    // current blink phase (only meaningful when batLow)
  int batCharge = CHARGE_UNKNOWN;  // UiCharge

  // network ----------------------------------------------------------------
  bool wifiUp = false;
  bool wifiOff = false;       // switched off in the settings: no weather, no network time, no Spotify
  int rssi = -100;
  char location[40] = "";     // place shown in the status bar ("Earth" when nothing is known)
  char status[56] = "";       // transient progress / error message ("" = none)

  // data -------------------------------------------------------------------
  WeatherData weather;
  SpotifyInfo spotify;

  // the moon at this time today, tomorrow and the day after (moon.h); the picture is mirrored
  // in the southern hemisphere
  bool moonValid = false;
  uint8_t moonLit[3] = {0, 0, 0};          // percent of the disc that is lit
  bool moonWaxing[3] = {true, true, true};
  bool southern = false;

  // presentation -----------------------------------------------------------
  UiPage page = PAGE_DASHBOARD;
  bool useFahrenheit = false;
  bool time12h = false;       // 12 hour clock with AM / PM
  uint8_t dateFormat = 0;     // DateFormat (settings.h)
  bool showWeek = true;       // "Sunday - W40 - day 277" or just "Sunday"
  char toast[40] = "";        // short feedback overlay ("" = none)
  int toastIcon = 0;          // UiToastIcon
  int infoCount = 0;
  char info[UI_INFO_LINES][UI_INFO_LINE_LEN] = {};
};

enum UiToastIcon {
  TOAST_NONE = 0,
  TOAST_PLAY,
  TOAST_PAUSE,
  TOAST_NEXT,
  TOAST_PREV,
  TOAST_WARN
};
