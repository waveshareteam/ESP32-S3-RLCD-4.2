#include "settings.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tz_table.h"

namespace {

// ---------------------------------------------------------------------------
// The table
// ---------------------------------------------------------------------------
enum Type : uint8_t { T_BOOL, T_INT, T_FLOAT, T_DOUBLE, T_STRING, T_ENUM };

struct EnumName {
  const char *name;  // the first name listed for a value is the one written out
  int value;
};

struct Def {
  const char *key;
  const char *nvs;  // flash key, 15 characters at most
  Type type;
  size_t offset;    // into Settings
  size_t size;      // string buffer size
  double lo, hi;    // numeric limits
  const EnumName *names;
  bool secret;
  const char *group;  // heading in the example file, printed when it changes
  const char *help;   // comment lines for the example file, '\n' separated
};

const EnumName kUnitNames[] = {{"metric", UNITS_METRIC},     {"celsius", UNITS_METRIC},  {"c", UNITS_METRIC},
                               {"si", UNITS_METRIC},         {"imperial", UNITS_IMPERIAL}, {"fahrenheit", UNITS_IMPERIAL},
                               {"f", UNITS_IMPERIAL},        {"us", UNITS_IMPERIAL},      {"mph", UNITS_IMPERIAL},
                               {nullptr, 0}};
const EnumName kTimeNames[] = {{"24h", TIME_24H}, {"24", TIME_24H}, {"24hour", TIME_24H}, {"24hours", TIME_24H},
                               {"12h", TIME_12H}, {"12", TIME_12H}, {"12hour", TIME_12H}, {"12hours", TIME_12H},
                               {"ampm", TIME_12H}, {"am/pm", TIME_12H}, {nullptr, 0}};
const EnumName kDateNames[] = {
    {"iso", DATE_ISO},          {"yyyy-mm-dd", DATE_ISO},     {"ymd", DATE_ISO},         {"iso8601", DATE_ISO},
    {"dmy", DATE_DMY},          {"dd/mm/yyyy", DATE_DMY},     {"d/m/yyyy", DATE_DMY},
    {"mdy", DATE_MDY},          {"mm/dd/yyyy", DATE_MDY},     {"m/d/yyyy", DATE_MDY},
    {"dmy-dot", DATE_DMY_DOT},  {"dd.mm.yyyy", DATE_DMY_DOT}, {"d.m.yyyy", DATE_DMY_DOT},
    {"d-mon-y", DATE_D_MON_Y},  {"d mon yyyy", DATE_D_MON_Y}, {"dd mon yyyy", DATE_D_MON_Y}, {"d mmm yyyy", DATE_D_MON_Y},
    {"mon-d-y", DATE_MON_D_Y},  {"mon d, yyyy", DATE_MON_D_Y}, {"mmm d, yyyy", DATE_MON_D_Y}, {"mon d yyyy", DATE_MON_D_Y},
    {nullptr, 0}};
const EnumName kBatteryNames[] = {{"auto", BATTERY_AUTO}, {"automatic", BATTERY_AUTO}, {"fitted", BATTERY_AUTO},
                                  {"present", BATTERY_AUTO}, {"yes", BATTERY_AUTO},    {"on", BATTERY_AUTO},
                                  {"none", BATTERY_NONE},
                                  {"no", BATTERY_NONE},     {"usb", BATTERY_NONE},     {"absent", BATTERY_NONE},
                                  {"off", BATTERY_NONE},    {nullptr, 0}};
const EnumName kWifiSaveNames[] = {{"normal", WIFISAVE_NORMAL}, {"off", WIFISAVE_NORMAL}, {"min", WIFISAVE_NORMAL},
                                   {"default", WIFISAVE_NORMAL}, {"max", WIFISAVE_MAX},   {"maximum", WIFISAVE_MAX},
                                   {"on", WIFISAVE_MAX},        {"saver", WIFISAVE_MAX},  {nullptr, 0}};
const EnumName kWifiModeNames[] = {{"always", WIFIMODE_ALWAYS}, {"on", WIFIMODE_ALWAYS}, {"connected", WIFIMODE_ALWAYS}, {"stay", WIFIMODE_ALWAYS},
                                   {"sync", WIFIMODE_SYNC},     {"sync-only", WIFIMODE_SYNC}, {"periodic", WIFIMODE_SYNC}, {"saver", WIFIMODE_SYNC},
                                   {nullptr, 0}};
const EnumName kCpuIdleNames[] = {{"off", CPUIDLE_OFF}, {"same", CPUIDLE_OFF}, {"none", CPUIDLE_OFF}, {"no", CPUIDLE_OFF}, {"0", CPUIDLE_OFF},
                                  {"80", CPUIDLE_80},   {"80mhz", CPUIDLE_80}, {"40", CPUIDLE_40},    {"40mhz", CPUIDLE_40},
                                  {"20", CPUIDLE_20},   {"20mhz", CPUIDLE_20}, {nullptr, 0}};
const EnumName kConsoleNames[] = {{"on", CONSOLE_ON},     {"yes", CONSOLE_ON},  {"always", CONSOLE_ON}, {"true", CONSOLE_ON},  {"1", CONSOLE_ON},
                                  {"auto", CONSOLE_AUTO}, {"usb", CONSOLE_AUTO}, {"automatic", CONSOLE_AUTO},
                                  {"off", CONSOLE_OFF},   {"no", CONSOLE_OFF},  {"never", CONSOLE_OFF}, {"false", CONSOLE_OFF}, {"0", CONSOLE_OFF},
                                  {nullptr, 0}};
const EnumName kDisplayPowerNames[] = {{"normal", DISPPOWER_NORMAL}, {"high", DISPPOWER_NORMAL}, {"hpm", DISPPOWER_NORMAL},
                                       {"default", DISPPOWER_NORMAL}, {"low", DISPPOWER_LOW},     {"lpm", DISPPOWER_LOW},
                                       {"saver", DISPPOWER_LOW},      {nullptr, 0}};
const EnumName kCpuNames[] = {{"80", CPU_80},   {"80mhz", CPU_80},   {"160", CPU_160}, {"160mhz", CPU_160},
                              {"240", CPU_240}, {"240mhz", CPU_240}, {nullptr, 0}};

#define OFF(field) offsetof(Settings, field)
#define SZ(field) sizeof(((Settings *)nullptr)->field)

const Def kDefs[] = {
    // 0
    {"wifi", "wifi", T_BOOL, OFF(wifi), 0, 0, 0, nullptr, false, "WiFi",
     "on or off.  Off keeps the radio off: no network time, weather or Spotify, and less\n"
     "power drawn.  Without network time the clock runs on its own crystal and drifts\n"
     "(see the note at the end of this file)."},
    {"wifi_ssid", "ssid", T_STRING, OFF(wifiSsid), SZ(wifiSsid), 0, 0, nullptr, false, "WiFi",
     "Name of your WiFi network (2.4 GHz only)."},
    {"wifi_password", "pass", T_STRING, OFF(wifiPassword), SZ(wifiPassword), 0, 0, nullptr, true, "WiFi",
     "Its password.  Kept as plain text here and in the clock's flash.  For an open\n"
     "network write two quotes: wifi_password = \"\""},
    {"wifi_backup_ssid", "ssid2", T_STRING, OFF(wifiBackupSsid), SZ(wifiBackupSsid), 0, 0, nullptr, false, "WiFi",
     "A second network, for when the first cannot be joined (a phone's hotspot, another\n"
     "router).  The clock tries the main network first.  While it is on the backup it looks\n"
     "for the main one every 10 minutes and goes back to it.  Empty = no backup."},
    {"wifi_backup_password", "pass2", T_STRING, OFF(wifiBackupPassword), SZ(wifiBackupPassword), 0, 0, nullptr, true, "WiFi",
     "Its password (for an open network write two quotes)."},
    {"hostname", "host", T_STRING, OFF(hostname), SZ(hostname), 0, 0, nullptr, false, "WiFi",
     "The clock's name on your network: letters, digits and '-'."},
    {"ntp_server", "ntp", T_STRING, OFF(ntpServer), SZ(ntpServer), 0, 0, nullptr, false, "WiFi",
     "Time server to try first (pool.ntp.org, time.cloudflare.com and time.google.com\n"
     "follow).  Empty = the built-in list."},
    {"wifi_power_save", "wps", T_ENUM, OFF(wifiPowerSave), 0, 0, 0, kWifiSaveNames, false, "WiFi",
     "normal or max.  Max lets the radio sleep longer between messages: a little less\n"
     "power, but every reply can come up to a third of a second late, the network time\n"
     "included (look at \"last step\" on the Info page).  The Spotify setup page always\n"
     "runs in normal."},
    {"wifi_mode", "wmode", T_ENUM, OFF(wifiMode), 0, 0, 0, kWifiModeNames, false, "WiFi",
     "always or sync.  Always keeps the radio connected.  Sync switches it off between syncs: it\n"
     "wakes for the network time (hourly) and the weather (every weather_interval_min), looks at\n"
     "Spotify once each time, and stays on while music plays (see spotify_live), after a key press\n"
     "and on the Now Playing page.  Draws far less on a battery; the Spotify strip and the link page\n"
     "need that page or a key press to wake the radio.  When none of its networks is in range the\n"
     "clock looks again every 2 minutes, later every 5, and at once when a button is pressed.\n"
     "See the README."},
    // 8
    {"units", "units", T_ENUM, OFF(units), 0, 0, 0, kUnitNames, false, "Units and formats",
     "metric (degrees C, km/h) or imperial (degrees F, mph)."},
    {"time_format", "tfmt", T_ENUM, OFF(timeFormat), 0, 0, 0, kTimeNames, false, "Units and formats",
     "24h or 12h (12h shows AM or PM)."},
    {"date_format", "dfmt", T_ENUM, OFF(dateFormat), 0, 0, 0, kDateNames, false, "Units and formats",
     "iso (2026-10-04), dmy (04/10/2026), mdy (10/04/2026), dmy-dot (04.10.2026),\n"
     "d-mon-y (4 Oct 2026) or mon-d-y (Oct 4, 2026)."},
    {"show_week", "week", T_BOOL, OFF(showWeek), 0, 0, 0, nullptr, false, "Units and formats",
     "on or off.  Shows the ISO week number and the day of the year after the weekday."},
    // 12
    {"latitude", "lat", T_DOUBLE, OFF(latitude), 0, -90, 90, nullptr, false, "Location",
     "Exact coordinates of the clock, in degrees: -31.952 and 115.861, or 31.952 S and\n"
     "115.861 E.  They win over the place name below."},
    {"longitude", "lon", T_DOUBLE, OFF(longitude), 0, -180, 180, nullptr, false, "Location", nullptr},
    {"location_label", "label", T_STRING, OFF(locationLabel), SZ(locationLabel), 0, 0, nullptr, false, "Location",
     "What the status bar shows.  Empty = the name that was found."},
    {"location", "place", T_STRING, OFF(location), SZ(location), 0, 0, nullptr, false, "Location",
     "A place to look up (Open-Meteo), such as Perth or \"Paris, France\".  Used when no\n"
     "coordinates are set."},
    {"timezone", "tz", T_STRING, OFF(timezone), SZ(timezone), 0, 0, nullptr, false, "Location",
     "Overrides the automatic time zone: an IANA name (Australia/Perth) or a POSIX rule\n"
     "(AWST-8).  Empty = follow the location, which needs WiFi: with wifi = off set it here."},
    // 17
    {"spotify", "spot", T_BOOL, OFF(spotify), 0, 0, 0, nullptr, false, "Spotify",
     "on or off.  Spotify also needs the Client ID below."},
    {"spotify_client_id", "spid", T_STRING, OFF(spotifyClientId), SZ(spotifyClientId), 0, 0, nullptr, true, "Spotify",
     "The Client ID of your app in the Spotify developer dashboard (see the README)."},
    {"spotify_live", "splive", T_BOOL, OFF(spotifyLive), 0, 0, 0, nullptr, false, "Spotify",
     "on or off.  Only matters with wifi_mode = sync.  On: while music plays the radio stays on\n"
     "and the clock asks Spotify every 10 seconds, so the strip follows your phone within seconds.\n"
     "Off: the radio sleeps while music plays too, and the clock looks again when the track on\n"
     "screen should be over.  Far less radio, but a track skipped or paused on a phone shows late:\n"
     "when it would have ended, or at the next weather update.  KEY presses and the Now Playing\n"
     "page are live either way."},
    // 19
    {"indoor_offset", "ioff", T_FLOAT, OFF(indoorOffsetC), 0, -15, 15, nullptr, false, "Sensors and battery",
     "Degrees C added to the indoor temperature to make up for the board warming the\n"
     "sensor (the factory value is -4.0)."},
    {"battery", "bat", T_ENUM, OFF(battery), 0, 0, 0, kBatteryNames, false, "Sensors and battery",
     "auto or none.  Say none when no battery is fitted (running from USB): the gauge\n"
     "then shows USB, nothing is estimated and the clock never shuts down for a low\n"
     "battery.  The clock cannot tell by itself: on USB the empty battery socket reads\n"
     "like a full battery."},
    {"battery_capacity_mah", "batmah", T_INT, OFF(batteryCapacityMah), 0, 0, 20000, nullptr, false, "Sensors and battery",
     "Capacity of your battery in mAh, so the Power and settings page can show the\n"
     "average current.  0 = unknown."},
    {"battery_calibration", "batcal", T_FLOAT, OFF(batteryCalibration), 0, 0.80, 1.25, nullptr, false, "Sensors and battery",
     "Corrects the battery voltage the clock measures: the real voltage (from a LiPo tester\n"
     "or a meter) divided by what the Power and settings page shows.  1 = no correction.\n"
     "Example: the page says 4.13 V while the tester says 4.20 V, so 4.20 / 4.13 = 1.017.\n"
     "Or type  batcal 4.20  in the serial console: the clock works it out and keeps it."},
    {"low_battery_shutdown", "shut", T_BOOL, OFF(lowBatteryShutdown), 0, 0, 0, nullptr, false, "Sensors and battery",
     "on or off.  Shut down (deep sleep, with a message on the screen) before the battery\n"
     "is flat, so a LiPo is not ruined.  It wakes by itself once charging has brought the\n"
     "battery back, or when you press a button."},
    {"battery_cutoff_v", "cutoff", T_FLOAT, OFF(batteryCutoffV), 0, 3.10, 3.60, nullptr, false, "Sensors and battery",
     "Battery voltage, as measured while the clock runs, at which it shuts down:\n"
     "3.10 to 3.60 (default 3.30)."},
    // 25
    {"cpu_mhz", "cpu", T_ENUM, OFF(cpuSpeed), 0, 0, 0, kCpuNames, false, "Power",
     "80, 160 or 240.  80 uses the least power and is plenty for a clock."},
    {"cpu_idle_mhz", "cpuidle", T_ENUM, OFF(cpuIdle), 0, 0, 0, kCpuIdleNames, false, "Power",
     "off, 80, 40 or 20: the CPU clock while the radio is off (wifi = off, or between the syncs\n"
     "of wifi_mode = sync).  Slower draws less and a frame takes longer to draw: about 30 ms at\n"
     "80, 60 at 40, 120 at 20.  Try 40 first: it is the lowest clock the chip maker gives figures\n"
     "for.  The clock goes back to cpu_mhz while the radio is on, for a few seconds after a button\n"
     "press, and while a computer is on the USB port with the console running (the console may\n"
     "stop below 80 MHz).  off = no change."},
    {"console", "console", T_ENUM, OFF(console), 0, 0, 0, kConsoleNames, false, "Power",
     "on, auto or off: the USB serial console (the log and the commands).  On keeps it running.\n"
     "Auto runs it while a computer is on the USB port and shuts it down, with the USB\n"
     "transceiver, once there has been none for 10 seconds; it then stays off until the clock is\n"
     "restarted with the cable in.  Off shuts it down at the end of every start-up, computer or\n"
     "not.  A console that is shut down saves a little power, and the USB port is dead: to upload\n"
     "a firmware, restart the clock with KEY held down (the console then stays on for that run),\n"
     "or hold BOOT while switching it on, or use the SD card."},
    {"power_log", "plog", T_BOOL, OFF(powerLog), 0, 0, 0, nullptr, false, "Power",
     "on or off.  On: the clock writes its own power readings (the battery, the runtime estimate,\n"
     "the CPU clock, the radio's share of the time) into its flash as one line of CSV at each start\n"
     "and every 10 minutes, for comparing settings by how fast the battery goes down.  A start\n"
     "with an SD card in copies the log to the card as ESP32-S3-RLCD-PowerLog.csv.  The newest\n"
     "0.5 to 1 MB are kept (weeks).  The first time, the flash partition for it is formatted.\n"
     "See the README."},
    {"light_sleep", "lsleep", T_BOOL, OFF(lightSleep), 0, 0, 0, nullptr, false, "Power",
     "on or off.  On: between the frames, once a second, the processor goes into light sleep instead\n"
     "of idling, whenever nothing needs it: the radio off (wifi = off, or between the syncs of the\n"
     "sync mode), the console shut down (console = auto or off), no button pressed for 8 seconds.\n"
     "KEY and BOOT wake it at once.  While it sleeps its clock runs on an RC oscillator in the chip,\n"
     "less exact than the crystal: the hourly network time puts it right, and without WiFi nothing\n"
     "does.  Not yet tried on the board; see the README."},
    {"display_power", "dpower", T_ENUM, OFF(displayPower), 0, 0, 0, kDisplayPowerNames, false, "Power",
     "normal or low: how the display's controller drives the panel.  Low is its low power mode,\n"
     "which refreshes the panel 8 times a second instead of 25.5: less current (how much has not\n"
     "been measured), and a frame shows up 0 to 125 ms after it is sent instead of 0 to 39.  The\n"
     "clock allows for that, so the seconds change up to about 63 ms early or late instead of 20.\n"
     "Not yet tried on the board; see the README."},
    {"weather_interval_min", "wxint", T_INT, OFF(weatherIntervalMin), 0, 5, 240, nullptr, false, "Weather",
     "Minutes between weather updates, 5 to 240 (default 15)."},
};
const size_t kDefCount = sizeof kDefs / sizeof kDefs[0];
static_assert(sizeof kDefs / sizeof kDefs[0] <= 64, "userSet is a 64 bit mask");

// The bit of setting i in Settings::userSet.
inline uint64_t userBit(size_t i) { return (uint64_t)1 << i; }

struct Alias {
  const char *from;
  const char *to;
};
const Alias kAliases[] = {
    {"ssid", "wifi_ssid"},          {"network", "wifi_ssid"},        {"wifi_name", "wifi_ssid"},
    {"password", "wifi_password"},  {"pass", "wifi_password"},       {"wifi_pass", "wifi_password"},
    {"wifi_key", "wifi_password"},  {"lat", "latitude"},             {"lon", "longitude"},
    {"lng", "longitude"},           {"long", "longitude"},           {"tz", "timezone"},
    {"time_zone", "timezone"},      {"client_id", "spotify_client_id"}, {"spotify_id", "spotify_client_id"},
    {"unit", "units"},              {"temperature_units", "units"},  {"hour_format", "time_format"},
    {"clock_format", "time_format"}, {"label", "location_label"},    {"place", "location"},
    {"city", "location"},           {"wifi_enabled", "wifi"},        {"cpu", "cpu_mhz"},
    {"cpu_speed", "cpu_mhz"},       {"offset", "indoor_offset"},     {"temp_offset", "indoor_offset"},
    {"wifi_sync", "wifi_mode"},     {"radio_mode", "wifi_mode"},     {"cpu_idle", "cpu_idle_mhz"},
    {"idle_cpu", "cpu_idle_mhz"},   {"idle_clock", "cpu_idle_mhz"},
    {"serial", "console"},          {"serial_console", "console"},   {"usb_console", "console"},
    {"console_mode", "console"},    {"log", "console"},
    {"spotify_follow", "spotify_live"}, {"spotify_stay_connected", "spotify_live"},
    {"battery_log", "power_log"},   {"log_power", "power_log"},
    {"backup_ssid", "wifi_backup_ssid"},          {"backup_wifi", "wifi_backup_ssid"},
    {"backup_network", "wifi_backup_ssid"},       {"wifi_ssid2", "wifi_backup_ssid"},
    {"ssid2", "wifi_backup_ssid"},                {"second_ssid", "wifi_backup_ssid"},
    {"fallback_ssid", "wifi_backup_ssid"},        {"alt_ssid", "wifi_backup_ssid"},
    {"backup_password", "wifi_backup_password"},  {"backup_pass", "wifi_backup_password"},
    {"backup_wifi_password", "wifi_backup_password"}, {"wifi_password2", "wifi_backup_password"},
    {"password2", "wifi_backup_password"},        {"pass2", "wifi_backup_password"},
    {"second_password", "wifi_backup_password"},  {"fallback_password", "wifi_backup_password"},
    {"alt_password", "wifi_backup_password"},     {"backup_key", "wifi_backup_password"},
    {"battery_cal", "battery_calibration"},       {"adc_calibration", "battery_calibration"},
    {"voltage_calibration", "battery_calibration"}, {"battery_scale", "battery_calibration"},
    {"battery_gain", "battery_calibration"},      {"adc_scale", "battery_calibration"},
    {"battery_capacity", "battery_capacity_mah"}, {"capacity_mah", "battery_capacity_mah"},
};

// ---------------------------------------------------------------------------
// Small text helpers
// ---------------------------------------------------------------------------
char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }
bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f'; }

void trim(char *s) {
  char *start = s;
  while (isSpace(*start)) start++;
  size_t n = strlen(start);
  while (n > 0 && isSpace(start[n - 1])) n--;
  start[n] = 0;
  if (start != s) memmove(s, start, n + 1);
}

// Lower case with spaces, '-' and '_' removed: "Date-Format", "date format" and "dateformat"
// compare equal, and so do "Wi-Fi password" and "wifi_password".
void normalizeToken(const char *in, char *out, size_t cap) {
  size_t n = 0;
  for (; *in && n + 1 < cap; ++in) {
    if (isSpace(*in) || *in == '-' || *in == '_') continue;
    out[n++] = lower(*in);
  }
  out[n] = 0;
}

// Quote marks come in two families, so the apostrophe in "Bob's WiFi" does not close it.
enum QuoteKind { Q_NONE = 0, Q_DOUBLE, Q_SINGLE };

// Length in bytes of the quote mark at p (straight, curly or guillemet), or 0.
int quoteAt(const char *p, QuoteKind *kind) {
  const unsigned char c = (unsigned char)p[0];
  *kind = Q_NONE;
  if (c == '"') return *kind = Q_DOUBLE, 1;
  if (c == '\'') return *kind = Q_SINGLE, 1;
  if (c == 0xE2 && (unsigned char)p[1] == 0x80) {
    const unsigned char d = (unsigned char)p[2];
    if (d == 0x9C || d == 0x9D || d == 0x9E || d == 0x9F) return *kind = Q_DOUBLE, 3;  // “ ” „ ‟
    if (d == 0x98 || d == 0x99 || d == 0x9A || d == 0x9B) return *kind = Q_SINGLE, 3;  // ‘ ’ ‚ ‛
  }
  if (c == 0xC2 && ((unsigned char)p[1] == 0xAB || (unsigned char)p[1] == 0xBB)) return *kind = Q_DOUBLE, 2;  // « »
  return 0;
}

bool onlyCommentFollows(const char *p) {
  while (isSpace(*p)) p++;
  return *p == 0 || *p == '#' || *p == ';';
}

// Takes the text after "=".  A value in quotes loses them, and whatever follows the closing
// quote is a comment; a bare value loses a trailing " # comment".  Sets *quoted.
//
// The closing quote is the first mark of the same family that is followed by nothing or a
// comment, so  "My "Network""  and  "Home" # it's the "good" one  both come out right; with
// no such mark the last one on the line is taken.  A leading " without any closing quote is
// dropped (a forgotten quote); a lone ' is kept, it may be an apostrophe.
void unquote(char *v, bool *quoted) {
  trim(v);
  *quoted = false;
  QuoteKind kind;
  const int open = quoteAt(v, &kind);
  if (open) {
    char *inner = v + open;
    char *close = nullptr, *last = nullptr;
    for (char *p = inner; *p;) {
      QuoteKind k;
      const int l = quoteAt(p, &k);
      if (l && k == kind) {
        last = p;
        if (!close && onlyCommentFollows(p + l)) close = p;
      }
      p += l ? l : 1;
    }
    if (!close) close = last;
    if (close || kind == Q_DOUBLE) {
      *quoted = true;
      if (close) *close = 0;
      memmove(v, inner, strlen(inner) + 1);
      if (!close) trim(v);
      return;
    }
  }
  for (char *p = v; *p; ++p) {  // a comment needs a space before the '#'
    if (*p == '#' && p > v && isSpace(p[-1])) {
      *p = 0;
      break;
    }
  }
  trim(v);
}

bool parseBool(const char *v, bool *out) {
  char t[16];
  normalizeToken(v, t, sizeof t);
  static const char *const yes[] = {"on", "yes", "true", "1", "y", "enabled", "enable", "t", nullptr};
  static const char *const no[] = {"off", "no", "false", "0", "n", "disabled", "disable", "f", nullptr};
  for (int i = 0; yes[i]; i++)
    if (!strcmp(t, yes[i])) return *out = true, true;
  for (int i = 0; no[i]; i++)
    if (!strcmp(t, no[i])) return *out = false, true;
  return false;
}

bool parseEnum(const EnumName *names, const char *v, int *out) {
  char t[40], n[40];
  normalizeToken(v, t, sizeof t);
  for (const EnumName *e = names; e->name; ++e) {
    normalizeToken(e->name, n, sizeof n);
    if (!strcmp(t, n)) return *out = e->value, true;
  }
  return false;
}

// "12.5", "12,5", "-3 C", "80 MHz", "31.952 S".  `suffix` receives what follows the number.
bool parseNumber(const char *v, double *out, char *suffix, size_t suffixCap) {
  char buf[48];
  size_t n = 0;
  bool dot = strchr(v, '.') != nullptr;
  for (; v[n] && n + 1 < sizeof buf; n++) buf[n] = (v[n] == ',' && !dot) ? '.' : v[n];
  buf[n] = 0;
  char *end = nullptr;
  const double d = strtod(buf, &end);
  if (end == buf || !isfinite(d)) return false;
  while (isSpace(*end)) end++;
  if (suffix) snprintf(suffix, suffixCap, "%s", end);
  *out = d;
  return true;
}

// What may follow a number: nothing, or a unit such as "V", "min", "MHz", "C", "%" or a
// degree sign.  A second number ("3.3.0") is not a unit.
bool plainSuffix(const char *s) {
  if (!*s) return true;
  const unsigned char first = (unsigned char)*s;
  if (!(isalpha(first) || first == '%' || first == 0xC2)) return false;
  size_t n = 0;
  for (; *s; ++s, ++n) {
    const unsigned char c = (unsigned char)*s;
    if (!(isalpha(c) || c == '%' || c == '/' || c == 0xC2 || c == 0xB0 || c == ' ')) return false;
  }
  return n <= 14;
}

// What follows a coordinate: nothing, a degree mark ("°", "deg", "degrees") and then a compass
// letter or word for the right axis: "31.95", "31.95 S", "31.95° south", "115.86 E", "74 W".
// Returns -1 for south/west (the number becomes negative), +1 for plain or north/east, and 0
// when the text makes no sense for that axis ("33 E" as a latitude).
int hemisphereSign(const char *suffix, bool latitude) {
  char t[24];
  size_t n = 0;
  for (const char *p = suffix; *p && n + 1 < sizeof t; ++p) {
    const unsigned char c = (unsigned char)*p;
    if (isSpace((char)c) || c == 0xC2 || c == 0xB0) continue;  // blanks and the two bytes of "°"
    t[n++] = lower((char)c);
  }
  t[n] = 0;
  const char *rest = t;
  if (!strncmp(rest, "degrees", 7)) rest += 7;
  else if (!strncmp(rest, "degree", 6)) rest += 6;
  else if (!strncmp(rest, "deg", 3)) rest += 3;
  if (!*rest) return 1;
  if (latitude) {
    if (!strcmp(rest, "n") || !strcmp(rest, "north")) return 1;
    if (!strcmp(rest, "s") || !strcmp(rest, "south")) return -1;
  } else {
    if (!strcmp(rest, "e") || !strcmp(rest, "east")) return 1;
    if (!strcmp(rest, "w") || !strcmp(rest, "west")) return -1;
  }
  return 0;
}

// Finds a setting by its name or one of the aliases, ignoring case, spaces, '-' and '_'.
const Def *findDef(const char *key, size_t *index) {
  char k[48], t[48];
  normalizeToken(key, k, sizeof k);
  const char *wanted = k;
  char target[48];
  for (const Alias &a : kAliases) {
    normalizeToken(a.from, t, sizeof t);
    if (!strcmp(k, t)) {
      normalizeToken(a.to, target, sizeof target);
      wanted = target;
      break;
    }
  }
  for (size_t i = 0; i < kDefCount; i++) {
    normalizeToken(kDefs[i].key, t, sizeof t);
    if (!strcmp(wanted, t)) {
      if (index) *index = i;
      return &kDefs[i];
    }
  }
  return nullptr;
}

char *fieldPtr(Settings &s, const Def &d) { return reinterpret_cast<char *>(&s) + d.offset; }
const char *fieldPtr(const Settings &s, const Def &d) { return reinterpret_cast<const char *>(&s) + d.offset; }

int enumValue(const Settings &s, const Def &d) { return *reinterpret_cast<const uint8_t *>(fieldPtr(s, d)); }

const char *enumName(const Def &d, int value) {
  for (const EnumName *e = d.names; e->name; ++e)
    if (e->value == value) return e->name;
  return "?";
}

void enumChoices(const Def &d, char *out, size_t cap) {
  size_t n = 0;
  out[0] = 0;
  int last = -1;
  for (const EnumName *e = d.names; e->name && n + 24 < cap; ++e) {
    if (e->value == last) continue;  // one name per value
    last = e->value;
    n += (size_t)snprintf(out + n, cap - n, "%s%s", n ? ", " : "", e->name);
  }
}

// Hostnames, time servers and the like.
bool allChars(const char *s, const char *extra, bool allowDigits = true) {
  if (!*s) return false;
  for (; *s; ++s) {
    const unsigned char c = (unsigned char)*s;
    if (isalpha(c) || (allowDigits && isdigit(c)) || strchr(extra, c)) continue;
    return false;
  }
  return true;
}

bool printable(const char *s) {
  for (; *s; ++s)
    if ((unsigned char)*s < 0x20 || *s == 0x7F) return false;
  return true;
}

// POSIX rules such as AWST-8, <+08>-8 or CET-1CEST,M3.5.0,M10.5.0/3: only a sanity check.
bool plausiblePosixTz(const char *s) {
  const size_t n = strlen(s);
  if (n < 3 || n > 70) return false;
  if (!(isalpha((unsigned char)s[0]) || s[0] == '<')) return false;
  bool digit = false;
  for (const char *p = s; *p; ++p) {
    const unsigned char c = (unsigned char)*p;
    if (isdigit(c)) digit = true;
    if (!(isalnum(c) || strchr("<>+-:,./", c))) return false;
  }
  return digit;
}

bool validateString(const Def &d, char *value, char *err, size_t errCap) {
  auto fail = [&](const char *msg) {
    if (err) snprintf(err, errCap, "%s", msg);
    return false;
  };
  if (!printable(value)) return fail("contains a control character");
  if (strlen(value) >= d.size) {
    if (err) snprintf(err, errCap, "too long (at most %d characters)", (int)d.size - 1);
    return false;
  }
  const char *k = d.key;
  if (!value[0]) return strcmp(k, "hostname") != 0 || fail("cannot be empty");  // empty is "none" for the rest
  if (!strcmp(k, "hostname")) {
    const size_t n = strlen(value);
    if (!allChars(value, "-") || value[0] == '-' || value[n - 1] == '-') return fail("use letters, digits and '-' only");
  } else if (!strcmp(k, "ntp_server")) {
    if (!allChars(value, ".-:")) return fail("expected a host name or an address");
  } else if (!strcmp(k, "spotify_client_id")) {
    const size_t n = strlen(value);
    if (n < 16 || !allChars(value, "")) return fail("expected the 32 letters and digits of the Client ID");
  } else if (!strcmp(k, "timezone")) {
    char canon[48];
    if (tzFindIanaIgnoreCase(value, canon, sizeof canon)) {
      snprintf(value, d.size, "%s", canon);  // the canonical spelling
    } else if (!plausiblePosixTz(value)) {
      return fail("not a known zone: use e.g. Australia/Perth or a rule like AWST-8");
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Reading and writing one setting
// ---------------------------------------------------------------------------
bool setFromText(Settings &s, const Def &d, const char *text, char *err, size_t errCap) {
  auto fail = [&](const char *msg) {
    if (err) snprintf(err, errCap, "%s", msg);
    return false;
  };
  char *field = fieldPtr(s, d);
  switch (d.type) {
    case T_BOOL: {
      bool b;
      if (!parseBool(text, &b)) return fail("expected on or off");
      *reinterpret_cast<bool *>(field) = b;
      return true;
    }
    case T_ENUM: {
      int v;
      if (!parseEnum(d.names, text, &v)) {
        char choices[96];
        enumChoices(d, choices, sizeof choices);
        if (err) snprintf(err, errCap, "expected %s", choices);
        return false;
      }
      *reinterpret_cast<uint8_t *>(field) = (uint8_t)v;
      return true;
    }
    case T_INT:
    case T_FLOAT:
    case T_DOUBLE: {
      double v;
      char suffix[24];
      if (!parseNumber(text, &v, suffix, sizeof suffix)) return fail("expected a number");
      const bool lat = !strcmp(d.key, "latitude"), lon = !strcmp(d.key, "longitude");
      if (lat || lon) {
        const int sign = hemisphereSign(suffix, lat);
        if (sign == 0) return fail(lat ? "expected degrees, e.g. -31.95 or 31.95 S" : "expected degrees, e.g. 115.86 or 115.86 E");
        if (sign < 0 && v > 0) v = -v;
      } else if (!plainSuffix(suffix)) {
        return fail("expected a number");
      }
      if (v < d.lo || v > d.hi) {
        if (err) snprintf(err, errCap, "must be between %g and %g", d.lo, d.hi);
        return false;
      }
      if (d.type == T_INT) {
        *reinterpret_cast<int32_t *>(field) = (int32_t)lround(v);
      } else if (d.type == T_FLOAT) {
        *reinterpret_cast<float *>(field) = (float)v;
      } else {
        *reinterpret_cast<double *>(field) = v;
      }
      return true;
    }
    case T_STRING: {
      char value[96];
      snprintf(value, sizeof value, "%s", text);
      if (strlen(text) >= sizeof value) return fail("too long");
      if (!validateString(d, value, err, errCap)) return false;
      snprintf(field, d.size, "%s", value);
      return true;
    }
  }
  return false;
}

bool formatValue(const Settings &s, const Def &d, char *out, size_t cap) {
  const char *field = fieldPtr(s, d);
  int n = 0;
  switch (d.type) {
    case T_BOOL: n = snprintf(out, cap, "%s", *reinterpret_cast<const bool *>(field) ? "on" : "off"); break;
    case T_ENUM: n = snprintf(out, cap, "%s", enumName(d, enumValue(s, d))); break;
    case T_INT: n = snprintf(out, cap, "%d", (int)*reinterpret_cast<const int32_t *>(field)); break;
    case T_FLOAT: n = snprintf(out, cap, "%g", (double)*reinterpret_cast<const float *>(field)); break;
    case T_DOUBLE: n = snprintf(out, cap, "%.8g", *reinterpret_cast<const double *>(field)); break;
    case T_STRING: n = snprintf(out, cap, "%s", field); break;
  }
  return n >= 0 && (size_t)n < cap;
}

void restoreDefault(Settings &s, const Settings &defaults, const Def &d, size_t i) {
  const size_t len = (d.type == T_STRING) ? d.size
                     : d.type == T_BOOL   ? sizeof(bool)
                     : d.type == T_ENUM   ? sizeof(uint8_t)
                     : d.type == T_INT    ? sizeof(int32_t)
                     : d.type == T_FLOAT  ? sizeof(float)
                                          : sizeof(double);
  memcpy(fieldPtr(s, d), fieldPtr(defaults, d), len);
  s.userSet &= ~userBit(i);
}

ApplyResult applyParsed(Settings &s, const Settings &defaults, const char *key, const char *raw, bool quoted,
                        char *err, size_t errCap) {
  size_t i = 0;
  const Def *d = findDef(key, &i);
  if (!d) {
    if (err) snprintf(err, errCap, "unknown setting '%.30s'", key);
    return ApplyResult::UnknownKey;
  }
  char before[100];
  formatValue(s, *d, before, sizeof before);
  const bool wasSet = (s.userSet & userBit(i)) != 0;

  if (!raw[0] && !quoted) {  // "key =" forgets the saved setting
    restoreDefault(s, defaults, *d, i);
    return wasSet ? ApplyResult::Cleared : ApplyResult::Unchanged;
  }
  Settings trial = s;
  char msg[80] = "";
  if (!setFromText(trial, *d, raw, msg, sizeof msg)) {
    if (err) snprintf(err, errCap, "'%s': %s", d->key, msg);
    return ApplyResult::BadValue;
  }
  s = trial;
  s.userSet |= userBit(i);
  char after[100];
  formatValue(s, *d, after, sizeof after);
  return strcmp(before, after) != 0 ? ApplyResult::Changed : ApplyResult::Unchanged;
}

}  // namespace

// ---------------------------------------------------------------------------
// Table access
// ---------------------------------------------------------------------------
size_t settingCount() { return kDefCount; }
const char *settingKey(size_t i) { return i < kDefCount ? kDefs[i].key : ""; }
const char *settingNvsKey(size_t i) { return i < kDefCount ? kDefs[i].nvs : ""; }
bool settingIsSecret(size_t i) { return i < kDefCount && kDefs[i].secret; }
bool settingIsUserSet(const Settings &s, size_t i) { return i < kDefCount && (s.userSet & userBit(i)) != 0; }

bool formatSetting(const Settings &s, size_t i, char *out, size_t cap) {
  return i < kDefCount && formatValue(s, kDefs[i], out, cap);
}

bool applyStoredSetting(Settings &s, size_t i, const char *value, char *err, size_t errCap) {
  if (err && errCap) err[0] = 0;
  if (i >= kDefCount || !value) return false;
  if (!value[0] && kDefs[i].type != T_STRING) {  // an empty text setting is a real choice
    if (err) snprintf(err, errCap, "empty");
    return false;
  }
  Settings trial = s;
  if (!setFromText(trial, kDefs[i], value, err, errCap)) return false;
  s = trial;
  s.userSet |= userBit(i);
  return true;
}

ApplyResult applySetting(Settings &s, const Settings &defaults, const char *key, const char *value, char *err,
                         size_t errCap) {
  char buf[160];
  snprintf(buf, sizeof buf, "%s", value ? value : "");
  bool quoted;
  unquote(buf, &quoted);
  return applyParsed(s, defaults, key, buf, quoted, err, errCap);
}

// ---------------------------------------------------------------------------
// The file
// ---------------------------------------------------------------------------
namespace {

void addIssue(ConfigReport &r, int line, const char *text) {
  if (r.issueCount >= kConfigMaxIssues) return;
  ConfigIssue &is = r.issues[r.issueCount++];
  is.line = line;
  snprintf(is.text, sizeof is.text, "%s", text);
}

const size_t kMaxLine = 300;

// Calls fn(line, lineNo, tooLong) for every line of the text (LF, CRLF or CR ends; a last
// line without one counts too).  A line longer than kMaxLine arrives cut off, flagged.
template <class F>
int eachLine(const char *buf, size_t n, F fn) {
  int lineNo = 0;
  size_t pos = 0;
  while (pos < n) {
    size_t e = pos;
    while (e < n && buf[e] != '\n' && buf[e] != '\r') e++;
    char line[kMaxLine + 1];
    const size_t len = e - pos;
    const size_t take = len > kMaxLine ? kMaxLine : len;
    memcpy(line, buf + pos, take);
    line[take] = 0;
    fn(line, ++lineNo, len > kMaxLine);
    pos = e;
    if (pos < n) {
      if (buf[pos] == '\r' && pos + 1 < n && buf[pos + 1] == '\n') pos++;
      pos++;
    }
  }
  return lineNo;
}

// What a line is: nothing to do, a "key = value" (split in place), or neither.
enum class LineKind { Skip, Pair, Broken };

LineKind splitLine(char *line, char **key, char **value) {
  trim(line);
  if (!line[0] || line[0] == '#' || line[0] == ';' || (line[0] == '/' && line[1] == '/')) return LineKind::Skip;
  if (line[0] == '[' && line[strlen(line) - 1] == ']') return LineKind::Skip;  // an INI [section] heading
  char *sep = line;
  while (*sep && *sep != '=' && *sep != ':') sep++;
  if (!*sep) return LineKind::Broken;
  *sep = 0;
  trim(line);
  *key = line;
  *value = sep + 1;
  return LineKind::Pair;
}

bool isResetAll(const char *key) {
  char t[48];
  normalizeToken(key, t, sizeof t);
  return !strcmp(t, "resetall");
}

void processLine(Settings &s, const Settings &defaults, char *line, int lineNo, ConfigReport &r) {
  char *key = nullptr, *value = nullptr;
  char msg[144];  // "line 99999: " and the longest error text; addIssue cuts it to the 96 the screen keeps
  const LineKind kind = splitLine(line, &key, &value);
  if (kind == LineKind::Skip) return;
  if (kind == LineKind::Broken) {
    snprintf(msg, sizeof msg, "line %d: no '=' found", lineNo);
    r.bad++;
    addIssue(r, lineNo, msg);
    return;
  }
  bool quoted;
  unquote(value, &quoted);

  if (isResetAll(key)) {  // already acted on before the first line was read
    bool b;
    if (!parseBool(value, &b)) {
      snprintf(msg, sizeof msg, "line %d: reset_all: expected on or off", lineNo);
      r.bad++;
      addIssue(r, lineNo, msg);
    }
    return;
  }

  char err[80] = "";
  switch (applyParsed(s, defaults, key, value, quoted, err, sizeof err)) {
    case ApplyResult::Changed: r.changed++; break;
    case ApplyResult::Unchanged: r.unchanged++; break;
    case ApplyResult::Cleared: r.cleared++; break;
    case ApplyResult::UnknownKey:
      r.unknown++;
      snprintf(msg, sizeof msg, "line %d: %s", lineNo, err);
      addIssue(r, lineNo, msg);
      break;
    case ApplyResult::BadValue:
      r.bad++;
      snprintf(msg, sizeof msg, "line %d: %s", lineNo, err);
      addIssue(r, lineNo, msg);
      break;
  }
}

// UTF-16 (what Notepad calls "Unicode") and UTF-8 with a byte-order mark to plain UTF-8, so
// such a file still reads.  Returns a malloc'd buffer; NUL bytes inside become '?'.
char *decodeText(const char *text, size_t len, size_t *outLen) {
  const unsigned char *b = (const unsigned char *)text;
  const bool utf16 = len >= 2 && ((b[0] == 0xFF && b[1] == 0xFE) || (b[0] == 0xFE && b[1] == 0xFF));
  char *buf = (char *)malloc(utf16 ? len + len / 2 + 4 : len + 1);  // a UTF-16 unit makes at most 3 bytes
  if (!buf) return nullptr;
  size_t n = 0;
  if (utf16) {
    const bool le = b[0] == 0xFF;
    for (size_t i = 2; i + 1 < len; i += 2) {
      const uint32_t u = le ? (uint32_t)(b[i] | (b[i + 1] << 8)) : (uint32_t)((b[i] << 8) | b[i + 1]);
      if (u == 0) {
        buf[n++] = '?';
      } else if (u < 0x80) {
        buf[n++] = (char)u;
      } else if (u < 0x800) {
        buf[n++] = (char)(0xC0 | (u >> 6));
        buf[n++] = (char)(0x80 | (u & 0x3F));
      } else if (u >= 0xD800 && u < 0xE000) {
        buf[n++] = '?';  // half of a surrogate pair
      } else {
        buf[n++] = (char)(0xE0 | (u >> 12));
        buf[n++] = (char)(0x80 | ((u >> 6) & 0x3F));
        buf[n++] = (char)(0x80 | (u & 0x3F));
      }
    }
  } else {
    const size_t start = (len >= 3 && b[0] == 0xEF && b[1] == 0xBB && b[2] == 0xBF) ? 3 : 0;
    for (size_t i = start; i < len; i++) buf[n++] = text[i] ? text[i] : '?';
  }
  buf[n] = 0;
  *outLen = n;
  return buf;
}

}  // namespace

ConfigReport applyConfigText(Settings &s, const Settings &defaults, const char *text, size_t len) {
  ConfigReport r;
  size_t n = 0;
  char *buf = text ? decodeText(text, len, &n) : nullptr;
  if (!buf) return r;

  // "reset_all = yes" acts first wherever it stands, so the rest of the file applies on top
  eachLine(buf, n, [&](char *line, int, bool) {
    char *key = nullptr, *value = nullptr;
    if (splitLine(line, &key, &value) != LineKind::Pair || !isResetAll(key)) return;
    bool quoted, b;
    unquote(value, &quoted);
    if (parseBool(value, &b) && b) {
      s = defaults;
      s.userSet = 0;
      r.resetAll = true;
    }
  });

  r.lines = eachLine(buf, n, [&](char *line, int lineNo, bool tooLong) {
    if (tooLong) {
      char msg[40];
      snprintf(msg, sizeof msg, "line %d: line too long", lineNo);
      r.bad++;
      addIssue(r, lineNo, msg);
      return;
    }
    processLine(s, defaults, line, lineNo, r);
  });
  free(buf);
  return r;
}

void formatConfigSummary(const ConfigReport &r, char *out, size_t cap) {
  if (cap == 0) return;
  size_t n = 0;
  out[0] = 0;
  auto append = [&](const char *sep, const char *text) {  // never past the end of `out`
    const int w = snprintf(out + n, cap - n, "%s%s", sep, text);
    if (w > 0) n = (n + (size_t)w < cap) ? n + (size_t)w : cap - 1;
  };
  auto add = [&](int count, const char *what) {
    if (count <= 0) return;
    char item[32];
    snprintf(item, sizeof item, "%d %s", count, what);
    append(n ? ", " : "", item);
  };
  if (r.resetAll) append("", "all forgotten");
  add(r.changed, "changed");
  add(r.cleared, "forgotten");
  add(r.unchanged, "unchanged");
  add(r.problems(), r.problems() == 1 ? "problem" : "problems");
  if (n == 0) snprintf(out, cap, "nothing to apply");
}

// ---------------------------------------------------------------------------
// The example file
// ---------------------------------------------------------------------------
namespace {

class Out {
 public:
  Out(char *buf, size_t cap, bool crlf) : buf_(buf), cap_(cap), crlf_(crlf) {
    if (cap_) buf_[0] = 0;
  }
  void add(const char *s) {
    for (; *s; ++s) {
      if (*s == '\n' && crlf_) put('\r');
      put(*s);
    }
  }
  void comment(const char *text) {  // each '\n' separated line behind "# "
    const char *p = text;
    while (p && *p) {
      const char *nl = strchr(p, '\n');
      const size_t l = nl ? (size_t)(nl - p) : strlen(p);
      if (l == 0) {
        add("#\n");  // a blank line inside the comment, without a trailing space
      } else {
        add("# ");
        char tmp[120];
        snprintf(tmp, sizeof tmp, "%.*s", (int)l, p);
        add(tmp);
        add("\n");
      }
      p = nl ? nl + 1 : nullptr;
    }
  }
  size_t size() const { return overflow_ ? 0 : n_; }

 private:
  void put(char c) {
    if (n_ + 2 > cap_) {  // room for this character and the terminator
      overflow_ = true;
      return;
    }
    buf_[n_++] = c;
    buf_[n_] = 0;
  }

  char *buf_;
  size_t cap_;
  bool crlf_;
  size_t n_ = 0;
  bool overflow_ = false;
};

}  // namespace

size_t renderExampleConfig(const Settings &cur, const char *driftNote, char *out, size_t cap, bool crlf) {
  Out o(out, cap, crlf);
  o.comment(
      "ESP32-S3 RLCD clock: settings file\n"
      "\n"
      "The clock found no settings file on this card, so it wrote this one.  Every setting\n"
      "below is commented out (it starts with #) and shows the value the clock uses now.\n"
      "To change one: remove the # in front of its line, edit the value, save the file,\n"
      "put the card back in the clock and restart it.  The clock saves what it reads in its\n"
      "own flash, so the card can be taken out again afterwards.\n"
      "\n"
      "The rules\n"
      "  * One \"name = value\" per line.  Spaces around the = do not matter.\n"
      "  * Values can be in quotes or not: \"My Network\" and My Network mean the same.\n"
      "    Use quotes if a value contains a # or begins or ends with a space.\n"
      "  * Names ignore case, spaces, - and _: wifi_ssid, WiFi SSID and Wi-Fi-SSID are one setting.\n"
      "  * Lines starting with # or ; are comments.\n"
      "  * A setting you leave out keeps whatever the clock has now.\n"
      "  * A line with nothing after the = (for example \"timezone =\") makes the clock forget\n"
      "    that saved setting and go back to its built-in default.\n"
      "  * reset_all = yes forgets every saved setting first; remove that line afterwards.\n"
      "  * What the clock does not understand is ignored and listed on its Power and settings\n"
      "    page (the second info page).\n"
      "\n"
      "WiFi passwords are plain text in this file and in the clock's flash: keep the card\n"
      "to yourself.");

  const char *lastGroup = "";
  for (size_t i = 0; i < kDefCount; i++) {
    const Def &d = kDefs[i];
    if (strcmp(d.group, lastGroup) != 0) {
      lastGroup = d.group;
      o.add("\n# ---- ");
      o.add(d.group);
      o.add(" ");
      for (size_t k = strlen(d.group); k < 60; k++) o.add("-");
      o.add("\n");
    }
    if (d.help) {
      o.add("#\n");
      o.comment(d.help);
    }
    char value[100] = "";
    if (d.secret) {
      snprintf(value, sizeof value, "%s", strstr(d.key, "password") ? "\"your password\"" : "\"your client id\"");
    } else if (d.type == T_STRING) {
      char raw[90];
      formatValue(cur, d, raw, sizeof raw);
      snprintf(value, sizeof value, "\"%s\"", raw);
    } else {
      formatValue(cur, d, value, sizeof value);
    }
    o.add("# ");
    o.add(d.key);
    o.add(" = ");
    o.add(value);
    o.add("\n");
  }

  o.add("\n# ---- Clock drift when WiFi is off ");
  for (size_t k = 33; k < 66; k++) o.add("-");
  o.add("\n#\n");
  o.comment(
      "With wifi = off the clock cannot ask the network for the time.  It starts from the\n"
      "time kept by its real-time clock chip and then counts with its own crystal, which\n"
      "is good for a second or two a day (10 ppm is about 0.9 seconds a day) and depends on\n"
      "temperature.  Over weeks that adds up to a visible error; nothing corrects it until\n"
      "WiFi has been on for a few minutes again.  If you leave WiFi off, switch it on now\n"
      "and then to let the clock set itself.");
  if (driftNote && driftNote[0]) {
    o.add("#\n");
    o.comment(driftNote);
  }
  return o.size();
}
