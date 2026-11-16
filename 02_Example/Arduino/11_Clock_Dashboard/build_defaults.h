#pragma once

// The settings as this build ships them: the defaults in config.h, each of which secrets.h can replace,
// turned into a Settings.  They are the bottom layer, under what the clock saved in its flash and
// under the SD card file.
//
// Every setting the card can set has a macro here (settings.cpp has the list, and tools/tests fails
// if one is missing), so a build can be pre-programmed with anything the card could say.  A value goes
// through the same checks as the card's, in the same words ("12h", "max", "d-mon-y"), with the same
// limits.  One that does not pass leaves the factory default in place and is reported, so a typo in
// secrets.h shows up on the Info page instead of being used.
//
// No Arduino dependency: app_settings.cpp uses it in the sketch and tools/tests on a PC.

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "settings.h"

// What was wrong with the build's defaults: how many, and the first few as text for the screen.
struct BuildReport {
  static const int kKept = 3;
  int problems = 0;
  int kept = 0;
  char text[kKept][128] = {};  // "MACRO_NAME: the reason" (a name is 23 characters at most, a reason 79)
};

namespace builddef {

inline bool isPlaceholderSsid(const char *s) { return !strcmp(s, "YourWiFiName") || !strcmp(s, "YourBackupWiFiName"); }

// One setting, by the name it has in the settings file, from the text of its macro.
inline void put(Settings &d, BuildReport *rep, const char *macro, const char *key, const char *text) {
  size_t i = 0;
  while (i < settingCount() && strcmp(settingKey(i), key) != 0) i++;
  char err[80] = "no such setting";
  if (i < settingCount() && applyStoredSetting(d, i, text, err, sizeof err)) return;
  if (!rep) return;
  rep->problems++;
  if (rep->kept < BuildReport::kKept) snprintf(rep->text[rep->kept++], sizeof rep->text[0], "%s: %s", macro, err);
}

inline void putBool(Settings &d, BuildReport *rep, const char *macro, const char *key, bool v) {
  put(d, rep, macro, key, v ? "on" : "off");
}

inline void putInt(Settings &d, BuildReport *rep, const char *macro, const char *key, long v) {
  char t[24];
  snprintf(t, sizeof t, "%ld", v);
  put(d, rep, macro, key, t);
}

// Six digits for a float: 3.10f is 3.0999999 as a double, which a limit of "3.10 and up" would refuse.
inline void putFloat(Settings &d, BuildReport *rep, const char *macro, const char *key, double v) {
  char t[32];
  snprintf(t, sizeof t, "%.6g", v);
  put(d, rep, macro, key, t);
}

inline void putDouble(Settings &d, BuildReport *rep, const char *macro, const char *key, double v) {
  char t[32];
  snprintf(t, sizeof t, "%.8g", v);
  put(d, rep, macro, key, t);
}

}  // namespace builddef

inline Settings buildDefaults(BuildReport *rep = nullptr) {
  using namespace builddef;
  Settings d;

  // WiFi
  putBool(d, rep, "WIFI_ENABLED", "wifi", WIFI_ENABLED);
  const bool noMain = isPlaceholderSsid(WIFI_SSID);  // the example file's name: no network yet
  put(d, rep, "WIFI_SSID", "wifi_ssid", noMain ? "" : WIFI_SSID);
  put(d, rep, "WIFI_PASSWORD", "wifi_password", noMain ? "" : WIFI_PASSWORD);
  const bool noBackup = isPlaceholderSsid(WIFI_SSID_BACKUP);
  put(d, rep, "WIFI_SSID_BACKUP", "wifi_backup_ssid", noBackup ? "" : WIFI_SSID_BACKUP);
  put(d, rep, "WIFI_PASSWORD_BACKUP", "wifi_backup_password", noBackup ? "" : WIFI_PASSWORD_BACKUP);
  put(d, rep, "APP_HOSTNAME", "hostname", APP_HOSTNAME);
  put(d, rep, "NTP_SERVER", "ntp_server", NTP_SERVER);
  put(d, rep, "WIFI_POWER_SAVE", "wifi_power_save", WIFI_POWER_SAVE);
  put(d, rep, "WIFI_MODE", "wifi_mode", WIFI_MODE);

  // Units and formats
  put(d, rep, "USE_FAHRENHEIT", "units", USE_FAHRENHEIT ? "imperial" : "metric");
  put(d, rep, "TIME_FORMAT", "time_format", TIME_FORMAT);
  put(d, rep, "DATE_FORMAT", "date_format", DATE_FORMAT);
  putBool(d, rep, "SHOW_WEEK", "show_week", SHOW_WEEK);

  // Location
  putDouble(d, rep, "LOCATION_LATITUDE", "latitude", LOCATION_LATITUDE);
  putDouble(d, rep, "LOCATION_LONGITUDE", "longitude", LOCATION_LONGITUDE);
  put(d, rep, "LOCATION_LABEL", "location_label", LOCATION_LABEL);
  put(d, rep, "LOCATION_QUERY", "location", LOCATION_QUERY);
  put(d, rep, "TIMEZONE_POSIX_OVERRIDE", "timezone", TIMEZONE_POSIX_OVERRIDE);

  // Spotify
  putBool(d, rep, "SPOTIFY_ENABLED", "spotify", SPOTIFY_ENABLED);
  put(d, rep, "SPOTIFY_CLIENT_ID", "spotify_client_id", SPOTIFY_CLIENT_ID);
  putBool(d, rep, "SPOTIFY_LIVE", "spotify_live", SPOTIFY_LIVE);

  // Sensors and battery
  putFloat(d, rep, "INDOOR_TEMP_OFFSET_C", "indoor_offset", INDOOR_TEMP_OFFSET_C);
  put(d, rep, "BATTERY_MODE", "battery", BATTERY_MODE);
  putInt(d, rep, "BATTERY_CAPACITY_MAH", "battery_capacity_mah", BATTERY_CAPACITY_MAH);
  putFloat(d, rep, "BATTERY_CALIBRATION", "battery_calibration", BATTERY_CALIBRATION);
  putBool(d, rep, "LOW_BATTERY_SHUTDOWN", "low_battery_shutdown", LOW_BATTERY_SHUTDOWN);
  putFloat(d, rep, "BATTERY_CUTOFF_V", "battery_cutoff_v", BATTERY_CUTOFF_V);

  // Power
  putInt(d, rep, "CPU_MHZ", "cpu_mhz", CPU_MHZ);
  putInt(d, rep, "CPU_IDLE_MHZ", "cpu_idle_mhz", CPU_IDLE_MHZ);
  put(d, rep, "CONSOLE_MODE", "console", CONSOLE_MODE);
  putBool(d, rep, "POWER_LOG", "power_log", POWER_LOG);
  putBool(d, rep, "LIGHT_SLEEP", "light_sleep", LIGHT_SLEEP);
  put(d, rep, "DISPLAY_POWER", "display_power", DISPLAY_POWER);

  // Weather
  putInt(d, rep, "WEATHER_INTERVAL_MIN", "weather_interval_min", WEATHER_INTERVAL_MIN);

  d.userSet = 0;  // these are the defaults, not something the user chose
  return d;
}
