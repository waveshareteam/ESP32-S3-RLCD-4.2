#pragma once

// What a secrets.h would say to pre-program every setting: one #define for each, with the values of
// all_settings.h.  test_builddefaults.cpp (built with -DTEST_ALL -include override_all.h) checks
// that the settings it ends up with are exactly what the card file of all_settings.h gives, and
// that each macro is documented in the README and in secrets.example.h.
//
// A macro that config.h forgot to guard with #ifndef is a "redefined" error here (-Werror).

#define WIFI_ENABLED 0
#define WIFI_SSID "Cafe Net"
#define WIFI_PASSWORD "p@ss w0rd"
#define WIFI_SSID_BACKUP "Phone Hotspot"
#define WIFI_PASSWORD_BACKUP "h0tsp0t pw"
#define APP_HOSTNAME "my-clock"
#define NTP_SERVER "time.nist.gov"
#define WIFI_POWER_SAVE "max"
#define WIFI_MODE "sync"
#define USE_FAHRENHEIT 1
#define TIME_FORMAT "12h"
#define DATE_FORMAT "mdy"
#define SHOW_WEEK 0
#define LOCATION_LATITUDE 40.7128
#define LOCATION_LONGITUDE (-74.0060)
#define LOCATION_LABEL "New York"
#define LOCATION_QUERY "New York"
#define TIMEZONE_POSIX_OVERRIDE "America/New_York"
#define SPOTIFY_ENABLED 0
#define SPOTIFY_CLIENT_ID "0123456789abcdef0123456789abcdef"
#define SPOTIFY_LIVE 0
#define INDOOR_TEMP_OFFSET_C (-2.5f)
#define BATTERY_MODE "none"
#define BATTERY_CAPACITY_MAH 2500
#define BATTERY_CALIBRATION 1.0157f
#define LOW_BATTERY_SHUTDOWN 0
#define BATTERY_CUTOFF_V 3.45f
#define CPU_MHZ 160
#define CPU_IDLE_MHZ 40
#define CONSOLE_MODE "auto"
#define POWER_LOG 1
#define LIGHT_SLEEP 1
#define DISPLAY_POWER "low"
#define WEATHER_INTERVAL_MIN 30
