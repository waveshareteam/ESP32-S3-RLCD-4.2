#pragma once

// Copy this file to "secrets.h" and fill it in.
// secrets.h is listed in .gitignore, so your passwords stay out of git.
//
// Everything here can also be set from a settings file on an SD card (see the README), which
// is handier once the clock is built: no reflashing to change the WiFi, the units or the place.
// And the other way round: every setting the card can set can be built in here, to come pre-programmed.
// The names are in the last column of the README's settings table ("Building the settings in"); the
// ones below are the ones people usually want.  config.h has the factory value and the choices of each.
// Text and choices are written as on the card ("12h", "max"), yes / no as 1 / 0.  A value that does
// not pass the card's own checks is left out and reported on the Info page.

// WiFi (2.4 GHz; the ESP32-S3 does not do 5 GHz)
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASSWORD "YourWiFiPassword"

// A backup network (optional): tried when the one above cannot be joined, say a phone's hotspot.
// While the clock is on the backup it looks for the main network every 10 minutes and goes back.
// #define WIFI_SSID_BACKUP "YourBackupWiFiName"
// #define WIFI_PASSWORD_BACKUP "YourBackupWiFiPassword"

// Spotify (optional).  Paste the Client ID of your own app from
// https://developer.spotify.com/dashboard to turn on now-playing and the
// KEY button controls.  Leave it empty to run without Spotify.
// The README walks through creating the app and linking your account.
#define SPOTIFY_CLIENT_ID ""

// Your battery (optional): its capacity in mAh, so the Power and settings page can show the average current,
// and a correction for the voltage the clock measures (real voltage / shown voltage, see the README).
// #define BATTERY_CAPACITY_MAH 2500
// #define BATTERY_CALIBRATION 1.016f
// #define BATTERY_MODE "none"            // "auto", or "none" when no battery is fitted (USB only)
// #define LOW_BATTERY_SHUTDOWN 1         // 1 = shut down before the cell is flat, 0 = never
// #define BATTERY_CUTOFF_V 3.30f         // ... at this voltage, 3.10 to 3.60

// Where the clock is (optional, for the weather and the time zone).  Give coordinates, or a place
// to look up; see config.h for the details.  Without either the clock still tells the time.
// #define LOCATION_LATITUDE (-33.865)
// #define LOCATION_LONGITUDE 151.209
// #define LOCATION_QUERY "Sydney"
// #define LOCATION_LABEL "Home"      // what the status bar shows
// #define TIMEZONE_POSIX_OVERRIDE "Australia/Sydney"  // an IANA name or a POSIX rule; needed with WiFi off

// Units and formats (optional)
// #define USE_FAHRENHEIT 1               // 1 = degF and mph
// #define TIME_FORMAT "12h"              // "24h" or "12h"
// #define DATE_FORMAT "d-mon-y"          // iso, dmy, mdy, dmy-dot, d-mon-y or mon-d-y
// #define SHOW_WEEK 0                    // 0 = no week number and day of the year

// WiFi and power (optional)
// #define WIFI_ENABLED 0                 // 0 = radio off: no network time, weather or Spotify, much less power
// #define WIFI_POWER_SAVE "max"          // "normal" or "max": the radio sleeps longer (replies can come a third of a second late)
// #define WIFI_MODE "sync"               // "always", or "sync": the radio is on only for the syncs, key presses and Spotify (README: Saving power)
// #define APP_HOSTNAME "desk-clock"      // the clock's name on the network
// #define NTP_SERVER "time.nist.gov"     // a time server to try first
// #define CPU_MHZ 80                     // 80, 160 or 240
// #define CPU_IDLE_MHZ 40                // the clock while the radio is off: 0 (= CPU_MHZ), 80, 40 or 20 (untried: see config.h)
// #define CONSOLE_MODE "auto"            // the USB serial console: "on", "auto" (only while a computer is on the USB port) or "off" (README: Saving power)
// #define POWER_LOG 1                    // 1 = a line of power readings in the clock's flash every 10 minutes, copied to an SD card at start-up (README: Logging a battery run)
// #define LIGHT_SLEEP 1                  // 1 = light sleep between the frames while the radio and the console are off (untried: README: Light sleep)
// #define DISPLAY_POWER "low"            // the display refreshed 8 times a second instead of 25.5, for less current (untried: README: The display's low power mode)
// #define SPOTIFY_ENABLED 0              // 0 = off, even with a Client ID
// #define SPOTIFY_LIVE 0                 // with WIFI_MODE "sync": 0 = the radio sleeps while music plays, one look a track (1 = it stays on)

// Other (optional)
// #define INDOOR_TEMP_OFFSET_C (-4.0f)   // degrees C added to the indoor temperature
// #define WEATHER_INTERVAL_MIN 15        // minutes between weather updates, 5 to 240
