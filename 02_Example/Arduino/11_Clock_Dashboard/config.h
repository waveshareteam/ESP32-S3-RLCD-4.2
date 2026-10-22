#pragma once

// ============================================================================
//  Built-in defaults.
//
//  Most of what is here is only the factory default.  The clock also reads a settings file from a
//  FAT32 SD card (README: "Settings from an SD card") and keeps what it finds in its own flash, so
//  the card can come out again afterwards.  Later wins:
//
//      this file and secrets.h   <   saved in flash   <   the SD card file
//
//  Every setting the card can set has a default here, so a build can come pre-programmed: define it
//  in secrets.h (which git ignores, so it can hold credentials too) and it replaces the default below.
//  The values are checked by the same rules as the card's, in the same words ("12h", "max").
//  README: "Building the settings in".
// ============================================================================

#define APP_NAME "RLCD Clock"
// Not semver: 1.0 was the first release, and the number goes up by hand with every batch of changes that is
// handed over to run on a board (README: "Versions").  Two builds of one version are told apart by the build
// id next to it on the System info page, which changes whenever the program does.
#define APP_VERSION "1.12"

// The settings file the clock looks for in the root of the SD card, and writes (with every
// setting explained) when the card has none.  Upper or lower case, it is FAT.
#define CONFIG_FILE_NAME "ESP32-S3-RLCD-Config.txt"
#define CONFIG_FILE_NAME_ALT "ESP32-S31-RLCD-Config.txt"  // accepted too, in case of a typo

// The firmware file the clock installs from the root of the SD card (README: "Updating the firmware from
// the SD card"): the app image the Arduino IDE exports with Sketch > Export Compiled Binary, or a copy of
// it under the first name.  With more than one of these on the card the clock does nothing.  After an
// install the file gets FIRMWARE_DONE_SUFFIX appended to its name.
#define FIRMWARE_FILE_NAME "ESP32-S3-RLCD-Firmware.bin"
#define FIRMWARE_FILE_NAME_ALT "ESP32-S31-RLCD-Firmware.bin"  // accepted too, in case of a typo
#define FIRMWARE_FILE_EXPORT "11_Clock_Dashboard.ino.bin"     // what the Arduino IDE calls it
#define FIRMWARE_DONE_SUFFIX ".done"
// A freshly installed firmware is on trial: unless it has run this long, a reset puts the old one back.
#define FIRMWARE_TRIAL_MS 60000UL

// The copy of the power log that the clock leaves in the root of an SD card which is in the slot while it
// starts (README: "Logging a battery run"); a file of that name on the card is replaced.
#define POWER_LOG_FILE_NAME "ESP32-S3-RLCD-PowerLog.csv"

// secrets.h is optional: copy secrets.example.h to secrets.h to build your WiFi name and password in.
// Without it the clock starts with no network and takes them from the SD card settings file.
// (The host tests define CONFIG_NO_SECRETS, so that a secrets.h lying around cannot get into them.)
#if __has_include("secrets.h") && !defined(CONFIG_NO_SECRETS)
#include "secrets.h"
#endif

// ----------------------------------------------------------------------------
// Location: used for the weather and to work out the time zone.
//
// The device asks Open-Meteo for the IANA zone name of these coordinates
// (e.g. "Australia/Perth") and turns it into a POSIX TZ rule, so daylight
// saving changes are handled on the device without further network calls.
//
//   - Exact coordinates win.  Any maps app will give you these.
//   - Otherwise LOCATION_QUERY is looked up once (Open-Meteo geocoder) and the
//     result is remembered: "Perth", "Paris, France", ...
//   - LOCATION_LABEL is what the status bar shows (empty = the resolved name).
//
// Put your own in secrets.h (#define LOCATION_LATITUDE ...) or in the SD card settings file.
// ----------------------------------------------------------------------------
#ifndef LOCATION_LATITUDE
#define LOCATION_LATITUDE 0
#endif
#ifndef LOCATION_LONGITUDE
#define LOCATION_LONGITUDE 0
#endif
#ifndef LOCATION_QUERY
#define LOCATION_QUERY ""
#endif
#ifndef LOCATION_LABEL
#define LOCATION_LABEL ""
#endif

// Optional time zone that skips automatic detection: an IANA name ("Australia/Perth") or a
// POSIX rule ("AWST-8").  Leave empty to use the zone of the location above.
#ifndef TIMEZONE_POSIX_OVERRIDE
#define TIMEZONE_POSIX_OVERRIDE ""
#endif

// ----------------------------------------------------------------------------
// Units and formats
// ----------------------------------------------------------------------------
#ifndef USE_FAHRENHEIT
#define USE_FAHRENHEIT 0  // 0 = degC and km/h, 1 = degF and mph
#endif
#ifndef TIME_FORMAT
#define TIME_FORMAT "24h"  // "24h" or "12h" (12h shows AM or PM)
#endif
// "iso" (2026-10-04), "dmy" (04/10/2026), "mdy" (10/04/2026), "dmy-dot" (04.10.2026),
// "d-mon-y" (4 Oct 2026) or "mon-d-y" (Oct 4, 2026)
#ifndef DATE_FORMAT
#define DATE_FORMAT "iso"
#endif
#ifndef SHOW_WEEK
#define SHOW_WEEK 1  // 1 = the ISO week number and the day of the year after the weekday
#endif

// ----------------------------------------------------------------------------
// WiFi and network time
// ----------------------------------------------------------------------------
// 0 keeps the radio off: no network time, weather or Spotify, and less power drawn.  Without network
// time the clock runs on its own crystal and drifts (README: "Running without WiFi").
#ifndef WIFI_ENABLED
#define WIFI_ENABLED 1
#endif

// "normal" or "max": how long the radio sleeps between messages.  Max draws a little less, but every
// reply can come up to a third of a second late, the network time included.  The Spotify setup
// page always runs in normal.
#ifndef WIFI_POWER_SAVE
#define WIFI_POWER_SAVE "normal"
#endif

// "always" keeps the radio connected.  "sync" switches it off between syncs: it wakes for the network time (hourly)
// and the weather (every WEATHER_INTERVAL_MIN), looks at Spotify once each time, and stays on while music plays
// (see SPOTIFY_LIVE), after a key press and on the Now Playing page.  When none of its networks is in range it looks
// again every 2 minutes, later every 5, and at once when a button is pressed.  Far less power on a battery, see the
// README ("Saving power").  Not yet tried on the board.
#ifndef WIFI_MODE
#define WIFI_MODE "always"
#endif

// The clock's name on the network: letters, digits and '-'.  The Spotify setup page is http://<name>.local
#ifndef APP_HOSTNAME
#define APP_HOSTNAME "rlcd-clock"
#endif

// ----------------------------------------------------------------------------
// Time servers
// ----------------------------------------------------------------------------
// NTP_SERVER is tried first, the three below follow ("" = just those three).  Also the "ntp_server" setting.
#ifndef NTP_SERVER
#define NTP_SERVER ""
#endif
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.cloudflare.com"
#define NTP_SERVER_3 "time.google.com"

// ----------------------------------------------------------------------------
// Clock face
// ----------------------------------------------------------------------------
// 0  = the second hand ticks once per second, exactly on the second.
// >0 = redraw this many times per second so the second hand sweeps smoothly.
//      Uses noticeably more power; 5-10 is plenty.
#define CLOCK_SWEEP_FPS 0

// The next second is drawn ahead of time and sent so that the transfer ends this long
// before the second starts.  The panel refreshes itself at 25.5 Hz (every 39 ms),
// so a frame shows up 0-39 ms after it has been sent; half of that makes the average
// error zero.  Raise it if the digits seem to change late, lower it if early.  With
// DISPLAY_POWER "low" the panel refreshes at 8 Hz and the clock adds 43 ms to it.
#define DISPLAY_LATENCY_MS 20

// ----------------------------------------------------------------------------
// Display polarity
// ----------------------------------------------------------------------------
// 1 = black ink on a white background (what you want on a reflective LCD).
// If your panel shows the opposite, set this to 0.  You can also flip it
// without reflashing: hold the BOOT button for one second (remembered).
#define DISPLAY_INK_IS_BLACK 1

// ----------------------------------------------------------------------------
// Indoor sensor (SHTC3)
// ----------------------------------------------------------------------------
// The sensor sits next to the ESP32, the charger and the display driver, so it
// reads warmer than the room.  Waveshare's own driver subtracts 4 degC; tune
// this against a thermometer you trust (or with "indoor_offset" on the SD card).
// Humidity is corrected for the same offset (relative humidity falls as air warms).
#ifndef INDOOR_TEMP_OFFSET_C
#define INDOOR_TEMP_OFFSET_C (-4.0f)
#endif
#define SENSOR_INTERVAL_MS 10000UL

// ----------------------------------------------------------------------------
// Battery
// ----------------------------------------------------------------------------
#define BATTERY_LOW_PERCENT 20      // below this the gauge blinks (unless it is charging)
#define BATTERY_LOW_CLEAR_PERCENT 23  // ... and keeps blinking until it climbs back to this
#define BATTERY_INTERVAL_MS 5000UL

// "auto" or "none".  Say "none" when no battery is fitted (running from USB): the gauge then shows USB,
// nothing is estimated and the clock never shuts down for a low battery.  The clock cannot tell by
// itself: on USB the empty battery socket reads like a full battery.
#ifndef BATTERY_MODE
#define BATTERY_MODE "auto"
#endif

// 1 = shut down (deep sleep, with a message on the screen) before the battery is flat, so a LiPo is not
// ruined.  The clock wakes by itself once charging has brought the battery back, or on a button press.
// BATTERY_CUTOFF_V is the voltage, as measured while the clock runs, at which it shuts down (3.10 to 3.60).
#ifndef LOW_BATTERY_SHUTDOWN
#define LOW_BATTERY_SHUTDOWN 1
#endif
#ifndef BATTERY_CUTOFF_V
#define BATTERY_CUTOFF_V 3.30f
#endif

// Capacity of your battery in mAh, so the Power and settings page can show the average current it
// draws (0 = unknown; the runtime estimate does not need it).  Also a setting on the SD card
// (battery_capacity_mah).  Put your own value in secrets.h.
#ifndef BATTERY_CAPACITY_MAH
#define BATTERY_CAPACITY_MAH 0
#endif

// Correction for the battery voltage the clock measures, as a multiplier (1 = none).  The ADC and the
// 1:3 divider read a little low on many boards: a full cell (4.20 V on a LiPo tester) shows as
// 4.13 V, so the gauge stops at 93 % and the clock never sees the cell as full.  The factor is
// the real voltage divided by what the Power and settings page shows (4.20 / 4.13 = 1.017); the
// serial command "batcal 4.20" works it out.  Also a setting on the SD card (battery_calibration).
// Put your own value in secrets.h.
#ifndef BATTERY_CALIBRATION
#define BATTERY_CALIBRATION 1.0f
#endif

// The board has no software-readable charge signal, so charging / discharging / full
// is inferred from how the battery voltage moves (charge.h): it needs about a minute
// to notice the cable being plugged or pulled and about thirteen after a reboot.  For a
// certain answer, wire the charger's STAT output (the net that lights the charge LED
// on the board) to a free GPIO and put that pin number here.  UNTESTED on real
// hardware: check with a meter first that the pin goes LOW while charging and is
// high or floating otherwise.  -1 = not wired, use the voltage.
#define PIN_CHARGE_STATUS -1

// ----------------------------------------------------------------------------
// Power
// ----------------------------------------------------------------------------
// CPU clock in MHz: 80, 160 or 240.  80 uses the least power and is plenty for a clock.  (WiFi needs 80
// at least.)  The PSRAM switch is a build option of the Arduino IDE and cannot be set here.
#ifndef CPU_MHZ
#define CPU_MHZ 80
#endif
// The clock while the radio is off (WIFI_ENABLED 0, or between the syncs of WIFI_MODE "sync"): 0 = the same as CPU_MHZ,
// or 80, 40 or 20.  Slower draws less, and a frame takes longer to draw (about 30 ms at 80 MHz, 120 at 20).  40 is the
// lowest clock Espressif gives current figures for (13 mA idle against 22 at 80), so try that first.  Not yet tried on
// the board: below 80 MHz the clock of some peripherals follows the CPU, and the USB console may stop, so the clock
// stays at CPU_MHZ for as long as a computer is on the USB port and the console runs.
#ifndef CPU_IDLE_MHZ
#define CPU_IDLE_MHZ 0
#endif

// The USB serial console (the log, and the commands of the README): "on", "auto" or "off".  "auto" runs it while a
// computer is on the USB port and shuts it down, with the USB transceiver, when there has been none for 10 seconds;
// "off" shuts it down at the end of every start-up.  It then stays off until the next restart, and the USB port is
// dead: to upload a firmware restart the clock with KEY held down (the console stays on for that run), or hold BOOT
// while switching it on, or use the SD card.  Saves a little power.  Not yet tried on the board.
#ifndef CONSOLE_MODE
#define CONSOLE_MODE "on"
#endif

// 1 = the clock keeps a log of its own power readings (the battery, the runtime estimate, the CPU clock, the radio's
// share of the time): one line of CSV at each start and every 10 minutes, in the FAT partition of its flash, the
// newest 0.5 to 1 MB of them.  A start with an SD card in copies the log to the card (POWER_LOG_FILE_NAME), and the
// serial command "powerlog" prints it.  The first time, the partition is formatted.  A line costs a fraction of a
// second of flash writing (README: "Logging a battery run").  Not yet tried on the board.
#ifndef POWER_LOG
#define POWER_LOG 0
#endif

// 1 = between the frames, once a second, the processor goes into light sleep instead of idling, whenever nothing needs
// it awake: the radio off (WIFI_ENABLED 0, or between the syncs of WIFI_MODE "sync"), the console shut down
// (CONSOLE_MODE "auto" or "off"), no button pressed for 8 seconds.  KEY and BOOT wake it at once.  While it sleeps its
// clock runs on an RC oscillator inside the chip, calibrated before every sleep but less exact than the crystal: the
// hourly network time puts it right, and without WiFi nothing does.  Not yet tried on the board (README: "Light sleep").
#ifndef LIGHT_SLEEP
#define LIGHT_SLEEP 0
#endif

// "normal" or "low": how the display's controller (ST7305) drives the panel.  "low" = its low power mode, which
// refreshes the panel 8 times a second instead of 25.5: less current (how much has not been measured), and a frame
// shows up 0 to 125 ms after it was sent instead of 0 to 39, which the frame timing allows for (DISPLAY_LATENCY_MS
// and 43 ms more, display_power.h).  Not yet tried on the board (README: "The display's low power mode").
#ifndef DISPLAY_POWER
#define DISPLAY_POWER "normal"
#endif

// The audio chips on the board (a codec, a microphone ADC, an amplifier) are not used by the clock.  1 = at
// start-up they are told to power down over I2C, with the sequences Waveshare's own audio example uses when it
// closes them, and the I2S lines to them are held low.  0 = they are left as they come out of power-on (the
// amplifier is held off either way).  Not yet tried on the board.
#ifndef AUDIO_CHIPS_STANDBY
#define AUDIO_CHIPS_STANDBY 1
#endif

// ----------------------------------------------------------------------------
// Weather
// ----------------------------------------------------------------------------
#ifndef WEATHER_INTERVAL_MIN
#define WEATHER_INTERVAL_MIN 15  // minutes, 5 to 240; Open-Meteo refreshes every 15 min
#endif
#define WEATHER_RETRY_MS (60UL * 1000UL)

// ----------------------------------------------------------------------------
// Spotify (needs a Client ID: SPOTIFY_CLIENT_ID in secrets.h, or "spotify_client_id" on the SD card)
// ----------------------------------------------------------------------------
#ifndef SPOTIFY_ENABLED
#define SPOTIFY_ENABLED 1  // 0 = off, even with a Client ID
#endif
// Only matters with WIFI_MODE "sync".  1 = while music plays the radio stays on and the clock asks Spotify every
// 10 seconds.  0 = the radio sleeps while music plays too and the clock looks again when the track on screen should
// be over: far less radio, but a track skipped or paused on a phone shows late.  Not yet tried on the board.
#ifndef SPOTIFY_LIVE
#define SPOTIFY_LIVE 1
#endif

// Spotify gives development-mode apps a small, unpublished request quota; people
// polling every 3 s around the clock have been locked out for hours.  These
// intervals stay well inside it.  The progress bar is extrapolated between polls
// and a button press always triggers an immediate refresh.
#define SPOTIFY_POLL_PLAYING_MS 10000UL
#define SPOTIFY_POLL_PAUSED_MS 15000UL
#define SPOTIFY_POLL_IDLE_MS 30000UL  // doubled after ~30 minutes of nothing playing

// KEY button timing
#define KEY_MULTI_CLICK_GAP_MS 320   // pause after the last click before the count is acted on
#define KEY_LONG_PRESS_MS 800
#define KEY_RESTART_HOLD_MS 5000     // KEY held this long, then let go: the clock restarts (and so reads the SD card)
#define BOOT_LONG_PRESS_MS 1000

// ----------------------------------------------------------------------------
// Board pins (Waveshare ESP32-S3-RLCD-4.2) - fixed by the hardware
// ----------------------------------------------------------------------------
#define PIN_LCD_SCK 11
#define PIN_LCD_MOSI 12
#define PIN_LCD_DC 5
#define PIN_LCD_CS 40
#define PIN_LCD_RST 41
#define PIN_I2C_SDA 13
#define PIN_I2C_SCL 14
#define PIN_BATTERY_ADC 4  // battery through a 1:3 divider
#define PIN_KEY 18         // user key, active low
#define PIN_BOOT 0         // BOOT key, active low
#define PIN_SD_CLK 38      // SD card slot, SDMMC in 1-bit mode (same wiring as the 06_SD_Card example)
#define PIN_SD_CMD 21
#define PIN_SD_D0 39

// The audio section, which the clock does not use: the amplifier's enable and the I2S lines that go out to the chips
// (the same numbers as Waveshare's board file for this board in 02_Example/XiaoZhi)
#define PIN_AUDIO_AMP 46  // amplifier enable, high = on
#define PIN_I2S_MCLK 16
#define PIN_I2S_BCLK 9
#define PIN_I2S_WS 45
#define PIN_I2S_DOUT 8    // data to the codec (the line from it, GPIO10, is the codec's to drive)

#define BATTERY_DIVIDER 3.0f
#define I2C_ADDR_SHTC3 0x70
#define I2C_ADDR_PCF85063 0x51
#define I2C_ADDR_ES8311 0x18  // audio codec
#define I2C_ADDR_ES7210 0x40  // microphone ADC

// ----------------------------------------------------------------------------
// Credentials (from secrets.h, included above)
// ----------------------------------------------------------------------------
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
// An optional second network, used when the first cannot be joined.
#ifndef WIFI_SSID_BACKUP
#define WIFI_SSID_BACKUP ""
#endif
#ifndef WIFI_PASSWORD_BACKUP
#define WIFI_PASSWORD_BACKUP ""
#endif
#ifndef SPOTIFY_CLIENT_ID
#define SPOTIFY_CLIENT_ID ""
#endif
