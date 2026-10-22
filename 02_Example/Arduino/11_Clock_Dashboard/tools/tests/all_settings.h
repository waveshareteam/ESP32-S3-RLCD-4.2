#pragma once

// A settings file that sets every setting to something other than its default.  test_features.cpp
// uses it for the file parser and the flash round trip; test_builddefaults.cpp checks that a build
// whose secrets.h says the same things (override_all.h) ends up with the very same settings.

[[maybe_unused]] static const char *const kAllSettings =
    "wifi = off\n"
    "wifi_ssid = Cafe Net\n"
    "wifi_password = p@ss w0rd\n"
    "wifi_backup_ssid = Phone Hotspot\n"
    "wifi_backup_password = h0tsp0t pw\n"
    "hostname = my-clock\n"
    "ntp_server = time.nist.gov\n"
    "wifi_power_save = max\n"
    "wifi_mode = sync\n"
    "units = imperial\n"
    "time_format = 12h\n"
    "date_format = mdy\n"
    "show_week = off\n"
    "latitude = 40.7128 N\n"
    "longitude = 74.0060 W\n"
    "location_label = New York\n"
    "location = New York\n"
    "timezone = America/New_York\n"
    "spotify = off\n"
    "spotify_client_id = 0123456789abcdef0123456789abcdef\n"
    "spotify_live = off\n"
    "indoor_offset = -2.5\n"
    "battery = none\n"
    "battery_capacity_mah = 2500\n"
    "battery_calibration = 1.0157\n"
    "low_battery_shutdown = off\n"
    "battery_cutoff_v = 3.45\n"
    "cpu_mhz = 160\n"
    "cpu_idle_mhz = 40\n"
    "console = auto\n"
    "power_log = on\n"
    "light_sleep = on\n"
    "display_power = low\n"
    "weather_interval_min = 30\n";
