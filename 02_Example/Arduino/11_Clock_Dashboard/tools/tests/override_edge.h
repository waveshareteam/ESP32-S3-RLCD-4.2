#pragma once

// A secrets.h with values right on the limits of what the settings accept, and the placeholder
// names that secrets.example.h comes with.  test_builddefaults.cpp (-DTEST_EDGE) checks that every
// limit value is taken and that the placeholders count as "no network set".  3.10f is 3.0999999 as a
// double, below the limit of 3.10, which is why a float goes through the rules with six digits.

#define BATTERY_CUTOFF_V 3.10f
#define BATTERY_CALIBRATION 0.80f
#define INDOOR_TEMP_OFFSET_C (-15.0f)
#define BATTERY_CAPACITY_MAH 20000
#define WEATHER_INTERVAL_MIN 5
#define LOCATION_LATITUDE (-90)
#define LOCATION_LONGITUDE 180
#define CPU_MHZ 240

#define WIFI_SSID "YourWiFiName"
#define WIFI_PASSWORD "YourWiFiPassword"
#define WIFI_SSID_BACKUP "YourBackupWiFiName"
#define WIFI_PASSWORD_BACKUP "YourBackupWiFiPassword"
