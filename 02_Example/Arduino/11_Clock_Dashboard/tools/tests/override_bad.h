#pragma once

// A secrets.h with mistakes in it: values the SD card file would refuse too.  test_builddefaults.cpp
// (-DTEST_BAD) checks that each one is left out and reported, and that the good ones beside them
// still count.  (Seven bad values, in the order buildDefaults() looks at them: the report keeps three.)

#define APP_HOSTNAME "my clock!"       // letters, digits and '-' only
#define WIFI_POWER_SAVE "turbo"        // normal or max
#define TIME_FORMAT "noon"             // 24h or 12h
#define BATTERY_CUTOFF_V 2.0f          // 3.10 to 3.60: lower would ruin the cell
#define CPU_MHZ 100                    // 80, 160 or 240
#define CPU_IDLE_MHZ 10                // 0, 80, 40 or 20: 10 is not on offer
#define CONSOLE_MODE "sometimes"       // on, auto or off

#define DATE_FORMAT "d-mon-y"          // good ones
#define SHOW_WEEK 0
#define BATTERY_CAPACITY_MAH 1800
#define LOCATION_LATITUDE (-31.952240) // and coordinates that keep all their digits (a metre, like the card's)
#define LOCATION_LONGITUDE 115.861456
