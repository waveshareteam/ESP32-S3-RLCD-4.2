#pragma once

// Network task: WiFi, NTP, location, weather, time zone and Spotify.
// Runs on core 0 next to the WiFi stack so the UI loop on core 1 never waits on
// the network.

#include <Arduino.h>
#include <time.h>

// Restores the saved time zone / location from NVS (or applies the time zone from the settings).
// Call once from setup(), after the settings are loaded and before netBegin().
void netLoadSettings();

// Starts the network task (not at all when WiFi is switched off in the settings).
void netBegin();

// True once per NTP sync: the UI task should then store the time in the RTC.
bool netTakeRtcWriteRequest();

// Fetch the weather again as soon as possible.
void netRequestWeatherRefresh();

// wifi_mode = sync: a key press (or any other wish for an answer from the network) switches the radio on and
// holds it for 45 seconds.  Does nothing in always mode.  Any task may call it.
void netWake();

// wifi_mode = sync: someone is using the clock (any button).  If it found none of its networks last time it
// looks again now rather than at the next look, so a hotspot that was just switched on is picked up at once.
// Does nothing when the clock has a network, or in always mode.  Any task may call it.
void netNudge();

// True while the radio is on or about to be: the CPU clock must not go below 80 MHz then (clock_policy.h).
bool netRadioNeedsFastClock();
