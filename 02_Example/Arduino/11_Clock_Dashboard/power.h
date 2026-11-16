#pragma once

// Power: the CPU clock, and the low-battery shutdown (deep sleep with a timer and the KEY button
// as ways out).  See low_battery.h for when the shutdown happens and when the clock starts again.

#include <Arduino.h>

#include "light_sleep.h"

// Sets the CPU clock (80, 160 or 240 MHz; WiFi works from 80 up).  Does nothing if it is already
// there.  Once the display is running, wrap the call in ST7305_U8g2::busRelease() / busAcquire():
// the core tells every SPI bus about a clock change and waits for its lock.
void powerSetCpuMhz(int mhz);

// First thing in setup(): takes the display pins back from a deep sleep (their levels are
// carried over, so nothing floats in between).  The audio amplifier's enable is among them, held low (off).
void powerAfterWake();

// Gives the I2S lines to the unused audio chips a level (the chip's weak pull-downs), so they do not float.
void powerQuietAudioPins();
bool powerWokeFromTimer();   // woken by the five minute battery check, not by a person
bool powerWasLowShutdown();  // the last run ended in a low-battery shutdown (kept through deep sleep)
void powerForgetLowShutdown();

// True once the clock is on its way to deep sleep: the network task then leaves the radio alone.
extern volatile bool g_powerDown;

// light_sleep = on: one light sleep of up to `us` microseconds, ended early by KEY or BOOT (light_sleep.h says when,
// and how the steps go).  The UI loop calls it, with the buttons recorded by interrupt (buttonEdgesBegin()).
lightsleep::Nap powerLightSleep(int32_t us, lightsleep::WallFix &fix);

// Deep sleep; never returns.  The display keeps its picture, the timer wakes the chip every few
// minutes to look at the battery, and a press of KEY wakes it too (BOOT cannot: GPIO0 is the
// download-mode strapping pin).  markLowShutdown remembers "this was a low-battery shutdown", so
// the restart waits for the battery to recover; false is for testing the sleep itself.
[[noreturn]] void powerDeepSleep(bool markLowShutdown);
