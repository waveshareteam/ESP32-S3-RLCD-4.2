#pragma once

// Minimal serial logging: LOGF("tag", "format %d", value)
//
// The console can be shut down for good ("console = auto" or "off", console_policy.h): from then on LOGF
// prints nothing.  Any task may log, and the port must not be taken away under a task that is in the middle
// of a line, so every line is bracketed by logBegin() / logEnd() and the shutdown waits for the last one.

#include <Arduino.h>

// False once the console was shut down.  Code that talks to Serial directly (the commands) checks it.
extern volatile bool g_consoleOn;

bool logBegin();  // true: go ahead and print, then call logEnd()
void logEnd();

// Starts the serial port.  First thing in setup().  (Also undoes what consoleShutDown() did to the USB pins,
// which outlives a restart that is not a power-on.)
void consoleBegin();

// Shuts the console down until the next restart: nothing more is printed, the serial driver is stopped (its
// interrupt and buffers freed) and, when Serial is the chip's USB port, the USB transceiver with its pull-up
// is switched off, so a computer no longer sees the clock.  Call it from the UI task.
void consoleShutDown();

// While something long is printed that other tasks' lines must not get into the middle of (the power log,
// which is copied from the console as a file): LOGF lines are dropped, not kept for later.  consoleHold(true)
// returns once the line another task was in the middle of is out.
void consoleHold(bool hold);

#define LOGF(tag, fmt, ...)                                                                        \
  do {                                                                                             \
    if (logBegin()) {                                                                              \
      Serial.printf("[%8lu] %-8s " fmt "\n", (unsigned long)millis(), tag, ##__VA_ARGS__);       \
      logEnd();                                                                                    \
    }                                                                                              \
  } while (0)
