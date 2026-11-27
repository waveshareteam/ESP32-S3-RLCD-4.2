#pragma once

// Screen renderer.  Pure drawing code: it only uses the U8g2 C API and libc,
// so the same file runs on the ESP32 and in the host-side preview tool
// (tools/ui_preview) that renders frames to PNG for layout work.

#include "app_model.h"

#include <clib/u8g2.h>

// Logical drawing surface: 400 x 300, 1 = ink (black), 0 = paper (white).
#define UI_WIDTH 400
#define UI_HEIGHT 300

// Draws one complete frame described by `model`.  The caller clears the buffer
// beforehand and sends it to the panel afterwards.
void uiDraw(u8g2_t *u8g2, const UiModel &model);

// The "battery empty" screen shown when the clock switches itself off to protect the cell: how
// far the battery is, the level it restarts at, and (offerOverride) the hold-KEY way round it.
void uiDrawBatteryEmpty(u8g2_t *u8g2, float volts, float restartVolts, bool offerOverride);

// The firmware-update screen: a chip, a heading, what is going on, a progress bar and the file.
void uiDrawFirmwareUpdate(u8g2_t *u8g2, const UiFwScreen &screen);

// A tumbling cube, white on black, over the whole screen: one of mirrors, each face a window on the reflections
// of its own lit edges, or one with sand on its walls that runs to whichever side is down.  The caller sends the
// buffer.  It is no page of the clock.
struct UiCube {
  bool mirrors = false;    // which of the two
  uint32_t seed = 1;       // of the way it tumbles (and of where the sand lies to begin with): another one every showing
  float turnMs = 0;        // how far along its tumbling it is, in milliseconds
  float shove[3] = {0, 0, 0};  // what it has been shoved round by on top of that (infcube::Shove::angle)
  uint32_t elapsedMs = 0;  // since it came up; has to go up from call to call (the sand runs by it)
};
void uiDrawCube(u8g2_t *u8g2, const UiCube &cube);
