#pragma once

// Updating the firmware from a file on the SD card, and keeping a new firmware on trial.
//
//   * At start-up the clock looks for one firmware file in the card's top folder (config.h:
//     FIRMWARE_FILE_NAME and friends: the .ino.bin that the Arduino IDE exports).  If it is a good app
//     image for this chip that is not what runs already, it is checked against its own SHA-256, written to
//     the other app slot and the clock restarts into it.  The running firmware is not touched until the
//     new image has been written and checked; a card pulled out half way only loses the update.
//   * A freshly installed firmware is on trial: the bootloader puts the old one back if the new one resets
//     before it has run FIRMWARE_TRIAL_MS (the core would otherwise confirm it at once).
//
// What makes a file acceptable is in fw_logic.h (host-tested); this file is the part that touches the
// card, the flash and the screen.  Needs a partition scheme with two app slots (the README's one has).

#include <Arduino.h>

#include "app_model.h"

// Who are we: on trial or not, was the last update rolled back.  First thing in setup().
void fwBegin();

// The build that runs: the first seven hex digits of the hash that also tells builds apart for the updater
// (the serial command "fw" prints sixteen).  It changes whenever the program does, which a version number
// and a date do not.  The hash is that of the whole ELF file, debug information and all, and that names the
// folder it was built in: the same sources built in another folder, or on another computer, get another id.
const char *fwBuildId();

// Called while the card is mounted for the settings file: notes which firmware files are on it.
void fwScanCard();

// Draws a firmware-update screen (clear, draw, send); supplied by the sketch, which owns the display.
typedef void (*FwShowFn)(const UiFwScreen &screen);

// After the display is up.  With one good firmware file on the card: installs it, with progress on the
// screen, and restarts (so it does not return).  With a file that cannot be used: says why for a few
// seconds and returns.  With nothing on the card: returns at once.
void fwUpdateFromCard(FwShowFn show);

// A new firmware confirms itself once it has run FIRMWARE_TRIAL_MS.  Call from loop().
void fwTrialTick(uint32_t nowMs);
// Confirm it now: going to sleep on purpose, or a reset the user asked for, are not failures.
void fwConfirmNow();

bool fwOnTrial(uint32_t nowMs, uint32_t *secondsLeft);  // for the Info page
bool fwRolledBack();  // the other app slot holds an image the bootloader gave up on: the last update was undone

bool fwSwitchToOtherSlot();  // serial "rollback": the next start runs the other slot
void fwForgetLast();         // serial "fwforget": the card may install the same file again
void fwPrint(Print &out);    // serial "fw": what runs, what is in the other slot
