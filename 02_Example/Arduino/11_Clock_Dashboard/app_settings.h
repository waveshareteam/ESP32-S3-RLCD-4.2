#pragma once

// The settings in force, and where they come from: the defaults in config.h and secrets.h, then
// what the clock saved in its flash ("cfg" namespace), then the settings file on an SD card,
// which is read once at boot and saved to flash so the card can be taken out again.  The format
// and the parsing live in settings.h / settings.cpp (host-tested); this file is the part that
// touches flash and the card.
//
// g_cfg is filled in during setup(), before any task starts, and only read afterwards.

#include <Arduino.h>

#include "settings.h"

extern Settings g_cfg;

struct CfgStatus {
  enum Sd : uint8_t {
    SD_NOT_TRIED = 0,
    SD_NO_CARD,
    SD_UNREADABLE,       // a card, but no file system on it would mount (exFAT, a GPT partition table, ...: sdProblemText())
    SD_EXAMPLE_WRITTEN,  // a card with no settings file: an example was written to it
    SD_WRITE_FAILED,     // ... or could not be (write-protected card?)
    SD_READ_FAILED,      // the file is there but could not be read (too big, card error)
    SD_FILE_APPLIED      // the file was read; `report` says what it did
  };
  Sd sd = SD_NOT_TRIED;
  int fromFlash = 0;         // settings that came from flash at this boot
  bool savedToFlash = false;
  int carried = 0;           // settings a card changed at the start before this one, which restarted (to install a
                             // firmware from the card) before it could say so; see cfgAnnounced()
  char file[40] = "";        // the file name that was found or written
  char card[24] = "";        // "SDHC 14.8 GB"
  ConfigReport report;
};
extern CfgStatus g_cfgStatus;

// The defaults this build ships with: config.h and secrets.h (see build_defaults.h).  A value in
// there that the settings rules refuse is left out, and counted here.
Settings cfgBuildDefaults();
int cfgBuildProblems();

// g_cfg = defaults, then flash.  Quick; call it first in setup().
void cfgLoadFlash();

// Writes the settings in force that did not come from the defaults to flash (after something changed
// g_cfg at run time, as the serial command "batcal" does).  False if any of it could not be kept.
bool cfgSaveToFlash();

// Looks for a card.  With a settings file: applies it on top of g_cfg and saves what it changed
// to flash.  Without one: writes an example file to the card.  Unmounts the card again.
void cfgImportSdCard();

// The banner about what the card's settings did has been shown.  Until then the number of settings a card
// changed is kept in flash: a start that goes on to install a firmware from the card restarts before it
// gets to the banner, and the start after it reads the same file and finds nothing left to change.
// g_cfgStatus.carried is that number as the start before this one left it.
void cfgAnnounced();

// One line for the Info page / a toast, and the first few problems: built-in defaults that were not
// used, then those of the SD file.
void cfgSummary(char *out, size_t cap);
int cfgIssueCount();
void cfgIssueText(int i, char *out, size_t cap);

// Every setting on the serial port, passwords hidden.
void cfgPrint(Print &out);
