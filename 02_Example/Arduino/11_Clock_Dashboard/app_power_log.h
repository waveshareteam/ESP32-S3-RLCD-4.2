#pragma once

// The power log on the clock: the flash partition its two files live in, the copy to an SD card and the
// serial commands.  What a line holds, when one is due and what is done with the files is in power_log.h
// (host-tested); this file is the part that touches the flash, the card and the serial port.
//
//   * The files are in the FAT partition of the clock's own flash ("ffat" in the partition table, 9.9 MB in
//     the scheme the README asks for), on top of the IDF's wear levelling.
//   * The partition is mounted once, at start-up, and stays mounted while the clock runs.  Mounting for every
//     line would cost more than the line: an unmount makes the wear levelling move a sector each time, and a
//     mount reads its position record from the start.  Between two lines no file is open and nothing waits to
//     be written, so a restart, a shutdown or a power cut at any other moment loses nothing.
//   * The partition is only touched when the power_log setting is on, or has been on before (flash, "plog" /
//     "used"): a clock that never kept a log leaves it alone.  The first time, it is formatted if it holds no
//     FAT file system.
//
// Everything here is called from the UI task.

#include <Arduino.h>

#include "power_log.h"

// At start-up, once the settings are final: mounts the partition when the log is on (and formats it the first
// time, about half a second).
void powerLogBegin();

// At start-up, while an SD card is mounted and once its settings are in force: a copy of the log for the card
// (POWER_LOG_FILE_NAME), when the log is on and has lines.  `wasOn`: the log was on until this card's settings
// were read, so a card that switches it off still gets what there is, once.
void powerLogCopyToCard(bool wasOn);

// Is a line due?  `nowSec`: seconds since the start.  Never true while the log is off or cannot be written.
bool powerLogDue(uint32_t nowSec);

// Writes one line (the note of a start is added here).  Takes a tenth of a second or a few, so call it early
// in a second (powerlog::kLatestStartUs).
void powerLogWrite(powerlog::Reading reading, uint32_t nowSec);

// For the Power and settings page (powerlog::statusRows), and how long the writes took, for the "power" command.
powerlog::Status powerLogStatus(uint32_t nowSec);
void powerLogTimings(uint32_t *lastMs, uint32_t *worstMs);

// The serial commands "powerlog" (prints the log as CSV) and "powerlog clear" (forgets it).  `arg` is what
// follows the word.
void powerLogCommand(const char *arg, Print &out);
