#pragma once

// The SD card slot, used for one thing: the settings file, read (and, if the card has none,
// written) at boot.  The card is mounted for that and unmounted again; nothing touches it at
// run time.  Wiring as in the 06_SD_Card example: SDMMC, 1-bit, CLK 38, CMD 21, D0 39.
// FAT32 only (the core's FatFs has no exFAT, which is what 64 GB and larger cards come
// formatted with).  Long file names work.

#include <Arduino.h>

enum class SdStatus : uint8_t {
  NO_CARD,     // nothing answered (this also covers a card that is not seated)
  UNREADABLE,  // a card answered but its file system would not mount: sdProblemText() says what is on it
  READY        // mounted
};

// Mounts the card.  With no card in the slot this takes up to about a second.
SdStatus sdMount();
void sdUnmount();

// After sdMount() gave UNREADABLE: one line for the screen about what the card holds instead of a FAT file
// system the clock can read ("SD card is exFAT: format it as FAT32", "SD card: GPT partitions, needs MBR", ...).
// The clock reads the card's first sector to find out (sd_layout.h).
const char *sdProblemText();

// "SDHC 14.8 GB" for the Info page (after a successful sdMount()).
void sdCardSummary(char *out, size_t cap);

// Files in the root directory.
bool sdFileExists(const char *name);
// Reads a whole file into a malloc'd, NUL-terminated buffer (the caller frees it).  False if it
// cannot be opened, is longer than maxBytes or memory is short.  An empty file gives len 0.
bool sdReadFile(const char *name, char **data, size_t *len, size_t maxBytes);
// Creates or replaces a file and flushes it to the card.  Returns true if every byte was written.
bool sdWriteFile(const char *name, const char *data, size_t len);

// For the firmware file, which is read in pieces: its size, and a handle to read it with (-1 if it
// cannot be opened).  Plain POSIX reads, so 4 KB requests go straight to the card.
bool sdFileSize(const char *name, uint32_t *size);
int sdOpen(const char *name);
int sdRead(int fd, void *buf, size_t len);  // bytes read, or -1
bool sdSeek(int fd, uint32_t offset);
void sdClose(int fd);
// Renames a file; an existing file with the new name is replaced.
bool sdRename(const char *from, const char *to);

// For the copy of the power log, which is written in pieces: a new file (one of that name is replaced) and a
// handle to write it with (-1 if it cannot be made), the pieces, and the end, which flushes and closes it.
// sdCommit() always closes; true only if the file is safely on the card.
int sdCreate(const char *name);
bool sdWrite(int fd, const void *buf, size_t len);  // true if every byte was taken
bool sdCommit(int fd);
bool sdRemove(const char *name);
