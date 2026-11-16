#pragma once

// Why an SD card would not mount: worked out from its first sector, and from the first sector of its first
// partition where it has one, so that the clock can say what is on the card instead of guessing.
//
// The FAT library in the ESP32 Arduino core (FatFs) mounts a FAT12, FAT16 or FAT32 volume on a card with a
// classic partition table (MBR) or with none.  It is built without exFAT and without GUID partition tables
// (GPT), so a card that Windows shows as "FAT32" still fails when its partition table is a GPT.
//
// Pure logic with no Arduino dependency: sdcard.cpp reads the sectors, tools/tests feeds it made-up ones.

#include <stdint.h>
#include <string.h>

namespace sdlayout {

const int kSectorBytes = 512;

enum class Kind : uint8_t {
  UNKNOWN = 0,  // not looked at (the card could not be started again for a look)
  READ_ERROR,   // the card would not give up the sector
  BLANK,        // no boot signature, or a partition table with nothing in it: never formatted, or wiped
  GPT,          // a GUID partition table (the first sector is its "protective MBR", type 0xEE)
  EXFAT,        // an exFAT volume
  NTFS,         // an NTFS volume
  FAT,          // a FAT volume, which should have mounted: damaged, or formatted in a way the library refuses
  OTHER         // a partition table whose first partition is none of these (Linux, ...)
};

// What the first sector says.  With `lookAtPartition` the answer is in the partition's own first sector
// (`partitionLba`): read it and ask inspectPartition().
struct First {
  Kind kind = Kind::UNKNOWN;
  bool lookAtPartition = false;
  uint32_t partitionLba = 0;
  uint8_t partitionType = 0;  // the type byte of the partition table entry; 0 = no table
};

inline bool hasBootSignature(const uint8_t *s) { return s[510] == 0x55 && s[511] == 0xAA; }

// The boot sector of a volume, by the name it carries.
inline bool isExfat(const uint8_t *s) { return hasBootSignature(s) && !memcmp(s + 3, "EXFAT   ", 8); }
inline bool isNtfs(const uint8_t *s) { return hasBootSignature(s) && !memcmp(s + 3, "NTFS    ", 8); }
// A FAT boot sector: the jump at the start, 512 byte sectors, and the type name where FAT32 keeps it (byte 82)
// or where FAT12 and FAT16 do (byte 54).
inline bool isFat(const uint8_t *s) {
  if (!hasBootSignature(s)) return false;
  if (s[0] != 0xEB && s[0] != 0xE9 && s[0] != 0xE8) return false;
  if ((uint16_t)(s[11] | (s[12] << 8)) != 512) return false;
  return !memcmp(s + 82, "FAT32   ", 8) || !memcmp(s + 54, "FAT12   ", 8) || !memcmp(s + 54, "FAT16   ", 8);
}

inline Kind volumeKind(const uint8_t *s) {
  if (isExfat(s)) return Kind::EXFAT;
  if (isNtfs(s)) return Kind::NTFS;
  if (isFat(s)) return Kind::FAT;
  return Kind::UNKNOWN;
}

inline bool isFatPartitionType(uint8_t type) {
  return type == 0x01 || type == 0x04 || type == 0x06 || type == 0x0B || type == 0x0C || type == 0x0E;
}

// The first sector of the card.
inline First inspectFirst(const uint8_t *s) {
  First f;
  if (!hasBootSignature(s)) {
    f.kind = Kind::BLANK;
    return f;
  }
  const Kind whole = volumeKind(s);  // a volume that fills the card, with no partition table
  if (whole != Kind::UNKNOWN) {
    f.kind = whole;
    return f;
  }
  // a partition table: four entries of 16 bytes from byte 446 (type at +4, first sector at +8)
  for (int i = 0; i < 4; i++) {
    const uint8_t *e = s + 446 + 16 * i;
    const uint32_t lba = (uint32_t)e[8] | ((uint32_t)e[9] << 8) | ((uint32_t)e[10] << 16) | ((uint32_t)e[11] << 24);
    if (e[4] == 0 || lba == 0) continue;
    f.partitionType = e[4];
    f.partitionLba = lba;
    if (e[4] == 0xEE) {  // the placeholder a GUID partition table puts here
      f.kind = Kind::GPT;
    } else {
      f.lookAtPartition = true;
    }
    return f;
  }
  f.kind = Kind::BLANK;  // a boot signature and an empty table
  return f;
}

// The first sector of the partition that inspectFirst() pointed at.
inline Kind inspectPartition(const uint8_t *s, uint8_t partitionType) {
  const Kind k = volumeKind(s);
  if (k != Kind::UNKNOWN) return k;
  // nothing recognisable there: go by what the table calls it
  if (isFatPartitionType(partitionType)) return Kind::FAT;  // a FAT partition with a damaged start
  return Kind::OTHER;
}

// One line for the screen (a banner holds 39 characters; the layout check in tools/ui_preview measures
// every one of these).
inline const char *text(Kind k) {
  switch (k) {
    case Kind::READ_ERROR: return "SD card: cannot read it";
    case Kind::BLANK: return "SD card is not formatted: use FAT32";
    case Kind::GPT: return "SD card: GPT partitions, needs MBR";
    case Kind::EXFAT: return "SD card is exFAT: format it as FAT32";
    case Kind::NTFS: return "SD card is NTFS: format it as FAT32";
    case Kind::FAT: return "SD card: FAT found, will not mount";
    case Kind::OTHER: return "SD card: no FAT32 partition on it";
    default: return "SD card: no FAT32 found on it";
  }
}

const Kind kAllKinds[] = {Kind::UNKNOWN, Kind::READ_ERROR, Kind::BLANK, Kind::GPT, Kind::EXFAT, Kind::NTFS, Kind::FAT, Kind::OTHER};

// A word for the log.
inline const char *name(Kind k) {
  switch (k) {
    case Kind::READ_ERROR: return "read error";
    case Kind::BLANK: return "blank";
    case Kind::GPT: return "GPT";
    case Kind::EXFAT: return "exFAT";
    case Kind::NTFS: return "NTFS";
    case Kind::FAT: return "FAT";
    case Kind::OTHER: return "other";
    default: return "unknown";
  }
}

}  // namespace sdlayout
