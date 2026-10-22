#pragma once

// Updating the firmware from a file on the SD card: when a file is accepted and when it is left alone.
// Pure logic (no Arduino, no flash), so tools/tests covers it on a PC; fw_update.cpp does the card, the
// flash and the screen.
//
// The file is the app image the Arduino IDE exports (Sketch > Export Compiled Binary): <sketch>.ino.bin.
// An ESP-IDF app image is a 24 byte header, then segments (the first starts with the app descriptor), then
// a SHA-256 of everything before it (the header says so).  A build is identified by the ELF hash inside the
// descriptor, which changes with every build.  The tests check these offsets against the real image.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace fwlogic {

const size_t kHeadBytes = 208;        // enough to reach the ELF hash (bytes 176..207)
const size_t kDigestBytes = 32;       // SHA-256
const size_t kElfShaOffset = 176;     // esp_app_desc_t starts at byte 32; app_elf_sha256 is 144 bytes into it
const uint32_t kMinImageBytes = 128 * 1024;  // an app is far bigger than a bootloader (20 KB) or a partition table (3 KB)
const uint16_t kChipEsp32S3 = 9;
const uint8_t kImageMagic = 0xE9;
const uint32_t kAppDescMagic = 0xABCD5432;
const int kMinBatteryMv = 3600;       // below this on battery the update waits for a charge

enum class Problem : uint8_t {
  NONE = 0,
  TOO_SMALL,     // a bootloader or partition table, or a copy that was cut short
  TOO_BIG,       // does not fit the app slot: the .merged.bin, or another partition scheme
  NOT_AN_IMAGE,  // the first byte is not 0xE9
  WRONG_CHIP,    // built for another ESP32
  NOT_AN_APP,    // an image, but without an app descriptor (the bootloader)
  NO_DIGEST      // no SHA-256 at the end, so nothing to check the copy against
};

inline uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

// `head` holds the first kHeadBytes of the file.  slotSize 0 means unknown (no size limit applied).
inline Problem inspectImage(const uint8_t *head, size_t headLen, uint32_t fileSize, uint32_t slotSize) {
  if (fileSize < kMinImageBytes || headLen < kHeadBytes) return Problem::TOO_SMALL;
  if (slotSize != 0 && fileSize > slotSize) return Problem::TOO_BIG;
  if (head[0] != kImageMagic) return Problem::NOT_AN_IMAGE;
  if ((uint16_t)(head[12] | (head[13] << 8)) != kChipEsp32S3) return Problem::WRONG_CHIP;
  if (le32(head + 32) != kAppDescMagic) return Problem::NOT_AN_APP;
  if (head[23] != 1) return Problem::NO_DIGEST;
  return Problem::NONE;
}

inline const uint8_t *elfHash(const uint8_t *head) { return head + kElfShaOffset; }

enum class Verdict : uint8_t {
  NOTHING,          // no firmware file on the card
  SEVERAL,          // more than one: which is the newer?  Leave one.
  NO_SLOT,          // the partition scheme has no second app slot to write to
  BAD_FILE,         // see Decision::problem
  SAME_AS_RUNNING,  // this very build is what runs now
  TRIED_BEFORE,     // the card already installed this build once (it did not stay: rolled back, or flashed over)
  LOW_BATTERY,
  INSTALL
};

struct Inputs {
  int files = 0;                          // firmware files found on the card
  uint32_t fileSize = 0;                  // of the one to use
  uint8_t head[kHeadBytes] = {};          // its first bytes
  uint32_t slotSize = 0;                  // the app slot it would go to; 0 = there is none
  uint8_t runningElf[kDigestBytes] = {};  // the build that runs now
  bool haveLast = false;                  // the last build the card installed is known ...
  uint8_t lastElf[kDigestBytes] = {};     // ... and this is it
  bool hasBattery = false;                // a battery is fitted (not "battery = none")
  int batteryMv = 0;                      // its voltage; 0 = unknown
};

struct Decision {
  Verdict verdict = Verdict::NOTHING;
  Problem problem = Problem::NONE;
};

inline Decision decide(const Inputs &in) {
  Decision d;
  if (in.files <= 0) return d;
  if (in.files > 1) {
    d.verdict = Verdict::SEVERAL;
    return d;
  }
  if (in.slotSize == 0) {
    d.verdict = Verdict::NO_SLOT;
    return d;
  }
  d.problem = inspectImage(in.head, kHeadBytes, in.fileSize, in.slotSize);
  if (d.problem != Problem::NONE) {
    d.verdict = Verdict::BAD_FILE;
    return d;
  }
  if (!memcmp(elfHash(in.head), in.runningElf, kDigestBytes)) {
    d.verdict = Verdict::SAME_AS_RUNNING;
    return d;
  }
  if (in.haveLast && !memcmp(elfHash(in.head), in.lastElf, kDigestBytes)) {
    d.verdict = Verdict::TRIED_BEFORE;
    return d;
  }
  if (in.hasBattery && in.batteryMv > 0 && in.batteryMv < kMinBatteryMv) {
    d.verdict = Verdict::LOW_BATTERY;
    return d;
  }
  d.verdict = Verdict::INSTALL;
  return d;
}

// What to tell the person at the clock (one line of the small heading type, 46 characters at most);
// empty for the verdicts that need no message.
inline void describe(const Decision &d, int batteryMv, char *out, size_t cap) {
  const char *s = "";
  switch (d.verdict) {
    case Verdict::SEVERAL: s = "More than one firmware file on the card"; break;
    case Verdict::NO_SLOT: s = "No second app slot: pick the 3MB APP scheme"; break;
    case Verdict::BAD_FILE:
      switch (d.problem) {
        case Problem::TOO_SMALL: s = "The file is too small to be the firmware"; break;
        case Problem::TOO_BIG: s = "Too big (the .merged.bin? use the .ino.bin)"; break;
        case Problem::NOT_AN_IMAGE: s = "The file is not a firmware image"; break;
        case Problem::WRONG_CHIP: s = "Built for another chip (need ESP32-S3)"; break;
        case Problem::NOT_AN_APP: s = "Not an app image (the bootloader?)"; break;
        case Problem::NO_DIGEST: s = "The image has no checksum: rebuild it"; break;
        default: break;
      }
      break;
    case Verdict::TRIED_BEFORE: s = "This build was installed from the card before"; break;
    case Verdict::LOW_BATTERY:
      snprintf(out, cap, "Battery %d.%02d V: plug in USB, then restart", batteryMv / 1000, (batteryMv % 1000) / 10);
      return;
    default: break;
  }
  snprintf(out, cap, "%s", s);
}

// ---------------------------------------------------------------------------
// Installing: the two passes over the file
// ---------------------------------------------------------------------------
// First the whole file is read and checked against the SHA-256 at its end, so that nothing is written until
// the copy on the card is known to be good; then it is read again and written to the other app slot.
//
// Written against three small interfaces, so that tools/tests runs this very code on a PC with a made-up
// file and a made-up flash (fw_update.cpp plugs in the card, mbedTLS and the core's Update class):
//
//   Source   bool readAt(uint32_t offset, void *buf, uint32_t n)   all n bytes from that place, or false
//   Hash     void update(const uint8_t *p, uint32_t n);  void finish(uint8_t *out32)
//   Sink     bool begin(uint32_t size);  bool write(uint8_t *p, uint32_t n);  bool end();
//            const char *error();  void abort()
//   progress(Pass, percent)
//
// Every read says where it reads from.  (The first version read "what comes next", after the caller had
// already read the header for a look: the check ran 208 bytes off and came up short at the very end, on
// every file.  It had only ever been compiled.)
enum class Pass : uint8_t { CHECK, WRITE };

const char *const kReadError = "Cannot read the file (card error)";
const char *const kDamaged = "The file is damaged: copy it again";
const char *const kTooSmall = "The file is too small to be the firmware";

// Returns nullptr when the new firmware is in place, else a short reason for the screen.
template <class Source, class Hash, class Sink, class Progress>
const char *installImage(Source &source, uint32_t size, Hash &hash, Sink &sink, uint8_t *buf, uint32_t chunk, Progress progress) {
  if (size <= kDigestBytes || chunk == 0) return kTooSmall;

  // 1. the file against its own checksum
  const uint32_t body = size - (uint32_t)kDigestBytes;
  progress(Pass::CHECK, 0);
  int shown = 0;
  for (uint32_t done = 0; done < body;) {
    const uint32_t n = body - done < chunk ? body - done : chunk;
    if (!source.readAt(done, buf, n)) return kReadError;
    hash.update(buf, n);
    done += n;
    const int pct = (int)((uint64_t)done * 100 / body);
    if (pct >= shown + 4 || (done == body && pct != shown)) {
      shown = pct;
      progress(Pass::CHECK, pct);
    }
  }
  uint8_t calc[kDigestBytes], trailer[kDigestBytes];
  hash.finish(calc);
  if (!source.readAt(body, trailer, (uint32_t)kDigestBytes)) return kReadError;
  if (memcmp(calc, trailer, kDigestBytes) != 0) return kDamaged;

  // 2. the other app slot: the whole file, checksum and all
  progress(Pass::WRITE, 0);
  if (!sink.begin(size)) return sink.error();
  const char *err = nullptr;
  shown = 0;
  for (uint32_t done = 0; done < size && !err;) {
    const uint32_t n = size - done < chunk ? size - done : chunk;
    if (!source.readAt(done, buf, n)) {
      err = kReadError;
    } else if (!sink.write(buf, n)) {
      err = sink.error();
    } else {
      done += n;
      const int pct = (int)((uint64_t)done * 100 / size);
      if (pct >= shown + 2 || (done == size && pct != shown)) {
        shown = pct;
        progress(Pass::WRITE, pct);
      }
    }
  }
  if (!err && !sink.end()) err = sink.error();
  if (err) sink.abort();
  return err;
}

}  // namespace fwlogic
