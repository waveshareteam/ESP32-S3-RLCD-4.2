#include "app_power_log.h"

#include <Preferences.h>
#include <diskio_wl.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_vfs_fat.h>
#include <wear_levelling.h>

#include "app_settings.h"
#include "config.h"
#include "log.h"
#include "sdcard.h"

namespace {

using powerlog::Status;

const char *const TAG = "plog";
const char *const kMountPoint = "/ffat";
const char *const kPartition = "ffat";  // the FAT partition of the Arduino core's partition schemes
const char *const kNamespace = "plog";  // flash: "used" = the clock has kept a log, the partition is its own
const size_t kChunk = 4096;             // what is read and written at a time when the whole log is copied

wl_handle_t s_wl = WL_INVALID_HANDLE;  // the mounted partition
bool s_mountTried = false;             // ... which is tried once a run
Status::Problem s_problem = Status::FINE;
powerlog::Schedule s_schedule;
uint32_t s_logBytes = 0;               // what the two files hold
uint32_t s_lastMs = 0, s_worstMs = 0;  // how long the writes took
Status::Copy s_copy = Status::COPY_NONE;
uint32_t s_copyBytes = 0;

bool everUsed() {
  Preferences p;
  if (!p.begin(kNamespace, true)) return false;  // nothing has ever been saved
  const bool used = p.getBool("used", false);
  p.end();
  return used;
}

void markUsed() {
  if (everUsed()) return;
  Preferences p;
  if (!p.begin(kNamespace, false)) return;
  p.putBool("used", true);
  p.end();
}

// Mounts the partition (once a run: one that will not mount now will not in ten minutes either).  `format`:
// make a file system if there is none, which is what the first use of the log does.
bool mountFlash(bool format) {
  if (s_wl != WL_INVALID_HANDLE) return true;
  if (s_mountTried) return false;
  s_mountTried = true;
  if (!esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, kPartition)) {
    s_problem = Status::NO_PARTITION;
    LOGF(TAG, "the flash has no FAT partition called \"%s\": the power log needs the partition scheme of the README", kPartition);
    return false;
  }
  esp_vfs_fat_mount_config_t cfg = {};
  cfg.format_if_mount_failed = format;
  cfg.max_files = 1;  // one file is open at a time, and every open file costs a sector of memory
  cfg.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  wl_handle_t wl = WL_INVALID_HANDLE;
  const uint32_t t0 = millis();
  const esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(kMountPoint, kPartition, &cfg, &wl);
  if (err != ESP_OK) {
    // (when the file system will not mount, the IDF takes its own registrations back but leaves the wear
    // levelling it started running, and the drive number it gave it on the books)
    if (wl != WL_INVALID_HANDLE) {
      ff_diskio_clear_pdrv_wl(wl);
      wl_unmount(wl);
    }
    s_problem = Status::CANNOT_MOUNT;
    LOGF(TAG, "the flash partition for the power log will not mount: 0x%x%s", (unsigned)err, format ? "" : " (not formatted: the log is off)");
    return false;
  }
  s_wl = wl;
  s_problem = Status::FINE;
  if (format) markUsed();
  LOGF(TAG, "flash partition mounted in %u ms", (unsigned)(millis() - t0));
  return true;
}

void unmountFlash() {
  if (s_wl == WL_INVALID_HANDLE) return;
  esp_vfs_fat_spiflash_unmount_rw_wl(kMountPoint, s_wl);
  s_wl = WL_INVALID_HANDLE;
  s_mountTried = false;
}

// The heading of this firmware's lines.
const char *header() {
  static char text[powerlog::kMaxLine] = "";
  if (!text[0]) powerlog::formatHeader(text, sizeof text);
  return text;
}

// Why the clock started, for the note on the line of a start.
const char *startNote() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "start (power on)";
    case ESP_RST_SW: return "start (restart)";
    case ESP_RST_DEEPSLEEP: return "start (after sleep)";  // the low-battery shutdown, over
    case ESP_RST_BROWNOUT: return "start (brownout)";
    case ESP_RST_PANIC: return "start (after a crash)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "start (watchdog)";
    case ESP_RST_USB: return "start (reset over USB)";
    default: return "start";
  }
}

// The copy to the card, as exportLog() sees it.
struct CardSink {
  int fd;
  bool failed = false;
  bool write(const char *data, uint32_t n) {
    if (!sdWrite(fd, data, n)) failed = true;
    return !failed;
  }
};

// The serial port.  It takes what fits in its buffer and no more, so what is left is offered again; if nothing
// has gone out for two seconds nobody is reading, and the rest is dropped.
struct PrintSink {
  Print &out;
  bool write(const char *data, uint32_t n) {
    uint32_t movedMs = millis();
    while (n > 0) {
      const uint32_t took = (uint32_t)out.write((const uint8_t *)data, n);
      if (took > 0) {
        data += took;
        n -= took > n ? n : took;
        movedMs = millis();
      } else if (millis() - movedMs >= 2000) {
        return false;
      } else {
        delay(5);
      }
    }
    return true;
  }
};

const char *problemText() {
  return s_problem == Status::NO_PARTITION ? "the flash has no \"ffat\" partition (use the partition scheme of the README)"
                                           : "the flash partition will not mount";
}

}  // namespace

void powerLogBegin() {
  if (!g_cfg.powerLog) return;
  if (!mountFlash(true)) {
    s_schedule.stop();  // nothing to write to: say so on the Power and settings page and leave it at that
    return;
  }
  powerlog::FolderFs fs(kMountPoint);
  s_logBytes = powerlog::logBytes(fs);
  LOGF(TAG, "power log on: %u KB so far, a line now and every %u min (type \"powerlog\" to read it)", (unsigned)((s_logBytes + 1023) / 1024),
       (unsigned)(powerlog::kIntervalSec / 60));
}

void powerLogCopyToCard(bool wasOn) {
  if (!g_cfg.powerLog && !wasOn) return;  // off, and not just switched off: the card is not written to
  const bool hadLog = everUsed();
  if (!g_cfg.powerLog && !hadLog) return;  // this clock has never kept a log: its flash partition is left alone
  if (!mountFlash(g_cfg.powerLog)) {
    if (hadLog) s_copy = Status::COPY_CANNOT_READ;
    return;
  }
  powerlog::FolderFs fs(kMountPoint);
  if (powerlog::logBytes(fs) > 0) {  // (an empty log is not worth replacing an earlier copy on the card with)
    char *buf = (char *)malloc(kChunk);
    CardSink sink{buf ? sdCreate(POWER_LOG_FILE_NAME) : -1};
    powerlog::Exported e;
    if (sink.fd >= 0) e = powerlog::exportLog(fs, header(), sink, buf, (uint32_t)kChunk);
    const bool closed = sdCommit(sink.fd);  // (always: it also closes the file)
    free(buf);
    if (e.ok && closed) {
      s_copy = Status::COPY_DONE;
      s_copyBytes = e.bytes;
      LOGF(TAG, "power log copied to the SD card: %s, %u bytes", POWER_LOG_FILE_NAME, (unsigned)e.bytes);
    } else {
      // the log could not be read to its end, or (all the rest) the card would not take it: half a file is no
      // use to anybody, and the log itself is still in the clock
      const bool readError = sink.fd >= 0 && !sink.failed && !e.ok;
      s_copy = readError ? Status::COPY_CANNOT_READ : Status::COPY_CARD_ERROR;
      if (sink.fd >= 0) sdRemove(POWER_LOG_FILE_NAME);
      LOGF(TAG, "power log NOT copied to the SD card: %s", readError ? "the log cannot be read" : "the card cannot be written");
    }
  }
  if (!g_cfg.powerLog) unmountFlash();  // the log is off: nothing more is written, and the memory is given back
}

bool powerLogDue(uint32_t nowSec) { return g_cfg.powerLog && s_wl != WL_INVALID_HANDLE && s_schedule.due(nowSec); }

void powerLogWrite(powerlog::Reading reading, uint32_t nowSec) {
  if (s_wl == WL_INVALID_HANDLE) return;
  if (s_schedule.startLine()) reading.note = startNote();
  const uint32_t t0 = millis();
  char line[powerlog::kMaxLine];
  powerlog::Appended a;
  if (powerlog::formatLine(reading, line, sizeof line) > 0) {
    powerlog::FolderFs fs(kMountPoint);
    a = powerlog::appendLine(fs, header(), line);
  }
  const bool ok = a.what != powerlog::Append::FAILED;
  s_schedule.done(nowSec, ok);
  s_lastMs = millis() - t0;
  if (s_lastMs > s_worstMs) s_worstMs = s_lastMs;
  if (ok) {
    s_logBytes = a.logBytes;
    LOGF(TAG, "line %u written in %u ms, %u KB in the log%s", (unsigned)s_schedule.lines(), (unsigned)s_lastMs, (unsigned)((s_logBytes + 1023) / 1024),
         a.what == powerlog::Append::ROTATED ? " (the oldest lines were dropped)" : "");
  } else {
    LOGF(TAG, "a line could NOT be written (%d in a row)%s", s_schedule.failures(), s_schedule.stopped() ? ": no more are tried until the next start" : "");
  }
}

powerlog::Status powerLogStatus(uint32_t nowSec) {
  Status s;
  s.on = g_cfg.powerLog;
  s.problem = s_problem;
  s.stopped = s_schedule.stopped() && s_problem == Status::FINE;
  s.failures = s_schedule.failures();
  s.lines = s_schedule.lines();
  s.logBytes = s_logBytes;
  s.sinceLineSec = nowSec - s_schedule.lastSec();
  s.copy = s_copy;
  s.copyBytes = s_copyBytes;
  return s;
}

void powerLogTimings(uint32_t *lastMs, uint32_t *worstMs) {
  *lastMs = s_lastMs;
  *worstMs = s_worstMs;
}

void powerLogCommand(const char *arg, Print &out) {
  while (*arg == ' ') arg++;
  const bool clear = !strcmp(arg, "clear");
  if (*arg && !clear) {
    out.println("usage:  powerlog   (prints the log as CSV)      powerlog clear   (forgets it)");
    return;
  }
  if (!g_cfg.powerLog && !everUsed()) {
    out.println("there is no power log: power_log has never been on (set power_log = on, or put it in the settings file)");
    return;
  }
  const bool wasMounted = s_wl != WL_INVALID_HANDLE;
  if (!mountFlash(g_cfg.powerLog)) {
    out.printf("the power log cannot be read: %s\n", problemText());
    return;
  }
  powerlog::FolderFs fs(kMountPoint);
  if (clear) {
    const bool gone = powerlog::clearLog(fs);
    s_logBytes = powerlog::logBytes(fs);
    out.println(gone ? "power log cleared" : "the power log could NOT be deleted");
  } else {
    char *buf = (char *)malloc(kChunk);
    powerlog::Exported e;
    if (buf) {
      consoleHold(true);  // no log line of another task in the middle of the file
      PrintSink sink{out};
      e = powerlog::exportLog(fs, header(), sink, buf, (uint32_t)kChunk);
      consoleHold(false);
      free(buf);
    }
    if (!e.ok) out.println("(cut short: the power log could not be read to its end, or nobody is reading this port)");
  }
  if (!wasMounted && !g_cfg.powerLog) unmountFlash();  // mounted for this only
}
