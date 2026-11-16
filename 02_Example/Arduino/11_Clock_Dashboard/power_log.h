#pragma once

// The power log: the readings of the two info pages, written down by the clock itself as one line of CSV at
// each start and every ten minutes after it, so that a battery run no longer has to be noted by hand (README:
// "Logging a battery run").  The columns are those of the sheet docs/power-log.csv, by the same names and in
// the same order, so a line can be pasted under the sheet's heading and one script reads both.  The ones the
// clock cannot know (the name of the run, whether the card is still in, what a meter reads) stay empty.
//
// This file is the part that runs on a PC as well, and tools/tests runs it there:
//   * the heading and the line (formatHeader, formatLine);
//   * when a line is due (Schedule);
//   * what happens to the files: a line appended, the oldest lines dropped once the log is full, the whole
//     log read back in one piece (appendLine, exportLog).  They are written against a small file interface,
//     so the very same steps run on a made-up file system that can be told to fail;
//   * the files of a real folder behind that interface (FolderFs: plain POSIX calls), which on the clock is
//     the mounted flash partition (app_power_log.cpp) and in the tests a folder of the PC;
//   * what the Power and settings page says about the log (statusRows).
//
// The log is kept in two files, so that dropping the oldest lines never means writing the log out again:
// lines go to kFile until it holds half of kMaxBytes; then kOldFile is deleted, kFile becomes kOldFile and a
// new kFile is begun.  What is kept is therefore the newest half to the whole of kMaxBytes.  Each file starts
// with the heading its lines were written under, and a file begun by a firmware with other columns is put
// aside the same way, so no line ever stands under a heading that is not its own.

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <initializer_list>

#include "app_model.h"
#include "battery_est.h"
#include "settings.h"

namespace powerlog {

const uint32_t kIntervalSec = 600;         // a line this often ...
const uint32_t kFirstSec = 10;             // ... and one this long after a start, when the first frames are out
const uint32_t kMaxBytes = 1024 * 1024;    // the two files together never hold more
const int kMaxFailures = 3;                // writes in a row that may fail before the log gives up for this run
const size_t kMaxLine = 512;               // a line, or the heading, with its line end and a terminator
// A write takes a tenth of a second or a few, and has to be over before the coming second is drawn
// (frame_plan.h): it only starts this early in a second.
const int32_t kLatestStartUs = 200000;

const char *const kFile = "powerlog.csv";     // the newest lines
const char *const kOldFile = "powerlog.old";  // the ones before them

// ---------------------------------------------------------------------------
// The columns: those of docs/power-log.csv (a test compares the two headings)
// ---------------------------------------------------------------------------
enum Column : uint8_t {
  C_TEST = 0,  // (not known to the clock: the name of the run)
  C_FIRMWARE,
  C_WIFI,  // the eight settings a run is made with, by the names and in the words of the settings file
  C_WIFI_MODE,
  C_WIFI_POWER_SAVE,
  C_CPU_IDLE_MHZ,
  C_SPOTIFY_LIVE,
  C_CONSOLE,
  C_LIGHT_SLEEP,
  C_DISPLAY_POWER,
  C_BATTERY_CAPACITY_MAH,
  C_CARD,  // (not known: the card is only looked at while the clock starts)
  C_MUSIC,
  C_DATE,
  C_TIME,
  C_UPTIME,
  C_CLOCK_SHOWN,
  C_RADIO_ON_PCT,
  C_RADIO_SESSIONS,
  C_ASLEEP_PCT,
  C_BATTERY_V,
  C_BATTERY_PCT,
  C_BATTERY_STATE,
  C_LEFT,
  C_PCT_PER_HOUR,
  C_OVER_MIN,
  C_CLOCK_MA,
  C_METER_MA,  // (not known: what a meter in the supply reads)
  C_FREE_KB,
  C_FRAMES_SENT,
  C_FRAMES_LATE,
  C_FRAMES_WORST_MS,
  C_NOTES,
  kColumnCount
};

inline const char *columnName(int c) {
  static const char *const names[] = {
      "test",           "firmware",     "wifi",        "wifi_mode",  "wifi_power_save", "cpu_idle_mhz",
      "spotify_live",   "console",      "light_sleep", "display_power", "battery_capacity_mah",     "card",
      "music",          "date",         "time",        "uptime",     "clock_shown",     "radio_on_pct",
      "radio_sessions", "asleep_pct",   "battery_v",   "battery_pct", "battery_state",  "left",
      "pct_per_hour",   "over_min",     "clock_ma",    "meter_ma",   "free_kb",         "frames_sent",
      "frames_late",    "frames_worst_ms", "notes"};
  static_assert(sizeof names / sizeof names[0] == kColumnCount, "one name for each column");
  return (c >= 0 && c < kColumnCount) ? names[c] : "";
}

// What the clock knew when the line was taken.  Plain values: the text is made by formatLine().
struct Reading {
  const char *version = "";            // APP_VERSION
  const char *buildId = "";            // the build id of the System info page
  const Settings *settings = nullptr;  // the settings in force
  SpotifyStatus spotify = SPOTIFY_DISABLED;

  bool timeValid = false;  // the clock is trusted: `local` is the date and time
  struct tm local = {};
  uint32_t uptimeSec = 0;

  int cpuMhz = 0;                // the CPU clock right now ...
  const char *clockReason = "";  // ... and why it is there ("idle", "radio on", ...); "" when no idle clock is set

  bool radioSync = false;  // wifi_mode = sync with the radio in use: the two figures below mean something
  uint16_t radioOnPermille = 0;
  uint32_t radioSessions = 0;

  uint16_t asleepPermille = 0;  // light_sleep = on: the share of the time asleep (light_sleep.h), in tenths of a percent

  bool batteryPresent = false;
  float batteryVolts = 0;
  int charge = CHARGE_UNKNOWN;   // UiCharge
  bool chargeWarmingUp = false;  // the charge detector: the first minutes after a start are not used ...
  bool chargeReady = false;      // ... and it has enough readings to judge by
  battest::Estimate estimate;    // the runtime estimate (state OFF when there is none)

  uint32_t freeHeapBytes = 0;
  uint32_t framesSent = 0, framesLate = 0;
  int32_t framesWorstMs = 0;

  const char *note = "";  // "start (power on)" on the line of a start
};

// The setting `key` as the settings file has it ("sync", "20", "off"); "" if there is no such setting.
inline void settingText(const Settings &s, const char *key, char *out, size_t cap) {
  if (cap == 0) return;
  out[0] = 0;
  for (size_t i = 0; i < settingCount(); i++) {
    if (strcmp(settingKey(i), key) != 0) continue;
    if (!formatSetting(s, i, out, cap)) out[0] = 0;
    return;
  }
}

// What the System info page says the battery is doing, in a word or two.
inline const char *batteryStateText(bool present, int charge, bool warmingUp, bool ready) {
  if (!present) return "none";
  switch (charge) {
    case CHARGE_CHARGING: return "charging";
    case CHARGE_DISCHARGING: return "discharging";
    case CHARGE_FULL: return "full";
    default: break;
  }
  // nothing seen of a charger, and no fall seen either (under a light load that is how it stays)
  return warmingUp ? "starting up" : (ready ? "on battery?" : "learning");
}

// One field as CSV wants it: as it is, or in quotes (with the quotes inside doubled) when it holds a comma,
// a quote or a line end.  Returns the length, or 0 if it does not fit (an empty field is 0 too, and fits).
inline size_t csvField(const char *text, char *out, size_t cap) {
  const bool quote = strpbrk(text, ",\"\r\n") != nullptr;
  size_t n = 0;
  auto put = [&](char c) {
    if (n + 1 < cap) out[n] = c;
    n++;
  };
  if (quote) put('"');
  for (const char *p = text; *p; ++p) {
    if (*p == '"') put('"');
    put(*p);
  }
  if (quote) put('"');
  if (cap == 0 || n + 1 > cap) return 0;
  out[n] = 0;
  return n;
}

const char *const kLineEnd = "\r\n";  // what spreadsheets and Notepad expect of a CSV file

// The heading: the column names.  Returns the length, or 0 if `cap` is too small.
inline size_t formatHeader(char *out, size_t cap) {
  size_t n = 0;
  for (int c = 0; c < kColumnCount; c++) {
    const int w = snprintf(out + n, cap - n, "%s%s", c ? "," : "", columnName(c));
    if (w < 0 || n + (size_t)w >= cap) return 0;
    n += (size_t)w;
  }
  const int w = snprintf(out + n, cap - n, "%s", kLineEnd);
  if (w < 0 || n + (size_t)w >= cap) return 0;
  return n + (size_t)w;
}

// One line.  Returns the length, or 0 if `cap` is too small (nothing useful is in `out` then).
inline size_t formatLine(const Reading &r, char *out, size_t cap) {
  const size_t kField = 32;
  char f[kColumnCount][kField];
  for (int c = 0; c < kColumnCount; c++) f[c][0] = 0;

  // which firmware, which settings
  snprintf(f[C_FIRMWARE], kField, "%s%s%s", r.version, r.buildId[0] ? " " : "", r.buildId);
  if (r.settings) {
    for (int c : {C_WIFI, C_WIFI_MODE, C_WIFI_POWER_SAVE, C_CPU_IDLE_MHZ, C_SPOTIFY_LIVE, C_CONSOLE, C_LIGHT_SLEEP,
                  C_DISPLAY_POWER}) {
      settingText(*r.settings, columnName(c), f[c], kField);  // (the column is named after its setting)
    }
    // (0 means "not known", and what is not known is left empty)
    if (r.settings->batteryCapacityMah > 0) snprintf(f[C_BATTERY_CAPACITY_MAH], kField, "%d", (int)r.settings->batteryCapacityMah);
  }
  switch (r.spotify) {  // what Spotify said last; with no account linked the clock does not know
    case SPOTIFY_PLAYING: snprintf(f[C_MUSIC], kField, "playing"); break;
    case SPOTIFY_PAUSED: snprintf(f[C_MUSIC], kField, "paused"); break;
    case SPOTIFY_IDLE: snprintf(f[C_MUSIC], kField, "none"); break;
    default: break;
  }

  // when
  if (r.timeValid) {
    snprintf(f[C_DATE], kField, "%04d-%02d-%02d", (r.local.tm_year + 1900) % 10000, (r.local.tm_mon + 1) % 100, r.local.tm_mday % 100);
    snprintf(f[C_TIME], kField, "%02d:%02d", r.local.tm_hour % 100, r.local.tm_min % 100);
  }
  snprintf(f[C_UPTIME], kField, "%ud %uh %um", (unsigned)(r.uptimeSec / 86400), (unsigned)((r.uptimeSec / 3600) % 24),
           (unsigned)((r.uptimeSec / 60) % 60));

  // the CPU clock, the radio and the light sleep
  if (r.cpuMhz > 0) snprintf(f[C_CLOCK_SHOWN], kField, "%d%s%.20s", r.cpuMhz % 1000, r.clockReason[0] ? " " : "", r.clockReason);
  if (r.radioSync) {
    snprintf(f[C_RADIO_ON_PCT], kField, "%.1f", r.radioOnPermille / 10.0);
    snprintf(f[C_RADIO_SESSIONS], kField, "%u", (unsigned)r.radioSessions);
  }
  if (r.settings && r.settings->lightSleep) {  // (as the Sleep line: only with light sleep on)
    const unsigned p = r.asleepPermille > 1000 ? 1000 : r.asleepPermille;
    snprintf(f[C_ASLEEP_PCT], kField, "%u.%u", p / 10, p % 10);
  }

  // the battery, and what the runtime estimate makes of it (the Left and Current lines of the pages)
  snprintf(f[C_BATTERY_STATE], kField, "%s", batteryStateText(r.batteryPresent, r.charge, r.chargeWarmingUp, r.chargeReady));
  if (r.batteryPresent) {
    snprintf(f[C_BATTERY_V], kField, "%.3f", (double)r.batteryVolts);
    snprintf(f[C_BATTERY_PCT], kField, "%.1f", (double)calc::batteryPercentF(r.batteryVolts));
    const battest::Estimate &e = r.estimate;
    if (r.charge != CHARGE_CHARGING && r.charge != CHARGE_FULL && e.state == battest::Estimate::READY) {
      if (e.pctPerHour < battest::kMinRatePctPerHour) {
        snprintf(f[C_LEFT], kField, ">30 d");  // no fall that can be measured: the page says "more than a month"
      } else {
        battest::formatRemaining(e.hoursLeft, f[C_LEFT], kField);
      }
      if (e.pctPerHour < 9.995f) {  // two decimals where the page has two
        snprintf(f[C_PCT_PER_HOUR], kField, "%.2f", (double)e.pctPerHour);
      } else {
        snprintf(f[C_PCT_PER_HOUR], kField, "%.1f", (double)e.pctPerHour);
      }
      snprintf(f[C_OVER_MIN], kField, "%d", e.windowMin);
      if (r.settings && r.settings->batteryCapacityMah > 0) {  // the current needs the capacity
        if (e.avgMa < 99.95f) {
          snprintf(f[C_CLOCK_MA], kField, "%.1f", (double)e.avgMa);
        } else {
          snprintf(f[C_CLOCK_MA], kField, "%.0f", (double)e.avgMa);
        }
      }
    }
  }

  // memory, and whether the seconds came on time
  snprintf(f[C_FREE_KB], kField, "%u", (unsigned)(r.freeHeapBytes / 1024));
  snprintf(f[C_FRAMES_SENT], kField, "%u", (unsigned)r.framesSent);
  snprintf(f[C_FRAMES_LATE], kField, "%u", (unsigned)r.framesLate);
  snprintf(f[C_FRAMES_WORST_MS], kField, "%d", (int)r.framesWorstMs);
  snprintf(f[C_NOTES], kField, "%s", r.note);

  size_t n = 0;
  for (int c = 0; c < kColumnCount; c++) {
    if (c) {
      if (n + 1 >= cap) return 0;
      out[n++] = ',';
    }
    if (!f[c][0]) continue;
    const size_t w = csvField(f[c], out + n, cap - n);
    if (w == 0) return 0;
    n += w;
  }
  const size_t endLen = strlen(kLineEnd);
  if (n + endLen >= cap) return 0;
  memcpy(out + n, kLineEnd, endLen + 1);
  return n + endLen;
}

// ---------------------------------------------------------------------------
// When a line is due
// ---------------------------------------------------------------------------
// The first line kFirstSec after the start, the next ones kIntervalSec after the one before: the flash is
// never written more often than that, also when a write fails (the next try is a whole interval later).
// After kMaxFailures failed writes in a row the log gives up until the next start.  Times are seconds since
// the start; only differences are taken, so a counter that runs over does no harm.
class Schedule {
 public:
  bool due(uint32_t nowSec) const {
    if (stopped_) return false;
    return begun_ ? (uint32_t)(nowSec - lastSec_) >= kIntervalSec : nowSec >= kFirstSec;
  }
  bool startLine() const { return !begun_; }  // the line that comes next is the one of the start

  // A line was written (`ok`), or could not be.
  void done(uint32_t nowSec, bool ok) {
    begun_ = true;
    lastSec_ = nowSec;
    if (ok) {
      failures_ = 0;
      lines_++;
    } else if (++failures_ >= kMaxFailures) {
      stopped_ = true;
    }
  }
  void stop() { stopped_ = true; }  // nothing to write to: no line is ever due

  bool stopped() const { return stopped_; }
  int failures() const { return failures_; }
  uint32_t lines() const { return lines_; }       // lines written since the start
  uint32_t lastSec() const { return lastSec_; }   // when the last one was written or tried

 private:
  bool begun_ = false, stopped_ = false;
  uint32_t lastSec_ = 0, lines_ = 0;
  int failures_ = 0;
};

// ---------------------------------------------------------------------------
// The files
// ---------------------------------------------------------------------------
// Written against two small interfaces (file names are those above, in one folder):
//
//   Fs    long size(name)                         the file's length, or -1 if there is no such file
//         long read(name, offset, buf, n)         the bytes read: n, or fewer at the end of the file; -1 on an error
//         bool append(name, data, n)              at the end of the file, which is made if there is none; true only
//                                                 when all of it is in the file for good
//         bool remove(name)                       true when the file is gone, also when there was none
//         bool rename(from, to)                   there is no file `to`
//   Sink  bool write(data, n)
//
// Every read names its place in the file, and nothing is kept open between two calls.

enum class Append : uint8_t { FAILED = 0, OK, ROTATED };  // ROTATED: written, and the oldest lines were dropped first
struct Appended {
  Append what = Append::FAILED;
  uint32_t logBytes = 0;  // what the two files hold afterwards
};

// What the two files hold, in bytes.
template <class Fs>
uint32_t logBytes(Fs &fs) {
  const long a = fs.size(kFile), b = fs.size(kOldFile);
  return (uint32_t)(a > 0 ? a : 0) + (uint32_t)(b > 0 ? b : 0);
}

// Adds `line` (with its line end) to the log; `header` is the heading of this firmware's lines (with its line
// end).  A new file begins with the heading.
template <class Fs>
Appended appendLine(Fs &fs, const char *header, const char *line, uint32_t maxBytes = kMaxBytes) {
  Appended res;
  const uint32_t headerLen = (uint32_t)strlen(header), lineLen = (uint32_t)strlen(line);
  if (headerLen == 0 || headerLen >= kMaxLine || lineLen == 0 || lineLen >= kMaxLine) return res;

  long size = fs.size(kFile);
  bool rotate = false, midLine = false;
  if (size > 0) {
    char head[kMaxLine];
    const long got = fs.read(kFile, 0, head, headerLen);
    if (got < 0) return res;
    if ((uint32_t)got != headerLen || memcmp(head, header, headerLen) != 0) {
      rotate = true;  // begun under another heading (a firmware with other columns): those lines keep it, in the old file
    } else if ((uint64_t)size + lineLen > maxBytes / 2) {
      rotate = true;  // full
    } else {
      char last = 0;
      if (fs.read(kFile, (uint32_t)size - 1, &last, 1) != 1) return res;
      midLine = last != '\n';  // a write that was cut short left part of a line: the new one starts on a line of its own
    }
  }
  if (rotate) {
    if (!fs.remove(kOldFile) || !fs.rename(kFile, kOldFile)) return res;
    size = 0;
  }

  char buf[2 * kMaxLine];
  uint32_t n = 0;
  if (size <= 0) {
    memcpy(buf, header, headerLen);
    n = headerLen;
  } else if (midLine) {
    n = (uint32_t)strlen(kLineEnd);
    memcpy(buf, kLineEnd, n);
  }
  memcpy(buf + n, line, lineLen);
  n += lineLen;
  if (!fs.append(kFile, buf, n)) return res;

  res.what = rotate ? Append::ROTATED : Append::OK;
  const long old = fs.size(kOldFile);
  res.logBytes = (uint32_t)(size > 0 ? size : 0) + n + (uint32_t)(old > 0 ? old : 0);
  return res;
}

struct Exported {
  bool ok = false;
  uint32_t bytes = 0;  // what went to the sink
};

// The whole log, oldest line first, as one CSV file: the old file, then the new one.  A file's heading is
// left out when it is the heading that is already in force, so that the usual log has one heading, at the top;
// lines under other columns come with their own.  An empty log is the heading alone.  `buf` (at least
// kMaxLine bytes) is the piece that is read and written at a time.
template <class Fs, class Sink>
Exported exportLog(Fs &fs, const char *header, Sink &sink, char *buf, uint32_t cap) {
  Exported out;
  if (cap < kMaxLine) return out;
  char inForce[kMaxLine] = "";  // the heading that was put out last
  uint32_t inForceLen = 0;
  bool any = false, atLineStart = true;
  auto put = [&](const char *p, uint32_t n) -> bool {
    if (n == 0) return true;
    if (!sink.write(p, n)) return false;
    out.bytes += n;
    atLineStart = p[n - 1] == '\n';
    return true;
  };

  for (const char *name : {kOldFile, kFile}) {
    const long size = fs.size(name);
    if (size <= 0) continue;
    long got = fs.read(name, 0, buf, (uint32_t)kMaxLine);
    if (got <= 0) return out;
    uint32_t headLen = 0;  // the file's first line with its line end (0: it has none, so there is no heading to speak of)
    for (long i = 0; i < got; i++) {
      if (buf[i] == '\n') {
        headLen = (uint32_t)i + 1;
        break;
      }
    }
    uint32_t pos = 0;
    if (headLen > 0 && headLen == inForceLen && memcmp(buf, inForce, headLen) == 0) {
      pos = headLen;  // the same columns as the lines before: once is enough
    } else {
      memcpy(inForce, buf, headLen);
      inForceLen = headLen;
    }
    if (!atLineStart && !put(kLineEnd, (uint32_t)strlen(kLineEnd))) return out;  // (the file before ended in the middle of a line)
    while (pos < (uint32_t)size) {
      const uint32_t want = (uint32_t)size - pos < cap ? (uint32_t)size - pos : cap;
      got = fs.read(name, pos, buf, want);
      if (got <= 0) return out;
      if (!put(buf, (uint32_t)got)) return out;
      pos += (uint32_t)got;
    }
    any = true;
  }
  if (!any && !put(header, (uint32_t)strlen(header))) return out;
  out.ok = true;
  return out;
}

// Forgets the log.  True when both files are gone.
template <class Fs>
bool clearLog(Fs &fs) {
  const bool a = fs.remove(kFile), b = fs.remove(kOldFile);
  return a && b;
}

// ---------------------------------------------------------------------------
// The files of a folder
// ---------------------------------------------------------------------------
// The `Fs` of the steps above for a real folder, through the POSIX calls: on the clock the folder is the mounted
// flash partition, in tools/tests one on the PC, so the calls themselves (which flags, which place in the file,
// what counts as an error) have run before they run on the board.  Every call opens and closes what it needs:
// no file is open between two of them, and nothing is left unwritten.
class FolderFs {
 public:
  explicit FolderFs(const char *folder) : folder_(folder) {}

  long size(const char *name) const {
    char path[kMaxPath];
    struct stat st;
    return (pathFor(name, path) && stat(path, &st) == 0 && S_ISREG(st.st_mode)) ? (long)st.st_size : -1;
  }
  long read(const char *name, uint32_t offset, char *buf, uint32_t n) const {
    char path[kMaxPath];
    if (!pathFor(name, path)) return -1;
    const int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    long got = -1;
    if (lseek(fd, (off_t)offset, SEEK_SET) == (off_t)offset) got = (long)::read(fd, buf, n);
    close(fd);
    return got;
  }
  bool append(const char *name, const char *data, uint32_t n) const {
    char path[kMaxPath];
    if (!pathFor(name, path)) return false;
    const int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) return false;
    const bool written = ::write(fd, data, n) == (ssize_t)n;
    const bool flushed = fsync(fd) == 0;  // the data and the file's new length: all of it on the disk
    const bool closed = close(fd) == 0;
    return written && flushed && closed;
  }
  bool remove(const char *name) const {
    char path[kMaxPath];
    if (!pathFor(name, path)) return false;
    return unlink(path) == 0 || errno == ENOENT;
  }
  bool rename(const char *from, const char *to) const {
    char a[kMaxPath], b[kMaxPath];
    return pathFor(from, a) && pathFor(to, b) && ::rename(a, b) == 0;
  }

 private:
  static constexpr size_t kMaxPath = 128;
  // False if the path would not fit (a path that is cut short names another file).
  bool pathFor(const char *name, char *out) const {
    const int n = snprintf(out, kMaxPath, "%s/%s", folder_, name);
    return n > 0 && (size_t)n < kMaxPath;
  }
  const char *folder_;
};

// ---------------------------------------------------------------------------
// What the Power and settings page says (its "Log" row, and a second row about the copy to the SD card)
// ---------------------------------------------------------------------------
struct Status {
  enum Problem : uint8_t { FINE = 0, NO_PARTITION, CANNOT_MOUNT };
  enum Copy : uint8_t { COPY_NONE = 0, COPY_DONE, COPY_CANNOT_READ, COPY_CARD_ERROR };

  bool on = false;             // the power_log setting
  Problem problem = FINE;      // why nothing can be written at all
  bool stopped = false;        // given up after kMaxFailures failed writes
  int failures = 0;            // failed writes in a row
  uint32_t lines = 0;          // lines written since the start
  uint32_t logBytes = 0;       // what the log holds
  uint32_t sinceLineSec = 0;   // seconds since the last line
  Copy copy = COPY_NONE;       // what became of the copy to the SD card at this start
  uint32_t copyBytes = 0;
};

const size_t kStatusRow = 43;  // an info page shows 42 characters of a value

// Fills up to two rows and returns how many: the state of the log (only when it is on), then the copy to the
// card (only when one was tried at this start).
//
// The numbers are kept to as many digits as the row has room for (a megabyte is 1024 KB, and 99999 lines are
// two years of them), so that every text fits whatever it is given.
inline int statusRows(const Status &s, char rows[2][kStatusRow]) {
  auto atMost = [](uint32_t v, uint32_t top) { return (unsigned)(v > top ? top : v); };
  const unsigned lines = atMost(s.lines, 99999);
  int n = 0;
  if (s.on) {
    char *o = rows[n++];
    if (s.problem == Status::NO_PARTITION) {
      snprintf(o, kStatusRow, "OFF: no ffat partition (see the README)");
    } else if (s.problem == Status::CANNOT_MOUNT) {
      snprintf(o, kStatusRow, "OFF: the flash partition will not mount");
    } else if (s.stopped) {
      snprintf(o, kStatusRow, "STOPPED after %u write errors (%u lines)", atMost(s.failures > 0 ? (uint32_t)s.failures : 0, 9), lines);
    } else if (s.failures > 0) {
      snprintf(o, kStatusRow, "%u line%s this run, last write FAILED", lines, lines == 1 ? "" : "s");
    } else if (s.lines == 0) {
      snprintf(o, kStatusRow, "on: first line in a moment");
    } else {
      char ago[8];
      if (s.sinceLineSec < 90) {
        snprintf(ago, sizeof ago, "%u s", atMost(s.sinceLineSec, 89));
      } else if (s.sinceLineSec < 5400) {
        snprintf(ago, sizeof ago, "%u min", atMost((s.sinceLineSec + 30) / 60, 90));
      } else {
        snprintf(ago, sizeof ago, "%u h", atMost(s.sinceLineSec / 3600, 999));
      }
      snprintf(o, kStatusRow, "%u KB, %u line%s this run, %s ago", atMost((s.logBytes + 1023) / 1024, 9999), lines, lines == 1 ? "" : "s", ago);
    }
  }
  if (s.copy != Status::COPY_NONE) {
    char *o = rows[n];
    const char *lead = n == 0 ? "off, " : "";  // (the log is off, but there was one to copy)
    n++;
    if (s.copy == Status::COPY_DONE) {
      snprintf(o, kStatusRow, "%scopied to SD card at start (%u KB)", lead, atMost((s.copyBytes + 1023) / 1024, 9999));
    } else if (s.copy == Status::COPY_CANNOT_READ) {
      snprintf(o, kStatusRow, "%sNOT copied to SD: cannot read the log", lead);
    } else {
      snprintf(o, kStatusRow, "%sNOT copied to SD: card write error", lead);
    }
  }
  return n;
}

}  // namespace powerlog
