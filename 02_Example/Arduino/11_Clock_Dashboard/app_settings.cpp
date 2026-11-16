#include "app_settings.h"

#include <Preferences.h>
#include <math.h>

#include "app_power_log.h"
#include "build_defaults.h"
#include "config.h"
#include "fw_update.h"
#include "log.h"
#include "sdcard.h"
#include "util.h"

Settings g_cfg;
CfgStatus g_cfgStatus;

namespace {

const char *const TAG = "cfg";
const char *const kNamespace = "cfg";
const size_t kMaxFileBytes = 32 * 1024;  // a settings file is a few KB; more than this is not one
const size_t kExampleCap = 12 * 1024;

BuildReport s_build;  // what was wrong with config.h / secrets.h, found when the defaults were first worked out

// The measured clock drift, kept by the network task in the "clock" namespace, for the note in the
// example file.  Returns "" if nothing was measured yet.
void driftNote(char *out, size_t cap) {
  out[0] = 0;
  Preferences p;
  if (!p.begin("clock", true)) return;
  const float ppm = p.getFloat("dppm", NAN);
  const float hours = p.getFloat("dhours", 0);
  p.end();
  if (isnan(ppm) || hours < 3) return;
  snprintf(out, cap, "This clock measured itself while it had WiFi: %+.1f ppm, %+.2f s a day, over %.0f hours.\n"
                     "Without WiFi it would keep that error (and slowly change with temperature).",
           (double)ppm, (double)ppm * 0.0864, (double)hours);
}

// Writes the settings that came from flash or the card, and drops the ones that went back to their
// defaults.  True only if every step worked: a full or failing flash must not look like success, as the
// card then comes out and the settings are gone at the next restart.
bool saveToFlash(const Settings &s, bool clearFirst) {
  Preferences p;
  if (!p.begin(kNamespace, false)) return false;
  bool ok = true;
  if (clearFirst && !p.clear()) ok = false;
  char text[96];
  for (size_t i = 0; i < settingCount(); i++) {
    const char *key = settingNvsKey(i);
    if (settingIsUserSet(s, i)) {
      if (!formatSetting(s, i, text, sizeof text)) {  // too long to keep
        ok = false;
        continue;
      }
      // putString() returns the length it wrote, which is 0 for a failure and also for an empty string
      // that went in fine: for that one, look at what is stored
      const size_t n = p.putString(key, text);
      const bool stored = n == strlen(text) && (n > 0 || (p.isKey(key) && p.getString(key, "x").length() == 0));
      if (!stored) ok = false;
    } else if (p.isKey(key) && !p.remove(key)) {
      ok = false;
    }
  }
  p.end();
  return ok;
}

// Not a setting: how many settings a card has changed that no banner has told of yet (cfgAnnounced()).
const char *const kNoteKey = kSdNoteNvsKey;

int readNote() {
  Preferences p;
  if (!p.begin(kNamespace, true)) return 0;  // nothing has ever been saved
  const int n = p.isKey(kNoteKey) ? (int)p.getInt(kNoteKey, 0) : 0;
  p.end();
  return n > 0 ? n : 0;
}

void writeNote(int n) {  // 0 takes it away
  Preferences p;
  if (!p.begin(kNamespace, false)) return;
  if (n > 0) {
    p.putInt(kNoteKey, n);
  } else if (p.isKey(kNoteKey)) {
    p.remove(kNoteKey);
  }
  p.end();
}

// The last thing done with the card at start-up, once its settings are in force: a copy of the power log, when
// the clock keeps one.  `logWasOn`: it did until this card was read (which may just have switched it off).
void leaveCard(bool logWasOn) {
  powerLogCopyToCard(logWasOn);
  sdUnmount();
}

}  // namespace

Settings cfgBuildDefaults() {
  // Worked out once (it is asked for again whenever a setting is forgotten), and the problems are
  // logged once.
  static const Settings d = [] {
    const Settings s = buildDefaults(&s_build);
    for (int i = 0; i < s_build.kept; i++) LOGF(TAG, "built-in default not used: %s", s_build.text[i]);
    if (s_build.problems > s_build.kept) LOGF(TAG, "... and %d more", s_build.problems - s_build.kept);
    return s;
  }();
  return d;
}

int cfgBuildProblems() {
  cfgBuildDefaults();
  return s_build.problems;
}

bool cfgSaveToFlash() { return saveToFlash(g_cfg, false); }

void cfgLoadFlash() {
  g_cfg = cfgBuildDefaults();
  g_cfgStatus = CfgStatus();
  Preferences p;
  if (!p.begin(kNamespace, true)) return;  // nothing has ever been saved
  for (size_t i = 0; i < settingCount(); i++) {
    const char *key = settingNvsKey(i);
    if (!p.isKey(key)) continue;
    const String value = p.getString(key, "");
    if (applyStoredSetting(g_cfg, i, value.c_str())) g_cfgStatus.fromFlash++;
  }
  p.end();
}

void cfgImportSdCard() {
  CfgStatus &st = g_cfgStatus;
  const bool logWasOn = g_cfg.powerLog;  // (what the flash said, before a card has its say)
  st.carried = readNote();
  if (st.carried > 0) LOGF(TAG, "the start before this one changed %d setting(s) from a card and restarted before saying so", st.carried);
  const SdStatus mount = sdMount();
  if (mount == SdStatus::NO_CARD) {
    st.sd = CfgStatus::SD_NO_CARD;
    return;
  }
  if (mount == SdStatus::UNREADABLE) {
    st.sd = CfgStatus::SD_UNREADABLE;
    LOGF(TAG, "an SD card answered, but no FAT file system on it could be mounted: %s", sdProblemText());
    return;
  }
  sdCardSummary(st.card, sizeof st.card);
  fwScanCard();  // is there a firmware file on the card too?  (fwUpdateFromCard acts on it once the display is up)

  const char *names[] = {CONFIG_FILE_NAME, CONFIG_FILE_NAME_ALT};
  const char *found = nullptr;
  for (const char *n : names) {
    if (sdFileExists(n)) {
      found = n;
      break;
    }
  }

  if (!found) {  // a card with no settings file: leave an example (every line commented out)
    copyStr(st.file, sizeof st.file, CONFIG_FILE_NAME);
    char *buf = (char *)malloc(kExampleCap);
    char note[200];
    driftNote(note, sizeof note);
    const size_t n = buf ? renderExampleConfig(g_cfg, note, buf, kExampleCap, true) : 0;  // Windows line ends
    const bool ok = n > 0 && sdWriteFile(CONFIG_FILE_NAME, buf, n);
    free(buf);
    st.sd = ok ? CfgStatus::SD_EXAMPLE_WRITTEN : CfgStatus::SD_WRITE_FAILED;
    LOGF(TAG, "%s: %s on the %s card", CONFIG_FILE_NAME, ok ? "example file written" : "could not write the example file", st.card);
    leaveCard(logWasOn);
    return;
  }

  copyStr(st.file, sizeof st.file, found);
  char *data = nullptr;
  size_t len = 0;
  const bool read = sdReadFile(found, &data, &len, kMaxFileBytes);
  if (!read) {
    leaveCard(logWasOn);
    st.sd = CfgStatus::SD_READ_FAILED;
    LOGF(TAG, "could not read %s", found);
    return;
  }
  st.report = applyConfigText(g_cfg, cfgBuildDefaults(), data, len);
  free(data);
  st.sd = CfgStatus::SD_FILE_APPLIED;
  char sum[96];
  formatConfigSummary(st.report, sum, sizeof sum);
  LOGF(TAG, "%s: %s", found, sum);
  for (int i = 0; i < st.report.issueCount; i++) LOGF(TAG, "  %s", st.report.issues[i].text);

  if (st.report.touched() > 0) {  // keep what the card said, so the card can come out
    st.savedToFlash = saveToFlash(g_cfg, st.report.resetAll);
    LOGF(TAG, "settings %s flash", st.savedToFlash ? "saved to" : "could NOT be saved to");
    // ... and how many there were, until the banner has said so: with a firmware file on the card too, this
    // start ends in a restart before the banner, and the next one finds nothing left to change in the file.
    // (Settings that could not be saved are found changed again by the next start: nothing to carry.)
    if (st.savedToFlash) writeNote(st.carried + st.report.touched());
  }
  leaveCard(logWasOn);  // (the settings are safe in flash by now, whatever the card does next)
}

void cfgAnnounced() {
  if (readNote() > 0) writeNote(0);
}

void cfgSummary(char *out, size_t cap) {
  const CfgStatus &st = g_cfgStatus;
  char sum[96];
  switch (st.sd) {
    case CfgStatus::SD_NO_CARD:
    case CfgStatus::SD_NOT_TRIED:
      if (st.fromFlash > 0) {
        snprintf(out, cap, "%d saved in flash, no SD card", st.fromFlash);
      } else {
        snprintf(out, cap, "built-in defaults, no SD card");
      }
      break;
    case CfgStatus::SD_UNREADABLE: snprintf(out, cap, "%s", sdProblemText()); break;  // what the card holds instead
    case CfgStatus::SD_EXAMPLE_WRITTEN: snprintf(out, cap, "wrote an example file to the card"); break;
    case CfgStatus::SD_WRITE_FAILED: snprintf(out, cap, "SD card: cannot write (locked?)"); break;
    case CfgStatus::SD_READ_FAILED: snprintf(out, cap, "SD card: cannot read the file"); break;
    case CfgStatus::SD_FILE_APPLIED:
      formatConfigSummary(st.report, sum, sizeof sum);
      snprintf(out, cap, "SD: %s", sum);  // (the info page holds 42 characters of this)
      break;
  }
}

// The built-in defaults that were not used come first, then the problems of the SD file.
int cfgIssueCount() {
  cfgBuildDefaults();
  return s_build.kept + (g_cfgStatus.sd == CfgStatus::SD_FILE_APPLIED ? g_cfgStatus.report.issueCount : 0);
}

void cfgIssueText(int i, char *out, size_t cap) {
  if (i < 0 || i >= cfgIssueCount()) {
    if (cap) out[0] = 0;
    return;
  }
  if (i < s_build.kept) {
    copyStr(out, cap, s_build.text[i]);
  } else {
    copyStr(out, cap, g_cfgStatus.report.issues[i - s_build.kept].text);
  }
}

void cfgPrint(Print &out) {
  char v[100];
  for (size_t i = 0; i < settingCount(); i++) {
    if (settingIsSecret(i)) {
      formatSetting(g_cfg, i, v, sizeof v);
      snprintf(v, sizeof v, "%s", v[0] ? "(set)" : "(empty)");
    } else if (!formatSetting(g_cfg, i, v, sizeof v)) {
      snprintf(v, sizeof v, "?");
    }
    out.printf("  %-22s = %s%s\n", settingKey(i), v, settingIsUserSet(g_cfg, i) ? "" : "   (default)");
  }
}
