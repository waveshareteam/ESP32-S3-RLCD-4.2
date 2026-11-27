#include "fw_update.h"

#include <Preferences.h>
#include <Update.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <mbedtls/sha256.h>

#include "app_settings.h"
#include "config.h"
#include "fw_logic.h"
#include "log.h"
#include "sdcard.h"
#include "sensors.h"
#include "util.h"

// The core confirms a freshly installed image the moment it starts unless the sketch says it will do it
// later.  This one does it after FIRMWARE_TRIAL_MS (fwTrialTick), so a build that dies on the way up is
// undone by the bootloader at the next reset.  (Overrides the core's weak function of the same name.)
extern "C" bool verifyRollbackLater() { return true; }

namespace {

const char *const TAG = "fw";
const char *const kNamespace = "fw";  // flash: "last" = the build the card installed last
const size_t kChunk = 4096;

char s_name[40] = "";     // the firmware file found on the card (the first of them)
int s_count = 0;          // how many there are
uint32_t s_size = 0;      // size of the first
bool s_trial = false;     // the running image is on trial
bool s_rolledBack = false;

const char *stateName(esp_ota_img_states_t s) {
  switch (s) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "on trial";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "rolled back";
    default: return "undefined";
  }
}

void toHex(const uint8_t *in, size_t n, char *out) {
  static const char *const digits = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    out[2 * i] = digits[in[i] >> 4];
    out[2 * i + 1] = digits[in[i] & 15];
  }
  out[2 * n] = 0;
}

bool fromHex(const char *s, uint8_t *out, size_t n) {
  for (size_t i = 0; i < n; i++) {
    int v = 0;
    for (int k = 0; k < 2; k++) {
      const char c = s[2 * i + k];
      int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
      if (d < 0) return false;
      v = v * 16 + d;
    }
    out[i] = (uint8_t)v;
  }
  return true;
}

// The build in a partition: the ELF hash in its app descriptor.
bool partitionElf(const esp_partition_t *p, uint8_t *out) {
  esp_app_desc_t desc;
  if (!p || esp_ota_get_partition_description(p, &desc) != ESP_OK) return false;
  memcpy(out, desc.app_elf_sha256, fwlogic::kDigestBytes);
  return true;
}

bool loadLast(uint8_t *out) {
  Preferences p;
  if (!p.begin(kNamespace, true)) return false;
  const String hex = p.getString("last", "");
  p.end();
  return hex.length() == 2 * fwlogic::kDigestBytes && fromHex(hex.c_str(), out, fwlogic::kDigestBytes);
}

void saveLast(const uint8_t *elf) {
  Preferences p;
  if (!p.begin(kNamespace, false)) return;
  char hex[2 * fwlogic::kDigestBytes + 1];
  toHex(elf, fwlogic::kDigestBytes, hex);
  p.putString("last", hex);
  p.end();
}

void say(FwShowFn show, UiFwKind kind, const char *what, const char *detail, int percent, bool keepPowered) {
  UiFwScreen s;
  s.kind = kind;
  copyStr(s.what, sizeof s.what, what);
  copyStr(s.detail, sizeof s.detail, detail);
  s.percent = percent;
  s.keepPowered = keepPowered;
  show(s);
}

// What fwlogic::installImage() works with on the clock: the file on the card, mbedTLS's SHA-256, and the
// other app slot by way of the core's Update class.

// Every read names its place in the file, so nothing depends on what was read before.
struct CardFile {
  int fd;
  bool readAt(uint32_t offset, void *buf, uint32_t n) { return sdSeek(fd, offset) && sdRead(fd, buf, n) == (int)n; }
};

struct Sha256 {
  mbedtls_sha256_context ctx;
  Sha256() {
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
  }
  ~Sha256() { mbedtls_sha256_free(&ctx); }
  void update(const uint8_t *p, uint32_t n) { mbedtls_sha256_update(&ctx, p, n); }
  void finish(uint8_t *out) { mbedtls_sha256_finish(&ctx, out); }
};

// The Update class holds back the first bytes of the image until the end, so a half-written slot cannot
// boot, and switches the boot slot only after the image has been verified.
struct OtherSlot {
  bool begin(uint32_t size) { return Update.begin(size, U_FLASH); }
  bool write(uint8_t *p, uint32_t n) { return Update.write(p, n) == n; }
  bool end() { return Update.end(); }
  const char *error() { return Update.errorString(); }
  void abort() { Update.abort(); }
};

// Checks the whole file against the SHA-256 at its end, then writes it to the other app slot.  Returns
// nullptr when the new firmware is in place, else a short reason.
const char *installFrom(int fd, uint32_t size, const char *detail, FwShowFn show) {
  uint8_t *buf = (uint8_t *)malloc(kChunk);
  if (!buf) return "Out of memory";
  CardFile file{fd};
  Sha256 sha;
  OtherSlot slot;
  const char *err = fwlogic::installImage(file, size, sha, slot, buf, (uint32_t)kChunk, [&](fwlogic::Pass pass, int percent) {
    say(show, FW_BUSY, pass == fwlogic::Pass::CHECK ? "Checking the file" : "Installing the new firmware", detail, percent, true);
  });
  free(buf);
  return err;
}

}  // namespace

const char *fwBuildId() {
  static char id[8] = "";
  if (!id[0]) esp_app_get_elf_sha256(id, sizeof id);  // (as many hex digits as fit: seven)
  return id;
}

void fwBegin() {
  const esp_partition_t *run = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  s_trial = run && esp_ota_get_state_partition(run, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
  s_rolledBack = esp_ota_get_last_invalid_partition() != nullptr;
  if (s_trial) LOGF(TAG, "this firmware is on trial: kept once it has run %lu s", (unsigned long)(FIRMWARE_TRIAL_MS / 1000));
  if (s_rolledBack) LOGF(TAG, "the other app slot holds an image that was rolled back");
}

void fwScanCard() {
  s_count = 0;
  s_name[0] = 0;
  s_size = 0;
  const char *const names[] = {FIRMWARE_FILE_NAME, FIRMWARE_FILE_NAME_ALT, FIRMWARE_FILE_EXPORT};
  for (const char *n : names) {
    if (!sdFileExists(n)) continue;
    if (++s_count == 1) {
      copyStr(s_name, sizeof s_name, n);
      sdFileSize(n, &s_size);
    }
  }
  if (s_count) LOGF(TAG, "firmware file on the card: %s, %u bytes%s", s_name, (unsigned)s_size, s_count > 1 ? " (and more)" : "");
}

void fwUpdateFromCard(FwShowFn show) {
  if (s_count == 0) return;

  fwlogic::Inputs in;
  in.files = s_count;
  in.fileSize = s_size;
  const esp_partition_t *run = esp_ota_get_running_partition();
  const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
  in.slotSize = (next && next != run) ? next->size : 0;
  partitionElf(run, in.runningElf);
  in.haveLast = loadLast(in.lastElf);
  in.hasBattery = g_cfg.hasBattery();
  if (in.hasBattery) {
    const BatteryReading b = readBattery();
    in.batteryMv = b.present ? (int)lroundf(b.volts * 1000.0f) : 0;
  }

  char detail[64] = "";
  int fd = -1;
  const char *problem = nullptr;
  if (sdMount() != SdStatus::READY) {
    problem = "Cannot read the SD card";
  } else if (s_count == 1) {
    char size[16];
    snprintf(size, sizeof size, "%.2f MB", (double)s_size / 1048576.0);
    snprintf(detail, sizeof detail, "%s, %s", s_name, size);
    fd = sdOpen(s_name);
    if (fd < 0 || !CardFile{fd}.readAt(0, in.head, (uint32_t)fwlogic::kHeadBytes)) {
      // a file shorter than the header is inspected as it is: the head stays zero and "too small" is the verdict
      if (fd < 0 || s_size >= fwlogic::kHeadBytes) problem = "Cannot read the firmware file";
    }
  }

  fwlogic::Decision d;
  char text[64] = "";
  if (!problem) {
    d = fwlogic::decide(in);
    LOGF(TAG, "firmware file: verdict %d, problem %d, battery %d mV", (int)d.verdict, (int)d.problem, in.batteryMv);
    if (d.verdict == fwlogic::Verdict::NOTHING || d.verdict == fwlogic::Verdict::SAME_AS_RUNNING) {
      sdClose(fd);
      sdUnmount();
      return;  // nothing to do; a leftover copy of what runs now is not worth a message
    }
    if (d.verdict != fwlogic::Verdict::INSTALL) {
      fwlogic::describe(d, in.batteryMv, text, sizeof text);
      problem = text;
    }
  }
  if (!problem) problem = installFrom(fd, s_size, detail, show);

  sdClose(fd);
  if (problem) {
    LOGF(TAG, "not installed: %s", problem);
    sdUnmount();
    say(show, FW_PROBLEM, problem, detail, -1, false);
    delay(6000);
    return;
  }

  // installed: remember which build it was, move the file out of the way, restart into it
  saveLast(fwlogic::elfHash(in.head));
  say(show, FW_DONE, "Restarting", detail, 100, false);
  char doneName[64];
  snprintf(doneName, sizeof doneName, "%s%s", s_name, FIRMWARE_DONE_SUFFIX);
  const bool renamed = sdRename(s_name, doneName);
  LOGF(TAG, "installed %s (%s); restarting into it", s_name, renamed ? "file renamed" : "file could not be renamed");
  sdUnmount();
  delay(2500);
  ESP.restart();
}

// The image stays "on trial" until the bootloader has accepted the confirmation: if the call fails (flash
// trouble) the next reset could still undo the update, so the flag, and the Info page, must say so.
void fwConfirmNow() {
  if (!s_trial) return;
  const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) {
    s_trial = false;
    LOGF(TAG, "firmware confirmed");
  } else {
    LOGF(TAG, "firmware could NOT be confirmed (%s): still on trial", esp_err_to_name(err));
  }
}

void fwTrialTick(uint32_t nowMs) {
  // a confirmation that failed is tried again every 5 s, five times in all; after that it is left alone
  static uint8_t tries = 0;
  static uint32_t lastTryMs = 0;
  if (!s_trial || nowMs < FIRMWARE_TRIAL_MS || tries >= 5) return;
  if (tries > 0 && nowMs - lastTryMs < 5000) return;
  tries++;
  lastTryMs = nowMs ? nowMs : 1;
  fwConfirmNow();
}

bool fwOnTrial(uint32_t nowMs, uint32_t *secondsLeft) {
  if (!s_trial) return false;
  *secondsLeft = nowMs >= FIRMWARE_TRIAL_MS ? 0 : (FIRMWARE_TRIAL_MS - nowMs + 999) / 1000;
  return true;
}

bool fwRolledBack() { return s_rolledBack; }

bool fwSwitchToOtherSlot() { return Update.canRollBack() && Update.rollBack(); }

void fwForgetLast() {
  Preferences p;
  if (p.begin(kNamespace, false)) {
    p.remove("last");
    p.end();
  }
}

void fwPrint(Print &out) {
  const esp_partition_t *run = esp_ota_get_running_partition();
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  auto line = [&](const char *which, const esp_partition_t *p) {
    if (!p) {
      out.printf("  %-8s none\n", which);
      return;
    }
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    const bool haveState = esp_ota_get_state_partition(p, &st) == ESP_OK;
    uint8_t elf[fwlogic::kDigestBytes];
    char hex[17] = "-";
    if (partitionElf(p, elf)) toHex(elf, 8, hex);
    out.printf("  %-8s %-5s 0x%06x %u KB  build %s  %s\n", which, p->label, (unsigned)p->address, (unsigned)(p->size / 1024), hex,
               haveState ? stateName(st) : "state n/a");
  };
  line("running", run);
  line("other", other == run ? nullptr : other);
  uint8_t last[fwlogic::kDigestBytes];
  char lastHex[17] = "none";
  if (loadLast(last)) toHex(last, 8, lastHex);
  out.printf("  last build installed from the card: %s\n", lastHex);
  uint32_t left;
  if (fwOnTrial(millis(), &left)) out.printf("  on trial: kept in %u s\n", (unsigned)left);
}
