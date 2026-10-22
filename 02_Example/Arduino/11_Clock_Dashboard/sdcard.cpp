#include "sdcard.h"

#include <driver/sdmmc_host.h>
#include <esp_vfs_fat.h>
#include <fcntl.h>
#include <sdmmc_cmd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include <esp_heap_caps.h>

#include "config.h"
#include "log.h"
#include "sd_layout.h"

namespace {

const char *const TAG = "sd";
const char *const kMountPoint = "/sdcard";
sdmmc_card_t *s_card = nullptr;
sdlayout::Kind s_problem = sdlayout::Kind::UNKNOWN;  // what is on a card that would not mount
bool s_probed = false;                               // ... looked at once per run (the card is asked for twice at start-up)

void pathFor(const char *name, char *out, size_t cap) { snprintf(out, cap, "%s/%s", kMountPoint, name); }

// The file system would not mount.  Start the card once more, without a file system, and read its first
// sector (and that of its first partition) to see what is on it: sd_layout.h says what the bytes mean.
void probeCard(const sdmmc_slot_config_t &slot) {
  if (s_probed) return;
  s_probed = true;
  s_problem = sdlayout::Kind::UNKNOWN;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  if ((*host.init)() != ESP_OK) return;
  sdmmc_card_t *card = (sdmmc_card_t *)calloc(1, sizeof(sdmmc_card_t));
  uint8_t *sector = (uint8_t *)heap_caps_malloc(sdlayout::kSectorBytes, MALLOC_CAP_DMA);
  uint8_t type = 0;
  uint32_t lba = 0;
  if (card && sector && sdmmc_host_init_slot(host.slot, &slot) == ESP_OK && sdmmc_card_init(&host, card) == ESP_OK) {
    if (sdmmc_read_sectors(card, sector, 0, 1) != ESP_OK) {
      s_problem = sdlayout::Kind::READ_ERROR;
    } else {
      const sdlayout::First first = sdlayout::inspectFirst(sector);
      s_problem = first.kind;
      type = first.partitionType;
      lba = first.partitionLba;
      if (first.lookAtPartition) {
        s_problem = sdmmc_read_sectors(card, sector, lba, 1) == ESP_OK ? sdlayout::inspectPartition(sector, type) : sdlayout::Kind::READ_ERROR;
      }
    }
  }
  free(sector);
  free(card);
  if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
    host.deinit_p(host.slot);
  } else {
    host.deinit();
  }
  if (type) {
    LOGF(TAG, "the card holds: %s (first partition: type 0x%02X at sector %lu)", sdlayout::name(s_problem), type, (unsigned long)lba);
  } else {
    LOGF(TAG, "the card holds: %s (no partition table)", sdlayout::name(s_problem));
  }
}

}  // namespace

const char *sdProblemText() { return sdlayout::text(s_problem); }

SdStatus sdMount() {
  if (s_card) return SdStatus::READY;
  esp_vfs_fat_sdmmc_mount_config_t mount = {};
  mount.format_if_mount_failed = false;  // never format somebody's card
  mount.max_files = 3;
  mount.allocation_unit_size = 16 * 1024;

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.clk = (gpio_num_t)PIN_SD_CLK;
  slot.cmd = (gpio_num_t)PIN_SD_CMD;
  slot.d0 = (gpio_num_t)PIN_SD_D0;
  slot.d1 = slot.d2 = slot.d3 = GPIO_NUM_NC;  // 1-bit mode: the chip's defaults for these are other things on this board
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;  // belt and braces: the board has its own pull-ups

  const esp_err_t err = esp_vfs_fat_sdmmc_mount(kMountPoint, &host, &slot, &mount, &s_card);
  if (err == ESP_OK) return SdStatus::READY;
  s_card = nullptr;
  LOGF(TAG, "mount failed: 0x%x", (unsigned)err);
  // ESP_FAIL: the card answered but the file system did not mount; anything else (a timeout, as
  // when the slot is empty) means there is no usable card
  if (err != ESP_FAIL) return SdStatus::NO_CARD;
  probeCard(slot);  // ... and why not: sdProblemText() says
  return SdStatus::UNREADABLE;
}

void sdUnmount() {
  if (!s_card) return;
  esp_vfs_fat_sdcard_unmount(kMountPoint, s_card);
  s_card = nullptr;
}

void sdCardSummary(char *out, size_t cap) {
  if (!s_card) {
    snprintf(out, cap, "no card");
    return;
  }
  const double gb = (double)s_card->csd.capacity * (double)s_card->csd.sector_size / 1e9;
  const char *kind = (s_card->ocr & (1UL << 30)) ? "SDHC" : "SD";  // CCS bit: high capacity
  if (gb >= 1.0) {
    snprintf(out, cap, "%s %.1f GB", kind, gb);
  } else {
    snprintf(out, cap, "%s %.0f MB", kind, gb * 1000.0);
  }
}

bool sdFileExists(const char *name) {
  if (!s_card) return false;
  char path[80];
  pathFor(name, path, sizeof path);
  struct stat st;
  return stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

bool sdReadFile(const char *name, char **data, size_t *len, size_t maxBytes) {
  *data = nullptr;
  *len = 0;
  if (!s_card) return false;
  char path[80];
  pathFor(name, path, sizeof path);
  FILE *f = fopen(path, "rb");
  if (!f) return false;
  struct stat st;
  const long size = (stat(path, &st) == 0) ? (long)st.st_size : -1;
  if (size < 0 || (size_t)size > maxBytes) {
    fclose(f);
    return false;
  }
  char *buf = (char *)malloc((size_t)size + 1);
  if (!buf) {
    fclose(f);
    return false;
  }
  const size_t got = size ? fread(buf, 1, (size_t)size, f) : 0;
  fclose(f);
  if (got != (size_t)size) {
    free(buf);
    return false;
  }
  buf[size] = 0;
  *data = buf;
  *len = (size_t)size;
  return true;
}

bool sdWriteFile(const char *name, const char *data, size_t len) {
  if (!s_card) return false;
  char path[80];
  pathFor(name, path, sizeof path);
  FILE *f = fopen(path, "wb");
  if (!f) return false;
  const size_t put = fwrite(data, 1, len, f);
  const bool flushed = fflush(f) == 0 && fsync(fileno(f)) == 0;
  const bool closed = fclose(f) == 0;
  return put == len && flushed && closed;
}

bool sdFileSize(const char *name, uint32_t *size) {
  if (!s_card) return false;
  char path[80];
  pathFor(name, path, sizeof path);
  struct stat st;
  if (stat(path, &st) != 0 || !S_ISREG(st.st_mode) || st.st_size < 0) return false;
  *size = (uint32_t)st.st_size;
  return true;
}

int sdOpen(const char *name) {
  if (!s_card) return -1;
  char path[80];
  pathFor(name, path, sizeof path);
  return open(path, O_RDONLY);
}

int sdRead(int fd, void *buf, size_t len) { return (int)read(fd, buf, len); }

bool sdSeek(int fd, uint32_t offset) { return lseek(fd, (off_t)offset, SEEK_SET) == (off_t)offset; }

void sdClose(int fd) {
  if (fd >= 0) close(fd);
}

bool sdRename(const char *from, const char *to) {
  if (!s_card) return false;
  char a[80], b[80];
  pathFor(from, a, sizeof a);
  pathFor(to, b, sizeof b);
  unlink(b);  // FAT will not rename onto a name that exists
  return rename(a, b) == 0;
}

int sdCreate(const char *name) {
  if (!s_card) return -1;
  char path[80];
  pathFor(name, path, sizeof path);
  return open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
}

bool sdWrite(int fd, const void *buf, size_t len) { return fd >= 0 && write(fd, buf, len) == (ssize_t)len; }

bool sdCommit(int fd) {
  if (fd < 0) return false;
  const bool flushed = fsync(fd) == 0;
  const bool closed = close(fd) == 0;
  return flushed && closed;
}

bool sdRemove(const char *name) {
  if (!s_card) return false;
  char path[80];
  pathFor(name, path, sizeof path);
  return unlink(path) == 0;
}
