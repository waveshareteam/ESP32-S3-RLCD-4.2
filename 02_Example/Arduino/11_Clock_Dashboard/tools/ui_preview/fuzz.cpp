// Robustness test for ui.cpp: renders many frames built from adversarial data
// (invalid UTF-8, emoji, CJK, control characters, strings that fill their
// buffers, out-of-range numbers) and relies on AddressSanitizer / UBSan to catch
// any overflow or undefined behaviour in the drawing code.
//
//   ./run_fuzz.sh [iterations] [seed]

#include "host_display.h"

#include "../../ui.cpp"

#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

static UiModel *g_current = nullptr;
static int g_iteration = 0;

// A frame must render in milliseconds; if one takes seconds the drawing code is
// stuck in a loop.  Report the model that did it.
static void onHang(int) {
  fprintf(stderr, "HANG in frame %d: page=%d spotify=%d\n", g_iteration, g_current->page,
          g_current->spotify.status);
  fprintf(stderr, "  title=[%s]\n  artist=[%s]\n  album=[%s]\n  device=[%s]\n", g_current->spotify.title,
          g_current->spotify.artist, g_current->spotify.album, g_current->spotify.device);
  fprintf(stderr, "  message=[%s]\n  linkUrl=[%s]\n  location=[%s]\n  status=[%s]\n  toast=[%s]\n",
          g_current->spotify.message, g_current->spotify.linkUrl, g_current->location, g_current->status,
          g_current->toast);
  _exit(2);
}

static uint64_t g_rng = 0x9E3779B97F4A7C15ull;
static uint32_t rnd() {
  g_rng ^= g_rng << 13;
  g_rng ^= g_rng >> 7;
  g_rng ^= g_rng << 17;
  return (uint32_t)(g_rng >> 16);
}
static int rndRange(int lo, int hi) { return lo + (int)(rnd() % (uint32_t)(hi - lo + 1)); }
static float rndFloat(float lo, float hi) { return lo + (hi - lo) * (rnd() / 4294967296.0f) * 65536.0f / 65536.0f; }

// Fragments a track title might plausibly (or implausibly) contain.
static const char *kFragments[] = {
    "Blinding Lights", "Don\xE2\x80\x99t Stop", " - ", " (Remastered 2011)", "Caf\xC3\xA9 del Mar",
    "\xE5\xA4\x9C\xE3\x81\xAB\xE9\xA7\x86\xE3\x81\x91\xE3\x82\x8B",  // CJK
    "\xF0\x9F\x8E\xB5",                                             // emoji (4-byte)
    "\xD0\x9C\xD0\xB8\xD1\x80",                                     // Cyrillic
    "\xE2\x80\x94", "\xE2\x80\xA6", "\xC2\xA0", "\xC2\xB7",
    "\x80", "\xC0\xAF", "\xFF", "\xE2\x80", "\xF0\x9F",             // invalid / truncated
    "\t", "\n", "\x01", "  ", "WWWWWWWWWWWWWWWWWWWW", "iiiiiiiiiiiiiiiiiiii", "%s%d%n", "a",
};

static void fillString(char *dst, size_t cap) {
  size_t n = 0;
  int mode = rndRange(0, 9);
  size_t target = mode == 0 ? 0 : (mode <= 2 ? cap - 1 : (size_t)rndRange(1, (int)cap - 1));
  while (n < target) {
    const char *f = kFragments[rnd() % (sizeof(kFragments) / sizeof(kFragments[0]))];
    size_t l = strlen(f);
    if (n + l >= cap) {  // may cut a multi-byte sequence in half on purpose
      l = cap - 1 - n;
    }
    memcpy(dst + n, f, l);
    n += l;
    if (rndRange(0, 5) == 0) break;
  }
  if (mode == 1) {  // exactly full buffer, no terminator inside
    memset(dst, 'x', cap - 1);
    n = cap - 1;
  }
  dst[n] = 0;
}

static float wild(float lo, float hi) {
  switch (rnd() % 12) {
    case 0: return 0;
    case 1: return -0.0f;
    case 2: return 1e9f;
    case 3: return -1e9f;
    default: return lo + (hi - lo) * ((rnd() & 0xFFFF) / 65535.0f);
  }
}

static void randomModel(UiModel &m) {
  m = UiModel();
  m.timeValid = rndRange(0, 7) != 0;
  m.local.tm_year = rndRange(100, 200);
  m.local.tm_mon = rndRange(0, 11);
  m.local.tm_mday = rndRange(1, 31);
  m.local.tm_hour = rndRange(0, 23);
  m.local.tm_min = rndRange(0, 59);
  m.local.tm_sec = rndRange(0, 60);  // leap second
  m.local.tm_wday = rndRange(0, 6);
  m.local.tm_yday = rndRange(0, 365);
  m.secFrac = rndFloat(0, 1);
  m.utcOffsetMin = rndRange(-14 * 60, 14 * 60);
  fillString(m.tzAbbrev, sizeof m.tzAbbrev);

  m.indoorValid = rndRange(0, 5) != 0;
  m.indoorC = wild(-40, 80);
  m.indoorRh = wild(-5, 120);
  m.batPresent = rndRange(0, 3) != 0;
  m.batVolts = wild(0, 5);
  m.batPercent = rndRange(-20, 130);
  m.batLow = rndRange(0, 1);
  m.batBlinkOn = rndRange(0, 1);
  m.batCharge = rndRange(-2, 6);  // includes values that are not a UiCharge
  m.wifiUp = rndRange(0, 1);
  m.rssi = rndRange(-130, 10);
  fillString(m.location, sizeof m.location);
  fillString(m.status, sizeof m.status);

  WeatherData &w = m.weather;
  w.valid = rndRange(0, 5) != 0;
  w.ageSec = rnd();
  w.temp = wild(-60, 70);
  w.feels = wild(-60, 70);
  w.windKmh = wild(0, 300);
  w.humidity = (uint8_t)rnd();
  w.code = (uint8_t)rnd();
  w.isDay = rndRange(0, 1);
  for (auto &d : w.day) {
    d.code = (uint8_t)rnd();
    d.tmax = wild(-60, 70);
    d.tmin = wild(-60, 70);
    d.rainPct = (uint8_t)rnd();
    d.weekday = (uint8_t)rnd();  // deliberately out of range too
  }
  fillString(w.sunrise, sizeof w.sunrise);
  fillString(w.sunset, sizeof w.sunset);
  w.uvMax = wild(0, 20);

  SpotifyInfo &s = m.spotify;
  s.status = (SpotifyStatus)rndRange(0, 5);
  fillString(s.title, sizeof s.title);
  fillString(s.artist, sizeof s.artist);
  fillString(s.album, sizeof s.album);
  fillString(s.device, sizeof s.device);
  fillString(s.message, sizeof s.message);
  fillString(s.linkUrl, sizeof s.linkUrl);
  s.durationMs = rndRange(0, 3) == 0 ? 0 : rnd();
  s.progressMs = rndRange(0, 3) == 0 ? rnd() : s.durationMs / 3;  // sometimes beyond the end
  s.volume = (int8_t)rndRange(-1, 100);
  s.linkDaysLeft = (int16_t)rndRange(-1, 200);

  m.useFahrenheit = rndRange(0, 1);
  m.time12h = rndRange(0, 1);
  m.dateFormat = (uint8_t)rndRange(0, 9);  // two past the last format on purpose
  m.showWeek = rndRange(0, 1);
  m.wifiOff = rndRange(0, 4) == 0;
  m.moonValid = rndRange(0, 4) != 0;
  m.southern = rndRange(0, 1);
  for (int i = 0; i < 3; i++) {
    m.moonLit[i] = (uint8_t)rnd();  // up to 255 %: must be clamped
    m.moonWaxing[i] = rndRange(0, 1);
  }
  m.page = (UiPage)rndRange(0, PAGE_COUNT - 1);
  if (rndRange(0, 3) == 0) {
    fillString(m.toast, sizeof m.toast);
    m.toastIcon = rndRange(0, 6);
  }
  m.infoCount = rndRange(0, UI_INFO_LINES);
  for (int i = 0; i < UI_INFO_LINES; i++) fillString(m.info[i], UI_INFO_LINE_LEN);
}

int main(int argc, char **argv) {
  int iterations = argc > 1 ? atoi(argv[1]) : 3000;
  if (argc > 2) g_rng ^= (uint64_t)strtoull(argv[2], nullptr, 10) * 0xD1B54A32D192ED03ull;  // seed
  u8g2_t *u = hostDisplayInit();
  UiModel m;
  g_current = &m;
  signal(SIGALRM, onHang);
  for (int i = 0; i < iterations; i++) {
    g_iteration = i;
    randomModel(m);
    alarm(5);
    u8g2_ClearBuffer(u);
    uiDraw(u, m);
    if (i % 50 == 0) {  // the shutdown screen, with any voltage
      u8g2_ClearBuffer(u);
      uiDrawBatteryEmpty(u, wild(-1, 6), wild(0, 6), rndRange(0, 1));
    }
    if (i % 50 == 7) {  // the firmware-update screen, with anything in its fields
      UiFwScreen f;
      f.kind = (UiFwKind)rndRange(0, 4);  // two past the last kind on purpose
      fillString(f.what, sizeof f.what);
      fillString(f.detail, sizeof f.detail);
      f.percent = rndRange(-300, 300);
      f.keepPowered = rndRange(0, 1);
      u8g2_ClearBuffer(u);
      uiDrawFirmwareUpdate(u, f);
    }
  }
  alarm(0);
  printf("fuzz: %d frames rendered, no faults\n", iterations);
  return 0;
}
