// Host-side UI preview.  Renders ui.cpp for a set of scenarios and writes
// out/<name>.pgm (400x300).  Convert to PNG with to_png.py.
//
//   ./run_preview.sh            (Linux / WSL)
//
// ui.cpp is #included so the gallery can reach its internal helpers.

#include "host_display.h"

#include "../../ui.cpp"

#include "../../calc.h"
#include "../../sd_layout.h"
#include "../../settings.h"

#include <stdlib.h>
#include <sys/stat.h>
#include <string>
#include <vector>

static u8g2_t *g_u = nullptr;

static void snapshot(const char *name, const UiModel &m) {
  u8g2_ClearBuffer(g_u);
  uiDraw(g_u, m);
  char path[128];
  snprintf(path, sizeof path, "out/%s.pgm", name);
  hostDumpPgm(path);
  printf("rendered %s\n", path);
}

static void setStr(char *dst, size_t cap, const char *s) { snprintf(dst, cap, "%s", s); }

// Sunday 2026-10-04 08:21:07 in Perth, WA, with sample weather numbers.
static UiModel baseModel() {
  UiModel m;
  m.timeValid = true;
  memset(&m.local, 0, sizeof m.local);
  m.local.tm_year = 2026 - 1900;
  m.local.tm_mon = 9;
  m.local.tm_mday = 4;
  m.local.tm_hour = 8;
  m.local.tm_min = 21;
  m.local.tm_sec = 7;
  m.local.tm_wday = 0;
  m.local.tm_yday = 276;
  m.utcOffsetMin = 480;
  setStr(m.tzAbbrev, sizeof m.tzAbbrev, "AWST");

  m.indoorValid = true;
  m.indoorC = 22.4f;
  m.indoorRh = 45;

  m.batPresent = true;
  m.batVolts = 4.05f;
  m.batPercent = 87;

  m.wifiUp = true;
  m.rssi = -58;
  setStr(m.location, sizeof m.location, "Perth");

  WeatherData &w = m.weather;
  w.valid = true;
  w.temp = 15.3f;
  w.feels = 15.1f;
  w.humidity = 78;
  w.code = 0;
  w.isDay = true;
  w.windKmh = 3.8f;
  w.day[0] = {3, 18.8f, 9.4f, 4, 0};
  w.day[1] = {51, 19.5f, 11.5f, 35, 1};
  w.day[2] = {3, 20.5f, 9.1f, 0, 2};
  setStr(w.sunrise, sizeof w.sunrise, "05:52");
  setStr(w.sunset, sizeof w.sunset, "18:23");
  w.uvMax = 7.2f;

  // the moon for this date (2026-10-04 08:21 AWST = 00:21 UTC), today and the next two days
  m.moonValid = true;
  m.southern = true;  // Perth
  const int64_t base = calc::epochFromUtc(2026, 10, 4, 0, 21, 7);
  for (int i = 0; i < 3; i++) {
    const moon::Phase p = moon::phaseAt(base + (int64_t)i * 86400);
    m.moonLit[i] = (uint8_t)lroundf(p.lit * 100);
    m.moonWaxing[i] = p.waxing;
  }

  SpotifyInfo &s = m.spotify;
  s.status = SPOTIFY_PLAYING;
  setStr(s.title, sizeof s.title, "Blinding Lights");
  setStr(s.artist, sizeof s.artist, "The Weeknd");
  setStr(s.album, sizeof s.album, "After Hours");
  setStr(s.device, sizeof s.device, "Kitchen speaker");
  s.durationMs = 200040;
  s.progressMs = 83000;
  s.volume = 59;
  return m;
}

static void setTime(UiModel &m, int h, int mi, int s) {
  m.local.tm_hour = h;
  m.local.tm_min = mi;
  m.local.tm_sec = s;
}


// ---------------------------------------------------------------------------
// Layout stability: numbers that change every second/day must not wobble.
// U8g2's own string width is ink based, so centring on it moved the time by up
// to 5 px depending on its digits; ui.cpp lays digits out on fixed cells.
//
// Method: for a string like "00:00:00" work out the glyph cells exactly as the
// renderer does, and collect the pixels inside the cells of the punctuation
// (':' and '-').  Those cells must hold identical pixels in every frame.  If the
// text shifted, they would not.
// ---------------------------------------------------------------------------
static bool inkAt(int lx, int ly) {
  int px = kPhysW - 1 - ly, py = lx;
  return (g_hostBuf[(py / 8) * (kTileW * 8) + px] >> (py % 8)) & 1;
}

static std::string punctuationCells(const uint8_t *font, int cx, const char *layout, int y0, int y1) {
  u8g2_SetFont(g_u, font);
  const int pitch = digitPitch(g_u);
  int x = cx - pitchWidth(g_u, layout) / 2;
  std::string pattern;
  for (const char *c = layout; *c; ++c) {
    const bool digit = *c >= '0' && *c <= '9';
    const int adv = digit ? pitch : u8g2_GetGlyphWidth(g_u, (unsigned char)*c);
    if (!digit) {
      for (int xx = x; xx < x + adv; xx++)
        for (int y = y0; y <= y1; y++) pattern += inkAt(xx, y) ? '#' : '.';
    }
    x += adv;
  }
  return pattern;
}

static int checkLayoutStability() {
  const int cx = (kRightX0 + kRightX1) / 2;
  int timeBad = 0, dateBad = 0, barBad = 0, oldWayBad = 0, frames = 0;
  std::string refTime, refDate, refBar, refOld;

  const int hours[] = {0, 1, 7, 8, 10, 11, 12, 17, 19, 21, 22, 23};
  const int mins[] = {0, 1, 7, 10, 11, 21, 34, 59};
  const int secs[] = {0, 1, 7, 10, 11, 21, 59};
  for (int h : hours)
    for (int mi : mins)
      for (int sc : secs) {
        UiModel m = baseModel();
        setTime(m, h, mi, sc);
        m.spotify.status = SPOTIFY_IDLE;

        u8g2_ClearBuffer(g_u);  // dashboard: the big time
        uiDraw(g_u, m);
        std::string t = punctuationCells(F_TIME, cx, "00:00:00", 57, 98);
        if (refTime.empty()) refTime = t;
        if (t != refTime && timeBad++ < 3) printf("  TIME MOVED at %02d:%02d:%02d\n", h, mi, sc);

        m.page = PAGE_INFO;  // secondary pages: the clock in the status bar
        u8g2_ClearBuffer(g_u);
        uiDraw(g_u, m);
        std::string b = punctuationCells(F_BOLD, 200, "00:00:00", 2, 16);
        if (refBar.empty()) refBar = b;
        if (b != refBar && barBad++ < 3) printf("  STATUS-BAR CLOCK MOVED at %02d:%02d:%02d\n", h, mi, sc);

        // negative control: the previous method (centre on U8g2's own width) must be caught by the same check
        char buf[16];
        snprintf(buf, sizeof buf, "%02d:%02d:%02d", h, mi, sc);
        u8g2_ClearBuffer(g_u);
        u8g2_SetFontMode(g_u, 1);
        u8g2_SetFont(g_u, F_TIME);
        ink(g_u);
        txtC(g_u, cx, 99, buf);
        std::string o = punctuationCells(F_TIME, cx, "00:00:00", 57, 98);
        if (refOld.empty()) refOld = o;
        if (o != refOld) oldWayBad++;
        frames += 3;
      }

  const int dates[][3] = {{2026, 10, 4}, {2026, 11, 11}, {2027, 1, 7}, {2026, 12, 31}, {2026, 7, 17}, {2031, 3, 1}, {2029, 9, 27}};
  for (auto &d : dates) {
    UiModel m = baseModel();
    m.local.tm_year = d[0] - 1900;
    m.local.tm_mon = d[1] - 1;
    m.local.tm_mday = d[2];
    m.spotify.status = SPOTIFY_IDLE;
    u8g2_ClearBuffer(g_u);
    uiDraw(g_u, m);
    std::string p = punctuationCells(F_DATE, cx, "0000-00-00", 25, 50);
    if (refDate.empty()) refDate = p;
    if (p != refDate && dateBad++ < 3) printf("  DATE MOVED for %04d-%02d-%02d\n", d[0], d[1], d[2]);
    frames++;
  }

  int bad = timeBad + dateBad + barBad;
  printf("layout stability: %d frames, %d problems (time %d, date %d, status-bar clock %d)\n", frames, bad, timeBad,
         dateBad, barBad);
  if (oldWayBad == 0) {  // the check itself must be able to see the old wobble, or it proves nothing
    printf("  CHECK IS BLIND: the old centring method showed no movement\n");
    bad++;
  } else {
    printf("  negative control OK: the old centring method moved the time in %d of %d frames\n", oldWayBad, frames / 3);
  }
  return bad;
}

// ---------------------------------------------------------------------------
// Label plates: white capitals on a black plate need the same black above and
// below them, and left and right (the INDOOR label once had 1 px above and 3 px
// below).  Measured on the rendered pixels: the plate is the run of solid rows
// just inside its left edge, the text is whatever is white inside it.
// ---------------------------------------------------------------------------
struct PlateMargins {
  bool found = false;
  int above = 0, below = 0, left = 0, right = 0;
};

static PlateMargins measurePlate(int plateX) {
  PlateMargins r;
  auto solidRow = [&](int y) { return inkAt(plateX, y) && inkAt(plateX + 1, y) && inkAt(plateX + 2, y); };
  int mid = -1;
  for (int y = 120; y < 170 && mid < 0; y++)
    if (solidRow(y)) mid = y;
  if (mid < 0) return r;
  int top = mid, bot = mid, left = plateX, right = plateX;
  while (solidRow(top - 1)) top--;
  while (solidRow(bot + 1)) bot++;
  while (inkAt(left - 1, top)) left--;  // the plate's own top row is solid all the way across
  while (inkAt(right + 1, top)) right++;
  int tTop = -1, tBot = -1, tLeft = 1 << 20, tRight = -1;
  for (int y = top; y <= bot; y++)
    for (int x = left; x <= right; x++)
      if (!inkAt(x, y)) {
        if (tTop < 0) tTop = y;
        tBot = y;
        if (x < tLeft) tLeft = x;
        if (x > tRight) tRight = x;
      }
  if (tTop < 0) return r;
  r.found = true;
  r.above = tTop - top;
  r.below = bot - tBot;
  r.left = tLeft - left;
  r.right = right - tRight;
  return r;
}

static int checkLabelPlates() {
  int bad = 0;
  UiModel m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  u8g2_ClearBuffer(g_u);
  uiDraw(g_u, m);
  const int plateX = kRightX0 + 8;
  PlateMargins p = measurePlate(plateX);
  printf("INDOOR label plate: %d px of black above, %d below, %d left, %d right\n", p.above, p.below, p.left, p.right);
  if (!p.found || p.above < 2 || p.above != p.below || abs(p.left - p.right) > 1) {
    printf("  PROBLEM: the label text is not centred on its plate\n");
    bad++;
  }

  // negative control: the previous geometry (plate rows y0-6..y0+5, baseline y0+3) must be caught by the same check
  const int x0 = kRightX0, y0 = 144;
  u8g2_ClearBuffer(g_u);
  u8g2_SetFontMode(g_u, 1);
  ink(g_u);
  u8g2_SetFont(g_u, F_SMALLB);
  u8g2_DrawBox(g_u, x0 + 8, y0 - 6, tw(g_u, "INDOOR") + 8, 12);
  paper(g_u);
  txt(g_u, x0 + 12, y0 + 3, "INDOOR");
  ink(g_u);
  PlateMargins o = measurePlate(plateX);
  if (!o.found || o.above == o.below) {
    printf("  CHECK IS BLIND: the old label geometry (%d above, %d below) was not flagged\n", o.above, o.below);
    bad++;
  } else {
    printf("  negative control OK: the old geometry measures %d above and %d below\n", o.above, o.below);
  }
  return bad;
}

// ---------------------------------------------------------------------------
// Battery gauge: it must not move when the percentage gains or loses a digit, the
// charge icon has to be there, and a long status message must stop short of it.
// ---------------------------------------------------------------------------
static int leftmostInk(int x0, int x1, int y0, int y1) {
  for (int x = x0; x < x1; x++)
    for (int y = y0; y <= y1; y++)
      if (inkAt(x, y)) return x;
  return -1;
}

static int checkBatteryGauge() {
  int bad = 0;
  UiModel m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;

  int ref = -2;
  for (int pct : {100, 99, 87, 50, 10, 9, 0}) {
    m.batPercent = pct;
    u8g2_ClearBuffer(g_u);
    uiDraw(g_u, m);
    const int x = leftmostInk(300, 396, 4, 15);  // the body's frame
    if (ref == -2) ref = x;
    if (x != ref) {
      printf("  GAUGE MOVED at %d%%: left edge x=%d, expected %d\n", pct, x, ref);
      bad++;
    }
  }

  // What the battery is doing is shown by a bolt (charging) or a tick (full) INSIDE the body: the pixels that
  // differ from the same bar on the battery lie inside the frame, there are some, and the gauge itself never
  // moves with the state or collides with the status message.
  const char *names[] = {"unknown", "discharging", "charging", "full"};
  struct Look {
    int pct;
    bool low, blinkOn;
  };
  const Look looks[] = {{0, false, false}, {17, false, false}, {62, false, false}, {100, false, false}, {17, true, true}, {17, true, false}};
  static bool plain[20][100];  // the right end of the status bar on the battery, without art
  int gaugeFrames = 0, artFrames = 0;
  auto render = [&](const Look &lk, int st) {
    UiModel s = baseModel();
    s.spotify.status = SPOTIFY_IDLE;
    s.batPercent = lk.pct;
    s.batLow = lk.low;
    s.batBlinkOn = lk.blinkOn;
    s.batCharge = st;
    setStr(s.status, sizeof s.status, "Weather: could not reach api.open-meteo.com, retrying in 60 s ...");
    u8g2_ClearBuffer(g_u);
    uiDraw(g_u, s);
    return s;
  };
  // pixels that differ from `plain`, inside and outside the body's frame
  auto differences = [&](int bodyL, int *inside, int *outside) {
    *inside = *outside = 0;
    for (int y = 0; y < 20; y++)
      for (int x = 300; x < 400; x++)
        if (inkAt(x, y) != plain[y][x - 300]) {
          const bool in = x > bodyL && x < bodyL + kBatBodyW - 1 && y > 4 && y < 4 + kBatBodyH - 1;
          (in ? *inside : *outside)++;
        }
  };
  for (const Look &lk : looks) {
    const UiModel ref = render(lk, CHARGE_DISCHARGING);
    for (int y = 0; y < 20; y++)
      for (int x = 300; x < 400; x++) plain[y][x - 300] = inkAt(x, y);
    u8g2_SetFont(g_u, F_BOLD);
    const int bodyL = 396 - batteryLayout(g_u, ref).width;
    for (int st = CHARGE_UNKNOWN; st <= CHARGE_FULL; st++) {
      render(lk, st);
      int inside, outside, intrusions = 0;
      differences(bodyL, &inside, &outside);
      const bool plate = lk.low && lk.blinkOn;  // the blink's black plate reaches 4 px left of the gauge
      for (int x = bodyL - 8; x < bodyL - (plate ? 4 : 0); x++)
        for (int y = 2; y <= 17; y++) intrusions += inkAt(x, y);
      // the bolt sits in a window and shows at every level; the tick is cut out of the fill, so it needs a full
      // one (a "full" gauge below 100 % may show less of it, or none)
      const bool wantsArt = st == CHARGE_CHARGING || (st == CHARGE_FULL && lk.pct == 100);
      const bool optionalArt = st == CHARGE_FULL && !wantsArt;
      // the body's frame starts the group, in every state (white on the black plate while it blinks)
      const bool stays = plate ? (!inkAt(bodyL, 8) && inkAt(bodyL - 1, 8)) : (inkAt(bodyL, 8) && !inkAt(bodyL - 1, 8));
      gaugeFrames++;
      artFrames += wantsArt;
      if (!(wantsArt ? inside >= 8 : (optionalArt || inside == 0)) || outside || intrusions || !stays) {
        printf("  GAUGE PROBLEM (%s, %d%%%s): %d art pixels inside the body, %d outside it, %d message pixels within 8 px, "
               "frame %s\n",
               names[st], lk.pct, lk.low ? (lk.blinkOn ? ", low, blink on" : ", low, blink off") : "", inside, outside, intrusions,
               stays ? "in place" : "MOVED");
        bad++;
      }
    }
  }
  printf("battery gauge states: %d frames (%d with a bolt or a tick), the gauge never moves, nothing leaves the body\n", gaugeFrames, artFrames);
  {  // negative controls: art on the frame, and art that is not there, must both be seen
    const Look lk = looks[2];
    render(lk, CHARGE_DISCHARGING);
    for (int y = 0; y < 20; y++)
      for (int x = 300; x < 400; x++) plain[y][x - 300] = inkAt(x, y);
    const UiModel s = render(lk, CHARGE_CHARGING);
    u8g2_SetFont(g_u, F_BOLD);
    const int bodyL = 396 - batteryLayout(g_u, s).width;
    int inside, outside;
    paper(g_u);
    u8g2_DrawPixel(g_u, bodyL + 10, 4);  // a bolt that reached the top of the frame
    ink(g_u);
    differences(bodyL, &inside, &outside);
    const bool trampled = outside > 0;
    render(lk, CHARGE_DISCHARGING);  // art that is missing
    differences(bodyL, &inside, &outside);
    const bool missing = inside < 8;
    if (!trampled || !missing) {
      printf("  CHECK IS BLIND: art on the frame was %s, missing art was %s\n", trampled ? "seen" : "missed", missing ? "seen" : "missed");
      bad++;
    } else {
      printf("  negative control OK: a pixel off the body's frame and a gauge without its bolt are both seen\n");
    }
  }
  printf("battery gauge: %d problems\n", bad);
  return bad;
}

// ---------------------------------------------------------------------------
// Weather band.  Today's big number starts where the condition text under it does (their
// *ink*, which is not where the glyph cells start: a "1" has 5 blank columns in front of it, a
// "0" has 2); nothing may touch the separators; the moon and rain stacks of the two forecast
// days sit at the same place in both columns.
// ---------------------------------------------------------------------------
static bool anyInk(int x0, int x1, int y0, int y1) { return leftmostInk(x0, x1, y0, y1) >= 0; }

static void renderModel(const UiModel &m) {
  u8g2_ClearBuffer(g_u);
  uiDraw(g_u, m);
}

static int checkWeatherBand() {
  int bad = 0;
  const int temps[] = {-25, -12, -3, 0, 1, 8, 11, 15, 19, 21, 29, 99};
  const int codes[] = {0, 1, 2, 3, 45, 48, 51, 61, 63, 71, 80, 95, 96, 77};
  const int numRows0 = kWxTop + 7, numRows1 = kWxTop + 30, textRows0 = kWxTop + 36, textRows1 = kWxTop + 46;
  int frames = 0, misaligned = 0, oldMisaligned = 0;
  for (int t : temps)
    for (int c : codes) {
      UiModel m = baseModel();
      m.spotify.status = SPOTIFY_IDLE;
      m.weather.temp = (float)t;
      m.weather.code = (uint8_t)c;
      renderModel(m);
      // the icon ends at x=47; the text and the number start at 54 or so
      const int numL = leftmostInk(50, 75, numRows0, numRows1);
      const int textL = leftmostInk(50, 75, textRows0, textRows1);
      frames++;
      if (numL != textL) {
        if (misaligned++ < 3) printf("  TODAY MISALIGNED: %d deg, %s: number ink at x=%d, text ink at x=%d\n", t, wmoText(c), numL, textL);
      }

      // negative control: the number drawn from the same x as the text, as before
      u8g2_ClearBuffer(g_u);
      u8g2_SetFontMode(g_u, 1);
      ink(g_u);
      char num[8];
      snprintf(num, sizeof num, "%d", t);
      u8g2_SetFont(g_u, F_BIG);
      txt(g_u, 54, kWxTop + 31, num);
      u8g2_SetFont(g_u, F_BODY);
      txt(g_u, 54, kWxTop + 47, wmoText(c));
      if (leftmostInk(50, 75, numRows0, numRows1) != leftmostInk(50, 75, textRows0, textRows1)) oldMisaligned++;
    }
  printf("weather band, today: %d frames, %d with the number off the text's left edge\n", frames, misaligned);
  bad += misaligned;
  if (oldMisaligned == 0) {
    printf("  CHECK IS BLIND: drawing both from the same x was not flagged\n");
    bad++;
  } else {
    printf("  negative control OK: drawing both from the same x misaligns %d of %d\n", oldMisaligned, frames);
  }

  // nothing within 3 px of a separator, whatever the numbers
  struct Extreme {
    float temp, hi, lo;
    uint8_t rain;
    bool f;
  };
  const Extreme extremes[] = {{-30, -12, -35, 100, false}, {-9, 5, -12, 100, false}, {44, 49, 38, 100, false},
                              {100, 110, 95, 100, true},   {-5, 19, -4, 100, true},   {15.3f, 18.8f, 9.4f, 0, false}};
  int touching = 0;
  for (const Extreme &e : extremes) {
    UiModel m = baseModel();
    m.spotify.status = SPOTIFY_IDLE;
    m.weather.temp = e.temp;
    m.weather.day[0].tmax = e.hi;
    m.weather.day[0].tmin = e.lo;
    m.weather.day[0].rainPct = e.rain;
    m.weather.day[1].tmax = e.hi;
    m.weather.day[1].tmin = e.lo;
    m.weather.day[1].rainPct = e.rain;
    m.weather.day[2].tmax = e.hi;
    m.weather.day[2].tmin = e.lo;
    m.weather.day[2].rainPct = e.rain;
    m.moonLit[0] = m.moonLit[1] = m.moonLit[2] = 100;
    m.useFahrenheit = e.f;
    renderModel(m);
    const int y0 = kWxTop + 4, y1 = kWxBottom - 4;
    const int seps[] = {kColA, kColB, UI_WIDTH};
    for (int s : seps)
      if (anyInk(s - 3, s, y0, y1)) {
        if (touching++ < 3) printf("  INK WITHIN 3 PX OF A SEPARATOR at x=%d (temp %.0f hi %.0f lo %.0f)\n", s, e.temp, e.hi, e.lo);
      }
  }
  printf("weather band, margins: %d separators touched\n", touching);
  bad += touching;
  {  // negative control: ink put where the check looks must be seen
    UiModel m = baseModel();
    renderModel(m);
    ink(g_u);
    u8g2_DrawBox(g_u, kColA - 3, kWxTop + 10, 2, 2);
    if (!anyInk(kColA - 3, kColA, kWxTop + 4, kWxBottom - 4)) {
      printf("  CHECK IS BLIND: ink 2 px from a separator was not seen\n");
      bad++;
    } else {
      printf("  negative control OK: ink 2 px from a separator is seen\n");
    }
  }

  // the stack of moon and rain starts at the same x in both forecast days
  UiModel m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  renderModel(m);
  const int a = leftmostInk(kColA + 68, kColB - 4, kWxTop + 22, kWxTop + 31);
  const int b = leftmostInk(kColB + 68, UI_WIDTH - 4, kWxTop + 22, kWxTop + 31);
  const int ra = leftmostInk(kColA + 68, kColB - 4, kWxTop + 36, kWxTop + 44);
  const int rb = leftmostInk(kColB + 68, UI_WIDTH - 4, kWxTop + 36, kWxTop + 44);
  printf("weather band, stacks: moon at +%d / +%d, rain at +%d / +%d px from the column start\n", a - kColA, b - kColB,
         ra - kColA, rb - kColB);
  if (a - kColA != b - kColB || ra - kColA != rb - kColB || a < 0 || ra < 0) {
    printf("  PROBLEM: the moon and rain stacks are not in the same place in both columns\n");
    bad++;
  }
  {  // negative control: wide temperatures in one column change what the same measurement reads there
    UiModel w = baseModel();
    w.spotify.status = SPOTIFY_IDLE;
    w.weather.day[1].tmin = -17;
    w.weather.day[1].tmax = -12;
    renderModel(w);
    const int moved = leftmostInk(kColA + 68, kColB - 4, kWxTop + 22, kWxTop + 31) - kColA;
    if (moved == b - kColB) {
      printf("  CHECK IS BLIND: wide temperatures changed nothing (still +%d)\n", moved);
      bad++;
    } else {
      printf("  negative control OK: with -12 / -17 the same measurement reads +%d instead of +%d\n", moved, b - kColB);
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// The 12 hour clock and the date formats.  The colon of "9:05" must not move as the digits
// change (that is what shows when centring goes wrong), the digits and AM / PM stay in the right
// column, and every date, in every format, fits the space under the status bar.
// ---------------------------------------------------------------------------
static int checkClockFormats() {
  int bad = 0;
  const int cx = (kRightX0 + kRightX1) / 2;

  // geometry as drawTime12() lays it out
  u8g2_SetFont(g_u, F_TIME);
  const int pitch = digitPitch(g_u), colon = u8g2_GetGlyphWidth(g_u, ':');
  const int bigW = 4 * pitch + colon;
  u8g2_SetFont(g_u, F_BIG);
  const int secW = 2 * digitPitch(g_u), gap = 8;
  const int x0 = cx - (bigW + gap + secW) / 2;
  const int colonX = x0 + 2 * pitch;

  std::string ref;
  int moved = 0, frames = 0, secBad = 0;
  for (int h = 0; h < 24; h++)
    for (int mi : {0, 5, 11, 20, 45, 59})
      for (int sc : {0, 1, 9, 10, 31, 59}) {
        UiModel m = baseModel();
        m.time12h = true;
        m.spotify.status = SPOTIFY_IDLE;
        setTime(m, h, mi, sc);
        renderModel(m);
        std::string cell;
        for (int xx = colonX; xx < colonX + colon; xx++)
          for (int y = 57; y <= 98; y++) cell += inkAt(xx, y) ? '#' : '.';
        if (ref.empty()) ref = cell;
        if (cell != ref && moved++ < 3) printf("  12 H COLON MOVED at %02d:%02d:%02d\n", h, mi, sc);
        // the seconds live in their own column, right of the big digits
        if (!anyInk(x0 + bigW + gap, x0 + bigW + gap + secW, 75, 98)) secBad++;
        // nothing of the big time reaches into the seconds column
        if (anyInk(x0 + bigW, x0 + bigW + gap - 1, 57, 98)) secBad++;
        frames++;
      }
  // negative control: the plain 24 hour layout, centred on U8g2's own width, is seen to move
  int oldMoved = 0;
  std::string oldRef;
  for (int h = 1; h <= 12; h++) {
    char buf[8];
    snprintf(buf, sizeof buf, "%d:05", h);
    u8g2_ClearBuffer(g_u);
    u8g2_SetFontMode(g_u, 1);
    ink(g_u);
    u8g2_SetFont(g_u, F_TIME);
    txtC(g_u, cx, 99, buf);
    std::string cell;
    for (int xx = colonX; xx < colonX + colon; xx++)
      for (int y = 57; y <= 98; y++) cell += inkAt(xx, y) ? '#' : '.';
    if (oldRef.empty()) oldRef = cell;
    if (cell != oldRef) oldMoved++;
  }
  printf("12 hour clock: %d frames, %d with a moving colon, %d with digits out of place\n", frames, moved, secBad);
  bad += moved + secBad;
  if (oldMoved == 0) {
    printf("  CHECK IS BLIND: centring \"9:05\" on its ink width did not move the colon\n");
    bad++;
  } else {
    printf("  negative control OK: centring on the ink width moves the colon in %d of 12 hours\n", oldMoved);
  }

  // every date fits the date line (rows 22..52, x 186..396), in every format
  int overflow = 0, dates = 0, widest = 0;
  for (int fmt = 0; fmt < DATE_FORMAT_COUNT; fmt++)
    for (int mo = 0; mo < 12; mo++)
      for (int d : {1, 9, 10, 28, 30}) {
        UiModel m = baseModel();
        m.dateFormat = (uint8_t)fmt;
        m.local.tm_mon = mo;
        m.local.tm_mday = d;
        m.spotify.status = SPOTIFY_IDLE;
        renderModel(m);
        dates++;
        const int left = leftmostInk(kRightX0 - 10, kRightX1 + 4, 22, 52);
        int right = -1;
        for (int x = kRightX1 + 3; x >= kRightX0 - 10; x--)
          if (anyInk(x, x + 1, 22, 52)) {
            right = x;
            break;
          }
        if (right - left + 1 > widest) widest = right - left + 1;
        if (left < kRightX0 || right > kRightX1) {
          if (overflow++ < 3) printf("  DATE OUT OF BOUNDS: format %d month %d day %d: ink %d..%d\n", fmt, mo + 1, d, left, right);
        }
      }
  printf("date formats: %d dates, widest %d px of %d, %d out of bounds\n", dates, widest, kRightX1 - kRightX0 + 1, overflow);
  bad += overflow;
  {  // negative control: a date in the big time font is too wide, and the same measurement must say so
    u8g2_ClearBuffer(g_u);
    u8g2_SetFontMode(g_u, 1);
    ink(g_u);
    u8g2_SetFont(g_u, F_TIME);
    txtPitchC(g_u, cx, 51, "2026-10-04");
    const int left = leftmostInk(kRightX0 - 60, kRightX1 + 4, 22, 52);
    if (left >= kRightX0) {
      printf("  CHECK IS BLIND: a date 270 px wide did not leave the date line (ink starts at x=%d)\n", left);
      bad++;
    } else {
      printf("  negative control OK: an oversized date starts at x=%d, left of the %d it must not pass\n", left, kRightX0);
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// The other pages: nothing may run off the right or bottom edge (the legend is dense).
// ---------------------------------------------------------------------------
static int checkPageEdges() {
  int bad = 0;
  // ink in the columns x0..x1 below the status bar, apart from the rows of the full-width rules
  auto edgeInk = [&](int x0, int x1) {
    for (int y = 22; y <= 296; y++) {
      if (y == 196 || y == 199 || y == 223 || y == 256) continue;
      if (anyInk(x0, x1, y, y)) return true;
    }
    return false;
  };
  const UiPage pages[] = {PAGE_INFO, PAGE_POWER, PAGE_LEGEND, PAGE_NOW_PLAYING, PAGE_DASHBOARD};
  for (UiPage p : pages) {
    UiModel m = baseModel();
    m.page = p;
    m.infoCount = UI_INFO_LINES;
    for (int i = 0; i < UI_INFO_LINES; i++) {
      snprintf(m.info[i], UI_INFO_LINE_LEN, "%-9s %.*s", "Label", UI_INFO_LINE_LEN - 12,
               "012345678901234567890123456789012345678901234567890123456789");
    }
    renderModel(m);
    const bool right = edgeInk(UI_WIDTH - 3, UI_WIDTH), bottom = anyInk(0, UI_WIDTH, 298, 300);  // (the progress bar ends on row 297)
    const bool left = edgeInk(0, 4);
    if (right || bottom || (left && p != PAGE_DASHBOARD)) {
      printf("  PAGE %d: ink at the edge (right %d, bottom %d, left %d)\n", (int)p, right, bottom, left);
      bad++;
    }
  }
  // the battery-empty screen too
  u8g2_ClearBuffer(g_u);
  uiDrawBatteryEmpty(g_u, 3.28f, 3.70f, true);
  if (anyInk(UI_WIDTH - 3, UI_WIDTH, 0, 299) || anyInk(0, UI_WIDTH, 297, 300) || anyInk(0, 3, 0, 299)) {
    printf("  BATTERY-EMPTY SCREEN: ink at the edge\n");
    bad++;
  }
  {  // negative control: a pixel in the last column, and one in the last row, must be seen
    UiModel m = baseModel();
    m.page = PAGE_INFO;
    renderModel(m);
    u8g2_DrawPixel(g_u, UI_WIDTH - 1, 120);
    const bool right = edgeInk(UI_WIDTH - 3, UI_WIDTH);
    renderModel(m);
    u8g2_DrawPixel(g_u, 120, 299);
    const bool bottom = anyInk(0, UI_WIDTH, 298, 300);
    if (!right || !bottom) {
      printf("  CHECK IS BLIND: an edge pixel was not seen (right %d, bottom %d)\n", right, bottom);
      bad++;
    } else {
      printf("  negative control OK: edge pixels are seen\n");
    }
  }
  printf("page edges: %d problems\n", bad);
  return bad;
}

// ---------------------------------------------------------------------------
// The moon glyph as drawn in the UI: an outline that fills with the lit part, mirrored in the
// south, and nothing outside its 10 x 10 box.
// ---------------------------------------------------------------------------
static int inkIn(int x, int y, int w, int h) {
  int n = 0;
  for (int yy = y; yy < y + h; yy++)
    for (int xx = x; xx < x + w; xx++) n += inkAt(xx, yy);
  return n;
}

static int checkMoonGlyph() {
  int bad = 0;
  int last = -1;
  for (int pct = 0; pct <= 100; pct += 5) {
    u8g2_ClearBuffer(g_u);
    ink(g_u);
    iconMoon(g_u, 50, 50, kMoonD, pct, true, false);
    const int n = inkIn(50, 50, kMoonD, kMoonD);
    const int outside = inkIn(40, 40, 30, 30) - n;
    if (n < last || outside != 0) {
      printf("  MOON %d%%: %d ink pixels (previous %d), %d outside the box\n", pct, n, last, outside);
      bad++;
    }
    last = n;
    // northern growing moon: lit part on the right; southern: on the left
    if (pct == 50) {
      const int leftN = inkIn(50, 50, 5, kMoonD), rightN = inkIn(55, 50, 5, kMoonD);
      u8g2_ClearBuffer(g_u);
      iconMoon(g_u, 50, 50, kMoonD, pct, true, true);
      const int leftS = inkIn(50, 50, 5, kMoonD), rightS = inkIn(55, 50, 5, kMoonD);
      if (!(rightN > leftN && leftS > rightS && leftS == rightN && rightS == leftN)) {
        printf("  MOON: the picture is not mirrored for the south (N %d/%d, S %d/%d)\n", leftN, rightN, leftS, rightS);
        bad++;
      }
    }
  }
  // new moon: a ring; full moon: a solid disc
  u8g2_ClearBuffer(g_u);
  iconMoon(g_u, 50, 50, kMoonD, 0, true, false);
  const int ring = inkIn(50, 50, kMoonD, kMoonD);
  u8g2_ClearBuffer(g_u);
  iconMoon(g_u, 50, 50, kMoonD, 100, true, false);
  const int disc = inkIn(50, 50, kMoonD, kMoonD);
  printf("moon glyph: a new moon is a ring of %d pixels, a full moon a disc of %d, %d problems\n", ring, disc, bad);
  if (!(ring > 20 && ring < disc && disc > 60)) bad++;
  return bad;
}

// ---------------------------------------------------------------------------
// Rain glyph.  It has to read as a drop and not as text: mirror-symmetric (three slashes are not) and of
// the size ui.cpp says.  Wherever it is drawn (today's row, both forecast days, the legend) it must be
// there intact exactly once, end on the same row as the digits beside it, and keep 2 to 5 blank columns
// from them (the moon's own gap is 2, plus the side bearing of the first digit; the forecast days add 1
// for the centring).  The glyph is found by matching the pixels iconRain() itself draws.
// ---------------------------------------------------------------------------
static int lowestInk(int x0, int x1, int y0, int y1) {
  for (int y = y1; y >= y0; y--)
    for (int x = x0; x < x1; x++)
      if (inkAt(x, y)) return y;
  return -1;
}

// size of the ink in the window, and how many of its pixels have no partner mirrored about its vertical axis
static int asymmetry(int x0, int y0, int w, int h, int &bw, int &bh) {
  int minX = 1 << 20, maxX = -1, minY = 1 << 20, maxY = -1;
  for (int y = y0; y < y0 + h; y++)
    for (int x = x0; x < x0 + w; x++)
      if (inkAt(x, y)) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
  bw = maxX - minX + 1;
  bh = maxY - minY + 1;
  int odd = 0;
  for (int y = minY; y <= maxY; y++)
    for (int x = minX; x <= maxX; x++)
      if (inkAt(x, y) != inkAt(maxX - (x - minX), y)) odd++;
  return odd;
}

static bool g_drop[kRainH][kRainW];

static void captureDrop() {
  u8g2_ClearBuffer(g_u);
  ink(g_u);
  iconRain(g_u, 50, 50);
  for (int r = 0; r < kRainH; r++)
    for (int c = 0; c < kRainW; c++) g_drop[r][c] = inkAt(50 + c, 50 + r);
}

// How often the glyph appears intact inside the window (x0 <= x < x1, y0 <= y <= y1), and where first.
static int findDrop(int x0, int x1, int y0, int y1, int &fx, int &fy) {
  int n = 0;
  for (int y = y0; y + kRainH - 1 <= y1; y++)
    for (int x = x0; x + kRainW <= x1; x++) {
      bool same = true;
      for (int r = 0; r < kRainH && same; r++)
        for (int c = 0; c < kRainW; c++)
          if (inkAt(x + c, y + r) != g_drop[r][c]) {
            same = false;
            break;
          }
      if (same && n++ == 0) {
        fx = x;
        fy = y;
      }
    }
  return n;
}

// glyph bottom row minus the bottom row of the text to its right, and the blank columns between them
// (-1 for both when there is no text: the narrowest form shows the glyph alone)
static bool measureDrop(int fx, int fy, int &bottomDelta, int &gap) {
  const int textLeft = leftmostInk(fx + kRainW, fx + kRainW + 12, fy, fy + kRainH - 1);
  if (textLeft < 0) {
    bottomDelta = gap = -1;
    return false;
  }
  bottomDelta = (fy + kRainH - 1) - lowestInk(fx + kRainW + 1, fx + kRainW + 26, fy - 1, fy + kRainH + 2);
  gap = textLeft - (fx + kRainW);
  return true;
}

static int checkRainGlyph() {
  int bad = 0;

  // the shape on its own
  u8g2_ClearBuffer(g_u);
  ink(g_u);
  iconRain(g_u, 50, 50);
  int bw, bh;
  const int odd = asymmetry(45, 45, 20, 20, bw, bh);
  u8g2_ClearBuffer(g_u);  // negative control: the three slanted strokes it replaced
  ink(g_u);
  for (int i = 0; i < 3; i++) u8g2_DrawLine(g_u, 50 + i * 3, 50 + 8, 50 + i * 3 + 3, 50);
  int ow, oh;
  const int oldOdd = asymmetry(45, 45, 20, 20, ow, oh);
  printf("rain glyph: %d x %d px, %d pixels without a mirror partner\n", bw, bh, odd);
  if (odd != 0 || bw != kRainW || bh != kRainH) {
    printf("  PROBLEM: the glyph is not a symmetric %d x %d drop\n", kRainW, kRainH);
    bad++;
  }
  if (oldOdd == 0) {
    printf("  CHECK IS BLIND: the old slanted strokes passed as symmetric\n");
    bad++;
  } else {
    printf("  negative control OK: the old strokes have %d pixels without a partner\n", oldOdd);
  }

  // in place: today, both forecast days, for short and wide numbers, both units, and the legend
  captureDrop();
  int placements = 0, missing = 0, offBaseline = 0, badGap = 0;
  auto inspect = [&](const char *where, int n, int fx, int fy) {
    placements++;
    if (n != 1) {
      if (missing++ < 3) printf("  %s: the drop is there %d times (expected once)\n", where, n);
      return;
    }
    int delta, gap;
    if (!measureDrop(fx, fy, delta, gap)) return;
    if (delta != 0 && offBaseline++ < 3) printf("  %s: the drop ends %d rows below the digits\n", where, delta);
    if ((gap < 2 || gap > 5) && badGap++ < 3) printf("  %s: %d blank columns between the drop and its number\n", where, gap);
  };
  const float temps[] = {-25, 5, 99};
  const int rains[] = {0, 35, 100};
  for (float t : temps)
    for (int rain : rains)
      for (int f = 0; f < 2; f++) {
        UiModel m = baseModel();
        m.spotify.status = SPOTIFY_IDLE;
        m.useFahrenheit = f == 1;
        m.weather.temp = t;
        for (int d = 0; d < 3; d++) m.weather.day[d].rainPct = (uint8_t)rain;
        renderModel(m);
        int fx = 0, fy = 0, n;
        char tag[48];
        snprintf(tag, sizeof tag, "today (%.0f deg%s, rain %d%%)", t, f ? " F" : "", rain);
        n = findDrop(60, kColA - 4, kWxTop + 4, kWxTop + 22, fx, fy);
        inspect(tag, n, fx, fy);
        snprintf(tag, sizeof tag, "tomorrow (rain %d%%)", rain);
        n = findDrop(kColA + 60, kColB - 4, kWxTop + 30, kWxTop + 52, fx, fy);
        inspect(tag, n, fx, fy);
        snprintf(tag, sizeof tag, "day after (rain %d%%)", rain);
        n = findDrop(kColB + 60, UI_WIDTH - 4, kWxTop + 30, kWxTop + 52, fx, fy);
        inspect(tag, n, fx, fy);
      }
  {
    UiModel m = baseModel();
    m.page = PAGE_LEGEND;
    renderModel(m);
    int fx = 0, fy = 0;
    const int n = findDrop(0, 60, 80, 106, fx, fy);
    inspect("legend", n, fx, fy);
  }
  printf("rain glyph in place: %d placements, %d missing or doubled, %d off the digits' bottom row, %d badly spaced\n",
         placements, missing, offBaseline, badGap);
  bad += missing + offBaseline + badGap;

  {  // negative controls: one row too low, and a number two pixels too close
    int fx = 0, fy = 0, lowDelta = 0, closeGap = 0, unused = 0;
    u8g2_ClearBuffer(g_u);
    ink(g_u);
    u8g2_SetFont(g_u, F_SMALL);
    iconRain(g_u, 60, 61);                      // rows 61..69
    pctText(g_u, 60 + kRainW + 2, 69, 35, 0);  // digits on rows 61..68
    const bool lowSeen = findDrop(40, 120, 40, 90, fx, fy) == 1 && measureDrop(fx, fy, lowDelta, unused) && lowDelta == 1;
    u8g2_ClearBuffer(g_u);
    ink(g_u);
    iconRain(g_u, 60, 60);  // rows 60..68, the glyph still whole
    pctText(g_u, 60 + kRainW, 69, 35, 0);  // ... with its number 2 px closer than it is drawn
    const bool closeSeen = findDrop(40, 120, 40, 90, fx, fy) == 1 && measureDrop(fx, fy, unused, closeGap) && closeGap < 2;
    if (!lowSeen || !closeSeen) {
      printf("  CHECK IS BLIND: a drop one row too low was %s, a number 2 px too close was %s\n", lowSeen ? "seen" : "missed",
             closeSeen ? "seen" : "missed");
      bad++;
    } else {
      printf("  negative control OK: a drop one row too low reads +%d, a number 2 px too close leaves %d blank columns\n", lowDelta,
             closeGap);
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// Firmware-update screen.  The progress bar must fill in proportion (a 2 px frame, a 1 px gap, then
// (width - 6) * percent / 100 pixels), and nothing, with the longest texts the fields hold, may come near
// the edge of the screen or run into the next line.
// ---------------------------------------------------------------------------
static void fwScreen(UiFwKind kind, const char *what, const char *detail, int percent, bool keep) {
  UiFwScreen s;
  s.kind = kind;
  setStr(s.what, sizeof s.what, what);
  setStr(s.detail, sizeof s.detail, detail);
  s.percent = percent;
  s.keepPowered = keep;
  u8g2_ClearBuffer(g_u);
  uiDrawFirmwareUpdate(g_u, s);
}

// the run of ink along the middle row of the bar, starting where the fill starts
static int fwFillWidth() {
  const int y = kFwBarY + kFwBarH / 2;
  int n = 0;
  while (kFwBarX + 3 + n < kFwBarX + kFwBarW && inkAt(kFwBarX + 3 + n, y)) n++;
  return n;
}

static int checkFirmwareScreen() {
  int bad = 0;
  int wrong = 0, last = -1, frames = 0;
  for (int p = 0; p <= 100; p += (p < 5 || p > 95 ? 1 : 5)) {
    fwScreen(FW_BUSY, "Installing the new firmware", "ESP32-S3-RLCD-Firmware.bin, 1.40 MB", p, true);
    const int want = (kFwBarW - 6) * p / 100;
    const int got = fwFillWidth();
    frames++;
    if (got != want || got < last) {
      if (wrong++ < 3) printf("  BAR AT %d%%: filled %d px, expected %d (previous %d)\n", p, got, want, last);
    }
    last = got;
    // the frame is whole: both ends and a gap between it and the fill
    const int y = kFwBarY + kFwBarH / 2;
    if (!inkAt(kFwBarX, y) || !inkAt(kFwBarX + kFwBarW - 1, y) || inkAt(kFwBarX + 2, y) || inkAt(kFwBarX + kFwBarW - 3, y)) {
      if (wrong++ < 3) printf("  BAR AT %d%%: the frame or its gap is wrong\n", p);
    }
  }
  printf("firmware screen, progress bar: %d fills, %d wrong\n", frames, wrong);
  bad += wrong;
  {  // negative control: a bar scaled by the outer width (no room for the frame) overruns the gap at 100 %
    u8g2_ClearBuffer(g_u);
    ink(g_u);
    u8g2_DrawFrame(g_u, kFwBarX, kFwBarY, kFwBarW, kFwBarH);
    u8g2_DrawFrame(g_u, kFwBarX + 1, kFwBarY + 1, kFwBarW - 2, kFwBarH - 2);
    u8g2_DrawBox(g_u, kFwBarX + 3, kFwBarY + 3, kFwBarW * 100 / 100, kFwBarH - 6);
    if (fwFillWidth() == (kFwBarW - 6)) {
      printf("  CHECK IS BLIND: a bar drawn too wide measured as right\n");
      bad++;
    } else {
      printf("  negative control OK: a bar scaled by the outer width reads %d px instead of %d\n", fwFillWidth(), kFwBarW - 6);
    }
  }

  // the longest texts, every kind of screen: nothing within 3 px of an edge
  const std::string longWhat(63, 'W'), longDetail(63, 'W');
  int edge = 0;
  for (int kind = FW_BUSY; kind <= FW_PROBLEM; kind++)
    for (int keep = 0; keep <= 1; keep++)
      for (int pct : {-1, 0, 100}) {
        fwScreen((UiFwKind)kind, longWhat.c_str(), longDetail.c_str(), pct, keep);
        if (anyInk(UI_WIDTH - 3, UI_WIDTH, 0, 299) || anyInk(0, 3, 0, 299) || anyInk(0, UI_WIDTH, 0, 2) || anyInk(0, UI_WIDTH, 297, 300)) {
          if (edge++ < 3) printf("  FIRMWARE SCREEN: ink at the edge (kind %d, keep %d, %d%%)\n", kind, keep, pct);
        }
      }
  // the texts the clock really shows
  const char *whats[] = {"Checking the file", "Installing the new firmware", "Restarting", "The file is damaged: copy it again",
                         "Too big (the .merged.bin? use the .ino.bin)", "No second app slot: pick the 3MB APP scheme",
                         "This build was installed from the card before", "Battery 3.50 V: plug in USB, then restart",
                         "Built for another chip (need ESP32-S3)", "Cannot read the file (card error)"};
  for (const char *w : whats) {
    fwScreen(FW_PROBLEM, w, "11_Clock_Dashboard.ino.bin, 1.40 MB", -1, false);
    if (anyInk(UI_WIDTH - 3, UI_WIDTH, 0, 299) || anyInk(0, 3, 0, 299)) {
      if (edge++ < 3) printf("  FIRMWARE SCREEN: \"%s\" touches the edge\n", w);
    }
  }
  printf("firmware screen, edges: %d problems\n", edge);
  bad += edge;
  {  // negative control: ink in the last column is seen
    fwScreen(FW_BUSY, "x", "y", 10, true);
    u8g2_DrawPixel(g_u, UI_WIDTH - 1, 120);
    if (!anyInk(UI_WIDTH - 3, UI_WIDTH, 0, 299)) {
      printf("  CHECK IS BLIND: a pixel in the last column was not seen\n");
      bad++;
    } else {
      printf("  negative control OK: edge pixels are seen\n");
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// The help for the buttons on the legend page, and the longest banner.  KEY's four lines must stop
// short of the BOOT column beside them, and the banner's text must fit its band with room at both ends.
// ---------------------------------------------------------------------------
static int checkButtonHelp() {
  int bad = 0;
  const int kBootX = 208, kGap = 10, kRow0 = 241, kRow1 = 297;  // BOOT's column, the least gap before it, the rows of the four lines
  auto keyRightEdge = [&]() {  // the last column with ink left of the BOOT column
    for (int x = kBootX - 1; x >= 8; x--)
      if (anyInk(x, x + 1, kRow0, kRow1)) return x;
    return -1;
  };
  UiModel m = baseModel();
  m.page = PAGE_LEGEND;
  renderModel(m);
  const int right = keyRightEdge();
  int lines = 0;
  for (int i = 0; i < 4; i++) lines += anyInk(8, 150, 252 + i * 13 - 9, 252 + i * 13) ? 1 : 0;  // each of KEY's lines is there
  const bool bootThere = anyInk(kBootX, kBootX + 60, kRow0, kRow1);
  printf("legend buttons: KEY's lines end at x=%d, %d px short of the BOOT column (%d wanted), %d of 4 lines drawn\n", right,
         kBootX - 1 - right, kGap, lines);
  if (right < 0 || right > kBootX - 1 - kGap || lines != 4 || !bootThere) {
    printf("  LEGEND: KEY's lines run into the BOOT column, or a line is missing\n");
    bad++;
  }
  {  // negative control: a line that is too long is seen to reach the BOOT column
    renderModel(m);
    u8g2_SetFont(g_u, F_BODY);
    ink(g_u);
    txt(g_u, 8, 291, "hold: refresh now; hold 5 s: restart the clock");
    const int r = keyRightEdge();
    if (r <= kBootX - 1 - kGap) {
      printf("  CHECK IS BLIND: a line of 46 characters did not reach the BOOT column (ends at x=%d)\n", r);
      bad++;
    } else {
      printf("  negative control OK: a line that is too long ends at x=%d, inside the gap\n", r);
    }
  }

  // The banner: a black band with white text, centred.  The columns its text takes up:
  auto bannerText = [&](const char *text, int icon, int *left, int *rightEnd) {
    UiModel t = baseModel();
    setStr(t.toast, sizeof t.toast, text);
    t.toastIcon = icon;
    renderModel(t);
    *left = *rightEnd = -1;
    for (int x = 0; x < UI_WIDTH; x++) {
      bool paperHere = false;
      for (int y = kBottomTop + 2; y <= UI_HEIGHT - 3 && !paperHere; y++) paperHere = !inkAt(x, y);
      if (paperHere) {
        if (*left < 0) *left = x;
        *rightEnd = x;
      }
    }
  };
  const int kMargin = 12;
  int l = -1, r = -1;
  bannerText(UI_TOAST_RESTART_HOLD, TOAST_NONE, &l, &r);
  printf("restart banner: \"%s\" takes x=%d..%d, margins %d and %d (%d wanted)\n", UI_TOAST_RESTART_HOLD, l, r, l, UI_WIDTH - 1 - r, kMargin);
  if (l < kMargin || r > UI_WIDTH - 1 - kMargin || strlen(UI_TOAST_RESTART_HOLD) >= sizeof m.toast) {
    printf("  BANNER: the restart text does not fit\n");
    bad++;
  }
  bannerText(UI_TOAST_RESTARTING, TOAST_NONE, &l, &r);
  if (l < kMargin || r > UI_WIDTH - 1 - kMargin) {
    printf("  BANNER: \"%s\" does not fit\n", UI_TOAST_RESTARTING);
    bad++;
  }
  // what the clock says about an SD card that will not mount (sd_layout.h), each with the warning mark it is shown with
  int widest = 0, sdBad = 0, sdTexts = 0;
  for (sdlayout::Kind k : sdlayout::kAllKinds) {
    const char *text = sdlayout::text(k);
    bannerText(text, TOAST_WARN, &l, &r);
    sdTexts++;
    if (r - l + 1 > widest) widest = r - l + 1;
    if (l < kMargin || r > UI_WIDTH - 1 - kMargin || strlen(text) >= sizeof m.toast) {
      printf("  BANNER: \"%s\" does not fit (x=%d..%d)\n", text, l, r);
      sdBad++;
    }
  }
  printf("SD card banners: %d texts, the widest takes %d px of %d, %d do not fit\n", sdTexts, widest, UI_WIDTH - 2 * kMargin, sdBad);
  bad += sdBad;
  // what the clock says about the settings file of a card (settings.h), with counts beyond what a file can hold
  {
    int cfgTexts = 0, cfgBad = 0, cfgWidest = 0;
    char widestText[sizeof m.toast] = "";
    for (int changed : {0, 8888})
      for (int unchanged : {0, 8888})
        for (int problems : {0, 8888})
          for (int carried : {0, 8888})
            for (int flags = 0; flags < 4; flags++) {
              ConfigReport rep;
              rep.changed = changed;
              rep.unchanged = unchanged;
              rep.unknown = problems;
              char text[sizeof m.toast];
              const bool warn = formatSdBanner(rep, (flags & 1) != 0, carried, (flags & 2) != 0, text, sizeof text);
              bannerText(text, warn ? TOAST_WARN : TOAST_NONE, &l, &r);
              cfgTexts++;
              if (r - l + 1 > cfgWidest) {
                cfgWidest = r - l + 1;
                setStr(widestText, sizeof widestText, text);
              }
              if (l < kMargin || r > UI_WIDTH - 1 - kMargin) {
                if (cfgBad++ < 3) printf("  BANNER: \"%s\" does not fit (x=%d..%d)\n", text, l, r);
              }
            }
    printf("settings banners: %d texts, the widest (\"%s\") takes %d px of %d, %d do not fit\n", cfgTexts, widestText, cfgWidest,
           UI_WIDTH - 2 * kMargin, cfgBad);
    bad += cfgBad;
  }
  {  // negative control: the longest text a banner can be given does not fit, and the same measurement says so
    const std::string longest(sizeof m.toast - 1, 'W');
    bannerText(longest.c_str(), TOAST_NONE, &l, &r);
    if (l >= kMargin && r <= UI_WIDTH - 1 - kMargin) {
      printf("  CHECK IS BLIND: %d wide letters measured as fitting (x=%d..%d)\n", (int)longest.size(), l, r);
      bad++;
    } else {
      printf("  negative control OK: %d wide letters take x=%d..%d, past the margins\n", (int)longest.size(), l, r);
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// The outline of the analogue clock: a ring two pixels wide with no pixel missing.  It used to be two circles,
// one inside the other, and wherever the two rounded differently a single pixel between them stayed white:
// seen on the panel as holes in the outline.
// ---------------------------------------------------------------------------
static int checkClockRing() {
  int bad = 0;
  // white pixels well inside the band of the ring, and black ones just outside it
  auto count = [&](int *holes, int *spill) {
    *holes = *spill = 0;
    for (int y = kClockCy - kClockR - 4; y <= kClockCy + kClockR + 4; y++) {
      for (int x = kClockCx - kClockR - 4; x <= kClockCx + kClockR + 4; x++) {
        const float d = sqrtf((float)((x - kClockCx) * (x - kClockCx) + (y - kClockCy) * (y - kClockCy)));
        if (d >= (float)kClockR - 1.2f && d <= (float)kClockR + 0.2f && !inkAt(x, y)) (*holes)++;
        if (d >= (float)kClockR + 1.2f && d <= (float)kClockR + 3.0f && inkAt(x, y)) (*spill)++;
      }
    }
  };
  int holes = 0, spill = 0, frames = 0;
  for (int hour = 0; hour < 12; hour++) {  // (the hands never reach the ring; a few times of day all the same)
    UiModel m = baseModel();
    m.local.tm_hour = hour;
    m.local.tm_min = hour * 5;
    m.local.tm_sec = (hour * 7) % 60;
    renderModel(m);
    int h, s;
    count(&h, &s);
    holes += h;
    spill += s;
    frames++;
  }
  printf("clock outline: %d frames, %d pixels missing from the ring, %d outside it\n", frames, holes, spill);
  if (holes || spill) {
    printf("  CLOCK: the outline has holes in it, or ink beside it\n");
    bad++;
  }
  {  // negative control: the two circles it used to be
    u8g2_ClearBuffer(g_u);
    ink(g_u);
    u8g2_DrawCircle(g_u, kClockCx, kClockCy, kClockR, U8G2_DRAW_ALL);
    u8g2_DrawCircle(g_u, kClockCx, kClockCy, kClockR - 1, U8G2_DRAW_ALL);
    int oldHoles, oldSpill;
    count(&oldHoles, &oldSpill);
    if (oldHoles == 0) {
      printf("  CHECK IS BLIND: drawn as two circles the outline shows no holes\n");
      bad++;
    } else {
      printf("  negative control OK: drawn as two circles, %d pixels of the ring are missing\n", oldHoles);
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// The turning cubes that both buttons bring up (ui.cpp; infinity_cube.h, sand_cube.h): white on black.  Whichever
// way a cube has turned, it has to stay clear of the edges of the screen, and it has to be a picture: neither
// nothing nor a white screen.
// ---------------------------------------------------------------------------
static uint32_t g_cubeSeed = 1;
static void drawCube(bool mirrors, float turnMs, uint32_t elapsedMs, const float *shove = nullptr) {
  UiCube c;
  c.mirrors = mirrors;
  c.seed = g_cubeSeed;
  c.turnMs = turnMs;
  c.elapsedMs = elapsedMs;
  for (int i = 0; i < 3; i++) c.shove[i] = shove ? shove[i] : 0.0f;
  uiDrawCube(g_u, c);
}
static void drawMirrorCube(u8g2_t *, uint32_t ms) { drawCube(true, (float)ms, ms); }
static void drawSandCube(u8g2_t *, uint32_t ms) { drawCube(false, (float)ms, ms); }

static int checkCube(const char *name, void (*drawFn)(u8g2_t *, uint32_t), int leastWanted) {
  int bad = 0;
  const int kMargin = 8;
  auto litOutside = [&]() {  // white pixels in the band of kMargin along the four edges
    int n = 0;
    for (int y = 0; y < UI_HEIGHT; y++)
      for (int x = 0; x < UI_WIDTH; x++) {
        const bool band = x < kMargin || x >= UI_WIDTH - kMargin || y < kMargin || y >= UI_HEIGHT - kMargin;
        if (band && !inkAt(x, y)) n++;
      }
    return n;
  };
  int frames = 0, least = 1 << 30, most = 0, touching = 0, top = UI_HEIGHT, bottom = 0;
  for (uint32_t ms = 0; ms < 180000; ms += 131, frames++) {  // three minutes of it: every turn of the cube, every stage of the fall
    u8g2_ClearBuffer(g_u);
    drawFn(g_u, ms);
    if (litOutside() > 0) touching++;
    int lit = 0;
    for (int y = 0; y < UI_HEIGHT; y++) {
      bool row = false;
      for (int x = 0; x < UI_WIDTH; x++)
        if (!inkAt(x, y)) {
          lit++;
          row = true;
        }
      if (row && y < top) top = y;
      if (row && y > bottom) bottom = y;
    }
    if (lit < least) least = lit;
    if (lit > most) most = lit;
  }
  printf("%s: %d frames, %d to %d white pixels of %d, rows %d to %d, %d touch the %d px margin\n", name, frames, least, most,
         UI_WIDTH * UI_HEIGHT, top, bottom, touching, kMargin);
  if (touching || least < leastWanted || most > UI_WIDTH * UI_HEIGHT / 4 || top > 40 || bottom < UI_HEIGHT - 41) {
    printf("  CUBE: it leaves the screen, fills it, or is too small to be seen\n");
    bad++;
  }
  {  // negative control: a pixel in the margin is seen, and so is a frame with nothing in it
    u8g2_ClearBuffer(g_u);
    drawFn(g_u, 0);
    u8g2_SetDrawColor(g_u, 0);
    u8g2_DrawPixel(g_u, kMargin - 1, UI_HEIGHT / 2);
    u8g2_SetDrawColor(g_u, 1);
    const int seen = litOutside();
    u8g2_ClearBuffer(g_u);
    u8g2_DrawBox(g_u, 0, 0, UI_WIDTH, UI_HEIGHT);
    int lit = 0;
    for (int y = 0; y < UI_HEIGHT; y++)
      for (int x = 0; x < UI_WIDTH; x++) lit += !inkAt(x, y);
    if (seen != 1 || lit != 0) {
      printf("  CHECK IS BLIND: a white pixel in the margin counted %d, a black screen %d white pixels\n", seen, lit);
      bad++;
    } else {
      printf("  negative control OK: a pixel in the margin is seen, a black screen has none\n");
    }
  }
  return bad;
}

int main() {
  mkdir("out", 0755);
  g_u = hostDisplayInit();

  // 1. normal dashboard, Spotify playing, running on the battery
  UiModel m = baseModel();
  m.batCharge = CHARGE_DISCHARGING;
  snapshot("dash_playing", m);

  // 2. nothing playing -> extras strip
  m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  snapshot("dash_idle", m);

  // 3. low battery, both blink phases
  m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  m.batPercent = 17;
  m.batVolts = 3.66f;
  m.batLow = true;
  m.batCharge = CHARGE_DISCHARGING;
  m.batBlinkOn = true;
  snapshot("dash_lowbat_on", m);
  m.batBlinkOn = false;
  snapshot("dash_lowbat_off", m);

  // 3b. what the battery is doing: running on it, charging, full; and low while charging (no blink)
  m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  m.batCharge = CHARGE_DISCHARGING;
  snapshot("dash_bat_discharging", m);
  m.batCharge = CHARGE_CHARGING;
  m.batPercent = 62;
  snapshot("dash_bat_charging", m);
  m.batCharge = CHARGE_FULL;
  m.batPercent = 100;
  snapshot("dash_bat_full", m);
  m.batCharge = CHARGE_CHARGING;
  m.batPercent = 17;
  m.batVolts = 3.66f;
  m.batLow = false;  // the sketch holds the warning back while charging
  snapshot("dash_lowbat_charging", m);
  m.batCharge = CHARGE_DISCHARGING;
  m.batLow = true;
  m.batBlinkOn = true;
  snapshot("dash_lowbat_discharging_on", m);

  // 4. just booted: no time, no weather, no wifi
  m = baseModel();
  m.timeValid = false;
  m.weather = WeatherData();
  m.wifiUp = false;
  m.indoorValid = false;
  m.batPresent = false;
  m.spotify = SpotifyInfo();
  setStr(m.status, sizeof m.status, "Connecting to WiFi...");
  snapshot("dash_boot", m);

  // 5. long / unicode title
  m = baseModel();
  setStr(m.spotify.title, sizeof m.spotify.title,
         "Don\xE2\x80\x99t Stop Me Now \xE2\x80\x93 Remastered 2011 (Caf\xC3\xA9 del Mar Extended Mix) \xE5\xA4\x9C\xE3\x81\xAB\xE9\xA7\x86\xE3\x81\x91\xE3\x82\x8B");
  setStr(m.spotify.artist, sizeof m.spotify.artist, "Queen, David Bowie, Freddie Mercury & Friends");
  snapshot("dash_longtitle", m);

  // 6. toast + paused
  m = baseModel();
  m.spotify.status = SPOTIFY_PAUSED;
  setStr(m.toast, sizeof m.toast, "Next track");
  m.toastIcon = TOAST_NEXT;
  snapshot("dash_toast_next", m);
  {  // the banner while KEY is held for a restart (the longest one)
    UiModel hold = m;
    setStr(hold.toast, sizeof hold.toast, UI_TOAST_RESTART_HOLD);
    hold.toastIcon = TOAST_NONE;
    snapshot("dash_toast_restart", hold);
  }

  // 7. Fahrenheit, rainy, stale weather
  m = baseModel();
  m.useFahrenheit = true;
  m.weather.code = 63;
  m.weather.ageSec = 4000;
  m.weather.day[1].code = 95;
  m.weather.day[2].code = 71;
  m.spotify.status = SPOTIFY_NEEDS_LINK;
  setStr(m.spotify.linkUrl, sizeof m.spotify.linkUrl, "http://192.168.1.50");
  snapshot("dash_fahrenheit", m);

  // 7b. link page not up yet / link about to expire
  m = baseModel();
  m.spotify.status = SPOTIFY_NEEDS_LINK;
  snapshot("dash_link_wait", m);
  m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  m.spotify.linkDaysLeft = 9;
  setStr(m.spotify.linkUrl, sizeof m.spotify.linkUrl, "http://192.168.1.50");
  snapshot("dash_link_renew", m);

  // 8. Now Playing pages
  m = baseModel();
  m.page = PAGE_NOW_PLAYING;
  snapshot("np_playing", m);
  setStr(m.spotify.title, sizeof m.spotify.title,
         "Bohemian Rhapsody - Remastered 2011 (Live at Wembley Stadium 1986 Extended Version)");
  setStr(m.spotify.artist, sizeof m.spotify.artist, "Queen");
  m.spotify.status = SPOTIFY_PAUSED;
  snapshot("np_long_paused", m);
  m.spotify = SpotifyInfo();
  m.spotify.status = SPOTIFY_IDLE;
  snapshot("np_idle", m);

  // 9. Info page
  m = baseModel();
  m.page = PAGE_INFO;
  m.batCharge = CHARGE_DISCHARGING;
  const char *lines[] = {
      "Firmware  RLCD Clock 1.4, 2026-10-07, build 3f9a12c",
      "WiFi      HomeNet  -58 dBm",
      "Address   192.168.1.50  rlcd-clock.local",
      "Time      NTP synced 12 min ago, last step -38 ms",
      "Zone      Australia/Perth  AWST-8",
      "Location  -31.952, 115.861",
      "Weather   2 min ago - Open-Meteo.com",
      "Battery   4.12 V  87%  discharging -0.1 mV/min",
      "Left      3 d 0 h (1.20 %/h over 300 min)",
      "Indoor    22.4 C  45%   (sensor 26.4 C)",
      "Spotify   linked, playing (link: 152 d left)",
      "Uptime    0d 3h 12m, 187 KB free",
      "Frames    11520 sent, 2 late, worst 61 ms",
  };
  m.infoCount = 13;
  for (int i = 0; i < 13; i++) setStr(m.info[i], UI_INFO_LINE_LEN, lines[i]);
  snapshot("info", m);

  // 9b. the other pages
  m = baseModel();
  m.page = PAGE_LEGEND;
  snapshot("legend", m);
  m.page = PAGE_POWER;
  m.batCharge = CHARGE_DISCHARGING;
  {
    const char *power[] = {
        // (the same clock as on the Info page: 87 % of a 2500 mAh cell, drawing 30 mA with WiFi always on)
        "Power     80 MHz, WiFi saver normal",
        "Sleep     100.0 % of the time (now: button pressed)",  // (light_sleep = on: the widest the row gets)
        "Battery   4.117 V = 87.0 % on the curve",
        "Left      3 d 0 h (1.20 %/h over 300 min)",
        "Current   about 30.0 mA (of 2500 mAh)",
        "Shutdown  at 3.30 V, back on at 3.70 V",
        "Display   low power mode chosen, not in it",  // (display_power = low: the wider of its two)
        "Drift     +1.8 ppm = +0.16 s/day over 18 h",
        "Config    SD: 7 changed, 1 unchanged, 2 problems",
        "          line 5: 'wifi': expected on or off",
        "          line 9: unknown setting 'colour'",
        "Units     metric, 24 h, ISO date, week numbers",
    };
    m.infoCount = 12;
    for (int i = 0; i < 12; i++) setStr(m.info[i], UI_INFO_LINE_LEN, power[i]);
  }
  snapshot("power", m);
  u8g2_ClearBuffer(g_u);
  uiDrawBatteryEmpty(g_u, 3.28f, 3.70f, true);
  hostDumpPgm("out/battery_empty.pgm");
  printf("rendered out/battery_empty.pgm\n");
  fwScreen(FW_BUSY, "Installing the new firmware", "ESP32-S3-RLCD-Firmware.bin, 1.40 MB", 42, true);
  hostDumpPgm("out/fw_installing.pgm");
  fwScreen(FW_PROBLEM, "Too big (the .merged.bin? use the .ino.bin)", "11_Clock_Dashboard.ino.merged.bin, 16.00 MB", -1, false);
  hostDumpPgm("out/fw_problem.pgm");
  fwScreen(FW_DONE, "Restarting", "ESP32-S3-RLCD-Firmware.bin, 1.40 MB", 100, false);
  hostDumpPgm("out/fw_done.pgm");
  printf("rendered out/fw_*.pgm\n");

  // 9b2. wide numbers squeeze the weather band: a cold day, a hot day in Fahrenheit
  m = baseModel();
  m.spotify.status = SPOTIFY_IDLE;
  m.weather.temp = -12.4f;
  m.weather.code = 71;
  m.weather.day[0].tmax = -8;
  m.weather.day[0].tmin = -15;
  m.weather.day[0].rainPct = 100;
  m.weather.day[1] = {73, -9.0f, -17.0f, 100, 1};
  m.weather.day[2] = {75, -12.0f, -21.0f, 85, 2};
  m.moonLit[0] = 100;
  m.moonLit[1] = 99;
  snapshot("dash_cold", m);
  m.useFahrenheit = true;
  m.weather.temp = 40.0f;
  m.weather.day[0].tmax = 43;
  m.weather.day[0].tmin = 38;
  m.weather.day[1] = {1, 41.0f, 36.0f, 100, 1};
  snapshot("dash_hot_f", m);

  // 9c. settings: 12 hour clock, other date formats, WiFi off
  m = baseModel();
  m.time12h = true;
  setTime(m, 21, 5, 9);
  m.dateFormat = DATE_D_MON_Y;
  m.showWeek = false;
  m.spotify.status = SPOTIFY_IDLE;
  snapshot("dash_12h_pm", m);
  setTime(m, 0, 7, 59);
  m.dateFormat = DATE_MDY;
  snapshot("dash_12h_midnight", m);
  m = baseModel();
  m.wifiOff = true;
  m.weather = WeatherData();
  m.spotify = SpotifyInfo();
  snapshot("dash_wifi_off", m);
  m.timeValid = false;
  snapshot("dash_wifi_off_no_time", m);

  // 10. clock face gallery: four times at a smaller radius
  {
    u8g2_ClearBuffer(g_u);
    u8g2_SetFontMode(g_u, 1);
    ink(g_u);
    const int times[4][3] = {{10, 9, 35}, {3, 0, 0}, {12, 0, 0}, {18, 45, 59}};
    for (int i = 0; i < 4; i++) {
      UiModel c = baseModel();
      setTime(c, times[i][0], times[i][1], times[i][2]);
      c.secFrac = 0;
      drawAnalogClock(g_u, 100 + (i % 2) * 200, 75 + (i / 2) * 150, 70, c);
    }
    hostDumpPgm("out/clocks.pgm");
    printf("rendered out/clocks.pgm\n");
  }

  // 11. weather icon gallery (day), then night
  for (int night = 0; night < 2; night++) {
    u8g2_ClearBuffer(g_u);
    u8g2_SetFontMode(g_u, 1);
    ink(g_u);
    const WxIcon icons[8] = {WX_CLEAR, WX_PARTLY, WX_CLOUDY, WX_FOG, WX_DRIZZLE, WX_RAIN, WX_SNOW, WX_THUNDER};
    for (int i = 0; i < 8; i++) {
      drawWxIcon(g_u, icons[i], !night, 6 + i * 48, 10, 42);
      drawWxIcon(g_u, icons[i], !night, 10 + i * 48, 70, 32);
      drawWxIcon(g_u, icons[i], !night, 12 + i * 48, 120, 24);
      drawWxIcon(g_u, icons[i], !night, 12 + i * 48, 160, 64);
    }
    hostDumpPgm(night ? "out/icons_night.pgm" : "out/icons_day.pgm");
    printf("rendered icons (%s)\n", night ? "night" : "day");
  }
  int bad = checkLayoutStability();  // run_preview.sh fails if any number wobbles...
  bad += checkLabelPlates();         // ...or a label is off-centre on its plate...
  bad += checkBatteryGauge();        // ...or the battery gauge moves or collides with the status message...
  bad += checkWeatherBand();         // ...or the weather band is misaligned or touches its separators...
  bad += checkClockFormats();        // ...or the 12 hour clock or a date format misbehaves...
  bad += checkPageEdges();           // ...or a page runs off the screen...
  bad += checkMoonGlyph();           // ...or the moon is drawn wrongly...
  bad += checkRainGlyph();           // ...or the rain drop is not a drop, or not where its number is...
  bad += checkFirmwareScreen();      // ...or the firmware-update screen's bar or text is off...
  bad += checkButtonHelp();          // ...or the legend's button help or the restart banner does not fit...
  bad += checkClockRing();           // ...or the clock's outline has a hole in it...
  for (uint32_t seed : {20261008u, 7u}) {  // ...or a cube leaves the screen, whichever way it tumbles
    g_cubeSeed = seed;
    bad += checkCube("mirror cube", drawMirrorCube, 2000);
    bad += checkCube("sand cube", drawSandCube, 2000);
  }

  // the cube as a strip of frames (to_png.py, or any tool that reads PGM, makes a film of them)
  infcube::Shove shove;  // (a shove now and then, as KEY gives it)
  for (int i = 0; i < 72; i++) {
    if (i == 30 || i == 52) shove.bump(0x2545F491u * (uint32_t)i);
    shove.advance(85.0f);
    u8g2_ClearBuffer(g_u);
    drawCube(true, (float)i * 85.0f, (uint32_t)i * 85, shove.angle);
    char path[64];
    snprintf(path, sizeof path, "out/cube_%02d.pgm", i);
    hostDumpPgm(path);
  }
  shove = infcube::Shove();
  for (int i = 0; i < 160; i++) {
    if (i == 60 || i == 110) shove.bump(0x9E3779B9u * (uint32_t)i);
    shove.advance(70.0f);
    u8g2_ClearBuffer(g_u);
    drawCube(false, (float)i * 70.0f, (uint32_t)i * 70, shove.angle);
    char path[64];
    snprintf(path, sizeof path, "out/sand_%03d.pgm", i);
    hostDumpPgm(path);
  }
  printf("rendered out/cube_00.pgm .. cube_71.pgm\n");
  return bad ? 1 : 0;
}
