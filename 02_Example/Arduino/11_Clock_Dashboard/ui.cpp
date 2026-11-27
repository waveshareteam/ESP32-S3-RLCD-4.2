#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "datefmt.h"
#include "infinity_cube.h"
#include "sand_cube.h"
#include "moon.h"
#include "timeutil.h"
#include "weather_codes.h"

#define DEG "\xC2\xB0"  // UTF-8 degree sign

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// Fonts
// ---------------------------------------------------------------------------
#define F_TIME u8g2_font_logisoso42_tn
#define F_DATE u8g2_font_logisoso26_tr  // letters too, for "4 Oct 2026"; the digits are the same as _tn's
#define F_BIG u8g2_font_logisoso24_tr
#define F_H1 u8g2_font_helvB14_tf
#define F_H2 u8g2_font_helvB12_tf
#define F_BOLD u8g2_font_helvB10_tf
#define F_BODY u8g2_font_helvR10_tf
#define F_SMALL u8g2_font_helvR08_tf
#define F_SMALLB u8g2_font_helvB08_tf
#define F_MONO u8g2_font_profont12_tf

namespace {

// ---------------------------------------------------------------------------
// Layout (all coordinates are logical pixels, origin top-left)
// ---------------------------------------------------------------------------
const int kBarH = 20;        // status bar; separator on its last row
const int kClockCx = 92;
const int kClockCy = 108;
const int kClockR = 80;
const int kRightX0 = 186;    // right-hand column of the dashboard
const int kRightX1 = 396;
const int kWxTop = 199;      // weather band top (separator row)
const int kWxBottom = 255;
const int kBottomTop = 256;  // spotify / extras strip (to the bottom edge)
const int kColA = 176;       // weather column boundaries: today, then two days of 112 px
const int kColB = 288;

// ---------------------------------------------------------------------------
// Tiny helpers
// ---------------------------------------------------------------------------
inline int iround(float v) { return (int)lroundf(v); }
inline void ink(u8g2_t *u) { u8g2_SetDrawColor(u, 1); }
inline void paper(u8g2_t *u) { u8g2_SetDrawColor(u, 0); }

inline int tw(u8g2_t *u, const char *s) { return (int)u8g2_GetUTF8Width(u, s); }
inline void txt(u8g2_t *u, int x, int y, const char *s) { u8g2_DrawUTF8(u, x, y, s); }
inline void txtR(u8g2_t *u, int xr, int y, const char *s) { txt(u, xr - tw(u, s), y, s); }
inline void txtC(u8g2_t *u, int xc, int y, const char *s) { txt(u, xc - tw(u, s) / 2, y, s); }

inline int tempOf(float c, bool fahrenheit) {
  return iround(fahrenheit ? c * 9.0f / 5.0f + 32.0f : c);
}

// ---------------------------------------------------------------------------
// Numbers that change must not wobble.  U8g2 measures a multi-glyph string by its
// *ink* ("balanced" width: the last glyph's ink width plus the first glyph's left
// bearing), so centring or right-aligning on that moves the whole string by a few
// pixels depending on which digits it starts and ends with (a "1" is narrower than
// a "0").  These helpers lay the glyphs out on their advances instead, giving every
// digit the cell of the widest digit, so a given string length always occupies the
// same pixels.  ASCII only: digits, ':' '-' '.' '/' ' ' and the like.
// ---------------------------------------------------------------------------
int digitPitch(u8g2_t *u) {
  int w = 0;
  for (int c = '0'; c <= '9'; c++) {
    int a = u8g2_GetGlyphWidth(u, (uint16_t)c);
    if (a > w) w = a;
  }
  return w;
}

int pitchWidth(u8g2_t *u, const char *s) {
  const int pitch = digitPitch(u);
  int w = 0;
  for (; *s; ++s) {
    unsigned char c = (unsigned char)*s;
    w += (c >= '0' && c <= '9') ? pitch : u8g2_GetGlyphWidth(u, c);
  }
  return w;
}

void txtPitch(u8g2_t *u, int x, int y, const char *s) {
  const int pitch = digitPitch(u);
  for (; *s; ++s) {
    unsigned char c = (unsigned char)*s;
    int adv = u8g2_GetGlyphWidth(u, c);
    if (c >= '0' && c <= '9') {
      u8g2_DrawGlyph(u, x + (pitch - adv) / 2, y, c);  // narrow digits sit centred in the cell
      x += pitch;
    } else {
      u8g2_DrawGlyph(u, x, y, c);
      x += adv;
    }
  }
}
inline void txtPitchR(u8g2_t *u, int xr, int y, const char *s) { txtPitch(u, xr - pitchWidth(u, s), y, s); }
inline void txtPitchC(u8g2_t *u, int xc, int y, const char *s) { txtPitch(u, xc - pitchWidth(u, s) / 2, y, s); }

// Draws a degree sign as a small ring; the numeric fonts have no glyph for it.
void degreeRing(u8g2_t *u, int x, int y, int r) {
  u8g2_DrawCircle(u, x, y, r, U8G2_DRAW_ALL);
  if (r >= 3) u8g2_DrawCircle(u, x, y, r - 1, U8G2_DRAW_ALL);
}

// ---------------------------------------------------------------------------
// UTF-8 handling.  Track names are arbitrary Unicode; the bundled fonts cover
// Latin-1.  Common typographic punctuation is mapped to ASCII, anything else
// the font lacks becomes '?'.
// ---------------------------------------------------------------------------
int utf8Decode(const unsigned char *s, uint32_t *cp) {
  if (s[0] < 0x80) {
    *cp = s[0];
    return 1;
  }
  if ((s[0] & 0xE0) == 0xC0 && (s[1] & 0xC0) == 0x80) {
    *cp = ((uint32_t)(s[0] & 0x1F) << 6) | (s[1] & 0x3F);
    return 2;
  }
  if ((s[0] & 0xF0) == 0xE0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80) {
    *cp = ((uint32_t)(s[0] & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) | (s[2] & 0x3F);
    return 3;
  }
  if ((s[0] & 0xF8) == 0xF0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80 &&
      (s[3] & 0xC0) == 0x80) {
    *cp = ((uint32_t)(s[0] & 0x07) << 18) | ((uint32_t)(s[1] & 0x3F) << 12) |
          ((uint32_t)(s[2] & 0x3F) << 6) | (s[3] & 0x3F);
    return 4;
  }
  *cp = 0xFFFD;  // invalid byte
  return 1;
}

// Copies src to dst (always valid UTF-8, NUL terminated) using only glyphs the
// currently selected font can draw.
void sanitizeForFont(u8g2_t *u, char *dst, size_t cap, const char *src) {
  size_t n = 0;
  bool lastUnknown = false;
  auto put = [&](const char *s) {
    size_t l = strlen(s);
    if (n + l + 1 > cap) return false;
    memcpy(dst + n, s, l);
    n += l;
    return true;
  };
  const unsigned char *p = (const unsigned char *)src;
  while (*p) {
    uint32_t cp;
    int len = utf8Decode(p, &cp);
    const char *mapped = nullptr;
    switch (cp) {
      case 0x2018: case 0x2019: case 0x201A: case 0x201B: case 0x2032: mapped = "'"; break;
      case 0x201C: case 0x201D: case 0x201E: case 0x2033: mapped = "\""; break;
      case 0x2010: case 0x2011: case 0x2012: case 0x2013: case 0x2014: case 0x2015:
      case 0x2212: mapped = "-"; break;
      case 0x2026: mapped = "..."; break;
      case 0x2022: case 0x00B7: case 0x2219: mapped = "-"; break;
      case 0x00A0: case 0x2002: case 0x2003: case 0x2009: mapped = " "; break;
      default: break;
    }
    if (cp < 0x20 || cp == 0x7F) mapped = " ";
    if (mapped) {
      if (!put(mapped)) break;
      lastUnknown = false;
    } else if (cp < 0x10000 && u8g2_IsGlyph(u, (uint16_t)cp)) {
      if (n + len + 1 > cap) break;
      memcpy(dst + n, p, len);
      n += len;
      lastUnknown = false;
    } else if (!lastUnknown) {
      if (!put("?")) break;
      lastUnknown = true;
    }
    p += len;
  }
  dst[n] = 0;
}

// Draws text clipped to maxW pixels, ending with "..." when it had to be cut.
// Returns the drawn width.
int txtFit(u8g2_t *u, int x, int y, int maxW, const char *src) {
  char buf[160];
  sanitizeForFont(u, buf, sizeof buf, src);
  int w = tw(u, buf);
  if (w <= maxW) {
    txt(u, x, y, buf);
    return w;
  }
  size_t n = strlen(buf);
  while (n > 0) {
    --n;
    while (n > 0 && (buf[n] & 0xC0) == 0x80) --n;  // stay on a code point boundary
    buf[n] = 0;
    char tmp[170];
    snprintf(tmp, sizeof tmp, "%s...", buf);
    int tmpW = tw(u, tmp);
    if (tmpW <= maxW) {
      txt(u, x, y, tmp);
      return tmpW;
    }
  }
  return 0;
}

// Greedy word wrap into at most maxLines lines of at most maxW pixels; the last
// line is ellipsised if text remains.  Returns the number of lines produced.
int wrapText(u8g2_t *u, const char *src, int maxW, char lines[][96], int maxLines) {
  char clean[200];
  sanitizeForFont(u, clean, sizeof clean, src);
  int count = 0;
  const char *p = clean;
  while (*p && count < maxLines) {
    while (*p == ' ') ++p;
    if (!*p) break;
    char line[96] = "";
    size_t lineLen = 0;
    const char *lastBreak = nullptr;  // pointer just after the last space that fits
    size_t lastBreakLen = 0;
    const char *q = p;
    while (*q) {
      // advance one code point
      uint32_t cp;
      int cl = utf8Decode((const unsigned char *)q, &cp);
      if (lineLen + cl + 1 > sizeof line) break;
      memcpy(line + lineLen, q, cl);
      line[lineLen + cl] = 0;
      if (tw(u, line) > maxW) {
        line[lineLen] = 0;
        break;
      }
      lineLen += cl;
      q += cl;
      if (cp == ' ') {
        lastBreak = q;
        lastBreakLen = lineLen;
      }
    }
    if (*q && lastBreak) {  // wrapped mid-text: back up to the last space
      lineLen = lastBreakLen;
      q = lastBreak;
    }
    line[lineLen] = 0;
    while (lineLen > 0 && line[lineLen - 1] == ' ') line[--lineLen] = 0;
    if (lineLen == 0) break;  // a single glyph wider than maxW; give up
    p = q;
    while (*p == ' ') ++p;
    if (count == maxLines - 1 && *p) {  // out of lines: ellipsise
      char tmp[96];
      snprintf(tmp, sizeof tmp, "%.90s", line);  // leave room for "..."
      size_t n = strlen(tmp);
      if (strlen(line) > n) {  // truncated: don't leave half a UTF-8 sequence behind
        while (n > 0 && (line[n] & 0xC0) == 0x80) --n;
        tmp[n] = 0;
      }
      for (;;) {
        char with[104];
        snprintf(with, sizeof with, "%s...", tmp);
        if (tw(u, with) <= maxW || n == 0) {
          snprintf(lines[count], 96, "%.95s", with);
          break;
        }
        --n;
        while (n > 0 && (tmp[n] & 0xC0) == 0x80) --n;
        tmp[n] = 0;
      }
    } else {
      snprintf(lines[count], 96, "%.95s", line);
    }
    ++count;
  }
  return count;
}

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------
void disc(u8g2_t *u, float cx, float cy, float r) {
  u8g2_DrawDisc(u, (int16_t)iround(cx), (int16_t)iround(cy), (uint16_t)iround(r), U8G2_DRAW_ALL);
}

// A line segment with a width tapering from w0 (at x0,y0) to w1 (at x1,y1),
// drawn as two filled triangles.  Widths <= 1 fall back to a hairline.
void taper(u8g2_t *u, float x0, float y0, float x1, float y1, float w0, float w1) {
  float dx = x1 - x0, dy = y1 - y0;
  float len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) return;
  if (w0 <= 1.2f && w1 <= 1.2f) {
    u8g2_DrawLine(u, iround(x0), iround(y0), iround(x1), iround(y1));
    return;
  }
  float nx = -dy / len, ny = dx / len;
  float ax = x0 + nx * w0 / 2, ay = y0 + ny * w0 / 2;
  float bx = x0 - nx * w0 / 2, by = y0 - ny * w0 / 2;
  float cx = x1 - nx * w1 / 2, cy = y1 - ny * w1 / 2;
  float ex = x1 + nx * w1 / 2, ey = y1 + ny * w1 / 2;
  u8g2_DrawTriangle(u, iround(ax), iround(ay), iround(bx), iround(by), iround(cx), iround(cy));
  u8g2_DrawTriangle(u, iround(ax), iround(ay), iround(cx), iround(cy), iround(ex), iround(ey));
}

// Media glyphs, drawn inside an s x s box whose top-left is (x, y).
void iconPlay(u8g2_t *u, int x, int y, int s) {
  u8g2_DrawTriangle(u, x + s / 6, y, x + s / 6, y + s - 1, x + s - 1, y + s / 2);
}
void iconPause(u8g2_t *u, int x, int y, int s) {
  int bw = (s + 2) / 4;
  if (bw < 2) bw = 2;
  int gap = s / 5;
  if (gap < 2) gap = 2;
  int x0 = x + (s - (2 * bw + gap)) / 2;
  u8g2_DrawBox(u, x0, y, bw, s);
  u8g2_DrawBox(u, x0 + bw + gap, y, bw, s);
}
void iconNext(u8g2_t *u, int x, int y, int s) {
  int bw = s / 5 > 1 ? s / 5 : 1;
  u8g2_DrawTriangle(u, x, y, x, y + s - 1, x + s - bw - 2, y + s / 2);
  u8g2_DrawBox(u, x + s - bw, y, bw, s);
}
void iconPrev(u8g2_t *u, int x, int y, int s) {
  int bw = s / 5 > 1 ? s / 5 : 1;
  u8g2_DrawBox(u, x, y, bw, s);
  u8g2_DrawTriangle(u, x + s - 1, y, x + s - 1, y + s - 1, x + bw + 2, y + s / 2);
}
void iconNote(u8g2_t *u, int x, int y, int s) {  // musical note
  int r = (s + 3) / 6;  // note head radius
  if (r < 2) r = 2;
  int stemX = x + 2 * r - 1;
  u8g2_DrawDisc(u, x + r, y + s - r - 1, r, U8G2_DRAW_ALL);
  u8g2_DrawBox(u, stemX, y, 2, s - r);
  taper(u, stemX + 2, y, x + s - 1, y + s / 2, 3, 2);  // flag
}

// Chance of rain: a drop, outlined like the moon.  kRainW x kRainH with its top-left at (x, y); its
// bottom row is the bottom row of the digits beside it.  (Three slanted strokes, "///", read as text.)
const int kRainW = 7, kRainH = 9;
void iconRain(u8g2_t *u, int x, int y) {
  static const char *const kDrop[kRainH] = {
      "...#...",  //
      "..#.#..",  //
      "..#.#..",  //
      ".#...#.",  //
      ".#...#.",  //
      "#.....#",  //
      "#.....#",  //
      ".#...#.",  //
      "..###..",  //
  };
  for (int r = 0; r < kRainH; r++)
    for (int c = 0; c < kRainW; c++)
      if (kDrop[r][c] == '#') u8g2_DrawPixel(u, x + c, y + r);
}

// The moon as a disc of `d` pixels: an outline, with the lit part filled (moon.h decides which
// pixels).  `litPct` 0..100; a growing moon is lit on the right in the northern hemisphere and on
// the left in the southern.
const int kMoonD = 10;
// Where the moon and the drop share a column (the forecast days, the legend) the drop is centred in a
// cell as wide as the moon, so the two percentages start at the same x.
const int kRainInset = (kMoonD - kRainW + 1) / 2;
void iconMoon(u8g2_t *u, int x, int y, int d, int litPct, bool waxing, bool southern) {
  const float lit = (litPct < 0 ? 0 : (litPct > 100 ? 100 : litPct)) / 100.0f;
  const bool onRight = waxing != southern;
  auto inside = [&](int c, int r) { return moon::discPixel(c, r, d, lit, onRight).inside; };
  for (int r = 0; r < d; r++)
    for (int c = 0; c < d; c++) {
      const moon::DiscPixel p = moon::discPixel(c, r, d, lit, onRight);
      if (!p.inside) continue;
      const bool rim = !(inside(c - 1, r) && inside(c + 1, r) && inside(c, r - 1) && inside(c, r + 1));
      if (p.lit || rim) u8g2_DrawPixel(u, x + c, y + r);
    }
}

// ---------------------------------------------------------------------------
// Weather icons.  Drawn in a 32-unit design box and scaled to `size` pixels.
// ---------------------------------------------------------------------------
struct Box {
  float x, y, k;  // top-left and scale
  float X(float v) const { return x + v * k; }
  float Y(float v) const { return y + v * k; }
};

void sunShape(u8g2_t *u, const Box &b, float cx, float cy, float r, bool rays) {
  disc(u, b.X(cx), b.Y(cy), r * b.k);
  if (!rays) return;
  for (int i = 0; i < 8; i++) {
    float a = i * (float)M_PI / 4.0f;
    float s = sinf(a), c = cosf(a);
    taper(u, b.X(cx + s * (r + 3)), b.Y(cy - c * (r + 3)), b.X(cx + s * (r + 6.5f)),
          b.Y(cy - c * (r + 6.5f)), 2.2f * b.k, 2.2f * b.k);
  }
}

void moonShape(u8g2_t *u, const Box &b, float cx, float cy, float r) {
  disc(u, b.X(cx), b.Y(cy), r * b.k);
  paper(u);
  disc(u, b.X(cx + r * 0.55f), b.Y(cy - r * 0.35f), r * 0.85f * b.k);
  ink(u);
}

// Cloud occupying roughly x 3..30, y 7..27 of the design box, shifted by (dx, dy).
void cloudShape(u8g2_t *u, const Box &b, float dx, float dy, bool halo) {
  auto draw = [&](float grow) {
    disc(u, b.X(9 + dx), b.Y(21 + dy), (6 + grow) * b.k);
    disc(u, b.X(24 + dx), b.Y(21 + dy), (6 + grow) * b.k);
    disc(u, b.X(16.5f + dx), b.Y(15 + dy), (8 + grow) * b.k);
    float x0 = b.X(9 + dx), x1 = b.X(24 + dx);
    float y0 = b.Y(15 + dy - grow), y1 = b.Y(27 + dy + grow);
    u8g2_DrawBox(u, iround(x0), iround(y0), iround(x1 - x0) + 1, iround(y1 - y0) + 1);
  };
  if (halo) {
    paper(u);
    draw(2.0f);
    ink(u);
  }
  draw(0.0f);
}

void rainStreaks(u8g2_t *u, const Box &b, float y0, int n, float w) {
  for (int i = 0; i < n; i++) {
    float x = 10 + i * (n == 2 ? 10 : 6.5f);
    taper(u, b.X(x + 2), b.Y(y0), b.X(x - 1), b.Y(y0 + 5.5f), w * b.k, w * b.k);
  }
}

void snowFlakes(u8g2_t *u, const Box &b, float y0) {
  for (int i = 0; i < 3; i++) {
    float cx = 9.5f + i * 7;
    float cy = y0 + (i == 1 ? 3.5f : 0.5f);
    u8g2_DrawPixel(u, iround(b.X(cx)), iround(b.Y(cy)));
    u8g2_DrawBox(u, iround(b.X(cx - 1)), iround(b.Y(cy - 1)), iround(2.2f * b.k) + 1, iround(2.2f * b.k) + 1);
  }
}

void boltShape(u8g2_t *u, const Box &b) {
  // zig-zag lightning bolt below the cloud
  u8g2_DrawTriangle(u, iround(b.X(18)), iround(b.Y(21.5f)), iround(b.X(11)), iround(b.Y(29)),
                    iround(b.X(17)), iround(b.Y(29)));
  u8g2_DrawTriangle(u, iround(b.X(15)), iround(b.Y(26)), iround(b.X(21.5f)), iround(b.Y(26)),
                    iround(b.X(14)), iround(b.Y(32)));
}

void drawWxIcon(u8g2_t *u, WxIcon icon, bool day, int x, int y, int size) {
  Box b = {(float)x, (float)y, size / 32.0f};
  ink(u);
  switch (icon) {
    case WX_CLEAR:
      if (day) {
        sunShape(u, b, 16, 16, 6.5f, true);
      } else {
        moonShape(u, b, 15, 16, 10);
      }
      break;
    case WX_PARTLY:
      if (day) {
        sunShape(u, b, 11, 11, 5, true);
      } else {
        moonShape(u, b, 10, 11, 7);
      }
      cloudShape(u, b, 2, 3, true);
      break;
    case WX_CLOUDY:
      cloudShape(u, b, 0, 1, false);
      break;
    case WX_FOG:
      cloudShape(u, b, 0, -5, false);
      for (int i = 0; i < 3; i++) {
        float yy = 23 + i * 3.6f;
        u8g2_DrawBox(u, iround(b.X(4 + (i % 2) * 3)), iround(b.Y(yy)), iround(24 * b.k), iround(1.6f * b.k) > 0 ? iround(1.6f * b.k) : 1);
      }
      break;
    case WX_DRIZZLE:
      cloudShape(u, b, 0, -4, false);
      rainStreaks(u, b, 24, 3, 1.0f);
      break;
    case WX_RAIN:
      cloudShape(u, b, 0, -4, false);
      rainStreaks(u, b, 24, 3, 1.8f);
      break;
    case WX_SNOW:
      cloudShape(u, b, 0, -4, false);
      snowFlakes(u, b, 26);
      break;
    case WX_THUNDER:
      cloudShape(u, b, 0, -4, false);
      boltShape(u, b);
      break;
  }
  ink(u);
}

// ---------------------------------------------------------------------------
// Status bar widgets
// ---------------------------------------------------------------------------
int wifiLevel(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

void drawWifi(u8g2_t *u, int x, int yb, bool up, int rssi) {
  int level = up ? wifiLevel(rssi) : 0;
  for (int i = 0; i < 4; i++) {
    int h = 3 + i * 3;
    int bx = x + i * 4;
    if (i < level) {
      u8g2_DrawBox(u, bx, yb - h + 1, 3, h);
    } else {
      u8g2_DrawHLine(u, bx, yb, 3);
    }
  }
  if (!up) {  // cross
    u8g2_DrawLine(u, x + 3, yb - 9, x + 12, yb - 1);
    u8g2_DrawLine(u, x + 3, yb - 8, x + 11, yb - 1);
    u8g2_DrawLine(u, x + 12, yb - 9, x + 3, yb - 1);
    u8g2_DrawLine(u, x + 11, yb - 9, x + 3, yb - 2);
  }
}

// What the battery is doing, as pixel art ('#' = ink) drawn inside the battery body: a bolt while
// charging, a tick when it is full, nothing when it runs on the battery or the clock does not know
// (the percentage and the fill say the rest).  The bolt sits in a window cleared in the fill, so it is
// crisp at every level; the tick is cut out of the solid fill of a full battery.
const char *const kArtBolt[] = {  // charging, 6 x 8
    "....##", "...##.", "..##..", ".#####", "#####.", "..##..", ".##...", "##....",
};
const char *const kArtTick[] = {  // full, 7 x 6
    ".....##", "....##.", "#..##..", "##.##..", ".###...", "..#....",
};

// Picks the art for a UiCharge value; returns its height in rows (0 = none).
int chargeArt(int state, const char *const **rows) {
  switch (state) {
    case CHARGE_CHARGING: *rows = kArtBolt; return (int)(sizeof kArtBolt / sizeof *kArtBolt);
    case CHARGE_FULL: *rows = kArtTick; return (int)(sizeof kArtTick / sizeof *kArtTick);
    default: *rows = nullptr; return 0;
  }
}

// Where the pieces of the battery gauge go.  The percentage sits in a cell as wide as "100%"
// so the gauge does not shift when the number gains or loses a digit.
struct BatteryLayout {
  char label[8];
  int width;
};
const int kBatBodyW = 24, kBatBodyH = 12, kBatNub = 2, kBatGap = 4;

// The art in the middle of the body at (bx, by).  `window`: first clear the art's columns, a pixel wider on
// each side and as tall as the inside of the frame, in `bg`, then draw the art in `fg`.  Otherwise the art is
// cut out of whatever is there, in `bg`.
void drawBatteryArt(u8g2_t *u, int bx, int by, const char *const *rows, int h, int fg, int bg, bool window) {
  const int w = (int)strlen(rows[0]);
  const int ax = bx + 2 + (kBatBodyW - 4 - w) / 2, ay = by + 2 + (kBatBodyH - 4 - h) / 2;
  if (window) {
    u8g2_SetDrawColor(u, bg);
    u8g2_DrawBox(u, ax - 1, by + 1, w + 2, kBatBodyH - 2);
  }
  u8g2_SetDrawColor(u, window ? fg : bg);
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++)
      if (rows[r][c] == '#') u8g2_DrawPixel(u, ax + c, ay + r);
}

BatteryLayout batteryLayout(u8g2_t *u, const UiModel &m) {
  BatteryLayout b;
  if (m.batPresent) {
    snprintf(b.label, sizeof b.label, "%d%%", m.batPercent);
  } else {
    snprintf(b.label, sizeof b.label, "USB");
  }
  u8g2_SetFont(u, F_BOLD);
  const int labelW = m.batPresent ? pitchWidth(u, "100%") : tw(u, b.label);
  b.width = (m.batPresent ? kBatBodyW + kBatNub + kBatGap : 0) + labelW;
  return b;
}

// Right-aligned battery gauge ending at xr, `dy` rows below where it sits in the status bar (0
// there; the legend page draws samples further down).  Inverts (black plate, white artwork)
// during the "on" phase of the blink when the level is low.
void drawBatteryAt(u8g2_t *u, int xr, int dy, const UiModel &m) {
  const BatteryLayout b = batteryLayout(u, m);  // leaves F_BOLD selected
  const int x0 = xr - b.width;
  bool flash = m.batPresent && m.batLow && m.batBlinkOn;

  if (flash) {
    ink(u);
    u8g2_DrawRBox(u, x0 - 4, 1 + dy, b.width + 8, kBarH - 3, 3);
    paper(u);
  } else {
    ink(u);
  }
  int ty = 15 + dy;
  if (m.batPresent) {
    const int bx = x0, by = 4 + dy;
    u8g2_DrawFrame(u, bx, by, kBatBodyW, kBatBodyH);
    u8g2_DrawBox(u, bx + kBatBodyW, by + 3, kBatNub, kBatBodyH - 6);
    int inner = kBatBodyW - 4;
    int fill = (inner * m.batPercent + 50) / 100;
    if (m.batPercent > 0 && fill < 1) fill = 1;
    if (fill > inner) fill = inner;
    u8g2_DrawBox(u, bx + 2, by + 2, fill, kBatBodyH - 4);
    const char *const *rows;
    const int h = chargeArt(m.batCharge, &rows);
    if (h > 0) drawBatteryArt(u, bx, by, rows, h, flash ? 0 : 1, flash ? 1 : 0, m.batCharge == CHARGE_CHARGING);  // (the plate is black while it blinks)
    if (flash) paper(u); else ink(u);
    txtPitch(u, bx + kBatBodyW + kBatNub + kBatGap, ty, b.label);
  } else {
    txt(u, x0, ty, b.label);
  }
  ink(u);
}

void drawBattery(u8g2_t *u, int xr, const UiModel &m) { drawBatteryAt(u, xr, 0, m); }

void drawStatusClock(u8g2_t *u, int cx, int y, const UiModel &m);  // below, with the time drawing

void drawStatusBar(u8g2_t *u, const UiModel &m) {
  ink(u);
  drawWifi(u, 4, 16, m.wifiUp, m.rssi);

  const int messageEnd = 396 - batteryLayout(u, m).width - 8;  // the status message stops short of the gauge
  u8g2_SetFont(u, F_BODY);
  int leftEnd = 24;
  if (m.location[0]) {
    leftEnd += txtFit(u, 24, 15, 120, m.location);
  }

  // centre: clock on the secondary pages, otherwise the latest status message
  if (m.page != PAGE_DASHBOARD && m.timeValid) {
    drawStatusClock(u, 200, 15, m);
  } else if (m.status[0]) {
    u8g2_SetFont(u, F_SMALL);
    int avail = messageEnd - (leftEnd + 10);
    int w = tw(u, m.status);
    int x = leftEnd + 10;
    if (w < avail) x += (avail - w) / 2;
    txtFit(u, x, 14, avail, m.status);
  }

  drawBattery(u, 396, m);
  u8g2_DrawHLine(u, 0, kBarH - 1, UI_WIDTH);
}

// ---------------------------------------------------------------------------
// Analog clock
// ---------------------------------------------------------------------------
// A ring two pixels wide whose outside is the circle of radius r: every pixel whose middle lies between r - 1.5
// and r + 0.5 from the centre.  (Two circles drawn one inside the other, r and r - 1, cover the same band but
// for single pixels here and there, where the two happen to round differently: on the panel they showed as
// holes in the clock's outline.)
void ring(u8g2_t *u, int cx, int cy, int r) {
  const float outer2 = ((float)r + 0.5f) * ((float)r + 0.5f), inner2 = ((float)r - 1.5f) * ((float)r - 1.5f);
  for (int dy = -r; dy <= r; dy++) {
    const float d2 = (float)(dy * dy);
    const int xo = (int)floorf(sqrtf(outer2 - d2));  // the last column of this row that is inside the outer edge
    if (d2 >= inner2) {                               // above or below the hole: one run from side to side
      u8g2_DrawHLine(u, cx - xo, cy + dy, 2 * xo + 1);
      continue;
    }
    const int xi = (int)ceilf(sqrtf(inner2 - d2));  // the first column that is outside the inner edge
    if (xi > xo) continue;
    u8g2_DrawHLine(u, cx - xo, cy + dy, xo - xi + 1);
    u8g2_DrawHLine(u, cx + xi, cy + dy, xo - xi + 1);
  }
}

void drawAnalogClock(u8g2_t *u, int cx, int cy, int R, const UiModel &m) {
  ink(u);
  ring(u, cx, cy, R);

  // minute / hour ticks
  for (int i = 0; i < 60; i++) {
    float a = i * 6.0f * (float)M_PI / 180.0f;
    float s = sinf(a), c = -cosf(a);
    if (i % 5 == 0) {
      taper(u, cx + s * (R - 10), cy + c * (R - 10), cx + s * (R - 3), cy + c * (R - 3), 3, 3);
    } else {
      u8g2_DrawPixel(u, iround(cx + s * (R - 4)), iround(cy + c * (R - 4)));
      u8g2_DrawPixel(u, iround(cx + s * (R - 5)), iround(cy + c * (R - 5)));
    }
  }

  // numerals
  u8g2_SetFont(u, F_BOLD);
  for (int h = 1; h <= 12; h++) {
    float a = h * 30.0f * (float)M_PI / 180.0f;
    char n[12];
    snprintf(n, sizeof n, "%d", h);
    int x = iround(cx + sinf(a) * (R - 22));
    int y = iround(cy - cosf(a) * (R - 22));
    txtC(u, x, y + 4, n);
  }

  if (!m.timeValid) {
    u8g2_SetFont(u, F_SMALL);
    txtC(u, cx, cy + 28, "waiting for time");
    disc(u, cx, cy, 4);
    return;
  }

  float sec = m.local.tm_sec + m.secFrac;
  float min = m.local.tm_min + sec / 60.0f;
  float hr = (m.local.tm_hour % 12) + min / 60.0f;
  float aH = hr * 30.0f * (float)M_PI / 180.0f;
  float aM = min * 6.0f * (float)M_PI / 180.0f;
  float aS = sec * 6.0f * (float)M_PI / 180.0f;

  auto hand = [&](float ang, float len, float tail, float wBase, float wTip) {
    float s = sinf(ang), c = -cosf(ang);
    float x0 = cx - s * tail, y0 = cy - c * tail;
    float x1 = cx + s * len, y1 = cy + c * len;
    paper(u);  // thin paper-coloured halo so hands stay readable over ticks/numerals
    taper(u, x0, y0, x1, y1, wBase + 3, wTip + 3);
    ink(u);
    taper(u, x0, y0, x1, y1, wBase, wTip);
  };
  hand(aH, R * 0.50f, 7, 7, 3);
  hand(aM, R * 0.78f, 9, 5, 2);

  // second hand: slim wedge with a short counterweight tail and a ring hub
  float s = sinf(aS), c = -cosf(aS);
  ink(u);
  taper(u, cx - s * 6, cy - c * 6, cx + s * (R * 0.90f), cy + c * (R * 0.90f), 2.6f, 1.3f);
  taper(u, cx - s * 18, cy - c * 18, cx - s * 6, cy - c * 6, 3.2f, 3.2f);
  disc(u, cx, cy, 6);
  paper(u);
  disc(u, cx, cy, 2);
  ink(u);
}

// The time on a 12 hour clock: HH:MM big, with the seconds and AM / PM small to its right.
//
//    9:45  07        the seconds sit on the baseline of the big digits,
//          PM        AM / PM level with their top.
//
// A one digit hour leaves its cell empty rather than shifting everything, so the picture only
// changes where the digits do.
void drawTime12(u8g2_t *u, int cx, int yTime, const struct tm &t) {
  bool pm;
  const int h = datefmt::hour12(t.tm_hour, &pm);
  u8g2_SetFont(u, F_TIME);
  const int pitch = digitPitch(u);
  const int bigW = 4 * pitch + u8g2_GetGlyphWidth(u, ':');
  const int top = yTime - (int)u8g2_GetAscent(u);
  u8g2_SetFont(u, F_BIG);
  const int secW = 2 * digitPitch(u);
  const int gap = 8;
  const int x0 = cx - (bigW + gap + secW) / 2;

  char big[8];
  snprintf(big, sizeof big, "%d:%02d", h, t.tm_min);
  u8g2_SetFont(u, F_TIME);
  txtPitch(u, x0 + (h < 10 ? pitch : 0), yTime, big);

  char sec[8];
  snprintf(sec, sizeof sec, "%02d", t.tm_sec);
  u8g2_SetFont(u, F_BIG);
  const int sx = x0 + bigW + gap;
  txtPitch(u, sx, yTime, sec);

  u8g2_SetFont(u, F_BOLD);
  txt(u, sx + 1, top + (int)u8g2_GetAscent(u), datefmt::ampm(pm));
}

// The clock in the status bar of the other pages: 08:21:07, or 8:21:07 PM, on fixed digit cells.
void drawStatusClock(u8g2_t *u, int cx, int y, const UiModel &m) {
  u8g2_SetFont(u, F_BOLD);
  char t[16];
  if (!m.time12h) {
    snprintf(t, sizeof t, "%02d:%02d:%02d", m.local.tm_hour, m.local.tm_min, m.local.tm_sec);
    txtPitchC(u, cx, y, t);
    return;
  }
  bool pm;
  const int h = datefmt::hour12(m.local.tm_hour, &pm);
  snprintf(t, sizeof t, "%d:%02d:%02d %s", h, m.local.tm_min, m.local.tm_sec, datefmt::ampm(pm));
  const int width = pitchWidth(u, "00:00:00 PM");
  txtPitch(u, cx - width / 2 + (h < 10 ? digitPitch(u) : 0), y, t);
}

// ---------------------------------------------------------------------------
// Dashboard blocks
// ---------------------------------------------------------------------------
void drawDateTime(u8g2_t *u, const UiModel &m) {
  const int cx = (kRightX0 + kRightX1) / 2;
  // Baselines chosen so the glyph rows leave clear gaps (measured on the host
  // renderer): bar->date 5 px, date->time 6, time->weekday 6, weekday->zone 4.
  const int yDate = 51, yTime = 99, yWeek = 116, yZone = 134;
  char buf[48];
  if (!m.timeValid) {
    u8g2_SetFont(u, F_DATE);
    txtC(u, cx, yDate, "----------");
    u8g2_SetFont(u, F_TIME);
    txtC(u, cx, yTime, "--:--:--");
    u8g2_SetFont(u, F_BODY);
    txtC(u, cx, yWeek + 7, m.wifiOff ? "Time not set: turn WiFi on once" : "Waiting for network time");
    return;
  }
  const struct tm &t = m.local;
  datefmt::formatDate(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, m.dateFormat, buf, sizeof buf);
  u8g2_SetFont(u, F_DATE);
  txtPitchC(u, cx, yDate, buf);

  if (m.time12h) {
    drawTime12(u, cx, yTime, t);
  } else {
    snprintf(buf, sizeof buf, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    u8g2_SetFont(u, F_TIME);
    txtPitchC(u, cx, yTime, buf);
  }

  // weekday, and with "show_week" the ISO week number and day of year
  if (m.showWeek) {
    int week = timeutil::isoWeek(t.tm_year + 1900, t.tm_yday, t.tm_wday);
    snprintf(buf, sizeof buf, "%s \xC2\xB7 W%02d \xC2\xB7 day %d", timeutil::weekdayName(t.tm_wday), week, t.tm_yday + 1);
  } else {
    snprintf(buf, sizeof buf, "%s", timeutil::weekdayName(t.tm_wday));
  }
  u8g2_SetFont(u, F_BOLD);
  txtC(u, cx, yWeek, buf);

  // time zone
  char off[16];
  timeutil::formatUtcOffset(m.utcOffsetMin, off, sizeof off);
  if (m.tzAbbrev[0]) {
    snprintf(buf, sizeof buf, "%s  UTC%s", m.tzAbbrev, off);
  } else {
    snprintf(buf, sizeof buf, "UTC%s", off);
  }
  u8g2_SetFont(u, F_BODY);
  txtC(u, cx, yZone, buf);
}

void drawIndoor(u8g2_t *u, const UiModel &m) {
  const int x0 = kRightX0, x1 = kRightX1, y0 = 144, y1 = 194;
  u8g2_DrawRFrame(u, x0, y0, x1 - x0, y1 - y0, 4);
  u8g2_SetFont(u, F_SMALLB);
  // Label plate: 2 px of black above and below the capitals (the glyph rows run from
  // baseline - cap to baseline - 1), straddling the frame line.
  const int cap = u8g2_GetAscent(u), plateTop = y0 - 6;
  u8g2_DrawBox(u, x0 + 8, plateTop, tw(u, "INDOOR") + 8, cap + 4);
  paper(u);
  txt(u, x0 + 12, plateTop + 2 + cap, "INDOOR");
  ink(u);

  if (!m.indoorValid) {
    u8g2_SetFont(u, F_BODY);
    txtC(u, (x0 + x1) / 2, y0 + 30, "sensor unavailable");
    return;
  }
  char num[16];
  int base = y0 + 37;  // digits centred in the (now 50 px) box: 12 px above and below
  float t = m.useFahrenheit ? m.indoorC * 9.0f / 5.0f + 32.0f : m.indoorC;
  snprintf(num, sizeof num, "%.1f", t);
  u8g2_SetFont(u, F_BIG);
  const char unit = m.useFahrenheit ? 'F' : 'C';
  int w1 = pitchWidth(u, num);
  int w2 = u8g2_GetGlyphWidth(u, (uint16_t)unit);
  const int unitGap = 12, sectionGap = 22;
  char hum[8];
  snprintf(hum, sizeof hum, "%d", iround(m.indoorRh));
  int w3 = pitchWidth(u, hum);
  int w4 = u8g2_GetGlyphWidth(u, (uint16_t)'%');
  int total = w1 + unitGap + w2 + sectionGap + w3 + w4;
  int x = (x0 + x1) / 2 - total / 2;
  txtPitch(u, x, base, num);
  degreeRing(u, x + w1 + 6, base - 18, 3);
  u8g2_DrawGlyph(u, x + w1 + unitGap, base, (uint16_t)unit);
  x += w1 + unitGap + w2 + sectionGap;
  txtPitch(u, x, base, hum);
  u8g2_DrawGlyph(u, x + w3, base, (uint16_t)'%');
}

// Blank columns before the ink of a glyph, measured on the rendered fonts (the preview checks them),
// so that things can be lined up by their ink rather than by where the glyph's cell starts.
int bearingBig(char c) { return c == '1' ? 5 : (c == '-' ? 3 : 2); }       // logisoso24: every digit advances 15
int inkRightBig(char c) { return c == '1' ? 10 : (c == '-' ? 11 : 13); }  // last inked column in that cell
int bearingBody(char c) {                                                   // helvR10 capitals
  static const int8_t kCap[26] = {0, 1, 1, 1, 1, 1, 1, 1, 2, 0, 1, 2, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1};
  return (c >= 'A' && c <= 'Z') ? kCap[c - 'A'] : 1;
}

// "35%" in the small font on fixed digit cells.  form 0: with the percent sign, 1: without it,
// 2: nothing at all (for when there is no room).
int pctWidth(u8g2_t *u, int pct, int form) {
  if (form >= 2) return 0;
  u8g2_SetFont(u, F_SMALL);
  char s[8];
  snprintf(s, sizeof s, form == 0 ? "%d%%" : "%d", pct);
  return pitchWidth(u, s);
}

int pctText(u8g2_t *u, int x, int baseline, int pct, int form) {
  if (form >= 2) return x;
  u8g2_SetFont(u, F_SMALL);
  char s[8];
  snprintf(s, sizeof s, form == 0 ? "%d%%" : "%d", pct);
  txtPitch(u, x, baseline, s);
  return x + pitchWidth(u, s);
}

int clampPct(int pct) { return pct < 0 ? 0 : (pct > 100 ? 100 : pct); }

// A glyph followed by its percentage (gap 2): how wide it is in each form.
int groupWidth(u8g2_t *u, int glyphW, int pct, int form) { return glyphW + (form < 2 ? 2 + pctWidth(u, pct, form) : 0); }

// The widest form of a glyph-and-percentage that fits in `room` px.
int fittingForm(u8g2_t *u, int glyphW, int pct, int room) {
  for (int form = 0; form < 2; form++)
    if (groupWidth(u, glyphW, pct, form) <= room) return form;
  return 2;
}

// Today's block, 0 .. kColA wide:
//
//   icon   15 deg   (moon) 52%  (drop) 35%      <- the first row shares its top with the big number
//          Overcast  H 19 deg  L 9 deg          <- the second its bottom
//
// The big number starts where the condition text below it does (their ink, not their cells).
// Wide numbers (-12, 104) squeeze the rows to the right of them: the moon loses its percentage,
// then the moon goes, and the high and low shrink their gaps and then their type.
void drawWeatherCurrent(u8g2_t *u, const UiModel &m) {
  const WeatherData &w = m.weather;
  if (!w.valid) {
    u8g2_SetFont(u, F_BODY);
    txtC(u, kColA / 2, kWxTop + 32, m.wifiOff ? "WiFi is off" : "Weather: waiting...");
    return;
  }
  drawWxIcon(u, wmoIcon(w.code), w.isDay, 6, kWxTop + 8, 42);
  const int tx = 54;            // where the condition text starts
  const int limit = kColA - 4;  // nothing reaches the separator
  char buf[40];

  u8g2_SetFont(u, F_BODY);
  if (w.ageSec > 3600) {  // stale data: say so rather than silently showing old numbers
    snprintf(buf, sizeof buf, "%s (old)", wmoText(w.code));
  } else {
    snprintf(buf, sizeof buf, "%s", wmoText(w.code));
  }
  txtFit(u, tx, kWxTop + 47, limit - tx, buf);
  const int textInk = tx + bearingBody(buf[0]);

  u8g2_SetFont(u, F_BIG);
  char num[12];
  snprintf(num, sizeof num, "%d", tempOf(w.temp, m.useFahrenheit));
  const int numX = textInk - bearingBig(num[0]);
  txtPitch(u, numX, kWxTop + 31, num);
  const int lastCell = numX + pitchWidth(u, num) - digitPitch(u);
  const int inkEnd = lastCell + inkRightBig(num[strlen(num) - 1]);
  degreeRing(u, inkEnd + 5, kWxTop + 13, 3);
  const int hx = inkEnd + 5 + 3 + 7;  // right of the ring
  const int room = limit - hx;

  // moon and chance of rain, side by side
  const int top = kWxTop + 7;
  const int rain = clampPct(w.day[0].rainPct), moonPct = clampPct(m.moonLit[0]);
  bool moonShown = m.moonValid;
  int moonForm = 0;
  if (moonShown && groupWidth(u, kMoonD, moonPct, 0) + 6 + groupWidth(u, kRainW, rain, 0) > room) moonForm = 2;
  if (moonShown && groupWidth(u, kMoonD, moonPct, moonForm) + 6 + groupWidth(u, kRainW, rain, 0) > room) moonShown = false;
  const int rainForm = fittingForm(u, kRainW, rain, room);
  int x = hx;
  if (moonShown) {
    iconMoon(u, x, top, kMoonD, moonPct, m.moonWaxing[0], m.southern);
    x = pctText(u, x + kMoonD + 2, top + 9, moonPct, moonForm) + (moonForm < 2 ? 6 : 4);
  }
  iconRain(u, x, top + 9 - kRainH);  // same rule: bottom row = bottom of the digits (baseline top + 9, minus 1)
  pctText(u, x + kRainW + 2, top + 9, rain, rainForm);

  // high and low on one line: spaced, then tight, then in the small bold type
  char hi[8], lo[8];
  snprintf(hi, sizeof hi, "%d", tempOf(w.day[0].tmax, m.useFahrenheit));
  snprintf(lo, sizeof lo, "%d", tempOf(w.day[0].tmin, m.useFahrenheit));
  for (int variant = 0; variant < 3; variant++) {
    u8g2_SetFont(u, variant < 2 ? F_BOLD : F_SMALLB);
    const bool spaced = variant == 0;
    const int gap = variant == 0 ? 7 : (variant == 1 ? 5 : 4);
    char a[12], b[12];
    snprintf(a, sizeof a, spaced ? "H %s" : "H%s", hi);
    snprintf(b, sizeof b, spaced ? "L %s" : "L%s", lo);
    const int wa = tw(u, a), wb = tw(u, b);
    const int end = hx + wa + 3 + 2 + gap + wb + 3 + 2;  // each followed by its degree ring
    if (end > limit && variant < 2) continue;
    const int y = kWxTop + 31;
    txt(u, hx, y, a);
    degreeRing(u, hx + wa + 3, y - (variant < 2 ? 8 : 6), 2);
    const int bx = hx + wa + 3 + 2 + gap;
    txt(u, bx, y, b);
    degreeRing(u, bx + wb + 3, y - (variant < 2 ? 8 : 6), 2);
    break;
  }
}

// A day of the forecast, 112 px wide:
//
//   Mon
//   icon  20 deg   (moon) 52%
//         12 deg   (drop) 35%
// The stack on the right is at the same place in every column; a wide temperature pushes it right
// and, if that leaves too little room, each row drops its percent sign and then its number.
void drawWeatherDay(u8g2_t *u, const UiModel &m, int idx, int x0, int x1) {
  const WeatherData &w = m.weather;
  if (!w.valid) return;
  const WeatherDay &d = w.day[idx];
  char buf[24];
  u8g2_SetFont(u, F_BOLD);
  txt(u, x0 + 8, kWxTop + 15, timeutil::weekdayShort(d.weekday));

  drawWxIcon(u, wmoIcon(d.code), true, x0 + 6, kWxTop + 19, 32);

  const int tx = x0 + 44;
  u8g2_SetFont(u, F_H2);
  snprintf(buf, sizeof buf, "%d", tempOf(d.tmax, m.useFahrenheit));
  const int hiW = tw(u, buf);
  txt(u, tx, kWxTop + 33, buf);
  degreeRing(u, tx + hiW + 3, kWxTop + 25, 2);
  u8g2_SetFont(u, F_BODY);
  snprintf(buf, sizeof buf, "%d", tempOf(d.tmin, m.useFahrenheit));
  const int loW = tw(u, buf);
  txt(u, tx, kWxTop + 47, buf);
  degreeRing(u, tx + loW + 3, kWxTop + 40, 2);

  int sx = tx + (hiW > loW ? hiW : loW) + 3 + 2 + 4;
  if (sx < x0 + 72) sx = x0 + 72;
  const int room = x1 - 3 - sx;
  if (m.moonValid) {
    const int pct = clampPct(m.moonLit[idx]);
    iconMoon(u, sx, kWxTop + 22, kMoonD, pct, m.moonWaxing[idx], m.southern);
    pctText(u, sx + kMoonD + 2, kWxTop + 31, pct, fittingForm(u, kMoonD, pct, room));
  }
  const int rain = clampPct(d.rainPct);
  iconRain(u, sx + kRainInset, kWxTop + 45 - kRainH);  // its bottom row is the bottom of the digits (baseline - 1)
  pctText(u, sx + kMoonD + 2, kWxTop + 45, rain, fittingForm(u, kMoonD, rain, room));
}

void drawWeatherBand(u8g2_t *u, const UiModel &m) {
  ink(u);
  u8g2_DrawHLine(u, 0, kWxTop, UI_WIDTH);
  u8g2_DrawVLine(u, kColA, kWxTop + 4, kWxBottom - kWxTop - 8);
  u8g2_DrawVLine(u, kColB, kWxTop + 4, kWxBottom - kWxTop - 8);
  drawWeatherCurrent(u, m);
  drawWeatherDay(u, m, 1, kColA, kColB);
  drawWeatherDay(u, m, 2, kColB, UI_WIDTH);
}

void drawProgress(u8g2_t *u, int x, int y, int w, int h, uint32_t pos, uint32_t dur) {
  u8g2_DrawFrame(u, x, y, w, h);
  if (dur > 0) {
    uint64_t f = (uint64_t)(w - 4) * (pos > dur ? dur : pos) / dur;
    if (f > 0) u8g2_DrawBox(u, x + 2, y + 2, (uint16_t)f, h - 4);
  }
}

void drawBottomBand(u8g2_t *u, const UiModel &m) {
  ink(u);
  u8g2_DrawHLine(u, 0, kBottomTop, UI_WIDTH);
  const SpotifyInfo &s = m.spotify;
  const int top = kBottomTop;

  bool active = (s.status == SPOTIFY_PLAYING || s.status == SPOTIFY_PAUSED) && s.title[0];
  if (active) {
    iconNote(u, 8, top + 7, 15);
    // title row
    u8g2_SetFont(u, F_BOLD);
    txtFit(u, 32, top + 18, 336, s.title);
    if (s.status == SPOTIFY_PLAYING) {
      iconPlay(u, 379, top + 7, 13);
    } else {
      iconPause(u, 379, top + 7, 13);
    }
    // artist row + elapsed / total
    char t1[12], t2[12], tm[28];
    timeutil::formatDuration(s.progressMs, t1, sizeof t1);
    timeutil::formatDuration(s.durationMs, t2, sizeof t2);
    snprintf(tm, sizeof tm, "%s / %s", t1, t2);
    u8g2_SetFont(u, F_SMALL);
    int tmW = pitchWidth(u, tm);
    u8g2_SetFont(u, F_BODY);
    txtFit(u, 32, top + 32, 396 - 32 - tmW - 12, s.artist);
    u8g2_SetFont(u, F_SMALL);
    txtPitchR(u, 394, top + 32, tm);
    drawProgress(u, 8, top + 36, 384, 6, s.progressMs, s.durationMs);
    return;
  }

  // nothing playing: setup hint, error, or weather extras
  u8g2_SetFont(u, F_BODY);
  char buf[112];
  const int base = top + 26;
  if (s.status == SPOTIFY_NEEDS_LINK && s.linkUrl[0]) {
    snprintf(buf, sizeof buf, "Spotify: open %s to link", s.linkUrl);
    txtC(u, 200, base, buf);
    return;
  }
  if (s.linkDaysLeft >= 0 && s.linkDaysLeft <= 14 && s.linkUrl[0] && s.status != SPOTIFY_NEEDS_LINK) {
    // Spotify expires the link after six months: say so while there is time to renew
    snprintf(buf, sizeof buf, "Spotify link ends in %d d - renew at %s", s.linkDaysLeft, s.linkUrl);
    txtC(u, 200, base, buf);
    return;
  }
  if (s.status == SPOTIFY_ERROR && s.message[0]) {
    snprintf(buf, sizeof buf, "Spotify: %s", s.message);
    txtFit(u, 8, base, 384, buf);
    return;
  }
  if (m.weather.valid) {
    const WeatherData &w = m.weather;
    int wind = iround(m.useFahrenheit ? w.windKmh * 0.621371f : w.windKmh);
    snprintf(buf, sizeof buf, "Sunrise %s   Sunset %s   UV %.0f   Wind %d %s", w.sunrise, w.sunset,
             w.uvMax, wind, m.useFahrenheit ? "mph" : "km/h");
    txtC(u, 200, base, buf);
  } else if (m.wifiOff) {
    txtC(u, 200, base, "WiFi is off: clock and indoor sensor only");
  } else if (s.status == SPOTIFY_IDLE) {
    txtC(u, 200, base, "Spotify idle");
  }
}

// Feedback overlay for key presses: replaces the bottom strip with a black band.
void drawToast(u8g2_t *u, const UiModel &m) {
  u8g2_SetFont(u, F_H1);
  int iconW = m.toastIcon == TOAST_NONE ? 0 : 26;
  int x = (UI_WIDTH - (tw(u, m.toast) + iconW)) / 2;
  int cy = kBottomTop + (UI_HEIGHT - kBottomTop) / 2;
  ink(u);
  u8g2_DrawBox(u, 0, kBottomTop, UI_WIDTH, UI_HEIGHT - kBottomTop);
  paper(u);
  int iy = cy - 8;
  switch (m.toastIcon) {
    case TOAST_PLAY: iconPlay(u, x, iy, 16); break;
    case TOAST_PAUSE: iconPause(u, x, iy, 16); break;
    case TOAST_NEXT: iconNext(u, x, iy, 16); break;
    case TOAST_PREV: iconPrev(u, x, iy, 16); break;
    case TOAST_WARN: txt(u, x + 4, cy + 6, "!"); break;
    default: break;
  }
  txt(u, x + iconW, cy + 6, m.toast);
  ink(u);
}

void drawDashboard(u8g2_t *u, const UiModel &m) {
  drawAnalogClock(u, kClockCx, kClockCy, kClockR, m);
  drawDateTime(u, m);
  drawIndoor(u, m);
  drawWeatherBand(u, m);
  drawBottomBand(u, m);
}

// ---------------------------------------------------------------------------
// Now Playing page
// ---------------------------------------------------------------------------
void drawNowPlayingPage(u8g2_t *u, const UiModel &m) {
  const SpotifyInfo &s = m.spotify;
  ink(u);
  bool active = (s.status == SPOTIFY_PLAYING || s.status == SPOTIFY_PAUSED) && s.title[0];
  bool playing = s.status == SPOTIFY_PLAYING;

  u8g2_SetFont(u, F_SMALLB);
  txt(u, 8, 36, !active ? "SPOTIFY" : (playing ? "NOW PLAYING" : "PAUSED"));
  if (active && s.device[0]) {
    u8g2_SetFont(u, F_SMALL);
    char dev[72];
    if (s.volume >= 0) {
      snprintf(dev, sizeof dev, "on %s \xC2\xB7 vol %d%%", s.device, s.volume);
    } else {
      snprintf(dev, sizeof dev, "on %s", s.device);
    }
    int w = tw(u, dev);
    txtFit(u, 392 - (w > 240 ? 240 : w), 36, 240, dev);
  }

  if (!active) {
    u8g2_SetFont(u, F_H1);
    const char *msg = "Nothing playing";
    char buf[96];
    const char *hint = "Start something in Spotify and it will show up here";
    if (m.wifiOff) {
      msg = "WiFi is off";
      hint = "Set wifi = on in the SD card settings";
    } else if (s.status == SPOTIFY_DISABLED) {
      msg = "Spotify is not set up";
      hint = "Add a Client ID: secrets.h or SD card settings";
    } else if (s.status == SPOTIFY_NEEDS_LINK) {
      msg = "Link your account";
      if (s.linkUrl[0]) {
        snprintf(buf, sizeof buf, "Open %s in a browser", s.linkUrl);
        hint = buf;
      } else {
        hint = "Waiting for the network";
      }
    } else if (s.status == SPOTIFY_ERROR && s.message[0]) {
      msg = s.message;
      hint = "Will retry automatically";
    }
    txtC(u, 200, 100, msg);
    u8g2_SetFont(u, F_BODY);
    txtC(u, 200, 126, hint);
  } else {
    // title: one line at 18 px if it fits, otherwise up to two lines at 14 px
    char lines[2][96];
    u8g2_SetFont(u, u8g2_font_helvB18_tf);
    int n = wrapText(u, s.title, 384, lines, 1);
    int y = 64;
    bool single = n == 1 && strstr(lines[0], "...") == nullptr;
    if (single) {
      txt(u, 8, y, lines[0]);
    } else {
      u8g2_SetFont(u, F_H1);
      n = wrapText(u, s.title, 384, lines, 2);
      y = 60;
      for (int i = 0; i < n; i++) {
        txt(u, 8, y, lines[i]);
        if (i + 1 < n) y += 18;
      }
    }
    u8g2_SetFont(u, F_H2);
    y += 24;
    txtFit(u, 8, y, 384, s.artist);
    if (s.album[0]) {
      u8g2_SetFont(u, F_BODY);
      y += 19;
      txtFit(u, 8, y, 384, s.album);
    }
  }

  // progress
  if (active) {
    const int by = 158;
    drawProgress(u, 8, by, 384, 10, s.progressMs, s.durationMs);
    char t1[12], t2[12];
    timeutil::formatDuration(s.progressMs, t1, sizeof t1);
    timeutil::formatDuration(s.durationMs, t2, sizeof t2);
    u8g2_SetFont(u, F_BODY);
    txtPitch(u, 8, by + 25, t1);
    txtPitchR(u, 392, by + 25, t2);
  }

  // key legend: the three click patterns of the KEY button
  u8g2_DrawHLine(u, 0, 196, UI_WIDTH);
  const int iy = 208, isz = 24;
  const int cxs[3] = {100, 200, 300};
  iconPrev(u, cxs[0] - isz / 2, iy, isz);
  if (playing) {
    iconPause(u, cxs[1] - isz / 2, iy, isz);
  } else {
    iconPlay(u, cxs[1] - isz / 2, iy, isz);
  }
  iconNext(u, cxs[2] - isz / 2, iy, isz);
  u8g2_SetFont(u, F_SMALLB);
  txtC(u, cxs[0], iy + 42, "3 clicks");
  txtC(u, cxs[1], iy + 42, "1 click");
  txtC(u, cxs[2], iy + 42, "2 clicks");
  u8g2_SetFont(u, F_SMALL);
  txtC(u, cxs[0], iy + 54, "previous");
  txtC(u, cxs[1], iy + 54, playing ? "pause" : "play");
  txtC(u, cxs[2], iy + 54, "next");
  txtC(u, 200, 292, "KEY button   |   BOOT button: next page");
}

// ---------------------------------------------------------------------------
// Info pages: SYSTEM INFO and POWER AND SETTINGS share this; the sketch fills the lines
// ---------------------------------------------------------------------------
void drawInfoPage(u8g2_t *u, const UiModel &m) {
  ink(u);
  u8g2_SetFont(u, F_SMALLB);
  txt(u, 8, 36, m.page == PAGE_POWER ? "POWER AND SETTINGS" : "SYSTEM INFO");
  u8g2_SetFont(u, F_MONO);
  int y = 54;
  for (int i = 0; i < m.infoCount && i < UI_INFO_LINES; i++) {
    txt(u, 8, y, m.info[i]);
    y += 17;
  }
}

// ---------------------------------------------------------------------------
// Legend page: what the symbols and the button clicks mean, drawn with the same functions the
// other pages use so the two cannot drift apart.
// ---------------------------------------------------------------------------
void drawLegendPage(u8g2_t *u, const UiModel &) {
  ink(u);
  u8g2_SetFontMode(u, 1);
  u8g2_SetFont(u, F_SMALLB);
  txt(u, 8, 36, "LEGEND");

  // weather: day and night, each kind
  const WxIcon icons[9] = {WX_CLEAR, WX_CLEAR, WX_PARTLY, WX_CLOUDY, WX_FOG, WX_DRIZZLE, WX_RAIN, WX_SNOW, WX_THUNDER};
  const char *const names[9] = {"clear", "night", "partly", "cloudy", "fog", "drizzle", "rain", "snow", "storm"};
  for (int i = 0; i < 9; i++) {
    drawWxIcon(u, icons[i], i != 1, 8 + i * 44, 44, 24);
    u8g2_SetFont(u, F_SMALL);
    txtC(u, 8 + i * 44 + 12, 80, names[i]);
  }

  // chance of rain, and the moon
  u8g2_SetFont(u, F_SMALL);
  iconRain(u, 8 + kRainInset, 100 - kRainH);
  txt(u, 8 + kMoonD + 2, 100, "35%");
  txt(u, 56, 100, "chance of rain: today and the next two days");
  const int phases[5] = {0, 25, 50, 75, 100};
  for (int i = 0; i < 5; i++) {
    const int x = 8 + i * 48;
    iconMoon(u, x, 108, kMoonD, phases[i], i < 3, false);
    char s[8];
    snprintf(s, sizeof s, "%d%%", phases[i]);
    txtPitch(u, x + kMoonD + 2, 117, s);
  }
  txt(u, 256, 117, "moon: how much is lit");
  txt(u, 256, 128, "(the lit side flips in the south)");

  // WiFi
  for (int i = 0; i < 5; i++) {
    const int rssi[5] = {-50, -60, -70, -80, -100};
    drawWifi(u, 8 + i * 22, 148, i < 4, rssi[i]);
  }
  u8g2_SetFont(u, F_SMALL);
  txt(u, 124, 142, "WiFi signal, strong to weak;");
  txt(u, 124, 153, "crossed out: not connected");

  // the battery gauge, in the states it shows
  struct Gauge {
    int percent, charge;
    bool low;
    const char *what;
  };
  const Gauge gauges[4] = {{54, CHARGE_DISCHARGING, false, "on battery"},
                           {62, CHARGE_CHARGING, false, "charging"},
                           {100, CHARGE_FULL, false, "full"},
                           {17, CHARGE_DISCHARGING, true, "low: it blinks"}};
  for (int i = 0; i < 4; i++) {
    UiModel g;
    g.batPresent = true;
    g.batPercent = gauges[i].percent;
    g.batCharge = gauges[i].charge;
    g.batLow = gauges[i].low;
    g.batBlinkOn = gauges[i].low;  // the inverted phase of the blink
    const int gx = 12 + i * 98;    // left edge of the gauge and of its caption
    drawBatteryAt(u, gx + batteryLayout(u, g).width, 160, g);
    u8g2_SetFont(u, F_SMALL);
    txt(u, gx, 194, gauges[i].what);
  }

  // Spotify
  iconNote(u, 8, 203, 15);
  iconPlay(u, 168, 204, 13);
  iconPause(u, 288, 204, 13);
  u8g2_SetFont(u, F_SMALL);
  txt(u, 28, 215, "Spotify: now playing");
  txt(u, 186, 215, "playing");
  txt(u, 306, 215, "paused");

  // the buttons
  u8g2_DrawHLine(u, 0, 223, UI_WIDTH);
  u8g2_SetFont(u, F_SMALLB);
  txt(u, 8, 237, "KEY BUTTON");
  txt(u, 208, 237, "BOOT BUTTON");
  u8g2_SetFont(u, F_BODY);
  // (KEY held for five seconds and let go restarts the clock, which is when it reads the SD card)
  const char *const key[4] = {"1 click: play / pause", "2 clicks: next track", "3 clicks: previous track",
                              "hold: refresh; 5 s: restart"};
  const char *const boot[4] = {"1 click: next page", "2 clicks: dashboard", "3 clicks: system info",
                               "hold: invert the screen"};
  for (int i = 0; i < 4; i++) {
    txt(u, 8, 252 + i * 13, key[i]);
    txt(u, 208, 252 + i * 13, boot[i]);
  }
}

}  // namespace

void uiDraw(u8g2_t *u, const UiModel &m) {
  u8g2_SetFontMode(u, 1);  // transparent glyph backgrounds
  u8g2_SetFontDirection(u, 0);
  u8g2_SetFontPosBaseline(u);
  ink(u);

  drawStatusBar(u, m);
  switch (m.page) {
    case PAGE_NOW_PLAYING: drawNowPlayingPage(u, m); break;
    case PAGE_INFO:
    case PAGE_POWER: drawInfoPage(u, m); break;
    case PAGE_LEGEND: drawLegendPage(u, m); break;
    default: drawDashboard(u, m); break;
  }
  if (m.toast[0]) drawToast(u, m);
}

// The screen the clock leaves behind when it switches itself off to protect the battery.  Static:
// the panel keeps showing it while the chip sleeps.
void uiDrawBatteryEmpty(u8g2_t *u, float volts, float restartVolts, bool offerOverride) {
  u8g2_SetFontMode(u, 1);
  u8g2_SetFontDirection(u, 0);
  u8g2_SetFontPosBaseline(u);
  ink(u);
  const int w = 132, h = 64, x = (UI_WIDTH - w - 10) / 2, y = 34;
  for (int i = 0; i < 3; i++) u8g2_DrawFrame(u, x + i, y + i, w - 2 * i, h - 2 * i);  // a thick outline
  u8g2_DrawBox(u, x + w, y + 18, 10, h - 36);                                            // the contact
  u8g2_DrawBox(u, x + 9, y + 9, 7, h - 18);                                              // a sliver of charge
  char s[64];
  u8g2_SetFont(u, F_BIG);
  txtC(u, UI_WIDTH / 2, 146, "Battery empty");
  u8g2_SetFont(u, F_H2);
  snprintf(s, sizeof s, "Charge it: the clock restarts at %.2f V", restartVolts);
  txtC(u, UI_WIDTH / 2, 178, s);
  u8g2_SetFont(u, F_BODY);
  snprintf(s, sizeof s, "Battery now %.2f V (looked at every 5 minutes)", volts);
  txtC(u, UI_WIDTH / 2, 202, s);
  txtC(u, UI_WIDTH / 2, 222, "It was shut down to protect the cell.");
  if (offerOverride) {
    u8g2_SetFont(u, F_SMALL);
    txtC(u, UI_WIDTH / 2, 280, "Hold the KEY button for 3 seconds to start anyway");
  }
}

// ---------------------------------------------------------------------------
// The firmware-update screen (start-up, while the SD card's firmware is checked and written)
// ---------------------------------------------------------------------------
const int kFwBarX = 50, kFwBarY = 168, kFwBarW = 300, kFwBarH = 16;  // outer size of the progress bar
const int kFwTextW = 380;                                            // the widest line of text

// A 2 px frame, a 1 px gap, and a fill of (width - 6) * percent / 100 pixels.
static void fwBar(u8g2_t *u, int percent) {
  const int p = percent < 0 ? 0 : (percent > 100 ? 100 : percent);
  ink(u);
  u8g2_DrawFrame(u, kFwBarX, kFwBarY, kFwBarW, kFwBarH);
  u8g2_DrawFrame(u, kFwBarX + 1, kFwBarY + 1, kFwBarW - 2, kFwBarH - 2);
  u8g2_DrawBox(u, kFwBarX + 3, kFwBarY + 3, (kFwBarW - 6) * p / 100, kFwBarH - 6);
}

// A chip with four pins on each side.
static void fwChip(u8g2_t *u, int cx, int y) {
  const int s = 44, x = cx - s / 2;
  ink(u);
  for (int i = 0; i < 3; i++) u8g2_DrawFrame(u, x + i, y + i, s - 2 * i, s - 2 * i);
  u8g2_DrawBox(u, x + 14, y + 14, s - 28, s - 28);
  for (int i = 0; i < 4; i++) {
    const int p = 7 + i * 10;
    u8g2_DrawBox(u, x - 8, y + p, 8, 3);
    u8g2_DrawBox(u, x + s, y + p, 8, 3);
    u8g2_DrawBox(u, x + p, y - 8, 3, 8);
    u8g2_DrawBox(u, x + p, y + s, 3, 8);
  }
}

// Centred in the larger type if it fits, else in the body type, else cut short.
static void fwLine(u8g2_t *u, int y, const char *s, const uint8_t *big, const uint8_t *small) {
  u8g2_SetFont(u, big);
  if (tw(u, s) > kFwTextW) u8g2_SetFont(u, small);
  if (tw(u, s) <= kFwTextW) {
    txtC(u, UI_WIDTH / 2, y, s);
  } else {
    txtFit(u, (UI_WIDTH - kFwTextW) / 2, y, kFwTextW, s);
  }
}

void uiDrawFirmwareUpdate(u8g2_t *u, const UiFwScreen &s) {
  u8g2_SetFontMode(u, 1);
  u8g2_SetFontDirection(u, 0);
  u8g2_SetFontPosBaseline(u);
  ink(u);
  fwChip(u, UI_WIDTH / 2, 26);
  const char *heading = s.kind == FW_DONE ? "Firmware updated" : (s.kind == FW_PROBLEM ? "Not updated" : "Firmware update");
  u8g2_SetFont(u, F_BIG);
  txtC(u, UI_WIDTH / 2, 118, heading);
  if (s.what[0]) fwLine(u, 152, s.what, F_H2, F_BODY);
  if (s.percent >= 0) {
    fwBar(u, s.percent);
    char pct[16];
    snprintf(pct, sizeof pct, "%d %%", s.percent < 0 ? 0 : (s.percent > 100 ? 100 : s.percent));
    u8g2_SetFont(u, F_BODY);
    txtC(u, UI_WIDTH / 2, 206, pct);
  }
  if (s.detail[0]) fwLine(u, 232, s.detail, F_BODY, F_SMALL);
  u8g2_SetFont(u, F_SMALL);
  if (s.keepPowered) {
    txtC(u, UI_WIDTH / 2, 282, "Do not switch off or take the card out");
  } else if (s.kind == FW_PROBLEM) {
    txtC(u, UI_WIDTH / 2, 282, "The clock carries on with the firmware it has");
  } else if (s.kind == FW_DONE) {
    txtC(u, UI_WIDTH / 2, 282, "On trial for its first minute: if it fails, the old one returns");
  }
}

// An infinity-mirror cube, in white on black: a cube of mirrors that turns, each face a window on the lattice of
// its own lit frame, reflected without end (infinity_cube.h has the geometry, and how bright each line is).  It
// is no page of the clock: the sketch shows it to whoever holds both buttons down for long enough.

// A line of which `on` pixels are drawn and `off` are not, over and over: the fainter reflections.
static void cubeDashes(u8g2_t *u, int x0, int y0, int x1, int y1, int on, int off) {
  const int dx = abs(x1 - x0), dy = abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx - dy, i = 0;
  for (;;) {
    if (i < on) u8g2_DrawPixel(u, x0, y0);
    if (++i == on + off) i = 0;
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

// An edge of the cube itself, three pixels wide.
static void cubeEdge(u8g2_t *u, int x0, int y0, int x1, int y1) {
  u8g2_DrawLine(u, x0, y0, x1, y1);
  u8g2_DrawLine(u, x0 + 1, y0, x1 + 1, y1);
  u8g2_DrawLine(u, x0 - 1, y0, x1 - 1, y1);
  u8g2_DrawLine(u, x0, y0 + 1, x1, y1 + 1);
  u8g2_DrawLine(u, x0, y0 - 1, x1, y1 - 1);
}

namespace {
struct CubePen {
  u8g2_t *u;
  void line(int x0, int y0, int x1, int y1, infcube::Style style) {
    switch (style) {
      case infcube::BOLD: cubeEdge(u, x0, y0, x1, y1); break;
      case infcube::SOLID: u8g2_DrawLine(u, x0, y0, x1, y1); break;
      case infcube::FADE1: cubeDashes(u, x0, y0, x1, y1, 3, 1); break;
      case infcube::FADE2: cubeDashes(u, x0, y0, x1, y1, 1, 1); break;
      case infcube::FADE3: cubeDashes(u, x0, y0, x1, y1, 1, 3); break;
      case infcube::FADE4: cubeDashes(u, x0, y0, x1, y1, 1, 7); break;
      default: break;
    }
  }
  void dot(int x, int y) { u8g2_DrawDisc(u, x, y, 3, U8G2_DRAW_ALL); }  // a glint on a corner
};
}  // namespace

// ... and the other cube: sand on its walls, which runs to whatever side of each face is down (sand_cube.h).
namespace {
struct SandPen {
  u8g2_t *u;
  void grain(int x, int y, int size) { u8g2_DrawBox(u, x - size / 2, y - size / 2, size, size); }
  void line(int x0, int y0, int x1, int y1, infcube::Style) { cubeEdge(u, x0, y0, x1, y1); }
  void dot(int x, int y) { u8g2_DrawDisc(u, x, y, 3, U8G2_DRAW_ALL); }
};
}  // namespace


// One of the two cubes, tumbling (infcube::tumble()).  The sand keeps its own state from call to call: it starts
// afresh when the time does, or with another seed.
void uiDrawCube(u8g2_t *u, const UiCube &cube) {
  static sandcube::Sand sand;
  static uint32_t lastMs = 0xFFFFFFFFu, lastSeed = 0;
  if (cube.elapsedMs < lastMs || cube.seed != lastSeed) sand.reset(cube.seed);  // a new showing
  lastMs = cube.elapsedMs;
  lastSeed = cube.seed;
  infcube::V3 axis[3];
  infcube::tumble(cube.turnMs, cube.seed, cube.shove, axis);
  sand.advance(cube.elapsedMs, axis);  // (the sand runs on while the mirrors are shown: it is the same cube)
  ink(u);
  u8g2_DrawBox(u, 0, 0, UI_WIDTH, UI_HEIGHT);
  paper(u);
  if (cube.mirrors) {
    CubePen pen{u};
    infcube::draw(axis, cube.elapsedMs, UI_WIDTH, UI_HEIGHT, pen);
  } else {
    SandPen pen{u};
    sand.draw(axis, UI_WIDTH, UI_HEIGHT, pen);
  }
  ink(u);
}
