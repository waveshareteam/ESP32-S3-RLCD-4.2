#pragma once

// A cube with sand on its walls: it tumbles (infcube::tumble()), and on every face the grains run to whatever side is down at the
// moment, pile up there, and pour away again as the cube goes on turning.  Each face has its own grains, in a
// grid; "down" is the bottom of the screen, as that face sees it.  A face that lies flat keeps its sand where it
// is.
//
// Pure logic (no drawing, no Arduino): Sand keeps the grains and moves them, draw() hands the picture to a sink
// for a screen of a given size.  ui.cpp draws it, and tools/tests and the UI preview run the same code on a PC.

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "infinity_cube.h"  // (the vectors, the eye and the styles of line)

namespace sandcube {

using infcube::V3;

const int kCells = 28;             // cells along an edge of a face (32 at most: a row is the bits of a word)
const int kFillPercent = 36;       // how much of every face is sand
const uint32_t kStepMs = 35;       // the grains move this often
const int kMaxStepsAtOnce = 4;     // ... and no more than this many times in one call, however late it comes
const float kFlat = 0.10f;         // a face that sees less than this much of gravity keeps its sand still
const float kGrain = 0.74f;        // a grain's width, of the cell it sits in

// The two axes that span face `f` (0..5: the faces at +x, -x, +y, -y, +z, -z of the cube), and the way it looks.
inline void faceAxes(const V3 ax[3], int f, V3 *n, V3 *u, V3 *v) {
  *n = infcube::mul(ax[f >> 1], (f & 1) ? -1.0f : 1.0f);
  *u = ax[((f >> 1) + 1) % 3];
  *v = ax[((f >> 1) + 2) % 3];
}

class Sand {
 public:
  Sand() { reset(1); }

  // Sand at random places, the same amount on every face; the same `seed` gives the same sand.
  void reset(uint32_t seed) {
    rng_ = seed ? seed : 1;
    steps_ = 0;
    memset(rows_, 0, sizeof rows_);
    const int want = kCells * kCells * kFillPercent / 100;
    for (int f = 0; f < 6; f++) {
      for (int placed = 0; placed < want;) {
        const uint32_t r = next();
        const int i = (int)(r % (uint32_t)kCells), j = (int)((r >> 12) % (uint32_t)kCells);
        if (has(f, i, j)) continue;
        put(f, i, j, true);
        placed++;
      }
    }
  }

  // Lets the sand run until `ms` after the start, with the cube's axes at `ax` (infcube::tumble()).  Call it with
  // the time of every frame.
  void advance(uint32_t ms, const V3 ax[3]) {
    const uint32_t due = ms / kStepMs;
    if (due < steps_) steps_ = due;  // (the time started again)
    int n = (int)(due - steps_);
    if (n > kMaxStepsAtOnce) n = kMaxStepsAtOnce;  // a frame that came late is not made up for
    for (int k = n; k > 0; k--) step(ax);
    steps_ = due;
  }

  bool has(int f, int i, int j) const { return (rows_[f][j] >> i) & 1u; }
  int grains(int f) const {
    int n = 0;
    for (int j = 0; j < kCells; j++)
      for (int i = 0; i < kCells; i++) n += has(f, i, j) ? 1 : 0;
    return n;
  }
  // Where the middle of face `f`'s sand is, each way across the face, from -1 to 1 (0, 0 = the middle of the face).
  void centre(int f, float *a, float *b) const {
    long si = 0, sj = 0, n = 0;
    for (int j = 0; j < kCells; j++)
      for (int i = 0; i < kCells; i++)
        if (has(f, i, j)) {
          si += 2 * i + 1;
          sj += 2 * j + 1;
          n++;
        }
    *a = n ? (float)si / (float)(n * kCells) - 1.0f : 0.0f;
    *b = n ? (float)sj / (float)(n * kCells) - 1.0f : 0.0f;
  }

  // One step of the sand, with the cube's axes at `ax`.
  void step(const V3 ax[3]) {
    for (int f = 0; f < 6; f++) {
      V3 n, u, v;
      faceAxes(ax, f, &n, &u, &v);
      stepFace(f, -u.y, -v.y);  // gravity pulls down the screen: what of it lies along each of the face's two sides
    }
  }

  // One step on face `f`, with gravity gu along its first side and gv along its second (as parts of 1).
  void stepFace(int f, float gu, float gv) {
    const float au = fabsf(gu), av = fabsf(gv), pull = sqrtf(gu * gu + gv * gv);
    if (pull < kFlat) return;
    const int su = gu > 0 ? 1 : -1, sv = gv > 0 ? 1 : -1;
    const uint32_t chanceU = (uint32_t)(au / (au + av) * 256.0f);                     // of 256: this grain's step is along the first side
    const uint32_t chanceMove = (uint32_t)((pull > 0.8f ? 1.0f : pull * 1.25f) * 256.0f);  // of 256: it moves at all (slowly on a gentle slope)
    // from the corner that is lowest, so that a grain can follow the one below it in the same step
    for (int b = 0; b < kCells; b++) {
      for (int a = 0; a < kCells; a++) {
        const int i = su > 0 ? kCells - 1 - a : a, j = sv > 0 ? kCells - 1 - b : b;
        if (!has(f, i, j)) continue;
        const uint32_t r = next();
        if ((r & 255u) >= chanceMove) continue;
        const bool alongU = ((r >> 8) & 255u) < chanceU;
        // Down, the way this grain takes for down (the steeper side of the face more often than the other).  If
        // something lies there: down and to one side, as sand slips off a heap, to the side gravity leans to
        // more often than not; then to the other.
        const int di = alongU ? su : 0, dj = alongU ? 0 : sv;
        if (go(f, i, j, i + di, j + dj)) continue;
        const uint32_t lean = alongU ? 256u - chanceU : chanceU;  // of 256: how much of the pull goes sideways for this grain
        const int toSide = (((r >> 16) & 255u) < 128u + lean) ? 1 : -1;  // (half and half when it pulls straight down)
        const int si = alongU ? 0 : su * toSide, sj = alongU ? sv * toSide : 0;
        if (go(f, i, j, i + di + si, j + dj + sj)) continue;
        if (go(f, i, j, i + di - si, j + dj - sj)) continue;
        // stuck on the heap: now and then it rolls sideways along it, which is what makes a heap settle flat
        if (((r >> 24) & 7u) == 0) go(f, i, j, i + si, j + sj);
      }
    }
  }

  // Hands the picture (the cube with its axes at `ax`, the sand as it lies) for a screen of w x h to `sink`:
  //   sink.grain(x, y, size)             a grain: a square of `size` pixels with its middle at x, y
  //   sink.line(x0, y0, x1, y1, style)   the edges of the faces that are seen (infcube::BOLD), after their sand
  //   sink.dot(x, y)                     a glint on every corner that is seen
  template <class Sink>
  void draw(const V3 ax[3], int w, int h, Sink &sink) const {
    const float focal = infcube::kFill * 0.5f * (float)h * sqrtf(infcube::kCamera * infcube::kCamera - 3.0f) / 1.7320508f;
    const float midX = 0.5f * (float)(w - 1), midY = 0.5f * (float)(h - 1);
    const float cell = 2.0f / (float)kCells;
    V3 corners[3][4];
    int seen = 0;
    for (int f = 0; f < 6 && seen < 3; f++) {
      V3 n, u, v;
      faceAxes(ax, f, &n, &u, &v);
      if (n.z > -1.0f / infcube::kCamera - 0.02f) continue;  // it looks away from the eye
      for (int j = 0; j < kCells; j++) {
        if (!rows_[f][j]) continue;
        const float b = -1.0f + cell * ((float)j + 0.5f);
        for (int i = 0; i < kCells; i++) {
          if (!has(f, i, j)) continue;
          const float a = -1.0f + cell * ((float)i + 0.5f);
          const V3 p = {n.x + u.x * a + v.x * b, n.y + u.y * a + v.y * b, n.z + u.z * a + v.z * b};
          const float k = focal / (infcube::kCamera + p.z);
          int size = (int)lroundf(cell * k * kGrain);
          if (size < 1) size = 1;
          sink.grain((int)lroundf(midX + p.x * k), (int)lroundf(midY - p.y * k), size);
        }
      }
      static const float kCorner[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
      for (int i = 0; i < 4; i++)
        corners[seen][i] = {n.x + u.x * kCorner[i][0] + v.x * kCorner[i][1], n.y + u.y * kCorner[i][0] + v.y * kCorner[i][1],
                            n.z + u.z * kCorner[i][0] + v.z * kCorner[i][1]};
      seen++;
    }
    auto px = [&](const V3 &p) { return (int)lroundf(midX + p.x * focal / (infcube::kCamera + p.z)); };
    auto py = [&](const V3 &p) { return (int)lroundf(midY - p.y * focal / (infcube::kCamera + p.z)); };
    for (int f = 0; f < seen; f++)
      for (int i = 0; i < 4; i++) sink.line(px(corners[f][i]), py(corners[f][i]), px(corners[f][(i + 1) & 3]), py(corners[f][(i + 1) & 3]), infcube::BOLD);
    for (int f = 0; f < seen; f++)
      for (int i = 0; i < 4; i++) sink.dot(px(corners[f][i]), py(corners[f][i]));
  }

 private:
  uint32_t next() {  // xorshift
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
  }
  void put(int f, int i, int j, bool on) {
    if (on) {
      rows_[f][j] |= 1u << i;
    } else {
      rows_[f][j] &= ~(1u << i);
    }
  }
  // Moves the grain at i, j to ni, nj if that is on the face and free.
  bool go(int f, int i, int j, int ni, int nj) {
    if (ni < 0 || ni >= kCells || nj < 0 || nj >= kCells || has(f, ni, nj)) return false;
    put(f, i, j, false);
    put(f, ni, nj, true);
    return true;
  }

  uint32_t rows_[6][kCells];  // a bit for every cell: bit i of row j of face f
  uint32_t rng_ = 1;
  uint32_t steps_ = 0;  // steps taken so far
};

}  // namespace sandcube
