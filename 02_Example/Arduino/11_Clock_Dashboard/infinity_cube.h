#pragma once

// An infinity-mirror cube: a cube of mirrors with lights along its edges, turning.  It is solid (only the faces
// that look at you are drawn), and each of those faces is a window: behind it the lit frame of the cube is seen
// again and again, one cube's length further on each time, in every direction, as mirrors facing mirrors show
// it.  That is a lattice of light without end, and every reflection costs a little of it: the nearest lines are
// solid, the ones behind them dashes, then dots, then nothing.  Now and then a pulse of light comes up out of
// the depth.
//
// Pure geometry (no drawing, no Arduino): draw() hands the picture at a time to a sink, line by line with how
// bright each is, then the corners that glint, for a screen of a given size.  ui.cpp draws them, and tools/tests
// and the UI preview run the same code on a PC.

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

namespace infcube {

enum Style : uint8_t {
  BOLD = 0,  // an edge of the cube itself
  SOLID,     // the nearest reflections, and the pulse
  FADE1,     // further in: three pixels in four ...
  FADE2,     // ... one in two ...
  FADE3,     // ... one in four ...
  FADE4,     // ... one in eight
  kStyles,
  NONE = kStyles  // too faint to draw
};

const int kDepth = 12;              // how many cubes deep the lattice is drawn behind a face
const int kSide = 2;                // ... and how many cubes to each side of the cube's own
const float kCamera = 6.0f;         // the eye's distance from the centre, in half edges of the cube
const float kFill = 0.90f;          // how much of the screen's half height the corners may reach
const uint32_t kPulseStepMs = 110;  // the pulse comes one cube nearer this often ...
const int kPulseRest = 10;          // ... and waits this many steps before the next one sets out
const int kSideCost = 2;            // what a cube to the side costs in brightness, against one straight behind

// How bright a line is that has been reflected `bounces` times on its way to the eye (1 = the cube's own far side).
// A reflection to the side counts double (kSideCost): seen at a slant, a mirror gives back less.
inline Style fade(int bounces) {
  return bounces <= 1 ? SOLID : bounces <= 2 ? FADE1 : bounces <= 4 ? FADE2 : bounces <= 7 ? FADE3 : bounces <= 11 ? FADE4 : NONE;
}

// How deep the pulse is at `ms` (kDepth = the deepest, 1 = the nearest; 0 or less = between two pulses).
inline int pulseAt(uint32_t ms) { return kDepth - (int)((ms / kPulseStepMs) % (uint32_t)(kDepth + kPulseRest)); }

struct V3 {
  float x, y, z;
};
inline V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 mul(V3 a, float k) { return {a.x * k, a.y * k, a.z * k}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

// How a cube tumbles: where its three axes point (x to the right, y up, z away from the eye) when it is
// `turnMs` along its way, and has been shoved round by `shove` on top of that (Shove::angle; null for none).
// It turns about all three axes at once, and on each of them now faster, now slower, now the other way, by a
// slow rule of its own that `seed` picks: no two showings turn alike, and nothing ever jerks.  Without a shove
// the way is a function of turnMs alone, so a cube that is taken back along it (turnMs going down) turns
// exactly the other way round.
inline void tumble(float turnMs, uint32_t seed, const float shove[3], V3 out[3]) {
  // six phases out of the seed (a small generator: the same seed, the same tumble)
  float phase[6];
  uint32_t r = seed ? seed : 1;
  for (int i = 0; i < 6; i++) {
    r ^= r << 13;
    r ^= r >> 17;
    r ^= r << 5;
    phase[i] = (float)(r & 0xFFFFu) * (6.2831853f / 65536.0f);
  }
  // each angle: a steady turn, and two slow swings on top of it that speed it up, hold it back and at times
  // turn it round (in radians, with turnMs in milliseconds: a swing takes 25 to 50 seconds)
  const float more[3] = {shove ? shove[0] : 0.0f, shove ? shove[1] : 0.0f, shove ? shove[2] : 0.0f};
  const float yaw = more[0] + 0.00030f * turnMs + 1.5f * sinf(0.00021f * turnMs + phase[0]) + 0.8f * sinf(0.00013f * turnMs + phase[1]);
  const float pitch = more[1] + 0.00022f * turnMs + 1.4f * sinf(0.00017f * turnMs + phase[2]) + 0.9f * sinf(0.00025f * turnMs + phase[3]);
  const float roll = more[2] + 0.00026f * turnMs + 1.3f * sinf(0.00019f * turnMs + phase[4]) + 0.9f * sinf(0.00015f * turnMs + phase[5]);
  const float cy = cosf(yaw), sy = sinf(yaw), cp = cosf(pitch), sp = sinf(pitch), cr = cosf(roll), sr = sinf(roll);
  const V3 turned[3] = {{cy, sy * sp, -sy * cp}, {0, cp, sp}, {sy, -cy * sp, cy * cp}};  // about the upright axis, then tipped
  for (int i = 0; i < 3; i++) out[i] = {turned[i].x * cr - turned[i].y * sr, turned[i].x * sr + turned[i].y * cr, turned[i].z};  // then rolled
}

inline void tumble(float turnMs, uint32_t seed, V3 out[3]) { tumble(turnMs, seed, nullptr, out); }

// A shove given to a tumbling cube: it spins off in some direction, fast at first, and that dies away in under
// a second while the tumbling goes on underneath.  `angle` is what all the shoves so far have turned the cube
// by, for tumble().
const float kShoveRate = 0.0045f;   // radians a millisecond at the moment of the shove: 260 degrees a second
const float kShoveFadeMs = 600.0f;  // ... falling to a third of what it was in every stretch of this long
struct Shove {
  float angle[3] = {0, 0, 0};
  float rate[3] = {0, 0, 0};  // how fast the angles still grow, radians a millisecond

  // One shove, in the direction that `random` (any 32 bits of chance) picks.
  void bump(uint32_t random) {
    float v[3], length2 = 0;
    for (int i = 0; i < 3; i++) {
      v[i] = (float)((random >> (10 * i)) & 1023u) - 511.5f;
      length2 += v[i] * v[i];
    }
    const float scale = kShoveRate / sqrtf(length2);  // (never nought: the parts are halves)
    for (int i = 0; i < 3; i++) rate[i] += v[i] * scale;
  }

  // `dtMs` later (the same whether it is counted in one go or frame by frame).
  void advance(float dtMs) {
    const float keep = expf(-dtMs / kShoveFadeMs);
    for (int i = 0; i < 3; i++) {
      angle[i] = fmodf(angle[i] + rate[i] * kShoveFadeMs * (1.0f - keep), 6.2831853f);
      rate[i] *= keep;
    }
  }
};

// What can be seen through a face from the eye: the inside of the four planes through the eye and the face's edges.
struct Window {
  V3 eye;
  V3 normal[4];  // of those planes, pointing inwards

  // The part of the segment from p to p + d that is seen through the window, as the range t0..t1 of 0..1 along
  // it.  False if none of it is.
  bool clip(V3 p, V3 d, float *t0, float *t1) const {
    float lo = 0, hi = 1;
    const V3 rel = sub(p, eye);
    for (int i = 0; i < 4; i++) {
      const float at = dot(normal[i], rel);  // how far p is inside this plane (negative: outside)
      const float go = dot(normal[i], d);    // ... and how that changes along the segment
      if (go > -1e-6f && go < 1e-6f) {
        if (at < 0) return false;  // alongside the plane, outside it
        continue;
      }
      const float t = -at / go;
      if (go > 0) {
        if (t > lo) lo = t;  // comes in here
      } else {
        if (t < hi) hi = t;  // goes out here
      }
      if (lo >= hi) return false;
    }
    *t0 = lo;
    *t1 = hi;
    return true;
  }
};

// Hands the picture of the cube with its axes at `axis` (tumble()), `ms` after it came up (the pulse goes by
// that), for a screen of w x h pixels, to `sink`:
//   sink.line(x0, y0, x1, y1, style)   the lattice behind each face first, the cube's own edges (BOLD) last, so
//                                      that they are drawn over the ends of what is behind them
//   sink.dot(x, y)                     a glint on every corner that is seen
template <class Sink>
void draw(const V3 axis[3], uint32_t ms, int w, int h, Sink &sink) {
  const V3 eye = {0, 0, -kCamera};
  // A corner is sqrt(3) from the centre, and wherever the cube has turned to it shows no further from the middle
  // of the screen than focal * sqrt(3) / sqrt(kCamera^2 - 3): the focal length makes that kFill of half the height.
  const float focal = kFill * 0.5f * (float)h * sqrtf(kCamera * kCamera - 3.0f) / 1.7320508f;
  const float midX = 0.5f * (float)(w - 1), midY = 0.5f * (float)(h - 1);
  auto line = [&](V3 a, V3 b, Style style) {
    const float ka = focal / (kCamera + a.z), kb = focal / (kCamera + b.z);
    const int x0 = (int)lroundf(midX + a.x * ka), y0 = (int)lroundf(midY - a.y * ka);
    const int x1 = (int)lroundf(midX + b.x * kb), y1 = (int)lroundf(midY - b.y * kb);
    if (x0 == x1 && y0 == y1 && style != BOLD) return;  // nothing to see of it
    sink.line(x0, y0, x1, y1, style);
  };
  const int pulse = pulseAt(ms);
  const int reach = 2 * kSide + 1;  // the lattice's lines stand at the odd places from -reach to reach

  V3 corners[3][4];  // of the faces that are seen, for their own edges at the end
  int seen = 0;
  for (int f = 0; f < 6 && seen < 3; f++) {
    const float side = (f & 1) ? -1.0f : 1.0f;
    const V3 n = mul(axis[f >> 1], side);  // the face's centre, and the way it looks
    const V3 u = axis[((f >> 1) + 1) % 3], v = axis[((f >> 1) + 2) % 3];
    if (n.z > -1.0f / kCamera - 0.02f) continue;  // it looks away from the eye (or is seen all but edge on)
    // a place in the lattice as seen through this face: `depth` half edges behind it, a and b along its two sides
    auto at = [&](float depth, float a, float b) { return add(mul(n, 1.0f - depth), add(mul(u, a), mul(v, b))); };
    V3 *c = corners[seen++];
    c[0] = at(0, 1, 1);
    c[1] = at(0, -1, 1);
    c[2] = at(0, -1, -1);
    c[3] = at(0, 1, -1);
    Window win;
    win.eye = eye;
    for (int i = 0; i < 4; i++) {
      V3 m = cross(sub(c[i], eye), sub(c[(i + 1) & 3], eye));
      if (dot(m, sub(n, eye)) < 0) m = mul(m, -1.0f);  // (inwards: the middle of the face is inside)
      win.normal[i] = m;
    }
    // A piece of a lattice line from p to p + d, of which the part t0..t1 is seen: in `cells` equal lengths, the
    // i-th of them bounces(i) reflections away.
    auto pieces = [&](V3 p, V3 d, int cells, auto bounces) {
      float t0, t1;
      if (!win.clip(p, d, &t0, &t1)) return;
      for (int i = (int)floorf(t0 * (float)cells); i < cells; i++) {
        const float from = (float)i / (float)cells, to = (float)(i + 1) / (float)cells;
        if (from >= t1) break;
        const Style style = bounces(i);
        if (style == NONE) continue;
        const float a = from > t0 ? from : t0, b = to < t1 ? to : t1;
        if (b > a) line(add(p, mul(d, a)), add(p, mul(d, b)), style);
      }
    };
    auto away = [](int place) { return (abs(place) - 1) / 2; };  // cubes to the side of the cube's own: 0, 1, 2 ...
    // the rails: the lines that run away from the eye, through every corner of every cube of the lattice
    for (int a = -reach; a <= reach; a += 2) {
      for (int b = -reach; b <= reach; b += 2) {
        const int sideways = kSideCost * (away(a) + away(b));
        pieces(at(0, (float)a, (float)b), mul(n, -2.0f * (float)kDepth), kDepth, [&](int i) { return fade(i + 1 + sideways); });
      }
    }
    // the frames: at every cube's length of depth, the lines across, both ways
    for (int k = 1; k <= kDepth; k++) {
      const float depth = 2.0f * (float)k;
      for (int m = -reach; m <= reach; m += 2) {
        // (cell i of such a line lies between the places -reach + 2 i and -reach + 2 i + 2: i - kSide cubes to the side)
        auto bounces = [&](int i) {
          const int sideways = kSideCost * (away(m) + abs(i - kSide));
          return (k == pulse && sideways == 0) ? SOLID : fade(k + sideways);
        };
        pieces(at(depth, (float)-reach, (float)m), mul(u, 2.0f * (float)reach), reach, bounces);
        pieces(at(depth, (float)m, (float)-reach), mul(v, 2.0f * (float)reach), reach, bounces);
      }
    }
  }
  for (int f = 0; f < seen; f++)
    for (int i = 0; i < 4; i++) line(corners[f][i], corners[f][(i + 1) & 3], BOLD);
  for (int f = 0; f < seen; f++) {
    for (int i = 0; i < 4; i++) {
      const float k = focal / (kCamera + corners[f][i].z);
      sink.dot((int)lroundf(midX + corners[f][i].x * k), (int)lroundf(midY - corners[f][i].y * k));
    }
  }
}

}  // namespace infcube
