#pragma once

// Moon phase from the date alone: no network, no table.  The angle between the Sun and the
// Moon comes from the main terms of the standard series for both longitudes (Meeus, "Astronomical
// Algorithms", ch. 47, and the Astronomical Almanac's low-precision Sun).  Checked against 24
// published new and full moons and four quarters (tools/tests): the darkest and brightest moments
// land within half an hour of the almanac, and the lit fraction is good to a fraction of a
// percent, far finer than the 10 pixel disc it is drawn in.  Header-only and free of Arduino
// dependencies so tools/tests can check it on a PC.

#include <math.h>
#include <stdint.h>

namespace moon {

struct Phase {
  float lit = 0;        // 0..1, the fraction of the disc that is lit
  bool waxing = true;   // growing (new -> full); false when shrinking (full -> new)
  float age = 0;        // 0..1 round the lunation: 0 new, 0.25 first quarter, 0.5 full, 0.75 last quarter
};

inline Phase phaseAt(int64_t unixUtc) {
  const double kRad = 3.14159265358979323846 / 180.0;
  const double d = (double)unixUtc / 86400.0 + 2440587.5 - 2451545.0;  // days since J2000.0

  // Sun: apparent ecliptic longitude, degrees
  const double sunMean = 280.460 + 0.9856474 * d;
  const double g = (357.528 + 0.9856003 * d) * kRad;
  const double sunLon = sunMean + 1.915 * sin(g) + 0.020 * sin(2 * g);

  // Moon: mean longitude, elongation, Sun's and Moon's anomaly, argument of latitude
  const double lMoon = 218.3164477 + 13.17639648 * d;
  const double D = (297.8501921 + 12.19074912 * d) * kRad;
  const double M = (357.5291092 + 0.98560028 * d) * kRad;
  const double Mp = (134.9633964 + 13.06499295 * d) * kRad;
  const double F = (93.2720950 + 13.22935025 * d) * kRad;
  const double moonLon = lMoon + 6.288774 * sin(Mp) + 1.274027 * sin(2 * D - Mp) + 0.658314 * sin(2 * D) +
                         0.213618 * sin(2 * Mp) - 0.185116 * sin(M) - 0.114332 * sin(2 * F) +
                         0.058793 * sin(2 * D - 2 * Mp) + 0.057066 * sin(2 * D - M - Mp) +
                         0.053322 * sin(2 * D + Mp) + 0.045758 * sin(2 * D - M) - 0.040923 * sin(M - Mp) -
                         0.034720 * sin(D) - 0.030383 * sin(M + Mp) + 0.015327 * sin(2 * D - 2 * F);
  const double moonLat = 5.128122 * sin(F) + 0.280602 * sin(Mp + F) + 0.277693 * sin(Mp - F) +
                         0.173237 * sin(2 * D - F) + 0.055413 * sin(2 * D - Mp + F) +
                         0.046271 * sin(2 * D - Mp - F) + 0.032573 * sin(2 * D + F);

  double dLon = fmod(moonLon - sunLon, 360.0);
  if (dLon < 0) dLon += 360.0;
  // elongation: the angle between the two bodies in the sky
  const double cosPsi = cos(moonLat * kRad) * cos(dLon * kRad);
  double psi = acos(cosPsi < -1 ? -1 : (cosPsi > 1 ? 1 : cosPsi));  // radians, 0..pi
  psi += 0.1468 * kRad * sin(psi);  // the Sun is not infinitely far: the terminator is a touch later

  Phase p;
  p.lit = (float)((1.0 - cos(psi)) / 2.0);
  p.waxing = dLon < 180.0;
  p.age = (float)(dLon / 360.0);
  return p;
}

// One pixel of a moon disc of `diameter` pixels, for drawing the phase.  `col` and `row` count
// from the top-left of the disc's bounding box.  A moon is lit on its right side while it grows
// when seen from the northern hemisphere; from the southern hemisphere the picture is mirrored
// (litOnRight = waxing != southern).  The disc is round, the terminator an ellipse.
struct DiscPixel {
  bool inside = false;
  bool lit = false;
};

inline DiscPixel discPixel(int col, int row, int diameter, float lit, bool litOnRight) {
  DiscPixel px;
  const float centre = (diameter - 1) * 0.5f;
  const float x = (float)col - centre, y = (float)row - centre;
  const float r = diameter * 0.5f;
  const float r2 = r * r + 1.0f;  // a little over r: small discs look rounder
  if (x * x + y * y > r2) return px;
  px.inside = true;
  if (lit <= 0.005f) return px;
  if (lit >= 0.995f) {
    px.lit = true;
    return px;
  }
  const float halfChord = sqrtf(r2 - y * y);
  const float terminator = (1.0f - 2.0f * lit) * halfChord;  // +chord at new moon, -chord at full
  px.lit = litOnRight ? (x > terminator) : (-x > terminator);
  return px;
}

}  // namespace moon
