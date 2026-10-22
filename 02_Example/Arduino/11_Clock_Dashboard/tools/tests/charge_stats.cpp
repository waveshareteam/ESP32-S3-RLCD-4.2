// How well does charge.h do?  Detection latency and false-event rates over many random
// voltage traces, which is where the figures in the README's Battery section and the
// thresholds in charge.h come from.  Not part of run_tests.sh (it takes a minute).
//
//   bash run_charge_stats.sh [sequences]            all scenarios, 300 sequences each by default
//   bash run_charge_stats.sh 300 <n> <seed>         trace scenario [n] on one sequence: the detector's
//                                                   internals once a minute, and the bins at the moment
//                                                   it first says CHARGING (for finding out why)
//
// The traces are what a battery plausibly does: a reading every 5 s with Gaussian noise
// (4 mV), one reading in ten pulled down 20-70 mV as if a WiFi burst hit the ADC, and a slow
// 1.5 mV wander.  The scenarios (steps when USB goes in or out, slow charging and discharging,
// a charger holding 4.2 V, level shifts from a change in load, a start-up sag) are guesses
// about the board, not measurements of it; the point is to see how the detector copes.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <vector>

// The trace output looks inside the detector.
#define private public
#include "../../charge.h"
#undef private

using CS = ChargeDetector;

// Plain least-squares slope of the detector's (lifted) bins [from, to) in mV per minute.
static double plainSlope(const CS &d, int from, int to) {
  const int n = to - from;
  double mean = 0;
  for (int i = from; i < to; i++) mean += d.lifted(i);
  mean /= n;
  const double xm = 0.5 * (n - 1);
  double sxy = 0, sxx = 0;
  for (int i = 0; i < n; i++) {
    sxy += (i - xm) * (d.lifted(from + i) - mean);
    sxx += (i - xm) * (i - xm);
  }
  return sxy / sxx * 4.0 * 1000.0;  // four 15 s bins to the minute
}

struct Sim {
  CS d;
  uint32_t t = 0;  // ms since the trace began
  uint64_t rng;
  double wander;
  unsigned seen = 0;                     // bit per State seen since clearSeen()
  double firstAt[4] = {-1, -1, -1, -1};  // minute each state was first seen since clearSeen()
  int changes = 0;                       // state changes since clearSeen()
  CS::State last = CS::UNKNOWN;
  double noiseMv = 4.0, wanderMv = 1.5, dipProb = 0.10;
  bool trace = false;
  uint32_t nextPrint = 0;

  explicit Sim(uint64_t seed) : rng(seed * 0x9E3779B97F4A7C15ull + 12345) { wander = uniform() * 6.2831853; }
  double uniform() {
    rng = rng * 6364136223846793005ull + 1442695040888963407ull;
    return (double)(rng >> 11) / 9007199254740992.0;
  }
  double gauss() {
    const double u1 = uniform() + 1e-12, u2 = uniform();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
  }
  void clearSeen() {
    seen = 0;
    changes = 0;
    last = d.state();
    for (double &f : firstAt) f = -1;
  }
  double now() const { return t / 60000.0; }

  // Runs `minutes` of a trace whose true battery voltage is truth(minutes since the start).
  template <class F>
  void run(double minutes, F truth) {
    const uint32_t end = t + (uint32_t)(minutes * 60000.0 + 0.5);
    while (t < end) {
      const double m = t / 60000.0;
      double v = truth(m) + noiseMv / 1000.0 * gauss() + wanderMv / 1000.0 * sin(m * 1.26 + wander);
      if (uniform() < dipProb) v -= 0.020 + 0.050 * uniform();
      d.addSample((float)v, t);
      const CS::State s = d.state();
      seen |= 1u << s;
      if (firstAt[s] < 0) firstAt[s] = m;
      if (trace) printTrace(m, s, truth(m));
      if (s != last) {
        changes++;
        last = s;
      }
      t += 4500 + (uint32_t)(uniform() * 1000.0);  // readings 5 s apart, give or take
    }
  }

  void printTrace(double m, CS::State s, double truth) {
    static const char *names[] = {"UNKNOWN", "DISCHARGING", "CHARGING", "FULL"};
    if (s == CS::CHARGING && last != CS::CHARGING) {
      const int n = d.bins_;
      printf("--- bins at the flip to CHARGING (t=%.2f min, n=%d), oldest first; mV above 3.80 V (bin/lifted) ---\n", m, n);
      for (int i = 0; i < n; i++)
        printf("%s%5.1f/%5.1f%s", i % 8 == 0 ? "  " : " ", (d.means_[i] - 3.80f) * 1000, (d.lifted(i) - 3.80f) * 1000,
               i % 8 == 7 ? "\n" : "");
      printf("\n  detector slope %+.2f mV/min, shift-tolerant slope %+.2f mV/min\n", d.slope_ * 1000,
             d.shiftTolerantSlope(n) * 1000);
    }
    if (t >= nextPrint) {
      nextPrint = t + 60000;
      const int n = d.bins_;
      const float level = n >= 4 ? (d.lifted(n - 1) + d.lifted(n - 2) + d.lifted(n - 3) + d.lifted(n - 4)) * 0.25f : 0;
      printf("t=%7.2f min  truth=%.4f level=%.4f  trend=%+6.2f  plainLS=%+6.2f  step=%+6.1f mV  fullRef=%.4f  slow=%.4f blk=%d  %s\n", m, truth,
             level, d.slope_ * 1000, n >= 2 ? plainSlope(d, 0, n) : 0.0, d.step_ * 1000, d.fullRef_,
             d.slowCount_ ? d.slow_[d.slowCount_ - 1] : 0.0f, d.fullBlocked_ ? 1 : 0, names[s]);
    }
  }
};

static int N = 300;
static int g_only = -1, g_idx = 0;
static uint64_t g_traceSeed = 0;

// Latency report: fn runs one sequence and returns minutes until the event, or < 0 if it never happened.
static void latency(const char *name, std::function<double(Sim &)> fn) {
  const int idx = g_idx++;
  if (g_only >= 0 && idx != g_only) return;
  std::vector<double> v;
  int miss = 0;
  for (int i = 1; i <= N; i++) {
    if (g_traceSeed && (uint64_t)i != g_traceSeed) continue;
    Sim s(i);
    s.trace = g_traceSeed != 0;
    const double r = fn(s);
    if (r < 0) {
      miss++;
    } else {
      v.push_back(r);
    }
  }
  std::sort(v.begin(), v.end());
  if (v.empty()) {
    printf("[%d] %-58s never detected (%d of %d)\n", idx, name, miss, N);
    return;
  }
  printf("[%d] %-58s median %5.1f  p90 %5.1f  max %5.1f min   missed %d of %d\n", idx, name, v[v.size() / 2],
         v[v.size() * 9 / 10], v.back(), miss, N);
}

// Rate report: fn returns how many wrong events it saw in one sequence.
static void rate(const char *name, const char *unit, std::function<int(Sim &)> fn) {
  const int idx = g_idx++;
  if (g_only >= 0 && idx != g_only) return;
  int total = 0, seqs = 0;
  std::vector<int> bad;
  for (int i = 1; i <= N; i++) {
    if (g_traceSeed && (uint64_t)i != g_traceSeed) continue;
    Sim s(1000 + i);
    s.trace = g_traceSeed != 0;
    const int r = fn(s);
    total += r;
    if (r) {
      seqs++;
      if (bad.size() < 6) bad.push_back(i);
    }
  }
  printf("[%d] %-58s %d events, in %d of %d sequences  (%s)", idx, name, total, seqs, N, unit);
  if (!bad.empty()) {
    printf("  first failing i:");
    for (int b : bad) printf(" %d", b);
  }
  printf("\n");
}

// The load is heavier for a minute or so after power-up (WiFi join, TLS fetches), then eases.
static double startupSag(double minutes, double sagMv) {
  return minutes < 1.0 ? sagMv : (minutes < 2.5 ? sagMv * (2.5 - minutes) / 1.5 : 0.0);
}

// How a clock comes to FULL: a charge is seen, from 4.10 V at 3 mV a minute up to a plateau of 4.19 V, which the
// charger then holds (an hour in all; the detector says FULL some twelve minutes after the voltage stops).  A cell
// that is merely flat near the top when the clock starts is not called full for 160 minutes.
static bool chargeToFull(Sim &s) {
  const double t0 = s.now();
  s.run(60, [t0](double m) {
    const double x = 4.10 + 0.003 * (m - t0);
    return x > 4.19 ? 4.19 : x;
  });
  return s.d.state() == CS::FULL;
}

int main(int argc, char **argv) {
  if (argc > 1) N = atoi(argv[1]);
  if (argc > 2) g_only = atoi(argv[2]);
  if (argc > 3) g_traceSeed = strtoull(argv[3], nullptr, 10);
  const CS::State DIS = CS::DISCHARGING, CHG = CS::CHARGING, FUL = CS::FULL;
  const unsigned powerBits = (1u << CHG) | (1u << FUL);

  printf("== latency: cable events ==\n");
  for (double stepMv : {90.0, 60.0, 45.0}) {
    char name[96];
    snprintf(name, sizeof name, "plug-in while discharging, +%.0f mV step, then +2.5 mV/min", stepMv);
    latency(name, [stepMv](Sim &s) {
      s.run(20, [](double m) { return 3.83 - 0.0006 * m; });
      const double t0 = s.now();
      s.clearSeen();
      s.run(30, [=](double m) { return 3.83 - 0.0006 * t0 + stepMv / 1000 + 0.0025 * (m - t0); });
      return s.firstAt[CHG] < 0 ? -1 : s.firstAt[CHG] - t0;
    });
  }
  for (double stepMv : {30.0, 20.0, 12.0}) {
    char name[96];
    snprintf(name, sizeof name, "plug-in while discharging, small +%.0f mV step, +2 mV/min", stepMv);
    latency(name, [stepMv](Sim &s) {
      s.run(20, [](double m) { return 3.80 - 0.0006 * m; });
      const double t0 = s.now();
      s.clearSeen();
      s.run(40, [=](double m) { return 3.80 - 0.0006 * t0 + stepMv / 1000 + 0.002 * (m - t0); });
      return s.firstAt[CHG] < 0 ? -1 : s.firstAt[CHG] - t0;
    });
  }
  for (double stepMv : {80.0, 50.0, 25.0}) {
    char name[96];
    snprintf(name, sizeof name, "unplug while charging, -%.0f mV step, then -0.6 mV/min", stepMv);
    latency(name, [stepMv](Sim &s) {
      s.run(30, [](double m) { return 3.90 + 0.0025 * m; });
      const double t0 = s.now();
      s.clearSeen();
      s.run(30, [=](double m) { return 3.90 + 0.0025 * t0 - stepMv / 1000 - 0.0006 * (m - t0); });
      return s.firstAt[DIS] < 0 ? -1 : s.firstAt[DIS] - t0;
    });
  }

  printf("== latency: after boot (includes the 3 minute warm-up) ==\n");
  for (double r : {3.0, 2.0, 1.5, 1.0}) {
    char name[96];
    snprintf(name, sizeof name, "booted while charging at +%.1f mV/min -> CHARGING", r);
    latency(name, [r](Sim &s) {
      s.run(40, [r](double m) { return 3.80 + r / 1000 * m; });
      return s.firstAt[CHG];
    });
  }
  for (double r : {1.5, 1.0, 0.6, 0.4, 0.3}) {
    char name[96];
    snprintf(name, sizeof name, "booted on battery at -%.1f mV/min -> DISCHARGING", r);
    latency(name, [r](Sim &s) {
      s.run(40, [r](double m) { return 3.85 - r / 1000 * m; });
      return s.firstAt[DIS];
    });
  }
  for (double r : {0.20, 0.10, 0.05}) {  // a light load in the middle of the curve: the slope is tiny
    char name[96];
    snprintf(name, sizeof name, "booted on battery at -%.2f mV/min, light load -> DISCHARGING", r);
    latency(name, [r](Sim &s) {
      s.run(900, [r](double m) { return 3.85 - r / 1000 * m; });
      return s.firstAt[DIS];
    });
  }
  for (double r : {0.15, 0.07}) {  // booted on battery near the top: a flat-looking fall that may first read as FULL
    char name[96];
    snprintf(name, sizeof name, "booted on battery at 4.17 V falling %.2f mV/min -> DISCHARGING", r);
    latency(name, [r](Sim &s) {
      s.run(900, [r](double m) { return 4.17 - r / 1000 * m; });
      return s.firstAt[DIS];
    });
  }
  // (no charge was seen, so twelve flat minutes are not enough: the slow history has to show 160 minutes without a fall)
  latency("booted at a flat 4.18 V (on a charger, full) -> FULL", [](Sim &s) {
    s.run(300, [](double) { return 4.18; });
    return s.firstAt[FUL];
  });
  latency("booted at a flat 4.20 V (on a charger, full) -> FULL", [](Sim &s) {
    s.run(300, [](double) { return 4.20; });
    return s.firstAt[FUL];
  });
  latency("booted on a charger, full, cell still relaxing (8 mV, tau 30 min) -> FULL", [](Sim &s) {
    s.run(480, [](double m) { return 4.19 + 0.008 * exp(-m / 30.0); });
    return s.firstAt[FUL];
  });
  latency("charging reaches 4.2 V at minute 33 -> FULL (minutes after)", [](Sim &s) {
    s.run(90, [](double m) {
      const double x = 4.10 + 0.003 * m;
      return x > 4.2 ? 4.2 : x;
    });
    return s.firstAt[FUL] < 0 ? -1 : s.firstAt[FUL] - 33.0;
  });

  printf("== latency: unplugging a full battery (no step; plateau 4.19 V) ==\n");
  for (double r : {2.5, 1.7, 1.2, 0.8, 0.5}) {
    char name[96];
    snprintf(name, sizeof name, "unplugged when full, falls at %.1f mV/min -> DISCHARGING", r);
    latency(name, [r](Sim &s) {
      chargeToFull(s);
      const double t0 = s.now();
      s.clearSeen();
      s.run(80, [=](double m) { return 4.19 - r / 1000 * (m - t0); });
      return s.firstAt[DIS] < 0 ? -1 : s.firstAt[DIS] - t0;
    });
  }
  // A light load (the clock with the radio mostly off draws 10-45 mA from a 2500 mAh cell) makes the top of
  // the curve fall by 0.07-0.3 mV a minute: the case that sat on FULL for hours with the first version.
  for (double r : {0.40, 0.30, 0.20, 0.12, 0.07}) {
    char name[96];
    snprintf(name, sizeof name, "unplugged when full, light load, falls at %.2f mV/min -> DISCHARGING", r);
    latency(name, [r](Sim &s) {
      chargeToFull(s);
      const double t0 = s.now();
      s.clearSeen();
      s.run(600, [=](double m) { return 4.19 - r / 1000 * (m - t0); });
      return s.firstAt[DIS] < 0 ? -1 : s.firstAt[DIS] - t0;
    });
  }
  for (double r : {0.30, 0.12, 0.07}) {
    char name[96];
    snprintf(name, sizeof name, "unplugged when full with a 7 mV IR step, light load, falls at %.2f mV/min", r);
    latency(name, [r](Sim &s) {
      chargeToFull(s);
      const double t0 = s.now();
      s.clearSeen();
      s.run(600, [=](double m) { return 4.19 - 0.007 - r / 1000 * (m - t0); });
      return s.firstAt[DIS] < 0 ? -1 : s.firstAt[DIS] - t0;
    });
  }
  for (double drop : {40.0, 25.0}) {
    char name[96];
    snprintf(name, sizeof name, "unplugged when full with a %.0f mV IR step, then -1 mV/min", drop);
    latency(name, [drop](Sim &s) {
      chargeToFull(s);
      const double t0 = s.now();
      s.clearSeen();
      s.run(60, [=](double m) { return 4.19 - drop / 1000 - 0.001 * (m - t0); });
      return s.firstAt[DIS] < 0 ? -1 : s.firstAt[DIS] - t0;
    });
  }

  printf("== false events over 8 hours ==\n");
  rate("on battery -0.5 mV/min: CHARGING or FULL claimed", "should be 0", [=](Sim &s) {
    s.run(480, [](double m) { return 4.05 - 0.0005 * m; });
    return (s.seen & powerBits) ? 1 : 0;
  });
  rate("on battery -0.3 mV/min: CHARGING or FULL claimed", "should be 0", [=](Sim &s) {
    s.run(480, [](double m) { return 3.90 - 0.0003 * m; });
    return (s.seen & powerBits) ? 1 : 0;
  });
  rate("on battery -0.07 mV/min, light load: CHARGING or FULL claimed", "should be 0", [=](Sim &s) {
    s.run(480, [](double m) { return 3.95 - 0.00007 * m; });
    return (s.seen & powerBits) ? 1 : 0;
  });
  rate("pure noise at 3.70 V: CHARGING or FULL claimed", "should be 0", [=](Sim &s) {
    s.run(480, [](double) { return 3.70; });
    return (s.seen & powerBits) ? 1 : 0;
  });
  rate("on battery with a random 15 mV level shift every ~40 min: CHARGING", "about 1 in 60 shifts", [&](Sim &s) {
    double level = 0, next = 40;
    s.run(480, [&](double m) {
      if (m > next) {
        level += (s.uniform() < 0.5 ? 1 : -1) * 0.015;
        next += 20 + 40 * s.uniform();
      }
      return 3.95 - 0.0005 * m + level;
    });
    return (s.seen & (1u << CHG)) ? 1 : 0;
  });
  rate("held full at 4.19 V: state changes after reaching FULL", "should be 0", [&](Sim &s) {
    chargeToFull(s);
    s.clearSeen();
    s.run(480, [](double) { return 4.19; });
    return s.changes;
  });
  // After the charger ends the cell relaxes a few millivolts over some minutes: that is not a discharge.
  for (double tau : {15.0, 30.0}) {
    char name[96];
    snprintf(name, sizeof name, "held full after the charger ends, 8 mV tail (tau %.0f min): changes", tau);
    rate(name, "should be 0", [tau](Sim &s) {
      const double top = (4.198 - 4.10) / 0.003;  // the minute the charge reaches its end and the cell starts to relax
      auto f = [=](double m) { return m < top ? 4.10 + 0.003 * m : 4.19 + 0.008 * exp(-(m - top) / tau); };
      s.run(top + 3 * tau + 25, f);  // (called FULL once the tail has flattened)
      const bool full = s.d.state() == CS::FULL;
      s.clearSeen();
      s.run(480, f);
      return s.changes + (full ? 0 : 100);  // 100 = it was not FULL to begin with
    });
  }
  // A clock started on its battery with the cell nearly full: flat for twelve minutes, and not held by anything.
  for (double r : {0.12, 0.07, 0.05, 0.03}) {
    char name[96];
    snprintf(name, sizeof name, "booted on battery at 4.19 V falling %.2f mV/min, 16 h: FULL or CHARGING", r);
    rate(name, "should be 0", [r, powerBits](Sim &s) {
      s.run(960, [r](double m) { return 4.19 - r / 1000 * m; });
      return (s.seen & powerBits) ? 1 : 0;
    });
  }
  rate("booted on battery at 4.19 V falling 0.02 mV/min (a month's drain), 16 h: FULL", "informational", [FUL](Sim &s) {
    s.run(960, [](double m) { return 4.19 - 0.00002 * m; });
    return (s.seen & (1u << FUL)) ? 1 : 0;
  });
  for (double dipMv : {13.0, 25.0}) {  // a cable knocked loose and pushed back: four minutes low, every hour or so
    char name[96];
    snprintf(name, sizeof name, "held full with a four minute dip of %.0f mV every hour or so: changes", dipMv);
    rate(name, "informational: the twelve minute trend says discharging for a while", [dipMv](Sim &s) {
      chargeToFull(s);
      s.clearSeen();
      double dipAt = 20 + 50 * s.uniform();
      s.run(480, [&](double m) {
        if (m > dipAt + 4) dipAt = m + 40 + 50 * s.uniform();
        return 4.19 - ((m >= dipAt && m < dipAt + 4) ? dipMv / 1000 : 0.0);
      });
      return s.changes;
    });
  }
  rate("held full with random level shifts within +-4 mV every ~40 min: changes", "should be 0", [&](Sim &s) {
    chargeToFull(s);
    s.clearSeen();
    double level = 0, next = 40;
    s.run(480, [&](double m) {
      if (m > next) {
        level = (s.uniform() < 0.5 ? 1 : -1) * 0.004 * s.uniform();
        next += 20 + 40 * s.uniform();
      }
      return 4.19 + level;
    });
    return s.changes;
  });
  rate("held full with random level shifts within +-8 mV every ~40 min: changes", "informational: may read as a fall", [&](Sim &s) {
    chargeToFull(s);
    s.clearSeen();
    double level = 0, next = 40;
    s.run(480, [&](double m) {
      if (m > next) {
        level = (s.uniform() < 0.5 ? 1 : -1) * 0.008 * s.uniform();
        next += 20 + 40 * s.uniform();
      }
      return 4.19 + level;
    });
    return s.changes + ((s.seen & (1u << CHG)) ? 1000 : 0);  // 1000 = it also said CHARGING
  });
  for (double sagMv : {30.0, 60.0, 100.0}) {
    char name[96];
    snprintf(name, sizeof name, "boot on battery with a %.0f mV start-up sag: CHARGING", sagMv);
    rate(name, "should be 0", [sagMv, powerBits](Sim &s) {
      s.run(30, [sagMv](double m) { return 3.90 - 0.0006 * m - startupSag(m, sagMv) / 1000.0; });
      return (s.seen & (1u << CHG)) ? 1 : 0;
    });
  }
  for (double sagMv : {30.0, 60.0, 100.0}) {
    char name[96];
    snprintf(name, sizeof name, "boot on USB (4.19 V full) with a %.0f mV start-up sag: DISCHARGING", sagMv);
    rate(name, "should be 0", [sagMv, DIS](Sim &s) {
      s.run(30, [sagMv](double m) { return 4.19 - startupSag(m, sagMv) / 1000.0; });
      return (s.seen & (1u << DIS)) ? 1 : 0;
    });
  }
  rate("plugged in when nearly full (+60 mV, capped 4.2 V): changes beyond DIS->CHG->FULL", "should be 0", [&](Sim &s) {
    s.run(20, [](double m) { return 4.14 - 0.001 * m; });
    const double t0 = s.now();
    s.clearSeen();
    s.run(25, [t0](double m) {
      const double x = 4.12 + 0.060 + 0.004 * (m - t0);
      return x > 4.2 ? 4.2 : x;
    });
    return s.changes > 2 ? 1 : 0;
  });
  rate("charging +2 mV/min then flat 4.2 V: state changes beyond CHG->FULL", "should be 0", [&](Sim &s) {
    s.run(30, [](double m) { return 4.12 + 0.002 * m; });
    s.clearSeen();
    s.run(480, [](double m) {
      const double x = 4.12 + 0.002 * (m + 30);
      return x > 4.2 ? 4.2 : x;
    });
    return s.changes > 1 ? 1 : 0;
  });
  return 0;
}
