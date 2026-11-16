// Simulates whole discharges to choose the constants of battery_est.h (the averaging window).
//
//   ./battery_sim            prints a table: window settings against several kinds of drain
//   ./battery_sim trace [n]  prints one discharge (scenario n) estimate by estimate against the truth
//
// The cell model: open-circuit voltage from a *smooth* curve through the clock's discharge table
// (the estimator only has the table, so the two differ a little, as a real cell and the table
// do), minus current x internal resistance, minus an optional "just came off the charger"
// settling offset that decays with a ten minute time constant; the ADC adds noise and an
// occasional WiFi transmit dip.  The truth is the time until the *loaded* voltage at the
// present current reaches the cut-off, which is what the estimate promises (it cannot know
// about future load changes).
//
// This is a model, not a measurement: it picks sensible constants and shows how the estimate
// behaves; the real cell will differ.  Build:  g++ -std=c++17 -O2 -I../.. battery_sim.cpp -o battery_sim
//
// What the sweep showed (median / 90th percentile of |estimate - truth| / truth, estimates from
// one hour after the start, six noise seeds), the chosen setting being "2h..8h 20 %": the window
// is the span in which the cell loses about 20 %, but never under 2 h or over 8 h.
//
//                                  fixed 30 min   fixed 2 h   fixed 8 h   2h..8h 20 % (chosen)
//   35 mA, 2500 mAh from 95 %        17% / 57%    12% / 44%    7% / 27%       7% / 27%
//   35 mA, 1000 mAh from 80 %        16% / 89%    12% / 57%    7% / 14%       7% / 21%
//   35 mA,  300 mAh from 95 %        11% / 57%     5% / 14%    9% / 16%       5% / 14%
//   90 mA, 1000 mAh from 90 %        25% / 83%    16% / 37%   25% / 57%      16% / 37%
//   8 mA, 2500 mAh (sleeping)        33% / 100%    9% / 20%    6% / 11%       6% / 11%
//
// Short windows chase the noise of the nearly flat middle of the discharge curve (a few mV there
// are several percent) and a window of a fixed length is wrong for either a fast or a slow drain;
// letting the window follow the drain does well for both.  With a load that changes every couple
// of hours every window does badly (the truth moves under it), which cannot be helped.
//
// That table is from the generic discharge curve of 1.0 to 1.9.  With the curve measured on the owner's cell (1.10),
// which is far less flat in the middle, every window does better and the chosen one stays among the best: in the
// steady drains a median of 2 to 8 % and a 90th percentile of 5 to 20 % (run ./battery_sim for the whole table).

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "battery_est.h"
#include "calc.h"

namespace {

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 2654435761u + 88172645463325252ull) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return (uint32_t)(s >> 11);
  }
};

double uni(Rng &r) { return ((r.next() & 0xFFFFFF) + 0.5) / 16777216.0; }
double gauss(Rng &r) { return sqrt(-2.0 * log(uni(r))) * cos(6.283185307179586 * uni(r)); }

// ---------------------------------------------------------------------------
// The cell: a smooth monotone curve through the table's points (Fritsch-Carlson cubic)
// ---------------------------------------------------------------------------
struct SmoothCurve {
  std::vector<double> x, y, m;  // percent, volts, slopes
  SmoothCurve() {
    int n;
    const calc::BatteryPoint *c = calc::batteryCurve(&n);
    for (int i = n - 1; i >= 0; i--) {
      x.push_back(c[i].pct);
      y.push_back(c[i].v);
    }
    const size_t k = x.size();
    std::vector<double> d(k - 1), mm(k);
    for (size_t i = 0; i + 1 < k; i++) d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    mm[0] = d[0];
    mm[k - 1] = d[k - 2];
    for (size_t i = 1; i + 1 < k; i++) mm[i] = (d[i - 1] * d[i] <= 0) ? 0 : 2 * d[i - 1] * d[i] / (d[i - 1] + d[i]);
    for (size_t i = 0; i + 1 < k; i++) {  // keep it monotone
      if (d[i] == 0) {
        mm[i] = mm[i + 1] = 0;
        continue;
      }
      const double a = mm[i] / d[i], b = mm[i + 1] / d[i], s = a * a + b * b;
      if (s > 9) {
        const double t = 3 / sqrt(s);
        mm[i] = t * a * d[i];
        mm[i + 1] = t * b * d[i];
      }
    }
    m = mm;
  }
  double volts(double pct) const {
    if (pct <= x.front()) return y.front();
    if (pct >= x.back()) return y.back();
    size_t i = 0;
    while (x[i + 1] < pct) i++;
    const double h = x[i + 1] - x[i], t = (pct - x[i]) / h;
    const double t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * y[i] + (t3 - 2 * t2 + t) * h * m[i] + (-2 * t3 + 3 * t2) * y[i + 1] +
           (t3 - t2) * h * m[i + 1];
  }
  // the charge at which the unloaded voltage is v
  double pctFor(double v) const {
    double lo = 0, hi = 100;
    for (int i = 0; i < 60; i++) {
      const double mid = 0.5 * (lo + hi);
      (volts(mid) < v ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
  }
};
const SmoothCurve &truthCurve() {
  static SmoothCurve c;
  return c;
}

struct Scenario {
  const char *name;
  double capacityMah;
  double startPct;
  double rintOhm;
  double settleMv;                        // offset just after leaving the charger
  std::function<double(double)> loadMa;   // current at time t (seconds)
  double maxHours;
  double truthMa = 0;                     // judge against this average current instead of the present one
};

const double kCutoffV = 3.30;

struct Score {
  std::vector<double> errors;  // relative |estimate - truth| / truth, per checkpoint
};

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  return v[(size_t)((v.size() - 1) * p)];
}

// Runs one discharge; every estimate (every 5 minutes) after `from` seconds is scored.
void run(const Scenario &sc, const battest::Params &params, uint64_t seed, double from, Score *out, bool trace) {
  Rng rng(seed);
  battest::Estimator est(params);
  const SmoothCurve &cell = truthCurve();
  double soc = sc.startPct;
  const double dt = 5.0;
  double nextCheck = 0;
  for (double t = 0; t < sc.maxHours * 3600.0; t += dt) {
    const double i = sc.loadMa(t);
    const double cut = cell.pctFor(kCutoffV + i / 1000.0 * sc.rintOhm);  // charge at which the loaded cell hits the cut-off
    if (soc <= cut + 0.3) break;
    soc -= i / sc.capacityMah * dt / 3600.0 * 100.0;
    double v = cell.volts(soc) - i / 1000.0 * sc.rintOhm;
    v += sc.settleMv / 1000.0 * exp(-t / 600.0);
    // the clock's ADC: 16 averaged samples, a few millivolts of noise; a transmit burst drags
    // an occasional reading down
    v += gauss(rng) * 0.003;
    if (uni(rng) < 0.01) v -= 0.04 + 0.08 * uni(rng);
    est.add((uint32_t)(1000 + t), (float)v);
    if (t >= nextCheck) {
      nextCheck = t + 300.0;
      const battest::Estimate e = est.estimate((float)kCutoffV, 0);
      if (e.state != battest::Estimate::READY || t < from) continue;
      const double judgedMa = sc.truthMa > 0 ? sc.truthMa : i;  // the load the promise is about
      const double judgedCut = cell.pctFor(kCutoffV + judgedMa / 1000.0 * sc.rintOhm);
      const double rate = judgedMa / sc.capacityMah * 100.0;  // percent per hour
      const double truth = (soc - judgedCut) / rate;          // hours
      if (truth < 0.25) continue;                      // the last quarter hour is not interesting
      if (e.unbounded) {
        out->errors.push_back(truth > 24 * 30 ? 0.0 : 1.0);
      } else {
        out->errors.push_back(fabs(e.hoursLeft - truth) / truth);
      }
      if (trace)
        printf("t %5.1f h  soc %5.1f %%  V %.3f  est %7.2f h (%5.2f %%/h over %3d min)  truth %7.2f h (%5.2f %%/h)  err %+6.1f %%\n",
               t / 3600.0, soc, v, e.hoursLeft, e.pctPerHour, e.windowMin, truth, rate, (e.hoursLeft - truth) / truth * 100.0);
    }
  }
}

std::vector<Scenario> scenarios() {
  std::vector<Scenario> s;
  auto steady = [](double ma) { return [ma](double) { return ma; }; };
  auto stepped = [](double a, double b, double every) {
    return [=](double t) { return fmod(t, 2 * every) < every ? a : b; };
  };
  s.push_back({"35 mA, 2500 mAh from 95 %", 2500, 95, 0.25, 0, steady(35), 40});
  s.push_back({"35 mA, 1000 mAh from 80 %", 1000, 80, 0.25, 0, steady(35), 40});
  s.push_back({"35 mA, 1000 mAh from 45 %", 1000, 45, 0.25, 0, steady(35), 40});
  s.push_back({"35 mA, 1000 mAh, R 0.5 ohm", 1000, 85, 0.50, 0, steady(35), 40});
  s.push_back({"35 mA,  300 mAh from 95 %", 300, 95, 0.25, 0, steady(35), 40});
  s.push_back({"35 mA,  100 mAh from 90 %", 100, 90, 0.25, 0, steady(35), 40});
  s.push_back({"90 mA, 1000 mAh from 90 %", 1000, 90, 0.25, 0, steady(90), 40});
  s.push_back({"35 mA,  300 mAh, off charger", 300, 100, 0.25, 40, steady(35), 40});
  s.push_back({"35 mA,  100 mAh, off charger", 100, 100, 0.25, 60, steady(35), 40});
  s.push_back({"8 mA, 2500 mAh (sleeping)", 2500, 95, 0.25, 0, steady(8), 60});
  s.push_back({"35/90 mA every 2 h, 1000 mAh", 1000, 90, 0.25, 0, stepped(35, 90, 7200), 40});
  s.push_back({"35/90 mA every 2 h, vs average", 1000, 90, 0.25, 0, stepped(35, 90, 7200), 40, 62.5});
  s.push_back({"35/90 mA every 40 min, 600 mAh", 600, 90, 0.25, 0, stepped(35, 90, 2400), 40});
  s.push_back({"35/90 mA every 40 min, vs avg", 600, 90, 0.25, 0, stepped(35, 90, 2400), 40, 62.5});
  return s;
}

}  // namespace

int main(int argc, char **argv) {
  const std::vector<Scenario> all = scenarios();
  if (argc > 1 && !strcmp(argv[1], "trace")) {
    const int which = argc > 2 ? atoi(argv[2]) : 3;
    battest::Params p;
    if (argc > 3) p.maxWindowSec = (uint32_t)atoi(argv[3]) * 60;
    Score sc;
    run(all[which], p, 1, 0, &sc, true);
    return 0;
  }

  struct Variant {
    const char *label;
    battest::Params p;
  };
  std::vector<Variant> variants;
  auto add = [&](const char *label, uint32_t minW, uint32_t maxW, float drop, uint32_t skip = 900) {
    battest::Params p;
    p.minWindowSec = minW;
    p.maxWindowSec = maxW;
    p.targetDropPct = drop;
    p.skipSec = skip;
    variants.push_back({label, p});
  };
  add("fixed 1 h", 3600, 3600, 0);
  add("fixed 2 h", 7200, 7200, 0);
  add("fixed 4 h", 14400, 14400, 0);
  add("fixed 8 h", 28800, 28800, 0);
  add("2h..8h 5 %", 7200, 28800, 5.0f);
  add("2h..8h 10 %", 7200, 28800, 10.0f);
  add("2h..8h 15 %", 7200, 28800, 15.0f);
  add("2h..8h 20 %", 7200, 28800, 20.0f);
  add("1h..8h 10 %", 3600, 28800, 10.0f);
  add("1h..8h 15 %", 3600, 28800, 15.0f);
  add("3h..8h 10 %", 10800, 28800, 10.0f);
  add("2h..6h 15 %", 7200, 21600, 15.0f);

  printf("median / 90th percentile of |estimate - truth| / truth, checkpoints from 1 h after the start\n\n");
  printf("%-34s", "");
  for (const Variant &v : variants) printf("| %-13.13s", v.label);
  printf("\n");
  std::vector<double> sumMed(variants.size(), 0), sumP90(variants.size(), 0);
  std::vector<int> counted(variants.size(), 0);
  for (const Scenario &sc : all) {
    printf("%-34s", sc.name);
    for (size_t k = 0; k < variants.size(); k++) {
      Score score;
      for (uint64_t seed = 1; seed <= 6; seed++) run(sc, variants[k].p, seed, 3600, &score, false);
      const double med = percentile(score.errors, 0.5), p90 = percentile(score.errors, 0.9);
      printf("| %4.0f%% /%4.0f%% ", med * 100, p90 * 100);
      if (!std::isnan(med)) {
        sumMed[k] += med;
        sumP90[k] += p90;
        counted[k]++;
      }
    }
    printf("\n");
  }
  printf("%-34s", "mean over scenarios");
  for (size_t k = 0; k < variants.size(); k++)
    printf("| %4.0f%% /%4.0f%% ", sumMed[k] / counted[k] * 100, sumP90[k] / counted[k] * 100);
  printf("\n");
  return 0;
}
