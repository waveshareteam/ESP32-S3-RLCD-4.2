#pragma once

// Battery runtime estimate, for the Info page (never the main screen).
//
// The voltage is turned into a charge percentage through the discharge curve first.  The same
// drain in mA is a very different number of mV per minute depending on where on the curve the
// cell is (about 2 mV per percent in the middle, 15 and more near empty), so a trend in volts
// means little; a trend in percent is one steady rate all the way down.
//
//   * readings are boiled down to one point per 5 minutes, the median of that bin, which throws
//     away the dips of WiFi transmit bursts;
//   * the first 15 minutes after the clock went onto the battery are ignored (a cell just off
//     the charger is still settling and falls faster than it really drains);
//   * the rate is the slope of a straight line through the points of a window that adapts to the
//     drain: long enough that the cell loses about 20 % inside it, but never under 2 hours or
//     over 8.  (The middle of the curve is nearly flat, a few millivolts there are several
//     percent, so short windows chase noise: a 30 minute fit was off by 60 % or more one time in
//     ten in simulation, the adaptive 2 to 8 hour window by 15 to 40 %.  A fast drain still
//     gets the short end of that range, a slow one the long.)  Until two hours of history exist
//     it uses all there is, and the Info page says how many minutes the figure is based on;
//   * one round of outlier rejection, then "hours left" is the percent above the cut-off level
//     divided by that rate.
//
// It promises the time left at the *average* load of the window, which is the best guess when the
// load varies; it cannot know what you will do next.  Header-only and free of Arduino dependencies
// so tools/tests can simulate whole discharges; the constants in Params were chosen with
// tools/tests/battery_sim.cpp.  That is a model of a cell, not a measurement of yours.

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "calc.h"

namespace battest {

const int kMaxBins = 96;     // 8 hours of 5 minute points
const int kMaxSamples = 40;  // readings kept per bin (one every 5 s fills 60 per bin; the first 40 count)
const float kMinRatePctPerHour = 0.02f;  // a fall slower than this is no fall that can be measured: no rate, no time left

struct Params {
  uint32_t binSec = 300;
  uint32_t skipSec = 900;         // ignore this long after the cell went onto the battery
  uint32_t readySec = 1800;       // the first estimate comes when this much settled history exists
  uint32_t minWindowSec = 7200;   // the fit never looks at less than this (or at everything there is, while less)
  uint32_t maxWindowSec = 28800;  // ... and never further back than this
  float targetDropPct = 20.0f;    // aim for a window in which the cell loses about this much (0 = always the maximum)
};

struct Estimate {
  enum State : uint8_t { OFF, LEARNING, READY };
  State state = OFF;       // OFF: nothing to say (not on battery yet, or reset)
  float pctPerHour = 0;    // drain rate, positive while discharging (READY)
  float hoursLeft = 0;     // until the cut-off level at that rate (READY; meaningless if unbounded)
  bool unbounded = false;  // the drain is too slow to measure: "more than a month"
  float avgMa = 0;         // average current in mA when the capacity is known, else 0
  float levelPct = 0;      // the fitted charge level now
  int windowMin = 0;       // minutes of history the rate was fitted on
  int learnMin = 0;        // LEARNING: minutes until the first estimate
};

// "12 min", "5 h 40 min", "2 d 4 h", ">30 d"
inline void formatRemaining(float hours, char *out, size_t cap) {
  if (!(hours >= 0)) {
    snprintf(out, cap, "--");
    return;
  }
  const int mins = (int)lroundf(hours * 60.0f);
  if (mins < 60) {
    snprintf(out, cap, "%d min", mins);
  } else if (mins < 48 * 60) {
    snprintf(out, cap, "%d h %02d min", mins / 60, mins % 60);
  } else if (mins < 30 * 24 * 60) {
    snprintf(out, cap, "%d d %d h", mins / 1440, (mins % 1440) / 60);
  } else {
    snprintf(out, cap, ">30 d");
  }
}

class Estimator {
 public:
  explicit Estimator(const Params &p = Params()) : p_(p) {}

  // Forget everything: the charger came or went, the battery was swapped.
  void reset() {
    nBins_ = 0;
    curN_ = 0;
    curOffsetSum_ = 0;  // the open bin's time offsets belong to the history that is being dropped
    started_ = false;
  }

  // One reading (volts at the battery) taken while running on the battery.
  void add(uint32_t nowSec, float volts) {
    if (!(volts > 0.5f && volts < 6.0f)) return;
    if (started_ && nowSec < lastSec_) reset();  // the time base restarted
    const uint32_t bin = nowSec / p_.binSec;
    if (!started_) {
      started_ = true;
      startSec_ = nowSec;
      curBin_ = bin;
    } else if (bin != curBin_) {
      closeBin();
      curBin_ = bin;
    }
    lastSec_ = nowSec;
    if (curN_ < kMaxSamples) {
      cur_[curN_++] = calc::batteryPercentOpen(volts);  // (not cut off at 100: a fall from 4.22 V to 4.20 V is a fall)
      curOffsetSum_ += (double)(nowSec - bin * p_.binSec);
    }
  }

  // cutoffV: the level the clock shuts down at; capacityMah: 0 if unknown.
  Estimate estimate(float cutoffV, float capacityMah) const {
    Estimate e;
    if (!started_) return e;
    const uint32_t settledAt = startSec_ + p_.skipSec;
    double *t = t_, *y = y_;  // scratch in the object, not on the (small) task stack
    int n = 0;
    for (int i = 0; i < nBins_; i++)
      if (bins_[i].t >= settledAt) {
        t[n] = (double)bins_[i].t;
        y[n] = (double)bins_[i].pct;
        n++;
      }
    const double span = n >= 2 ? t[n - 1] - t[0] : 0;
    if (n < 4 || span < (double)p_.readySec - 1.0) {
      e.state = Estimate::LEARNING;
      // when will the first estimate be there: the end of the skip plus the history it needs
      const double ready = (double)settledAt + (double)p_.readySec + (double)p_.binSec;
      const double left = ready - (double)lastSec_;
      e.learnMin = left > 0 ? (int)ceil(left / 60.0) : 1;
      return e;
    }

    const double tLast = t[n - 1];
    LineFit fit = fitWindow(t, y, n, tLast, (double)p_.maxWindowSec);
    if (p_.targetDropPct > 0 && fit.ok && fit.slope < 0) {
      const double want = (double)p_.targetDropPct / (-fit.slope);  // seconds for that drop at this rate
      double w = want < (double)p_.minWindowSec ? (double)p_.minWindowSec : want;
      if (w > (double)p_.maxWindowSec) w = (double)p_.maxWindowSec;
      if (w < (double)p_.maxWindowSec) {
        const LineFit shorter = fitWindow(t, y, n, tLast, w);
        if (shorter.ok) fit = shorter;
      }
    }
    if (!fit.ok) {
      e.state = Estimate::LEARNING;
      e.learnMin = 5;
      return e;
    }

    e.state = Estimate::READY;
    e.windowMin = (int)lround(fit.window / 60.0);
    e.levelPct = (float)fit.level;
    e.pctPerHour = (float)(-fit.slope * 3600.0);
    if (e.pctPerHour < 0) e.pctPerHour = 0;
    if (capacityMah > 0) e.avgMa = e.pctPerHour / 100.0f * capacityMah;
    const float cutPct = calc::batteryPercentF(cutoffV);
    float above = (e.levelPct > 100.0f ? 100.0f : e.levelPct) - cutPct;  // (a cell holds no more than all of its charge)
    if (above < 0) above = 0;
    if (e.pctPerHour < kMinRatePctPerHour) {
      e.unbounded = true;
      e.hoursLeft = 0;
    } else {
      e.hoursLeft = above / e.pctPerHour;
      if (e.hoursLeft > 24.0f * 30.0f) e.unbounded = true;
    }
    return e;
  }

  int binCount() const { return nBins_; }
  const Params &params() const { return p_; }

 private:
  struct Bin {
    uint32_t t;
    float pct;
  };
  struct LineFit {
    bool ok = false;
    double slope = 0;   // percent per second
    double level = 0;   // percent at the newest point
    double window = 0;  // seconds covered
  };

  // Median of the readings in the bin that just ended, placed at their mean time.
  void closeBin() {
    if (curN_ == 0) return;
    float s[kMaxSamples];
    memcpy(s, cur_, sizeof(float) * (size_t)curN_);
    for (int i = 1; i < curN_; i++) {  // insertion sort: small n
      const float v = s[i];
      int j = i - 1;
      while (j >= 0 && s[j] > v) {
        s[j + 1] = s[j];
        j--;
      }
      s[j + 1] = v;
    }
    const float median = (curN_ & 1) ? s[curN_ / 2] : 0.5f * (s[curN_ / 2 - 1] + s[curN_ / 2]);
    Bin b;
    b.t = curBin_ * p_.binSec + (uint32_t)(curOffsetSum_ / curN_ + 0.5);
    b.pct = median;
    curN_ = 0;
    curOffsetSum_ = 0;

    // A cell that is being discharged cannot gain charge: a clear rise means a charger came
    // (or the battery was changed) without anyone telling us.  Start over from here.
    if (nBins_ > 0 && b.pct > bins_[nBins_ - 1].pct + 2.5f) {
      nBins_ = 0;
      startSec_ = b.t;
    }
    if (nBins_ == kMaxBins) {
      memmove(bins_, bins_ + 1, sizeof(Bin) * (kMaxBins - 1));
      nBins_--;
    }
    bins_[nBins_++] = b;
  }

  // Least-squares line through the points of the last `window` seconds, after throwing out
  // the points that sit far from a first fit.
  LineFit fitWindow(const double *t, const double *y, int n, double tLast, double window) const {
    LineFit f;
    double *tt = tt_, *yy = yy_, *res = res_;
    bool *keep = keep_;
    int m = 0;
    for (int i = 0; i < n; i++)
      if (t[i] >= tLast - window - 1e-9) {
        tt[m] = t[i];
        yy[m] = y[i];
        m++;
      }
    if (m < 4) return f;
    for (int i = 0; i < m; i++) keep[i] = true;
    double slope = 0, level = 0;
    for (int pass = 0; pass < 2; pass++) {
      double st = 0, sy = 0;
      int k = 0;
      for (int i = 0; i < m; i++)
        if (keep[i]) {
          st += tt[i];
          sy += yy[i];
          k++;
        }
      if (k < 4) return f;
      const double mt = st / k, my = sy / k;
      double sxx = 0, sxy = 0;
      for (int i = 0; i < m; i++)
        if (keep[i]) {
          sxx += (tt[i] - mt) * (tt[i] - mt);
          sxy += (tt[i] - mt) * (yy[i] - my);
        }
      if (sxx <= 0) return f;
      slope = sxy / sxx;
      level = my + slope * (tLast - mt);
      if (pass == 1) break;
      // residuals against this line: the median absolute one sets the scale
      int r = 0;
      for (int i = 0; i < m; i++)
        if (keep[i]) res[r++] = fabs(yy[i] - (my + slope * (tt[i] - mt)));
      for (int i = 1; i < r; i++) {
        const double v = res[i];
        int j = i - 1;
        while (j >= 0 && res[j] > v) {
          res[j + 1] = res[j];
          j--;
        }
        res[j + 1] = v;
      }
      const double mad = (r & 1) ? res[r / 2] : 0.5 * (res[r / 2 - 1] + res[r / 2]);
      double limit = 3.0 * 1.4826 * mad;
      if (limit < 0.3) limit = 0.3;  // percent: the curve table itself is no finer than this
      bool dropped = false;
      for (int i = 0; i < m; i++)
        if (keep[i] && fabs(yy[i] - (my + slope * (tt[i] - mt))) > limit) {
          keep[i] = false;
          dropped = true;
        }
      if (!dropped) break;
    }
    f.ok = true;
    f.slope = slope;
    f.level = level;
    f.window = tt[m - 1] - tt[0];
    return f;
  }

  Params p_;
  Bin bins_[kMaxBins];
  int nBins_ = 0;
  // working space of estimate(): a few KB that would be a lot on a task stack
  mutable double t_[kMaxBins], y_[kMaxBins], tt_[kMaxBins], yy_[kMaxBins], res_[kMaxBins];
  mutable bool keep_[kMaxBins];
  float cur_[kMaxSamples];
  int curN_ = 0;
  double curOffsetSum_ = 0;
  uint32_t curBin_ = 0;
  uint32_t startSec_ = 0;
  uint32_t lastSec_ = 0;
  bool started_ = false;
};

// When the estimator is fed: while the clock runs on its battery as far as anyone can tell, which is whenever
// nothing shows a charger (the charge detector says neither "charging" nor "held full").  Waiting for the
// detector to say "discharging" will not do: under a light load the voltage falls too slowly for it ever to
// say so (0.05 mV a minute against the 0.3 it asks for in twelve minutes), and nothing would be estimated at
// all.  The history starts afresh whenever a charger comes or goes.
class Gate {
 public:
  // One battery reading.  `charger`: one shows.  Returns true if the reading went to the estimator.
  bool feed(Estimator &est, bool haveBattery, bool charger, uint32_t nowSec, float volts) {
    const bool on = haveBattery && !charger;
    if (on != on_) {
      est.reset();
      on_ = on;
    }
    if (on) est.add(nowSec, volts);
    return on;
  }
  bool onBattery() const { return on_; }

 private:
  bool on_ = false;
};

}  // namespace battest
