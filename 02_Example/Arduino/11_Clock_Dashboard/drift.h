#pragma once

// Measures how far the clock's crystal runs from true time, from the corrections that network
// time syncs make.  The microsecond timer (esp_timer_get_time) counts on the crystal and is never
// adjusted; the system clock is that timer plus an offset K that each sync resets to the right
// time.  So K_i - K_(i-1), the size of a sync's correction, divided by the time between them is the
// crystal's error rate, and a line through many (timer, K) points averages the network jitter
// (tens of milliseconds a sync) down to a fraction of a ppm over a day.
//
//   ppm > 0: the clock runs fast (gains), ppm < 0: slow (loses).  1 ppm = 0.0864 s a day.
//
// This is what the clock would drift by with WiFi off.  Header-only and free of Arduino
// dependencies so tools/tests can check it on a PC.

#include <math.h>
#include <stdint.h>

namespace drift {

const int kMaxPoints = 48;
const int kMinPoints = 4;
const double kMinSpanSec = 3.0 * 3600.0;   // no figure before this much history
const int64_t kMinGapUs = 30LL * 60 * 1000000;  // one point per half hour at most

struct Result {
  bool valid = false;
  float ppm = 0;         // positive: the clock gains
  float errorPpm = 0;    // one standard error of that figure
  float secPerDay = 0;   // the same as seconds per day (positive: gains)
  float spanHours = 0;   // how much history it rests on
  int points = 0;
};

class Tracker {
 public:
  void reset() { n_ = 0; }

  // Call right after a time sync has set the clock: monoUs = esp_timer_get_time(), sysUs =
  // the system clock in microseconds since 1970, both read together.
  void addSync(int64_t monoUs, int64_t sysUs) {
    if (n_ > 0 && monoUs <= mono_[n_ - 1]) {  // the timer restarted: the old points mean nothing
      reset();
    }
    if (n_ == 0) {
      mono0_ = monoUs;
      k0_ = sysUs - monoUs;
    } else if (monoUs - mono_[n_ - 1] < kMinGapUs) {
      return;
    }
    if (n_ == kMaxPoints) {
      for (int i = 1; i < n_; i++) {
        mono_[i - 1] = mono_[i];
        k_[i - 1] = k_[i];
      }
      n_--;
    }
    mono_[n_] = monoUs;
    k_[n_] = (sysUs - monoUs) - k0_;
    n_++;
  }

  Result result() const {
    Result r;
    r.points = n_;
    if (n_ < kMinPoints) return r;
    double t[kMaxPoints], y[kMaxPoints];
    for (int i = 0; i < n_; i++) {
      t[i] = (double)(mono_[i] - mono0_) / 1e6;  // seconds
      y[i] = (double)k_[i] / 1e6;                // seconds
    }
    const double span = t[n_ - 1] - t[0];
    if (span < kMinSpanSec) return r;

    bool keep[kMaxPoints];
    for (int i = 0; i < n_; i++) keep[i] = true;
    double slope = 0, se = 0;
    int kept = n_;
    for (int pass = 0; pass < 2; pass++) {
      double mt = 0, my = 0;
      for (int i = 0; i < n_; i++)
        if (keep[i]) {
          mt += t[i];
          my += y[i];
        }
      mt /= kept;
      my /= kept;
      double sxx = 0, sxy = 0;
      for (int i = 0; i < n_; i++)
        if (keep[i]) {
          sxx += (t[i] - mt) * (t[i] - mt);
          sxy += (t[i] - mt) * (y[i] - my);
        }
      if (sxx <= 0) return r;
      slope = sxy / sxx;
      double ss = 0;
      double res[kMaxPoints];
      int m = 0;
      for (int i = 0; i < n_; i++)
        if (keep[i]) {
          const double e = y[i] - (my + slope * (t[i] - mt));
          ss += e * e;
          res[m++] = fabs(e);
        }
      se = kept > 2 ? sqrt(ss / (kept - 2) / sxx) : 0;
      if (pass == 1) break;
      // one round of outlier rejection: a sync that landed on a slow network path
      for (int i = 1; i < m; i++) {
        const double v = res[i];
        int j = i - 1;
        while (j >= 0 && res[j] > v) {
          res[j + 1] = res[j];
          j--;
        }
        res[j + 1] = v;
      }
      const double mad = (m & 1) ? res[m / 2] : 0.5 * (res[m / 2 - 1] + res[m / 2]);
      double limit = 3.0 * 1.4826 * mad;
      if (limit < 0.025) limit = 0.025;  // 25 ms: NTP over WiFi is no better than that
      int dropped = 0;
      for (int i = 0; i < n_; i++)
        if (keep[i]) {
          const double e = fabs(y[i] - (my + slope * (t[i] - mt)));
          if (e > limit) {
            keep[i] = false;
            dropped++;
          }
        }
      if (dropped == 0 || kept - dropped < kMinPoints) break;
      kept -= dropped;
    }
    r.valid = true;
    r.ppm = (float)(-slope * 1e6);
    r.errorPpm = (float)(se * 1e6);
    r.secPerDay = (float)(-slope * 86400.0);
    r.spanHours = (float)(span / 3600.0);
    return r;
  }

  int points() const { return n_; }

 private:
  int64_t mono_[kMaxPoints];
  int64_t k_[kMaxPoints];  // relative to k0_, microseconds
  int64_t mono0_ = 0, k0_ = 0;
  int n_ = 0;
};

}  // namespace drift
