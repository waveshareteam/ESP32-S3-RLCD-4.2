// Host-side tests for the portable parts of the firmware: calendar maths,
// sensor calculations, the time zone table, the Open-Meteo / Spotify parsers,
// URL helpers and the button gesture detector.
//
//   ./run_tests.sh        (Linux / WSL)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <string>
#include <vector>

#include "../../battery_est.h"
#include "../../buttons.h"
#include "../../calc.h"
#include "../../charge.h"
#include "../../display_power.h"
#include "../../frame_plan.h"
#include "../../infinity_cube.h"
#include "../../link_page.h"
#include "../../sand_cube.h"
#include "../../spotify_parse.h"
#include "../../timeutil.h"
#include "../../util.h"
#include "../../weather.h"
#include "../../weather_codes.h"

// Pull the table in directly so the test can see inside it.
#include "../../tz_table.cpp"

#include "check.h"

// ---------------------------------------------------------------------------
static void testCalendar() {
  section("calendar");
  CHECK(timeutil::weekday(2026, 10, 4) == 0);   // Sunday
  CHECK(timeutil::weekday(2026, 10, 5) == 1);   // Monday
  CHECK(timeutil::weekday(2000, 1, 1) == 6);    // Saturday
  CHECK(timeutil::weekday(2024, 2, 29) == 4);   // Thursday

  auto week = [](int y, int m, int d) {
    struct tm t = {};
    t.tm_year = y - 1900;
    t.tm_mon = m - 1;
    t.tm_mday = d;
    t.tm_hour = 12;
    timegm(&t);  // normalises tm_yday / tm_wday
    return timeutil::isoWeek(y, t.tm_yday, t.tm_wday);
  };
  CHECK(week(2026, 10, 4) == 40);
  CHECK(week(2026, 1, 1) == 1);
  CHECK(week(2027, 1, 3) == 53);   // 2026 has 53 ISO weeks
  CHECK(week(2021, 1, 1) == 53);   // Friday belonging to 2020's last week
  CHECK(week(2020, 12, 31) == 53);
  CHECK(week(2024, 12, 30) == 1);  // Monday already in 2025's week 1
  CHECK(week(2018, 12, 31) == 1);
  CHECK(week(2024, 2, 29) == 9);

  char buf[16];
  timeutil::formatUtcOffset(480, buf, sizeof buf);
  CHECK_STR(buf, "+08:00");
  timeutil::formatUtcOffset(-210, buf, sizeof buf);
  CHECK_STR(buf, "-03:30");
  timeutil::formatUtcOffset(0, buf, sizeof buf);
  CHECK_STR(buf, "+00:00");
  timeutil::formatUtcOffset(345, buf, sizeof buf);
  CHECK_STR(buf, "+05:45");

  timeutil::formatDuration(83000, buf, sizeof buf);
  CHECK_STR(buf, "1:23");
  timeutil::formatDuration(200040, buf, sizeof buf);
  CHECK_STR(buf, "3:20");
  timeutil::formatDuration(3661000, buf, sizeof buf);
  CHECK_STR(buf, "1:01:01");
  timeutil::formatDuration(0, buf, sizeof buf);
  CHECK_STR(buf, "0:00");

  CHECK_STR(timeutil::weekdayName(0), "Sunday");
  CHECK_STR(timeutil::weekdayShort(6), "Sat");
  CHECK_STR(timeutil::monthShort(9), "Oct");

  // epoch conversion against the C library
  CHECK(calc::epochFromUtc(2026, 10, 4, 0, 21, 7) == 1791073267);
  CHECK(calc::epochFromUtc(1970, 1, 1, 0, 0, 0) == 0);
  CHECK(calc::epochFromUtc(2000, 2, 29, 23, 59, 59) == 951868799);
  for (int y = 2024; y <= 2040; y++) {
    for (int m = 1; m <= 12; m++) {
      struct tm t = {};
      t.tm_year = y - 1900;
      t.tm_mon = m - 1;
      t.tm_mday = 28;
      t.tm_hour = 13;
      t.tm_min = 7;
      t.tm_sec = 9;
      CHECK(calc::epochFromUtc(y, m, 28, 13, 7, 9) == (int64_t)timegm(&t));
    }
  }
}

static void testTimeZones() {
  section("time zones");
  auto setTz = [](const char *tz) {
    setenv("TZ", tz, 1);
    tzset();
  };
  time_t jan = (time_t)calc::epochFromUtc(2026, 1, 15, 0, 0, 0);
  time_t jul = (time_t)calc::epochFromUtc(2026, 7, 15, 0, 0, 0);

  setTz("AWST-8");
  CHECK(timeutil::utcOffsetMinutes(jan) == 480);
  CHECK(timeutil::utcOffsetMinutes(jul) == 480);

  setTz("AEST-10AEDT,M10.1.0,M4.1.0/3");  // Sydney: DST in the southern summer
  CHECK(timeutil::utcOffsetMinutes(jan) == 660);
  CHECK(timeutil::utcOffsetMinutes(jul) == 600);

  setTz("EST5EDT,M3.2.0,M11.1.0");  // New York
  CHECK(timeutil::utcOffsetMinutes(jan) == -300);
  CHECK(timeutil::utcOffsetMinutes(jul) == -240);

  setTz("IST-1GMT0,M10.5.0,M3.5.0/1");  // Dublin uses "negative DST" in tzdata
  CHECK(timeutil::utcOffsetMinutes(jan) == 0);
  CHECK(timeutil::utcOffsetMinutes(jul) == 60);

  // offset spanning the date line / year boundary: local date differs from UTC date
  setTz("<+13>-13");
  time_t nye = (time_t)calc::epochFromUtc(2026, 12, 31, 20, 0, 0);  // already 2027 locally
  CHECK(timeutil::utcOffsetMinutes(nye) == 780);
  setTz("<-11>11");
  time_t ny = (time_t)calc::epochFromUtc(2027, 1, 1, 5, 0, 0);  // still 2026 locally
  CHECK(timeutil::utcOffsetMinutes(ny) == -660);

  // table lookups
  CHECK_STR(tzPosixForIana("Australia/Perth"), "AWST-8");
  CHECK_STR(tzPosixForIana("America/New_York"), "EST5EDT,M3.2.0,M11.1.0");
  CHECK_STR(tzPosixForIana("Europe/Kyiv"), "EET-2EEST,M3.5.0/3,M10.5.0/4");
  CHECK_STR(tzPosixForIana("Asia/Kolkata"), "IST-5:30");
  CHECK_STR(tzPosixForIana("Pacific/Auckland"), "NZST-12NZDT,M9.5.0,M4.1.0/3");
  CHECK(tzPosixForIana("Mars/Olympus_Mons") == nullptr);
  CHECK(tzPosixForIana("") == nullptr);
  CHECK(tzPosixForIana(nullptr) == nullptr);
  CHECK(tzPosixForIana("australia/perth") == nullptr);  // names are case sensitive

  // binary search depends on sorted order; check it and round-trip every entry
  size_t n = sizeof(kZones) / sizeof(kZones[0]);
  CHECK(n > 500);
  bool sorted = true;
  for (size_t i = 1; i < n; i++) sorted &= strcmp(kZones[i - 1].name, kZones[i].name) < 0;
  CHECK(sorted);
  size_t bad = 0;
  for (size_t i = 0; i < n; i++) {
    const char *p = tzPosixForIana(kZones[i].name);
    if (!p || strcmp(p, kZones[i].posix) != 0) bad++;
  }
  CHECK(bad == 0);

  // every rule in the table must be accepted by the C library's TZ parser and
  // yield a plausible offset (guards against a typo in the generator)
  size_t implausible = 0;
  for (size_t i = 0; i < n; i++) {
    setTz(kZones[i].posix);
    int a = timeutil::utcOffsetMinutes(jan), b = timeutil::utcOffsetMinutes(jul);
    if (a < -12 * 60 || a > 14 * 60 || b < -12 * 60 || b > 14 * 60) implausible++;
  }
  CHECK(implausible == 0);

  // synthesised fixed-offset rules
  char buf[40];
  calc::posixFromUtcOffset(28800, buf, sizeof buf);
  CHECK_STR(buf, "<+08>-8");
  calc::posixFromUtcOffset(19800, buf, sizeof buf);
  CHECK_STR(buf, "<+0530>-5:30");
  calc::posixFromUtcOffset(-12600, buf, sizeof buf);
  CHECK_STR(buf, "<-0330>3:30");
  calc::posixFromUtcOffset(0, buf, sizeof buf);
  CHECK_STR(buf, "<+00>0");
  calc::posixFromUtcOffset(-18000, buf, sizeof buf);
  CHECK_STR(buf, "<-05>5");
  int offsets[] = {28800, 19800, -12600, 0, -18000, 45900, 50400, -39600};
  for (int off : offsets) {
    calc::posixFromUtcOffset(off, buf, sizeof buf);
    setTz(buf);
    CHECK(timeutil::utcOffsetMinutes(jan) == off / 60);
  }
  setTz("UTC0");
}

// The ADC reads a few percent low: one multiplier, what a tester says over what the clock measured.
static void testBatteryCalibration() {
  section("battery voltage calibration");
  float f = 0;
  // the case that started it: a full cell is 4.20 V on the tester and 4.135 V to the clock, which calls that 89 %
  CHECK(calc::batteryPercent(4.135f) == 89);
  CHECK(calc::calibrationFactor(4.20f, 4.135f, &f));
  CHECK_NEAR(f, 1.01572, 1e-4);
  CHECK(calc::batteryPercent(4.135f * f) == 100);  // corrected, it is full (and over the 4.16 V the charge detector wants)
  CHECK(4.135f * f >= 4.16f);
  // the direction: a reading that is too high gets a factor under 1
  CHECK(calc::calibrationFactor(4.20f, 4.00f, &f));
  CHECK_NEAR(f, 1.05, 1e-6);
  CHECK(calc::calibrationFactor(4.00f, 4.20f, &f));
  CHECK_NEAR(f, 0.95238, 1e-4);
  CHECK(calc::calibrationFactor(3.90f, 3.90f, &f));
  CHECK_NEAR(f, 1.0, 1e-9);
  // the limits are the setting's (0.80 to 1.25), inclusive
  CHECK(calc::kCalibrationMin == 0.80f && calc::kCalibrationMax == 1.25f);
  CHECK(calc::calibrationFactor(3.2f, 4.0f, &f) && f == 0.8f);
  f = 7.0f;
  CHECK(!calc::calibrationFactor(3.19f, 4.0f, &f));
  CHECK(f == 7.0f);  // a refused figure leaves the caller's factor alone
  CHECK(calc::calibrationFactor(4.5f, 3.6f, &f) && f == 1.25f);
  f = 7.0f;
  CHECK(!calc::calibrationFactor(4.5f, 3.59f, &f));
  CHECK(f == 7.0f);
  // a reading under 2.5 V is a floating pin, not a cell, even when it would give a factor in range
  CHECK(!calc::calibrationFactor(2.6f, 2.4f, &f));
  CHECK(calc::calibrationFactor(2.6f, 2.5f, &f));  // (and 2.5 V itself is a reading)
  f = 7.0f;
  // a figure that cannot be one LiPo cell, or a pin that reads nothing, is refused and leaves the factor alone
  f = 7.0f;
  const float bad[] = {0.0f, 2.4f, 4.6f, -4.2f, NAN, INFINITY};
  for (float t : bad) CHECK(!calc::calibrationFactor(t, 4.1f, &f));
  const float badMeasured[] = {0.0f, 2.4f, 6.1f, -4.1f, NAN, INFINITY};
  for (float m : badMeasured) CHECK(!calc::calibrationFactor(4.2f, m, &f));
  CHECK(f == 7.0f);
}

static void testSensorMaths() {
  section("sensor maths");
  CHECK(calc::batteryPercent(4.25f) == 100);
  CHECK(calc::batteryPercent(4.20f) == 100);
  CHECK(calc::batteryPercent(3.784f) == 50);
  CHECK(calc::batteryPercent(3.584f) == 20);
  CHECK(calc::batteryPercent(3.27f) == 0);
  CHECK(calc::batteryPercent(2.9f) == 0);
  int mid = calc::batteryPercent(4.00f);
  CHECK(mid >= 77 && mid <= 78);
  int prev = -1;
  bool monotonic = true;
  for (int mv = 3000; mv <= 4300; mv++) {
    int p = calc::batteryPercent(mv / 1000.0f);
    monotonic &= p >= prev && p >= 0 && p <= 100;
    prev = p;
  }
  CHECK(monotonic);

  const uint8_t beef[2] = {0xBE, 0xEF};
  CHECK(calc::crc8(beef, 2) == 0x92);  // Sensirion datasheet check value

  // 25.0 C, 50 %RH
  uint8_t raw[6] = {0x66, 0x66, 0, 0x80, 0x00, 0};
  raw[2] = calc::crc8(raw, 2);
  raw[5] = calc::crc8(raw + 3, 2);
  float t = 0, rh = 0;
  CHECK(calc::shtc3Decode(raw, &t, &rh));
  CHECK_NEAR(t, 25.0, 0.01);
  CHECK_NEAR(rh, 50.0, 0.01);
  raw[2] ^= 0x01;  // corrupt the temperature CRC
  CHECK(!calc::shtc3Decode(raw, &t, &rh));
  raw[2] ^= 0x01;
  raw[5] ^= 0x80;  // corrupt the humidity CRC
  CHECK(!calc::shtc3Decode(raw, &t, &rh));

  CHECK_NEAR(calc::humidityAtTemperature(40, 22.0f, 22.0f), 40.0, 1e-4);
  CHECK_NEAR(calc::humidityAtTemperature(40, 26.4f, 22.4f), 50.8, 0.6);  // cooler air, higher RH
  CHECK_NEAR(calc::humidityAtTemperature(90, 30.0f, 20.0f), 100.0, 1e-4);  // clamped
  CHECK(calc::humidityAtTemperature(0, 25.0f, 20.0f) == 0.0f);
}

// ---------------------------------------------------------------------------
// Charge detection on simulated voltage traces.  A reading every 5 s (+-0.5 s)
// with Gaussian noise (sigma 4 mV), one reading in ten pulled down 20..70 mV as
// if a WiFi transmit burst hit the ADC, and a slow 1.5 mV wander for load changes.
// Every scenario runs over 24 random sequences so one lucky draw cannot pass.
// ---------------------------------------------------------------------------
namespace {
using CS = ChargeDetector;

struct Sim {
  CS d;
  uint32_t t = 0;  // ms since the trace began
  uint64_t rng;
  double wander;
  unsigned seen = 0;                     // bit per State seen since clearSeen()
  double firstAt[4] = {-1, -1, -1, -1};  // minute each state was first seen since clearSeen()
  int changes = 0;                       // state changes since clearSeen()
  CS::State last = CS::UNKNOWN;
  void (*tap)(void *ctx, float volts, uint32_t tMs) = nullptr;  // is handed every reading after the detector had it
  void *tapCtx = nullptr;

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
  template <class F>
  void run(double minutes, F truth) {
    const uint32_t end = t + (uint32_t)(minutes * 60000.0 + 0.5);
    while (t < end) {
      const double m = t / 60000.0;
      double v = truth(m) + 0.004 * gauss() + 0.0015 * sin(m * 1.26 + wander);
      if (uniform() < 0.10) v -= 0.020 + 0.050 * uniform();
      d.addSample((float)v, t);
      if (tap) tap(tapCtx, (float)v, t);
      const CS::State s = d.state();
      seen |= 1u << s;
      if (firstAt[s] < 0) firstAt[s] = m;
      if (s != last) {
        changes++;
        last = s;
      }
      t += 4500 + (uint32_t)(uniform() * 1000.0);
    }
  }
};

struct Verdict {
  bool ok = true;
  const char *why = "";
  void need(bool cond, const char *what) {
    if (ok && !cond) {
      ok = false;
      why = what;
    }
  }
};

// Runs a scenario over 24 random sequences.  maxBad is how many may fail: zero for things the
// detector must always get right, a couple for the odd coincidence that no voltage-only method
// can exclude (a level shift that happens to be followed by a drift the same way).
template <class F>
void forSeeds(const char *name, F scenario, int maxBad = 0) {
  int bad = 0;
  uint64_t firstSeed = 0;
  const char *firstWhy = "";
  for (uint64_t seed = 1; seed <= 24; seed++) {
    Verdict v;
    scenario(seed, v);
    g_checks++;
    if (!v.ok && bad++ == 0) {
      firstSeed = seed;
      firstWhy = v.why;
    }
  }
  g_checks++;
  if (bad > maxBad) {
    g_failed++;
    printf("  FAIL %s: %d of 24 sequences (at most %d allowed), first seed %llu: %s\n", name, bad, maxBad,
           (unsigned long long)firstSeed, firstWhy);
  }
}

constexpr unsigned bit(CS::State s) { return 1u << s; }
const unsigned kClaimsPower = bit(CS::CHARGING) | bit(CS::FULL);  // "USB is in" states

// How a clock comes to FULL: a charge is seen, from 4.10 V at 3 mV a minute up to a plateau of 4.19 V that the
// charger then holds (an hour in all; FULL comes some twelve minutes after the voltage stops).  A cell that is
// merely flat near the top when the clock starts is not called full for some three hours: see "started on a
// charger" and "started on its battery" below.
bool chargeToFull(Sim &s) {
  const double t0 = s.now();
  s.run(60, [t0](double m) {
    const double x = 4.10 + 0.003 * (m - t0);
    return x > 4.19 ? 4.19 : x;
  });
  return s.d.state() == CS::FULL;
}
}  // namespace

static void testCharge() {
  section("charge detection");
  using S = CS;

  forSeeds("steady discharge on battery", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    auto f = [](double m) { return 3.95 - 0.0006 * m; };
    s.run(10.5, f);
    v.need(s.d.state() == S::UNKNOWN, "claimed something before three minutes of warm-up and eight of data");
    s.run(9.5, f);
    v.need(s.d.state() == S::DISCHARGING, "not discharging by minute 20");
    s.clearSeen();
    s.run(120, f);
    v.need((s.seen & kClaimsPower) == 0, "called it charging or full");
    v.need(s.changes == 0, "flapped");
    v.need(fabs(s.d.slopeMvPerMin() + 0.6) < 0.6, "slope not near -0.6 mV/min");
  });

  forSeeds("USB plugged in while on battery (+90 mV step)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(25, [](double m) { return 3.83 - 0.0006 * m; });
    v.need(s.d.state() == S::DISCHARGING, "not discharging before the plug-in");
    const double t0 = s.now();
    s.clearSeen();
    s.run(45, [t0](double m) { return 3.83 - 0.0006 * t0 + 0.090 + 0.0025 * (m - t0); });
    v.need(s.firstAt[S::CHARGING] >= 0 && s.firstAt[S::CHARGING] - t0 <= 2.0, "charging not shown within 2 minutes");
    v.need(s.changes == 1, "flapped after the plug-in");
    v.need(s.d.state() == S::CHARGING, "not charging at the end");
  });

  forSeeds("charged from the middle of the curve up to full", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    auto f = [](double m) {
      const double x = 4.10 + 0.003 * m;  // constant current, then the charger holds 4.2 V from minute 33
      return x > 4.2 ? 4.2 : x;
    };
    s.run(17, f);
    v.need(s.d.state() == S::CHARGING, "not charging by minute 17");
    s.clearSeen();
    s.run(37, f);
    v.need(s.d.state() == S::FULL, "not full 20 minutes after the voltage stopped climbing");
    v.need((s.seen & bit(S::DISCHARGING)) == 0, "called it discharging");
    s.run(60, f);
    v.need(s.d.state() == S::FULL, "left FULL while on the charger");
  });

  // Unplugged from a full battery: the charger had stopped so no current flowed, nothing steps,
  // and only the slow decline gives it away.
  auto unplugFromFull = [](const char *name, double rate, double withinMin) {
    forSeeds(name, [rate, withinMin](uint64_t seed, Verdict &v) {
      Sim s(seed);
      v.need(chargeToFull(s), "not full on the charger");
      const double t0 = s.now();
      s.clearSeen();
      s.run(withinMin + 5, [=](double m) { return 4.19 - rate * (m - t0); });
      v.need(s.firstAt[S::DISCHARGING] >= 0 && s.firstAt[S::DISCHARGING] - t0 <= withinMin, "discharging not shown in time");
      v.need((s.seen & bit(S::CHARGING)) == 0, "called it charging");
    });
  };
  unplugFromFull("unplugged when full, steep fall (1.7 mV/min)", 0.0017, 14);
  unplugFromFull("unplugged when full, moderate fall (0.8 mV/min)", 0.0008, 30);
  unplugFromFull("unplugged when full, gentle fall (0.5 mV/min)", 0.0005, 45);

  // A light load (the clock with its radio mostly off: 10..45 mA on a 2500 mAh cell) makes the top of the
  // curve fall by 0.07..0.3 mV a minute, which twelve minutes cannot tell from flat: it sat on FULL for
  // hours.  The slow history ends FULL when the level is 12 mV under its plateau, so how long that takes
  // depends on the fall; and then it must stay ended however flat the last twelve minutes look.
  unplugFromFull("unplugged when full, light load (0.3 mV/min)", 0.0003, 75);
  unplugFromFull("unplugged when full, lighter load (0.12 mV/min)", 0.00012, 150);
  unplugFromFull("unplugged when full, light load (0.07 mV/min)", 0.00007, 240);

  {  // 200 sequences of a 0.3 mV/min fall: the older rules sometimes say discharging, then flat again, before the slow history has spoken
    int bad = 0;
    for (uint64_t seed = 1; seed <= 200; seed++) {
      Sim s(seed + 1100);
      chargeToFull(s);
      const double t0 = s.now();
      auto f = [=](double m) { return 4.19 - 0.0003 * (m - t0); };
      s.run(75, f);
      s.clearSeen();
      s.run(180, f);
      bad += s.changes != 0 ? 1 : 0;  // 75 minutes after the unplug it must be settled on DISCHARGING for good
    }
    g_checks++;
    if (bad > 0) {
      g_failed++;
      printf("  FAIL a 0.3 mV/min fall: the state still changed after 75 minutes in %d of 200 sequences\n", bad);
    }
  }

  for (double rate : {0.0003, 0.00007}) {
    char name[96];
    snprintf(name, sizeof name, "after the fall is seen, FULL does not come back (%.2f mV/min)", rate * 1000);
    forSeeds(name, [rate](uint64_t seed, Verdict &v) {
      Sim s(seed);
      v.need(chargeToFull(s) && !s.d.leftFull(), "not full on the charger, or falling already");
      const double t0 = s.now();
      auto f = [=](double m) { return 4.19 - rate * (m - t0); };
      s.run(rate > 0.0001 ? 90 : 240, f);
      v.need(s.d.state() == S::DISCHARGING, "not discharging in time");
      v.need(s.d.leftFull(), "no sag found");
      s.clearSeen();
      s.run(360, f);  // six more hours of the same slow fall: the twelve minute trend says "flat" the whole time
      v.need((s.seen & kClaimsPower) == 0, "called it full or charging again");
      v.need(s.changes == 0, "flapped");
      v.need(s.d.longSlopeMvPerMin() < -rate * 1000 * 0.5 && s.d.longSlopeMvPerMin() > -rate * 1000 * 1.5,
             "the slow trend does not show the fall");
    });
  }

  // What the charger itself does must not look like a discharge: its plateau wanders by a few millivolts,
  // and the cell relaxes after the charger ends.
  {  // 300 sequences: a plateau that wanders is the one case where a single lucky draw (the highest level) would show
    int bad = 0, notFull = 0;
    for (uint64_t seed = 1; seed <= 300; seed++) {
      Sim s(seed + 500);
      if (!chargeToFull(s)) notFull++;
      s.clearSeen();
      double level = 0, next = s.now() + 15;
      s.run(480, [&](double m) {
        if (m > next) {
          level = (s.uniform() < 0.5 ? 1 : -1) * 0.004 * s.uniform();
          next += 20 + 40 * s.uniform();
        }
        return 4.19 + level;
      });
      bad += (s.changes != 0 || s.d.leftFull()) ? 1 : 0;
    }
    g_checks += 2;
    if (notFull) {
      g_failed++;
      printf("  FAIL held full with plateau steps: %d of 300 sequences were not full on the charger\n", notFull);
    }
    if (bad) {
      g_failed++;
      printf("  FAIL held full with plateau steps of up to 4 mV every 20-60 minutes for 8 hours: left FULL in %d of 300 sequences\n", bad);
    }
  }
  // A dip of a few minutes (a cable knocked loose and pushed back) takes the cell down and it comes back.  The
  // twelve minute trend or the slow history may call it a discharge for a while, and while the dips keep
  // coming it may stay at that: a cell that was not seen charging is full only once the slow history holds no
  // fall, and a dip in its newest 40 minutes is one.  It must never be called charging, and once the dips are
  // over FULL has to be back within the hour.
  forSeeds("held full with a four minute dip of 13 mV every hour or so", [](uint64_t seed, Verdict &v) {
    Sim s(seed + 550);
    chargeToFull(s);
    s.run(150, [](double) { return 4.19; });  // (in the first three hours after a start the slow history is too short to bring FULL back)
    v.need(s.d.state() == S::FULL, "not full on the charger");
    s.clearSeen();
    double dipAt = s.now() + 20 + 50 * s.uniform();
    auto truth = [&](double m) {
      if (m > dipAt + 4) dipAt = m + 40 + 50 * s.uniform();
      return 4.19 - ((m >= dipAt && m < dipAt + 4) ? 0.013 : 0.0);
    };
    s.run(480, truth);
    v.need((s.seen & bit(S::CHARGING)) == 0, "a dip was called charging");
    if (s.now() < dipAt + 4 && s.now() >= dipAt) s.run(4, truth);  // (let a dip that is under way end)
    dipAt = 1e9;  // no more dips
    s.run(60, truth);
    v.need((s.seen & bit(S::CHARGING)) == 0, "a dip was called charging");
    v.need(s.d.state() == S::FULL, "FULL was not back an hour after the last dip");
  });
  for (double tau : {15.0, 30.0}) {
    char name[96];
    snprintf(name, sizeof name, "held full after the charger ends, 8 mV tail (tau %.0f min)", tau);
    forSeeds(name, [tau](uint64_t seed, Verdict &v) {
      Sim s(seed + 600);
      const double top = (4.198 - 4.10) / 0.003;  // the minute the charge reaches its end, and the cell starts to relax
      auto f = [=](double m) { return m < top ? 4.10 + 0.003 * m : 4.19 + 0.008 * exp(-(m - top) / tau); };
      s.run(top + 3 * tau + 25, f);  // (it is called FULL once the tail has flattened)
      v.need(s.d.state() == S::FULL, "not full on the charger");
      v.need((s.seen & bit(S::DISCHARGING)) == 0, "the cell relaxing was called a discharge");
      s.clearSeen();
      s.run(480, f);
      v.need(s.changes == 0, "left FULL");
    });
  }
  forSeeds("on battery at -0.07 mV/min, light load: never full or charging", [](uint64_t seed, Verdict &v) {
    Sim s(seed + 700);
    s.run(480, [](double m) { return 3.95 - 0.00007 * m; });
    v.need((s.seen & kClaimsPower) == 0, "called it full or charging");
  });

  // Once FULL has ended, the level stepping up by 12 mV (a glitch) and staying there is not a plug-in: off a
  // plateau the climb has to be clearer to count as charging, and that stays so while FULL is blocked.  Run
  // over 200 sequences, because a few cases of a glitch that lines up with the wander are unavoidable.
  {
    int bad = 0, setupFailed = 0;
    for (uint64_t seed = 1; seed <= 200; seed++) {
      Sim s(seed + 900);
      chargeToFull(s);
      const double t0 = s.now();
      s.run(150, [=](double m) { return 4.19 - 0.0003 * (m - t0); });
      if (!(s.d.state() == S::DISCHARGING && s.d.leftFull())) {
        setupFailed++;
        continue;
      }
      const double low = 4.19 - 0.0003 * (s.now() - t0);
      s.clearSeen();
      s.run(60, [=](double) { return low + 0.012; });
      bad += (s.seen & bit(S::CHARGING)) ? 1 : 0;
    }
    g_checks += 2;
    if (setupFailed) {
      g_failed++;
      printf("  FAIL glitch after FULL ended: %d of 200 sequences were not discharging with FULL blocked\n", setupFailed);
    }
    if (bad > 4) {
      g_failed++;
      printf("  FAIL a 12 mV step up after FULL ended was called charging in %d of 200 sequences (at most 4)\n", bad);
    }
  }

  // Plugged in again after the sag: charging is seen at once (a step) and, once the charger holds the cell,
  // it is full again, with nothing of the sag left over.
  forSeeds("plugged in again after a sag", [](uint64_t seed, Verdict &v) {
    Sim s(seed + 800);
    chargeToFull(s);
    const double t0 = s.now();
    s.run(150, [=](double m) { return 4.19 - 0.0003 * (m - t0); });
    v.need(s.d.state() == S::DISCHARGING && s.d.leftFull(), "no sag found before the plug-in");
    const double t1 = s.now();
    const double low = 4.19 - 0.0003 * (t1 - t0);
    s.clearSeen();
    s.run(60, [=](double m) {
      const double x = low + 0.075 + 0.002 * (m - t1);  // the charger takes over: a step up, held at 4.2 V
      return x > 4.2 ? 4.2 : x;
    });
    v.need(s.firstAt[S::CHARGING] >= 0 && s.firstAt[S::CHARGING] - t1 <= 3.0, "charging not shown within 3 minutes");
    v.need(s.d.state() == S::FULL, "not full again an hour after the plug-in");
    v.need(!s.d.leftFull(), "the sag was not forgotten");
  });

  forSeeds("booted while charging (+2 mV/min)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(17, [](double m) { return 3.90 + 0.002 * m; });
    v.need(s.d.state() == S::CHARGING, "not charging by minute 17");
    s.clearSeen();
    s.run(40, [](double m) { return 3.90 + 0.002 * (m + 17); });
    v.need(s.changes == 0, "flapped");
  });

  forSeeds("booted while charging gently (+1 mV/min)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(60, [](double m) { return 3.80 + 0.001 * m; });
    v.need(s.d.state() != S::DISCHARGING, "a rising voltage was called discharging");
  });

  forSeeds("booted on battery (-1 mV/min)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(18, [](double m) { return 3.85 - 0.001 * m; });
    v.need(s.d.state() == S::DISCHARGING, "not discharging by minute 18");
    s.clearSeen();
    s.run(60, [](double m) { return 3.85 - 0.001 * (m + 18); });
    v.need((s.seen & kClaimsPower) == 0, "called it charging or full");
  });

  // A clock started on a charger that has finished.  Nothing was seen of the charge, and twelve flat minutes at
  // the top are just what a nearly full cell under a light load looks like too, so nothing is claimed until
  // the slow history has shown some three hours without a fall.  Then it is FULL, and stays.
  forSeeds("started on a charger with the cell full (4.18 V)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(120, [](double) { return 4.18; });
    v.need(s.seen == bit(S::UNKNOWN), "claimed something of a cell that is merely flat at the top");
    s.run(105, [](double) { return 4.18; });
    v.need(s.d.state() == S::FULL, "not full after 225 minutes without a fall");
    v.need((s.seen & (bit(S::CHARGING) | bit(S::DISCHARGING))) == 0, "called it charging or discharging");
    s.clearSeen();
    s.run(180, [](double) { return 4.18; });
    v.need(s.seen == bit(S::FULL), "wavered while sitting flat");
  });

  // Found on the board: the clock was restarted on its battery with the cell at 99 %, the radio mostly off and
  // the processor slow.  The top of the curve then falls by some 0.05 mV a minute, which twelve minutes cannot
  // tell from flat: it was called FULL eleven minutes after the start, and the runtime estimate (which waits
  // for a clock that is on its own) was kept back for hours.  Sixteen hours each, down to 4.07 .. 4.16 V.
  for (double rate : {0.00012, 0.00007, 0.00005, 0.00003}) {
    char name[96];
    snprintf(name, sizeof name, "started on its battery with the cell nearly full (-%.2f mV/min)", rate * 1000);
    forSeeds(name, [rate](uint64_t seed, Verdict &v) {
      Sim s(seed + 1300);
      s.run(960, [rate](double m) { return 4.19 - rate * m; });
      v.need((s.seen & bit(S::FULL)) == 0, "a cell that carries the clock was called full");
      v.need((s.seen & bit(S::CHARGING)) == 0, "called it charging");
    });
  }

  // The board draws more while it joins WiFi and fetches over TLS in the first minutes after boot, the
  // battery sags and then recovers.  The recovery must not pass for a plug-in.
  auto startupSag = [](double m, double sagMv) { return m < 1.0 ? sagMv : (m < 2.5 ? sagMv * (2.5 - m) / 1.5 : 0.0); };
  forSeeds("boot on battery with a 100 mV start-up sag", [startupSag](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(40, [startupSag](double m) { return 3.90 - 0.0006 * m - startupSag(m, 100) / 1000.0; });
    v.need((s.seen & kClaimsPower) == 0, "the start-up sag looked like charging");
    v.need(s.d.state() == S::DISCHARGING, "not discharging after 40 minutes");
  });
  forSeeds("boot on a charger (full) with a 100 mV start-up sag", [startupSag](uint64_t seed, Verdict &v) {
    Sim s(seed);
    auto f = [startupSag](double m) { return 4.19 - startupSag(m, 100) / 1000.0; };
    s.run(40, f);
    v.need(s.seen == bit(S::UNKNOWN), "the start-up sag looked like discharging, or a flat cell was called something");
    s.run(185, f);
    v.need(s.d.state() == S::FULL, "not full after 225 minutes");
    v.need((s.seen & (bit(S::CHARGING) | bit(S::DISCHARGING))) == 0, "called it charging or discharging");
  });
  {  // the first three minutes are thrown away, and the Info page can say so
    Sim s(5);
    auto f = [](double m) { return 3.90 - 0.001 * m; };
    s.run(2.9, f);
    CHECK(s.d.warmingUp() && s.d.minutesOfData() == 0.0f);
    s.run(1.1, f);
    CHECK(!s.d.warmingUp() && s.d.minutesOfData() > 0.0f && s.d.minutesOfData() < 1.5f);
  }

  // A load burst (a WiFi reconnect, say) every three minutes pulling the battery down 40 mV for 20 s,
  // long enough to fill a whole 15 s bin.
  auto bursts = [](double m) { return fmod(m, 3.0) < 0.333 ? -0.040 : 0.0; };
  forSeeds("charging with regular WiFi bursts", [bursts](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(20, [bursts](double m) { return 3.90 + 0.002 * m + bursts(m); });
    v.need(s.d.state() == S::CHARGING, "not charging by minute 20");
    s.clearSeen();
    s.run(80, [bursts](double m) { return 3.90 + 0.002 * (m + 20) + bursts(m); });
    v.need((s.seen & bit(S::DISCHARGING)) == 0, "bursts looked like discharging");
  });
  forSeeds("discharging with regular WiFi bursts", [bursts](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(22, [bursts](double m) { return 3.95 - 0.0008 * m + bursts(m); });
    v.need(s.d.state() == S::DISCHARGING, "not discharging by minute 22");
    s.clearSeen();
    s.run(80, [bursts](double m) { return 3.95 - 0.0008 * (m + 22) + bursts(m); });
    v.need((s.seen & kClaimsPower) == 0, "bursts looked like charging");
  });

  forSeeds("the flat middle of the discharge curve (-0.3 mV/min)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(240, [](double m) { return 3.80 - 0.0003 * m; });
    v.need((s.seen & kClaimsPower) == 0, "called it charging or full");
    v.need(s.changes <= 2, "flapped");
  });

  forSeeds("pure noise around 3.70 V for four hours", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(240, [](double) { return 3.70; });
    v.need((s.seen & kClaimsPower) == 0, "noise looked like charging or full");
  });

  forSeeds("unplugged while charging (-80 mV step)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(30, [](double m) { return 3.90 + 0.0025 * m; });
    v.need(s.d.state() == S::CHARGING, "not charging before the unplug");
    const double t0 = s.now();
    s.clearSeen();
    s.run(8, [t0](double m) { return 3.90 + 0.0025 * t0 - 0.080 - 0.0006 * (m - t0); });
    v.need(s.firstAt[S::DISCHARGING] >= 0 && s.firstAt[S::DISCHARGING] - t0 <= 2.5, "discharging not shown within 2.5 minutes");
    v.need((s.seen & kClaimsPower & ~bit(S::CHARGING)) == 0, "called it full");
  });

  forSeeds("plugged in when nearly full (+60 mV, capped at 4.2 V)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(24, [](double m) { return 4.14 - 0.001 * m; });
    v.need(s.d.state() == S::DISCHARGING, "not discharging before the plug-in");
    const double t0 = s.now();
    s.clearSeen();
    s.run(25, [t0](double m) {
      const double x = 4.12 + 0.060 + 0.004 * (m - t0);
      return x > 4.2 ? 4.2 : x;
    });
    v.need(s.firstAt[S::CHARGING] >= 0 && s.firstAt[S::CHARGING] - t0 <= 2.5, "charging not shown within 2.5 minutes");
    v.need(s.d.state() == S::FULL, "not full 25 minutes later");
    v.need(s.changes == 2, "wrong sequence of states");
  });

  forSeeds("plugged in with only a small step (+25 mV, then +2 mV/min)", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    s.run(20, [](double m) { return 3.80 - 0.0006 * m; });
    const double t0 = s.now();
    s.clearSeen();
    s.run(30, [t0](double m) { return 3.80 - 0.0006 * t0 + 0.025 + 0.002 * (m - t0); });
    v.need(s.firstAt[S::CHARGING] >= 0 && s.firstAt[S::CHARGING] - t0 <= 14.0, "charging not shown within 14 minutes");
  });

  // A change in load moves the level for good; to a straight-line fit that looks like a ramp.
  forSeeds("load drops by 15 mV while on battery", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    auto f = [](double m) { return 3.90 - 0.0006 * m + (m > 30 ? 0.015 : 0.0); };
    s.run(20, f);
    s.clearSeen();
    s.run(60, f);
    v.need((s.seen & kClaimsPower) == 0, "a level shift looked like charging");
  }, 2);
  forSeeds("load drops by 25 mV in the middle of discharging", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    auto f = [](double m) { return 3.90 - 0.0006 * m + (m > 40 ? 0.025 : 0.0); };
    s.run(25, f);
    s.clearSeen();
    s.run(60, f);
    v.need((s.seen & kClaimsPower) == 0, "a level shift looked like charging");
  }, 2);
  forSeeds("load rises by 8 mV while full on the charger", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    v.need(chargeToFull(s), "not full before the shift");
    const double t0 = s.now();
    s.clearSeen();
    s.run(60, [t0](double m) { return 4.19 - (m > t0 + 5 ? 0.008 : 0.0); });
    v.need(s.seen == bit(S::FULL), "a small level shift took it out of FULL");
  });
  // A bigger drop at the top of the curve is, to a voltage reading, a battery that has started
  // supplying the load.  It may say so for a while; it must never say charging, and it must settle once the
  // level has sat still.  That takes a few hours: once FULL has ended it waits until the last hour has not
  // fallen, and a cell that was not seen charging is only called full when the slow history has no fall in it
  // at all, which is what keeps a cell that carries the clock from reading as full.
  forSeeds("load rises by 15 mV while full on the charger", [](uint64_t seed, Verdict &v) {
    Sim s(seed);
    v.need(chargeToFull(s), "not full before the shift");
    const double t0 = s.now();
    auto f = [t0](double m) { return 4.19 - (m > t0 + 5 ? 0.015 : 0.0); };
    s.clearSeen();
    s.run(45, f);
    v.need((s.seen & bit(S::CHARGING)) == 0, "a drop was called charging");
    s.run(255, f);
    v.need((s.seen & bit(S::CHARGING)) == 0, "a drop was called charging");
    v.need(s.d.state() == S::FULL, "did not settle back to FULL five hours on");
  });

  {  // the battery being removed, or readings stopping, starts it afresh
    Sim s(7);
    s.run(30, [](double m) { return 3.90 + 0.003 * m; });
    CHECK(s.d.state() == S::CHARGING && s.d.ready());
    s.d.reset();
    CHECK(s.d.state() == S::UNKNOWN && !s.d.ready() && s.d.minutesOfData() == 0.0f);

    ChargeDetector g;
    uint32_t t = 0;
    for (; t < 20 * 60000u; t += 5000) g.addSample(3.95f + 0.0025f * (t / 60000.0f), t);
    CHECK(g.state() == S::CHARGING);
    g.addSample(3.9f, t + 60000);  // 60 s of silence breaks the time axis
    CHECK(g.state() == S::UNKNOWN && g.minutesOfData() == 0.0f);
    // and the millisecond counter wrapping around is no gap at all
    ChargeDetector w;
    uint32_t tw = 0xFFFFFFFFu - 5 * 60000u;
    for (int i = 0; i < 12 * 60 / 5 + 40; i++, tw += 5000) w.addSample(3.95f - 0.001f * (i * 5 / 60.0f), tw);
    CHECK(w.state() == S::DISCHARGING);
  }
}

// ---------------------------------------------------------------------------
// The charge detector, the gate and the runtime estimator wired together as the sketch wires them
// (readSlowSensors()), on traces of a clock that runs on its battery under a light load.
//
// Found on the board: restarted with the cell at 99 %, the radio mostly off and the processor at 20 MHz, the
// clock said "full" and gave no estimate.  The voltage then falls too slowly for the detector to call it a
// discharge, the estimator was only fed once it had, and near the top a flat-looking cell was called full.
// ---------------------------------------------------------------------------
static void testEstimateUnderLightLoad() {
  section("runtime estimate under a light load");
  struct Rig {
    Sim s;
    battest::Estimator est;
    battest::Gate gate;
    double firstReadyMin = -1;  // the minute the estimator first had a figure
    bool heldBack = false;      // a reading was kept from the estimator
    explicit Rig(uint64_t seed) : s(seed) {
      s.tap = [](void *ctx, float volts, uint32_t tMs) {
        Rig *r = (Rig *)ctx;
        const CS::State st = r->s.d.state();
        if (!r->gate.feed(r->est, true, st == CS::CHARGING || st == CS::FULL, tMs / 1000, volts)) r->heldBack = true;
        if (r->firstReadyMin < 0 && r->est.estimate(3.30f, 0).state == battest::Estimate::READY) r->firstReadyMin = tMs / 60000.0;
      };
      s.tapCtx = this;
    }
  };

  // Near the top of the curve 5 to 6 mV are one percent, so 0.05 mV a minute is about 0.55 % an hour (a week on a
  // charge).  The rate the estimator should find is the fall on the curve over the hours it fits (after the first 15
  // minutes, up to the fifth hour).
  struct Case {
    const char *name;
    double startV, mvPerMin;
  };
  const Case cases[] = {
      {"started at 99 %, falling 0.05 mV/min", 4.195, 0.05},
      {"started at 99 %, falling 0.12 mV/min", 4.195, 0.12},
      {"started at 97 %, falling 0.03 mV/min", 4.184, 0.03},
      // as read on the board a minute after the charger: over the top of the curve, where a level that is cut
      // off at 100 % stands still for the first hours
      {"started at 4.215 V, over the top of the curve, falling 0.05 mV/min", 4.215, 0.05},
  };
  for (const Case &c : cases) {
    forSeeds(c.name, [c](uint64_t seed, Verdict &v) {
      const double pctPerHour = (calc::batteryPercentOpen((float)(c.startV - c.mvPerMin * 15 / 1000.0)) -
                                 calc::batteryPercentOpen((float)(c.startV - c.mvPerMin * 300 / 1000.0))) / 4.75;
      Rig r(seed + 1400);
      r.s.run(300, [c](double m) { return c.startV - c.mvPerMin / 1000.0 * m; });
      v.need(!r.heldBack, "readings were kept from the estimator (something claimed a charger)");
      v.need(r.firstReadyMin > 0 && r.firstReadyMin <= 60, "no figure within the first hour");
      const battest::Estimate e = r.est.estimate(3.30f, 0);
      v.need(e.state == battest::Estimate::READY && !e.unbounded, "no figure after five hours");
      v.need(fabs(e.pctPerHour - pctPerHour) <= 0.25 * pctPerHour, "the rate is more than a quarter off after five hours");
      v.need(e.windowMin >= 240, "the figure does not rest on the whole run");
    });
  }

  // Half way down, a fall of 0.01 mV a minute (under 0.1 % an hour): nothing the detector looks at moves; the
  // estimator is fed all the same, and says how little it rests on.
  forSeeds("started at 50 %, falling 0.01 mV/min", [](uint64_t seed, Verdict &v) {
    Rig r(seed + 1500);
    r.s.run(300, [](double m) { return 3.784 - 0.00001 * m; });
    v.need(!r.heldBack, "readings were kept from the estimator");
    v.need(r.firstReadyMin > 0 && r.firstReadyMin <= 60, "no figure within the first hour");
  });

  // A charger shows: the history is dropped, nothing is fed while it is there, and it starts afresh after.
  {
    battest::Estimator est;
    battest::Gate gate;
    uint32_t t = 0;
    auto feed = [&](int minutes, bool charger, float fromV, float perMin) {
      int fed = 0;
      for (int i = 0; i < minutes * 12; i++, t += 5) fed += gate.feed(est, true, charger, t, fromV + perMin * (float)(i / 12.0)) ? 1 : 0;
      return fed;
    };
    CHECK(!gate.onBattery());
    CHECK(feed(90, false, 4.00f, -0.0005f) == 90 * 12 && gate.onBattery());
    CHECK(est.estimate(3.30f, 0).state == battest::Estimate::READY && est.binCount() >= 16);
    CHECK(feed(30, true, 4.00f, 0.002f) == 0 && !gate.onBattery());
    CHECK(est.estimate(3.30f, 0).state == battest::Estimate::OFF && est.binCount() == 0);  // nothing of the discharge is left
    CHECK(feed(20, false, 4.06f, -0.0005f) == 20 * 12 && gate.onBattery());
    CHECK(est.estimate(3.30f, 0).state == battest::Estimate::LEARNING);  // ... and the new one starts from nothing
    // no battery at all (battery = none, or none fitted): nothing is fed
    CHECK(!gate.feed(est, false, false, t, 4.0f) && !gate.onBattery() && est.binCount() == 0);
  }
}

// ---------------------------------------------------------------------------
// Frame scheduling on a simulated UI loop.  The clock advances by what each step
// costs (draw about 30 ms, send about 15 ms, naps of 4-5 ms) and by stalls injected
// at random moments (a flash write, the WiFi stack hogging the memory bus); every
// frame's finishing time is compared with the second it is for.  The loop the
// firmware had before the planner (draw for the coming second `cost + 8 ms` ahead)
// is simulated alongside for comparison.
// ---------------------------------------------------------------------------
namespace {
// The panel as display_power.h has it: its controller refreshes the rows of the digits once every `frameUs` by its
// own oscillator (off by `ppm`), the first time at `phaseUs`; a frame is on the glass at the first refresh after its
// transfer has ended.
struct PanelScan {
  double frameUs;
  double phaseUs;
  double ppm;
  int64_t onGlassAt(int64_t endUs) const {
    const double period = frameUs * (1.0 + ppm * 1e-6);
    const double k = ceil(((double)endUs - phaseUs) / period);
    return (int64_t)ceil(phaseUs + k * period);
  }
};

struct LoopSim {
  int64_t t;  // wall clock, microseconds
  uint64_t rng;
  double stallEverySec = 0;  // mean time between stalls; 0 = none
  int stallMinMs = 0, stallMaxMs = 0;
  double spikeProb = 0, spikeFactor = 4;  // some draws take this many times longer
  double forceEverySec = 0;               // mean time between forced redraws (a key press, a toast); 0 = none
  int32_t latencyUs = 20000;              // the planner's time from a transfer to the glass (DISPLAY_LATENCY_MS)
  int64_t nextStallAt = INT64_MAX, nextForceAt = INT64_MAX, nextSlowAt = 0;
  bool forcePending = false;

  struct Shown {
    int64_t sec;
    int64_t endUs;
    bool scheduled;
  };
  std::vector<Shown> shown;

  explicit LoopSim(uint64_t seed) : t(1000000LL * 1000000LL + 123456), rng(seed * 0x9E3779B97F4A7C15ull + 99) {}

  double uniform() {
    rng = rng * 6364136223846793005ull + 1442695040888963407ull;
    return (double)(rng >> 11) / 9007199254740992.0;
  }
  double expo(double mean) { return -mean * log(1.0 - uniform() + 1e-12); }
  void arm() {
    if (stallEverySec > 0) nextStallAt = t + (int64_t)(expo(stallEverySec) * 1e6);
    if (forceEverySec > 0) nextForceAt = t + (int64_t)(expo(forceEverySec) * 1e6);
  }

  void advance(int64_t us) {
    t += us;
    while (t >= nextStallAt) {  // the task is frozen for a while, asleep or not
      t += (int64_t)((stallMinMs + uniform() * (stallMaxMs - stallMinMs)) * 1000.0);
      nextStallAt = t + (int64_t)(expo(stallEverySec) * 1e6);
    }
    if (t >= nextForceAt) {
      forcePending = true;
      nextForceAt = t + (int64_t)(expo(forceEverySec) * 1e6);
    }
  }
  int64_t drawCost() { return (int64_t)(30000 * (0.9 + 0.2 * uniform()) * (uniform() < spikeProb ? spikeFactor : 1.0)); }
  int64_t sendCost() { return (int64_t)(15000 * (0.9 + 0.2 * uniform())); }
  void nap(int ms) { advance(ms * 1000 + (int64_t)(uniform() * 1000)); }  // timer tick granularity
  void slowWork() {                                                       // the sensors, every 5 s when allowed
    if (t >= nextSlowAt) {
      advance(25000);
      nextSlowAt = t + 5000000;
    }
  }

  // The planner-driven loop
  void runNew(int seconds) {
    FramePlanner p(latencyUs);
    arm();
    const int64_t end = t + (int64_t)seconds * 1000000;
    auto draw = [&](time_t sec, bool scheduled) {
      const int64_t c = drawCost();
      advance(c);
      p.drew(sec, (uint32_t)c, scheduled);
    };
    auto send = [&]() {
      const int64_t c = sendCost();
      advance(c);
      shown.push_back({(int64_t)p.preparedSec(), t, p.preparedScheduled()});
      p.sent((uint32_t)c);
    };
    draw((time_t)(t / 1000000), false);  // start-up
    send();
    while (t < end) {
      const bool force = forcePending;
      forcePending = false;
      const FramePlanner::Step st = p.next((time_t)(t / 1000000), (int32_t)(t % 1000000), force);
      switch (st.action) {
        case FramePlanner::DRAW_NOW:
          draw(st.sec, false);
          send();
          break;
        case FramePlanner::PREPARE:
          draw(st.sec, true);
          break;
        case FramePlanner::SEND:
          advance(st.waitUs);
          send();
          break;
        case FramePlanner::SLEEP:
          nap(st.waitUs > 12000 ? 4 : 1);
          break;
        default:
          if (p.quiet((int32_t)(t % 1000000))) slowWork();
          nap(4);
          break;
      }
    }
  }

  // The loop as it was before: one step that draws and sends, started `cost + 8 ms` ahead.
  void runOld(int seconds) {
    arm();
    const int64_t end = t + (int64_t)seconds * 1000000;
    uint32_t frameCost = 45000;
    time_t lastDrawn = 0;
    while (t < end) {
      const int32_t usToNext = 1000000 - (int32_t)(t % 1000000);
      bool draw = forcePending;
      forcePending = false;
      time_t sec = (time_t)(t / 1000000);
      if (usToNext <= (int32_t)(frameCost + 8000)) sec += 1;
      if (sec != lastDrawn) draw = true;
      if (draw) {
        const int64_t c = drawCost() + sendCost();
        advance(c);
        frameCost = (uint32_t)((frameCost * 3 + c) / 4);
        shown.push_back({(int64_t)sec, t, true});
        lastDrawn = sec;
      } else {
        if (usToNext > 300000) slowWork();
        nap(4);
      }
    }
  }

  // Lateness of each scheduled frame once it is on the glass (sent + 20 ms of panel), in ms
  // after its second began.  0 is perfect.
  std::vector<double> lateness(size_t skipFrames = 5) const {
    std::vector<double> v;
    for (size_t i = skipFrames; i < shown.size(); i++)
      if (shown[i].scheduled) v.push_back(((shown[i].endUs + 20000) - shown[i].sec * 1000000LL) / 1000.0);
    return v;
  }
  // The same with the panel modelled: from the start of its second to each scheduled frame being on the glass, in ms.
  std::vector<double> onGlass(const PanelScan &panel, size_t skipFrames = 5) const {
    std::vector<double> v;
    for (size_t i = skipFrames; i < shown.size(); i++)
      if (shown[i].scheduled) v.push_back((panel.onGlassAt(shown[i].endUs) - shown[i].sec * 1000000LL) / 1000.0);
    return v;
  }
  static int countAbove(const std::vector<double> &v, double ms) {
    int n = 0;
    for (double x : v) n += x > ms;
    return n;
  }
  // Every second between the 5th and the one 5 s before the end appears at least once, in order.
  bool everySecondShown(int fromSec = 5, int untilSecFromEnd = 5) const {
    if (shown.size() < 10) return false;
    const int64_t first = shown.front().sec + fromSec, last = shown.back().sec - untilSecFromEnd;
    int64_t want = first;
    int64_t prev = -1;
    for (const Shown &s : shown) {
      if (prev >= 0 && s.sec < prev) return false;  // never backwards
      prev = s.sec;
      if (s.sec == want) want++;
      else if (s.sec > want && want <= last) return false;  // a second was skipped
    }
    return want > last;
  }
};
}  // namespace

static void testFramePlan() {
  section("frame scheduling");
  using P = FramePlanner;

  {  // quiet(): slow work only with plenty of time before the tick and nothing waiting to be sent
    P p(20000);
    CHECK(p.quiet(100000));
    CHECK(!p.quiet(800000));  // 200 ms to the tick
    p.drew(100, 30000, true);
    CHECK(!p.quiet(100000));
    p.sent(15000);
    CHECK(p.quiet(100000));
  }
  {  // the decisions at a few moments, with the default costs (send lead 35 ms, draw lead 125 ms)
    P p(20000);
    p.drew(500, 30000, false);
    p.sent(15000);  // the frame for second 500 is on the glass
    P::Step s = p.next(500, 400000, false);
    CHECK(s.action == P::IDLE);
    s = p.next(500, 880000, false);  // 120 ms to the tick: time to draw the next second
    CHECK(s.action == P::PREPARE && s.sec == 501);
    p.drew(501, 30000, true);
    s = p.next(500, 890000, false);  // 110 ms left, due in 75 ms: sleep
    CHECK(s.action == P::SLEEP);
    s = p.next(500, 960000, false);  // 40 ms left, due in 5 ms: send after a short wait
    CHECK(s.action == P::SEND && s.waitUs > 0 && s.waitUs <= 5000);
    p.sent(15000);
    s = p.next(500, 990000, false);
    CHECK(s.action == P::IDLE);
    s = p.next(501, 100000, false);  // the second has begun and it was shown: nothing to do
    CHECK(s.action == P::IDLE);
  }
  {  // a forced redraw is immediate, and in the last ~110 ms before a tick it shows the second about to start
    P p(20000);
    p.drew(500, 30000, false);
    p.sent(15000);
    P::Step s = p.next(500, 300000, true);
    CHECK(s.action == P::DRAW_NOW && s.sec == 500);
    s = p.next(500, 850000, true);  // 150 ms left: still the current second
    CHECK(s.action == P::DRAW_NOW && s.sec == 500);
    s = p.next(500, 920000, true);  // 80 ms left: not enough to draw this one and still prepare the next
    CHECK(s.action == P::DRAW_NOW && s.sec == 501);
    s = p.next(500, 980000, true);
    CHECK(s.action == P::DRAW_NOW && s.sec == 501);
  }
  {  // far behind (start-up or a long stall): draw the current second at once
    P p(20000);
    CHECK(p.next(500, 100000, false).action == P::DRAW_NOW);
    p.drew(500, 30000, false);
    p.sent(15000);
    P::Step s = p.next(503, 100000, false);
    CHECK(s.action == P::DRAW_NOW && s.sec == 503);
  }
  {  // a prepared frame whose second has begun is sent without waiting; one from the future is dropped
    P p(20000);
    p.drew(500, 30000, false);
    p.sent(15000);
    p.drew(501, 30000, true);
    P::Step s = p.next(501, 20000, false);
    CHECK(s.action == P::SEND && s.waitUs == 0);
    P q(20000);
    q.drew(500, 30000, false);
    q.sent(15000);
    q.drew(501, 30000, true);
    s = q.next(450, 100000, false);  // the clock jumped back by 50 seconds
    CHECK(!q.prepared());
    CHECK(s.action == P::DRAW_NOW && s.sec == 450);
  }
  {  // one freak cost does not move the schedule much
    P p(20000);
    p.drew(500, 30000, false);
    p.sent(15000);
    p.drew(501, 5000000, true);  // a draw that "took" five seconds
    CHECK(p.drawCostUs() <= 60000);
  }

  // --- simulated loops -------------------------------------------------------------------
  {  // no stalls, no spikes: every frame lands within a few ms of plan
    LoopSim s(1);
    s.runNew(300);
    CHECK(s.everySecondShown());
    const std::vector<double> late = s.lateness();
    double lo = 1e9, hi = -1e9;
    for (double x : late) lo = fmin(lo, x), hi = fmax(hi, x);
    CHECK(late.size() > 280);
    CHECK(lo > -12.0 && hi < 12.0);  // the nap granularity is the only error
  }
  {  // one draw in ten takes four times as long (120 ms): the old loop shows those frames late, the new one never does
    int newLate = 0, oldLate = 0, frames = 0;
    for (uint64_t seed = 1; seed <= 8; seed++) {
      LoopSim a(seed), b(seed);
      a.spikeProb = b.spikeProb = 0.10;
      a.runNew(200);
      b.runOld(200);
      CHECK(a.everySecondShown() && b.everySecondShown());
      newLate += LoopSim::countAbove(a.lateness(), 50.0);
      oldLate += LoopSim::countAbove(b.lateness(), 50.0);
      frames += (int)a.lateness().size();
    }
    printf("  draw spikes: frames more than 50 ms late: new %d, old %d of %d\n", newLate, oldLate, frames);
    CHECK(newLate == 0);
    CHECK(oldLate > frames / 40);  // the comparison would prove nothing if the old loop coped
  }
  {  // stalls of 20-300 ms about every 8 s, plus the draw spikes: fewer late frames than before at every threshold
    int newL[2] = {0, 0}, oldL[2] = {0, 0}, frames = 0;
    for (uint64_t seed = 1; seed <= 12; seed++) {
      LoopSim a(seed), b(seed);
      for (LoopSim *s : {&a, &b}) {
        s->stallEverySec = 8;
        s->stallMinMs = 20;
        s->stallMaxMs = 300;
        s->spikeProb = 0.05;
      }
      a.runNew(300);
      b.runOld(300);
      CHECK(a.everySecondShown() && b.everySecondShown());
      const double thr[2] = {50.0, 150.0};
      for (int k = 0; k < 2; k++) {
        newL[k] += LoopSim::countAbove(a.lateness(), thr[k]);
        oldL[k] += LoopSim::countAbove(b.lateness(), thr[k]);
      }
      frames += (int)a.lateness().size();
    }
    printf("  stalls: frames more than 50 ms late: new %d, old %d; more than 150 ms: new %d, old %d (of %d)\n", newL[0],
           oldL[0], newL[1], oldL[1], frames);
    CHECK(newL[0] <= oldL[0]);
    CHECK(newL[1] <= oldL[1]);
    CHECK(newL[0] < frames / 12);  // and they are rare in absolute terms
  }
  {  // forced redraws (key presses, toasts) at random moments: nothing skipped or sent backwards, the schedule recovers
    for (uint64_t seed = 1; seed <= 6; seed++) {
      LoopSim s(seed);
      s.forceEverySec = 3.0;
      s.runNew(200);
      CHECK(s.everySecondShown());
      const std::vector<double> late = s.lateness(20);
      CHECK(late.size() > 120);
      CHECK(LoopSim::countAbove(late, 50.0) == 0);
    }
  }
  {  // the clock jumps (an NTP correction) forwards and backwards: no second is repeated or lost for long, nothing sticks
    LoopSim s(3);
    FramePlanner p(20000);
    int64_t jumps[][2] = {{40, 420000}, {80, -380000}, {120, 1300000}, {160, -1600000}, {200, 5000000}, {240, -3000000}};
    size_t nextJump = 0;
    const int64_t t0 = s.t;
    auto draw = [&](time_t sec, bool scheduled) {
      const int64_t c = s.drawCost();
      s.advance(c);
      p.drew(sec, (uint32_t)c, scheduled);
    };
    auto send = [&]() {
      const int64_t c = s.sendCost();
      s.advance(c);
      s.shown.push_back({(int64_t)p.preparedSec(), s.t, p.preparedScheduled()});
      p.sent((uint32_t)c);
    };
    draw((time_t)(s.t / 1000000), false);
    send();
    int64_t lastJumpAt = 0;
    int wrongAfterRecovery = 0, framesAfterRecovery = 0;
    while (s.t < t0 + 300 * 1000000LL) {
      if (nextJump < sizeof jumps / sizeof *jumps && s.t - t0 >= jumps[nextJump][0] * 1000000LL) {
        s.t += jumps[nextJump][1];
        lastJumpAt = s.t;
        nextJump++;
      }
      const FramePlanner::Step st = p.next((time_t)(s.t / 1000000), (int32_t)(s.t % 1000000), false);
      switch (st.action) {
        case FramePlanner::DRAW_NOW: draw(st.sec, false); send(); break;
        case FramePlanner::PREPARE: draw(st.sec, true); break;
        case FramePlanner::SEND: s.advance(st.waitUs); send(); break;
        case FramePlanner::SLEEP: s.nap(st.waitUs > 12000 ? 4 : 1); break;
        default: s.nap(4); break;
      }
      // once 3 s past a jump the frames must be back on schedule
      if (!s.shown.empty() && s.t - lastJumpAt > 3000000 && s.shown.back().scheduled) {
        const double late = ((s.shown.back().endUs + 20000) - s.shown.back().sec * 1000000LL) / 1000.0;
        framesAfterRecovery++;
        if (late > 15.0 || late < -15.0) wrongAfterRecovery++;
        s.shown.back().scheduled = false;  // count each frame once
      }
    }
    CHECK(framesAfterRecovery > 100);
    CHECK(wrongAfterRecovery == 0);
  }
}

// display_power (display_power.h): the panel's two refresh rates, the switch between them, and when a second is on
// the glass at each with the planner given the matching time to the glass.
static void testFramePlanPanel() {
  section("frame scheduling: the panel's refresh (display_power)");
  using P = FramePlanner;
  // the register and the figures: 25.5 Hz in the high power mode as before (HFRA 0), 8 Hz in the low one (LFRA 5)
  CHECK((dispower::kFrameRate & 0x10) == 0 && (dispower::kFrameRate & 0x07) == 5 && (dispower::kFrameRate & 0xE8) == 0);
  // for the deep-sleep shutdown: the same high power rate, the low one at its slowest (LFRA 0 = 0.25 Hz), nothing else
  CHECK((dispower::kRestFrameRate & 0x10) == (dispower::kFrameRate & 0x10) && (dispower::kRestFrameRate & 0x07) == 0 &&
        (dispower::kRestFrameRate & 0xE8) == 0);
  CHECK(dispower::latencyUs(false, 20000) == 20000);
  CHECK(dispower::latencyUs(true, 20000) == 20000 + dispower::kExtraLatencyUs);
  CHECK(dispower::kExtraLatencyUs > 42000 && dispower::kExtraLatencyUs < 43500);  // (125 - 39.2) / 2 ms
  {  // the datasheet's sequences (7.11): into LPM the voltages, 20 ms, 39h, 100 ms; out of it 38h, 300 ms, the voltages, 20 ms
    dispower::Step st[dispower::kMaxSteps];
    CHECK(dispower::switchSteps(true, st) == 2);
    CHECK(st[0].cmd == dispower::VOLTAGES && st[0].waitMs == 20 && st[1].cmd == 0x39 && st[1].waitMs == 100);
    CHECK(dispower::switchSteps(false, st) == 2);
    CHECK(st[0].cmd == 0x38 && st[0].waitMs == 300 && st[1].cmd == dispower::VOLTAGES && st[1].waitMs == 20);
  }
  {  // a frame that is prepared goes out by the new figure once it changes
    P p(20000);
    p.drew(500, 30000, false);
    p.sent(15000);
    p.drew(501, 30000, true);
    P::Step s = p.next(500, 940000, false);  // 60 ms left, sent 35 ms ahead: 25 ms to wait
    CHECK(s.action == P::SLEEP && s.waitUs == 25000);
    p.setLatencyUs(dispower::latencyUs(true, 20000));
    CHECK(p.latencyUs() == 20000 + dispower::kExtraLatencyUs);
    s = p.next(500, 940000, false);  // now 78 ms ahead: due already
    CHECK(s.action == P::SEND && s.waitUs == 0);
    s = p.next(500, 900000, false);  // (from 100 ms before the tick: 22 ms to wait)
    CHECK(s.action == P::SLEEP && s.waitUs > 21000 && s.waitUs < 23000);
  }
  {  // ... and the next second is drawn earlier: 168 ms ahead instead of 125
    P p(dispower::latencyUs(true, 20000));
    p.drew(500, 30000, false);
    p.sent(15000);
    CHECK(p.next(500, 820000, false).action == P::IDLE);
    P::Step s = p.next(500, 840000, false);
    CHECK(s.action == P::PREPARE && s.sec == 501);
  }

  // Simulated loops (LoopSim, no stalls or slow draws: those are the tests above), the panel's oscillator off by up
  // to 2 %, so its refreshes move through the second.  The error of each second: when it is on the glass less when it
  // began.
  struct Glass {
    double mean = 0, lo = 1e9, hi = -1e9;
    int n = 0;
    bool everySecond = true;
  };
  auto glass = [](double frameUs, int32_t latencyUs) {
    Glass g;
    double sum = 0;
    const double ppms[] = {-20000, -7000, -2000, 2500, 6000, 15000};
    for (int i = 0; i < 6; i++) {
      LoopSim s(40 + (uint64_t)i);
      s.latencyUs = latencyUs;
      s.runNew(240);
      g.everySecond = g.everySecond && s.everySecondShown();
      const PanelScan panel = {frameUs, s.uniform() * frameUs, ppms[i]};
      for (double e : s.onGlass(panel)) {
        sum += e;
        g.n++;
        g.lo = fmin(g.lo, e);
        g.hi = fmax(g.hi, e);
      }
    }
    g.mean = g.n ? sum / g.n : 0;
    return g;
  };
  const Glass hpm = glass(dispower::kHpmFrameUs, dispower::latencyUs(false, 20000));
  const Glass lpm = glass(dispower::kLpmFrameUs, dispower::latencyUs(true, 20000));
  const Glass wrong = glass(dispower::kLpmFrameUs, dispower::latencyUs(false, 20000));  // the 25.5 Hz figure on the 8 Hz panel
  printf("  on the glass: 25.5 Hz %+.1f ms on average (%+.0f to %+.0f), 8 Hz %+.1f ms (%+.0f to %+.0f), 8 Hz timed as 25.5 %+.1f ms"
         " (%+.0f to %+.0f), %d frames each\n",
         hpm.mean, hpm.lo, hpm.hi, lpm.mean, lpm.lo, lpm.hi, wrong.mean, wrong.lo, wrong.hi, lpm.n);
  CHECK(hpm.n > 1300 && lpm.n > 1300 && hpm.everySecond && lpm.everySecond);
  CHECK(fabs(hpm.mean) < 3.0 && hpm.lo > -33.0 && hpm.hi < 33.0);    // half of 39 ms either way, and the loop's own 12
  CHECK(fabs(lpm.mean) < 4.0 && lpm.lo > -76.0 && lpm.hi < 76.0);    // half of 125 ms either way, and the same 12
  CHECK(lpm.hi - lpm.lo > 110.0);                                     // (the refreshes did go round the whole second)
  CHECK(wrong.mean > 35.0 && wrong.hi > 90.0);                        // the check above would see the wrong figure
}

// ---------------------------------------------------------------------------
static void testWeather() {
  section("weather");
  std::string body = readFile("fixtures/open-meteo-forecast-perth.json");
  WeatherData w;
  ZoneInfo z;
  CHECK(parseOpenMeteoForecast(body.c_str(), body.size(), &w, &z));
  CHECK(w.valid);
  CHECK_NEAR(w.temp, 20.2, 1e-4);
  CHECK_NEAR(w.feels, 19.7, 1e-4);
  CHECK(w.humidity == 57);
  CHECK(w.code == 2);
  CHECK(w.isDay);
  CHECK_NEAR(w.windKmh, 12.3, 1e-4);
  CHECK(w.day[0].code == 3 && w.day[1].code == 3 && w.day[2].code == 51);
  CHECK_NEAR(w.day[0].tmax, 20.3, 1e-4);
  CHECK_NEAR(w.day[0].tmin, 12.0, 1e-4);
  CHECK_NEAR(w.day[1].tmax, 24.4, 1e-4);
  CHECK_NEAR(w.day[1].tmin, 11.6, 1e-4);
  CHECK_NEAR(w.day[2].tmax, 19.9, 1e-4);
  CHECK_NEAR(w.day[2].tmin, 15.2, 1e-4);
  CHECK(w.day[0].rainPct == 2 && w.day[1].rainPct == 2 && w.day[2].rainPct == 45);
  CHECK(w.day[0].weekday == 1 && w.day[1].weekday == 2 && w.day[2].weekday == 3);  // Mon, Tue, Wed
  CHECK_STR(w.sunrise, "05:49");
  CHECK_STR(w.sunset, "18:21");
  CHECK_NEAR(w.uvMax, 7.05, 1e-4);
  CHECK_STR(z.iana, "Australia/Perth");
  CHECK(z.utcOffsetSec == 28800);
  CHECK_STR(tzPosixForIana(z.iana), "AWST-8");

  // icon / text mapping for the codes in the fixture
  CHECK(wmoIcon(0) == WX_CLEAR && wmoIcon(2) == WX_PARTLY && wmoIcon(3) == WX_CLOUDY && wmoIcon(51) == WX_DRIZZLE);
  CHECK(wmoIcon(95) == WX_THUNDER && wmoIcon(73) == WX_SNOW && wmoIcon(65) == WX_RAIN);
  CHECK_STR(wmoText(0), "Clear");
  CHECK_STR(wmoText(1234), "Unknown");

  // malformed input must fail cleanly
  WeatherData bad;
  CHECK(!parseOpenMeteoForecast("", 0, &bad, &z));
  CHECK(!parseOpenMeteoForecast("{}", 2, &bad, &z));
  CHECK(!parseOpenMeteoForecast("{\"current\":{},\"daily\":{}}", 25, &bad, &z));
  CHECK(!parseOpenMeteoForecast(body.c_str(), body.size() / 2, &bad, &z));  // truncated
  std::string two = body;
  size_t p = two.find("\"2026-10-07\"");  // drop the third day's date
  if (p != std::string::npos) two.replace(p, 12, "null");
  CHECK(!parseOpenMeteoForecast(two.c_str(), two.size(), &bad, &z));

  // Everything shown as a fact is required: with one field missing (null, or not in the reply at all)
  // the reply is refused rather than shown with a plausible default (a clear sky, 0 % humidity).
  auto replaced = [&](const std::string &from, const std::string &to) {
    std::string s = body;
    const size_t at = s.find(from);
    CHECK(at != std::string::npos);  // the text this test edits is in the fixture
    if (at != std::string::npos) s.replace(at, from.size(), to);
    return s;
  };
  auto parses = [&](const std::string &s, WeatherData *out = nullptr, ZoneInfo *zone = nullptr) {
    WeatherData tmp;
    ZoneInfo tmpZone;
    return parseOpenMeteoForecast(s.c_str(), s.size(), out ? out : &tmp, zone ? zone : &tmpZone);
  };
  const char *currentFields[][2] = {{"temperature_2m", "20.2"}, {"apparent_temperature", "19.7"}, {"relative_humidity_2m", "57"},
                                    {"weather_code", "2"},      {"wind_speed_10m", "12.3"},      {"is_day", "1"}};
  for (const auto &f : currentFields) {
    const std::string lit = std::string("\"") + f[0] + "\":" + f[1];
    CHECK(parses(body));
    CHECK(!parses(replaced(lit, std::string("\"") + f[0] + "\":null")));    // null
    CHECK(!parses(replaced(lit, std::string("\"x_") + f[0] + "\":" + f[1])));  // not there
  }
  struct Column {
    const char *key;
    const char *v[3];
  };
  const Column columns[] = {{"weather_code", {"3", "3", "51"}},
                            {"temperature_2m_max", {"20.3", "24.4", "19.9"}},
                            {"temperature_2m_min", {"12.0", "11.6", "15.2"}}};
  for (const Column &c : columns) {
    const std::string lit = std::string("\"") + c.key + "\":[" + c.v[0] + "," + c.v[1] + "," + c.v[2] + "]";
    for (int hole = 0; hole < 3; hole++) {  // any one of the three days missing
      std::string v[3] = {c.v[0], c.v[1], c.v[2]};
      v[hole] = "null";
      CHECK(!parses(replaced(lit, std::string("\"") + c.key + "\":[" + v[0] + "," + v[1] + "," + v[2] + "]")));
    }
    CHECK(!parses(replaced(lit, std::string("\"x_") + c.key + "\":[" + c.v[0] + "," + c.v[1] + "," + c.v[2] + "]")));  // the column missing
    CHECK(!parses(replaced(lit, std::string("\"") + c.key + "\":[" + c.v[0] + "," + c.v[1] + "]")));              // two days only
  }
  {  // what the service leaves null where it has no figure is optional: 0 %, "--:--" and 0
    WeatherData o;
    CHECK(parses(replaced("\"precipitation_probability_max\":[2,2,45]", "\"precipitation_probability_max\":[null,null,null]"), &o));
    CHECK(o.day[0].rainPct == 0 && o.day[1].rainPct == 0 && o.day[2].rainPct == 0 && o.day[2].code == 51);
    CHECK(parses(replaced("\"precipitation_probability_max\":[", "\"x_precipitation_probability_max\":["), &o));
    CHECK(o.day[2].rainPct == 0);
    CHECK(parses(replaced("\"sunrise\":[", "\"x_sunrise\":["), &o));
    CHECK_STR(o.sunrise, "--:--");
    CHECK_STR(o.sunset, "18:21");  // the other one is still read
    CHECK(parses(replaced("\"sunset\":[", "\"x_sunset\":["), &o));
    CHECK_STR(o.sunset, "--:--");
    CHECK_STR(o.sunrise, "05:49");
    CHECK(parses(replaced("\"uv_index_max\":[", "\"x_uv_index_max\":["), &o));
    CHECK_NEAR(o.uvMax, 0.0, 1e-9);
    CHECK_NEAR(o.temp, 20.2, 1e-4);  // and the rest of the reply is intact
  }
  {  // a zone name needs an offset to go with it (a name the clock does not know becomes a fixed offset)
    WeatherData o;
    ZoneInfo zi;
    CHECK(parses(body, &o, &zi));
    CHECK_STR(zi.iana, "Australia/Perth");
    ZoneInfo noName;
    CHECK(parses(replaced("\"timezone\":\"Australia/Perth\"", "\"x_timezone\":\"Australia/Perth\""), &o, &noName));
    CHECK_STR(noName.iana, "");
    ZoneInfo noOffset;
    CHECK(parses(replaced("\"utc_offset_seconds\":28800", "\"x_utc_offset_seconds\":28800"), &o, &noOffset));
    CHECK_STR(noOffset.iana, "");  // not "UTC": the clock leaves its zone alone
    CHECK(o.valid);
    CHECK(parseOpenMeteoForecast(body.c_str(), body.size(), &o, nullptr));  // no zone wanted
  }

  // geocoder
  std::string geo = readFile("fixtures/open-meteo-geocode-perth.json");
  GeoResult g;
  CHECK(parseOpenMeteoGeocode(geo.c_str(), geo.size(), &g));
  CHECK_NEAR(g.lat, -31.95224, 1e-5);
  CHECK_NEAR(g.lon, 115.8614, 1e-5);
  CHECK_STR(g.name, "Perth");
  CHECK_STR(g.admin1, "Western Australia");
  CHECK_STR(g.country, "AU");
  CHECK_STR(g.iana, "Australia/Perth");
  CHECK(!parseOpenMeteoGeocode("{\"generationtime_ms\":0.1}", 24, &g));  // no results key
  CHECK(!parseOpenMeteoGeocode("{\"results\":[]}", 14, &g));
}

// ---------------------------------------------------------------------------
static void testSpotify() {
  section("spotify parsing");
  char devId[48];
  SpotifyInfo s;
  s.status = SPOTIFY_IDLE;
  copyStr(s.message, sizeof s.message, "keep me");

  std::string track = readFile("fixtures/spotify-player-track.json");
  CHECK(parsePlayerState(track.c_str(), track.size(), &s, devId, sizeof devId));
  CHECK(s.status == SPOTIFY_PLAYING);
  CHECK_STR(s.title, "Blinding Lights");
  CHECK_STR(s.artist, "The Weeknd, Daft Punk");
  CHECK_STR(s.album, "After Hours");
  CHECK_STR(s.device, "Kitchen speaker");
  CHECK(s.durationMs == 200040 && s.progressMs == 83000);
  CHECK(s.volume == 59);
  CHECK(s.shuffle);
  CHECK_STR(devId, "0d1841b0976bae2a3a310dd74c0f3df354899bc8");
  CHECK_STR(s.message, "keep me");  // caller-owned fields are preserved

  std::string ep = readFile("fixtures/spotify-player-episode.json");
  CHECK(parsePlayerState(ep.c_str(), ep.size(), &s, devId, sizeof devId));
  CHECK(s.status == SPOTIFY_PAUSED);
  CHECK_STR(s.title, "#412 - A very long episode title about many things");
  CHECK_STR(s.artist, "The Example Podcast");
  CHECK_STR(s.album, "");
  CHECK(s.volume == -1);  // null volume
  CHECK(s.progressMs == 1234567 && s.durationMs == 5400000);
  CHECK(!s.shuffle);

  std::string ad = readFile("fixtures/spotify-player-ad.json");
  CHECK(parsePlayerState(ad.c_str(), ad.size(), &s, devId, sizeof devId));
  CHECK(s.status == SPOTIFY_PLAYING);
  CHECK_STR(s.title, "Advertisement");
  CHECK_STR(s.artist, "");
  CHECK(s.durationMs == 0);

  // oversized strings are cut, never overflow
  std::string longName(400, 'x');
  std::string big = "{\"is_playing\":true,\"progress_ms\":1,\"item\":{\"name\":\"" + longName +
                    "\",\"duration_ms\":10,\"artists\":[";
  for (int i = 0; i < 40; i++) big += std::string(i ? "," : "") + "{\"name\":\"Artist Number " + std::to_string(i) + "\"}";
  big += "],\"album\":{\"name\":\"" + longName + "\"}},\"device\":{\"name\":\"" + longName + "\"}}";
  CHECK(parsePlayerState(big.c_str(), big.size(), &s, devId, sizeof devId));
  CHECK(strlen(s.title) == sizeof(s.title) - 1);
  CHECK(strlen(s.artist) < sizeof(s.artist));
  CHECK(strncmp(s.artist, "Artist Number 0, Artist Number 1", 32) == 0);
  CHECK(s.artist[strlen(s.artist) - 1] != ' ' && s.artist[strlen(s.artist) - 1] != ',');
  CHECK(strlen(s.device) == sizeof(s.device) - 1);

  CHECK(!parsePlayerState("not json", 8, &s, devId, sizeof devId));
  CHECK(!parsePlayerState("", 0, &s, devId, sizeof devId));

  // token replies
  SpotifyTokenReply tr;
  const char *tok =
      "{\"access_token\":\"BQDabc\",\"token_type\":\"Bearer\",\"expires_in\":3600,"
      "\"refresh_token\":\"AQBxyz\",\"scope\":\"user-read-playback-state\"}";
  CHECK(parseTokenReply(tok, strlen(tok), &tr));
  CHECK_STR(tr.accessToken, "BQDabc");
  CHECK_STR(tr.refreshToken, "AQBxyz");
  CHECK(tr.expiresInSec == 3600);
  const char *tok2 = "{\"access_token\":\"BQDdef\",\"token_type\":\"Bearer\",\"expires_in\":3600}";
  CHECK(parseTokenReply(tok2, strlen(tok2), &tr));
  CHECK_STR(tr.refreshToken, "");  // not rotated
  CHECK(!parseTokenReply("{\"error\":\"invalid_grant\"}", 25, &tr));
  CHECK(!parseTokenReply("{}", 2, &tr));

  // authorize URL: exact expected text, produced independently with Python's urllib.parse.quote
  char authUrl[600];
  size_t authLen = buildAuthorizeUrl("abc123", "http://127.0.0.1:8888/callback",
                                     "user-read-playback-state user-modify-playback-state",
                                     "E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM", "Zx9", authUrl, sizeof authUrl);
  CHECK(authLen == strlen(authUrl) && authLen > 0);
  CHECK_STR(authUrl,
            "https://accounts.spotify.com/authorize?client_id=abc123&response_type=code"
            "&redirect_uri=http%3A%2F%2F127.0.0.1%3A8888%2Fcallback"
            "&scope=user-read-playback-state%20user-modify-playback-state"
            "&code_challenge_method=S256&code_challenge=E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM&state=Zx9");
  char tinyUrl[40];
  CHECK(buildAuthorizeUrl("abc123", "http://127.0.0.1:8888/callback", "x", "y", "z", tinyUrl, sizeof tinyUrl) == 0);

  // error bodies
  char reason[40], msg[80];
  const char *e1 = "{\"error\":\"invalid_grant\",\"error_description\":\"Refresh token revoked\"}";
  parseSpotifyError(e1, strlen(e1), reason, sizeof reason, msg, sizeof msg);
  CHECK_STR(reason, "invalid_grant");
  CHECK_STR(msg, "Refresh token revoked");
  const char *e2 =
      "{\"error\":{\"status\":404,\"message\":\"Player command failed: No active device found\","
      "\"reason\":\"NO_ACTIVE_DEVICE\"}}";
  parseSpotifyError(e2, strlen(e2), reason, sizeof reason, msg, sizeof msg);
  CHECK_STR(reason, "NO_ACTIVE_DEVICE");
  CHECK_STR(msg, "Player command failed: No active device found");
  parseSpotifyError("garbage", 7, reason, sizeof reason, msg, sizeof msg);
  CHECK_STR(reason, "");
  parseSpotifyError(nullptr, 0, reason, sizeof reason, msg, sizeof msg);
  CHECK_STR(reason, "");
}

// ---------------------------------------------------------------------------
static void testLinkPage() {
  section("setup page");
  LinkPageInfo info;
  info.appName = "RLCD Clock";
  info.redirectUri = "http://127.0.0.1:8888/callback";
  info.clockAddress = "192.168.1.50";
  info.authorizeUrl = "https://accounts.spotify.com/authorize?client_id=abc&response_type=code&state=Zx9";

  static char body[6000], doc[8000];
  size_t n = renderLinkPageBody(info, body, sizeof body);
  CHECK(n > 500 && n == strlen(body));
  std::string b(body);
  CHECK(b.find("not linked") != std::string::npos);
  CHECK(b.find("value='http://127.0.0.1:8888/callback'") != std::string::npos);  // the URI to register
  CHECK(b.find("python spotify_link.py 192.168.1.50</code>") != std::string::npos);  // the clock's own address
  CHECK(b.find("href='/spotify_link.py'") != std::string::npos);
  CHECK(b.find("action=/link") != std::string::npos);
  CHECK(b.find("client_id=abc&amp;response_type=code&amp;state=Zx9") != std::string::npos);  // & escaped in the href
  CHECK(b.find("client_id=abc&response_type") == std::string::npos);
  CHECK(b.find("class=err") == std::string::npos && b.find("class=ok") == std::string::npos);  // no message yet
  CHECK(b.find("expected: nothing there is listening") != std::string::npos);  // the dead end is explained

  info.linked = true;
  info.daysLeft = 9;
  info.message = "Linked! <b>done</b> & dusted";
  info.messageIsError = false;
  n = renderLinkPageBody(info, body, sizeof body);
  b = body;
  CHECK(b.find("linked (Spotify expires the link in 9 days") != std::string::npos);
  CHECK(b.find("<p class=ok>Linked! &lt;b&gt;done&lt;/b&gt; &amp; dusted</p>") != std::string::npos);  // escaped
  info.messageIsError = true;
  renderLinkPageBody(info, body, sizeof body);
  CHECK(std::string(body).find("<p class=err>") != std::string::npos);

  char tiny[100];
  CHECK(renderLinkPageBody(info, tiny, sizeof tiny) == 0);  // too small is reported, not truncated silently

  info.linked = false;
  info.message = "";
  renderLinkPageBody(info, body, sizeof body);
  size_t dn = renderPageDocument("RLCD Clock", body, doc, sizeof doc);
  CHECK(dn == strlen(doc) && dn > strlen(body));
  CHECK(strncmp(doc, "<!doctype html>", 15) == 0);
  CHECK(strcmp(doc + dn - 14, "</body></html>") == 0);
  CHECK(strstr(doc, "<title>RLCD Clock - Spotify</title>") != nullptr);
  CHECK(renderPageDocument("RLCD Clock", body, tiny, sizeof tiny) == 0);

  FILE *f = fopen("build/link_page.html", "wb");  // for looking at in a browser
  if (f) {
    fwrite(doc, 1, dn, f);
    fclose(f);
  }
}

static void testUtil() {
  section("util");
  char buf[128];
  CHECK(urlEncode("a b&c=d/\xC3\xA9", buf, sizeof buf) > 0);
  CHECK_STR(buf, "a%20b%26c%3Dd%2F%C3%A9");
  CHECK(urlEncode("http://127.0.0.1:8888/callback", buf, sizeof buf) > 0);
  CHECK_STR(buf, "http%3A%2F%2F127.0.0.1%3A8888%2Fcallback");
  CHECK(urlEncode("user-read-playback-state user-modify-playback-state", buf, sizeof buf) > 0);
  CHECK_STR(buf, "user-read-playback-state%20user-modify-playback-state");
  char small[8];
  CHECK(urlEncode("this is far too long", small, sizeof small) == 0);

  char out[256];
  const char *url = "http://127.0.0.1:8888/callback?code=AQD%2Bxy%2Fz_1-2&state=abc123#frag";
  CHECK(queryParam(url, "code", out, sizeof out));
  CHECK_STR(out, "AQD+xy/z_1-2");
  CHECK(queryParam(url, "state", out, sizeof out));
  CHECK_STR(out, "abc123");
  CHECK(!queryParam(url, "error", out, sizeof out));
  CHECK(!queryParam(url, "cod", out, sizeof out));  // prefix must not match
  CHECK(queryParam("error=access_denied&state=s", "error", out, sizeof out));
  CHECK_STR(out, "access_denied");
  CHECK(!queryParam("", "code", out, sizeof out));
  char tiny[4];
  CHECK(queryParam("?code=abcdefgh", "code", tiny, sizeof tiny));
  CHECK_STR(tiny, "abc");  // truncated, still terminated

  copyStr(tiny, sizeof tiny, "abcdefgh");
  CHECK_STR(tiny, "abc");
  copyStr(tiny, sizeof tiny, nullptr);
  CHECK_STR(tiny, "");

  // PKCE (RFC 7636 appendix B): verifier dBjftJeZ4CVP-mB92K27uhbUJU1p1r_wW1gFWFOEjXk
  const uint8_t digest[32] = {0x13, 0xd3, 0x1e, 0x96, 0x1a, 0x1a, 0xd8, 0xec, 0x2f, 0x16, 0xb1,
                              0x0c, 0x4c, 0x98, 0x2e, 0x08, 0x76, 0xa8, 0x78, 0xad, 0x6d, 0xf1,
                              0x44, 0x56, 0x6e, 0xe1, 0x89, 0x4a, 0xcb, 0x70, 0xf9, 0xc3};
  CHECK(base64UrlEncode(digest, 32, out, sizeof out) == 43);
  CHECK_STR(out, "E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM");
  const uint8_t a[1] = {0xFB}, b[2] = {0xFB, 0xFF}, c[3] = {0xFB, 0xFF, 0xBF};
  CHECK(base64UrlEncode(a, 1, out, sizeof out) == 2);
  CHECK_STR(out, "-w");
  CHECK(base64UrlEncode(b, 2, out, sizeof out) == 3);
  CHECK_STR(out, "-_8");
  CHECK(base64UrlEncode(c, 3, out, sizeof out) == 4);
  CHECK_STR(out, "-_-_");
  CHECK(base64UrlEncode(digest, 32, small, sizeof small) == 0);
}

// ---------------------------------------------------------------------------
struct Press {
  uint32_t down, up;
};

static std::vector<std::pair<uint32_t, ClickEvent>> runButton(const std::vector<Press> &presses,
                                                              uint32_t endMs,
                                                              bool bounce = false) {
  ClickDetector d(320, 800, 25);
  std::vector<std::pair<uint32_t, ClickEvent>> events;
  for (uint32_t t = 0; t <= endMs; t += 5) {
    bool down = false;
    for (auto &p : presses) down |= (t >= p.down && t < p.up);
    if (bounce) {  // contact chatter for 12 ms after each edge
      for (auto &p : presses) {
        if ((t >= p.down && t < p.down + 12) || (t >= p.up && t < p.up + 12)) down = (t / 5) % 2;
      }
    }
    ClickEvent e = d.update(down, t);
    if (e != CLICK_NONE) events.push_back({t, e});
  }
  return events;
}

static void testButtons() {
  section("buttons");
  auto one = runButton({{100, 180}}, 2000);
  CHECK(one.size() == 1 && one[0].second == CLICK_1);
  CHECK(one.size() == 1 && one[0].first >= 180 + 320 && one[0].first <= 180 + 320 + 40);  // after the gap

  auto two = runButton({{100, 160}, {260, 320}}, 2000);
  CHECK(two.size() == 1 && two[0].second == CLICK_2);

  auto three = runButton({{100, 160}, {260, 320}, {420, 480}}, 2000);
  CHECK(three.size() == 1 && three[0].second == CLICK_3);

  auto four = runButton({{100, 160}, {260, 320}, {420, 480}, {580, 640}}, 2000);
  CHECK(four.size() == 1 && four[0].second == CLICK_3);  // 3+ clicks all map to CLICK_3

  // clicks slower than the gap are separate gestures
  auto slow = runButton({{100, 160}, {900, 960}}, 3000);
  CHECK(slow.size() == 2 && slow[0].second == CLICK_1 && slow[1].second == CLICK_1);

  // long press: fires once, while held, with no click afterwards
  auto lng = runButton({{100, 1500}}, 3000);
  CHECK(lng.size() == 1 && lng[0].second == CLICK_LONG);
  CHECK(lng.size() == 1 && lng[0].first >= 100 + 800 && lng[0].first <= 100 + 800 + 40);

  // a click followed by a long press reports only the long press
  auto mixed = runButton({{100, 160}, {260, 1400}}, 3000);
  CHECK(mixed.size() == 1 && mixed[0].second == CLICK_LONG);

  // contact bounce must not create phantom clicks
  auto bouncy = runButton({{100, 200}}, 2000, true);
  CHECK(bouncy.size() == 1 && bouncy[0].second == CLICK_1);

  // a glitch shorter than the debounce time is ignored
  auto glitch = runButton({{100, 110}}, 2000);
  CHECK(glitch.empty());

  // millis() wrap-around
  ClickDetector d(320, 800, 25);
  uint32_t base = 0xFFFFFF00u;
  ClickEvent e = CLICK_NONE;
  for (uint32_t i = 0; i < 700; i += 5) {  // base + i wraps past 2^32 around i = 256
    bool down = i >= 20 && i < 100;
    ClickEvent r = d.update(down, base + i);
    if (r != CLICK_NONE) e = r;
  }
  CHECK(e == CLICK_1);
}

// A loop that is busy part of the time, as the UI loop is while it draws a frame: `busyMs` out of every second,
// from `phaseMs` on, it does not look at the pin; otherwise every 5 ms.  With `edges` every change of the pin is
// also recorded with its time, as the interrupt of button_edges.cpp does (16 places, the rest is dropped), and
// the loop replays the record before it looks itself.  The pin is modelled to the millisecond.
static std::vector<std::pair<uint32_t, ClickEvent>> runBusyButton(const std::vector<Press> &presses, uint32_t endMs, uint32_t busyMs,
                                                                  uint32_t phaseMs, bool edges, uint32_t bounceMs = 0, uint32_t startMs = 0) {
  ClickDetector d(320, 800, 25);
  std::vector<std::pair<uint32_t, ClickEvent>> events;
  struct Edge {
    uint32_t at;
    bool down;
  };
  std::vector<Edge> log;
  auto level = [&](uint32_t t) {
    bool down = false;
    for (auto &p : presses) down |= (t >= p.down && t < p.up);
    for (auto &p : presses) {  // contact chatter after each change: the level flips every millisecond
      if ((t >= p.down && t < p.down + bounceMs) || (t >= p.up && t < p.up + bounceMs)) down = (t & 1) != 0;
    }
    return down;
  };
  bool last = false;
  for (uint32_t t = 0; t <= endMs; t++) {
    const bool now = level(t);
    if (edges && now != last && log.size() < 15) log.push_back({startMs + t, now});  // (a ring of 16 holds 15)
    last = now;
    const uint32_t inSecond = (t + 1000 - phaseMs % 1000) % 1000;
    if (t % 5 != 0 || inSecond < busyMs) continue;  // the loop is not looking
    for (const Edge &ed : log) {
      const ClickEvent e = d.edge(ed.down, ed.at);
      if (e != CLICK_NONE) events.push_back({t, e});
    }
    log.clear();
    // (with the record, the loop's own look is one more change at "now": if the record was full and the end
    // of a press is missing from it, the press still gets the time it lasted)
    const ClickEvent e = edges ? d.edge(now, startMs + t) : d.update(now, startMs + t);
    if (e != CLICK_NONE) events.push_back({t, e});
  }
  return events;
}

static void testButtonEdges() {
  section("buttons: a loop that is busy drawing");
  // A 100 ms tap that begins and ends inside a 120 ms frame (20 MHz): a loop that only looks never sees it ...
  CHECK(runBusyButton({{1010, 1110}}, 3000, 120, 0, false).empty());
  // ... and with the changes recorded it counts, as one click, reported once the gap after it has passed
  {
    const auto got = runBusyButton({{1010, 1110}}, 3000, 120, 0, true);
    CHECK(got.size() == 1 && got[0].second == CLICK_1);
    CHECK(got.size() == 1 && got[0].first >= 1110 + 320 && got[0].first <= 1110 + 320 + 130);
  }
  // Taps of 40 to 150 ms at every moment of the second, frames of 60, 120 and 240 ms: none is lost with the
  // record, and without it some are (which is what the record is for: the check of the check)
  for (uint32_t busy : {60u, 120u, 240u}) {
    int lostPlain = 0, lostEdges = 0, wrong = 0, taps = 0;
    for (uint32_t len : {40u, 60u, 100u, 150u}) {
      for (uint32_t start = 1000; start < 2000; start += 7) {
        taps++;
        const auto plain = runBusyButton({{start, start + len}}, start + 1500, busy, 0, false);
        const auto withEdges = runBusyButton({{start, start + len}}, start + 1500, busy, 0, true);
        if (plain.empty()) lostPlain++;
        if (withEdges.empty()) lostEdges++;
        if (withEdges.size() > 1 || (withEdges.size() == 1 && withEdges[0].second != CLICK_1)) wrong++;
        if (plain.size() > 1 || (plain.size() == 1 && plain[0].second != CLICK_1)) wrong++;
      }
    }
    printf("  (frames of %u ms: %d of %d taps lost by looking alone, %d with the changes recorded)\n", (unsigned)busy, lostPlain, taps, lostEdges);
    CHECK(lostEdges == 0 && wrong == 0);
    CHECK(lostPlain > 0);
  }
  // two and three clicks, with a frame somewhere in them, at every phase
  {
    int wrong = 0, runs = 0;
    for (uint32_t phase = 0; phase < 1000; phase += 13) {
      runs += 2;
      const auto two = runBusyButton({{1000, 1070}, {1170, 1240}}, 3000, 120, phase, true);
      const auto three = runBusyButton({{1000, 1070}, {1170, 1240}, {1340, 1410}}, 3000, 120, phase, true);
      if (two.size() != 1 || two[0].second != CLICK_2) wrong++;
      if (three.size() != 1 || three[0].second != CLICK_3) wrong++;
    }
    if (wrong) printf("  %d of %d multi-clicks came out wrong\n", wrong, runs);
    CHECK(wrong == 0);
  }
  // a long press is one long press, whenever the frames fall
  {
    int wrong = 0;
    for (uint32_t phase = 0; phase < 1000; phase += 37) {
      const auto lng = runBusyButton({{1000, 2400}}, 4000, 240, phase, true);
      if (lng.size() != 1 || lng[0].second != CLICK_LONG) wrong++;
    }
    CHECK(wrong == 0);
  }
  // contact chatter: still one click, also when the chatter fills the record (12 ms of it at each end is 24 changes)
  {
    int wrong = 0;
    for (uint32_t phase = 0; phase < 1000; phase += 37) {
      const auto bouncy = runBusyButton({{1000, 1100}}, 3000, 120, phase, true, 6);
      const auto storm = runBusyButton({{1000, 1100}}, 3000, 240, phase, true, 12);
      if (bouncy.size() != 1 || bouncy[0].second != CLICK_1) wrong++;
      if (storm.size() != 1 || storm[0].second != CLICK_1) wrong++;  // (the record overflows: still the one click, no phantom)
    }
    if (wrong) printf("  %d runs with contact chatter came out wrong\n", wrong);
    CHECK(wrong == 0);
  }
  // a glitch shorter than the debounce time stays a glitch, recorded or not
  CHECK(runBusyButton({{1010, 1020}}, 3000, 120, 0, true).empty());
  CHECK(runBusyButton({{1500, 1512}}, 3000, 120, 0, true).empty());
  // across the wrap of millis()
  {
    const auto got = runBusyButton({{1010, 1110}}, 3000, 120, 0, true, 0, 0xFFFFFA00u);  // the tap straddles 2^32
    CHECK(got.size() == 1 && got[0].second == CLICK_1);
  }
  // time never runs backwards for the detector: a look stamped earlier than a change already replayed (the
  // loop took its time before the interrupt came) is taken as "now", so nothing is accepted undebounced
  {
    ClickDetector det(320, 800, 25);
    CHECK(det.update(false, 1000) == CLICK_NONE);
    CHECK(det.edge(true, 1010) == CLICK_NONE);      // pressed at 1010 ...
    CHECK(det.update(true, 1005) == CLICK_NONE);    // ... and a look that says 1005: still 0 ms of "pressed"
    CHECK(det.edge(false, 1020) == CLICK_NONE);     // released after 10 ms: a glitch
    ClickEvent e = CLICK_NONE;
    for (uint32_t t = 1025; t < 2000 && e == CLICK_NONE; t += 5) e = det.update(false, t);
    CHECK(e == CLICK_NONE);
  }
}

// KEY held for five seconds and then let go restarts the clock (HoldRelease, fed from a ClickDetector).
struct HoldRun {
  int armed = 0, fired = 0, longs = 0, clicks = 0;
  int64_t armedAt = -1, firedAt = -1;
};

// `level(t)` is the pin; the loop looks every 5 ms, starting at `startMs` on the clock's own millis().
template <class Level>
static HoldRun runHold(Level level, uint32_t endMs, uint32_t holdMs = 5000, uint32_t startMs = 0) {
  ClickDetector det(320, 800, 25);
  HoldRelease hold(holdMs);
  HoldRun r;
  for (uint32_t t = 0; t <= endMs; t += 5) {
    const ClickEvent e = det.update(level(t), startMs + t);
    if (e == CLICK_LONG) r.longs++;
    else if (e != CLICK_NONE) r.clicks++;
    switch (hold.update(det.isDown(), det.heldMs())) {
      case HoldRelease::ARMED:
        r.armed++;
        r.armedAt = t;
        break;
      case HoldRelease::FIRED:
        r.fired++;
        r.firedAt = t;
        break;
      default:
        break;
    }
  }
  return r;
}

static void testHoldRelease() {
  section("buttons: a long hold that acts when the button is let go");
  {  // held from 1.0 s to 7.0 s: armed once at five seconds of holding, fired once when it comes up; the
     // ordinary long press (0.8 s) is reported as always, and no click
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000 && t < 7000; }, 9000);
    CHECK(r.armed == 1 && r.armedAt >= 6000 && r.armedAt <= 6000 + 40);
    CHECK(r.fired == 1 && r.firedAt >= 7000 && r.firedAt <= 7000 + 40);
    CHECK(r.longs == 1 && r.clicks == 0);
  }
  {  // let go a moment too early (4.9 s): nothing, however long one waits
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000 && t < 5900; }, 20000);
    CHECK(r.armed == 0 && r.fired == 0 && r.longs == 1);
  }
  {  // clicks and the ordinary hold: nothing
    const HoldRun one = runHold([](uint32_t t) { return t >= 1000 && t < 1100; }, 5000);
    const HoldRun two = runHold([](uint32_t t) { return (t >= 1000 && t < 1080) || (t >= 1200 && t < 1280); }, 5000);
    const HoldRun lng = runHold([](uint32_t t) { return t >= 1000 && t < 2500; }, 9000);
    CHECK(one.armed == 0 && one.fired == 0 && one.clicks == 1);
    CHECK(two.armed == 0 && two.fired == 0 && two.clicks == 1);
    CHECK(lng.armed == 0 && lng.fired == 0 && lng.longs == 1);
  }
  {  // several short holds do not add up
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000 && t < 20000 && (t - 1000) % 4000 < 3000; }, 22000);
    CHECK(r.armed == 0 && r.fired == 0);
  }
  {  // held for half a minute: armed once, nothing more while it is down, fired once at the end
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000 && t < 31000; }, 33000);
    CHECK(r.armed == 1 && r.fired == 1 && r.firedAt >= 31000 && r.firedAt <= 31040);
  }
  {  // still down at the end: armed, not fired (the clock does not restart under a finger)
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000; }, 20000);
    CHECK(r.armed == 1 && r.fired == 0);
  }
  {  // a contact that chatters for 10 ms in the middle of the hold is not "let go" (the debounced state is used)
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000 && t < 8000 && !(t >= 6500 && t < 6510); }, 10000);
    CHECK(r.armed == 1 && r.fired == 1 && r.firedAt >= 8000);
    // ... and one that chatters before the five seconds are up does not start the count again
    const HoldRun early = runHold([](uint32_t t) { return t >= 1000 && t < 8000 && !(t >= 3000 && t < 3010); }, 10000);
    CHECK(early.armed == 1 && early.armedAt >= 6000 && early.armedAt <= 6040);
  }
  {  // twice in a row works twice
    const HoldRun r = runHold([](uint32_t t) { return (t >= 1000 && t < 7000) || (t >= 9000 && t < 15000); }, 17000);
    CHECK(r.armed == 2 && r.fired == 2);
  }
  {  // across the wrap of millis()
    const HoldRun r = runHold([](uint32_t t) { return t >= 1000 && t < 7000; }, 9000, 5000, 0xFFFFF000u);
    CHECK(r.armed == 1 && r.fired == 1 && r.armedAt >= 6000 && r.armedAt <= 6040);
  }
  {  // the detector's own view: up means 0 ms, down counts from the moment the press was accepted
    ClickDetector det(320, 800, 25);
    CHECK(!det.isDown() && det.heldMs() == 0);
    for (uint32_t t = 0; t <= 1000; t += 5) det.update(false, t);
    CHECK(!det.isDown() && det.heldMs() == 0);
    for (uint32_t t = 1005; t <= 3000; t += 5) det.update(true, t);
    CHECK(det.isDown() && det.heldMs() >= 1960 && det.heldMs() <= 1995);  // 2 s less the 25 ms of debounce
    for (uint32_t t = 3005; t <= 3100; t += 5) det.update(false, t);
    CHECK(!det.isDown() && det.heldMs() == 0);
    // fed from recorded changes (the idle clock): a press recorded at 5000, first looked at after a slow frame
    // (120 ms later, which is when it is accepted and the count starts), then every 5 ms
    det.edge(true, 5000);
    det.edge(true, 5120);
    for (uint32_t t = 5125; t <= 11000; t += 5) det.edge(true, t);
    CHECK(det.isDown() && det.heldMs() == 11000 - 5120);
    HoldRelease hold(5000);
    CHECK(hold.update(det.isDown(), det.heldMs()) == HoldRelease::ARMED && hold.armed());
    CHECK(hold.update(det.isDown(), det.heldMs()) == HoldRelease::NONE);
    det.edge(false, 11500);
    det.edge(false, 11600);
    CHECK(hold.update(det.isDown(), det.heldMs()) == HoldRelease::FIRED && !hold.armed());
    CHECK(hold.update(det.isDown(), det.heldMs()) == HoldRelease::NONE);
  }
}

// ---------------------------------------------------------------------------
// The two buttons held down together for ten seconds, wired as the sketch wires them (pollButtons(), onKey(),
// onBoot(), cubeButton()): two click detectors, the hold that restarts the clock, and the chord.  What matters most is what must NOT
// happen while two buttons are held: KEY alone restarts the clock after five seconds, BOOT alone inverts the
// screen after one.
// ---------------------------------------------------------------------------
namespace {
struct Pad {
  ClickDetector key{320, 800}, boot{320, 1000};
  HoldRelease restart{5000};
  ChordHold both{10000, 520};
  bool cubeOn = false;
  int keyClicks = 0, keyLongs = 0, bootClicks = 0, bootLongs = 0;  // what handleKey() / handleBoot() would be given
  int armed = 0, restarts = 0, cubes = 0, ends = 0;
  int shoved = 0, switched = 0;  // what the cube was given while it was up: KEY's clicks, BOOT's clicks
  uint32_t cubeAt = 0;

  // A gesture of KEY or BOOT: to the cube while it is up (a long press ends it), else to the button's own handler.
  void gesture(bool isKey, ClickEvent e) {
    if (!cubeOn) {
      if (isKey) {
        (e == CLICK_LONG ? keyLongs : keyClicks)++;
      } else {
        (e == CLICK_LONG ? bootLongs : bootClicks)++;
      }
    } else if (e == CLICK_LONG) {
      cubeOn = false;
      ends++;
    } else {
      (isKey ? shoved : switched)++;
    }
  }

  void poll(bool k, bool b, uint32_t now) {
    const bool muted = both.muted();
    const ClickEvent ek = key.update(k, now), eb = boot.update(b, now);
    if (ek != CLICK_NONE && !muted) gesture(true, ek);
    if (eb != CLICK_NONE && !muted) gesture(false, eb);
    if (both.update(key.isDown(), boot.isDown(), now) == ChordHold::FIRED && !cubeOn) {
      cubeOn = true;
      cubes++;
      cubeAt = now;
    }
    if (both.muted()) {
      restart.cancel();
      return;
    }
    switch (restart.update(key.isDown(), key.heldMs())) {
      case HoldRelease::ARMED: armed++; break;
      case HoldRelease::FIRED: restarts++; break;
      default: break;
    }
  }
  // Runs `ms` of it, polled every 4 ms, from `*t`; the two functions say whether each button is down at a time
  // counted from `t0`.
  template <class K, class B>
  void run(uint32_t *t, uint32_t t0, uint32_t ms, K keyDown, B bootDown) {
    for (const uint32_t end = *t + ms; (int32_t)(end - *t) > 0; *t += 4) poll(keyDown(*t - t0), bootDown(*t - t0), *t);
  }
  int events() const { return keyClicks + keyLongs + bootClicks + bootLongs; }
};
}  // namespace

static void testChord() {
  section("buttons: two held down together");
  auto never = [](uint32_t) { return false; };
  for (uint32_t t0 : {0u, 0xFFFFE000u}) {  // (the second: across the wrap of millis())
    Pad p;
    uint32_t t = t0;
    // both down within 30 ms of each other, for 10.4 s; KEY comes up 80 ms before BOOT
    auto key = [](uint32_t s) { return s >= 1000 && s < 11400; };
    auto boot = [](uint32_t s) { return s >= 1030 && s < 11480; };
    p.run(&t, t0, 10900, key, boot);
    CHECK(p.cubes == 0);  // 9.9 s of the two together
    p.run(&t, t0, 400, key, boot);
    CHECK(p.cubes == 1 && p.cubeOn && p.cubeAt - t0 >= 11055 && p.cubeAt - t0 <= 11100);  // ten seconds after both were down
    p.run(&t, t0, 3000, key, boot);  // let go, and wait
    CHECK(p.cubes == 1 && p.cubeOn && p.ends == 0);
    CHECK(p.events() == 0);                  // no long press of either (0.8 s, 1 s), no click
    CHECK(p.armed == 0 && p.restarts == 0);  // and KEY's five seconds did not count: the clock does not restart

    // while it is up a tap of BOOT is "the other cube", not "next page", and a tap of KEY a shove, not "play"
    const uint32_t t1 = t;
    p.run(&t, t1, 2000, never, [](uint32_t s) { return s >= 200 && s < 300; });
    CHECK(p.cubeOn && p.switched == 1 && p.shoved == 0 && p.events() == 0);
    const uint32_t t1b = t;
    p.run(&t, t1b, 3000, [](uint32_t s) { return (s >= 200 && s < 300) || (s >= 1500 && s < 1600); }, never);
    CHECK(p.cubeOn && p.switched == 1 && p.shoved == 2 && p.events() == 0 && p.ends == 0);
    // a long press of either button ends it, and is neither "refresh" nor "invert"; held on, it is still nothing
    const uint32_t t1c = t;
    p.run(&t, t1c, 4000, never, [](uint32_t s) { return s >= 200 && s < 3200; });
    CHECK(!p.cubeOn && p.ends == 1 && p.events() == 0 && p.cubes == 1);
    // ... and after that the buttons are themselves again
    const uint32_t t2 = t;
    p.run(&t, t2, 2000, [](uint32_t s) { return s >= 200 && s < 300; }, [](uint32_t s) { return s >= 900 && s < 1000; });
    CHECK(p.keyClicks == 1 && p.bootClicks == 1 && p.events() == 2);
    const uint32_t t3 = t;
    p.run(&t, t3, 8000, [](uint32_t s) { return s >= 200 && s < 5700; }, never);
    CHECK(p.keyLongs == 1 && p.armed == 1 && p.restarts == 1);  // KEY alone for 5.5 s still restarts
  }
  {  // let go a little too early (9.6 s): nothing at all, and nothing afterwards
    Pad p;
    uint32_t t = 0;
    p.run(&t, 0, 14000, [](uint32_t s) { return s >= 1000 && s < 10650; }, [](uint32_t s) { return s >= 1020 && s < 10620; });
    CHECK(p.cubes == 0 && p.events() == 0 && p.armed == 0 && p.restarts == 0);
  }
  {  // KEY alone long enough to restart (the banner is up), then BOOT joins: letting go no longer restarts
    Pad p;
    uint32_t t = 0;
    p.run(&t, 0, 12000, [](uint32_t s) { return s >= 1000 && s < 9000; }, [](uint32_t s) { return s >= 7000 && s < 8000; });
    CHECK(p.armed == 1 && p.restarts == 0 && p.cubes == 0);
    CHECK(p.keyLongs == 1 && p.bootLongs == 0 && p.keyClicks == 0 && p.bootClicks == 0);  // (KEY's own long press came before BOOT did)
  }
  {  // BOOT lets go for a moment in the middle: the ten seconds start again, and KEY alone meanwhile is nothing
    Pad p;
    uint32_t t = 0;
    auto key = [](uint32_t s) { return s >= 1000 && s < 19000; };
    auto boot = [](uint32_t s) { return (s >= 1000 && s < 7000) || (s >= 7300 && s < 19000); };
    p.run(&t, 0, 16000, key, boot);
    CHECK(p.cubes == 0 && p.armed == 0);  // 15 s of KEY, but never ten of the two together
    p.run(&t, 0, 1400, key, boot);
    CHECK(p.cubes == 1 && p.cubeAt >= 17325 && p.cubeAt <= 17370);  // ten seconds after BOOT came back
    p.run(&t, 0, 4000, key, boot);
    CHECK(p.events() == 0 && p.armed == 0 && p.restarts == 0);
  }
  {  // one after the other: KEY's long press is its own (nobody was holding BOOT yet), then the two count from BOOT
    Pad p;
    uint32_t t = 0;
    auto key = [](uint32_t s) { return s >= 1000 && s < 15000; };
    auto boot = [](uint32_t s) { return s >= 4000 && s < 15000; };
    p.run(&t, 0, 13900, key, boot);
    CHECK(p.keyLongs == 1 && p.cubes == 0 && p.armed == 0);  // (KEY has been down for 12.9 s: no restart banner)
    p.run(&t, 0, 3000, key, boot);
    CHECK(p.cubes == 1 && p.cubeAt >= 14025 && p.cubeAt <= 14070 && p.restarts == 0 && p.events() == 1);
  }
  {  // KEY held to end the cube, and let go after two seconds: the clock is back, nothing is refreshed, nothing restarts
    Pad p;
    uint32_t t = 0;
    p.run(&t, 0, 13000, [](uint32_t s) { return s >= 1000 && s < 11500; }, [](uint32_t s) { return s >= 1000 && s < 11500; });
    CHECK(p.cubes == 1 && p.cubeOn);
    const uint32_t t1 = t;
    p.run(&t, t1, 5000, [](uint32_t s) { return s >= 200 && s < 2200; }, [](uint32_t) { return false; });
    CHECK(!p.cubeOn && p.ends == 1 && p.events() == 0 && p.armed == 0 && p.restarts == 0);
  }
  {  // held on and on: it comes up once
    Pad p;
    uint32_t t = 0;
    p.run(&t, 0, 45000, [](uint32_t s) { return s >= 1000; }, [](uint32_t s) { return s >= 1000; });
    CHECK(p.cubes == 1 && p.cubeOn && p.ends == 0 && p.events() == 0);
  }
  {  // the pieces: not muted at rest; muted from the moment both are down until both are up and quiet again
    ChordHold c(10000, 520);
    CHECK(!c.muted());
    CHECK(c.update(true, false, 100) == ChordHold::NONE && !c.muted());
    CHECK(c.update(true, true, 200) == ChordHold::NONE && c.muted());
    CHECK(c.update(true, false, 300) == ChordHold::NONE && c.muted());   // one is still down
    CHECK(c.update(false, false, 400) == ChordHold::NONE && c.muted());  // both up: quiet for 520 ms
    CHECK(c.update(false, false, 919) == ChordHold::NONE && c.muted());
    CHECK(c.update(false, false, 920) == ChordHold::NONE && !c.muted());
    ChordHold d(10000, 520);  // held for half a minute: fired once, at ten seconds
    int fired = 0;
    uint32_t firedAt = 0;
    for (uint32_t t = 0; t <= 30000; t += 4) {
      if (d.update(true, true, t) == ChordHold::FIRED) {
        fired++;
        firedAt = t;
      }
    }
    CHECK(fired == 1 && firedAt == 10000);
    HoldRelease h(5000);
    CHECK(h.update(true, 5000) == HoldRelease::ARMED && h.armed());
    h.cancel();
    CHECK(!h.armed() && h.update(false, 0) == HoldRelease::NONE);  // let go after a cancel: nothing fires
  }
}

// ---------------------------------------------------------------------------
// The cube those two buttons bring up (infinity_cube.h): the geometry only.  That it stays on the screen is
// measured on the rendered frames by the UI preview.
// ---------------------------------------------------------------------------
static void testInfinityCube() {
  section("infinity cube");
  using namespace infcube;
  struct Count {
    int w, h, margin;
    int lines = 0, bold = 0, dots = 0, outside = 0, unknown = 0, afterBold = 0, byStyle[kStyles] = {};
    int top = 1 << 20, bottom = -1;
    void see(int x, int y) {
      if (x < margin || x > w - 1 - margin || y < margin || y > h - 1 - margin) outside++;
      if (y < top) top = y;
      if (y > bottom) bottom = y;
    }
    void line(int x0, int y0, int x1, int y1, Style style) {
      lines++;
      if (style >= kStyles) {
        unknown++;
      } else {
        byStyle[style]++;
      }
      if (style == BOLD) {
        bold++;
      } else if (bold > 0) {
        afterBold++;  // the cube's own edges come last, over the ends of what is behind them
      }
      see(x0, y0);
      see(x1, y1);
    }
    void dot(int x, int y) {
      dots++;
      see(x, y);
    }
  };
  int frames = 0, bad = 0, offScreen = 0, empty = 0, small = 0, faces[4] = {}, least = 1 << 30, most = 0;
  for (uint32_t ms = 0; ms < 600000; ms += 173, frames++) {
    Count c{400, 300, 8};
    V3 ax[3];
    tumble((float)ms, 5, ax);
    draw(ax, ms, 400, 300, c);
    const int seen = c.bold / 4;
    if (c.bold % 4 != 0 || seen < 1 || seen > 3 || c.dots != c.bold) {
      bad++;
    } else {
      faces[seen]++;
    }
    if (c.outside || c.unknown || c.afterBold) bad++, offScreen++;
    if (c.lines - c.bold < 20 || c.byStyle[SOLID] < 1) bad++, empty++;  // behind the faces there is something, some of the cube's far side at least
    if (c.bottom - c.top < 150) bad++, small++;                         // and the cube is of a size to be seen
    if (c.lines < least) least = c.lines;
    if (c.lines > most) most = c.lines;
  }
  printf("  (%d frames: one face seen in %d, two in %d, three in %d; %d to %d lines a frame)\n", frames, faces[1], faces[2], faces[3], least, most);
  if (bad) printf("  %d frames are wrong: %d off the screen or out of order, %d with nothing behind the faces, %d too small\n", bad, offScreen, empty, small);
  CHECK(bad == 0);
  CHECK(faces[2] > 0 && faces[3] > 0 && faces[1] + faces[2] + faces[3] == frames);
  {  // seen square on: one face, its four edges, its four corners
    const V3 square[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Count c{400, 300, 8};
    draw(square, 0, 400, 300, c);
    CHECK(c.bold == 4 && c.dots == 4 && c.outside == 0);
  }
  {  // a small screen gets a small cube, not a cut one
    int outside = 0;
    for (uint32_t ms = 0; ms < 60000; ms += 211) {
      Count c{128, 64, 0};
      V3 ax[3];
      tumble((float)ms, 9, ax);
      draw(ax, ms, 128, 64, c);
      outside += c.outside;
    }
    CHECK(outside == 0);
  }
  {  // the tumble: three unit axes at right angles, always; never a jerk; every side of the cube comes to the front;
     // another seed, another way; and back along the same way, it is where it was
    int skewed = 0, jerks = 0;
    float least[3] = {9, 9, 9}, most[3] = {-9, -9, -9};
    V3 was[3];
    tumble(0.0f, 5, was);
    for (uint32_t ms = 16; ms < 900000; ms += 16) {
      V3 ax[3];
      tumble((float)ms, 5, ax);
      for (int i = 0; i < 3; i++) {
        if (fabsf(dot(ax[i], ax[i]) - 1.0f) > 1e-4f || fabsf(dot(ax[i], ax[(i + 1) % 3])) > 1e-4f) skewed++;
        const V3 moved = sub(ax[i], was[i]);
        if (dot(moved, moved) > 0.04f * 0.04f) jerks++;  // (a frame of 16 ms: under two and a half degrees)
        if (ax[i].z < least[i]) least[i] = ax[i].z;
        if (ax[i].z > most[i]) most[i] = ax[i].z;
        was[i] = ax[i];
      }
    }
    CHECK(skewed == 0 && jerks == 0);
    for (int i = 0; i < 3; i++) CHECK(least[i] < -0.9f && most[i] > 0.9f);
    V3 a[3], b[3], c[3];
    tumble(30000.0f, 5, a);
    tumble(30000.0f, 5, b);
    tumble(30000.0f, 6, c);
    CHECK(a[0].x == b[0].x && a[1].y == b[1].y && a[2].z == b[2].z);
    CHECK(fabsf(a[0].x - c[0].x) + fabsf(a[1].y - c[1].y) + fabsf(a[2].z - c[2].z) > 0.05f);
    tumble(0.0f, 5, a);
    tumble(0.0f, 6, c);
    CHECK(fabsf(a[0].x - c[0].x) + fabsf(a[1].y - c[1].y) + fabsf(a[2].z - c[2].z) > 0.05f);  // (they do not even start alike)
    // it is a slow thing: a quarter of a second moves no axis by more than a quarter of the way to its neighbour
    tumble(60000.0f, 5, a);
    tumble(60250.0f, 5, b);
    for (int i = 0; i < 3; i++) CHECK(dot(a[i], b[i]) > 0.92f);
  }
  {  // a shove (KEY, while a cube is up): off at a set speed in some direction, dying away; the tumbling goes on under it
    Shove a;
    a.bump(0x12345678u);
    CHECK_NEAR(sqrtf(a.rate[0] * a.rate[0] + a.rate[1] * a.rate[1] + a.rate[2] * a.rate[2]), kShoveRate, 1e-6);
    CHECK(a.angle[0] == 0.0f && a.angle[1] == 0.0f && a.angle[2] == 0.0f);  // (it has not moved yet)
    Shove b = a, first = a;
    a.advance(3000.0f);
    for (int i = 0; i < 300; i++) b.advance(10.0f);  // counted frame by frame it comes to the same
    for (int i = 0; i < 3; i++) CHECK_NEAR(a.angle[i], b.angle[i], 2e-3);
    // by then it has all but stopped, having turned the cube by its speed times the time it takes to fade
    CHECK(sqrtf(a.rate[0] * a.rate[0] + a.rate[1] * a.rate[1] + a.rate[2] * a.rate[2]) < 0.01f * kShoveRate);
    CHECK_NEAR(sqrtf(a.angle[0] * a.angle[0] + a.angle[1] * a.angle[1] + a.angle[2] * a.angle[2]), kShoveRate * kShoveFadeMs, 0.03);
    for (int i = 0; i < 3; i++) CHECK(a.angle[i] * first.rate[i] >= 0.0f);  // ... the way it was shoved
    first.advance(kShoveFadeMs);  // one such stretch on, a third of the speed is left
    CHECK_NEAR(sqrtf(first.rate[0] * first.rate[0] + first.rate[1] * first.rate[1] + first.rate[2] * first.rate[2]), kShoveRate * 0.3679, 1e-5);
    // it is something to see and no more than that: a good part of a turn, less than a whole one; still going
    // after a sixth of a second, over within two
    CHECK(kShoveRate * kShoveFadeMs > 1.0f && kShoveRate * kShoveFadeMs < 6.0f);
    Shove seen;
    seen.bump(0x12345678u);
    seen.advance(150.0f);
    CHECK(sqrtf(seen.rate[0] * seen.rate[0] + seen.rate[1] * seen.rate[1] + seen.rate[2] * seen.rate[2]) > 0.5f * kShoveRate);
    seen.advance(1850.0f);
    CHECK(sqrtf(seen.rate[0] * seen.rate[0] + seen.rate[1] * seen.rate[1] + seen.rate[2] * seen.rate[2]) < 0.05f * kShoveRate);
    // the direction comes with the bits, and over many shoves it is every direction
    float sum[3] = {0, 0, 0};
    int alike = 0;
    uint32_t r = 2463534242u;
    Shove before;
    before.bump(r);
    for (int n = 0; n < 4000; n++) {
      r ^= r << 13;
      r ^= r >> 17;
      r ^= r << 5;
      Shove s;
      s.bump(r);
      for (int i = 0; i < 3; i++) sum[i] += s.rate[i] / kShoveRate;
      if (s.rate[0] == before.rate[0] && s.rate[1] == before.rate[1]) alike++;
      before = s;
    }
    CHECK(alike == 0 && fabsf(sum[0]) < 200 && fabsf(sum[1]) < 200 && fabsf(sum[2]) < 200);
    Shove twice;  // two shoves the same way are twice the shove
    twice.bump(77);
    twice.bump(77);
    CHECK_NEAR(sqrtf(twice.rate[0] * twice.rate[0] + twice.rate[1] * twice.rate[1] + twice.rate[2] * twice.rate[2]), 2 * kShoveRate, 1e-6);
    Shove odd;  // (bits that are all nought are a direction too)
    odd.bump(0);
    CHECK_NEAR(sqrtf(odd.rate[0] * odd.rate[0] + odd.rate[1] * odd.rate[1] + odd.rate[2] * odd.rate[2]), kShoveRate, 1e-6);
    // the cube with a shove in it: still three unit axes at right angles, and somewhere else than without
    V3 plain[3], same[3], moved[3];
    const float none[3] = {0, 0, 0}, some[3] = {0.7f, -1.1f, 2.0f};
    tumble(1234.0f, 5, plain);
    tumble(1234.0f, 5, none, same);
    tumble(1234.0f, 5, some, moved);
    CHECK(plain[0].x == same[0].x && plain[1].y == same[1].y && plain[2].z == same[2].z);
    CHECK(fabsf(plain[0].x - moved[0].x) + fabsf(plain[1].y - moved[1].y) + fabsf(plain[2].z - moved[2].z) > 0.1f);
    for (int i = 0; i < 3; i++) CHECK(fabsf(dot(moved[i], moved[i]) - 1.0f) < 1e-4f && fabsf(dot(moved[i], moved[(i + 1) % 3])) < 1e-4f);
  }
  // the further a reflection, the fainter, down to nothing
  CHECK(fade(1) == SOLID && fade(2) == FADE1 && fade(1000) == NONE);
  for (int b = 1; b < 40; b++) CHECK(fade(b + 1) >= fade(b));
  bool everyStep = true;
  for (int style = SOLID; style <= FADE4; style++) {
    bool found = false;
    for (int b = 1; b < 40; b++) found = found || fade(b) == style;
    everyStep = everyStep && found;
  }
  CHECK(everyStep);
  // the pulse: from the depth to the front, one cube a step, then a rest
  CHECK(pulseAt(0) == kDepth && pulseAt(kPulseStepMs - 1) == kDepth && pulseAt(kPulseStepMs) == kDepth - 1);
  CHECK(pulseAt((kDepth - 1) * kPulseStepMs) == 1 && pulseAt(kDepth * kPulseStepMs) == 0);
  CHECK(pulseAt((kDepth + kPulseRest) * kPulseStepMs) == kDepth && pulseAt((kDepth + kPulseRest) * kPulseStepMs - 1) == 1 - kPulseRest);
  // the window: what is seen from the eye through the square from (-1, -1) to (1, 1), one in front of it
  Window win;
  win.eye = {0, 0, -1};
  const V3 sq[4] = {{1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};
  for (int i = 0; i < 4; i++) {
    V3 m = cross(sub(sq[i], win.eye), sub(sq[(i + 1) & 3], win.eye));
    if (dot(m, sub(V3{0, 0, 0}, win.eye)) < 0) m = mul(m, -1.0f);
    win.normal[i] = m;
  }
  float t0 = -1, t1 = -1;
  CHECK(win.clip({-0.5f, 0, 0}, {1, 0, 0}, &t0, &t1) && t0 == 0.0f && t1 == 1.0f);  // inside from end to end
  CHECK(win.clip({-3, 0, 0}, {6, 0, 0}, &t0, &t1));                                  // across the square: the middle third
  CHECK_NEAR(t0, 1.0 / 3.0, 1e-5);
  CHECK_NEAR(t1, 2.0 / 3.0, 1e-5);
  CHECK(win.clip({-10, 0, 1}, {20, 0, 0}, &t0, &t1));  // one further away, the window is twice as wide: -2 to 2
  CHECK_NEAR(t0, 0.4, 1e-5);
  CHECK_NEAR(t1, 0.6, 1e-5);
  CHECK(win.clip({0, 0, 0}, {0, 9, 0}, &t0, &t1) && t0 == 0.0f);  // from the middle out through the top edge
  CHECK_NEAR(t1, 1.0 / 9.0, 1e-5);
  CHECK(win.clip({0, 0, 0}, {0, 9, 9}, &t0, &t1) && t0 == 0.0f && t1 == 1.0f);  // up and away as steeply as the window widens: never out
  CHECK(!win.clip({2, 2, 0}, {5, 0, 0}, &t0, &t1));    // beside it
  CHECK(!win.clip({-3, 1.5f, 0}, {6, 0, 0}, &t0, &t1));  // past it, above
  CHECK(!win.clip({1.5f, -3, 0}, {0, 6, 0}, &t0, &t1));  // alongside an edge, outside it
}

// ---------------------------------------------------------------------------
// The other cube those buttons bring up (sand_cube.h): sand on the walls of a cube that tumbles.
// ---------------------------------------------------------------------------
static void testSandCube() {
  section("sand cube");
  using namespace sandcube;
  const int want = kCells * kCells * kFillPercent / 100;
  auto same = [](const Sand &a, const Sand &b) {
    for (int f = 0; f < 6; f++)
      for (int j = 0; j < kCells; j++)
        for (int i = 0; i < kCells; i++)
          if (a.has(f, i, j) != b.has(f, i, j)) return false;
    return true;
  };
  {  // the same amount of sand on every face; the same seed gives the same sand, another seed other sand
    Sand a, b, c;
    a.reset(7);
    b.reset(7);
    c.reset(8);
    for (int f = 0; f < 6; f++) CHECK(a.grains(f) == want && c.grains(f) == want);
    CHECK(same(a, b) && !same(a, c));
    V3 ax[3];
    infcube::tumble(1000.0f, 3, ax);
    a.step(ax);
    b.step(ax);
    CHECK(same(a, b));  // ... and runs the same way
  }
  {  // gravity down one side: the sand ends up there, packed, and none of it is lost on the way
    Sand s;
    s.reset(3);
    float a0, b0, a1, b1;
    s.centre(2, &a0, &b0);
    CHECK(fabsf(a0) < 0.15f && fabsf(b0) < 0.15f);  // scattered evenly to begin with
    for (int k = 0; k < 600; k++) s.stepFace(2, 0.0f, -1.0f);
    s.centre(2, &a1, &b1);
    CHECK(s.grains(2) == want);
    CHECK(b1 < -0.55f && fabsf(a1) < 0.1f);
    int fullRows = 0, strays = 0;
    for (int j = 0; j < kCells; j++) {
      int n = 0;
      for (int i = 0; i < kCells; i++) n += s.has(2, i, j) ? 1 : 0;
      if (n == kCells && j == fullRows) fullRows++;
      if (j >= want / kCells + 3) strays += n;
    }
    CHECK(fullRows >= want / kCells - 2 && strays == 0);  // a level bed, but for a grain or two on top
    // the other faces were not touched
    Sand fresh;
    fresh.reset(3);
    for (int f = 0; f < 6; f++) {
      if (f == 2) continue;
      float fa, fb, ga, gb;
      s.centre(f, &fa, &fb);
      fresh.centre(f, &ga, &gb);
      CHECK(fa == ga && fb == gb);
    }
    // then the other way: it pours back, all of it
    for (int k = 0; k < 600; k++) s.stepFace(2, 0.0f, 1.0f);
    s.centre(2, &a1, &b1);
    CHECK(s.grains(2) == want && b1 > 0.55f);
    // along the other side, and into a corner
    for (int k = 0; k < 600; k++) s.stepFace(2, 1.0f, 0.0f);
    s.centre(2, &a1, &b1);
    CHECK(s.grains(2) == want && a1 > 0.55f && fabsf(b1) < 0.25f);
    for (int k = 0; k < 600; k++) s.stepFace(2, -0.7f, -0.7f);
    s.centre(2, &a1, &b1);
    CHECK(s.grains(2) == want && a1 < -0.3f && b1 < -0.3f);
  }
  {  // a face that lies flat keeps its sand where it is; on a gentle slope it creeps
    Sand s, before;
    s.reset(5);
    before.reset(5);
    for (int k = 0; k < 200; k++) s.stepFace(0, kFlat * 0.6f, -kFlat * 0.6f);
    CHECK(same(s, before));
    float a0, b0, a1, b1, a2, b2;
    before.centre(0, &a0, &b0);
    for (int k = 0; k < 10; k++) s.stepFace(0, 0.0f, -0.2f);
    s.centre(0, &a1, &b1);
    Sand steep;
    steep.reset(5);
    for (int k = 0; k < 10; k++) steep.stepFace(0, 0.0f, -1.0f);
    steep.centre(0, &a2, &b2);
    CHECK(b1 < b0 - 0.01f && b2 < b1 - 0.05f);  // ten steps: a little way on the slope, a long way straight down
  }
  {  // the cube tumbles for ten minutes: every face keeps its sand, and every face has had it at more than one side
    Sand s;
    s.reset(11);
    float lowA[6], highA[6], lowB[6], highB[6];
    for (int f = 0; f < 6; f++) lowA[f] = lowB[f] = 9, highA[f] = highB[f] = -9;
    for (uint32_t ms = 0; ms < 600000; ms += kStepMs) {
      V3 ax[3];
      infcube::tumble((float)ms, 11, ax);
      s.advance(ms, ax);
      if (ms % (kStepMs * 20) != 0) continue;
      for (int f = 0; f < 6; f++) {
        float a, b;
        s.centre(f, &a, &b);
        if (a < lowA[f]) lowA[f] = a;
        if (a > highA[f]) highA[f] = a;
        if (b < lowB[f]) lowB[f] = b;
        if (b > highB[f]) highB[f] = b;
      }
    }
    int kept = 0, sloshed = 0;
    for (int f = 0; f < 6; f++) {
      kept += s.grains(f) == want ? 1 : 0;
      sloshed += (highA[f] - lowA[f] > 0.6f || highB[f] - lowB[f] > 0.6f) ? 1 : 0;
    }
    CHECK(kept == 6 && sloshed == 6);
  }
  {  // "down" is the bottom of the screen: with the cube held as it starts, the sand of the face in front ends up low on it
    struct Mean {
      long sumY = 0, n = 0;
      void grain(int, int y, int) {
        sumY += y;
        n++;
      }
      void line(int, int, int, int, infcube::Style) {}
      void dot(int, int) {}
    };
    const V3 square[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Sand s;
    s.reset(6);
    Mean before, after;
    s.draw(square, 400, 300, before);
    for (int k = 0; k < 600; k++) s.step(square);
    s.draw(square, 400, 300, after);
    CHECK(before.n == want && after.n == want);  // one face is seen, square on
    CHECK(labs(before.sumY / before.n - 150) < 12 && after.sumY / after.n > 200);
  }
  {  // advance(): a step every kStepMs, a late call is not made up for, time that starts again is taken as it comes
    V3 ax[3];
    infcube::tumble(5000.0f, 2, ax);  // (tipped, so that the sand has somewhere to run)
    Sand a, b, start;
    a.reset(9);
    b.reset(9);
    start.reset(9);
    a.advance(0, ax);
    CHECK(same(a, b));  // nothing is due at the start
    a.advance(kStepMs - 1, ax);
    CHECK(same(a, b));
    a.advance(kStepMs, ax);
    b.step(ax);
    CHECK(same(a, b) && !same(a, start));
    a.advance(kStepMs * 3, ax);  // two more
    b.step(ax);
    b.step(ax);
    CHECK(same(a, b));
    a.advance(kStepMs * 1000, ax);  // very late: only a few steps are taken, not a thousand
    for (int k = 0; k < kMaxStepsAtOnce; k++) b.step(ax);
    CHECK(same(a, b));
    a.advance(kStepMs * 2, ax);  // the time started again: nothing is taken, and it goes on from there
    CHECK(same(a, b));
    a.advance(kStepMs * 3, ax);
    b.step(ax);
    CHECK(same(a, b));
  }
  {  // the picture: the sand of the faces that are seen, their edges after it, all of it on the screen
    struct Count {
      int grains = 0, bold = 0, dots = 0, outside = 0, lateGrains = 0, tiny = 0, huge = 0;
      void see(int x, int y, int r) {
        if (x - r < 8 || x + r > 391 || y - r < 8 || y + r > 291) outside++;
      }
      void grain(int x, int y, int size) {
        grains++;
        if (bold) lateGrains++;
        if (size < 1) tiny++;
        if (size > 9) huge++;
        see(x, y, size / 2);
      }
      void line(int x0, int y0, int x1, int y1, infcube::Style style) {
        if (style == infcube::BOLD) bold++;
        see(x0, y0, 1);
        see(x1, y1, 1);
      }
      void dot(int x, int y) {
        dots++;
        see(x, y, 3);
      }
    };
    Sand s;
    s.reset(4);
    int bad = 0, faces[4] = {};
    for (uint32_t ms = 0; ms < 300000; ms += 149) {
      V3 ax[3];
      infcube::tumble((float)ms, 4, ax);
      s.advance(ms, ax);
      Count c;
      s.draw(ax, 400, 300, c);
      const int seen = c.bold / 4;
      if (c.bold % 4 != 0 || seen < 1 || seen > 3 || c.dots != c.bold || c.grains != seen * want) {
        bad++;
      } else {
        faces[seen]++;
      }
      if (c.outside || c.lateGrains || c.tiny || c.huge) bad++;
    }
    CHECK(bad == 0 && faces[2] > 0 && faces[3] > 0);
  }
}

int main() {
  testCalendar();
  testTimeZones();
  testSensorMaths();
  testBatteryCalibration();
  testCharge();
  testEstimateUnderLightLoad();
  testFramePlan();
  testFramePlanPanel();
  testWeather();
  testSpotify();
  testUtil();
  testLinkPage();
  testButtons();
  testButtonEdges();
  testHoldRelease();
  testChord();
  testInfinityCube();
  testSandCube();
  printf("\n%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
