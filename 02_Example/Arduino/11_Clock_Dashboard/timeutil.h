#pragma once

// Small calendar helpers.  Header-only and free of Arduino dependencies so the
// host-side tests (tools/tests) can exercise them.

#include <stdint.h>
#include <stdio.h>
#include <time.h>

namespace timeutil {

// Offset of the current time zone (set through TZ / tzset()) from UTC, in
// minutes, at instant t.  Includes any daylight saving in effect.
inline int utcOffsetMinutes(time_t t) {
  struct tm lt, gt;
  localtime_r(&t, &lt);
  gmtime_r(&t, &gt);
  int dayDiff = lt.tm_yday - gt.tm_yday;  // -1, 0 or +1 (or a year wrap)
  if (dayDiff > 1) dayDiff = -1;
  if (dayDiff < -1) dayDiff = 1;
  return dayDiff * 1440 + (lt.tm_hour - gt.tm_hour) * 60 + (lt.tm_min - gt.tm_min);
}

inline bool isLeap(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

// 0 = Sunday ... 6 = Saturday (Sakamoto's method).  month is 1..12.
inline int weekday(int year, int month, int day) {
  static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (month < 3) year -= 1;
  return (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7;
}

inline int weeksInIsoYear(int y) {
  auto p = [](int yy) { return (yy + yy / 4 - yy / 100 + yy / 400) % 7; };
  return 52 + ((p(y) == 4 || p(y - 1) == 3) ? 1 : 0);
}

// ISO 8601 week number.  yday0 is tm_yday (0-based), wday0 is tm_wday (0 = Sunday).
inline int isoWeek(int year, int yday0, int wday0) {
  int wd = (wday0 == 0) ? 7 : wday0;  // Monday = 1 ... Sunday = 7
  int week = (yday0 + 1 - wd + 10) / 7;
  if (week < 1) return weeksInIsoYear(year - 1);
  if (week == 53 && weeksInIsoYear(year) == 52) return 1;
  return week;
}

// "+08:00" / "-03:30" / "+00:00"
inline void formatUtcOffset(int minutes, char *out, size_t cap) {
  char sign = minutes < 0 ? '-' : '+';
  int a = minutes < 0 ? -minutes : minutes;
  snprintf(out, cap, "%c%02d:%02d", sign, (a / 60) % 100, a % 60);
}

inline const char *weekdayName(int wday0) {
  static const char *const n[] = {"Sunday", "Monday", "Tuesday", "Wednesday",
                                  "Thursday", "Friday", "Saturday"};
  return n[((wday0 % 7) + 7) % 7];
}

inline const char *weekdayShort(int wday0) {
  static const char *const n[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  return n[((wday0 % 7) + 7) % 7];
}

inline const char *monthShort(int mon0) {
  static const char *const n[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  return n[((mon0 % 12) + 12) % 12];
}

// "m:ss" or "h:mm:ss"
inline void formatDuration(uint32_t ms, char *out, size_t cap) {
  uint32_t s = ms / 1000;
  uint32_t h = s / 3600, m = (s / 60) % 60, sec = s % 60;
  if (h > 0) {
    snprintf(out, cap, "%u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)sec);
  } else {
    snprintf(out, cap, "%u:%02u", (unsigned)m, (unsigned)sec);
  }
}

}  // namespace timeutil
