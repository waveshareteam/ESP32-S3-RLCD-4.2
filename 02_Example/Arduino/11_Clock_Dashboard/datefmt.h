#pragma once

// Date and time text in the formats the settings offer.  Header-only and free of Arduino
// dependencies so tools/tests and the UI preview can use it.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "settings.h"  // DateFormat
#include "timeutil.h"  // monthShort

namespace datefmt {

// The 12 hour clock face: 0 -> 12 AM, 1 -> 1 AM, 12 -> 12 PM, 13 -> 1 PM, 23 -> 11 PM.
inline int hour12(int hour24, bool *pm) {
  const int h = ((hour24 % 24) + 24) % 24;
  if (pm) *pm = h >= 12;
  const int h12 = h % 12;
  return h12 == 0 ? 12 : h12;
}

// month 1..12.  Numeric parts are zero padded to keep a column's width fixed, except in the
// formats with a month name, where "4 Oct 2026" is how people write it.
//   DATE_ISO        2026-10-04
//   DATE_DMY        04/10/2026
//   DATE_MDY        10/04/2026
//   DATE_DMY_DOT    04.10.2026
//   DATE_D_MON_Y    4 Oct 2026
//   DATE_MON_D_Y    Oct 4, 2026
inline void formatDate(int year, int month, int day, uint8_t format, char *out, size_t cap) {
  switch (format) {
    case DATE_DMY: snprintf(out, cap, "%02d/%02d/%04d", day, month, year); break;
    case DATE_MDY: snprintf(out, cap, "%02d/%02d/%04d", month, day, year); break;
    case DATE_DMY_DOT: snprintf(out, cap, "%02d.%02d.%04d", day, month, year); break;
    case DATE_D_MON_Y: snprintf(out, cap, "%d %s %04d", day, timeutil::monthShort(month - 1), year); break;
    case DATE_MON_D_Y: snprintf(out, cap, "%s %d, %04d", timeutil::monthShort(month - 1), day, year); break;
    case DATE_ISO:
    default: snprintf(out, cap, "%04d-%02d-%02d", year, month, day); break;
  }
}

// "9:45:21 PM" style pieces for the 12 hour layout.
inline const char *ampm(bool pm) { return pm ? "PM" : "AM"; }

}  // namespace datefmt
