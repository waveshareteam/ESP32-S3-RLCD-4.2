#include "weather.h"

#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

#include "timeutil.h"
#include "util.h"

namespace {

// "2026-10-04" -> weekday 0..6 (0 = Sunday); false if malformed.
bool weekdayOfIsoDate(const char *date, uint8_t *wday) {
  int y, m, d;
  if (!date || sscanf(date, "%d-%d-%d", &y, &m, &d) != 3) return false;
  if (m < 1 || m > 12 || d < 1 || d > 31) return false;
  *wday = (uint8_t)timeutil::weekday(y, m, d);
  return true;
}

// "2026-10-04T05:52" -> "05:52"
void timeOfIsoDateTime(const char *s, char *out, size_t cap) {
  const char *t = s ? strchr(s, 'T') : nullptr;
  if (t && strlen(t + 1) >= 5) {
    snprintf(out, cap, "%.5s", t + 1);
  } else {
    copyStr(out, cap, "--:--");
  }
}

uint8_t clampPct(int v) { return (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v)); }

}  // namespace

bool parseOpenMeteoForecast(const char *json, size_t len, WeatherData *out, ZoneInfo *zone) {
  JsonDocument doc;
  if (deserializeJson(doc, json, len)) return false;

  JsonObjectConst cur = doc["current"];
  JsonObjectConst daily = doc["daily"];
  if (cur.isNull() || daily.isNull()) return false;
  // Everything the clock shows as a fact about now has to be in the reply.  A missing field would
  // otherwise become a plausible default (a clear sky, 0 % humidity, no wind) that is simply wrong.
  static const char *const kCurrent[] = {"temperature_2m",   "apparent_temperature", "relative_humidity_2m",
                                         "weather_code",     "wind_speed_10m",       "is_day"};
  for (const char *key : kCurrent)
    if (cur[key].isNull()) return false;

  WeatherData w;
  w.valid = true;
  w.temp = cur["temperature_2m"] | 0.0f;
  w.feels = cur["apparent_temperature"] | w.temp;
  w.windKmh = cur["wind_speed_10m"] | 0.0f;
  w.humidity = clampPct(cur["relative_humidity_2m"] | 0);
  w.code = (uint8_t)(cur["weather_code"] | 0);
  w.isDay = (cur["is_day"] | 1) != 0;

  JsonArrayConst times = daily["time"];
  JsonArrayConst codes = daily["weather_code"];
  JsonArrayConst tmax = daily["temperature_2m_max"];
  JsonArrayConst tmin = daily["temperature_2m_min"];
  JsonArrayConst pop = daily["precipitation_probability_max"];
  JsonArrayConst rise = daily["sunrise"];
  JsonArrayConst set = daily["sunset"];
  JsonArrayConst uv = daily["uv_index_max"];
  // The dates, conditions and temperatures of all three days are needed.  The rain chance, sunrise,
  // sunset and UV are not: the service leaves them null where it has no figure (no probability model for
  // a region, a polar day), and the clock then shows 0 %, "--:--" and 0.
  if (times.size() < 3 || codes.size() < 3 || tmax.size() < 3 || tmin.size() < 3) return false;

  for (int i = 0; i < 3; i++) {
    WeatherDay &d = w.day[i];
    if (codes[i].isNull() || tmax[i].isNull() || tmin[i].isNull()) return false;
    d.code = (uint8_t)(codes[i] | 0);
    d.tmax = tmax[i] | 0.0f;
    d.tmin = tmin[i] | 0.0f;
    d.rainPct = clampPct(pop[i] | 0);
    if (!weekdayOfIsoDate(times[i] | "", &d.weekday)) return false;
  }
  timeOfIsoDateTime(rise[0] | "", w.sunrise, sizeof w.sunrise);
  timeOfIsoDateTime(set[0] | "", w.sunset, sizeof w.sunset);
  w.uvMax = uv[0] | 0.0f;

  *out = w;
  if (zone) {
    copyStr(zone->iana, sizeof zone->iana, doc["timezone"] | "");
    zone->utcOffsetSec = doc["utc_offset_seconds"] | 0;
    // A zone name that is not in the clock's table is turned into a fixed offset, so a reply that names
    // a zone but gives no offset must not count as one: it would mean UTC.  (No zone at all leaves the
    // clock's zone alone.)
    if (doc["utc_offset_seconds"].isNull()) zone->iana[0] = 0;
  }
  return true;
}

bool parseOpenMeteoGeocode(const char *json, size_t len, GeoResult *out) {
  JsonDocument doc;
  if (deserializeJson(doc, json, len)) return false;
  JsonObjectConst r = doc["results"][0];
  if (r.isNull() || r["latitude"].isNull() || r["longitude"].isNull()) return false;

  GeoResult g;
  g.lat = r["latitude"] | 0.0;
  g.lon = r["longitude"] | 0.0;
  copyStr(g.name, sizeof g.name, r["name"] | "");
  copyStr(g.admin1, sizeof g.admin1, r["admin1"] | "");
  copyStr(g.country, sizeof g.country, r["country_code"] | "");
  copyStr(g.iana, sizeof g.iana, r["timezone"] | "");
  *out = g;
  return true;
}
