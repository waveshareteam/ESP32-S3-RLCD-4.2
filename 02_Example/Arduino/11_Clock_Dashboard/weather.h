#pragma once

// Parsers for Open-Meteo replies.  Pure functions (ArduinoJson only), so they
// are covered by the host-side tests in tools/tests.

#include <stddef.h>

#include "app_model.h"

struct ZoneInfo {
  char iana[48] = "";     // e.g. "Australia/Perth"
  int utcOffsetSec = 0;   // offset at the time of the request
};

// Parses a /v1/forecast reply requested with:
//   current=temperature_2m,relative_humidity_2m,apparent_temperature,is_day,weather_code,wind_speed_10m
//   daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max,
//         sunrise,sunset,uv_index_max
//   timezone=auto&forecast_days=3
// Returns false if the reply is malformed or incomplete.
bool parseOpenMeteoForecast(const char *json, size_t len, WeatherData *out, ZoneInfo *zone);

struct GeoResult {
  double lat = 0;
  double lon = 0;
  char name[40] = "";
  char admin1[40] = "";
  char country[8] = "";
  char iana[48] = "";
};

// Parses the first hit of a geocoding-api.open-meteo.com/v1/search reply.
bool parseOpenMeteoGeocode(const char *json, size_t len, GeoResult *out);
