#pragma once

// WMO weather interpretation codes as used by Open-Meteo
// (https://open-meteo.com/en/docs#weathervariables).

#include <stdint.h>

enum WxIcon : uint8_t {
  WX_CLEAR = 0,   // sun by day, moon by night
  WX_PARTLY,      // sun + cloud
  WX_CLOUDY,
  WX_FOG,
  WX_DRIZZLE,
  WX_RAIN,
  WX_SNOW,
  WX_THUNDER
};

inline WxIcon wmoIcon(int code) {
  switch (code) {
    case 0: return WX_CLEAR;
    case 1:
    case 2: return WX_PARTLY;
    case 3: return WX_CLOUDY;
    case 45:
    case 48: return WX_FOG;
    case 51:
    case 53:
    case 55:
    case 56:
    case 57: return WX_DRIZZLE;
    case 61:
    case 63:
    case 65:
    case 66:
    case 67:
    case 80:
    case 81:
    case 82: return WX_RAIN;
    case 71:
    case 73:
    case 75:
    case 77:
    case 85:
    case 86: return WX_SNOW;
    case 95:
    case 96:
    case 99: return WX_THUNDER;
    default: return WX_CLOUDY;
  }
}

inline const char *wmoText(int code) {
  switch (code) {
    case 0: return "Clear";
    case 1: return "Mostly clear";
    case 2: return "Partly cloudy";
    case 3: return "Overcast";
    case 45: return "Fog";
    case 48: return "Rime fog";
    case 51: return "Light drizzle";
    case 53: return "Drizzle";
    case 55: return "Heavy drizzle";
    case 56:
    case 57: return "Icy drizzle";
    case 61: return "Light rain";
    case 63: return "Rain";
    case 65: return "Heavy rain";
    case 66:
    case 67: return "Freezing rain";
    case 71: return "Light snow";
    case 73: return "Snow";
    case 75: return "Heavy snow";
    case 77: return "Snow grains";
    case 80: return "Light showers";
    case 81: return "Showers";
    case 82: return "Heavy showers";
    case 85:
    case 86: return "Snow showers";
    case 95: return "Thunderstorm";
    case 96:
    case 99: return "Storm + hail";
    default: return "Unknown";
  }
}
