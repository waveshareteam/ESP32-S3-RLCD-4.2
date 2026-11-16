// Host-side tests for the settings layer and the other pure-logic modules added in 1.3
// (moon phase, date formats, battery estimate, low-battery guard, clock drift).
// Built and run by run_tests.sh next to test_logic.cpp.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "all_settings.h"
#include "battery_est.h"
#include "calc.h"
#include "check.h"
#include "datefmt.h"
#include "drift.h"
#include "fw_logic.h"
#include "low_battery.h"
#include "moon.h"
#include "power_log.h"
#include "sd_layout.h"
#include "settings.h"
#include "tz_table.h"
#include "wifi_pick.h"

// ===========================================================================
// Settings
// ===========================================================================
namespace {

struct Applied {
  Settings s;
  ConfigReport r;
};

// Applies `text` to `base`, with a pristine Settings as the defaults.
Applied apply(const std::string &text, const Settings &base = Settings()) {
  Applied a;
  a.s = base;
  const Settings defaults;
  a.r = applyConfigText(a.s, defaults, text.data(), text.size());
  return a;
}

std::string utf16(const std::string &ascii, bool bigEndian) {
  std::string out = bigEndian ? std::string("\xFE\xFF", 2) : std::string("\xFF\xFE", 2);
  for (char c : ascii) {
    if (bigEndian) {
      out += '\0';
      out += c;
    } else {
      out += c;
      out += '\0';
    }
  }
  return out;
}

std::string fmt(const Settings &s, size_t i) {
  char buf[128];
  return formatSetting(s, i, buf, sizeof buf) ? buf : "<overflow>";
}

// The table position of a setting, by the name it has in the file (the tests must not depend on the order).
size_t idx(const char *key) {
  for (size_t i = 0; i < settingCount(); i++)
    if (!strcmp(settingKey(i), key)) return i;
  return (size_t)-1;
}

// Every field of every setting as text, to compare two Settings.
std::string dump(const Settings &s) {
  std::string out;
  for (size_t i = 0; i < settingCount(); i++) {
    out += settingKey(i);
    out += '=';
    out += fmt(s, i);
    out += '\n';
  }
  return out;
}

// (kAllSettings, a file that sets every setting to something other than its default, is in all_settings.h)

void testSettingsTable() {
  section("settings table");
  const size_t n = settingCount();
  CHECK(n == 34);
  CHECK(n <= 64);  // userSet is a 64 bit mask
  CHECK_STR(settingKey(n), "");
  CHECK_STR(settingNvsKey(n), "");
  for (size_t i = 0; i < n; i++) {
    CHECK(strlen(settingKey(i)) > 0);
    CHECK(strlen(settingNvsKey(i)) > 0 && strlen(settingNvsKey(i)) <= 15);  // NVS key limit
    for (size_t j = i + 1; j < n; j++) {
      CHECK(strcmp(settingKey(i), settingKey(j)) != 0);
      CHECK(strcmp(settingNvsKey(i), settingNvsKey(j)) != 0);
    }
  }
  int secrets = 0;
  for (size_t i = 0; i < n; i++) secrets += settingIsSecret(i);
  CHECK(secrets == 3);
  CHECK(settingIsSecret(2) && !strcmp(settingKey(2), "wifi_password"));
  CHECK(!settingIsSecret(3) && !strcmp(settingKey(3), "wifi_backup_ssid"));
  CHECK(settingIsSecret(4) && !strcmp(settingKey(4), "wifi_backup_password"));
  CHECK(settingIsSecret(idx("spotify_client_id")));

  // defaults a clean build ships with
  const Settings d;
  CHECK(d.wifi && d.units == UNITS_METRIC && d.timeFormat == TIME_24H && d.dateFormat == DATE_ISO);
  CHECK(d.cpuMhz() == 80 && d.lowBatteryShutdown && d.hasBattery() && !d.hasLatLon());
  CHECK_NEAR(d.batteryCutoffV, 3.30, 1e-6);
  CHECK(d.userSet == 0);
}

void testSettingsRoundTrip() {
  section("settings flash round trip");
  const Settings d;
  for (size_t i = 0; i < settingCount(); i++) {
    Settings t;
    CHECK(applyStoredSetting(t, i, fmt(d, i).c_str()));
    CHECK(settingIsUserSet(t, i));
    CHECK(fmt(t, i) == fmt(d, i));
  }

  const Applied all = apply(kAllSettings);
  CHECK(all.r.problems() == 0);
  CHECK(all.r.changed == (int)settingCount());
  CHECK(all.s.userSet == (uint64_t)((1ull << settingCount()) - 1));
  Settings t;
  for (size_t i = 0; i < settingCount(); i++) CHECK(applyStoredSetting(t, i, fmt(all.s, i).c_str()));
  CHECK(dump(t) == dump(all.s));
  CHECK(t.userSet == all.s.userSet);

  // the settings past bit 31 of the mask (it was 32 bits up to 1.9, with 32 settings): set, saved, read back and
  // forgotten like any other, and on their own bit
  CHECK(settingCount() > 32);
  for (size_t i = 31; i < settingCount(); i++) {
    Settings w;
    CHECK(applyStoredSetting(w, i, fmt(all.s, i).c_str()));
    CHECK(settingIsUserSet(w, i) && w.userSet == (1ull << i));
    for (size_t j = 0; j < settingCount(); j++) CHECK(settingIsUserSet(w, j) == (j == i));
    const ApplyResult r = applySetting(w, Settings(), settingKey(i), "");
    CHECK(r == ApplyResult::Cleared && w.userSet == 0 && fmt(w, i) == fmt(Settings(), i));
  }
  {  // what a 1.9 clock saved in its flash (one key per setting, the mask is not kept) still loads, and the new
     // setting is at its default until something sets it
    Settings old;
    for (size_t i = 0; i < settingCount(); i++) {
      if (!strcmp(settingKey(i), "light_sleep")) continue;
      CHECK(applyStoredSetting(old, i, fmt(all.s, i).c_str()));
    }
    CHECK(!old.lightSleep && !settingIsUserSet(old, idx("light_sleep")));
    CHECK(old.userSet == (uint64_t)(((1ull << settingCount()) - 1) & ~(1ull << idx("light_sleep"))));
    CHECK(old.weatherIntervalMin == all.s.weatherIntervalMin && settingIsUserSet(old, idx("weather_interval_min")));
  }
  CHECK(!Settings().lightSleep && all.s.lightSleep);
  CHECK_STR(settingNvsKey(idx("light_sleep")), "lsleep");
  {  // ... and what a clock saved that had light_sleep but not yet display_power: that one at its default, the rest as saved
    Settings old;
    for (size_t i = 0; i < settingCount(); i++) {
      if (!strcmp(settingKey(i), "display_power")) continue;
      CHECK(applyStoredSetting(old, i, fmt(all.s, i).c_str()));
    }
    CHECK(old.displayPower == DISPPOWER_NORMAL && !settingIsUserSet(old, idx("display_power")));
    CHECK(old.lightSleep && settingIsUserSet(old, idx("light_sleep")));
    CHECK(old.userSet == (uint64_t)(((1ull << settingCount()) - 1) & ~(1ull << idx("display_power"))));
  }
  CHECK(Settings().displayPower == DISPPOWER_NORMAL && all.s.displayPower == DISPPOWER_LOW);
  CHECK_STR(settingNvsKey(idx("display_power")), "dpower");
  {  // its words: normal (also high, hpm, default) and low (also lpm, saver); on and off say nothing here, so they are refused
    for (const char *w : {"normal", "Normal", "high", "HPM", "default"}) CHECK(apply(std::string("display_power = low\ndisplay_power = ") + w).s.displayPower == DISPPOWER_NORMAL);
    for (const char *w : {"low", "LOW", "lpm", "saver"}) CHECK(apply(std::string("display_power = ") + w).s.displayPower == DISPPOWER_LOW);
    for (const char *w : {"on", "off", "8", "lowest"}) {
      const Applied a = apply(std::string("display_power = ") + w);
      CHECK(a.r.problems() == 1 && a.s.displayPower == DISPPOWER_NORMAL);
    }
    CHECK(fmt(all.s, idx("display_power")) == "low" && fmt(Settings(), idx("display_power")) == "normal");
  }

  // stored text that no longer validates is ignored, and leaves the setting alone
  Settings u;
  CHECK(!applyStoredSetting(u, 0, "maybe"));
  CHECK(!applyStoredSetting(u, idx("latitude"), "500"));  // out of range
  CHECK(!applyStoredSetting(u, idx("units"), "kelvin"));
  CHECK(!applyStoredSetting(u, idx("units"), ""));        // an empty number or enum is no value
  CHECK(!applyStoredSetting(u, 999, "x"));
  CHECK(!applyStoredSetting(u, 0, nullptr));
  CHECK(u.userSet == 0 && u.wifi && u.units == UNITS_METRIC);
  // but an empty text setting is a real choice (an open network, for one)
  Settings v;
  snprintf(v.wifiPassword, sizeof v.wifiPassword, "old");
  CHECK(applyStoredSetting(v, 2, ""));
  CHECK_STR(v.wifiPassword, "");
  CHECK(settingIsUserSet(v, 2));
  // precision survives: coordinates keep 5 decimals, about a metre
  Settings w;
  w.latitude = -31.952240;
  w.longitude = 115.861456;
  Settings x;
  CHECK(applyStoredSetting(x, idx("latitude"), fmt(w, idx("latitude")).c_str()));
  CHECK(applyStoredSetting(x, idx("longitude"), fmt(w, idx("longitude")).c_str()));
  CHECK_NEAR(x.latitude, -31.952240, 1e-6);
  CHECK_NEAR(x.longitude, 115.861456, 1e-5);
}

void testSettingsSyntax() {
  section("settings file: what people type");
  // all of these say the same thing
  const char *variants[] = {
      "wifi_ssid = My Network",
      "wifi_ssid=My Network",
      "wifi_ssid   =    My Network   ",
      "\twifi_ssid\t=\tMy Network\t",
      "wifi_ssid: My Network",
      "wifi_ssid = \"My Network\"",
      "wifi_ssid = 'My Network'",
      "wifi_ssid=\"My Network\"",
      "WIFI_SSID = My Network",
      "WiFi SSID = My Network",
      "Wi-Fi-SSID = \"My Network\"",
      "wifi-ssid = My Network",
      "wifissid = My Network",
      "ssid = My Network",
      "SSID: \"My Network\"",
      "network = My Network",
      "wifi_ssid = \xE2\x80\x9CMy Network\xE2\x80\x9D",  // curly double quotes
      "wifi_ssid = \xE2\x80\x98My Network\xE2\x80\x99",  // curly single quotes
      "wifi_ssid = \xC2\xABMy Network\xC2\xBB",          // guillemets
      "wifi_ssid = \"My Network\" # my home",
      "wifi_ssid = My Network # my home",
      "wifi_ssid = My Network   ;",  // no: ';' is no comment for bare values; checked below
  };
  for (size_t i = 0; i + 1 < sizeof variants / sizeof variants[0]; i++) {
    const Applied a = apply(variants[i]);
    if (strcmp(a.s.wifiSsid, "My Network") != 0) printf("  variant %zu: [%s] gave [%s]\n", i, variants[i], a.s.wifiSsid);
    CHECK_STR(a.s.wifiSsid, "My Network");
    CHECK(a.r.problems() == 0 && a.r.changed == 1);
    CHECK(settingIsUserSet(a.s, 1));
  }
  CHECK_STR(apply(variants[sizeof variants / sizeof variants[0] - 1]).s.wifiSsid, "My Network   ;");

  // quotes and the characters inside them
  struct Case {
    const char *line;
    const char *ssid;
  } cases[] = {
      {"wifi_ssid = \"Bob's WiFi\"", "Bob's WiFi"},
      {"wifi_ssid = Bob's WiFi", "Bob's WiFi"},
      {"wifi_ssid = 'Bob\"s WiFi'", "Bob\"s WiFi"},
      {"wifi_ssid = \"Home\" # it's the \"good\" one", "Home"},
      {"wifi_ssid = \"Home\" ; the good one", "Home"},
      {"wifi_ssid = \"My \"Network\"\"", "My \"Network\""},
      {"wifi_ssid = \"Home#1\"", "Home#1"},
      {"wifi_ssid = Home#1", "Home#1"},
      {"wifi_ssid = Home #1", "Home"},
      {"wifi_ssid = \"  padded  \"", "  padded  "},
      {"wifi_ssid = \"unterminated", "unterminated"},
      {"wifi_ssid = 'tis", "'tis"},
      {"wifi_ssid = \"Caf\xC3\xA9 Net\"", "Caf\xC3\xA9 Net"},
      {"wifi_ssid = a=b:c", "a=b:c"},
      {"wifi_ssid : a=b:c", "a=b:c"},
      {"wifi_ssid = 1234 # 5678", "1234"},
      {"wifi_ssid = \"x\"y\"", "x\"y"},
  };
  for (const Case &c : cases) {
    const Applied a = apply(c.line);
    if (strcmp(a.s.wifiSsid, c.ssid) != 0) printf("  [%s] gave [%s], wanted [%s]\n", c.line, a.s.wifiSsid, c.ssid);
    CHECK_STR(a.s.wifiSsid, c.ssid);
    CHECK(a.r.problems() == 0);
  }

  // the password: bare, spaces inside, empty on purpose, forgotten
  CHECK_STR(apply("wifi_password = p@ss w0rd!").s.wifiPassword, "p@ss w0rd!");
  CHECK_STR(apply("password = \"p@ss w0rd!\"").s.wifiPassword, "p@ss w0rd!");
  CHECK_STR(apply("wifi_password = Pa$$ #1 word").s.wifiPassword, "Pa$$");  // a bare ' #' starts a comment
  CHECK_STR(apply("wifi_password = \"Pa$$ #1 word\"").s.wifiPassword, "Pa$$ #1 word");
  Settings withPassword;
  snprintf(withPassword.wifiPassword, sizeof withPassword.wifiPassword, "old");
  withPassword.userSet |= 1ull << 2;
  Applied open = apply("wifi_password = \"\"", withPassword);
  CHECK_STR(open.s.wifiPassword, "");
  CHECK(settingIsUserSet(open.s, 2) && open.r.changed == 1);
  Applied open2 = apply("wifi_password = ''", withPassword);
  CHECK_STR(open2.s.wifiPassword, "");
  Applied forgot = apply("wifi_password =", withPassword);
  CHECK_STR(forgot.s.wifiPassword, "");  // the default is no password
  CHECK(!settingIsUserSet(forgot.s, 2) && forgot.r.cleared == 1 && forgot.r.changed == 0);

  // line ends and encodings
  const std::string two = "wifi_ssid = Net\nwifi_password = pw\n";
  const char *ends[] = {"\n", "\r\n", "\r"};
  for (const char *e : ends) {
    std::string t = std::string("wifi_ssid = Net") + e + "wifi_password = pw" + e;
    Applied a = apply(t);
    CHECK_STR(a.s.wifiSsid, "Net");
    CHECK_STR(a.s.wifiPassword, "pw");
    CHECK(a.r.lines == 2 && a.r.changed == 2);
  }
  Applied noEnd = apply("wifi_ssid = Net\nwifi_password = pw");
  CHECK_STR(noEnd.s.wifiPassword, "pw");
  CHECK(noEnd.r.lines == 2);
  Applied bom = apply(std::string("\xEF\xBB\xBF") + two);
  CHECK_STR(bom.s.wifiSsid, "Net");
  CHECK(bom.r.problems() == 0);
  Applied le = apply(utf16(two, false));
  CHECK_STR(le.s.wifiSsid, "Net");
  CHECK_STR(le.s.wifiPassword, "pw");
  CHECK(le.r.problems() == 0);
  Applied be = apply(utf16(two, true));
  CHECK_STR(be.s.wifiSsid, "Net");
  CHECK_STR(be.s.wifiPassword, "pw");
  Applied le16crlf = apply(utf16("wifi_ssid = Net\r\nunits = imperial\r\n", false));
  CHECK_STR(le16crlf.s.wifiSsid, "Net");
  CHECK(le16crlf.s.imperial());
  CHECK(apply("").r.lines == 0 && apply("").r.touched() == 0);
  CHECK(apply("\n\n\n").r.lines == 3 && apply("\n\n\n").r.problems() == 0);
  CHECK(apply("# only a comment\n; and another\n// and a third\n[wifi]\n").r.problems() == 0);

  // comments and section headings are skipped
  Applied commented = apply("# wifi_ssid = Nope\n; units = imperial\n// time_format = 12h\n[Units]\nunits = imperial\n");
  CHECK_STR(commented.s.wifiSsid, "");
  CHECK(commented.s.imperial() && commented.s.time12h() == false);
  CHECK(commented.r.changed == 1 && commented.r.problems() == 0);
  // indented comments, headings and blank lines too
  const Applied indented = apply("   # wifi_ssid = Nope\n\t; units = imperial\n  [Units]  \n   \n\t\n");
  CHECK(indented.r.problems() == 0 && indented.r.touched() == 0 && indented.r.lines == 5);
  CHECK_STR(indented.s.wifiSsid, "");
}

void testSettingsValues() {
  section("settings file: values");
  // booleans
  const char *yes[] = {"on", "ON", "Yes", "true", "1", "enabled", "y", "\"on\""};
  const char *no[] = {"off", "OFF", "No", "false", "0", "disabled", "n", "'off'"};
  for (const char *v : yes) CHECK(apply(std::string("show_week = ") + v, [] { Settings s; s.showWeek = false; return s; }()).s.showWeek);
  for (const char *v : no) CHECK(!apply(std::string("wifi = ") + v).s.wifi);

  // enums and their spellings
  CHECK(apply("units = imperial").s.imperial());
  CHECK(apply("units = Fahrenheit").s.imperial());
  CHECK(apply("units = F").s.imperial());
  CHECK(apply("temperature units = US").s.imperial());
  CHECK(!apply("units = metric", apply("units = imperial").s).s.imperial());
  CHECK(!apply("units = celsius", apply("units = imperial").s).s.imperial());
  CHECK(apply("time_format = 12h").s.time12h());
  CHECK(apply("time format = 12").s.time12h());
  CHECK(apply("hour format = 12 hour").s.time12h());
  CHECK(apply("time_format = AM/PM").s.time12h());
  CHECK(!apply("time_format = 24h", apply("time_format = 12h").s).s.time12h());
  struct { const char *text; int fmt; } dates[] = {
      {"iso", DATE_ISO}, {"YYYY-MM-DD", DATE_ISO}, {"dmy", DATE_DMY}, {"DD/MM/YYYY", DATE_DMY},
      {"mdy", DATE_MDY}, {"mm/dd/yyyy", DATE_MDY}, {"dmy-dot", DATE_DMY_DOT}, {"dd.mm.yyyy", DATE_DMY_DOT},
      {"d-mon-y", DATE_D_MON_Y}, {"D Mon YYYY", DATE_D_MON_Y}, {"mon-d-y", DATE_MON_D_Y}, {"Mon D, YYYY", DATE_MON_D_Y},
      {"\"dmy\"", DATE_DMY}};
  for (const auto &d : dates) CHECK(apply(std::string("date_format = ") + d.text).s.dateFormat == d.fmt);
  CHECK(apply("battery = none").s.battery == BATTERY_NONE && !apply("battery = none").s.hasBattery());
  CHECK(apply("battery = no").s.battery == BATTERY_NONE);
  CHECK(apply("battery = USB").s.battery == BATTERY_NONE);
  CHECK(apply("battery = auto", apply("battery = none").s).s.battery == BATTERY_AUTO);
  CHECK(apply("cpu_mhz = 240").s.cpuMhz() == 240);
  CHECK(apply("cpu_mhz = 160 MHz").s.cpuMhz() == 160);
  CHECK(apply("CPU = 80").s.cpuMhz() == 80);
  CHECK(apply("wifi_power_save = normal").s.wifiPowerSave == WIFISAVE_NORMAL);
  CHECK(apply("wifi_power_save = max", apply("wifi_power_save = normal").s).s.wifiPowerSave == WIFISAVE_MAX);

  // numbers
  CHECK_NEAR(apply("indoor_offset = -3.5").s.indoorOffsetC, -3.5, 1e-6);
  CHECK_NEAR(apply("indoor offset = -3,5 C").s.indoorOffsetC, -3.5, 1e-6);
  CHECK_NEAR(apply("indoor_offset = 0").s.indoorOffsetC, 0, 1e-6);
  CHECK_NEAR(apply("indoor_offset = \"-2\"").s.indoorOffsetC, -2, 1e-6);
  CHECK_NEAR(apply("indoor_offset = -4.0 \xC2\xB0" "C").s.indoorOffsetC, -4, 1e-6);
  CHECK_NEAR(apply("battery_cutoff_v = 3.4 V").s.batteryCutoffV, 3.4, 1e-6);
  CHECK_NEAR(apply("battery_cutoff_v = 3,45").s.batteryCutoffV, 3.45, 1e-6);
  CHECK(apply("weather_interval_min = 30 min").s.weatherIntervalMin == 30);
  CHECK(apply("weather_interval_min = 30.4").s.weatherIntervalMin == 30);
  CHECK(apply("battery_capacity_mah = 2500 mAh").s.batteryCapacityMah == 2500);
  CHECK(apply("battery_capacity_mah = 0").s.batteryCapacityMah == 0);

  // coordinates
  CHECK_NEAR(apply("latitude = -31.952").s.latitude, -31.952, 1e-9);
  CHECK_NEAR(apply("latitude = 31.952 S").s.latitude, -31.952, 1e-9);
  CHECK_NEAR(apply("latitude = -31.952 S").s.latitude, -31.952, 1e-9);
  CHECK_NEAR(apply("latitude = 31.952s").s.latitude, -31.952, 1e-9);
  CHECK_NEAR(apply("latitude = 31.952 N").s.latitude, 31.952, 1e-9);
  CHECK_NEAR(apply("latitude = 31.952\xC2\xB0 S").s.latitude, -31.952, 1e-9);
  CHECK_NEAR(apply("lat = -31,952").s.latitude, -31.952, 1e-9);
  CHECK_NEAR(apply("longitude = 115.861 E").s.longitude, 115.861, 1e-9);
  CHECK_NEAR(apply("longitude = 115.861").s.longitude, 115.861, 1e-9);
  CHECK_NEAR(apply("lon = 74.006 W").s.longitude, -74.006, 1e-9);
  CHECK_NEAR(apply("Long = 74.006 w").s.longitude, -74.006, 1e-9);
  CHECK_NEAR(apply("lng = -180").s.longitude, -180, 1e-9);
  CHECK(apply("latitude = -31.952\nlongitude = 115.861").s.hasLatLon());

  // strings
  CHECK_STR(apply("hostname = my-clock").s.hostname, "my-clock");
  CHECK_STR(apply("ntp_server = time.google.com").s.ntpServer, "time.google.com");
  CHECK_STR(apply("location = \"Paris, France\"").s.location, "Paris, France");
  CHECK_STR(apply("location = Paris, France").s.location, "Paris, France");
  CHECK_STR(apply("city = Perth").s.location, "Perth");
  CHECK_STR(apply("location_label = Home sweet home").s.locationLabel, "Home sweet home");
  CHECK_STR(apply("label = \"\"").s.locationLabel, "");
  CHECK_STR(apply("spotify_client_id = 0123456789abcdef0123456789abcdef").s.spotifyClientId,
            "0123456789abcdef0123456789abcdef");
  CHECK_STR(apply("client id = \"0123456789abcdef0123456789abcdef\"").s.spotifyClientId,
            "0123456789abcdef0123456789abcdef");
}

void testSettingsRejects() {
  section("settings file: bad values change nothing");
  struct { const char *line; } bad[] = {
      {"wifi = maybe"},          {"wifi = 2"},                 {"show_week = sometimes"},
      {"units = kelvin"},        {"units = 5"},                {"time_format = 36h"},
      {"date_format = yyyy"},    {"battery = lots"},           {"cpu_mhz = 100"},
      {"cpu_mhz = fast"},        {"wifi_power_save = loud"},   {"latitude = 95"},
      {"latitude = -91"},        {"latitude = north"},         {"latitude = 33 E"},
      {"longitude = 181"},       {"longitude = 100 N"},        {"longitude = abc"},
      {"latitude = nan"},        {"latitude = inf"},           {"latitude = 3.3.0"},
      {"indoor_offset = 20"},    {"indoor_offset = -16"},      {"indoor_offset = warm"},
      {"battery_cutoff_v = 2.5"}, {"battery_cutoff_v = 3.7"},  {"battery_cutoff_v = 33"},
      {"battery_capacity_mah = -5"}, {"battery_capacity_mah = 99999"},
      {"weather_interval_min = 1"}, {"weather_interval_min = 1000"}, {"weather_interval_min = soon"},
      {"hostname = my clock"},   {"hostname = -clock"},        {"hostname = clock-"},
      {"hostname = cl_ock"},     {"hostname = clock.local"},   {"ntp_server = http://x"},
      {"ntp_server = bad host"}, {"timezone = Perth"},         {"timezone = Mars/Olympus"},
      {"timezone = 12345"},      {"spotify_client_id = short"}, {"spotify_client_id = 0123456789abcdef0123456789abcde!"},
      {"spotify_client_id = 0123456789abcdef 0123456789abcdef"},
  };
  for (const auto &b : bad) {
    const Applied a = apply(b.line);
    if (!(a.r.bad == 1 && a.r.changed == 0)) printf("  accepted [%s]\n", b.line);
    CHECK(a.r.bad == 1 && a.r.unknown == 0 && a.r.changed == 0 && a.r.unchanged == 0);
    CHECK(a.s.userSet == 0);
    CHECK(dump(a.s) == dump(Settings()));
    CHECK(a.r.issueCount == 1 && a.r.issues[0].line == 1);
  }
  // too long
  const std::string longSsid(33, 'x');
  CHECK(apply("wifi_ssid = " + longSsid).r.bad == 1);
  CHECK(apply("wifi_ssid = " + std::string(32, 'x')).r.bad == 0);
  CHECK(apply("wifi_password = " + std::string(65, 'x')).r.bad == 1);
  CHECK(apply("wifi_password = " + std::string(64, 'x')).r.bad == 0);
  CHECK(apply("location_label = " + std::string(40, 'x')).r.bad == 1);
  CHECK(apply("wifi_ssid = tab\there").r.bad == 1);  // control character
  // a rejected value leaves what was there
  Settings prev = apply("wifi_ssid = Keep\nunits = imperial\nlatitude = -31.952").s;
  Applied rej = apply("wifi_ssid = " + longSsid + "\nunits = kelvin\nlatitude = 500", prev);
  CHECK(rej.r.bad == 3 && rej.r.changed == 0);
  CHECK(dump(rej.s) == dump(prev) && rej.s.userSet == prev.userSet);
}

void testSettingsTimezone() {
  section("settings file: time zone");
  CHECK_STR(apply("timezone = Australia/Perth").s.timezone, "Australia/Perth");
  CHECK_STR(apply("timezone = australia/perth").s.timezone, "Australia/Perth");
  CHECK_STR(apply("timezone = AUSTRALIA/PERTH").s.timezone, "Australia/Perth");
  CHECK_STR(apply("timezone = \"America/New York\"").s.timezone, "America/New_York");
  CHECK_STR(apply("tz = Europe/London").s.timezone, "Europe/London");
  CHECK_STR(apply("time zone = UTC").s.timezone, "UTC");
  CHECK_STR(apply("timezone = AWST-8").s.timezone, "AWST-8");
  CHECK_STR(apply("timezone = <+0530>-5:30").s.timezone, "<+0530>-5:30");
  CHECK_STR(apply("timezone = CET-1CEST,M3.5.0,M10.5.0/3").s.timezone, "CET-1CEST,M3.5.0,M10.5.0/3");
  CHECK(apply("timezone = Australia/Perth").r.changed == 1);
  CHECK_STR(apply("timezone = \"\"", apply("timezone = AWST-8").s).s.timezone, "");  // empty on purpose = follow location
  CHECK(settingIsUserSet(apply("timezone = \"\"").s, idx("timezone")));
  CHECK_STR(apply("timezone =", apply("timezone = AWST-8").s).s.timezone, "");  // forgotten = the default
  CHECK(!settingIsUserSet(apply("timezone =", apply("timezone = AWST-8").s).s, idx("timezone")));
  // what the firmware does with it afterwards
  const Applied perth = apply("timezone = australia/perth");
  CHECK_STR(tzPosixForIana(perth.s.timezone), "AWST-8");
  char canon[48];
  CHECK_STR(tzFindIanaIgnoreCase("america/new york", canon, sizeof canon), "EST5EDT,M3.2.0,M11.1.0");
  CHECK_STR(canon, "America/New_York");
  CHECK(tzFindIanaIgnoreCase("Nowhere/Land", canon, sizeof canon) == nullptr);
  CHECK(tzFindIanaIgnoreCase("", canon, sizeof canon) == nullptr);
  CHECK(tzFindIanaIgnoreCase(nullptr, canon, sizeof canon) == nullptr);
  CHECK(tzFindIanaIgnoreCase("Australia/Perth", nullptr, 0) != nullptr);
  CHECK(tzFindIanaIgnoreCase("Australia/Per", canon, sizeof canon) == nullptr);   // prefix is not a match
  CHECK(tzFindIanaIgnoreCase("Australia/Perth2", canon, sizeof canon) == nullptr);
}

void testSettingsReport() {
  section("settings file: report");
  const char *file =
      "# comment\n"
      "wifi = maybe\n"
      "unknown_key = 1\n"
      "wifi_ssid = Home\n"
      "this line has no equals\n"
      "latitude = 95\n"
      "units = imperial\n"
      "units = imperial\n";
  const Applied a = apply(file);
  CHECK(a.r.lines == 8);
  CHECK(a.r.changed == 2 && a.r.unchanged == 1);  // the second "units" repeats the first
  CHECK(a.r.unknown == 1 && a.r.bad == 3 && a.r.problems() == 4);
  CHECK(a.r.issueCount == 4);
  CHECK(a.r.issues[0].line == 2 && strstr(a.r.issues[0].text, "line 2") && strstr(a.r.issues[0].text, "wifi"));
  CHECK(a.r.issues[1].line == 3 && strstr(a.r.issues[1].text, "unknown") && strstr(a.r.issues[1].text, "unknown_key"));
  CHECK(a.r.issues[2].line == 5 && strstr(a.r.issues[2].text, "line 5"));
  CHECK(a.r.issues[3].line == 6 && strstr(a.r.issues[3].text, "latitude"));
  CHECK_STR(a.s.wifiSsid, "Home");
  CHECK(a.s.imperial() && !a.s.hasLatLon() && a.s.wifi);
  char sum[96];
  formatConfigSummary(a.r, sum, sizeof sum);
  CHECK_STR(sum, "2 changed, 1 unchanged, 4 problems");
  formatConfigSummary(apply("units = imperial").r, sum, sizeof sum);
  CHECK_STR(sum, "1 changed");
  formatConfigSummary(apply("wifi = maybe").r, sum, sizeof sum);
  CHECK_STR(sum, "1 problem");
  formatConfigSummary(apply("# nothing").r, sum, sizeof sum);
  CHECK_STR(sum, "nothing to apply");
  formatConfigSummary(apply("reset_all = yes\nunits = imperial").r, sum, sizeof sum);
  CHECK_STR(sum, "all forgotten, 1 changed");
  char tiny[8];
  formatConfigSummary(a.r, tiny, sizeof tiny);  // must stay inside the buffer
  CHECK(strlen(tiny) < sizeof tiny);

  // only the first few problems are kept, but all are counted
  std::string many;
  for (int i = 0; i < 20; i++) many += "wifi = maybe\n";
  const Applied m = apply(many);
  CHECK(m.r.bad == 20 && m.r.issueCount == kConfigMaxIssues && m.r.lines == 20);
  CHECK(m.r.issues[kConfigMaxIssues - 1].line == kConfigMaxIssues);

  // a line that is far too long is reported, not parsed
  const Applied l = apply("wifi_ssid = " + std::string(500, 'x') + "\nunits = imperial\n");
  CHECK(l.r.bad == 1 && l.r.changed == 1 && l.s.imperial() && l.s.wifiSsid[0] == 0);
  CHECK(strstr(l.r.issues[0].text, "too long"));
  // text that is no settings file at all
  static const char junkText[] = "\x01\x02\xFF\xFE garbage \x80\x81\n\0\0\0=\n";
  const Applied junk = apply(std::string(junkText, sizeof junkText - 1));
  CHECK(junk.r.touched() == 0 && junk.r.problems() >= 1);
  static const char nulText[] = "wifi_ssid = a\0b\n";  // a NUL byte inside a value is replaced, not a terminator
  CHECK_STR(apply(std::string(nulText, sizeof nulText - 1)).s.wifiSsid, "a?b");
}

// The banner after a start that read a settings file, and after one that could not say what it had done.
void testSdBanner() {
  section("settings file: the banner");
  char b[40];
  bool warn = false;
  auto banner = [&](const ConfigReport &r, bool saved, int carried, bool justInstalled) {
    warn = formatSdBanner(r, saved, carried, justInstalled, b, sizeof b);
    return std::string(b);
  };

  // As it went on the board: a card with four settings and a firmware file.  The start that read it changed
  // the four, saved them, installed the firmware and restarted before its banner; the next start read the
  // same file, found nothing to change, and said "nothing new" of settings the owner had just added.
  const std::string file = "wifi_mode = sync\nspotify_live = off\ncpu_idle_mhz = 20\nconsole = auto\n";
  const Applied first = apply(file);
  CHECK(first.r.changed == 4 && first.r.touched() == 4 && first.r.unchanged == 0 && first.r.problems() == 0);
  CHECK(banner(first.r, true, 0, false) == "SD settings: 4 changed" && !warn);  // (what was never shown)
  const Applied second = apply(file, first.s);
  CHECK(second.r.touched() == 0 && second.r.unchanged == 4 && second.r.problems() == 0);
  CHECK(dump(second.s) == dump(first.s));
  // with the count the first start left behind, the second says what the card did
  CHECK(banner(second.r, false, first.r.touched(), true) == "SD settings: 4 changed" && !warn);
  // without one (the firmware that ran first did not keep a count yet) it says what it knows, and no more
  CHECK(banner(second.r, false, 0, true) == "SD settings: 4 in force" && !warn);
  // an ordinary restart with the card left in
  CHECK(banner(second.r, false, 0, false) == "SD settings: 4 in force, none new" && !warn);
  // (nothing was saved at such a start because nothing changed: that is not "NOT saved")
  CHECK(banner(second.r, false, 0, false).find("NOT") == std::string::npos);

  // the new firmware changes one more (a setting the old one did not know): the counts add up
  const Applied third = apply("units = imperial\n" + file, first.s);
  CHECK(third.r.changed == 1 && third.r.unchanged == 4);
  CHECK(banner(third.r, true, 3, true) == "SD settings: 4 changed" && !warn);
  CHECK(banner(third.r, true, 0, false) == "SD settings: 1 changed" && !warn);
  // flash would not take them: a warning, and only what this start changed (what was carried had been saved)
  CHECK(banner(third.r, false, 3, false) == "1 changed, NOT saved to flash" && warn);

  // problems are a warning, whatever else there is
  const Applied bad = apply("wifi = maybe\n" + file, first.s);
  CHECK(bad.r.problems() == 1 && bad.r.touched() == 0);
  CHECK(banner(bad.r, false, 0, false) == "SD: 0 changed, 1 problem" && warn);
  CHECK(banner(bad.r, false, 4, true) == "SD: 4 changed, 1 problem" && warn);
  CHECK(banner(apply("wifi = maybe\ncolour = red\nunits = imperial\n").r, true, 0, false) == "SD: 1 changed, 2 problems" && warn);

  // a file that is all comments, as the clock writes it
  const Applied comments = apply("# units = metric\r\n\r\n# wifi = on\r\n");
  CHECK(comments.r.touched() == 0 && comments.r.unchanged == 0 && comments.r.problems() == 0);
  CHECK(banner(comments.r, false, 0, false) == "SD settings: the file sets nothing" && !warn);
  CHECK(banner(comments.r, false, 0, true) == "SD settings: the file sets nothing" && !warn);
  CHECK(banner(comments.r, false, 2, true) == "SD settings: 2 changed" && !warn);  // (the card was swapped: still said)
  // a setting that is forgotten, and reset_all, are changes
  const Applied forgot = apply("units =\n", third.s);
  CHECK(forgot.r.cleared == 1 && forgot.r.changed == 0);
  CHECK(banner(forgot.r, true, 0, false) == "SD settings: 1 changed" && !warn);
  CHECK(banner(apply("reset_all = yes\n", third.s).r, true, 0, false) == "SD settings: 1 changed" && !warn);
  // no card at this start (an empty report): what the last start changed is still said
  CHECK(banner(ConfigReport(), true, 4, false) == "SD settings: 4 changed" && !warn);
  // a count that makes no sense is no count
  CHECK(banner(second.r, false, -3, false) == "SD settings: 4 in force, none new" && !warn);
  CHECK(banner(third.r, true, -3, false) == "SD settings: 1 changed" && !warn);

  // every text fits a banner of 39 characters, with counts beyond what a file of 32 KB can reach
  int texts = 0, tooLong = 0;
  for (int changed : {0, 1, 9999})
    for (int unchanged : {0, 1, 9999})
      for (int problems : {0, 1, 9999})
        for (int carried : {0, 9999})
          for (int flags = 0; flags < 4; flags++) {
            ConfigReport r;
            r.changed = changed;
            r.unchanged = unchanged;
            r.unknown = problems;
            const std::string text = banner(r, (flags & 1) != 0, carried, (flags & 2) != 0);
            texts++;
            if (text.empty() || text.size() > 39) tooLong++;
          }
  CHECK(texts == 216 && tooLong == 0);
  ConfigReport big;
  big.changed = big.unchanged = big.unknown = 9999;
  char tiny[8];
  volatile size_t tinyCap = sizeof tiny;  // (volatile: the compiler would otherwise warn that this text is cut, which is the point)
  formatSdBanner(big, true, 9999, false, tiny, tinyCap);  // must stay inside the buffer
  CHECK(strlen(tiny) < sizeof tiny);

  // the count that is carried lives in flash next to the settings: its name is no setting's, and fits (15 characters)
  CHECK(strlen(kSdNoteNvsKey) >= 1 && strlen(kSdNoteNvsKey) <= 15);
  int clashes = 0;
  for (size_t i = 0; i < settingCount(); i++) clashes += strcmp(settingNvsKey(i), kSdNoteNvsKey) == 0;
  CHECK(clashes == 0);
}

void testSettingsForget() {
  section("settings file: forgetting and reset_all");
  Settings defaults;
  snprintf(defaults.hostname, sizeof defaults.hostname, "factory-name");
  defaults.latitude = -31.952;
  defaults.longitude = 115.861;
  // a clock with a few saved settings
  Settings s = defaults;
  const std::string saved = "hostname = mine\nunits = imperial\nlatitude = 10\nwifi_ssid = Home\n";
  ConfigReport r0 = applyConfigText(s, defaults, saved.data(), saved.size());
  CHECK(r0.changed == 4 && s.userSet != 0);
  CHECK_STR(s.hostname, "mine");

  // an empty value forgets one setting: the build's default is back
  const std::string forget = "hostname =\nlatitude = \n";
  Settings s1 = s;
  const ConfigReport r1 = applyConfigText(s1, defaults, forget.data(), forget.size());
  CHECK(r1.cleared == 2 && r1.changed == 0 && r1.problems() == 0);
  CHECK_STR(s1.hostname, "factory-name");
  CHECK_NEAR(s1.latitude, -31.952, 1e-9);
  CHECK(!settingIsUserSet(s1, idx("hostname")) && !settingIsUserSet(s1, idx("latitude")));
  CHECK(settingIsUserSet(s1, idx("units")) && settingIsUserSet(s1, idx("wifi_ssid")));  // the others stay
  CHECK(s1.imperial() && !strcmp(s1.wifiSsid, "Home"));

  // forgetting something that was never saved is not an error
  Settings s2 = defaults;
  const ConfigReport r2 = applyConfigText(s2, defaults, forget.data(), forget.size());
  CHECK(r2.cleared == 0 && r2.unchanged == 2 && r2.problems() == 0 && r2.touched() == 0);

  // reset_all forgets everything first, even when it comes last in the file
  const std::string reset = "reset_all = yes\n";
  Settings s3 = s;
  const ConfigReport r3 = applyConfigText(s3, defaults, reset.data(), reset.size());
  CHECK(r3.resetAll && r3.touched() == 1 && r3.problems() == 0);
  CHECK(s3.userSet == 0 && dump(s3) == dump(defaults));
  const std::string reset2 = "units = metric\nreset_all = yes\nspotify = off\n";
  Settings s4 = s;
  const ConfigReport r4 = applyConfigText(s4, defaults, reset2.data(), reset2.size());
  CHECK(r4.resetAll && r4.problems() == 0);
  CHECK(!s4.imperial() && !s4.spotify);                              // lines apply on top of the reset
  CHECK_STR(s4.hostname, "factory-name");                            // saved values are gone
  CHECK(settingIsUserSet(s4, idx("units")) && settingIsUserSet(s4, idx("spotify")));  // units and spotify are saved again
  CHECK(!settingIsUserSet(s4, idx("hostname")) && !settingIsUserSet(s4, idx("wifi_ssid")));
  CHECK(r4.changed == 1 && r4.unchanged == 1);  // metric is the default again after the reset; spotify=off is new
  for (const char *spelling : {"reset_all = on", "Reset All = TRUE", "reset-all: 1", "reset_all = \"yes\""}) {
    Settings t = s;
    const ConfigReport rr = applyConfigText(t, defaults, spelling, strlen(spelling));
    CHECK(rr.resetAll && t.userSet == 0);
  }
  for (const char *no : {"reset_all = no", "reset_all = off", "# reset_all = yes"}) {
    Settings t = s;
    const ConfigReport rr = applyConfigText(t, defaults, no, strlen(no));
    CHECK(!rr.resetAll && t.userSet == s.userSet && rr.problems() == 0);
  }
  Settings t5 = s;
  const std::string badReset = "reset_all = please\n";
  const ConfigReport r5 = applyConfigText(t5, defaults, badReset.data(), badReset.size());
  CHECK(!r5.resetAll && r5.bad == 1 && t5.userSet == s.userSet);

  // single settings, the API a web page or a button handler could use
  Settings t6 = defaults;
  char err[96];
  CHECK(applySetting(t6, defaults, "units", "Imperial", err, sizeof err) == ApplyResult::Changed);
  CHECK(applySetting(t6, defaults, "units", "imperial", err, sizeof err) == ApplyResult::Unchanged);
  CHECK(applySetting(t6, defaults, "units", "kelvin", err, sizeof err) == ApplyResult::BadValue && strstr(err, "units"));
  CHECK(applySetting(t6, defaults, "colour", "red", err, sizeof err) == ApplyResult::UnknownKey && strstr(err, "colour"));
  CHECK(applySetting(t6, defaults, "units", "", err, sizeof err) == ApplyResult::Cleared);
  CHECK(!t6.imperial() && t6.userSet == 0);
  CHECK(applySetting(t6, defaults, "wifi_ssid", "\"Quoted Net\"") == ApplyResult::Changed && !strcmp(t6.wifiSsid, "Quoted Net"));
  CHECK(applySetting(t6, defaults, "wifi_ssid", nullptr) == ApplyResult::Cleared);
}

// Lines of an example file that look like "# key = value" for a known key.
std::vector<std::string> splitLines(const std::string &text) {
  std::vector<std::string> lines;
  size_t pos = 0;
  while (pos < text.size()) {
    size_t e = text.find('\n', pos);
    if (e == std::string::npos) e = text.size();
    lines.push_back(text.substr(pos, e - pos));
    pos = e + 1;
  }
  return lines;
}

bool isSettingLine(const std::string &line, size_t *index) {
  if (line.compare(0, 2, "# ") != 0) return false;
  for (size_t i = 0; i < settingCount(); i++) {
    const std::string prefix = std::string("# ") + settingKey(i) + " = ";
    if (line.compare(0, prefix.size(), prefix) == 0) {
      *index = i;
      return true;
    }
  }
  return false;
}

void checkExample(const Settings &cur, const char *label) {
  printf("  (%s)\n", label);
  static char buf[16384];
  const size_t n = renderExampleConfig(cur, "Measured drift: +1.2 s/day over 3 days", buf, sizeof buf);
  CHECK(n > 2000 && n < 11000);
  CHECK(n == strlen(buf));
  {  // the clock writes it with Windows line ends into a buffer of 12 KB (kExampleCap in app_settings.cpp): it has to fit
    static char onCard[12 * 1024];
    const size_t m = renderExampleConfig(cur, "Measured drift: +1.2 s/day over 3 days", onCard, sizeof onCard, true);
    CHECK(m > n && m + 1024 < sizeof onCard);  // ... with a kilobyte to spare for the settings still to come
    CHECK(renderExampleConfig(cur, "", onCard, n, true) == 0);  // (it can fail: a buffer that is too small gets nothing)
  }
  const std::string text(buf, n);
  CHECK(text.back() == '\n');
  CHECK(text.find('\r') == std::string::npos);
  CHECK(text.find("Measured drift: +1.2 s/day over 3 days") != std::string::npos);
  CHECK(text.find("drift") != std::string::npos);
  CHECK(text.find("reset_all") != std::string::npos);
  CHECK(text.find("quotes") != std::string::npos);

  // nothing but comments and blank lines, none too wide for a small editor window
  size_t settingLines = 0;
  bool seen[64] = {};
  for (const std::string &line : splitLines(text)) {
    CHECK(line.empty() || line[0] == '#');
    CHECK(line.size() <= 96);
    size_t idx;
    if (isSettingLine(line, &idx)) {
      CHECK(idx < sizeof seen);
      if (idx >= sizeof seen) continue;
      settingLines++;
      CHECK(!seen[idx]);
      seen[idx] = true;
    }
  }
  CHECK(settingLines == settingCount());  // every setting is listed, once

  // so an unedited example changes nothing
  Settings same = cur;
  const Settings defaults;
  const ConfigReport r = applyConfigText(same, defaults, text.data(), text.size());
  CHECK(r.problems() == 0 && r.touched() == 0 && r.changed == 0 && r.cleared == 0 && r.unchanged == 0);
  CHECK(dump(same) == dump(cur));
  CHECK(same.userSet == cur.userSet);

  // and removing the # in front of each (non-secret) setting gives the same values back
  std::string uncommented;
  for (const std::string &line : splitLines(text)) {
    size_t idx;
    if (isSettingLine(line, &idx) && !settingIsSecret(idx)) {
      uncommented += line.substr(2) + "\n";
    } else {
      uncommented += line + "\n";
    }
  }
  Settings round;
  const ConfigReport r2 = applyConfigText(round, defaults, uncommented.data(), uncommented.size());
  if (r2.problems()) {
    for (int i = 0; i < r2.issueCount; i++) printf("  issue: %s\n", r2.issues[i].text);
  }
  CHECK(r2.problems() == 0);
  for (size_t i = 0; i < settingCount(); i++) {
    if (settingIsSecret(i)) continue;
    if (fmt(round, i) != fmt(cur, i)) printf("  %s: [%s] vs [%s]\n", settingKey(i), fmt(round, i).c_str(), fmt(cur, i).c_str());
    CHECK(fmt(round, i) == fmt(cur, i));
  }

  // too small a buffer is reported, never overrun
  char small[200];
  CHECK(renderExampleConfig(cur, "", small, sizeof small) == 0);
  CHECK(renderExampleConfig(cur, "", buf, 0) == 0);
}

void testSettingsExample() {
  section("example settings file");
  checkExample(Settings(), "defaults");
  const Applied all = apply(kAllSettings);
  checkExample(all.s, "everything changed");

  // the password and the Client ID never reach the card
  Settings secret = all.s;
  snprintf(secret.wifiPassword, sizeof secret.wifiPassword, "hunter2-super-secret");
  static char buf[16384];
  const size_t n = renderExampleConfig(secret, "", buf, sizeof buf);
  CHECK(n > 0);
  CHECK(strstr(buf, "hunter2-super-secret") == nullptr);
  CHECK(strstr(buf, "0123456789abcdef0123456789abcdef") == nullptr);
  CHECK(strstr(buf, "p@ss") == nullptr);
  CHECK(strstr(buf, "Cafe Net") != nullptr);  // the network name is not secret, and helps the user
  CHECK(strstr(buf, "# wifi_password = \"your password\"") != nullptr);
  CHECK(strstr(buf, "# wifi_backup_password = \"your password\"") != nullptr);
  CHECK(strstr(buf, "h0tsp0t") == nullptr);  // (the backup password set by kAllSettings)
  CHECK(strstr(buf, "# spotify_client_id = \"your client id\"") != nullptr);

  // without a measurement the generic drift note is still there
  const size_t n2 = renderExampleConfig(Settings(), nullptr, buf, sizeof buf);
  CHECK(n2 > 0 && strstr(buf, "drift") != nullptr && strstr(buf, "Measured") == nullptr);
  // a name with a quote in it survives the round trip through the example text? (it is quoted as typed)
  Settings quote;
  snprintf(quote.wifiSsid, sizeof quote.wifiSsid, "Bob's WiFi");
  checkExample(quote, "apostrophe in the SSID");

  // the card gets Windows line ends: the same text with every newline preceded by CR, which reads the same
  static char crlfBuf[20000], lfBuf[20000];
  const size_t lfLen = renderExampleConfig(all.s, "Measured: +1.0 ppm", lfBuf, sizeof lfBuf, false);
  const size_t crlfLen = renderExampleConfig(all.s, "Measured: +1.0 ppm", crlfBuf, sizeof crlfBuf, true);
  CHECK(lfLen > 0 && crlfLen > lfLen);
  std::string lfText(lfBuf, lfLen), crlfText(crlfBuf, crlfLen), stripped;
  size_t newlines = 0, loneLf = 0;
  for (size_t i = 0; i < crlfText.size(); i++) {
    if (crlfText[i] == '\n') {
      newlines++;
      if (i == 0 || crlfText[i - 1] != '\r') loneLf++;
    }
    if (crlfText[i] != '\r') stripped += crlfText[i];
  }
  CHECK(stripped == lfText);
  CHECK(loneLf == 0 && crlfLen == lfLen + newlines);
  Settings same = all.s;
  const ConfigReport rr = applyConfigText(same, Settings(), crlfText.data(), crlfText.size());
  CHECK(rr.problems() == 0 && rr.touched() == 0 && rr.changed == 0);
  // a buffer one byte short is refused, with or without CRLF
  CHECK(renderExampleConfig(all.s, "", crlfBuf, 100, true) == 0);
  static char probe[20000];
  const size_t exact = renderExampleConfig(all.s, "", probe, sizeof probe, true);
  CHECK(exact > 0);
  CHECK(renderExampleConfig(all.s, "", probe, exact, true) == 0);      // no room for the terminator
  CHECK(renderExampleConfig(all.s, "", probe, exact + 1, true) == exact);
}

void testSettingsFuzz() {
  section("settings file: garbage in, no crash");
  static char example[16384];
  const size_t exampleLen = renderExampleConfig(Settings(), "", example, sizeof example);
  uint32_t rng = 12345;
  auto next = [&rng]() {
    rng = rng * 1664525u + 1013904223u;
    return rng >> 8;
  };
  int tested = 0;
  for (int round = 0; round < 3000; round++) {
    std::string text;
    if (round % 3 == 0) {  // random bytes
      const size_t len = next() % 1500;
      for (size_t i = 0; i < len; i++) text += (char)(next() & 0xFF);
    } else if (round % 3 == 1) {  // the example file with some bytes changed
      text.assign(example, exampleLen);
      std::string uncommented;
      for (const std::string &line : splitLines(text)) {
        size_t idx;
        uncommented += (isSettingLine(line, &idx) ? line.substr(2) : line) + "\n";
      }
      text = uncommented;
      const int flips = 1 + (int)(next() % 20);
      for (int i = 0; i < flips; i++) text[next() % text.size()] = (char)(next() & 0xFF);
    } else {  // settings-like lines with odd values
      static const char *keys[] = {"wifi", "wifi_ssid", "latitude", "timezone", "units", "date_format", "reset_all",
                                   "battery_cutoff_v", "hostname", "cpu_mhz", "", "=", ":"};
      static const char *values[] = {"", "\"", "'", "\"\"\"", "1e999", "-1e999", "nan", "0x1p3", "99999999999999999999",
                                     "\xE2\x80", "\xC2", "\xE2\x80\x9C", "a b c d e f g h i j k l m n o p q r s t u v w x y z",
                                     "# #", "  ", "S", "-", "1,2,3", ",", "."};
      const int lines = 1 + (int)(next() % 12);
      for (int i = 0; i < lines; i++) {
        text += keys[next() % (sizeof keys / sizeof keys[0])];
        text += (next() & 1) ? " = " : (next() & 1) ? ":" : "";
        text += values[next() % (sizeof values / sizeof values[0])];
        text += (next() & 1) ? "\r\n" : "\n";
      }
    }
    Settings s;
    const Settings defaults;
    const ConfigReport r = applyConfigText(s, defaults, text.data(), text.size());
    tested++;
    // whatever came in, the settings are still sane
    bool sane = r.lines >= 0 && r.issueCount <= kConfigMaxIssues && s.latitude >= -90 && s.latitude <= 90 &&
                s.longitude >= -180 && s.longitude <= 180 && s.indoorOffsetC >= -15 && s.indoorOffsetC <= 15 &&
                s.batteryCutoffV >= 3.1f && s.batteryCutoffV <= 3.6f && s.weatherIntervalMin >= 5 &&
                s.weatherIntervalMin <= 240 && s.batteryCapacityMah >= 0 && s.batteryCapacityMah <= 20000 &&
                s.units <= UNITS_IMPERIAL && s.timeFormat <= TIME_12H && s.dateFormat < DATE_FORMAT_COUNT &&
                s.battery <= BATTERY_NONE && s.wifiPowerSave <= WIFISAVE_MAX && s.cpuSpeed <= CPU_240;
    sane = sane && memchr(s.wifiSsid, 0, sizeof s.wifiSsid) && memchr(s.wifiPassword, 0, sizeof s.wifiPassword) &&
           memchr(s.hostname, 0, sizeof s.hostname) && memchr(s.ntpServer, 0, sizeof s.ntpServer) &&
           memchr(s.locationLabel, 0, sizeof s.locationLabel) && memchr(s.location, 0, sizeof s.location) &&
           memchr(s.timezone, 0, sizeof s.timezone) && memchr(s.spotifyClientId, 0, sizeof s.spotifyClientId);
    if (!sane) printf("  insane result for round %d\n", round);
    CHECK(sane);
    // and the text of every setting still goes through flash unchanged
    for (size_t i = 0; i < settingCount(); i++) {
      Settings t;
      if (!applyStoredSetting(t, i, fmt(s, i).c_str())) {
        printf("  round %d: %s=[%s] does not reload\n", round, settingKey(i), fmt(s, i).c_str());
        CHECK(false);
        break;
      }
    }
  }
  CHECK(tested == 3000);
  // also: null text, zero length
  Settings s;
  const Settings defaults;
  CHECK(applyConfigText(s, defaults, nullptr, 0).lines == 0);
  CHECK(applyConfigText(s, defaults, "x", 0).lines == 0);

  // the worst case for the UTF-16 decoder: every unit grows to three bytes (the buffer is sized for it)
  std::string euros = "\xFF\xFE";
  for (int i = 0; i < 4000; i++) {
    euros += '\xAC';
    euros += '\x20';
  }
  euros += '\n';
  euros += '\0';
  euros += '\0';  // an odd trailing byte pair of zeros
  const ConfigReport er = applyConfigText(s, defaults, euros.data(), euros.size());
  CHECK(er.lines >= 1 && er.bad >= 1);  // one very long line, reported as such
  std::string be = "\xFE\xFF";           // and a lone BOM, or a BOM and half a unit
  CHECK(applyConfigText(s, defaults, be.data(), be.size()).lines == 0);
  be += 'x';
  CHECK(applyConfigText(s, defaults, be.data(), be.size()).lines == 0);
}

// ===========================================================================
// Moon
// ===========================================================================
int64_t utc(int y, int mo, int d, int h, int mi) { return calc::epochFromUtc(y, mo, d, h, mi, 0); }

void testMoonPhase() {
  section("moon phase");
  struct Event {
    int y, mo, d, h, mi;
  };
  // new moons and full moons that came with eclipses, times from the published eclipse circumstances (UTC)
  const Event newMoons[] = {{2000, 1, 6, 18, 14}, {2017, 8, 21, 18, 30}, {2019, 7, 2, 19, 16}, {2021, 12, 4, 7, 43},
                            {2024, 4, 8, 18, 21}, {2024, 10, 2, 18, 49}, {2025, 3, 29, 10, 58}};
  const Event fullMoons[] = {{2000, 1, 21, 4, 40}, {2018, 7, 27, 20, 21}, {2019, 1, 21, 5, 16}, {2021, 5, 26, 11, 14},
                             {2022, 11, 8, 11, 2}, {2024, 9, 18, 2, 34},  {2025, 3, 14, 6, 55}};
  for (const Event &e : newMoons) {
    const moon::Phase p = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi));
    if (!(p.lit < 0.003f)) printf("  new moon %d-%02d-%02d: lit %.5f\n", e.y, e.mo, e.d, p.lit);
    CHECK(p.lit < 0.003f);
    CHECK(p.age < 0.01f || p.age > 0.99f);
    // a day before: a thin waning crescent; a day after: a thin waxing one
    const moon::Phase before = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi) - 86400);
    const moon::Phase after = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi) + 86400);
    CHECK(!before.waxing && before.lit > 0.005f && before.lit < 0.05f);
    CHECK(after.waxing && after.lit > 0.005f && after.lit < 0.05f);
  }
  for (const Event &e : fullMoons) {
    const moon::Phase p = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi));
    if (!(p.lit > 0.997f)) printf("  full moon %d-%02d-%02d: lit %.5f\n", e.y, e.mo, e.d, p.lit);
    CHECK(p.lit > 0.997f);
    CHECK(fabsf(p.age - 0.5f) < 0.01f);
    const moon::Phase before = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi) - 86400);
    const moon::Phase after = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi) + 86400);
    CHECK(before.waxing && before.lit < 0.995f && before.lit > 0.95f);
    CHECK(!after.waxing && after.lit < 0.995f && after.lit > 0.95f);
  }
  // quarters (UTC): half lit, growing at the first, shrinking at the last
  const Event first[] = {{2024, 4, 15, 19, 13}, {2024, 10, 10, 18, 55}};
  const Event last[] = {{2024, 5, 1, 11, 27}, {2024, 10, 24, 8, 3}};
  for (const Event &e : first) {
    const moon::Phase p = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi));
    CHECK_NEAR(p.lit, 0.5, 0.02);
    CHECK_NEAR(p.age, 0.25, 0.01);
    CHECK(moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi) + 3 * 3600).waxing);
  }
  for (const Event &e : last) {
    const moon::Phase p = moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi));
    CHECK_NEAR(p.lit, 0.5, 0.02);
    CHECK_NEAR(p.age, 0.75, 0.01);
    CHECK(!moon::phaseAt(utc(e.y, e.mo, e.d, e.h, e.mi) + 3 * 3600).waxing);
  }

  // every new moon of 2024, found by scanning for the darkest minute near the published time
  const Event year2024[] = {{2024, 1, 11, 11, 57}, {2024, 2, 9, 22, 59}, {2024, 3, 10, 9, 0},   {2024, 4, 8, 18, 21},
                            {2024, 5, 8, 3, 22},   {2024, 6, 6, 12, 38}, {2024, 7, 5, 22, 57},  {2024, 8, 4, 11, 13},
                            {2024, 9, 3, 1, 55},   {2024, 10, 2, 18, 49}, {2024, 11, 1, 12, 47}, {2024, 12, 1, 6, 21}};
  for (const Event &e : year2024) {
    const int64_t expected = utc(e.y, e.mo, e.d, e.h, e.mi);
    int64_t best = expected;
    float bestLit = 2;
    for (int64_t t = expected - 12 * 3600; t <= expected + 12 * 3600; t += 60) {
      const float lit = moon::phaseAt(t).lit;
      if (lit < bestLit) {
        bestLit = lit;
        best = t;
      }
    }
    const double offMin = (double)(best - expected) / 60.0;
    if (fabs(offMin) > 45) printf("  new moon %d-%02d-%02d: found %+.0f min from the published time\n", e.y, e.mo, e.d, offMin);
    CHECK(fabs(offMin) <= 45);  // measured worst case: 29 minutes (the darkest moment is not exactly the conjunction)
  }

  // away from an eclipse the Moon passes up to five degrees from the Sun, so it is never quite dark
  for (const Event &e : {Event{2024, 1, 11, 11, 57}, Event{2024, 2, 9, 22, 59}, Event{2024, 6, 6, 12, 38}}) {
    float darkest = 2;
    for (int64_t t = utc(e.y, e.mo, e.d, e.h, e.mi) - 6 * 3600; t <= utc(e.y, e.mo, e.d, e.h, e.mi) + 6 * 3600; t += 60)
      darkest = fminf(darkest, moon::phaseAt(t).lit);
    if (!(darkest > 0.0008f && darkest < 0.0025f)) printf("  darkest at %d-%02d-%02d: %.5f\n", e.y, e.mo, e.d, darkest);
    CHECK(darkest > 0.0008f && darkest < 0.0025f);  // (1 - cos 5 degrees) / 2 is 0.0019
  }

  // over one lunation the lit fraction rises and falls once, and the age advances steadily
  const int64_t nm = utc(2024, 4, 8, 18, 21);
  float prev = -1;
  int ups = 0, downs = 0;
  for (int h = 0; h <= 29 * 24 + 12; h++) {
    const moon::Phase p = moon::phaseAt(nm + (int64_t)h * 3600);
    CHECK(p.lit >= 0 && p.lit <= 1);
    CHECK(p.age >= 0 && p.age < 1);
    if (prev >= 0) {
      if (p.lit > prev + 1e-6f) ups++;
      else if (p.lit < prev - 1e-6f) downs++;
    }
    prev = p.lit;
  }
  CHECK(ups > 330 && ups < 380);      // about 14.8 days of waxing
  CHECK(downs > 330 && downs < 380);  // about 14.8 days of waning
  // the same instant a synodic month later is nearly the same phase
  CHECK_NEAR(moon::phaseAt(nm + (int64_t)(29.530589 * 86400)).lit, moon::phaseAt(nm).lit, 0.02);
  // dates a long way from today still work
  CHECK(moon::phaseAt(0).lit >= 0 && moon::phaseAt(0).lit <= 1);                 // 1970
  CHECK(moon::phaseAt(utc(2099, 12, 31, 12, 0)).lit <= 1);                       // far future
  CHECK(moon::phaseAt(-86400LL * 365 * 30).lit >= 0);                            // before 1970
}

// How many pixels of a disc are inside / lit.
void countDisc(int d, float lit, bool onRight, int *inside, int *litPx) {
  *inside = *litPx = 0;
  for (int r = 0; r < d; r++)
    for (int c = 0; c < d; c++) {
      const moon::DiscPixel p = moon::discPixel(c, r, d, lit, onRight);
      *inside += p.inside;
      *litPx += p.lit;
    }
}

void testMoonGlyph() {
  section("moon glyph");
  for (int d = 8; d <= 14; d++) {
    int inside, litPx;
    countDisc(d, 0.0f, true, &inside, &litPx);
    CHECK(inside >= d * d / 2 && inside <= d * d);
    CHECK(litPx == 0);  // new moon: nothing lit
    countDisc(d, 1.0f, true, &inside, &litPx);
    CHECK(litPx == inside);  // full moon: all of it
    countDisc(d, 0.003f, true, &inside, &litPx);
    CHECK(litPx == 0);
    countDisc(d, 0.997f, false, &inside, &litPx);
    CHECK(litPx == inside);

    // quarter: one half (even sizes are exactly half; odd sizes keep the middle column dark)
    int right, left;
    countDisc(d, 0.5f, true, &inside, &right);
    countDisc(d, 0.5f, false, &inside, &left);
    CHECK(right == left);
    if (d % 2 == 0) CHECK(right * 2 == inside);
    CHECK(abs(right * 2 - inside) <= d);

    // the pixels lit grow with the lit fraction, and track it
    int last = -1;
    for (int k = 0; k <= 20; k++) {
      const float lit = k / 20.0f;
      int n;
      countDisc(d, lit, true, &inside, &n);
      CHECK(n >= last);
      last = n;
      if (fabs((double)n / inside - lit) > 0.12) printf("  d=%d lit=%.2f: %d of %d pixels\n", d, lit, n, inside);
      CHECK(fabs((double)n / inside - lit) <= 0.12);
    }

    // the terminator is an arc, not a straight line: on a crescent it starts nearer the middle in
    // the rows towards the top and bottom than across the middle of the disc
    if (d >= 11) {
      auto firstLit = [&](int row) {
        for (int c = 0; c < d; c++)
          if (moon::discPixel(c, row, d, 0.25f, true).lit) return c;
        return -1;
      };
      const int mid = firstLit(d / 2), nearTop = firstLit(1), nearBottom = firstLit(d - 2);
      CHECK(mid >= 0 && nearTop >= 0 && nearBottom >= 0);
      CHECK(nearTop < mid && nearBottom < mid);
      CHECK(nearTop == nearBottom);
      // and a gibbous moon bulges the other way: the dark part is a crescent on the unlit side
      auto firstDark = [&](int row) {
        for (int c = d - 1; c >= 0; c--) {
          const moon::DiscPixel p = moon::discPixel(c, row, d, 0.75f, true);
          if (p.inside && !p.lit) return c;
        }
        return -1;
      };
      CHECK(firstDark(1) > firstDark(d / 2));  // the dark crescent's inner edge sits nearer the middle at the top
    }

    // mirrored when seen from the other hemisphere, and the lit part grows from the limb
    for (int k = 1; k < 20; k++) {
      const float lit = k / 20.0f;
      for (int r = 0; r < d; r++) {
        int firstIn = -1, lastIn = -1, litCount = 0, litFirst = -1, litLast = -1;
        for (int c = 0; c < d; c++) {
          const moon::DiscPixel a = moon::discPixel(c, r, d, lit, true);
          const moon::DiscPixel b = moon::discPixel(d - 1 - c, r, d, lit, false);
          CHECK(a.inside == b.inside && a.lit == b.lit);
          if (a.inside) {
            if (firstIn < 0) firstIn = c;
            lastIn = c;
          }
          if (a.lit) {
            if (litFirst < 0) litFirst = c;
            litLast = c;
            litCount++;
          }
        }
        if (litCount) {
          CHECK(litLast == lastIn);                        // touches the lit limb
          CHECK(litLast - litFirst + 1 == litCount);       // one run, no holes
        }
        CHECK(firstIn >= 0);  // every row of the box crosses the disc
      }
    }
  }
}

// ===========================================================================
// Date and time text
// ===========================================================================
void testDateFormats() {
  section("date formats");
  struct Case {
    uint8_t format;
    const char *oct4;   // 2026-10-04
    const char *jan9;   // 2026-01-09
    const char *dec31;  // 1999-12-31
  } cases[] = {
      {DATE_ISO, "2026-10-04", "2026-01-09", "1999-12-31"},
      {DATE_DMY, "04/10/2026", "09/01/2026", "31/12/1999"},
      {DATE_MDY, "10/04/2026", "01/09/2026", "12/31/1999"},
      {DATE_DMY_DOT, "04.10.2026", "09.01.2026", "31.12.1999"},
      {DATE_D_MON_Y, "4 Oct 2026", "9 Jan 2026", "31 Dec 1999"},
      {DATE_MON_D_Y, "Oct 4, 2026", "Jan 9, 2026", "Dec 31, 1999"},
  };
  char buf[32];
  for (const Case &c : cases) {
    datefmt::formatDate(2026, 10, 4, c.format, buf, sizeof buf);
    CHECK_STR(buf, c.oct4);
    datefmt::formatDate(2026, 1, 9, c.format, buf, sizeof buf);
    CHECK_STR(buf, c.jan9);
    datefmt::formatDate(1999, 12, 31, c.format, buf, sizeof buf);
    CHECK_STR(buf, c.dec31);
  }
  CHECK(DATE_FORMAT_COUNT == 6);
  // unknown formats fall back to ISO, a short buffer is cut and terminated
  datefmt::formatDate(2026, 10, 4, 200, buf, sizeof buf);
  CHECK_STR(buf, "2026-10-04");
  char tiny[6];
  datefmt::formatDate(2026, 10, 4, DATE_ISO, tiny, sizeof tiny);
  CHECK_STR(tiny, "2026-");
  // every date of 2026 in every format is short enough for the date line, and fixed width for the numeric ones
  for (int f = 0; f < DATE_FORMAT_COUNT; f++) {
    size_t shortest = 99, longest = 0;
    for (int mo = 1; mo <= 12; mo++)
      for (int d = 1; d <= 28; d++) {
        datefmt::formatDate(2026, mo, d, (uint8_t)f, buf, sizeof buf);
        const size_t n = strlen(buf);
        shortest = n < shortest ? n : shortest;
        longest = n > longest ? n : longest;
      }
    CHECK(longest <= 12);
    if (f <= DATE_DMY_DOT) CHECK(shortest == longest && longest == 10);  // numeric formats never change width
  }

  // 12 hour clock face
  bool pm = false;
  CHECK(datefmt::hour12(0, &pm) == 12 && !pm);
  CHECK(datefmt::hour12(1, &pm) == 1 && !pm);
  CHECK(datefmt::hour12(11, &pm) == 11 && !pm);
  CHECK(datefmt::hour12(12, &pm) == 12 && pm);
  CHECK(datefmt::hour12(13, &pm) == 1 && pm);
  CHECK(datefmt::hour12(23, &pm) == 11 && pm);
  CHECK(datefmt::hour12(24, &pm) == 12 && !pm);   // wraps like 0
  CHECK(datefmt::hour12(-1, &pm) == 11 && pm);    // never crashes on odd input
  CHECK(datefmt::hour12(5, nullptr) == 5);
  CHECK_STR(datefmt::ampm(false), "AM");
  CHECK_STR(datefmt::ampm(true), "PM");
  for (int h = 0; h < 24; h++) {
    const int h12 = datefmt::hour12(h, &pm);
    CHECK(h12 >= 1 && h12 <= 12 && pm == (h >= 12));
  }
}

// ===========================================================================
// Battery curve, estimate, shutdown guard
// ===========================================================================
void testBatteryCurve() {
  section("battery curve (float and inverse)");
  // the integer gauge is exactly the rounded float one
  for (int mv = 3000; mv <= 4400; mv++) {
    const float v = mv / 1000.0f;
    CHECK(calc::batteryPercent(v) == (int)lroundf(calc::batteryPercentF(v)));
  }
  CHECK_NEAR(calc::batteryPercentF(4.30f), 100, 1e-6);
  CHECK_NEAR(calc::batteryPercentF(4.20f), 100, 1e-6);
  CHECK_NEAR(calc::batteryPercentF(3.784f), 50, 1e-4);
  CHECK_NEAR(calc::batteryPercentF(3.27f), 0, 1e-4);
  CHECK_NEAR(calc::batteryPercentF(3.0f), 0, 1e-6);
  CHECK_NEAR(calc::batteryPercentF(3.93f), 65 + 5 * (3.93 - 3.907) / (3.945 - 3.907), 0.01);  // 68.0, between 3.907 (65) and 3.945 (70)
  // monotonic, and the inverse really is one
  float last = -1;
  for (int mv = 3200; mv <= 4250; mv++) {
    const float p = calc::batteryPercentF(mv / 1000.0f);
    CHECK(p >= last);
    last = p;
  }
  for (int i = 0; i <= 200; i++) {
    const float pct = i * 0.5f;
    const float v = calc::batteryVoltsForPercent(pct);
    CHECK_NEAR(calc::batteryPercentF(v), pct, 0.01);
  }
  CHECK_NEAR(calc::batteryVoltsForPercent(150), 4.20, 1e-6);
  CHECK_NEAR(calc::batteryVoltsForPercent(-5), 3.27, 1e-6);
  // volts fall as percent falls
  for (int i = 1; i <= 100; i++) CHECK(calc::batteryVoltsForPercent((float)i) > calc::batteryVoltsForPercent((float)i - 1));

  // The curve carried on above its top, for the runtime estimate: the same up to 4.20 V, then 5.2 mV to the percent
  // (seen on the board: 4.223 V on the charger, 4.210 V a minute after it), and it never stands still.
  for (int mv = 3000; mv <= 4200; mv++) CHECK(calc::batteryPercentOpen(mv / 1000.0f) == calc::batteryPercentF(mv / 1000.0f));
  CHECK_NEAR(calc::batteryPercentOpen(4.21f), 101.92, 0.01);
  CHECK_NEAR(calc::batteryPercentOpen(4.223f), 104.42, 0.01);
  CHECK_NEAR(calc::batteryPercentOpen(4.30f), 119.23, 0.01);
  float lastOpen = calc::batteryPercentOpen(4.150f);
  for (int mv = 4151; mv <= 4300; mv++) {
    const float p = calc::batteryPercentOpen(mv / 1000.0f);
    CHECK(p > lastOpen && p - lastOpen < 0.2f);  // rising all the way, with no step where the two halves meet
    lastOpen = p;
  }
  {  // a cell read as 4.215 V that falls 3 mV an hour: the rate is seen, and the time left is that of a full cell, no more
    battest::Estimator est;
    for (uint32_t t = 0; t <= 3 * 3600; t += 5) est.add(t, 4.215f - 0.003f * (float)t / 3600.0f);
    const battest::Estimate e = est.estimate(3.30f, 0);
    CHECK(e.state == battest::Estimate::READY && !e.unbounded);
    CHECK_NEAR(e.pctPerHour, 0.577, 0.03);  // (3 mV an hour at 5.2 mV to the percent)
    CHECK_NEAR(e.levelPct, 101.15, 0.1);  // 4.206 V after three hours: over the top of the curve still
    CHECK_NEAR(e.hoursLeft, (100.0 - calc::batteryPercentF(3.30f)) / e.pctPerHour, 0.5);  // (0.6 % more would be two hours more)
  }
}

// The run the battery curve was fitted to, replayed through the estimator: a 3400 mAh cell from a full charge until
// the clock shut itself down at 3.30 V, a reading every ten minutes (fixtures/rundown-2026-10.csv, from the clock's
// power log of 17 to 20 October 2026).  Between two lines the voltage is taken as a straight line, read every 5 s
// as the clock does.  Each estimate is held against the time the cell really had left.
void testBatteryEstimateReplay() {
  section("battery estimate: the logged run down to the shutdown");
  std::vector<std::pair<int, float>> pts;
  {
    const std::string csv = readFile("fixtures/rundown-2026-10.csv");
    size_t at = csv.find('\n');  // (past the heading)
    while (at != std::string::npos && at + 1 < csv.size()) {
      const size_t next = csv.find('\n', at + 1);
      int minute = 0;
      float volts = 0;
      if (sscanf(csv.c_str() + at + 1, "%d,%f", &minute, &volts) == 2) pts.push_back({minute, volts});
      at = next;
    }
  }
  CHECK(pts.size() == 490 && pts.front().first == 0 && pts.back().first == 4890);
  if (pts.size() < 2) return;
  const double endMin = pts.back().first + 5;  // (the last line, 3.307 V, then under 3.30 V for a minute: shut down)
  battest::Estimator est;
  // The first hours off the charger the cell sheds what is left of the charge faster than it will drain later, so
  // the first figures are short: they may be, as long as they never promise more than the cell has.  From the
  // third hour on they are held to the truth.
  const double settleH = 3;
  int checked = 0, within15 = 0, within25 = 0, firstReadyMin = -1;
  double worst = 0, worstAtH = 0, sumAbs = 0, settlingWorstLong = -1, settlingWorstShort = 0;
  size_t seg = 0;
  for (uint32_t t = 0; t <= (uint32_t)pts.back().first * 60; t += 5) {
    while (seg + 2 < pts.size() && (double)t >= pts[seg + 1].first * 60.0) seg++;
    const double m = t / 60.0, m0 = pts[seg].first, m1 = pts[seg + 1].first;
    const double f = m1 > m0 ? (m - m0) / (m1 - m0) : 0;
    est.add(t, (float)(pts[seg].second + f * (pts[seg + 1].second - pts[seg].second)));
    if (t % 3600 != 0) continue;  // judged once an hour
    const battest::Estimate e = est.estimate(3.30f, 0);
    if (e.state != battest::Estimate::READY) continue;
    if (firstReadyMin < 0) firstReadyMin = (int)m;
    const double trueH = (endMin - m) / 60.0;
    if (trueH < 6) continue;  // (the last hours are judged below, in hours rather than shares)
    const double err = (e.unbounded ? 1e3 : e.hoursLeft) / trueH - 1.0;
    if (m / 60.0 < settleH) {
      settlingWorstLong = fmax(settlingWorstLong, err);
      settlingWorstShort = fmin(settlingWorstShort, err);
      continue;
    }
    checked++;
    within15 += fabs(err) <= 0.15;
    within25 += fabs(err) <= 0.25;
    sumAbs += fabs(err);
    if (fabs(err) > fabs(worst)) worst = err, worstAtH = m / 60.0;
  }
  printf("  (first figure after %d min; in the first %.0f hours %+.0f to %+.0f %%; then %d hourly figures up to six hours before the end: "
         "%d within 15 %%, %d within 25 %%, mean error %.0f %%, worst %+.0f %% at %.0f h)\n",
         firstReadyMin, settleH, 100 * settlingWorstShort, 100 * settlingWorstLong, checked, within15, within25,
         100 * sumAbs / (checked ? checked : 1), 100 * worst, worstAtH);
  CHECK(firstReadyMin > 0 && firstReadyMin <= 60);
  CHECK(settlingWorstLong <= 0.10);  // settling: short is allowed, long is not
  CHECK(checked >= 70);
  CHECK(within15 >= checked * 9 / 10);  // nine in ten within 15 % of the truth
  CHECK(fabs(worst) <= 0.30);           // none out by more than 30 % (the old curve said 6 days to a cell with 2.6 left)
  CHECK(sumAbs / checked <= 0.10);
  // near the end, in hours: three hours before the shutdown it must not promise more than five
  {
    battest::Estimator late;
    size_t s = 0;
    for (uint32_t t = 0; t <= (uint32_t)((endMin - 180) * 60); t += 5) {
      while (s + 2 < pts.size() && (double)t >= pts[s + 1].first * 60.0) s++;
      const double m = t / 60.0, m0 = pts[s].first, m1 = pts[s + 1].first;
      late.add(t, (float)(pts[s].second + (m - m0) / (m1 - m0) * (pts[s + 1].second - pts[s].second)));
    }
    const battest::Estimate e = late.estimate(3.30f, 0);
    CHECK(e.state == battest::Estimate::READY && !e.unbounded && e.hoursLeft <= 5.0f);
  }
}

// The sketch itself (the .ino, which no test compiles): every way into the deep sleep first has the panel refresh as
// seldom as it can (displayRest(), 0.25 Hz), because the battery-empty screen stays up for as long as the clock sleeps.
void testShutdownRestsDisplay() {
  section("the deep-sleep shutdown rests the display first");
  const std::string ino = readFile("../../11_Clock_Dashboard.ino");
  int calls = 0, rested = 0;
  for (size_t at = ino.find("powerDeepSleep("); at != std::string::npos; at = ino.find("powerDeepSleep(", at + 1)) {
    const size_t lineStart = ino.rfind('\n', at) + 1;
    const size_t prevStart = lineStart >= 2 ? ino.rfind('\n', lineStart - 2) + 1 : 0;
    const std::string prev = ino.substr(prevStart, lineStart - prevStart);
    calls++;
    rested += prev.find("displayRest();") != std::string::npos;
  }
  printf("  (%d calls of powerDeepSleep, %d right after displayRest)\n", calls, rested);
  CHECK(calls == 3 && rested == calls);
  CHECK(ino.find("static void displayRest() {\n  if (s_lcd) s_lcd->setRestMode();\n}") != std::string::npos);
  // (the negative control: the same scan on the sketch as it was in 1.11, without displayRest, finds 3 and 0)
  const std::string old = [&] {
    std::string t = ino;
    for (size_t p; (p = t.find("displayRest();")) != std::string::npos;) t.erase(p, 14);
    return t;
  }();
  int oldRested = 0;
  for (size_t at = old.find("powerDeepSleep("); at != std::string::npos; at = old.find("powerDeepSleep(", at + 1)) {
    const size_t lineStart = old.rfind('\n', at) + 1;
    const size_t prevStart = old.rfind('\n', lineStart - 2) + 1;
    oldRested += old.substr(prevStart, lineStart - prevStart).find("displayRest();") != std::string::npos;
  }
  CHECK(oldRested == 0);
}

// A deterministic reading source for the estimator tests.
struct Cell {
  uint32_t rng = 12345;
  double noiseV = 0;     // standard deviation of the reading, volts
  double dipRate = 0;    // chance of a transmit dip per reading
  double gaussian() {
    auto u = [this]() {
      rng = rng * 1664525u + 1013904223u;
      return ((rng >> 8) & 0xFFFFFF) / 16777216.0 + 1e-9;
    };
    return sqrt(-2.0 * log(u())) * cos(6.283185307179586 * u());
  }
  double uniform() {
    rng = rng * 1664525u + 1013904223u;
    return ((rng >> 8) & 0xFFFFFF) / 16777216.0;
  }
  double read(double pct) {
    double v = calc::batteryVoltsForPercent((float)pct);
    if (noiseV > 0) v += gaussian() * noiseV;
    if (dipRate > 0 && uniform() < dipRate) v -= 0.05 + 0.07 * uniform();
    return v;
  }
};

// Feeds `hours` of discharge starting at second `t0`, one reading every 5 s; returns the end time.
uint32_t discharge(battest::Estimator &est, Cell &cell, double startPct, double pctPerHour, double hours, uint32_t t0) {
  uint32_t t = t0;
  const uint32_t end = t0 + (uint32_t)(hours * 3600);
  for (; t < end; t += 5) {
    const double pct = startPct - pctPerHour * (double)(t - t0) / 3600.0;
    est.add(t, (float)cell.read(pct));
  }
  return t;
}

void testBatteryEstimate() {
  section("battery estimate");
  char buf[32];
  battest::formatRemaining(0.0f, buf, sizeof buf);       CHECK_STR(buf, "0 min");
  battest::formatRemaining(0.5f, buf, sizeof buf);       CHECK_STR(buf, "30 min");
  battest::formatRemaining(0.999f, buf, sizeof buf);     CHECK_STR(buf, "1 h 00 min");  // rounds up to the hour, not "60 min"
  battest::formatRemaining(1.0f, buf, sizeof buf);       CHECK_STR(buf, "1 h 00 min");
  battest::formatRemaining(5.0f + 40.0f / 60, buf, sizeof buf); CHECK_STR(buf, "5 h 40 min");
  battest::formatRemaining(47.9f, buf, sizeof buf);      CHECK_STR(buf, "47 h 54 min");
  battest::formatRemaining(48.0f, buf, sizeof buf);      CHECK_STR(buf, "2 d 0 h");
  battest::formatRemaining(100.0f, buf, sizeof buf);     CHECK_STR(buf, "4 d 4 h");
  battest::formatRemaining(24.0f * 31, buf, sizeof buf); CHECK_STR(buf, ">30 d");
  battest::formatRemaining(-1.0f, buf, sizeof buf);      CHECK_STR(buf, "--");
  battest::formatRemaining(NAN, buf, sizeof buf);        CHECK_STR(buf, "--");

  const float cutoff = 3.30f;
  const double cutPct = calc::batteryPercentF(cutoff);

  // nothing yet
  {
    battest::Estimator est;
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::OFF);
    est.add(100, NAN);
    est.add(100, 0.0f);
    est.add(100, 9.0f);
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::OFF);  // junk readings are ignored
    CHECK(est.binCount() == 0);
  }

  // a steady, noiseless discharge at 3 % an hour
  {
    battest::Estimator est;
    Cell cell;
    // minute by minute: the state goes OFF -> LEARNING (counting down) -> READY
    int lastLearn = 1000;
    bool seenLearning = false, seenReady = false;
    for (uint32_t sec = 0; sec <= 3 * 3600; sec += 5) {
      const double pct = 90 - 3.0 * sec / 3600.0;
      est.add(2000 + sec, (float)cell.read(pct));
      if (sec % 60 != 0) continue;
      const battest::Estimate e = est.estimate(cutoff, 0);
      if (e.state == battest::Estimate::LEARNING) {
        seenLearning = true;
        CHECK(!seenReady);
        CHECK(e.learnMin <= lastLearn);  // the countdown only goes down
        CHECK(e.learnMin >= 1 && e.learnMin <= 51);
        lastLearn = e.learnMin;
      } else if (e.state == battest::Estimate::READY) {
        if (!seenReady) CHECK(sec >= 45 * 60 && sec <= 56 * 60);  // 15 min settling + 30 min of history (+ the bin that is still open)
        seenReady = true;
      } else {
        CHECK(sec == 0);  // OFF only before the first reading closes
      }
    }
    CHECK(seenLearning && seenReady);
    const battest::Estimate e = est.estimate(cutoff, 1000);
    CHECK(e.state == battest::Estimate::READY);
    CHECK_NEAR(e.pctPerHour, 3.0, 0.03);
    CHECK_NEAR(e.avgMa, 30.0, 0.5);  // 3 % of 1000 mAh an hour
    CHECK(e.windowMin >= 100 && e.windowMin <= 180);  // all of the history there is: just under 3 h
    // the fitted level is that of the newest bin, at most one bin old
    CHECK_NEAR(e.levelPct, 90 - 3.0 * 3.0 + 0.25, 0.4);
    CHECK_NEAR(e.hoursLeft, (e.levelPct - cutPct) / 3.0, 0.05);
    CHECK(!e.unbounded);
    // without a capacity there is no current
    CHECK(est.estimate(cutoff, 0).avgMa == 0);
  }

  // the window follows the drain: a fast one is judged over about two hours, a slow one over many
  {
    battest::Estimator fast, slow;
    Cell c1, c2;
    discharge(fast, c1, 95, 12.0, 7.0, 0);
    discharge(slow, c2, 95, 1.5, 12.0, 0);
    const battest::Estimate f = fast.estimate(cutoff, 0), s = slow.estimate(cutoff, 0);
    CHECK(f.state == battest::Estimate::READY && s.state == battest::Estimate::READY);
    CHECK(f.windowMin >= 110 && f.windowMin <= 130);
    CHECK(s.windowMin >= 440 && s.windowMin <= 480);
    CHECK_NEAR(f.pctPerHour, 12.0, 0.15);
    CHECK_NEAR(s.pctPerHour, 1.5, 0.03);
    CHECK(slow.binCount() == battest::kMaxBins);  // 12 h of 5 minute bins: the oldest were dropped
  }

  // noise, WiFi dips: still close
  {
    battest::Estimator est;
    Cell cell;
    cell.noiseV = 0.003;
    cell.dipRate = 0.01;
    discharge(est, cell, 85, 3.0, 8.0, 500);
    const battest::Estimate e = est.estimate(cutoff, 0);
    CHECK(e.state == battest::Estimate::READY);
    const double truth = (85 - 3.0 * 8 - cutPct) / 3.0;
    if (fabs(e.pctPerHour - 3.0) > 0.45) printf("  noisy drain: %.2f %%/h, %.1f h left (truth %.1f h)\n", e.pctPerHour, e.hoursLeft, truth);
    CHECK_NEAR(e.pctPerHour, 3.0, 0.45);
    CHECK(fabs(e.hoursLeft - truth) / truth < 0.2);
  }

  // no drain at all: "unbounded", not a division by zero
  {
    battest::Estimator est;
    Cell cell;
    for (uint32_t sec = 0; sec < 5 * 3600; sec += 5) est.add(sec, 3.90f);
    const battest::Estimate e = est.estimate(cutoff, 1000);
    CHECK(e.state == battest::Estimate::READY && e.unbounded && e.pctPerHour < 0.02f && e.avgMa < 0.2f);
  }

  // a lot of transmit dips: each bin's median ignores them (a mean would sag by several percent)
  {
    battest::Estimator est;
    Cell cell;
    uint32_t rng = 777;
    for (uint32_t sec = 0; sec < 4 * 3600; sec += 5) {
      rng = rng * 1664525u + 1013904223u;
      const bool dip = ((rng >> 8) & 0xFF) < 40;  // about 16 % of the readings
      est.add(sec, dip ? 3.78f : 3.90f);
    }
    const battest::Estimate e = est.estimate(cutoff, 0);
    CHECK(e.state == battest::Estimate::READY);
    CHECK_NEAR(e.levelPct, calc::batteryPercentF(3.90f), 0.3);
  }

  // below the cut-off already: no time left
  {
    battest::Estimator est;
    Cell cell;
    discharge(est, cell, 6.0, 3.0, 2.0, 0);
    const battest::Estimate e = est.estimate(3.50f, 0);  // a cut-off above where the cell is
    CHECK(e.state == battest::Estimate::READY && e.hoursLeft == 0 && !e.unbounded);
  }

  // reset forgets everything
  {
    battest::Estimator est;
    Cell cell;
    discharge(est, cell, 90, 3.0, 2.0, 0);
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::READY);
    est.reset();
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::OFF && est.binCount() == 0);
    est.add(99999, 3.9f);
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::LEARNING);
  }

  // a reset in the middle of a busy bin must not leave its time offsets behind: the first bin after it
  // holds one reading and would be stamped hours into the future.  The line fit throws that point out, so
  // the slope and the level still come out right; what it spoils is the span and the window, and with them
  // when the first figure appears.  A fresh estimator is the reference, compared on everything the estimate
  // reports, every minute of the run.
  {
    battest::Estimator used, fresh;
    for (uint32_t t = 560; t < 600; t++) used.add(t, 3.95f);  // 40 readings late in a bin
    used.reset();
    Cell c1, c2;
    const uint32_t t0 = 599;
    int compared = 0, differing = 0, ready = 0;
    for (uint32_t t = t0; t < t0 + 3 * 3600; t += 5) {
      const double pct = 90.0 - 3.0 * (double)(t - t0) / 3600.0;
      used.add(t, (float)c1.read(pct));
      fresh.add(t, (float)c2.read(pct));
      if ((t - t0) % 60 != 0) continue;
      const battest::Estimate a = used.estimate(cutoff, 2000), b = fresh.estimate(cutoff, 2000);
      compared++;
      if (b.state == battest::Estimate::READY) ready++;
      if (a.state != b.state || a.learnMin != b.learnMin || a.windowMin != b.windowMin || a.unbounded != b.unbounded ||
          fabs(a.pctPerHour - b.pctPerHour) > 1e-6 || fabs(a.levelPct - b.levelPct) > 1e-6 ||
          fabs(a.hoursLeft - b.hoursLeft) > 1e-6 || fabs(a.avgMa - b.avgMa) > 1e-6)
        differing++;
    }
    if (differing != 0) printf("  after a reset: %d of %d estimates differ from a fresh estimator\n", differing, compared);
    CHECK(compared == 180 && ready >= 100);
    CHECK(differing == 0);
    CHECK(used.binCount() == fresh.binCount());
  }

  // a charger appearing without being announced: the readings jump up, so start over
  {
    battest::Estimator est;
    Cell cell;
    uint32_t t = discharge(est, cell, 90, 3.0, 2.0, 0);
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::READY);
    for (uint32_t k = 0; k < 20 * 60; k += 5) est.add(t + k, 4.15f);  // charging: far above the discharge line
    const battest::Estimate e = est.estimate(cutoff, 0);
    CHECK(e.state == battest::Estimate::LEARNING);
    CHECK(est.binCount() <= 4);
  }

  // the time base going backwards (a reboot of the counter) restarts the estimate
  {
    battest::Estimator est;
    Cell cell;
    discharge(est, cell, 90, 3.0, 2.0, 5000);
    est.add(100, 3.9f);
    CHECK(est.estimate(cutoff, 0).state == battest::Estimate::LEARNING);
    CHECK(est.binCount() == 0);
  }

  // the first minutes after leaving the charger fall faster than the cell drains: ignored
  {
    battest::Estimator est;
    Cell cell;
    for (uint32_t sec = 0; sec < 4 * 3600; sec += 5) {
      const double pct = 90 - 3.0 * sec / 3600.0;
      double v = cell.read(pct);
      if (sec < 600) v += 0.04 * exp(-(double)sec / 200.0);  // settling after the charger
      est.add(sec, (float)v);
    }
    const battest::Estimate e = est.estimate(cutoff, 0);
    CHECK(e.state == battest::Estimate::READY);
    if (fabs(e.pctPerHour - 3.0) > 0.15) printf("  settling: %.2f %%/h\n", e.pctPerHour);
    CHECK_NEAR(e.pctPerHour, 3.0, 0.15);
  }

  // the constants in use
  const battest::Params p;
  CHECK(p.binSec == 300 && p.skipSec == 900 && p.readySec == 1800);
  CHECK(p.minWindowSec == 7200 && p.maxWindowSec == 28800);
  CHECK(p.maxWindowSec <= (uint32_t)battest::kMaxBins * p.binSec);  // the bins cover the longest window
}

void testLowBattery() {
  section("low battery guard");
  using lowbat::BootAction;
  // a clear sag, sustained: shuts down after a minute of readings
  {
    lowbat::Guard g;
    g.configure(true, 3.30f);
    CHECK_NEAR(g.cutoffV(), 3.30, 1e-6);
    for (uint32_t t = 0; t < 60; t += 5) CHECK(!g.feed(t, 3.28f, true));  // 12 readings, but not yet a minute
    CHECK(g.tripping());
    CHECK(g.feed(60, 3.28f, true));
    CHECK(g.feed(65, 3.29f, true));  // and it stays tripped
  }
  // sparse readings (the clock was busy): time alone is not enough, three readings are needed
  {
    lowbat::Guard g;
    g.configure(true, 3.30f);
    CHECK(!g.feed(0, 3.20f, true));
    CHECK(!g.feed(100, 3.20f, true));  // a minute and a half later, but only the second reading
    CHECK(g.feed(110, 3.20f, true));
  }
  // one dip is nothing
  {
    lowbat::Guard g;
    g.configure(true, 3.30f);
    uint32_t t = 0;
    for (int i = 0; i < 100; i++, t += 5) {
      const float v = (i % 20 == 7) ? 3.10f : 3.55f;
      CHECK(!g.feed(t, v, true));
    }
    CHECK(!g.tripping());
  }
  // dips every 30 s, each followed by a good reading: never trips
  {
    lowbat::Guard g;
    g.configure(true, 3.30f);
    uint32_t t = 0;
    for (int i = 0; i < 200; i++, t += 5) CHECK(!g.feed(t, (i % 6 == 0) ? 3.20f : 3.40f, true));
  }
  // hovering within the hysteresis band keeps the countdown running
  {
    lowbat::Guard g;
    g.configure(true, 3.30f);
    CHECK(!g.feed(0, 3.29f, true));
    CHECK(!g.feed(5, 3.31f, true));   // above the cut-off but not by the margin: neither trips nor cancels
    CHECK(g.tripping());
    CHECK(!g.feed(10, 3.29f, true));
    CHECK(g.feed(65, 3.29f, true));
    CHECK(!g.feed(70, 3.33f, true));  // clearly above: cancelled
    CHECK(!g.tripping());
    CHECK(!g.feed(75, 3.29f, true));  // and a fresh minute is needed
  }
  // charging or USB: the guard stands down
  {
    lowbat::Guard g;
    g.configure(true, 3.30f);
    for (uint32_t t = 0; t < 120; t += 5) g.feed(t, 3.28f, true);
    CHECK(g.tripping());
    CHECK(!g.feed(125, 3.28f, false));
    CHECK(!g.tripping());
    CHECK(!g.feed(130, 3.28f, true));  // restarts the minute
  }
  // switched off, and clamped to the limits the setting allows
  {
    lowbat::Guard g;
    g.configure(false, 3.30f);
    for (uint32_t t = 0; t < 300; t += 5) CHECK(!g.feed(t, 3.0f, true));
    g.configure(true, 2.0f);
    CHECK_NEAR(g.cutoffV(), lowbat::kMinCutoffV, 1e-6);
    g.configure(true, 5.0f);
    CHECK_NEAR(g.cutoffV(), lowbat::kMaxCutoffV, 1e-6);
    g.configure(true, 3.30f);
    CHECK(!g.feed(0, 0.0f, true));  // no reading is no verdict
    CHECK(!g.feed(5, NAN, true));
    CHECK(!g.tripping());
  }

  // what to do at power-up
  CHECK(lowbat::bootDecision(true, 3.90f, 3.30f, false) == BootAction::RUN);
  CHECK(lowbat::bootDecision(true, 3.29f, 3.30f, false) == BootAction::STAY_ASLEEP);  // flat on a cold start
  CHECK(lowbat::bootDecision(true, 3.30f, 3.30f, false) == BootAction::RUN);
  // after a shutdown the cell has to come back well above the cut-off
  CHECK(lowbat::bootDecision(true, 3.29f, 3.30f, true) == BootAction::STAY_ASLEEP);
  CHECK(lowbat::bootDecision(true, 3.45f, 3.30f, true) == BootAction::STAY_ASLEEP);  // rebounded, not charged
  CHECK(lowbat::bootDecision(true, 3.69f, 3.30f, true) == BootAction::STAY_ASLEEP);
  CHECK(lowbat::bootDecision(true, 3.70f, 3.30f, true) == BootAction::RUN);          // charging got it there
  CHECK(lowbat::bootDecision(true, 4.10f, 3.30f, true) == BootAction::RUN);
  // switched off, or no reading
  CHECK(lowbat::bootDecision(false, 3.0f, 3.30f, true) == BootAction::RUN);
  CHECK(lowbat::bootDecision(true, 0.0f, 3.30f, true) == BootAction::RUN);
  CHECK(lowbat::bootDecision(true, NAN, 3.30f, false) == BootAction::RUN);
  // the restart level sits above anything the rebound of a just-unloaded cell can reach
  CHECK(lowbat::kRestartV >= lowbat::kMaxCutoffV + 0.05f);
}

// ===========================================================================
// Clock drift
// ===========================================================================
void testDrift() {
  section("clock drift");
  const int64_t kHour = 3600LL * 1000000;
  // the clock runs `ppm` fast (positive) or slow; syncs every `everySec` seconds, each with
  // `jitterMs` of network error, from `startMono`
  auto run = [&](double ppm, double jitterMs, int hours, int everySec, uint32_t seed, drift::Tracker *tr) {
    uint32_t rng = seed;
    auto noise = [&]() {
      rng = rng * 1664525u + 1013904223u;
      return (((rng >> 8) & 0xFFFFFF) / 16777216.0 - 0.5) * 2 * jitterMs;  // uniform +-jitter
    };
    const int64_t mono0 = 12345678;
    const int64_t sys0 = 1760000000LL * 1000000;
    for (int64_t s = 0; s <= (int64_t)hours * 3600; s += everySec) {
      const int64_t mono = mono0 + s * 1000000;
      // true time = sys0 + s; the crystal-driven timer reads s*(1 - ppm*1e-6) of it... a fast clock gains: its
      // timer advances more than true time
      const double trueElapsedUs = (double)s * 1e6 / (1.0 + ppm * 1e-6);
      const int64_t sysNtp = sys0 + (int64_t)(trueElapsedUs + noise() * 1000.0);
      tr->addSync(mono, sysNtp);
    }
  };

  for (double ppm : {0.0, 2.0, -2.0, 10.0, -10.0, 25.0}) {
    drift::Tracker tr;
    run(ppm, 30, 24, 3600, 99, &tr);
    const drift::Result r = tr.result();
    CHECK(r.valid && r.points >= 20);
    if (fabs(r.ppm - ppm) > 1.0) printf("  true %+.1f ppm, measured %+.2f +- %.2f\n", ppm, r.ppm, r.errorPpm);
    CHECK_NEAR(r.ppm, ppm, 1.0);
    CHECK_NEAR(r.secPerDay, ppm * 0.0864, 0.1);
    CHECK(r.errorPpm > 0 && r.errorPpm < 1.5);
    CHECK(r.spanHours > 22 && r.spanHours < 25);
  }
  // a longer baseline is tighter
  {
    drift::Tracker a, b;
    run(5.0, 40, 6, 3600, 7, &a);
    run(5.0, 40, 40, 3600, 7, &b);
    CHECK(a.result().valid && b.result().valid);
    CHECK(b.result().errorPpm < a.result().errorPpm);
    CHECK_NEAR(b.result().ppm, 5.0, 0.6);
  }
  // not enough history: no figure
  {
    drift::Tracker tr;
    CHECK(!tr.result().valid);
    run(5.0, 20, 2, 3600, 3, &tr);  // 3 points over 2 hours
    CHECK(!tr.result().valid);
    drift::Tracker t2;
    run(5.0, 20, 3, 3600, 3, &t2);  // 4 points over 3 hours
    CHECK(t2.result().valid);
  }
  // an outlier sync (a bad network path, 400 ms off) does not move the figure much, wherever it is
  for (int badHour : {0, 1, 11, 23, 24}) {
    drift::Tracker tr;
    const int64_t mono0 = 1000000;
    const int64_t sys0 = 1760000000LL * 1000000;
    for (int h = 0; h <= 24; h++) {
      int64_t sys = sys0 + (int64_t)h * kHour;
      if (h == badHour) sys += 400000;  // 400 ms late
      tr.addSync(mono0 + (int64_t)h * kHour, sys);
    }
    const drift::Result r = tr.result();
    CHECK(r.valid);
    if (fabs(r.ppm) > 0.5) printf("  outlier at hour %d: %+.2f ppm\n", badHour, r.ppm);
    CHECK_NEAR(r.ppm, 0.0, 0.5);
  }
  // four points are not enough if they are close together
  {
    drift::Tracker tr;
    for (int i = 0; i < 4; i++) tr.addSync(1000000LL + i * 30LL * 60 * 1000000, 1760000000LL * 1000000 + i * 30LL * 60 * 1000000);
    CHECK(tr.points() == 4 && !tr.result().valid);  // an hour and a half of history
  }
  // syncs closer than half an hour are not new information
  {
    drift::Tracker tr;
    for (int i = 0; i < 100; i++) tr.addSync(1000000LL + i * 60LL * 1000000, 1760000000LL * 1000000 + i * 60LL * 1000000);
    CHECK(tr.points() == 4);  // 0, 30, 60, 90 minutes
  }
  // the timer restarting (a reboot) forgets the old points; the ring keeps the newest
  {
    drift::Tracker tr;
    run(3.0, 10, 10, 3600, 5, &tr);
    CHECK(tr.points() == 11);
    tr.addSync(500, 1760000000LL * 1000000);  // the timer is back near zero
    CHECK(tr.points() == 1 && !tr.result().valid);
    drift::Tracker many;
    run(3.0, 10, 200, 3600, 5, &many);
    CHECK(many.points() == drift::kMaxPoints);
    CHECK_NEAR(many.result().ppm, 3.0, 0.5);
  }
}

// ===========================================================================
// Backup WiFi
// ===========================================================================
void testBackupWifiSettings() {
  section("backup WiFi: settings");
  // the names people will try for the backup network and its password
  const char *ssidNames[] = {"wifi_backup_ssid", "WiFi backup SSID", "Wi-Fi-Backup-SSID", "backup_ssid", "Backup SSID",
                             "backup-wifi", "backup_network", "ssid2", "SSID 2", "wifi_ssid2", "second_ssid",
                             "fallback_ssid", "alt_ssid"};
  for (const char *k : ssidNames) {
    const Applied a = apply(std::string(k) + " = Phone Hotspot");
    if (strcmp(a.s.wifiBackupSsid, "Phone Hotspot") != 0) printf("  [%s] gave [%s]\n", k, a.s.wifiBackupSsid);
    CHECK_STR(a.s.wifiBackupSsid, "Phone Hotspot");
    CHECK_STR(a.s.wifiSsid, "");  // the main network is left alone
    CHECK(a.r.problems() == 0 && a.r.changed == 1);
    CHECK(settingIsUserSet(a.s, idx("wifi_backup_ssid")) && !settingIsUserSet(a.s, idx("wifi_ssid")));
  }
  const char *passNames[] = {"wifi_backup_password", "WiFi backup password", "backup_password", "backup_pass",
                             "backup_wifi_password", "password2", "pass2", "wifi_password2", "second_password",
                             "fallback_password", "alt_password", "backup_key"};
  for (const char *k : passNames) {
    const Applied a = apply(std::string(k) + " = \"h0tsp0t pw\"");
    if (strcmp(a.s.wifiBackupPassword, "h0tsp0t pw") != 0) printf("  [%s] gave [%s]\n", k, a.s.wifiBackupPassword);
    CHECK_STR(a.s.wifiBackupPassword, "h0tsp0t pw");
    CHECK_STR(a.s.wifiPassword, "");
    CHECK(a.r.problems() == 0 && a.r.changed == 1);
  }
  // the plain names still mean the main network
  const Applied both = apply("ssid = Home\npassword = homepw\nbackup_ssid = Phone\nbackup_password = phonepw\n");
  CHECK_STR(both.s.wifiSsid, "Home");
  CHECK_STR(both.s.wifiPassword, "homepw");
  CHECK_STR(both.s.wifiBackupSsid, "Phone");
  CHECK_STR(both.s.wifiBackupPassword, "phonepw");
  CHECK(both.s.hasMainWifi() && both.s.hasBackupWifi());

  // quoting works as for the main network
  CHECK_STR(apply("wifi_backup_ssid = \"Bob's Phone\"").s.wifiBackupSsid, "Bob's Phone");
  CHECK_STR(apply("wifi_backup_ssid = Home#1").s.wifiBackupSsid, "Home#1");
  CHECK_STR(apply("wifi_backup_ssid = Phone # my hotspot").s.wifiBackupSsid, "Phone");

  // limits: 32 characters for a name, 64 for a password
  CHECK(apply("wifi_backup_ssid = " + std::string(32, 'x')).r.bad == 0);
  CHECK(apply("wifi_backup_ssid = " + std::string(33, 'x')).r.bad == 1);
  CHECK(apply("wifi_backup_password = " + std::string(64, 'x')).r.bad == 0);
  CHECK(apply("wifi_backup_password = " + std::string(65, 'x')).r.bad == 1);

  // forgetting the backup, and an open backup network
  Settings has;
  snprintf(has.wifiBackupSsid, sizeof has.wifiBackupSsid, "Phone");
  snprintf(has.wifiBackupPassword, sizeof has.wifiBackupPassword, "pw");
  has.userSet |= (1ull << idx("wifi_backup_ssid")) | (1ull << idx("wifi_backup_password"));
  const Applied gone = apply("wifi_backup_ssid =\nwifi_backup_password =", has);
  CHECK_STR(gone.s.wifiBackupSsid, "");
  CHECK_STR(gone.s.wifiBackupPassword, "");
  CHECK(gone.r.cleared == 2 && !gone.s.hasBackupWifi());
  const Applied openNet = apply("wifi_backup_password = \"\"", has);
  CHECK_STR(openNet.s.wifiBackupPassword, "");
  CHECK_STR(openNet.s.wifiBackupSsid, "Phone");
  CHECK(settingIsUserSet(openNet.s, idx("wifi_backup_password")));

  // what counts as having a backup
  Settings s;
  CHECK(!s.hasMainWifi() && !s.hasBackupWifi());
  snprintf(s.wifiBackupSsid, sizeof s.wifiBackupSsid, "Phone");
  CHECK(!s.hasMainWifi() && s.hasBackupWifi());  // a backup alone is still a network to join
  snprintf(s.wifiSsid, sizeof s.wifiSsid, "Home");
  CHECK(s.hasMainWifi() && s.hasBackupWifi());
  snprintf(s.wifiBackupSsid, sizeof s.wifiBackupSsid, "Home");
  CHECK(!s.hasBackupWifi());  // the same network twice is no backup
  snprintf(s.wifiBackupSsid, sizeof s.wifiBackupSsid, "home");
  CHECK(s.hasBackupWifi());  // names are case sensitive

  // flash round trip
  Settings t;
  CHECK(applyStoredSetting(t, idx("wifi_backup_ssid"), "Phone Hotspot"));
  CHECK(applyStoredSetting(t, idx("wifi_backup_password"), "h0tsp0t pw"));
  CHECK_STR(t.wifiBackupSsid, "Phone Hotspot");
  CHECK_STR(t.wifiBackupPassword, "h0tsp0t pw");
  CHECK(fmt(t, idx("wifi_backup_ssid")) == "Phone Hotspot" && fmt(t, idx("wifi_backup_password")) == "h0tsp0t pw");

  // the password stays out of the example file and of the summary lines; the name may be shown
  Settings withBoth = both.s;
  snprintf(withBoth.wifiBackupPassword, sizeof withBoth.wifiBackupPassword, "backup-pw-123");
  static char buf[16384];
  const size_t n = renderExampleConfig(withBoth, "", buf, sizeof buf);
  CHECK(n > 0 && strstr(buf, "backup-pw-123") == nullptr && strstr(buf, "homepw") == nullptr);
  CHECK(strstr(buf, "# wifi_backup_password = \"your password\"") != nullptr);
  CHECK(strstr(buf, "# wifi_backup_ssid = \"Phone\"") != nullptr);
}

// ===========================================================================
// WiFi mode and the idle CPU clock
// ===========================================================================
void testPowerSettings() {
  section("WiFi mode and idle clock settings");
  const Settings d;
  CHECK(d.wifiMode == WIFIMODE_ALWAYS && !d.syncMode());  // nothing changes unless asked for
  CHECK(d.cpuIdle == CPUIDLE_OFF && d.cpuIdleMhz() == 0);
  CHECK_STR(settingNvsKey(idx("wifi_mode")), "wmode");
  CHECK_STR(settingNvsKey(idx("cpu_idle_mhz")), "cpuidle");

  for (const char *v : {"sync", "Sync", "sync-only", "periodic", "saver", " SYNC "}) {
    const Applied a = apply(std::string("wifi_mode = ") + v);
    if (a.s.wifiMode != WIFIMODE_SYNC) printf("  [%s] was not sync\n", v);
    CHECK(a.s.wifiMode == WIFIMODE_SYNC && a.r.problems() == 0 && a.s.syncMode());
  }
  for (const char *v : {"always", "on", "connected", "stay"}) {
    Settings sync;
    sync.wifiMode = WIFIMODE_SYNC;
    const Applied a = apply(std::string("wifi_mode = ") + v, sync);
    CHECK(a.s.wifiMode == WIFIMODE_ALWAYS && a.r.problems() == 0);
  }
  for (const char *v : {"sometimes", "1", "wifi", "30"}) {
    const Applied a = apply(std::string("wifi_mode = ") + v);
    if (a.r.bad != 1) printf("  [%s] was not refused\n", v);
    CHECK(a.r.bad == 1 && a.s.wifiMode == WIFIMODE_ALWAYS);
  }
  CHECK(apply("WiFi Mode: sync").s.syncMode());
  CHECK(apply("wifi_sync = sync").s.syncMode());  // an alias
  CHECK(apply("radio-mode = sync").s.wifiMode == WIFIMODE_SYNC);
  {  // sync mode needs the radio: with WiFi off it is moot
    const Applied a = apply("wifi_mode = sync\nwifi = off");
    CHECK(a.s.wifiMode == WIFIMODE_SYNC && !a.s.syncMode());
  }

  // the idle clock: a number, off, or the usual "MHz" after it
  struct Case {
    const char *text;
    int mhz;  // the value, 0 = off
  };
  const Case cases[] = {{"off", 0}, {"same", 0}, {"none", 0}, {"no", 0}, {"0", 0},   {"80", 80}, {"80 MHz", 80}, {"80mhz", 80},
                        {"40", 40}, {"40 MHz", 40}, {"20", 20}, {"20MHz", 20}};
  for (const Case &c : cases) {
    Settings fast;
    fast.cpuSpeed = CPU_240;  // so that every idle value is below it
    const Applied a = apply(std::string("cpu_idle_mhz = ") + c.text, fast);
    if (a.r.problems() != 0 || a.s.cpuIdleMhz() != c.mhz) printf("  [%s] gave %d (problems %d)\n", c.text, a.s.cpuIdleMhz(), a.r.problems());
    CHECK(a.r.problems() == 0 && a.s.cpuIdleMhz() == c.mhz);
  }
  // 10 MHz is not on offer (a frame would take a quarter of a second to draw), nor is anything in between
  for (const char *v : {"10", "10 MHz", "10mhz", "30", "5", "160", "240", "fast", "20.5"}) {
    const Applied a = apply(std::string("cpu_idle_mhz = ") + v);
    if (a.r.bad != 1) printf("  [%s] was not refused\n", v);
    CHECK(a.r.bad == 1 && a.s.cpuIdle == CPUIDLE_OFF);
  }
  CHECK(apply("cpu idle = 40").s.cpuIdle == CPUIDLE_40);  // spelling and aliases
  CHECK(apply("idle_clock = 20").s.cpuIdle == CPUIDLE_20);
  CHECK(apply("CPU-Idle-MHz: 20").s.cpuIdle == CPUIDLE_20);
  {  // a value an older firmware saved in flash and this one no longer has is ignored, not misread
    Settings old;
    char err[64] = "";
    CHECK(!applyStoredSetting(old, idx("cpu_idle_mhz"), "10", err, sizeof err) && old.cpuIdle == CPUIDLE_OFF && err[0]);
    CHECK(applyStoredSetting(old, idx("cpu_idle_mhz"), "20") && old.cpuIdle == CPUIDLE_20);
  }

  // an idle clock that is not below the working one means "no change": the working clock is the floor
  Settings s;
  s.cpuIdle = CPUIDLE_80;
  s.cpuSpeed = CPU_80;
  CHECK(s.cpuIdleMhz() == 0);
  s.cpuSpeed = CPU_160;
  CHECK(s.cpuIdleMhz() == 80);
  s.cpuIdle = CPUIDLE_40;
  s.cpuSpeed = CPU_80;
  CHECK(s.cpuIdleMhz() == 40);
  s.cpuIdle = CPUIDLE_20;
  s.cpuSpeed = CPU_240;
  CHECK(s.cpuIdleMhz() == 20);
  s.cpuIdle = (uint8_t)(CPUIDLE_20 + 1);  // a number that is no choice (it was 10 MHz once): no idle clock
  CHECK(s.cpuIdleMhz() == 0);

  // the console: on by default; auto and off by the usual words
  CHECK(d.console == CONSOLE_ON);
  CHECK_STR(settingNvsKey(idx("console")), "console");
  struct ConsoleCase {
    const char *text;
    int mode;
  };
  const ConsoleCase consoleCases[] = {{"on", CONSOLE_ON},     {"On", CONSOLE_ON},   {"yes", CONSOLE_ON},   {"always", CONSOLE_ON}, {"true", CONSOLE_ON},
                                      {"1", CONSOLE_ON},      {"auto", CONSOLE_AUTO}, {"AUTO", CONSOLE_AUTO}, {"usb", CONSOLE_AUTO},
                                      {"automatic", CONSOLE_AUTO}, {"off", CONSOLE_OFF}, {"no", CONSOLE_OFF}, {"never", CONSOLE_OFF},
                                      {"false", CONSOLE_OFF}, {"0", CONSOLE_OFF},   {" Off ", CONSOLE_OFF}};
  for (const ConsoleCase &c : consoleCases) {
    Settings other;  // start from a value that is not the one asked for
    other.console = (uint8_t)(c.mode == CONSOLE_AUTO ? CONSOLE_OFF : CONSOLE_AUTO);
    const Applied a = apply(std::string("console = ") + c.text, other);
    if (a.r.problems() != 0 || a.s.console != c.mode) printf("  console [%s] gave %d (problems %d)\n", c.text, a.s.console, a.r.problems());
    CHECK(a.r.problems() == 0 && a.s.console == c.mode);
  }
  for (const char *v : {"maybe", "2", "quiet", "115200"}) {
    const Applied a = apply(std::string("console = ") + v);
    if (a.r.bad != 1) printf("  console [%s] was not refused\n", v);
    CHECK(a.r.bad == 1 && a.s.console == CONSOLE_ON);
  }
  CHECK(apply("serial = off").s.console == CONSOLE_OFF);  // the names people will try
  CHECK(apply("Serial Console: auto").s.console == CONSOLE_AUTO);
  CHECK(apply("usb_console = off").s.console == CONSOLE_OFF);
  CHECK(apply("console-mode = auto").s.console == CONSOLE_AUTO);
  CHECK(apply("log = off").s.console == CONSOLE_OFF);

  // Spotify while music plays, in sync mode: live by default, "off" lets the radio sleep
  CHECK(d.spotifyLive);
  CHECK_STR(settingNvsKey(idx("spotify_live")), "splive");
  CHECK(!apply("spotify_live = off").s.spotifyLive && apply("spotify_live = off").r.problems() == 0);
  CHECK(!apply("Spotify Live: no").s.spotifyLive);
  CHECK(!apply("spotify_follow = off").s.spotifyLive && !apply("spotify-stay-connected = 0").s.spotifyLive);
  {
    Settings off;
    off.spotifyLive = false;
    CHECK(apply("spotify_live = on", off).s.spotifyLive);
    CHECK(!apply("spotify_live = sometimes", off).s.spotifyLive && apply("spotify_live = sometimes", off).r.bad == 1);
  }

  // the power log: off unless it is asked for (it writes to the flash)
  CHECK(!d.powerLog);
  CHECK_STR(settingNvsKey(idx("power_log")), "plog");
  CHECK(apply("power_log = on").s.powerLog && apply("power_log = on").r.problems() == 0);
  CHECK(apply("Power Log: yes").s.powerLog && apply("battery_log = on").s.powerLog && apply("log-power = 1").s.powerLog);
  {
    Settings on;
    on.powerLog = true;
    CHECK(!apply("power_log = off", on).s.powerLog && !apply("POWERLOG = no", on).s.powerLog);
    CHECK(apply("power_log = hourly", on).s.powerLog && apply("power_log = hourly", on).r.bad == 1);  // a value it does not know changes nothing
    CHECK(apply("log = off", on).s.powerLog && apply("log = off", on).s.console == CONSOLE_OFF);      // "log" alone is the console's name
    CHECK(settingIsUserSet(apply("power_log = off", on).s, idx("power_log")));                        // the 32nd bit of the mask is a bit like the others
    CHECK(!settingIsUserSet(apply("power_log =", apply("power_log = on").s).s, idx("power_log")));
  }

  // they come out of the example file, with their help text
  static char buf[16384];
  Settings cur;
  cur.wifiMode = WIFIMODE_SYNC;
  cur.cpuIdle = CPUIDLE_20;
  cur.console = CONSOLE_AUTO;
  cur.spotifyLive = false;
  cur.powerLog = true;
  CHECK(renderExampleConfig(cur, "", buf, sizeof buf) > 0);
  CHECK(strstr(buf, "# wifi_mode = sync") != nullptr && strstr(buf, "# cpu_idle_mhz = 20") != nullptr);
  CHECK(strstr(buf, "# console = auto") != nullptr && strstr(buf, "# spotify_live = off") != nullptr);
  CHECK(strstr(buf, "# power_log = on") != nullptr && strstr(buf, "ESP32-S3-RLCD-PowerLog.csv") != nullptr);
  CHECK(strstr(buf, "off, 80, 40 or 20:") != nullptr && strstr(buf, "20 or 10") == nullptr);  // the help does not offer 10 MHz
  const Applied back = apply(buf);  // and read back, they give the same
  CHECK(back.s.wifiMode == WIFIMODE_ALWAYS);  // (everything commented out: nothing is applied)
}

// ===========================================================================
// Battery capacity and voltage calibration
// ===========================================================================
void testBatteryCalibrationSetting() {
  section("battery calibration and capacity settings");
  const Settings d;
  CHECK_NEAR(d.batteryCalibration, 1.0, 1e-9);  // no correction unless asked for
  CHECK(d.batteryCapacityMah == 0);
  CHECK(!settingIsSecret(idx("battery_calibration")));
  CHECK_STR(settingNvsKey(idx("battery_calibration")), "batcal");

  // the names people will try
  const char *names[] = {"battery_calibration", "Battery Calibration", "Battery-Calibration", "battery_cal", "adc_calibration",
                         "ADC calibration", "voltage_calibration", "battery_scale", "battery_gain", "adc_scale"};
  for (const char *k : names) {
    const Applied a = apply(std::string(k) + " = 1.016");
    if (fabs(a.s.batteryCalibration - 1.016) > 1e-6) printf("  [%s] gave %g\n", k, (double)a.s.batteryCalibration);
    CHECK_NEAR(a.s.batteryCalibration, 1.016, 1e-6);
    CHECK(a.r.problems() == 0 && a.r.changed == 1);
    CHECK(settingIsUserSet(a.s, idx("battery_calibration")));
  }
  // spellings of the number: a decimal comma, four decimals
  CHECK_NEAR(apply("battery_calibration = 1,016").s.batteryCalibration, 1.016, 1e-6);
  CHECK_NEAR(apply("battery_calibration = \"1.0157\"").s.batteryCalibration, 1.0157, 1e-6);
  CHECK_NEAR(apply("battery_calibration = 0.9847").s.batteryCalibration, 0.9847, 1e-6);
  // the range is 0.80 to 1.25, inclusive; anything else is a mistake and changes nothing
  CHECK_NEAR(apply("battery_calibration = 0.8").s.batteryCalibration, 0.8, 1e-6);
  CHECK_NEAR(apply("battery_calibration = 1.25").s.batteryCalibration, 1.25, 1e-6);
  for (const char *bad : {"0.79", "1.26", "0", "-1", "2", "abc", "1.0.1"}) {
    const Applied a = apply(std::string("battery_calibration = ") + bad);
    if (a.r.bad != 1) printf("  [%s] was not refused\n", bad);
    CHECK(a.r.bad == 1 && a.r.changed == 0);
    CHECK_NEAR(a.s.batteryCalibration, 1.0, 1e-9);
  }
  // forgetting it brings the default back (a build can have its own: here the pristine one)
  Settings set;
  set.batteryCalibration = 1.03f;
  set.userSet |= 1ull << idx("battery_calibration");
  const Applied gone = apply("battery_calibration =", set);
  CHECK_NEAR(gone.s.batteryCalibration, 1.0, 1e-9);
  CHECK(gone.r.cleared == 1 && !settingIsUserSet(gone.s, idx("battery_calibration")));
  Settings withDefault;
  withDefault.batteryCalibration = 1.02f;  // config.h or secrets.h said so
  Settings fromFlash = withDefault;
  const std::string text = "battery_calibration = 1.05\n";
  Settings defaults = withDefault;
  applyConfigText(fromFlash, defaults, text.data(), text.size());
  CHECK_NEAR(fromFlash.batteryCalibration, 1.05, 1e-6);
  const std::string forget = "battery_calibration =\n";
  applyConfigText(fromFlash, defaults, forget.data(), forget.size());
  CHECK_NEAR(fromFlash.batteryCalibration, 1.02, 1e-6);  // back to the build's own value, not to 1

  // the figure survives flash (it is stored as text) with its four decimals
  Settings w;
  w.batteryCalibration = 1.0157f;
  Settings x;
  CHECK(fmt(w, idx("battery_calibration")) == "1.0157");
  CHECK(applyStoredSetting(x, idx("battery_calibration"), fmt(w, idx("battery_calibration")).c_str()));
  CHECK_NEAR(x.batteryCalibration, 1.0157, 1e-6);
  CHECK(fmt(Settings(), idx("battery_calibration")) == "1");

  // the capacity: more names, and a unit
  CHECK(apply("battery_capacity = 2500").s.batteryCapacityMah == 2500);
  CHECK(apply("capacity_mah = 3400").s.batteryCapacityMah == 3400);
  CHECK(apply("Battery Capacity mAh = \"2600 mAh\"").s.batteryCapacityMah == 2600);

  // the example file explains the calibration and names the serial command
  static char buf[16384];
  const size_t n = renderExampleConfig(Settings(), "", buf, sizeof buf);
  CHECK(n > 0);
  CHECK(strstr(buf, "# battery_calibration = 1\n") != nullptr);
  CHECK(strstr(buf, "batcal 4.20") != nullptr);
  CHECK(strstr(buf, "# battery_capacity_mah = 0\n") != nullptr);
}

void testWifiPick() {
  section("backup WiFi: which network next");
  using namespace wifipick;
  const uint32_t kMin = 60UL * 1000UL;

  {  // no names, one name
    Picker none;
    none.configure(false, false);
    CHECK(none.nextAttempt() == NET_NONE && none.nextAttempt() == NET_NONE);
    Picker onlyMain;
    onlyMain.configure(true, false);
    for (int i = 0; i < 6; i++) CHECK(onlyMain.nextAttempt() == NET_MAIN);
    onlyMain.joined(NET_MAIN, 0);
    CHECK(!onlyMain.timeToLookForMain(100 * 60 * kMin) && !onlyMain.onBackup());
    Picker onlyBackup;
    onlyBackup.configure(false, true);
    for (int i = 0; i < 6; i++) CHECK(onlyBackup.nextAttempt() == NET_BACKUP);
    onlyBackup.joined(NET_BACKUP, 0);
    CHECK(onlyBackup.onBackup() && !onlyBackup.timeToLookForMain(100 * 60 * kMin));  // nothing to go back to
  }

  {  // both: main first, then they alternate
    Picker p;
    p.configure(true, true);
    const Net want[] = {NET_MAIN, NET_BACKUP, NET_MAIN, NET_BACKUP, NET_MAIN};
    for (Net w : want) CHECK(p.nextAttempt() == w);
  }

  {  // after a connection is lost the main network is tried first again, whatever was tried last
    Picker p;
    p.configure(true, true);
    CHECK(p.nextAttempt() == NET_MAIN);
    p.joined(NET_MAIN, 1000);
    p.lost();
    CHECK(p.nextAttempt() == NET_MAIN);  // not the backup, although the last attempt was the main one
    CHECK(p.nextAttempt() == NET_BACKUP);
    p.joined(NET_BACKUP, 5000);
    CHECK(p.onBackup() && p.current() == NET_BACKUP);
    p.lost();
    CHECK(p.current() == NET_NONE && !p.onBackup());
    CHECK(p.nextAttempt() == NET_MAIN);
  }

  {  // looking for the main network from the backup: not before 10 minutes, then every 10
    Picker p;
    p.configure(true, true);
    const uint32_t t0 = 123456;
    p.joined(NET_BACKUP, t0);
    CHECK(!p.timeToLookForMain(t0) && !p.timeToLookForMain(t0 + 10 * kMin - 1));
    CHECK(p.timeToLookForMain(t0 + 10 * kMin) && p.timeToLookForMain(t0 + 3 * 60 * kMin));
    p.lookedForMain(t0 + 10 * kMin, false);  // it was not there
    CHECK(!p.timeToLookForMain(t0 + 10 * kMin + 1) && !p.timeToLookForMain(t0 + 20 * kMin - 1));
    CHECK(p.timeToLookForMain(t0 + 20 * kMin));
    p.joined(NET_MAIN, t0 + 30 * kMin);
    CHECK(!p.timeToLookForMain(t0 + 300 * kMin));  // on the main network there is nothing to look for
    Picker q;
    q.configure(true, true);
    q.joined(NET_BACKUP, 0);
    q.lost();
    CHECK(!q.timeToLookForMain(100 * 60 * kMin));  // not connected: no looking either
  }

  {  // the millisecond counter wraps after 49 days: the timing must not care
    Picker p;
    p.configure(true, true);
    const uint32_t t0 = 0xFFFFFFFFu - 3 * kMin;
    p.joined(NET_BACKUP, t0);
    // the deadline wrapped past zero; the clock itself has not yet (for the first 3 minutes), and a plain
    // "now >= deadline" would call that moment late already
    CHECK(!p.timeToLookForMain(t0) && !p.timeToLookForMain(t0 + 1 * kMin) && !p.timeToLookForMain(t0 + 2 * kMin));
    CHECK(!p.timeToLookForMain(t0 + 5 * kMin));  // after the wrap, before the deadline
    CHECK(!p.timeToLookForMain(t0 + 10 * kMin - 1));
    CHECK(p.timeToLookForMain(t0 + 10 * kMin));
  }

  {  // going back fails again and again: the looks come further apart (20, 40, 60, 60 minutes)
    Picker p;
    p.configure(true, true);
    uint32_t now = 1000;
    p.joined(NET_BACKUP, now);
    CHECK(p.lookInterval() == 10 * kMin);
    const uint32_t gaps[] = {20 * kMin, 40 * kMin, 60 * kMin, 60 * kMin, 60 * kMin};
    for (uint32_t gap : gaps) {
      now += p.lookInterval();
      CHECK(p.timeToLookForMain(now));
      p.lookedForMain(now, true);  // the main network is in range: leave
      p.lost();
      CHECK(p.nextAttempt() == NET_MAIN);   // ... but it cannot be joined
      CHECK(p.nextAttempt() == NET_BACKUP);  // so back to the backup
      now += 40 * 1000;
      p.joined(NET_BACKUP, now);
      CHECK(p.lookInterval() == gap);
      CHECK(!p.timeToLookForMain(now + gap - 1) && p.timeToLookForMain(now + gap));
    }
    // the main network is joined at last: the next stay on the backup starts over at 10 minutes
    now += p.lookInterval();
    p.lookedForMain(now, true);
    p.lost();
    CHECK(p.nextAttempt() == NET_MAIN);
    p.joined(NET_MAIN, now + 5000);
    CHECK(p.lookInterval() == 10 * kMin);
    p.lost();
    p.joined(NET_BACKUP, now + 20 * kMin);
    CHECK(p.lookInterval() == 10 * kMin && p.timeToLookForMain(now + 30 * kMin));
  }

  {  // a backup that drops for its own reasons is not a failed return
    Picker p;
    p.configure(true, true);
    p.joined(NET_BACKUP, 0);
    p.lost();
    CHECK(p.nextAttempt() == NET_MAIN && p.nextAttempt() == NET_BACKUP);
    p.joined(NET_BACKUP, 1000);
    CHECK(p.lookInterval() == 10 * kMin);
    // looking and finding nothing does not count either
    p.lookedForMain(10 * kMin, false);
    p.lookedForMain(20 * kMin, false);
    CHECK(p.lookInterval() == 10 * kMin);
  }

  // which of the two a connection is, by the name the stack reports
  CHECK(classify("Home", "Home", "Phone") == NET_MAIN);
  CHECK(classify("Phone", "Home", "Phone") == NET_BACKUP);
  CHECK(classify("Elsewhere", "Home", "Phone") == NET_MAIN);   // a name we do not know: the main one
  CHECK(classify("Phone", "", "Phone") == NET_BACKUP);          // only a backup is set
  CHECK(classify("Elsewhere", "", "Phone") == NET_BACKUP);
  CHECK(classify("Home", "Home", "") == NET_MAIN);
  CHECK(classify("", "Home", "Phone") == NET_MAIN);
  CHECK(classify("home", "Home", "Phone") == NET_MAIN && classify("PHONE", "Home", "Phone") == NET_MAIN);  // case matters
}

// A day in the life of the clock: a model of the radio around the Picker, one second at a time.  An
// attempt takes 20 s to give up and 3 s to succeed, as in net_task.cpp.
void testWifiPickDay() {
  section("backup WiFi: a simulated day");
  using namespace wifipick;
  struct Sim {
    Picker pick;
    bool mainUp = true, backupUp = true;
    Net on = NET_NONE;  // connected to
    uint32_t attemptAt = 0, joinAt = 0;
    Net trying = NET_NONE;
    int scans = 0, scansOnMain = 0, switches = 0;
    uint32_t joinedMain = 0;

    void step(uint32_t nowMs) {
      if (on != NET_NONE && ((on == NET_MAIN && !mainUp) || (on == NET_BACKUP && !backupUp))) {
        on = NET_NONE;
        pick.lost();
        attemptAt = nowMs ? nowMs : 1;  // the stack's own reconnect gets 20 s, as in the sketch
        trying = NET_NONE;
      }
      if (on == NET_NONE) {
        if (trying != NET_NONE && nowMs >= joinAt) {
          on = trying;
          trying = NET_NONE;
          pick.joined(on, nowMs);
          if (on == NET_MAIN) joinedMain = nowMs;
        } else if (attemptAt == 0 || nowMs - attemptAt > 20000) {
          trying = pick.nextAttempt();
          const bool up = trying == NET_MAIN ? mainUp : backupUp;
          attemptAt = nowMs ? nowMs : 1;
          joinAt = up ? nowMs + 3000 : 0xFFFFFFFFu;
          if (!up) trying = NET_NONE;
        }
        return;
      }
      if (pick.timeToLookForMain(nowMs)) {
        scans++;
        const bool seen = mainUp;
        pick.lookedForMain(nowMs, seen);
        if (seen) {
          switches++;
          on = NET_NONE;
          pick.lost();
          attemptAt = 0;  // start on the main network at once
          trying = NET_NONE;
        }
      } else if (on == NET_MAIN) {
        // nothing: the clock sits on the main network
      }
    }
  };

  Sim sim;
  sim.pick.configure(true, true);
  sim.mainUp = false;  // away from home: only the phone is there
  uint32_t t = 1000;
  uint32_t onBackupAt = 0;
  for (; t < 2 * 3600 * 1000UL; t += 1000) {
    sim.step(t);
    if (sim.on == NET_BACKUP && !onBackupAt) onBackupAt = t;
  }
  CHECK(onBackupAt > 0 && onBackupAt <= 1000 + 20000 + 3000 + 1000);  // main fails (20 s), the backup works (3 s)
  CHECK(sim.on == NET_BACKUP);
  CHECK(sim.scans >= 11 && sim.scans <= 12);  // a look every 10 minutes, nothing found

  const uint32_t homeAt = t;  // the main network is up again
  sim.mainUp = true;
  for (; t < homeAt + 20 * 60 * 1000UL; t += 1000) sim.step(t);
  CHECK(sim.on == NET_MAIN);
  CHECK(sim.joinedMain > homeAt && sim.joinedMain <= homeAt + 10 * 60 * 1000UL + 5000);  // back within one look interval
  CHECK(sim.switches == 1);

  const int scansBefore = sim.scans;
  for (; t < homeAt + 3 * 3600 * 1000UL; t += 1000) sim.step(t);
  CHECK(sim.scans == scansBefore && sim.on == NET_MAIN);  // no scanning while on the main network

  // the router dies: the clock ends up on the backup within a minute, and comes back when the router does
  sim.mainUp = false;
  const uint32_t deadAt = t;
  uint32_t backAt = 0;
  for (; t < deadAt + 3 * 60 * 1000UL; t += 1000) {
    sim.step(t);
    if (sim.on == NET_BACKUP && !backAt) backAt = t;
  }
  CHECK(backAt > deadAt && backAt <= deadAt + 20000 + 20000 + 3000 + 2000);  // head start, main (20 s), backup
  sim.mainUp = true;
  const uint32_t routerAt = t;
  for (; t < routerAt + 15 * 60 * 1000UL; t += 1000) sim.step(t);
  CHECK(sim.on == NET_MAIN);

  // both gone, then the backup alone comes back: the clock finds it
  sim.mainUp = false;
  sim.backupUp = false;
  for (int i = 0; i < 600; i++, t += 1000) sim.step(t);
  CHECK(sim.on == NET_NONE);
  sim.backupUp = true;
  for (int i = 0; i < 90; i++, t += 1000) sim.step(t);
  CHECK(sim.on == NET_BACKUP);
}

// ===========================================================================
// Firmware update from the SD card
// ===========================================================================
void testFirmwareLogic() {
  section("firmware update: which file is accepted");
  using namespace fwlogic;

  // the real thing: the first bytes of an exported build of this sketch, and of the bootloader the IDE exports with it
  const std::string app = readFile("fixtures/esp32s3_app_head.bin");
  const std::string boot = readFile("fixtures/esp32s3_bootloader_head.bin");
  CHECK(app.size() == kHeadBytes && boot.size() == kHeadBytes);
  const uint8_t *appHead = (const uint8_t *)app.data(), *bootHead = (const uint8_t *)boot.data();
  const uint32_t kRealSize = 1467536;  // that build's .ino.bin
  const uint32_t kSlot = 0x300000;     // an app slot of the README's partition scheme
  CHECK(inspectImage(appHead, app.size(), kRealSize, kSlot) == Problem::NONE);
  CHECK(elfHash(appHead)[0] == 0xb0 && elfHash(appHead)[1] == 0x1e && elfHash(appHead)[2] == 0x2f && elfHash(appHead)[3] == 0x9c);
  CHECK(le32(appHead + 32) == kAppDescMagic);  // (the offsets above are the real ones)
  // the other files the IDE exports next to it are refused, each for its own reason
  CHECK(inspectImage(bootHead, boot.size(), 19984, kSlot) == Problem::TOO_SMALL);       // ...bootloader.bin
  CHECK(inspectImage(bootHead, boot.size(), 3072, kSlot) == Problem::TOO_SMALL);        // ...partitions.bin
  CHECK(inspectImage(bootHead, boot.size(), 16777216, kSlot) == Problem::TOO_BIG);      // ...merged.bin: 16 MB, starts like a bootloader
  CHECK(inspectImage(bootHead, boot.size(), 400 * 1024, kSlot) == Problem::NOT_AN_APP);  // a bootloader of a plausible size

  // every header field the check reads, changed in turn on a copy of the real header
  auto with = [&](size_t at, int value) {
    std::string h = app;
    h[at] = (char)value;
    return h;
  };
  auto problemIn = [&](const std::string &h, uint32_t size, uint32_t slot) {
    return inspectImage((const uint8_t *)h.data(), h.size(), size, slot);
  };
  auto problemOf = [&](const std::string &h, uint32_t size) { return problemIn(h, size, kSlot); };
  CHECK(problemOf(with(0, 0x00), kRealSize) == Problem::NOT_AN_IMAGE);
  CHECK(problemOf(with(0, 0xAA), kRealSize) == Problem::NOT_AN_IMAGE);  // a partition table
  for (int chip : {0, 2, 5, 12, 13, 18}) CHECK(problemOf(with(12, chip), kRealSize) == Problem::WRONG_CHIP);  // other ESP32s
  CHECK(problemOf(with(13, 1), kRealSize) == Problem::WRONG_CHIP);  // 0x0109 is not 9 either
  CHECK(problemOf(with(32, 0x00), kRealSize) == Problem::NOT_AN_APP);
  CHECK(problemOf(with(35, 0x00), kRealSize) == Problem::NOT_AN_APP);  // the last byte of the magic word counts too
  CHECK(problemOf(with(23, 0), kRealSize) == Problem::NO_DIGEST);
  CHECK(problemOf(with(23, 2), kRealSize) == Problem::NO_DIGEST);
  // sizes: the limits are inclusive
  CHECK(problemOf(app, kMinImageBytes - 1) == Problem::TOO_SMALL);
  CHECK(problemOf(app, kMinImageBytes) == Problem::NONE);
  CHECK(problemOf(app, 0) == Problem::TOO_SMALL);
  CHECK(problemOf(app, kSlot) == Problem::NONE);
  CHECK(problemOf(app, kSlot + 1) == Problem::TOO_BIG);
  CHECK(problemOf(app, 0xFFFFFFFFu) == Problem::TOO_BIG);
  CHECK(problemIn(app, kSlot + 1, 0) == Problem::NONE);  // slot size unknown: no upper limit here
  CHECK(inspectImage(appHead, kHeadBytes - 1, kRealSize, kSlot) == Problem::TOO_SMALL);  // a head that was not read in full
  // size problems are reported before header problems
  CHECK(problemOf(with(12, 5), 1000) == Problem::TOO_SMALL);
  CHECK(problemOf(with(12, 5), kSlot + 1) == Problem::TOO_BIG);

  section("firmware update: what the clock decides");
  Inputs in;
  memcpy(in.head, appHead, kHeadBytes);
  in.files = 1;
  in.fileSize = kRealSize;
  in.slotSize = kSlot;
  in.runningElf[0] = 0x42;  // some other build
  CHECK(decide(in).verdict == Verdict::INSTALL && decide(in).problem == Problem::NONE);

  Inputs none = in;
  none.files = 0;
  CHECK(decide(none).verdict == Verdict::NOTHING);
  Inputs two = in;
  two.files = 2;
  CHECK(decide(two).verdict == Verdict::SEVERAL);
  two.head[0] = 0;  // several wins over a bad file: nothing is looked at
  CHECK(decide(two).verdict == Verdict::SEVERAL);
  Inputs noSlot = in;
  noSlot.slotSize = 0;
  CHECK(decide(noSlot).verdict == Verdict::NO_SLOT);
  Inputs junk = in;
  junk.head[0] = 0;
  CHECK(decide(junk).verdict == Verdict::BAD_FILE && decide(junk).problem == Problem::NOT_AN_IMAGE);
  Inputs small = in;
  small.fileSize = 19984;
  CHECK(decide(small).verdict == Verdict::BAD_FILE && decide(small).problem == Problem::TOO_SMALL);

  // the same build as the one that runs: nothing to do, whatever else is true
  Inputs same = in;
  memcpy(same.runningElf, elfHash(appHead), kDigestBytes);
  CHECK(decide(same).verdict == Verdict::SAME_AS_RUNNING);
  same.hasBattery = true;
  same.batteryMv = 3000;
  same.haveLast = true;
  memcpy(same.lastElf, elfHash(appHead), kDigestBytes);
  CHECK(decide(same).verdict == Verdict::SAME_AS_RUNNING);
  // one byte of the ELF hash differs: a different build
  Inputs near = in;
  memcpy(near.runningElf, elfHash(appHead), kDigestBytes);
  near.runningElf[kDigestBytes - 1] ^= 1;
  CHECK(decide(near).verdict == Verdict::INSTALL);
  near.runningElf[kDigestBytes - 1] ^= 1;
  near.runningElf[0] ^= 0x80;
  CHECK(decide(near).verdict == Verdict::INSTALL);

  // installed from the card before, and not what runs: not again
  Inputs again = in;
  again.haveLast = true;
  memcpy(again.lastElf, elfHash(appHead), kDigestBytes);
  CHECK(decide(again).verdict == Verdict::TRIED_BEFORE);
  again.haveLast = false;  // no memory of it: the stored bytes mean nothing
  CHECK(decide(again).verdict == Verdict::INSTALL);
  again.haveLast = true;
  again.lastElf[10] ^= 4;  // a different build was the last one
  CHECK(decide(again).verdict == Verdict::INSTALL);

  // the battery: only when one is fitted and read, and only below 3.60 V
  Inputs bat = in;
  bat.hasBattery = true;
  bat.batteryMv = kMinBatteryMv - 1;
  CHECK(decide(bat).verdict == Verdict::LOW_BATTERY);
  bat.batteryMv = kMinBatteryMv;
  CHECK(decide(bat).verdict == Verdict::INSTALL);
  bat.batteryMv = 0;  // could not be read
  CHECK(decide(bat).verdict == Verdict::INSTALL);
  bat.hasBattery = false;  // "battery = none": running from USB
  bat.batteryMv = 2500;
  CHECK(decide(bat).verdict == Verdict::INSTALL);
  Inputs lowAndTried = in;  // the cheaper reasons come first
  lowAndTried.hasBattery = true;
  lowAndTried.batteryMv = 3000;
  lowAndTried.haveLast = true;
  memcpy(lowAndTried.lastElf, elfHash(appHead), kDigestBytes);
  CHECK(decide(lowAndTried).verdict == Verdict::TRIED_BEFORE);
  Inputs lowAndJunk = in;
  lowAndJunk.hasBattery = true;
  lowAndJunk.batteryMv = 3000;
  lowAndJunk.head[0] = 0;
  CHECK(decide(lowAndJunk).verdict == Verdict::BAD_FILE);

  section("firmware update: what the screen says");
  char text[96];
  for (Verdict v : {Verdict::SEVERAL, Verdict::NO_SLOT, Verdict::TRIED_BEFORE, Verdict::LOW_BATTERY}) {
    Decision d;
    d.verdict = v;
    describe(d, 3500, text, sizeof text);
    CHECK(strlen(text) > 10 && strlen(text) <= 46);
  }
  for (Problem p : {Problem::TOO_SMALL, Problem::TOO_BIG, Problem::NOT_AN_IMAGE, Problem::WRONG_CHIP, Problem::NOT_AN_APP, Problem::NO_DIGEST}) {
    Decision d;
    d.verdict = Verdict::BAD_FILE;
    d.problem = p;
    describe(d, 0, text, sizeof text);
    CHECK(strlen(text) > 10 && strlen(text) <= 46);
  }
  Decision low;
  low.verdict = Verdict::LOW_BATTERY;
  describe(low, 3504, text, sizeof text);
  CHECK(strstr(text, "3.50 V") != nullptr);
  describe(low, 3999, text, sizeof text);
  CHECK(strstr(text, "3.99 V") != nullptr);
  describe(low, 3000, text, sizeof text);
  CHECK(strstr(text, "3.00 V") != nullptr);
  Decision quiet;
  quiet.verdict = Verdict::SAME_AS_RUNNING;
  describe(quiet, 0, text, sizeof text);
  CHECK_STR(text, "");  // a leftover copy of the running build gets no message
  quiet.verdict = Verdict::NOTHING;
  describe(quiet, 0, text, sizeof text);
  CHECK_STR(text, "");
  // the merged-image hint is the one people need most
  Decision big;
  big.verdict = Verdict::BAD_FILE;
  big.problem = Problem::TOO_BIG;
  describe(big, 0, text, sizeof text);
  CHECK(strstr(text, ".merged.bin") != nullptr && strstr(text, ".ino.bin") != nullptr);
}

// The example settings file kept in docs/ for people who want to see the format before they have
// a card: it must be exactly what the clock writes to a card with none.  Regenerate it with
//   ./build/test_features --write-example
// ===========================================================================
// Firmware update: the two passes over the file (fwlogic::installImage), with a made-up file and flash
// ===========================================================================
namespace fwtest {

// A file in memory.  A read that reaches past its end fails, as it does on the card.
struct MemFile {
  std::vector<uint8_t> data;
  long failAt = -1;    // a read that covers this place in the file fails (a card error)
  int failOnRead = 0;  // ... or the read with this number does (1 = the first)
  int reads = 0;
  bool readAt(uint32_t offset, void *buf, uint32_t n) {
    reads++;
    if ((uint64_t)offset + n > data.size()) return false;
    if (failOnRead && reads == failOnRead) return false;
    if (failAt >= 0 && (uint32_t)failAt >= offset && (uint32_t)failAt < offset + n) return false;
    memcpy(buf, data.data() + offset, n);
    return true;
  }
};

// The file as the first version of the updater read it: "whatever comes next", from wherever the last read
// left off.  The place that is asked for is ignored.
struct NextBytesFile {
  std::vector<uint8_t> data;
  size_t pos = 0;
  bool readAt(uint32_t, void *buf, uint32_t n) {
    if (pos + n > data.size()) return false;
    memcpy(buf, data.data() + pos, n);
    pos += n;
    return true;
  }
};

// A stand-in for SHA-256: 32 bytes that depend on every byte and on where it stands.
struct ToyHash {
  uint64_t h[4] = {0x9E3779B97F4A7C15ull, 0xC2B2AE3D27D4EB4Full, 0x165667B19E3779F9ull, 0x27D4EB2F165667C5ull};
  uint64_t count = 0;
  void update(const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
      const uint64_t v = (uint64_t)p[i] + 1 + 257 * count++;
      for (int k = 0; k < 4; k++) h[k] = (h[k] ^ (v * (uint64_t)(2 * k + 3))) * 0x100000001B3ull + (h[(k + 1) & 3] >> 7);
    }
  }
  void finish(uint8_t *out) {
    for (int k = 0; k < 4; k++)
      for (int b = 0; b < 8; b++) out[8 * k + b] = (uint8_t)(h[k] >> (8 * b));
  }
};

// The other app slot.
struct FakeSlot {
  std::vector<uint8_t> written;
  bool began = false, ended = false, aborted = false;
  uint32_t beginSize = 0;
  int readsBeforeBegin = -1;  // how many reads the file had seen when the writing began
  const int *fileReads = nullptr;
  bool failBegin = false, failEnd = false;
  long failWriteAt = -1;  // the write that would take the slot past this many bytes fails
  bool begin(uint32_t size) {
    began = true;
    beginSize = size;
    if (fileReads) readsBeforeBegin = *fileReads;
    return !failBegin;
  }
  bool write(uint8_t *p, uint32_t n) {
    if (failWriteAt >= 0 && (long)(written.size() + n) > failWriteAt) return false;
    written.insert(written.end(), p, p + n);
    return true;
  }
  bool end() {
    ended = true;
    return !failEnd;
  }
  const char *error() { return "flash trouble"; }
  void abort() { aborted = true; }
};

// A firmware file of `bodyBytes` and the 32 bytes of checksum that follow them.
std::vector<uint8_t> makeImage(size_t bodyBytes, uint32_t seed) {
  std::vector<uint8_t> f(bodyBytes + fwlogic::kDigestBytes);
  uint32_t x = seed * 2654435761u + 1;
  for (size_t i = 0; i < bodyBytes; i++) {
    x = x * 1664525u + 1013904223u;
    f[i] = (uint8_t)(x >> 24);
  }
  ToyHash h;
  h.update(f.data(), (uint32_t)bodyBytes);
  h.finish(f.data() + bodyBytes);
  return f;
}

typedef std::vector<std::pair<int, int>> Progress;  // (pass, percent), in the order reported

template <class File>
const char *install(File &file, FakeSlot &slot, uint32_t size, uint32_t chunk, Progress *progress = nullptr) {
  static std::vector<uint8_t> buf(16384);
  ToyHash hash;
  return fwlogic::installImage(file, size, hash, slot, buf.data(), chunk, [&](fwlogic::Pass pass, int percent) {
    if (progress) progress->push_back({(int)pass, percent});
  });
}

}  // namespace fwtest

void testFirmwareInstall() {
  section("firmware update: the two passes over the file");
  using namespace fwlogic;
  using namespace fwtest;

  // a good file goes into the slot whole, checksum and all, whatever its size and the size of the reads;
  // and not a byte is written before every byte has been read and checked
  int runs = 0, wrong = 0;
  for (size_t body : {(size_t)1, (size_t)31, (size_t)32, (size_t)33, (size_t)4095, (size_t)4096, (size_t)4097, (size_t)150001}) {
    for (uint32_t chunk : {7u, 512u, 4096u, 8192u}) {
      MemFile file;
      file.data = makeImage(body, (uint32_t)(body + chunk));
      FakeSlot slot;
      slot.fileReads = &file.reads;
      const uint32_t size = (uint32_t)file.data.size();
      Progress prog;
      const char *err = install(file, slot, size, chunk, &prog);
      runs++;
      const int checkReads = (int)((body + chunk - 1) / chunk) + 1;  // the body in pieces, and the checksum
      bool ok = err == nullptr && slot.began && slot.ended && !slot.aborted && slot.beginSize == size && slot.written == file.data;
      ok = ok && slot.readsBeforeBegin == checkReads;
      // the progress: the check from 0 to 100, then the writing from 0 to 100, never backwards
      ok = ok && prog.size() >= 4 && prog.front() == std::make_pair((int)Pass::CHECK, 0) && prog.back() == std::make_pair((int)Pass::WRITE, 100);
      int lastPass = (int)Pass::CHECK, lastPct = 0, checkEnd = -1;
      for (const auto &p : prog) {
        if (p.first != lastPass) {
          if (p.first != (int)Pass::WRITE || p.second != 0) ok = false;  // (the writing starts at 0, and nothing goes back to checking)
          checkEnd = lastPct;
          lastPass = p.first;
          lastPct = 0;
        }
        if (p.second < lastPct || p.second > 100) ok = false;
        lastPct = p.second;
      }
      ok = ok && checkEnd == 100;
      if (!ok) {
        if (wrong++ < 3) printf("  body %u, reads of %u: err '%s', wrote %u of %u, %d reads before writing (want %d)\n", (unsigned)body, (unsigned)chunk,
                                err ? err : "none", (unsigned)slot.written.size(), (unsigned)size, slot.readsBeforeBegin, checkReads);
      }
    }
  }
  CHECK(runs == 32 && wrong == 0);

  // what the clock did on the day it was first tried on a real card: the caller had read the 208 bytes of the
  // header for a look, and the check then read "what comes next".  208 bytes off, it came up short at the very
  // end of every file: "Cannot read the file (card error)" at what looked like 100 %.  Every read now names
  // its place; a file that ignores the place (as that code in effect did) is seen to fail in just that way.
  {
    NextBytesFile old;
    old.data = makeImage(150001, 7);
    old.pos = kHeadBytes;
    FakeSlot slot;
    Progress prog;
    const char *err = install(old, slot, (uint32_t)old.data.size(), 4096, &prog);
    CHECK(err != nullptr && !strcmp(err, kReadError) && !slot.began && slot.written.empty());
    CHECK(!prog.empty() && prog.back().first == (int)Pass::CHECK && prog.back().second >= 96);  // the bar was nearly full
    // ... and reading "what comes next" from the start gets through the check and then has nothing left to write
    NextBytesFile fromStart;
    fromStart.data = makeImage(150001, 7);
    FakeSlot slot2;
    err = install(fromStart, slot2, (uint32_t)fromStart.data.size(), 4096);
    CHECK(err != nullptr && !strcmp(err, kReadError) && slot2.began && slot2.aborted && !slot2.ended && slot2.written.empty());
    // the same file read by place is fine
    MemFile good;
    good.data = makeImage(150001, 7);
    FakeSlot slot3;
    CHECK(install(good, slot3, (uint32_t)good.data.size(), 4096) == nullptr && slot3.written == good.data);
  }

  // a damaged copy: one bit anywhere in the file, the checksum included, and nothing is written
  for (size_t at : {(size_t)0, (size_t)208, (size_t)75000, (size_t)150000, (size_t)150001, (size_t)150032}) {
    MemFile file;
    file.data = makeImage(150001, 3);
    file.data[at] ^= 0x10;
    FakeSlot slot;
    const char *err = install(file, slot, (uint32_t)file.data.size(), 4096);
    if (!err || strcmp(err, kDamaged) != 0) printf("  a flipped bit at %u gave '%s'\n", (unsigned)at, err ? err : "none");
    CHECK(err != nullptr && !strcmp(err, kDamaged) && !slot.began && slot.written.empty());
  }

  // a copy that was cut short (the card was pulled while Windows wrote it): shorter than its size says
  for (size_t cut : {(size_t)1, (size_t)31, (size_t)32, (size_t)33, (size_t)5000}) {
    MemFile file;
    file.data = makeImage(150001, 4);
    const uint32_t size = (uint32_t)file.data.size();
    file.data.resize(file.data.size() - cut);
    FakeSlot slot;
    const char *err = install(file, slot, size, 4096);
    CHECK(err != nullptr && !strcmp(err, kReadError) && !slot.began);
  }

  // a card error while checking, anywhere: nothing is written
  for (long at : {0L, 4096L, 149999L, 150001L, 150032L}) {
    MemFile file;
    file.data = makeImage(150001, 5);
    file.failAt = at;
    FakeSlot slot;
    const char *err = install(file, slot, (uint32_t)file.data.size(), 4096);
    CHECK(err != nullptr && !strcmp(err, kReadError) && !slot.began && slot.written.empty());
  }
  // a card error while writing (the check went through): the half-written slot is given up
  {
    MemFile file;
    file.data = makeImage(150001, 6);
    const int checkReads = (150001 + 4095) / 4096 + 1;
    file.failOnRead = checkReads + 5;  // the fifth read of the second pass
    FakeSlot slot;
    const char *err = install(file, slot, (uint32_t)file.data.size(), 4096);
    CHECK(err != nullptr && !strcmp(err, kReadError) && slot.began && slot.aborted && !slot.ended);
    CHECK(slot.written.size() == 4u * 4096u);
  }
  // the flash says no: at the start (nothing to give up), in the middle, at the end
  {
    MemFile file;
    file.data = makeImage(150001, 8);
    const uint32_t size = (uint32_t)file.data.size();
    FakeSlot a;
    a.failBegin = true;
    const char *err = install(file, a, size, 4096);
    CHECK(err != nullptr && !strcmp(err, "flash trouble") && a.written.empty() && !a.aborted && !a.ended);
    FakeSlot b;
    b.failWriteAt = 20000;
    err = install(file, b, size, 4096);
    CHECK(err != nullptr && !strcmp(err, "flash trouble") && b.aborted && !b.ended && b.written.size() == 4u * 4096u);
    FakeSlot c;
    c.failEnd = true;
    err = install(file, c, size, 4096);
    CHECK(err != nullptr && !strcmp(err, "flash trouble") && c.aborted && c.written == file.data);
  }
  // no file to speak of, or reads of no size: refused before anything is read
  {
    MemFile file;
    file.data = makeImage(1, 9);
    FakeSlot slot;
    const char *small = install(file, slot, 32, 4096);
    CHECK(small != nullptr && !strcmp(small, kTooSmall) && file.reads == 0);
    const char *zero = install(file, slot, 0, 4096);
    CHECK(zero != nullptr && !strcmp(zero, kTooSmall));
    const char *noChunk = install(file, slot, (uint32_t)file.data.size(), 0);
    CHECK(noChunk != nullptr && !strcmp(noChunk, kTooSmall) && !slot.began);
  }
}

// ===========================================================================
// What is on an SD card that would not mount (sd_layout.h)
// ===========================================================================
namespace sdtest {

typedef std::vector<uint8_t> Sector;

Sector blank() { return Sector(512, 0); }

void sign(Sector &s) {
  s[510] = 0x55;
  s[511] = 0xAA;
}

// The boot sector of a volume as Windows writes it: the jump, the name of the formatter, 512 byte sectors.
Sector bootSector(const char *oem) {
  Sector s = blank();
  s[0] = 0xEB;
  s[1] = 0x58;
  s[2] = 0x90;
  memcpy(&s[3], oem, 8);
  s[11] = 0x00;
  s[12] = 0x02;  // 512 bytes per sector
  sign(s);
  return s;
}

Sector fat32() {
  Sector s = bootSector("MSDOS5.0");
  memcpy(&s[82], "FAT32   ", 8);
  return s;
}

Sector fat16() {
  Sector s = bootSector("MSDOS5.0");
  memcpy(&s[54], "FAT16   ", 8);
  return s;
}

Sector exfat() { return bootSector("EXFAT   "); }
Sector ntfs() { return bootSector("NTFS    "); }

// A partition table with boot code in front of it (as every real one has).
Sector mbr() {
  Sector s = blank();
  s[0] = 0x33;
  s[1] = 0xC0;
  s[2] = 0x8E;
  sign(s);
  return s;
}

void entry(Sector &s, int slot, uint8_t type, uint32_t lba) {
  uint8_t *e = &s[446 + 16 * slot];
  e[4] = type;
  e[8] = (uint8_t)lba;
  e[9] = (uint8_t)(lba >> 8);
  e[10] = (uint8_t)(lba >> 16);
  e[11] = (uint8_t)(lba >> 24);
  e[12] = 0x00;
  e[13] = 0x00;
  e[14] = 0x10;  // some size
}

}  // namespace sdtest

void testSdLayout() {
  section("SD card: what is on a card that will not mount");
  using namespace sdlayout;
  using namespace sdtest;

  // nothing on it
  CHECK(inspectFirst(blank().data()).kind == Kind::BLANK && !inspectFirst(blank().data()).lookAtPartition);
  {
    Sector noise(512, 0xA5);  // no boot signature
    CHECK(inspectFirst(noise.data()).kind == Kind::BLANK);
    Sector emptyTable = mbr();  // a signature and a table with nothing in it (a wiped card)
    CHECK(inspectFirst(emptyTable.data()).kind == Kind::BLANK);
  }

  // a volume that fills the card, with no partition table
  CHECK(inspectFirst(fat32().data()).kind == Kind::FAT && !inspectFirst(fat32().data()).lookAtPartition);
  CHECK(inspectFirst(fat16().data()).kind == Kind::FAT);
  CHECK(inspectFirst(exfat().data()).kind == Kind::EXFAT);
  CHECK(inspectFirst(ntfs().data()).kind == Kind::NTFS);

  // the usual card: one FAT32 partition in a classic table.  The answer is in the partition's own first sector.
  {
    Sector t = mbr();
    entry(t, 0, 0x0C, 8192);
    const First f = inspectFirst(t.data());
    CHECK(f.kind == Kind::UNKNOWN && f.lookAtPartition && f.partitionLba == 8192 && f.partitionType == 0x0C);
    CHECK(inspectPartition(fat32().data(), f.partitionType) == Kind::FAT);
    CHECK(inspectPartition(blank().data(), f.partitionType) == Kind::FAT);  // called FAT by the table, its start wiped: still "FAT, will not mount"
  }
  // a big card as it comes from the shop: one exFAT partition, type 0x07 (NTFS has the same type)
  {
    Sector t = mbr();
    entry(t, 0, 0x07, 32768);
    const First f = inspectFirst(t.data());
    CHECK(f.lookAtPartition && f.partitionLba == 32768 && f.partitionType == 0x07);
    CHECK(inspectPartition(exfat().data(), 0x07) == Kind::EXFAT);
    CHECK(inspectPartition(ntfs().data(), 0x07) == Kind::NTFS);
    CHECK(inspectPartition(blank().data(), 0x07) == Kind::OTHER);
    CHECK(inspectPartition(fat32().data(), 0x07) == Kind::FAT);  // what is there counts, not what the table calls it
  }
  // a GUID partition table: its first sector is a table with one entry of type 0xEE.  Windows shows the FAT32
  // volume inside as "FAT32" all the same, and the clock's FAT library cannot find it.
  {
    Sector t = mbr();
    entry(t, 0, 0xEE, 1);
    const First f = inspectFirst(t.data());
    CHECK(f.kind == Kind::GPT && !f.lookAtPartition && f.partitionType == 0xEE);
  }
  // the first entries empty: the first one that is used counts; a Linux partition is "other"
  {
    Sector t = mbr();
    entry(t, 2, 0x0B, 2048);
    const First f = inspectFirst(t.data());
    CHECK(f.lookAtPartition && f.partitionLba == 2048 && f.partitionType == 0x0B);
    Sector far = mbr();  // a partition that starts far into a big card: all four bytes of the sector number count
    entry(far, 3, 0x0C, 0x81020304u);
    CHECK(inspectFirst(far.data()).lookAtPartition && inspectFirst(far.data()).partitionLba == 0x81020304u);
    Sector lin = mbr();
    entry(lin, 0, 0x83, 2048);
    const First g = inspectFirst(lin.data());
    CHECK(g.lookAtPartition && inspectPartition(blank().data(), g.partitionType) == Kind::OTHER);
  }
  // an entry with a type and no start, or a start and no type, is not an entry
  {
    Sector t = mbr();
    entry(t, 0, 0x0C, 0);
    entry(t, 1, 0x00, 4096);
    CHECK(inspectFirst(t.data()).kind == Kind::BLANK);
  }
  // a FAT boot sector needs its signature, its jump, 512 byte sectors and its name: each one missing is no FAT
  {
    Sector s = fat32();
    s[510] = 0;
    CHECK(!isFat(s.data()) && inspectFirst(s.data()).kind == Kind::BLANK);
    s = fat32();
    s[0] = 0x00;
    CHECK(!isFat(s.data()));
    s = fat32();
    s[12] = 0x10;  // 4096 byte sectors
    CHECK(!isFat(s.data()));
    s = fat32();
    memcpy(&s[82], "FAT64   ", 8);
    CHECK(!isFat(s.data()));
    for (uint8_t jump : {(uint8_t)0xEB, (uint8_t)0xE9, (uint8_t)0xE8}) {
      s = fat32();
      s[0] = jump;
      CHECK(isFat(s.data()));
    }
    Sector twelve = bootSector("MSDOS5.0");
    memcpy(&twelve[54], "FAT12   ", 8);
    CHECK(isFat(twelve.data()));
    Sector e = exfat();
    e[511] = 0;  // exFAT and NTFS need the signature too
    CHECK(!isExfat(e.data()) && inspectFirst(e.data()).kind == Kind::BLANK);
  }
  // the types a table uses for FAT
  for (int t = 0; t < 256; t++) {
    const bool want = t == 0x01 || t == 0x04 || t == 0x06 || t == 0x0B || t == 0x0C || t == 0x0E;
    CHECK(isFatPartitionType((uint8_t)t) == want);
  }

  // every answer has its own line for the screen, short enough for a banner (39 characters), and says "SD card"
  std::vector<std::string> seen;
  for (Kind k : kAllKinds) {
    const std::string line = text(k);
    CHECK(line.size() >= 20 && line.size() <= 39);
    CHECK(line.compare(0, 7, "SD card") == 0);
    for (const std::string &other : seen) CHECK(other != line);
    seen.push_back(line);
    CHECK(strlen(name(k)) > 0);
  }
  CHECK(sizeof kAllKinds / sizeof kAllKinds[0] == 8);
  CHECK(strstr(text(Kind::GPT), "GPT") && strstr(text(Kind::GPT), "MBR"));
  CHECK(strstr(text(Kind::EXFAT), "exFAT") && strstr(text(Kind::EXFAT), "FAT32"));
  CHECK(strstr(text(Kind::NTFS), "NTFS") && strstr(text(Kind::BLANK), "not formatted"));
  CHECK(strstr(text(Kind::READ_ERROR), "cannot read") && strstr(text(Kind::FAT), "will not mount"));
}

void testRepoExampleFile(bool write) {
  section("example settings file in docs/");
  static char buf[16384];
  const size_t n = renderExampleConfig(Settings(), "", buf, sizeof buf);
  const char *path = "../../docs/ESP32-S3-RLCD-Config.example.txt";
  if (write) {
    FILE *f = fopen(path, "wb");
    if (!f || fwrite(buf, 1, n, f) != n) {
      printf("  cannot write %s\n", path);
      g_failed++;
    } else {
      printf("  wrote %s (%zu bytes)\n", path, n);
    }
    if (f) fclose(f);
    return;
  }
  const std::string onDisk = readFile(path);
  const bool same = onDisk == std::string(buf, n);
  if (!same) printf("  %s is out of date: run  ./build/test_features --write-example\n", path);
  CHECK(same);
}

// Every setting is described in the README, and the README says nothing about one that is gone.
void testReadmeLists(const char *path) {
  section("README lists every setting");
  const std::string readme = readFile(path);
  CHECK(!readme.empty());
  for (size_t i = 0; i < settingCount(); i++) {
    const std::string key = std::string("`") + settingKey(i) + "`";
    if (readme.find(key) == std::string::npos) printf("  README does not mention %s\n", key.c_str());
    CHECK(readme.find(key) != std::string::npos);
  }
}

// ---------------------------------------------------------------------------
// docs/power-log.csv: the sheet for the readings of battery runs.  It is filled in by hand (and by spreadsheets,
// which leave their marks), so it is read here the way it will be read later, and a line that is wrong is named.
// ---------------------------------------------------------------------------

// One line of a CSV file as its fields, with quotes the way spreadsheets write them ("a, b" and "say ""hi""").
std::vector<std::string> csvFields(const std::string &line) {
  std::vector<std::string> out(1);
  bool quoted = false;
  for (size_t i = 0; i < line.size(); i++) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        out.back() += '"';
        i++;
      } else if (c == '"') {
        quoted = false;
      } else {
        out.back() += c;
      }
    } else if (c == '"') {
      quoted = true;
    } else if (c == ',') {
      out.emplace_back();
    } else {
      out.back() += c;
    }
  }
  return out;
}

// What is wrong with a power log: the number of problems, each said if `say`.  The heading has to be made of
// plain names that the README explains, every line needs as many fields as the heading, and a column that is
// named after a setting may only hold what the settings file would take for it (or nothing).
int powerLogProblems(const std::string &csv, const std::string &readme, bool say) {
  int problems = 0;
  auto bad = [&](int line, const std::string &what) {
    problems++;
    if (say) printf("  power-log.csv line %d: %s\n", line, what.c_str());
  };
  const size_t start = csv.compare(0, 3, "\xEF\xBB\xBF") == 0 ? 3 : 0;  // (a spreadsheet's byte-order mark)
  std::vector<std::string> head;
  int lineNo = 0;
  for (size_t pos = start; pos < csv.size();) {
    size_t end = csv.find('\n', pos);
    if (end == std::string::npos) end = csv.size();
    std::string line = csv.substr(pos, end - pos);
    pos = end + 1;
    lineNo++;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (lineNo == 1) {
      head = csvFields(line);
      for (size_t i = 0; i < head.size(); i++) {
        const std::string &name = head[i];
        bool plain = !name.empty();
        for (char c : name) plain = plain && ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_');
        if (!plain) bad(1, "the column name '" + name + "' is not made of a-z, 0-9 and _");
        for (size_t k = 0; k < i; k++)
          if (head[k] == name) bad(1, "the column '" + name + "' is there twice");
        if (plain && readme.find("`" + name + "`") == std::string::npos) bad(1, "the README does not explain the column `" + name + "`");
      }
      continue;
    }
    if (line.empty()) continue;  // (a blank line, as at the end of a file saved by a spreadsheet)
    const std::vector<std::string> f = csvFields(line);
    if (f.size() != head.size()) {
      bad(lineNo, std::to_string(f.size()) + " fields where the heading has " + std::to_string(head.size()));
      continue;
    }
    for (size_t i = 0; i < head.size(); i++) {
      if (f[i].empty() || idx(head[i].c_str()) == (size_t)-1) continue;  // empty, or no setting's column
      if (apply(head[i] + " = " + f[i] + "\n").r.problems() != 0) bad(lineNo, "'" + f[i] + "' is not a value of the setting " + head[i]);
    }
  }
  if (head.empty()) bad(1, "no heading");
  return problems;
}

void testPowerLog() {
  section("docs/power-log.csv");
  const std::string csv = readFile("../../docs/power-log.csv"), readme = readFile("../../README.md");
  CHECK(!csv.empty() && !readme.empty());
  CHECK(powerLogProblems(csv, readme, true) == 0);
  // the columns that say which settings a run was made with are settings, by the names the settings file uses
  // (a setting that is renamed takes its column along), and the ones a rate is worked out from are there
  const std::vector<std::string> head = csvFields(csv.substr(0, csv.find_first_of("\r\n")));
  auto has = [&](const char *name) {
    for (const std::string &h : head)
      if (h == name) return true;
    return false;
  };
  for (const char *setting : {"wifi", "wifi_mode", "wifi_power_save", "cpu_idle_mhz", "spotify_live", "console", "light_sleep",
                              "display_power", "battery_capacity_mah"}) {
    if (!has(setting) || idx(setting) == (size_t)-1) printf("  power-log.csv: no column for the setting %s\n", setting);
    CHECK(has(setting) && idx(setting) != (size_t)-1);
  }
  for (const char *reading : {"test", "firmware", "date", "time", "battery_v", "battery_pct", "left", "pct_per_hour", "over_min", "asleep_pct"})
    CHECK(has(reading));

  // the check can fail: a made-up sheet, right and then wrong in one way at a time
  const std::string doc = "`test` `wifi_mode` `cpu_idle_mhz` `battery_pct` `notes`";
  const std::string h = "test,wifi_mode,cpu_idle_mhz,battery_pct,notes\n";
  CHECK(powerLogProblems(h + "a,sync,20,99.1,\n" + "a,always,off,,\n", doc, false) == 0);
  CHECK(powerLogProblems("\xEF\xBB\xBF" + h + "a,sync,20,99.1,\"wet, cold, \"\"quoted\"\"\"\r\n\r\n", doc, false) == 0);  // as a spreadsheet saves it
  CHECK(csvFields("a,\"b, c\",,\"say \"\"hi\"\"\",").size() == 5 && csvFields("a,\"b, c\",,\"say \"\"hi\"\"\",")[3] == "say \"hi\"");
  CHECK(powerLogProblems(h + "a,sync,20,99.1\n", doc, false) == 1);        // a field short
  CHECK(powerLogProblems(h + "a,sync,20,99.1,x,y\n", doc, false) == 1);    // one too many (a comma in a note that is not quoted)
  CHECK(powerLogProblems(h + "a,synch,20,99.1,\n", doc, false) == 1);      // not a value of wifi_mode
  CHECK(powerLogProblems(h + "a,sync,30,99.1,\n", doc, false) == 1);       // 30 MHz is not on offer
  CHECK(powerLogProblems(h + "a,sync,20,99.1,\n" + "b,sync,10,98.9,\n", doc, false) == 1);  // ... and only the line that is wrong counts
  CHECK(powerLogProblems(h + "a,sync,20,99.1,\n", "`test` `wifi_mode` `cpu_idle_mhz` `battery_pct`", false) == 1);  // a column the README does not explain
  CHECK(powerLogProblems("test,Wifi Mode,notes\na,sync,\n", doc, false) == 1);   // a name a script could not use
  CHECK(powerLogProblems("test,notes,test\na,,b\n", doc, false) == 1);           // a column twice
  CHECK(powerLogProblems("", doc, false) == 1 && powerLogProblems("\n", doc, false) >= 1);  // nothing at all
}

// ---------------------------------------------------------------------------
// The power log the clock writes by itself (power_log.h): the sheet's columns, filled in by the firmware
// ---------------------------------------------------------------------------

// The settings and the readings of the README's own examples ("Logging a battery run").
Settings sampleLogSettings() {
  return apply("wifi_mode = sync\nwifi_power_save = max\ncpu_idle_mhz = 20\nspotify_live = off\nconsole = auto\nlight_sleep = on\n"
               "display_power = low\nbattery_capacity_mah = 2500\n").s;
}

powerlog::Reading sampleReading(const Settings &s) {
  powerlog::Reading r;
  r.version = "1.7";
  r.buildId = "3f9a12c";
  r.settings = &s;
  r.spotify = SPOTIFY_PLAYING;
  r.timeValid = true;
  r.local.tm_year = 2026 - 1900;
  r.local.tm_mon = 9;
  r.local.tm_mday = 8;
  r.local.tm_hour = 14;
  r.local.tm_min = 30;
  r.local.tm_sec = 59;
  r.uptimeSec = 3 * 3600 + 12 * 60 + 47;
  r.cpuMhz = 20;
  r.clockReason = "idle";
  r.radioSync = true;
  r.radioOnPermille = 8;
  r.radioSessions = 96;
  r.asleepPermille = 785;
  r.batteryPresent = true;
  r.batteryVolts = 4.117f;
  r.charge = CHARGE_DISCHARGING;
  r.chargeReady = true;
  r.estimate.state = battest::Estimate::READY;
  r.estimate.pctPerHour = 1.2f;
  r.estimate.hoursLeft = 72.0f;
  r.estimate.windowMin = 300;
  r.estimate.avgMa = 30.0f;
  r.freeHeapBytes = 187 * 1024 + 500;
  r.framesSent = 11520;
  r.framesLate = 2;
  r.framesWorstMs = 61;
  return r;
}

std::string logLine(const powerlog::Reading &r) {
  char line[powerlog::kMaxLine];
  const size_t n = powerlog::formatLine(r, line, sizeof line);
  return std::string(line, n);
}

// The fields of a line as the clock wrote it (its line end taken off).
std::vector<std::string> logFields(const std::string &line) {
  std::string l = line;
  while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
  return csvFields(l);
}

// One field of the line a reading makes, by its column.
std::string logField(const powerlog::Reading &r, powerlog::Column c) {
  const std::vector<std::string> f = logFields(logLine(r));
  return f.size() == (size_t)powerlog::kColumnCount ? f[c] : std::string("<") + std::to_string(f.size()) + " fields>";
}

void testPowerLogLine() {
  section("power log: the heading and the line");
  using namespace powerlog;
  const std::string readme = readFile("../../README.md"), sheet = readFile("../../docs/power-log.csv");

  // the heading is the sheet's, column for column: a line pastes under the sheet's heading, one script reads both
  char head[kMaxLine];
  const size_t headLen = formatHeader(head, sizeof head);
  CHECK(headLen > 0 && headLen == strlen(head) && headLen < kMaxLine);
  const std::string sheetHead = sheet.substr(0, sheet.find_first_of("\r\n"));
  const std::vector<std::string> names = csvFields(sheetHead);
  CHECK(names.size() == (size_t)kColumnCount);
  for (size_t i = 0; i < names.size() && i < (size_t)kColumnCount; i++) {
    if (names[i] != columnName((int)i)) printf("  column %zu is '%s' in docs/power-log.csv and '%s' in power_log.h\n", i, names[i].c_str(), columnName((int)i));
    CHECK(names[i] == columnName((int)i));
  }
  CHECK(std::string(head) == sheetHead + "\r\n");
  CHECK_STR(columnName(-1), "");
  CHECK_STR(columnName(kColumnCount), "");
  {  // a buffer that is too small gets no heading rather than part of one
    char small[kMaxLine];
    CHECK(formatHeader(small, headLen) == 0 && formatHeader(small, headLen + 1) == headLen && formatHeader(small, 0) == 0);
  }

  // the README's own example of every column, as the one line the clock would write of it
  const Settings s = sampleLogSettings();
  const Reading r = sampleReading(s);
  const std::string line = logLine(r);
  CHECK_STR(line.c_str(), ",1.7 3f9a12c,on,sync,max,20,off,auto,on,low,2500,,playing,2026-10-08,14:30,0d 3h 12m,20 idle,0.8,96,78.5,4.117,87.0,"
                          "discharging,3 d 0 h,1.20,300,30.0,,187,11520,2,61,\r\n");
  CHECK(line.size() >= 140 && line.size() <= 170);  // (the README's "3,000 to 7,000 lines" rests on a line of about this length)
  for (const char *example : {"`0d 3h 12m`", "`20 idle`", "`0.8` and `96`", "`78.5`", "`4.117` and `87.0`", "`3 d 0 h`, `1.20` and `300`", "`2026-10-08`",
                              "`14:30`"}) {
    if (readme.find(example) == std::string::npos) printf("  the README no longer gives %s as an example\n", example);
    CHECK(readme.find(example) != std::string::npos);
  }
  // ... and the sheet's own checks take the clock's heading and line as they take the sheet
  CHECK(powerLogProblems(std::string(head) + line, readme, true) == 0);
  CHECK(powerLogProblems(std::string(head) + line.substr(1), readme, false) == 1);  // (they can fail: the same line, a field short)
  {  // a buffer that is too small gets no line
    char small[kMaxLine];
    CHECK(formatLine(r, small, line.size()) == 0 && formatLine(r, small, line.size() + 1) == line.size() && formatLine(r, small, 0) == 0);
    for (size_t cap = 1; cap < line.size(); cap += 7) CHECK(formatLine(r, small, cap) == 0);
  }

  // the columns of the settings hold what the settings file has, for every choice of every one of them: the
  // text is the setting's own, and the settings parser takes it back
  auto col = [](const char *name) {
    for (int c = 0; c < kColumnCount; c++)
      if (!strcmp(columnName(c), name)) return c;
    return -1;
  };
  int combos = 0;
  std::string all = head;
  for (int wifi = 0; wifi < 2; wifi++)
    for (int mode : {WIFIMODE_ALWAYS, WIFIMODE_SYNC})
      for (int save : {WIFISAVE_NORMAL, WIFISAVE_MAX})
        for (int idle : {CPUIDLE_OFF, CPUIDLE_80, CPUIDLE_40, CPUIDLE_20})
          for (int live = 0; live < 2; live++)
            for (int console : {CONSOLE_ON, CONSOLE_AUTO, CONSOLE_OFF})
              for (int sleep = 0; sleep < 2; sleep++)
                for (int disp : {DISPPOWER_NORMAL, DISPPOWER_LOW}) {
              Settings t;
              t.wifi = wifi != 0;
              t.wifiMode = (uint8_t)mode;
              t.wifiPowerSave = (uint8_t)save;
              t.cpuIdle = (uint8_t)idle;
              t.spotifyLive = live != 0;
              t.console = (uint8_t)console;
              t.lightSleep = sleep != 0;
              t.displayPower = (uint8_t)disp;
              const std::string l = logLine(sampleReading(t));
              const std::vector<std::string> f = logFields(l);
              CHECK(f.size() == (size_t)kColumnCount);
              if (f.size() != (size_t)kColumnCount) continue;
              for (const char *name : {"wifi", "wifi_mode", "wifi_power_save", "cpu_idle_mhz", "spotify_live", "console", "light_sleep",
                                       "display_power"}) {
                const std::string &text = f[(size_t)col(name)];
                CHECK(!text.empty() && text == fmt(t, idx(name)));
                const Applied back = apply(std::string(name) + " = " + text + "\n", Settings());
                if (back.r.problems() != 0 || fmt(back.s, idx(name)) != fmt(t, idx(name))) printf("  %s: '%s' does not read back\n", name, text.c_str());
                CHECK(back.r.problems() == 0 && fmt(back.s, idx(name)) == fmt(t, idx(name)));
              }
              CHECK(f[C_BATTERY_CAPACITY_MAH].empty());  // (no capacity set in these)
              CHECK(f[C_ASLEEP_PCT] == (sleep ? "78.5" : ""));  // the share asleep only with light sleep on, as the Sleep line
              all += l;
              combos++;
            }
  CHECK(combos == 768);
  CHECK(powerLogProblems(all, readme, true) == 0);
  CHECK(col("wifi") == C_WIFI && col("console") == C_CONSOLE && col("cpu_idle_mhz") == C_CPU_IDLE_MHZ && col("notes") == C_NOTES);
  CHECK(col("light_sleep") == C_LIGHT_SLEEP && col("asleep_pct") == C_ASLEEP_PCT && col("display_power") == C_DISPLAY_POWER);
  {  // settingText: by the setting's name, and nothing for a name that is no setting
    char t[16] = "x";
    settingText(s, "wifi_mode", t, sizeof t);
    CHECK_STR(t, "sync");
    settingText(s, "wifi_modes", t, sizeof t);
    CHECK_STR(t, "");
    settingText(s, "wifi_ssid", t, 0);  // no room at all: nothing is written
  }

  // what the clock does not know stays empty: a reading with nothing in it
  {
    const std::vector<std::string> f = logFields(logLine(Reading()));
    CHECK(f.size() == (size_t)kColumnCount);
    if (f.size() == (size_t)kColumnCount) {
      for (int c : {C_TEST, C_FIRMWARE, C_WIFI, C_WIFI_MODE, C_WIFI_POWER_SAVE, C_CPU_IDLE_MHZ, C_SPOTIFY_LIVE, C_CONSOLE, C_LIGHT_SLEEP,
                    C_BATTERY_CAPACITY_MAH, C_CARD, C_MUSIC, C_DATE, C_TIME, C_CLOCK_SHOWN, C_RADIO_ON_PCT, C_RADIO_SESSIONS, C_ASLEEP_PCT,
                    C_BATTERY_V, C_BATTERY_PCT, C_LEFT, C_PCT_PER_HOUR, C_OVER_MIN, C_CLOCK_MA, C_METER_MA, C_NOTES}) {
        if (!f[(size_t)c].empty()) printf("  %s is '%s' for a reading that knows nothing\n", columnName(c), f[(size_t)c].c_str());
        CHECK(f[(size_t)c].empty());
      }
      CHECK(f[C_UPTIME] == "0d 0h 0m" && f[C_BATTERY_STATE] == "none" && f[C_FREE_KB] == "0" && f[C_FRAMES_SENT] == "0" && f[C_FRAMES_LATE] == "0" &&
            f[C_FRAMES_WORST_MS] == "0");
    }
    // the four columns only a person can fill are empty in the full reading too
    for (Column c : {C_TEST, C_CARD, C_METER_MA}) CHECK(logField(r, c).empty());
    CHECK(logField(r, C_NOTES).empty());
  }

  // the date and the time only when the clock is trusted
  {
    Reading t = r;
    t.timeValid = false;
    CHECK(logField(t, C_DATE).empty() && logField(t, C_TIME).empty());
    CHECK(logField(r, C_DATE) == "2026-10-08" && logField(r, C_TIME) == "14:30");
    t = r;
    t.local.tm_mon = 0;
    t.local.tm_mday = 3;
    t.local.tm_hour = 7;
    t.local.tm_min = 5;
    CHECK(logField(t, C_DATE) == "2026-01-03" && logField(t, C_TIME) == "07:05");
  }
  // the uptime as the System info page has it
  {
    Reading t = r;
    for (const auto &c : std::vector<std::pair<uint32_t, const char *>>{{0, "0d 0h 0m"}, {59, "0d 0h 0m"}, {60, "0d 0h 1m"}, {86399, "0d 23h 59m"},
                                                                         {90061, "1d 1h 1m"}, {4294967295u, "49710d 6h 28m"}}) {
      t.uptimeSec = c.first;
      if (logField(t, C_UPTIME) != c.second) printf("  uptime %u s gives '%s'\n", (unsigned)c.first, logField(t, C_UPTIME).c_str());
      CHECK(logField(t, C_UPTIME) == c.second);
    }
  }
  // the CPU clock, with the reason when there is an idle clock to give one
  {
    Reading t = r;
    CHECK(logField(t, C_CLOCK_SHOWN) == "20 idle");
    t.cpuMhz = 80;
    t.clockReason = "USB attached";
    CHECK(logField(t, C_CLOCK_SHOWN) == "80 USB attached");
    t.clockReason = "";
    CHECK(logField(t, C_CLOCK_SHOWN) == "80");
    t.cpuMhz = 240;
    CHECK(logField(t, C_CLOCK_SHOWN) == "240");
  }
  // the radio's share only in sync mode
  {
    Reading t = r;
    t.radioSync = false;
    CHECK(logField(t, C_RADIO_ON_PCT).empty() && logField(t, C_RADIO_SESSIONS).empty());
    t.radioSync = true;
    t.radioOnPermille = 1000;
    t.radioSessions = 0;
    CHECK(logField(t, C_RADIO_ON_PCT) == "100.0" && logField(t, C_RADIO_SESSIONS) == "0");
    t.radioOnPermille = 35;
    t.radioSessions = 4294967295u;
    CHECK(logField(t, C_RADIO_ON_PCT) == "3.5" && logField(t, C_RADIO_SESSIONS) == "4294967295");
  }
  // music: what Spotify said, and nothing when no account is linked
  {
    Reading t = r;
    const std::vector<std::pair<SpotifyStatus, const char *>> cases = {{SPOTIFY_PLAYING, "playing"}, {SPOTIFY_PAUSED, "paused"}, {SPOTIFY_IDLE, "none"},
                                                                       {SPOTIFY_DISABLED, ""},      {SPOTIFY_NEEDS_LINK, ""},  {SPOTIFY_ERROR, ""}};
    for (const auto &c : cases) {
      t.spotify = c.first;
      CHECK(logField(t, C_MUSIC) == c.second);
    }
  }
  // the battery: the voltage, the level on the curve with a decimal, and the state in the words of the System info page
  {
    Reading t = r;
    CHECK(logField(t, C_BATTERY_V) == "4.117" && logField(t, C_BATTERY_PCT) == "87.0" && logField(t, C_BATTERY_STATE) == "discharging");
    t.batteryVolts = 3.3004f;
    CHECK(logField(t, C_BATTERY_V) == "3.300" && logField(t, C_BATTERY_PCT) == "0.8");
    t.batteryVolts = 4.25f;
    CHECK(logField(t, C_BATTERY_PCT) == "100.0");
    struct StateCase {
      int charge;
      bool warm, ready;
      const char *text;
    };
    const StateCase states[] = {{CHARGE_CHARGING, false, true, "charging"},   {CHARGE_FULL, false, true, "full"},        {CHARGE_DISCHARGING, false, true, "discharging"},
                                {CHARGE_UNKNOWN, true, false, "starting up"}, {CHARGE_UNKNOWN, false, false, "learning"}, {CHARGE_UNKNOWN, false, true, "on battery?"},
                                {CHARGE_CHARGING, true, false, "charging"}};
    for (const StateCase &c : states) {
      t.charge = c.charge;
      t.chargeWarmingUp = c.warm;
      t.chargeReady = c.ready;
      if (logField(t, C_BATTERY_STATE) != c.text) printf("  battery state %d/%d/%d gives '%s'\n", c.charge, c.warm, c.ready, logField(t, C_BATTERY_STATE).c_str());
      CHECK(logField(t, C_BATTERY_STATE) == c.text);
    }
    for (const char *word : {"`on battery?`", "`discharging`", "`charging`", "`full`"}) CHECK(readme.find(word) != std::string::npos);  // the README names them
    t = r;
    t.batteryPresent = false;  // no battery: nothing is measured or estimated
    CHECK(logField(t, C_BATTERY_STATE) == "none");
    for (Column c : {C_BATTERY_V, C_BATTERY_PCT, C_LEFT, C_PCT_PER_HOUR, C_OVER_MIN, C_CLOCK_MA}) CHECK(logField(t, c).empty());
  }
  // the runtime estimate: the three figures of the Left line and the one of the Current line, when there are any
  {
    Reading t = r;
    CHECK(logField(t, C_LEFT) == "3 d 0 h" && logField(t, C_PCT_PER_HOUR) == "1.20" && logField(t, C_OVER_MIN) == "300" && logField(t, C_CLOCK_MA) == "30.0");
    for (int state : {(int)battest::Estimate::OFF, (int)battest::Estimate::LEARNING}) {  // no figure yet
      t = r;
      t.estimate.state = (battest::Estimate::State)state;
      for (Column c : {C_LEFT, C_PCT_PER_HOUR, C_OVER_MIN, C_CLOCK_MA}) CHECK(logField(t, c).empty());
    }
    for (int charge : {CHARGE_CHARGING, CHARGE_FULL}) {  // a charger shows: nothing is being drained
      t = r;
      t.charge = charge;
      for (Column c : {C_LEFT, C_PCT_PER_HOUR, C_OVER_MIN, C_CLOCK_MA}) CHECK(logField(t, c).empty());
    }
    t = r;
    t.charge = CHARGE_UNKNOWN;  // "on battery?": the estimate runs whenever no charger shows
    CHECK(logField(t, C_LEFT) == "3 d 0 h" && logField(t, C_PCT_PER_HOUR) == "1.20");
    // the rate with two decimals under ten, as the page shows it, and one from there
    struct RateCase {
      float rate;
      const char *text;
    };
    for (const RateCase &c : {RateCase{0.31f, "0.31"}, RateCase{0.114f, "0.11"}, RateCase{9.99f, "9.99"}, RateCase{9.996f, "10.0"}, RateCase{12.34f, "12.3"},
                              RateCase{0.02f, "0.02"}}) {
      t = r;
      t.estimate.pctPerHour = c.rate;
      if (logField(t, C_PCT_PER_HOUR) != c.text) printf("  rate %g gives '%s'\n", (double)c.rate, logField(t, C_PCT_PER_HOUR).c_str());
      CHECK(logField(t, C_PCT_PER_HOUR) == c.text);
    }
    t = r;  // a fall too slow to measure: the rate as it is, and "more than a month"
    t.estimate.pctPerHour = 0.004f;
    t.estimate.hoursLeft = 0;
    t.estimate.unbounded = true;
    CHECK(logField(t, C_LEFT) == ">30 d" && logField(t, C_PCT_PER_HOUR) == "0.00" && logField(t, C_OVER_MIN) == "300");
    t = r;  // a rate that can be measured, and more than a month left at it
    t.estimate.pctPerHour = 0.11f;
    t.estimate.hoursLeft = 800.0f;
    t.estimate.windowMin = 480;
    CHECK(logField(t, C_LEFT) == ">30 d" && logField(t, C_PCT_PER_HOUR) == "0.11" && logField(t, C_OVER_MIN) == "480");
    t = r;
    t.estimate.hoursLeft = 5.6667f;
    CHECK(logField(t, C_LEFT) == "5 h 40 min");
    // the current: a decimal under a hundred, and only with the capacity that it is worked out from
    t = r;
    t.estimate.avgMa = 99.94f;
    CHECK(logField(t, C_CLOCK_MA) == "99.9");
    t.estimate.avgMa = 99.96f;
    CHECK(logField(t, C_CLOCK_MA) == "100");
    t.estimate.avgMa = 8.26f;
    CHECK(logField(t, C_CLOCK_MA) == "8.3");
    Settings noCapacity = s;
    noCapacity.batteryCapacityMah = 0;
    t.settings = &noCapacity;
    CHECK(logField(t, C_CLOCK_MA).empty() && logField(t, C_BATTERY_CAPACITY_MAH).empty() && logField(t, C_PCT_PER_HOUR) == "1.20");
    CHECK(logField(r, C_BATTERY_CAPACITY_MAH) == "2500");
  }
  // memory and frames
  {
    Reading t = r;
    t.freeHeapBytes = 1023;
    t.framesSent = 4294967295u;
    t.framesLate = 4294967295u;
    t.framesWorstMs = -2147483647 - 1;
    CHECK(logField(t, C_FREE_KB) == "0" && logField(t, C_FRAMES_SENT) == "4294967295" && logField(t, C_FRAMES_LATE) == "4294967295" &&
          logField(t, C_FRAMES_WORST_MS) == "-2147483648");
    CHECK(logField(r, C_FREE_KB) == "187" && logField(r, C_FRAMES_WORST_MS) == "61");
  }
  // the firmware: the version, and the build id when there is one
  {
    Reading t = r;
    t.buildId = "";
    CHECK(logField(t, C_FIRMWARE) == "1.7" && logField(r, C_FIRMWARE) == "1.7 3f9a12c");
  }

  // a note, and what CSV asks of a field with a comma or a quote in it
  {
    Reading t = r;
    t.note = "start (power on)";
    CHECK(logField(t, C_NOTES) == "start (power on)" && logLine(t).find(",start (power on)\r\n") != std::string::npos);
    t.note = "wet, cold, \"quoted\"";
    const std::string l = logLine(t);
    CHECK(l.find(",\"wet, cold, \"\"quoted\"\"\"\r\n") != std::string::npos);
    CHECK(logFields(l).size() == (size_t)kColumnCount && logField(t, C_NOTES) == "wet, cold, \"quoted\"");
    CHECK(powerLogProblems(std::string(head) + l, readme, false) == 0);
    // (it can fail: the same note without the quotes is two fields too many for the sheet's checks)
    std::string bare = l;
    bare.replace(bare.find(",\"wet"), std::string::npos, ",wet, cold, quoted\r\n");
    CHECK(powerLogProblems(std::string(head) + bare, readme, false) == 1);

    char out[32];
    CHECK(csvField("plain text", out, sizeof out) == 10 && !strcmp(out, "plain text"));
    CHECK(csvField("a,b", out, sizeof out) == 5 && !strcmp(out, "\"a,b\""));
    CHECK(csvField("say \"hi\"", out, sizeof out) == 12 && !strcmp(out, "\"say \"\"hi\"\"\""));
    CHECK(csvField("two\nlines", out, sizeof out) == 11 && !strcmp(out, "\"two\nlines\""));
    CHECK(csvField("cr\rhere", out, sizeof out) == 9 && out[0] == '"');
    CHECK(csvField("", out, sizeof out) == 0 && out[0] == 0);
    CHECK(csvField("a,b", out, 5) == 0 && csvField("a,b", out, 6) == 5 && csvField("abc", out, 3) == 0 && csvField("abc", out, 4) == 3 && csvField("abc", out, 0) == 0);
  }

  // no reading makes a line too long for the buffers the clock uses, and every one has the sheet's number of fields
  {
    Settings big = s;
    big.batteryCapacityMah = 20000;
    Reading t = sampleReading(big);
    t.version = "12.34";
    t.note = "start (reset over USB) and a note that is longer than any";
    t.uptimeSec = 4294967295u;
    t.cpuMhz = 240;
    t.clockReason = "a reason that is far too long to be one";
    t.radioOnPermille = 1000;
    t.radioSessions = 4294967295u;
    t.batteryVolts = 4.2f;
    t.estimate.pctPerHour = 99.9f;
    t.estimate.hoursLeft = 47.99f;
    t.estimate.windowMin = 480;
    t.estimate.avgMa = 19980.0f;
    t.freeHeapBytes = 4294967295u;
    t.framesSent = t.framesLate = 4294967295u;
    t.framesWorstMs = -2147483647 - 1;
    const std::string l = logLine(t);
    printf("  a line is %zu bytes for the README's example and %zu at the most (the buffers hold %zu)\n", line.size(), l.size(), kMaxLine);
    CHECK(!l.empty() && l.size() + 128 < kMaxLine && logFields(l).size() == (size_t)kColumnCount);  // (room for a few more columns)
    CHECK(powerLogProblems(std::string(head) + l, readme, true) == 0);
    CHECK(logField(t, C_CLOCK_SHOWN) == "240 a reason that is far" && logField(t, C_NOTES).size() == 31);  // long texts are cut, the line stays whole
  }
}

// Files by name, each a string.  (Not a std::map: <map> brings in std::apply, which the calls of this file's own
// apply() with a std::string would then find as well.)
struct FakeFiles {
  std::vector<std::pair<std::string, std::string>> all;

  const std::string *find(const std::string &name) const {
    for (const auto &f : all)
      if (f.first == name) return &f.second;
    return nullptr;
  }
  size_t count(const std::string &name) const { return find(name) ? 1 : 0; }
  size_t size() const { return all.size(); }
  bool empty() const { return all.empty(); }
  const std::string &at(const std::string &name) const {
    static const std::string none;
    const std::string *f = find(name);
    return f ? *f : none;
  }
  std::string &operator[](const std::string &name) {  // (makes the file if there is none, as a map would)
    for (auto &f : all)
      if (f.first == name) return f.second;
    all.emplace_back(name, std::string());
    return all.back().second;
  }
  void erase(const std::string &name) {
    for (size_t i = 0; i < all.size(); i++) {
      if (all[i].first != name) continue;
      all.erase(all.begin() + (long)i);
      return;
    }
  }
  bool operator==(const FakeFiles &other) const {  // the same files with the same contents, in any order
    if (all.size() != other.all.size()) return false;
    for (const auto &f : all) {
      const std::string *o = other.find(f.first);
      if (!o || *o != f.second) return false;
    }
    return true;
  }
};

// A file system made of strings, for the steps of power_log.h that deal with the files; it can be told to fail.
struct FakeFs {
  FakeFiles files;
  bool failRead = false, failAppend = false, failRemove = false, failRename = false;
  int failReadFrom = -1;   // reads fail from this one on, counted from 0 (-1: none)
  uint32_t appendCut = 0;  // with failAppend: this many bytes get into the file before the write fails
  int reads = 0, appends = 0, removes = 0, renames = 0;

  long size(const char *name) {
    const std::string *f = files.find(name);
    return f ? (long)f->size() : -1;
  }
  long read(const char *name, uint32_t offset, char *buf, uint32_t n) {
    const int index = reads++;
    const std::string *f = files.find(name);
    if (failRead || (failReadFrom >= 0 && index >= failReadFrom) || !f) return -1;
    if (offset >= f->size()) return 0;
    const size_t take = f->size() - offset < n ? f->size() - offset : n;
    memcpy(buf, f->data() + offset, take);
    return (long)take;
  }
  bool append(const char *name, const char *data, uint32_t n) {
    appends++;
    if (failAppend) {
      if (appendCut > 0) files[name].append(data, appendCut < n ? appendCut : n);
      return false;
    }
    files[name].append(data, n);
    return true;
  }
  bool remove(const char *name) {
    removes++;
    if (failRemove) return false;
    files.erase(name);
    return true;
  }
  bool rename(const char *from, const char *to) {
    renames++;
    if (failRename || !files.count(from) || files.count(to)) return false;  // like FAT: not onto a name that exists
    const std::string content = files.at(from);  // (a copy first: making the new name can move the old one in memory)
    files.erase(from);
    files[to] = content;
    return true;
  }
};

struct StringSink {
  std::string text;
  long failAfter = -1;  // refuses what would take it past this many bytes
  int writes = 0;
  bool write(const char *data, uint32_t n) {
    writes++;
    if (failAfter >= 0 && (long)(text.size() + n) > failAfter) return false;
    text.append(data, n);
    return true;
  }
};

// Line `i` of a made-up log: its number, then a filling whose length and letter follow from the number, so a
// line that is cut short, joined to its neighbour or out of place is seen.
std::string numberedLine(int i) {
  const int fill = 10 + (int)(((uint32_t)i * 2654435761u) >> 24) % 190;  // 10 to 199 characters
  return "n=" + std::to_string(i) + "," + std::string((size_t)fill, (char)('a' + i % 26)) + "\r\n";
}

struct LogCheck {
  int problems = 0;  // lines that are not whole, not in order or not under a heading
  int headings = 0;  // how often `header` stands in it
  int first = 0, last = 0, lines = 0;
};

// Reads an exported log of numbered lines back: the heading first, then the lines, whole and in order.
LogCheck checkLog(const std::string &text, const std::string &header) {
  LogCheck c;
  size_t pos = 0;
  int lineNo = 0;
  while (pos < text.size()) {
    const size_t end = text.find("\r\n", pos);
    if (end == std::string::npos) {  // the last line has no end
      c.problems++;
      break;
    }
    const std::string line = text.substr(pos, end + 2 - pos);
    pos = end + 2;
    lineNo++;
    if (line == header) {
      c.headings++;
      continue;
    }
    if (lineNo == 1) c.problems++;  // lines before any heading
    int n = 0;
    if (sscanf(line.c_str(), "n=%d,", &n) != 1 || line != numberedLine(n)) {
      c.problems++;
      continue;
    }
    if (c.lines > 0 && n != c.last + 1) c.problems++;
    if (c.lines == 0) c.first = n;
    c.last = n;
    c.lines++;
  }
  if (c.headings == 0) c.problems++;
  return c;
}

template <class Fs>
std::string exported(Fs &fs, const std::string &header, uint32_t cap = 4096, bool *ok = nullptr, uint32_t *bytes = nullptr) {
  StringSink sink;
  std::vector<char> buf(cap ? cap : 1);
  const powerlog::Exported e = powerlog::exportLog(fs, header.c_str(), sink, buf.data(), cap);
  if (ok) *ok = e.ok;
  if (bytes) *bytes = e.bytes;
  return sink.text;
}

void testPowerLogFiles() {
  section("power log: the files (append, the cap, reading it back)");
  using namespace powerlog;
  char head[kMaxLine];
  CHECK(formatHeader(head, sizeof head) > 0);
  const std::string H = head;

  // the reader of made-up logs can tell a good one from a bad one
  {
    const std::string good = H + numberedLine(5) + numberedLine(6) + numberedLine(7);
    const LogCheck g = checkLog(good, H);
    CHECK(g.problems == 0 && g.headings == 1 && g.first == 5 && g.last == 7 && g.lines == 3);
    CHECK(checkLog(H + numberedLine(5) + numberedLine(7), H).problems == 1);                                // a line missing
    CHECK(checkLog(H + numberedLine(6) + numberedLine(5), H).problems == 1);                                // out of order
    CHECK(checkLog(H + numberedLine(5).substr(0, 9) + "\r\n" + numberedLine(6), H).problems >= 1);          // a line cut short
    CHECK(checkLog(H + numberedLine(5).substr(0, 9) + numberedLine(6), H).problems >= 1);                   // ... and joined to the next
    CHECK(checkLog(numberedLine(5) + numberedLine(6), H).problems >= 1);                                    // no heading
    CHECK(checkLog(good.substr(0, good.size() - 2), H).problems == 1);                                      // the last line end missing
    CHECK(checkLog(H + numberedLine(5) + H + numberedLine(6), H).headings == 2);                            // the heading twice
    for (int i = 0; i < 300; i++) CHECK(numberedLine(i).size() >= 16 && numberedLine(i).size() <= 208);     // (what the sizes below count on)
  }

  // a new log: the heading, then the line; the next line is just the line
  {
    FakeFs fs;
    CHECK(logBytes(fs) == 0);
    const Appended a = appendLine(fs, head, numberedLine(1).c_str());
    CHECK(a.what == Append::OK && fs.files.size() == 1 && fs.files[kFile] == H + numberedLine(1));
    CHECK(a.logBytes == H.size() + numberedLine(1).size() && logBytes(fs) == a.logBytes);
    const Appended b = appendLine(fs, head, numberedLine(2).c_str());
    CHECK(b.what == Append::OK && fs.files[kFile] == H + numberedLine(1) + numberedLine(2) && b.logBytes == fs.files[kFile].size());
    CHECK(fs.removes == 0 && fs.renames == 0 && fs.appends == 2);
    CHECK(exported(fs, H) == H + numberedLine(1) + numberedLine(2));
    // an empty file that was left behind (made, and then the write failed) begins like a new one
    FakeFs empty;
    empty.files[kFile] = "";
    CHECK(appendLine(empty, head, numberedLine(1).c_str()).what == Append::OK && empty.files[kFile] == H + numberedLine(1));
    // nothing is written for a line or a heading that is empty or too long
    FakeFs none;
    const std::string tooLong(kMaxLine, 'x');
    CHECK(appendLine(none, head, "").what == Append::FAILED && appendLine(none, "", "x\r\n").what == Append::FAILED);
    CHECK(appendLine(none, head, tooLong.c_str()).what == Append::FAILED && appendLine(none, tooLong.c_str(), "x\r\n").what == Append::FAILED);
    CHECK(none.files.empty() && none.appends == 0);
  }

  // the cap: the two files together never hold more, the newest lines are always there, whole and in order
  // under one heading, and nothing is dropped before the log is full
  for (uint32_t cap : {6000u, 20000u, 64u * 1024u}) {
    FakeFs fs;
    int rotations = 0, firstRotationAt = 0, bad = 0;
    uint32_t smallestFull = 0xFFFFFFFFu;  // the least the log held from its first rotation on
    const int kLines = 2500;
    for (int i = 1; i <= kLines; i++) {
      const std::string oldBefore = fs.files.count(kOldFile) ? fs.files[kOldFile] : std::string();
      const Appended a = appendLine(fs, head, numberedLine(i).c_str(), cap);
      if (a.what == Append::FAILED) bad++;
      if (a.what == Append::ROTATED) {
        rotations++;
        if (!firstRotationAt) firstRotationAt = i;
      }
      if ((a.what == Append::ROTATED) != (fs.files.count(kOldFile) && fs.files[kOldFile] != oldBefore)) bad++;  // it says so when lines were dropped
      const long cur = fs.size(kFile), old = fs.size(kOldFile);
      if (cur <= 0 || (uint32_t)cur > cap / 2 || old > (long)(cap / 2)) bad++;
      if (logBytes(fs) > cap || a.logBytes != logBytes(fs)) bad++;
      if (i % 7 == 0 || a.what == Append::ROTATED || i == kLines) {  // (reading it back every time would take long)
        bool ok = false;
        uint32_t bytes = 0;
        const std::string text = exported(fs, H, 4096, &ok, &bytes);
        const LogCheck c = checkLog(text, H);
        if (!ok || bytes != text.size() || c.problems != 0 || c.headings != 1 || c.last != i) bad++;
        if (rotations <= 1 && c.first != 1) bad++;  // nothing is dropped until the old file is replaced for the first time
        if (rotations >= 1 && bytes < smallestFull) smallestFull = bytes;
        if (text.size() > cap) bad++;
      }
    }
    printf("  cap %u: %d lines written, the old half dropped %d times (first at line %d); the log never held less than %u bytes after that\n", (unsigned)cap,
           kLines, rotations, firstRotationAt, (unsigned)smallestFull);
    CHECK(bad == 0);
    CHECK(rotations >= 2 && firstRotationAt > 1);
    CHECK(smallestFull + 208 >= cap / 2 && smallestFull <= cap);  // what is kept is at least half the cap, less a line
    // (it can fail: with no cap to speak of the same lines pile up past it)
    FakeFs loose;
    for (int i = 1; i <= 200; i++) appendLine(loose, head, numberedLine(i).c_str(), 0xFFFFFFFFu);
    CHECK(logBytes(loose) > 6000 && loose.files.count(kOldFile) == 0);
  }
  CHECK(kMaxBytes == 1024 * 1024 && kMaxBytes / 2 > 100 * kMaxLine);  // the real cap: a megabyte

  // a file that another firmware began, under other columns: its lines keep their heading, in the old file,
  // and the new lines get theirs
  {
    FakeFs fs;
    const std::string otherHead = "test,firmware,a_column_that_is_gone\r\n", otherLines = "x,1.6 abc,1\r\ny,1.6 abc,2\r\n";
    fs.files[kFile] = otherHead + otherLines;
    fs.files[kOldFile] = "older still\r\n";
    const Appended a = appendLine(fs, head, numberedLine(1).c_str());
    CHECK(a.what == Append::ROTATED && fs.files[kOldFile] == otherHead + otherLines && fs.files[kFile] == H + numberedLine(1));
    CHECK(appendLine(fs, head, numberedLine(2).c_str()).what == Append::OK);
    CHECK(exported(fs, H) == otherHead + otherLines + H + numberedLine(1) + numberedLine(2));  // each lot under its own heading
    // a heading that only begins like this firmware's is another heading, shorter or longer
    for (const std::string &other : {H.substr(0, H.size() - 2) + ",one_more\r\n", H.substr(0, H.size() - 8) + "\r\n", std::string("t\r\n")}) {
      FakeFs g;
      g.files[kFile] = other + "1\r\n";
      CHECK(appendLine(g, head, numberedLine(1).c_str()).what == Append::ROTATED && g.files[kOldFile] == other + "1\r\n" && g.files[kFile] == H + numberedLine(1));
    }
    // the same heading is no reason to start a new file (the control of the case above)
    FakeFs same;
    same.files[kFile] = H + numberedLine(1);
    CHECK(appendLine(same, head, numberedLine(2).c_str()).what == Append::OK && same.files.count(kOldFile) == 0);
    // a file that holds the heading and nothing else yet
    FakeFs bare;
    bare.files[kFile] = H;
    CHECK(appendLine(bare, head, numberedLine(1).c_str()).what == Append::OK && bare.files[kFile] == H + numberedLine(1));
  }

  // a write that was cut short left part of a line: the next line starts on a line of its own
  {
    FakeFs fs;
    fs.files[kFile] = H + numberedLine(1) + "n=2,bbb";
    CHECK(appendLine(fs, head, numberedLine(3).c_str()).what == Append::OK);
    CHECK(fs.files[kFile] == H + numberedLine(1) + "n=2,bbb\r\n" + numberedLine(3));
    const LogCheck c = checkLog(exported(fs, H), H);
    CHECK(c.problems == 2 && c.last == 3 && c.lines == 2);  // the half line is lost (and so is its number), its neighbours are whole
    // (the control: without that line end the two would be one line, and line 3 lost with it)
    CHECK(checkLog(H + numberedLine(1) + "n=2,bbb" + numberedLine(3), H).lines == 1);
  }

  // things that fail: nothing is half done, and the next line goes in as if nothing had happened
  {
    FakeFs fs;
    for (int i = 1; i <= 3; i++) appendLine(fs, head, numberedLine(i).c_str());
    const FakeFiles before = fs.files;
    fs.failRead = true;  // the file cannot be read: nothing is touched
    CHECK(appendLine(fs, head, numberedLine(4).c_str()).what == Append::FAILED && fs.files == before && fs.appends == 3);
    fs.failRead = false;
    fs.reads = 0;
    fs.failReadFrom = 1;  // ... or its heading can and its last byte cannot: the same
    CHECK(appendLine(fs, head, numberedLine(4).c_str()).what == Append::FAILED && fs.files == before && fs.appends == 3 && fs.reads == 2);
    fs.failReadFrom = -1;
    fs.failAppend = true;  // it cannot be written
    CHECK(appendLine(fs, head, numberedLine(4).c_str()).what == Append::FAILED && fs.files == before);
    fs.appendCut = 9;  // ... or only the start of the line got in
    CHECK(appendLine(fs, head, numberedLine(4).c_str()).what == Append::FAILED && fs.files[kFile].size() == before.at(kFile).size() + 9);
    fs.failAppend = false;
    CHECK(appendLine(fs, head, numberedLine(5).c_str()).what == Append::OK);
    const LogCheck c = checkLog(exported(fs, H), H);
    CHECK(c.first == 1 && c.last == 5 && c.lines == 4 && c.problems == 2);  // lines 1 to 3 and 5 are whole; the half of line 4 is all that is wrong

    // when the log is full and the old file cannot be deleted or the new one renamed: no line, and nothing lost
    // that was not going to be dropped anyway
    FakeFs stuck;
    int n = 0;
    // (both loops end by a count as well: a cap that does not work must fail this test, not keep it running)
    while (n < 1000 && stuck.files.count(kOldFile) == 0) appendLine(stuck, head, numberedLine(++n).c_str(), 6000);
    while (n < 1000 && stuck.files[kFile].size() + numberedLine(n + 1).size() <= 3000) appendLine(stuck, head, numberedLine(++n).c_str(), 6000);
    CHECK(n < 1000 && stuck.files.count(kOldFile) == 1);
    const FakeFiles atFull = stuck.files;  // (the next line is the one that makes room first)
    stuck.failRemove = true;
    CHECK(appendLine(stuck, head, numberedLine(n + 1).c_str(), 6000).what == Append::FAILED && stuck.files == atFull);
    stuck.failRemove = false;
    stuck.failRename = true;
    CHECK(appendLine(stuck, head, numberedLine(n + 1).c_str(), 6000).what == Append::FAILED);
    CHECK(stuck.files.count(kOldFile) == 0 && stuck.files[kFile] == atFull.at(kFile));  // the newest lines are all there
    stuck.failRename = false;
    CHECK(appendLine(stuck, head, numberedLine(n + 1).c_str(), 6000).what == Append::ROTATED);
    const LogCheck after = checkLog(exported(stuck, H), H);
    CHECK(after.problems == 0 && after.last == n + 1 && after.headings == 1);
  }

  // reading the log back: one heading at the top, whatever the size of the pieces it is read in
  {
    FakeFs fs;
    for (int i = 1; i <= 120; i++) appendLine(fs, head, numberedLine(i).c_str(), 12000);
    CHECK(fs.files.count(kOldFile) == 1);
    bool ok = false;
    uint32_t bytes = 0;
    const std::string whole = exported(fs, H, 4096, &ok, &bytes);
    CHECK(ok && bytes == whole.size() && checkLog(whole, H).problems == 0 && checkLog(whole, H).headings == 1);
    CHECK(whole == fs.files[kOldFile] + fs.files[kFile].substr(H.size()));  // the old file, then the new one without its heading
    for (uint32_t cap : {(uint32_t)kMaxLine, (uint32_t)kMaxLine + 1, 1000u, 100000u}) {
      bool okCap = false;
      CHECK(exported(fs, H, cap, &okCap) == whole && okCap);
    }
    for (uint32_t cap : {0u, 1u, (uint32_t)kMaxLine - 1}) {  // a piece that could not hold a heading: nothing is read
      bool okCap = true;
      CHECK(exported(fs, H, cap, &okCap).empty() && !okCap);
    }
    // the sink gives up (a card that is full, a serial port nobody reads): it stops there and says so
    for (long limit : {0L, 100L, (long)H.size(), (long)whole.size() - 1}) {
      StringSink sink;
      sink.failAfter = limit;
      std::vector<char> buf(4096);
      const Exported e = exportLog(fs, head, sink, buf.data(), 4096);
      CHECK(!e.ok && e.bytes == sink.text.size() && (long)sink.text.size() <= limit && whole.compare(0, sink.text.size(), sink.text) == 0);
    }
    {
      StringSink sink;
      sink.failAfter = (long)whole.size();  // (the control: room for exactly all of it)
      std::vector<char> buf(4096);
      CHECK(exportLog(fs, head, sink, buf.data(), 4096).ok && sink.text == whole);
    }
    fs.failRead = true;  // the log cannot be read
    bool okRead = true;
    exported(fs, H, 4096, &okRead);
    CHECK(!okRead);
    fs.failRead = false;
    // ... or only so far.  Wherever the reading stops, what went out is the start of the log and it is not
    // passed off as the whole of it (a copy on the card that is short and says nothing would be the worst kind).
    fs.reads = 0;
    CHECK(exported(fs, H, 1000, &okRead) == whole && okRead);
    const int readsOfAll = fs.reads;
    CHECK(readsOfAll >= 8);  // (both files' first lines, and several pieces of each)
    for (int from = 0; from <= readsOfAll; from++) {
      fs.reads = 0;
      fs.failReadFrom = from;
      bool okPart = true;
      const std::string part = exported(fs, H, 1000, &okPart);
      if (from < readsOfAll) {
        if (okPart || part.size() >= whole.size() || whole.compare(0, part.size(), part) != 0) printf("  reads failing from number %d: not noticed\n", from);
        CHECK(!okPart && part.size() < whole.size() && whole.compare(0, part.size(), part) == 0);
      } else {
        CHECK(okPart && part == whole);  // (the control: every read it needs works)
      }
    }
    fs.failReadFrom = -1;

    // an empty log is the heading alone
    FakeFs none;
    CHECK(exported(none, H, 4096, &ok, &bytes) == H && ok && bytes == H.size());
    none.files[kFile] = "";
    none.files[kOldFile] = "";
    CHECK(exported(none, H) == H);
    // only the old file is there (the new one could not be begun after the old half was put aside)
    FakeFs oldOnly;
    oldOnly.files[kOldFile] = H + numberedLine(1) + numberedLine(2);
    CHECK(exported(oldOnly, H) == H + numberedLine(1) + numberedLine(2));
    CHECK(appendLine(oldOnly, head, numberedLine(3).c_str()).what == Append::OK && checkLog(exported(oldOnly, H), H).problems == 0 &&
          checkLog(exported(oldOnly, H), H).lines == 3);
    // the old file ends in the middle of a line: the new file's lines start on a line of their own
    FakeFs torn;
    torn.files[kOldFile] = H + numberedLine(1) + "n=2,bb";
    torn.files[kFile] = H + numberedLine(3);
    CHECK(exported(torn, H) == H + numberedLine(1) + "n=2,bb\r\n" + numberedLine(3));
    // a file that does not begin with a line at all is put out as it is, and the next file's heading with it
    FakeFs junk;
    junk.files[kOldFile] = std::string(2 * kMaxLine, '#');
    junk.files[kFile] = H + numberedLine(1);
    CHECK(exported(junk, H) == std::string(2 * kMaxLine, '#') + "\r\n" + H + numberedLine(1));
  }

  // forgetting the log
  {
    FakeFs fs;
    for (int i = 1; i <= 120; i++) appendLine(fs, head, numberedLine(i).c_str(), 12000);
    CHECK(fs.files.size() == 2 && clearLog(fs) && fs.files.empty() && logBytes(fs) == 0);
    CHECK(clearLog(fs));  // nothing to forget is fine too
    CHECK(appendLine(fs, head, numberedLine(1).c_str()).what == Append::OK && exported(fs, H) == H + numberedLine(1));
    fs.failRemove = true;
    CHECK(!clearLog(fs) && fs.files.size() == 1);
  }
}

// A file of a folder as a string ("" if there is none), without the complaint readFile() makes.
std::string folderFile(const char *folder, const char *name) {
  const std::string path = std::string(folder) + "/" + name;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return "";
  std::string text;
  char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
  fclose(f);
  return text;
}

// The same steps on the files of a real folder, through the very calls the clock makes on its flash partition
// (FolderFs).  The made-up file system above is the reference: asked the same things, the two end up with the
// same files.  (A folder of this PC is not the FAT file system of the clock; what is run here is the calls: the
// flags, the places in the file, what counts as an error.)
void testPowerLogFolder() {
  section("power log: the files of a real folder");
  using namespace powerlog;
  char head[kMaxLine];
  CHECK(formatHeader(head, sizeof head) > 0);
  const std::string H = head;
  // (in memory where the PC has /dev/shm: every line is followed by an fsync, which takes tens of milliseconds on a disk)
  char inMemory[] = "/dev/shm/plog_test_XXXXXX", onDisk[] = "/tmp/plog_test_XXXXXX";
  const char *dir = mkdtemp(inMemory);
  if (!dir) dir = mkdtemp(onDisk);
  if (!dir) {
    printf("  cannot make a folder in /dev/shm or /tmp\n");
    g_failed++;
    return;
  }
  FolderFs real(dir);
  FakeFs fake;

  // an empty folder: no file, nothing to read, nothing to rename; deleting what is not there is no error
  char piece[64];
  CHECK(real.size(kFile) == -1 && real.size(kOldFile) == -1 && logBytes(real) == 0);
  CHECK(real.read(kFile, 0, piece, sizeof piece) == -1);
  CHECK(real.remove(kFile) && real.remove(kOldFile));
  CHECK(!real.rename(kFile, kOldFile));
  CHECK(exported(real, H) == H);

  // 400 lines with a small cap, to both
  int differ = 0;
  for (int i = 1; i <= 400; i++) {
    const std::string l = numberedLine(i);
    const Appended a = appendLine(real, head, l.c_str(), 20000), f = appendLine(fake, head, l.c_str(), 20000);
    if (a.what != f.what || a.what == Append::FAILED || a.logBytes != f.logBytes || real.size(kFile) != fake.size(kFile) ||
        real.size(kOldFile) != fake.size(kOldFile)) {
      if (differ++ < 3) printf("  line %d: the folder and the made-up files differ (%d/%d, %u/%u bytes)\n", i, (int)a.what, (int)f.what, (unsigned)a.logBytes, (unsigned)f.logBytes);
    }
  }
  CHECK(differ == 0);
  CHECK(fake.files.count(kOldFile) == 1 && fake.renames >= 2);  // (the old half was dropped more than once on the way)
  CHECK(folderFile(dir, kFile) == fake.files.at(kFile) && folderFile(dir, kOldFile) == fake.files.at(kOldFile));  // byte for byte
  for (uint32_t cap : {(uint32_t)kMaxLine, 4096u}) {
    bool okReal = false, okFake = false;
    uint32_t bytes = 0;
    const std::string text = exported(real, H, cap, &okReal, &bytes);
    CHECK(okReal && bytes == text.size() && text == exported(fake, H, cap, &okFake) && okFake);
    const LogCheck c = checkLog(text, H);
    CHECK(c.problems == 0 && c.headings == 1 && c.last == 400);
  }

  // reading: a piece from the middle, the last bytes (fewer than asked for), and past the end (nothing, which is no error)
  const std::string cur = fake.files.at(kFile);
  CHECK(cur.size() > 100);
  CHECK(real.read(kFile, 10, piece, 20) == 20 && memcmp(piece, cur.data() + 10, 20) == 0);
  CHECK(real.read(kFile, (uint32_t)cur.size() - 5, piece, 20) == 5 && memcmp(piece, cur.data() + cur.size() - 5, 5) == 0);
  CHECK(real.read(kFile, (uint32_t)cur.size(), piece, 20) == 0 && real.read(kFile, (uint32_t)cur.size() + 1000, piece, 20) == 0);
  CHECK(real.read(kFile, 0, piece, 0) == 0);

  // a file that ends in the middle of a line (cut short here by hand) is mended the same way
  {
    const std::string path = std::string(dir) + "/" + kFile;
    CHECK(truncate(path.c_str(), (off_t)cur.size() - 7) == 0);
    fake.files[kFile].resize(cur.size() - 7);
    CHECK(appendLine(real, head, numberedLine(401).c_str(), 20000).what == appendLine(fake, head, numberedLine(401).c_str(), 20000).what);
    const std::string mended = folderFile(dir, kFile), tail = "\r\n" + numberedLine(401);
    CHECK(mended == fake.files.at(kFile));
    CHECK(mended.size() > tail.size() && mended.compare(mended.size() - tail.size(), tail.size(), tail) == 0);
  }

  // another firmware's file: put aside under its own heading
  {
    CHECK(clearLog(real) && real.size(kFile) == -1 && real.size(kOldFile) == -1 && folderFile(dir, kFile).empty());
    const std::string other = "some,other,columns\r\n1,2,3\r\n";
    CHECK(real.append(kFile, other.data(), (uint32_t)other.size()) && folderFile(dir, kFile) == other);
    CHECK(appendLine(real, head, numberedLine(1).c_str()).what == Append::ROTATED);
    CHECK(folderFile(dir, kOldFile) == other && folderFile(dir, kFile) == H + numberedLine(1));
    CHECK(exported(real, H) == other + H + numberedLine(1));
  }

  // a name that is taken by something that is no file (here a folder): it has no size, and cannot be read, added to
  // or deleted; the log says so instead of pretending
  {
    CHECK(clearLog(real));
    const std::string blocker = std::string(dir) + "/" + kOldFile;
    CHECK(mkdir(blocker.c_str(), 0777) == 0);
    CHECK(real.size(kOldFile) == -1 && !real.remove(kOldFile) && real.read(kOldFile, 0, piece, 8) == -1 && !real.append(kOldFile, "x", 1));
    CHECK(!clearLog(real));
    // lines go in until the file is full; then the old half cannot be dropped and every further line is refused
    int written = 0, refused = 0;
    for (int i = 1; i <= 60; i++) (appendLine(real, head, numberedLine(written + 1).c_str(), 6000).what == Append::FAILED ? refused : written)++;
    CHECK(written > 10 && refused > 10 && written + refused == 60);
    const LogCheck c = checkLog(exported(real, H), H);
    CHECK(c.problems == 0 && c.first == 1 && c.last == written && real.size(kFile) <= 3000);
    CHECK(rmdir(blocker.c_str()) == 0);
    CHECK(appendLine(real, head, numberedLine(written + 1).c_str(), 6000).what == Append::ROTATED);  // the way is clear: it carries on
    CHECK(checkLog(exported(real, H), H).problems == 0 && checkLog(exported(real, H), H).last == written + 1);
  }

  // a path too long to be a path is an error, not another file's name; and a folder that is not there takes nothing
  {
    const std::string longDir = "/tmp/" + std::string(200, 'x');
    FolderFs tooLong(longDir.c_str());
    CHECK(tooLong.size(kFile) == -1 && tooLong.read(kFile, 0, piece, 8) == -1 && !tooLong.append(kFile, "x", 1) && !tooLong.remove(kFile) &&
          !tooLong.rename(kFile, kOldFile));
    FolderFs gone("/tmp/plog_test_no_such_folder");
    CHECK(gone.size(kFile) == -1 && !gone.append(kFile, "x", 1) && gone.read(kFile, 0, piece, 8) == -1);
    CHECK(appendLine(gone, head, numberedLine(1).c_str()).what == Append::FAILED);
    bool ok = false;
    CHECK(exported(gone, H, 4096, &ok) == H && ok);  // (no files is an empty log)
  }

  CHECK(clearLog(real) && real.size(kFile) == -1 && real.size(kOldFile) == -1);
  CHECK(rmdir(dir) == 0);  // nothing else was left in it
}

void testPowerLogSchedule() {
  section("power log: when a line is due");
  using namespace powerlog;
  CHECK(kIntervalSec == 600 && kFirstSec > 0 && kFirstSec < 60 && kLatestStartUs > 0 && kLatestStartUs < 300000);
  {
    Schedule s;
    CHECK(s.startLine() && !s.due(0) && !s.due(kFirstSec - 1) && s.due(kFirstSec) && s.due(100000));
    s.done(kFirstSec, true);
    CHECK(!s.startLine() && s.lines() == 1 && s.lastSec() == kFirstSec);
    CHECK(!s.due(kFirstSec) && !s.due(kFirstSec + kIntervalSec - 1) && s.due(kFirstSec + kIntervalSec));
  }

  // a day in which the loop gets round to it a little late every time: a line about every ten minutes, and
  // never two closer together than that
  {
    Schedule s;
    uint32_t rng = 4711, lastWrite = 0, minGap = 0xFFFFFFFFu, maxGap = 0;
    int lines = 0;
    for (uint32_t t = 0; t < 86400; t++) {
      rng = rng * 1664525u + 1013904223u;
      if (((rng >> 16) % 5) != 0) continue;  // the loop is busy with a frame four seconds in five
      if (!s.due(t)) continue;
      if (lines > 0) {
        const uint32_t gap = t - lastWrite;
        if (gap < minGap) minGap = gap;
        if (gap > maxGap) maxGap = gap;
      }
      lastWrite = t;
      lines++;
      s.done(t, true);
    }
    printf("  a simulated day: %d lines, %u to %u s apart\n", lines, (unsigned)minGap, (unsigned)maxGap);
    CHECK(lines >= 140 && lines <= 144 && (uint32_t)lines == s.lines());
    CHECK(minGap >= kIntervalSec && maxGap < kIntervalSec + 60);
  }

  // a write that fails is not tried again before the next line is due, and after three in a row no more at all
  {
    Schedule s;
    s.done(10, true);
    s.done(610, false);
    CHECK(s.failures() == 1 && !s.stopped() && s.lines() == 1 && !s.due(611) && !s.due(1209) && s.due(1210));
    s.done(1210, false);
    CHECK(s.failures() == 2 && !s.stopped());
    s.done(1810, true);  // one that works: the count starts again
    CHECK(s.failures() == 0 && s.lines() == 2);
    for (int i = 0; i < kMaxFailures - 1; i++) s.done(2410 + (uint32_t)i * 600, false);
    CHECK(!s.stopped() && s.due(100000));
    s.done(4000, false);
    CHECK(s.stopped() && s.failures() == kMaxFailures && !s.due(4000 + kIntervalSec) && !s.due(4000000000u));
    Schedule first;  // the line of the start fails: the next is a whole interval later, and is no longer "the start"
    first.done(kFirstSec, false);
    CHECK(!first.startLine() && !first.due(kFirstSec + kIntervalSec - 1) && first.due(kFirstSec + kIntervalSec) && first.lines() == 0);
    Schedule off;  // nothing to write to
    off.stop();
    CHECK(off.stopped() && !off.due(kFirstSec) && !off.due(1000000));
  }

  // seconds that run over (136 years on): the interval is a difference, so it still holds
  {
    Schedule s;
    s.done(4294967000u, true);
    CHECK(!s.due(4294967295u) && !s.due(4294967000u + kIntervalSec - 1) && s.due(4294967000u + kIntervalSec));
    CHECK((uint32_t)(4294967000u + kIntervalSec) < 1000);  // (that moment is past the wrap)
  }
}

void testPowerLogStatus() {
  section("power log: the rows of the Power and settings page");
  using namespace powerlog;
  const size_t kShown = 42;  // an info page shows this much of a value
  CHECK(kStatusRow == kShown + 1);
  // A row is whole when it ends the way its text ends: a text cut off at the edge of the page does not.
  auto whole = [](const char *row) {
    for (const char *end : {" ago", " KB)", "in a moment", "FAILED", "write error", "read the log", "README)", "will not mount", "lines)"}) {
      const size_t n = strlen(end), l = strlen(row);
      if (l >= n && !strcmp(row + l - n, end)) return true;
    }
    return false;
  };
  int rowsSeen = 0, bad = 0;
  size_t longest = 0;
  char longestText[kStatusRow] = "";
  for (int on = 0; on < 2; on++)
    for (int problem : {(int)Status::FINE, (int)Status::NO_PARTITION, (int)Status::CANNOT_MOUNT})
      for (int stopped = 0; stopped < 2; stopped++)
        for (int failures : {0, 1, kMaxFailures})
          for (uint32_t lines : {0u, 1u, 2u, 99999u})
            for (uint32_t bytes : {0u, 1u, 1024u, 1025u, kMaxBytes})
              for (uint32_t since : {0u, 89u, 90u, 5399u, 5400u, 36000u})
                for (int copy : {(int)Status::COPY_NONE, (int)Status::COPY_DONE, (int)Status::COPY_CANNOT_READ, (int)Status::COPY_CARD_ERROR}) {
                  Status s;
                  s.on = on != 0;
                  s.problem = (Status::Problem)problem;
                  s.stopped = stopped != 0;
                  s.failures = failures;
                  s.lines = lines;
                  s.logBytes = bytes;
                  s.sinceLineSec = since;
                  s.copy = (Status::Copy)copy;
                  s.copyBytes = bytes;
                  char rows[2][kStatusRow] = {"", ""};
                  const int n = statusRows(s, rows);
                  if (n != on + (copy != Status::COPY_NONE ? 1 : 0)) bad++;
                  for (int i = 0; i < n; i++) {
                    rowsSeen++;
                    const size_t len = strlen(rows[i]);
                    if (len == 0 || len > kShown || !whole(rows[i])) {
                      if (bad++ < 5) printf("  a row that does not fit or is cut: '%s'\n", rows[i]);
                    }
                    if (len > longest) {
                      longest = len;
                      snprintf(longestText, sizeof longestText, "%.*s", (int)(sizeof longestText - 1), rows[i]);  // (bounded: the compiler takes the two rows for one string)
                    }
                  }
                  if (!on && n == 1 && strncmp(rows[0], "off, ", 5) != 0) bad++;  // the log is off, and the row about its copy says so
                  if (on && n == 2 && !strncmp(rows[1], "off", 3)) bad++;
                }
  printf("  %d rows, the longest takes %zu of %zu characters: '%s'\n", rowsSeen, longest, kShown, longestText);
  CHECK(bad == 0 && rowsSeen > 1000 && longest <= kShown);

  // what they say
  char rows[2][kStatusRow];
  Status s;
  CHECK(statusRows(s, rows) == 0);  // off, and no copy: nothing on the page
  s.on = true;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "on: first line in a moment"));
  s.lines = 1;
  s.logBytes = 460;
  s.sinceLineSec = 45;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "1 KB, 1 line this run, 45 s ago"));
  s.lines = 37;
  s.logBytes = 412 * 1024;
  s.sinceLineSec = 240;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 4 min ago"));
  s.sinceLineSec = 89;  // seconds up to a minute and a half, then minutes to the nearest, then hours
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 89 s ago"));
  s.sinceLineSec = 90;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 2 min ago"));
  s.sinceLineSec = 209;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 3 min ago"));
  s.sinceLineSec = 210;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 4 min ago"));
  s.sinceLineSec = 5399;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 90 min ago"));
  s.sinceLineSec = 7200;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "412 KB, 37 lines this run, 2 h ago"));
  s.failures = 1;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "37 lines this run, last write FAILED"));
  s.failures = kMaxFailures;
  s.stopped = true;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "STOPPED after 3 write errors (37 lines)"));
  s.problem = Status::NO_PARTITION;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "OFF: no ffat partition (see the README)"));
  s.problem = Status::CANNOT_MOUNT;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "OFF: the flash partition will not mount"));
  s = Status();
  s.on = true;
  s.lines = 2;
  s.logBytes = 2048;
  s.copy = Status::COPY_DONE;
  s.copyBytes = 1500;
  CHECK(statusRows(s, rows) == 2 && !strcmp(rows[0], "2 KB, 2 lines this run, 0 s ago") && !strcmp(rows[1], "copied to SD card at start (2 KB)"));
  s.copy = Status::COPY_CARD_ERROR;
  CHECK(statusRows(s, rows) == 2 && !strcmp(rows[1], "NOT copied to SD: card write error"));
  s.on = false;
  s.copy = Status::COPY_CANNOT_READ;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "off, NOT copied to SD: cannot read the log"));
  s.copy = Status::COPY_DONE;
  s.copyBytes = kMaxBytes;
  CHECK(statusRows(s, rows) == 1 && !strcmp(rows[0], "off, copied to SD card at start (1024 KB)"));

  // numbers that no run reaches are kept to the digits there is room for, so the row is whole all the same
  Status absurd;
  absurd.on = true;
  absurd.stopped = true;
  absurd.failures = 2000000000;
  absurd.lines = 4000000000u;
  CHECK(statusRows(absurd, rows) == 1 && !strcmp(rows[0], "STOPPED after 9 write errors (99999 lines)") && strlen(rows[0]) == kShown);
  absurd.stopped = false;
  absurd.failures = 0;
  absurd.logBytes = 4000000000u;
  absurd.sinceLineSec = 4000000000u;
  absurd.copy = Status::COPY_DONE;
  absurd.copyBytes = 4000000000u;
  CHECK(statusRows(absurd, rows) == 2 && !strcmp(rows[0], "9999 KB, 99999 lines this run, 999 h ago") && !strcmp(rows[1], "copied to SD card at start (9999 KB)"));
  absurd.failures = -5;  // (no such count: it reads as none)
  absurd.stopped = true;
  CHECK(statusRows(absurd, rows) == 2 && !strcmp(rows[0], "STOPPED after 0 write errors (99999 lines)"));
  // (the check for a row that is cut can fail: the same text with its end off the page)
  CHECK(whole("412 KB, 37 lines this run, 4 min ago") && !whole("412 KB, 37 lines this run, 4 min a"));
}

}  // namespace

int main(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "--write-example")) {
    testRepoExampleFile(true);
    return g_failed ? 1 : 0;
  }
  testRepoExampleFile(false);
  testReadmeLists("../../README.md");
  testPowerLog();
  testPowerLogLine();
  testPowerLogFiles();
  testPowerLogFolder();
  testPowerLogSchedule();
  testPowerLogStatus();
  testMoonPhase();
  testMoonGlyph();
  testDateFormats();
  testBatteryCurve();
  testBatteryEstimate();
  testBatteryEstimateReplay();
  testShutdownRestsDisplay();
  testLowBattery();
  testDrift();
  testSettingsTable();
  testSettingsRoundTrip();
  testSettingsSyntax();
  testSettingsValues();
  testSettingsRejects();
  testSettingsTimezone();
  testSettingsReport();
  testSdBanner();
  testSettingsForget();
  testSettingsExample();
  testBackupWifiSettings();
  testPowerSettings();
  testBatteryCalibrationSetting();
  testWifiPick();
  testWifiPickDay();
  testFirmwareLogic();
  testFirmwareInstall();
  testSdLayout();
  testSettingsFuzz();

  printf("\n%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
