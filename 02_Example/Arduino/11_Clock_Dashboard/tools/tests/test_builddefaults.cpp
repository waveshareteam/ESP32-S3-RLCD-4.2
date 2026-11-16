// Host-side test of build_defaults.h: the defaults in config.h, as a secrets.h can replace them, become
// a Settings that follows the same rules as the SD card file, and every setting the card can set has
// one.  Built four times by run_tests.sh, with CONFIG_NO_SECRETS (a secrets.h lying around must not
// get in) and a force-included header that plays the part of secrets.h:
//
//   (nothing)       the factory build, no secrets.h at all
//   -DTEST_ALL      override_all.h: every macro replaced, with the values of all_settings.h
//   -DTEST_BAD      override_bad.h: values the card would refuse as well
//   -DTEST_EDGE     override_edge.h: values on the limits, and the placeholders of secrets.example.h

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "all_settings.h"
#include "build_defaults.h"
#include "check.h"

namespace {

[[maybe_unused]] std::string fmt(const Settings &s, size_t i) {
  char buf[128];
  return formatSetting(s, i, buf, sizeof buf) ? buf : "<overflow>";
}

[[maybe_unused]] std::string dump(const Settings &s) {
  std::string out;
  for (size_t i = 0; i < settingCount(); i++) out += std::string(settingKey(i)) + "=" + fmt(s, i) + "\n";
  return out;
}

// Says which settings differ, so a failure names them.
[[maybe_unused]] bool sameSettings(const Settings &a, const Settings &b, const char *what) {
  bool same = true;
  for (size_t i = 0; i < settingCount(); i++) {
    if (fmt(a, i) == fmt(b, i)) continue;
    printf("  %s: %s is '%s' here but '%s' there\n", what, settingKey(i), fmt(a, i).c_str(), fmt(b, i).c_str());
    same = false;
  }
  return same;
}

// `name` as a whole word of `text` (WIFI_SSID is not found in WIFI_SSID_BACKUP).
[[maybe_unused]] bool hasWord(const std::string &text, const std::string &name) {
  auto ident = [](char c) { return isalnum((unsigned char)c) || c == '_'; };
  for (size_t at = text.find(name); at != std::string::npos; at = text.find(name, at + 1)) {
    const bool startsOk = at == 0 || !ident(text[at - 1]);
    const size_t end = at + name.size();
    const bool endsOk = end >= text.size() || !ident(text[end]);
    if (startsOk && endsOk) return true;
  }
  return false;
}

#if defined(TEST_ALL)

// A secrets.h that sets everything gives the very settings that the card file of all_settings.h
// gives: the two ways in agree on what each setting means.
void testEverythingReplaced() {
  section("build defaults: every macro replaced");
  Settings card;
  const std::string text(kAllSettings);
  const ConfigReport r = applyConfigText(card, Settings(), text.data(), text.size());
  CHECK(r.problems() == 0);
  CHECK(r.changed == (int)settingCount());  // what the rest of this test stands on: the file sets every setting to something new

  BuildReport rep;
  const Settings b = buildDefaults(&rep);
  CHECK(rep.problems == 0 && rep.kept == 0);
  CHECK(b.userSet == 0);  // defaults, not choices: the file can still "forget" back to them
  CHECK(sameSettings(b, card, "build"));
  CHECK(dump(b) == dump(card));

  // setting by setting, so a setting that has no macro is named (the whole point of this program)
  const Settings factory;
  for (size_t i = 0; i < settingCount(); i++) {
    const bool changed = fmt(b, i) != fmt(factory, i);
    if (!changed) printf("  '%s' has no build-time default: nothing in buildDefaults() sets it\n", settingKey(i));
    CHECK(changed);
  }
}

// Each macro is described where people look for it.
void testMacrosDocumented() {
  section("build defaults: documented");
  const std::string over = readFile("override_all.h");
  const std::string readme = readFile("../../README.md");
  const std::string example = readFile("../../secrets.example.h");
  const std::string config = readFile("../../config.h");
  int macros = 0;
  size_t pos = 0;
  while ((pos = over.find("\n#define ", pos)) != std::string::npos) {  // (at the start of a line: the comments say "#define" too)
    pos += 9;
    const std::string name = over.substr(pos, over.find_first_of(" \t\n", pos) - pos);
    macros++;
    if (readme.find("`" + name + "`") == std::string::npos) printf("  README does not mention `%s`\n", name.c_str());
    CHECK(readme.find("`" + name + "`") != std::string::npos);
    if (!hasWord(example, name)) printf("  secrets.example.h does not have %s\n", name.c_str());
    CHECK(hasWord(example, name));
    if (config.find("#ifndef " + name + "\n") == std::string::npos) printf("  config.h does not guard %s with #ifndef\n", name.c_str());
    CHECK(config.find("#ifndef " + name + "\n") != std::string::npos);
  }
  CHECK(macros >= (int)settingCount());  // at least one macro per setting (a few have more: the backup network)
}

#elif defined(TEST_BAD)

void testMistakes() {
  section("build defaults: values the card would refuse");
  BuildReport rep;
  const Settings b = buildDefaults(&rep);
  const Settings factory;

  // the seven bad ones stay as they were ...
  CHECK_STR(b.hostname, factory.hostname);
  CHECK(b.wifiPowerSave == WIFISAVE_NORMAL);
  CHECK(b.timeFormat == TIME_24H);
  CHECK_NEAR(b.batteryCutoffV, factory.batteryCutoffV, 1e-6);  // 2.0 V would ruin a LiPo
  CHECK(b.cpuMhz() == 80);
  CHECK(b.cpuIdle == CPUIDLE_OFF && b.cpuIdleMhz() == 0);  // 10 MHz is not on offer
  CHECK(b.console == CONSOLE_ON);
  // ... the good ones beside them count
  CHECK(b.dateFormat == DATE_D_MON_Y && !b.showWeek && b.batteryCapacityMah == 1800);
  CHECK_NEAR(b.latitude, -31.952240, 1e-6);  // coordinates keep their digits: about a metre
  CHECK_NEAR(b.longitude, 115.861456, 1e-5);
  CHECK(b.userSet == 0);

  // and they are reported, by the name of the macro and with the reason; the first three are kept
  CHECK(rep.problems == 7);
  CHECK(rep.kept == BuildReport::kKept);
  const char *want[] = {"APP_HOSTNAME: ", "WIFI_POWER_SAVE: ", "TIME_FORMAT: "};
  for (int i = 0; i < 3; i++) {
    if (strncmp(rep.text[i], want[i], strlen(want[i])) != 0) printf("  report %d is '%s'\n", i, rep.text[i]);
    CHECK(strncmp(rep.text[i], want[i], strlen(want[i])) == 0);
    CHECK(strlen(rep.text[i]) > strlen(want[i]) + 8);  // with a reason after it
  }
  CHECK(strstr(rep.text[1], "normal") && strstr(rep.text[1], "max"));  // says what would do
  CHECK(strstr(rep.text[2], "24h") && strstr(rep.text[2], "12h"));

  // no report asked for: same settings, nobody to tell
  const Settings quiet = buildDefaults();
  CHECK(dump(quiet) == dump(b));
}

#elif defined(TEST_EDGE)

void testLimits() {
  section("build defaults: values on the limits, placeholders");
  BuildReport rep;
  const Settings b = buildDefaults(&rep);
  if (rep.problems) {
    for (int i = 0; i < rep.kept; i++) printf("  refused: %s\n", rep.text[i]);
  }
  CHECK(rep.problems == 0);
  CHECK_NEAR(b.batteryCutoffV, 3.10, 1e-6);  // 3.10f must not fall below "3.10 and up"
  CHECK_NEAR(b.batteryCalibration, 0.80, 1e-6);
  CHECK_NEAR(b.indoorOffsetC, -15.0, 1e-6);
  CHECK(b.batteryCapacityMah == 20000 && b.weatherIntervalMin == 5);
  CHECK_NEAR(b.latitude, -90.0, 1e-9);
  CHECK_NEAR(b.longitude, 180.0, 1e-9);
  CHECK(b.cpuMhz() == 240);

  // the names secrets.example.h comes with mean "no network yet", passwords included
  CHECK(!b.hasMainWifi() && !b.hasBackupWifi());
  CHECK_STR(b.wifiSsid, "");
  CHECK_STR(b.wifiPassword, "");
  CHECK_STR(b.wifiBackupSsid, "");
  CHECK_STR(b.wifiBackupPassword, "");
}

#else

// The factory build says what Settings() says: the defaults in config.h and the ones that the struct
// is initialised with cannot drift apart (the example file shows the second kind).
void testFactory() {
  section("build defaults: the factory build");
  BuildReport rep;
  const Settings b = buildDefaults(&rep);
  CHECK(rep.problems == 0 && rep.kept == 0);
  CHECK(b.userSet == 0);
  CHECK(sameSettings(b, Settings(), "build"));
  CHECK(dump(b) == dump(Settings()));

  // The name of the power log's copy on an SD card is said in three places: config.h (which the clock uses), the
  // help of the power_log setting in the settings file, and the README.
  static char example[16384];
  const bool rendered = renderExampleConfig(Settings(), "", example, sizeof example) > 0;
  CHECK(rendered && strstr(example, POWER_LOG_FILE_NAME) != nullptr);
  CHECK(readFile("../../README.md").find("`" POWER_LOG_FILE_NAME "`") != std::string::npos);
  CHECK(rendered && strstr(example, "ESP32-S3-RLCD-PowerLog.txt") == nullptr);  // (the check can fail: a name that is nowhere)
}

// Does the README give `v` as the version: in its first lines, and as the first entry (the newest) of its
// list of versions?
bool readmeHasVersion(const std::string &readme, const std::string &v) {
  const size_t top = readme.find("Version **" + v + "**");
  const size_t list = readme.find("\n## Versions\n");
  if (top == std::string::npos || list == std::string::npos || top > list) return false;
  const size_t first = readme.find("\n**", list);
  return first != std::string::npos && readme.compare(first + 1, v.size() + 4, "**" + v + "**") == 0;
}

// APP_VERSION goes up by hand, and the README has to go with it: a number changed in one place and not
// the other is caught here.  (Nothing can catch a number that was not changed at all.)
void testVersionDocumented() {
  section("version: config.h and the README agree");
  const std::string readme = readFile("../../README.md");
  if (!readmeHasVersion(readme, APP_VERSION)) {
    printf("  the README does not give %s as the version (at the top, and first under 'Versions')\n", APP_VERSION);
  }
  CHECK(readmeHasVersion(readme, APP_VERSION));
  // the check can fail: the number in one of the two places only, or not first in the list
  const std::string top = "A clock. Version **7.1** (see below).\n", list = "\n## Versions\n\nBy hand.\n\n**7.1** (a day)\n\n* more\n\n**7.0**: less.\n";
  CHECK(readmeHasVersion(top + list, "7.1"));
  CHECK(!readmeHasVersion(top + list, "7.0"));
  CHECK(!readmeHasVersion(top + list, "7"));
  CHECK(!readmeHasVersion(top, "7.1"));
  CHECK(!readmeHasVersion(list, "7.1"));
  CHECK(!readmeHasVersion("A clock. Version **7.0**.\n" + list, "7.0"));
  CHECK(!readmeHasVersion(list + top, "7.1"));
}

#endif

}  // namespace

int main() {
#if defined(TEST_ALL)
  testEverythingReplaced();
  testMacrosDocumented();
#elif defined(TEST_BAD)
  testMistakes();
#elif defined(TEST_EDGE)
  testLimits();
#else
  testFactory();
  testVersionDocumented();
#endif
  printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
