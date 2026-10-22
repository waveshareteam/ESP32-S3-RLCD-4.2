// 11_Clock_Dashboard - desk clock for the Waveshare ESP32-S3-RLCD-4.2
//
//   * joins WiFi, sets the clock over NTP and works out the time zone (with DST)
//   * date (ISO by default) and time with seconds, plus an analog clock; 12 hour clock optional
//   * current weather, today's high/low, the next two days, the moon and the chance of rain
//   * indoor temperature and humidity from the on-board SHTC3
//   * battery gauge with a charging / discharging / full icon (worked out from the
//     voltage, see charge.h) that blinks below 20 % unless it is charging; an estimate of the
//     runtime left on the Info page; a gentle shutdown before the cell is flat
//   * Spotify now-playing, controlled with the KEY button
//       1 click = play/pause    2 clicks = next    3 clicks = previous
//   * BOOT button: click = next page, double click = home, hold = invert screen
//   * settings from a file on an SD card (units, formats, WiFi, location, ...), see the README
//
// Setup: copy secrets.example.h to secrets.h and enter your WiFi details (or put them in a
// settings file on an SD card), adjust config.h if needed.  Arduino IDE settings are in the README.
//
// Threads: this file is the UI task (core 1) which owns the display, the I2C
// bus and the buttons.  net_task.cpp and spotify.cpp run on core 0 and hand data
// over through app_state.h, so a slow HTTPS request never stalls the clock.

#include <Arduino.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <sys/time.h>

#include "ST7305_U8g2.h"
#include "app_power_log.h"
#include "app_settings.h"
#include "app_state.h"
#include "battery_est.h"
#include "button_edges.h"
#include "buttons.h"
#include "calc.h"
#include "charge.h"
#include "clock_policy.h"
#include "config.h"
#include "console_policy.h"
#include "display_power.h"
#include "frame_plan.h"
#include "fw_update.h"
#include "infinity_cube.h"
#include "light_sleep.h"
#include "log.h"
#include "low_battery.h"
#include "moon.h"
#include "net_task.h"
#include "power.h"
#include "sdcard.h"
#include "sensors.h"
#include "spotify.h"
#include "timeutil.h"
#include "ui.h"
#include "util.h"

SET_LOOP_TASK_STACK_SIZE(12 * 1024);

static const char *TAG = "main";
static const time_t kMinPlausibleEpoch = 1704067200;  // 2024-01-01

static ST7305_U8g2 *s_lcd = nullptr;
static u8g2_t *s_u8g2 = nullptr;
static Preferences s_uiPrefs;  // namespace "ui": remembers the screen polarity toggle

static ClickDetector s_key(KEY_MULTI_CLICK_GAP_MS, KEY_LONG_PRESS_MS);
static ClickDetector s_boot(KEY_MULTI_CLICK_GAP_MS, BOOT_LONG_PRESS_MS);
static HoldRelease s_restartHold(KEY_RESTART_HOLD_MS);  // KEY held for five seconds, then let go: restart
// KEY and BOOT held down together for ten seconds: a cube that tumbles (cubeFrame).  It is in no legend and no
// manual.  While it is up, KEY gives it a shove in some direction, BOOT shows the other of the two cubes, and a
// long press of either (or two minutes with no button touched) brings the clock back.
static const uint32_t kCubeHoldMs = 10000, kCubeIdleMs = 120000;
static ChordHold s_bothHeld(kCubeHoldMs, KEY_MULTI_CLICK_GAP_MS + 200);
static bool s_cubeOn = false;
static UiCube s_cube;       // which cube (it stays the one that was shown last), and how far it has tumbled
static infcube::Shove s_cubeShove;  // what KEY has done to it
static uint32_t s_cubeStartMs = 0, s_cubeFrameMs = 0, s_cubeTouchMs = 0;
static bool s_restartAsked = false;                     // ... which loop() then does

static UiPage s_page = PAGE_DASHBOARD;
static bool s_forceRedraw = true;
static bool s_flipPolarity = false;  // user toggled the screen polarity (XOR with config)

// Frame pipeline.  The coming second is drawn into the display buffer early (prepareFrame) and
// sent just before it starts (flushFrame), so a slow draw cannot make the digits late; the
// planner (frame_plan.h) decides when.
static FramePlanner s_planner(DISPLAY_LATENCY_MS * 1000);
static uint32_t s_lastFrameMs = 0;

// The CPU clock (clock_policy.h): fast while the radio, a computer on the USB port or a button needs it
static uint32_t s_pokeUntilMs = 0;        // a button was pressed: stay fast until then (0 = no)
static uint32_t s_clockSwitches = 0;      // how often the clock was changed (shown by the "power" console command)
static const char *s_clockReason = "";    // why the clock is where it is, for the Power page
static int s_clockRefusedMhz = 0;         // a clock the core would not set: not asked for again
static bool usbHostAttached();            // (defined with the clock code further down)

// Light sleep between the frames (light_sleep = on, light_sleep.h)
static lightsleep::Stats s_sleepStats;               // how much of the time the chip slept
static lightsleep::WallFix s_wallFix;                // the time of day after a nap
static lightsleep::Why s_sleepWhy = lightsleep::OFF;  // why it did or did not sleep, the last time the loop asked
static uint32_t s_sleepRefusedMs = 0;                // the SDK refused to sleep then (0 = it did not): wait a while

// The USB serial console (console_policy.h, log.h)
static_assert((int)CONSOLE_ON == (int)consolepolicy::ON && (int)CONSOLE_AUTO == (int)consolepolicy::AUTO &&
                  (int)CONSOLE_OFF == (int)consolepolicy::OFF,
              "the console setting and its policy use the same numbers");
static consolepolicy::Watch s_consoleWatch;
static bool s_consoleKeep = false;        // KEY was held at start-up: the console stays on for this run
static int s_audioStandby = -1;           // which audio chips took the power-down (sensors.h), -1 = not asked

// How well the frames kept time, shown on the Info page
struct TimingStats {
  uint32_t frames = 0;      // frames sent at their scheduled moment
  uint32_t late = 0;        // ...of which this many went out more than 30 ms behind plan
  int32_t worstLateUs = 0;  // the worst of them
  uint32_t clockSteps = 0;  // times the system clock jumped (an NTP correction) by more than 5 ms
  int32_t lastStepUs = 0;
};
static TimingStats s_timing;

static IndoorReading s_indoor;
static uint32_t s_indoorMs = 0;
static uint32_t s_lastSensorMs = 0;

static float s_batVolts = 0;
static bool s_batHave = false;
static bool s_batLow = false;
static uint32_t s_lastBatMs = 0;
static ChargeDetector s_charge;  // works out charging / discharging from how the voltage moves
static int s_chargeState = CHARGE_UNKNOWN;  // UiCharge: the detector, overruled by the STAT pin if one is wired
static uint32_t s_lastBatLogMs = 0;

static battest::Estimator s_est;       // runtime left, from the discharge so far (battery_est.h)
static battest::Gate s_estGate;        // ... fed whenever no charger shows
static lowbat::Guard s_guard;          // switches the clock off before the cell is flat (low_battery.h)
static bool s_guardOverride = false;   // the user held KEY on the "battery empty" screen: run anyway

static char s_toast[40] = "";
static int s_toastIcon = TOAST_NONE;
static uint32_t s_toastUntilMs = 0;
static uint32_t s_seenNoticeSeq = 0;

// ---------------------------------------------------------------------------
// Toasts and key handling
// ---------------------------------------------------------------------------
static void showToast(const char *text, int icon, uint32_t durationMs = 2200) {
  copyStr(s_toast, sizeof s_toast, text);
  s_toastIcon = icon;
  s_toastUntilMs = millis() + durationMs;
  s_forceRedraw = true;
}

static void applyPolarity() {
  if (s_lcd) s_lcd->setInvert((DISPLAY_INK_IS_BLACK != 0) != s_flipPolarity);
}

// KEY: Spotify transport.  The toast appears immediately; the network task does
// the work and may replace the toast if Spotify refuses.
static void handleKey(ClickEvent e) {
  // wifi_mode = sync: the radio comes on for any press of KEY (a few seconds before it is answered), also
  // when Spotify is not set up: it is the way to say "look for the network now"
  netWake();
  if (e == CLICK_LONG) {  // refreshes the weather too, so it works without Spotify
    showToast("Refreshing...", TOAST_NONE, 1500);
    spotifyPost(SPOTIFY_CMD_REFRESH);
    netRequestWeatherRefresh();
    return;
  }
  SpotifyStatus st;
  {
    StateLock lock;
    st = g_state.spotify.status;
  }
  if (st == SPOTIFY_DISABLED) {
    showToast(g_cfg.wifi ? "Spotify not set up" : "WiFi is off", TOAST_WARN);
    return;
  }
  if (st == SPOTIFY_NEEDS_LINK) {
    showToast("Link Spotify first", TOAST_WARN);
    return;
  }
  switch (e) {
    case CLICK_1:
      if (st == SPOTIFY_PLAYING) {
        showToast("Pause", TOAST_PAUSE);
      } else {
        showToast("Play", TOAST_PLAY);
      }
      spotifyPost(SPOTIFY_CMD_PLAY_PAUSE);
      break;
    case CLICK_2:
      showToast("Next track", TOAST_NEXT);
      spotifyPost(SPOTIFY_CMD_NEXT);
      break;
    case CLICK_3:
      showToast("Previous track", TOAST_PREV);
      spotifyPost(SPOTIFY_CMD_PREVIOUS);
      break;
    default:
      break;
  }
}

// BOOT: pages and screen polarity.
static void handleBoot(ClickEvent e) {
  netNudge();  // wifi_mode = sync: someone is here, so a clock that found none of its networks looks again now
  switch (e) {
    case CLICK_1:
      s_page = (UiPage)((s_page + 1) % PAGE_COUNT);
      break;
    case CLICK_2:
      s_page = PAGE_DASHBOARD;
      break;
    case CLICK_3:
      s_page = PAGE_INFO;
      break;
    case CLICK_LONG:
      s_flipPolarity = !s_flipPolarity;
      s_uiPrefs.putBool("flip", s_flipPolarity);
      applyPolarity();
      showToast("Screen inverted", TOAST_NONE, 1500);
      break;
    default:
      break;
  }
  s_forceRedraw = true;
}

// display_power: how the controller drives the panel (its low power mode at 8 Hz, or the high one at 25.5 Hz,
// display_power.h), and the frame timing that goes with it.  The cube runs in the high power mode whatever the
// setting: it sends frames as fast as it can draw them, and 8 a second would make it judder.  A switch takes 120 ms
// into the low power mode and 320 ms out of it (the datasheet's waits).
static void applyDisplayPower() {
  if (!s_lcd) return;
  const bool low = g_cfg.displayPower == DISPPOWER_LOW && !s_cubeOn;
  s_lcd->setLowPower(low);
  s_planner.setLatencyUs(dispower::latencyUs(low, DISPLAY_LATENCY_MS * 1000));
}

// Before every deep sleep: the panel shows the battery-empty screen for as long as the clock sleeps, days perhaps,
// and its controller stays powered, so it refreshes as seldom as it can (0.25 Hz).  A wake-up that starts the clock
// runs fullInit() and with it the usual rate; a timer check that goes back to sleep leaves the panel alone.
static void displayRest() {
  if (s_lcd) s_lcd->setRestMode();
}

static void cubeBegin(uint32_t nowMs) {
  s_cubeOn = true;
  applyDisplayPower();
  s_cube.seed = esp_random() | 1u;  // it tumbles another way every time
  s_cube.turnMs = 0;
  s_cube.elapsedMs = 0;
  s_cubeShove = infcube::Shove();
  s_cubeStartMs = s_cubeFrameMs = s_cubeTouchMs = nowMs;
}

static void cubeEnd() {
  s_cubeOn = false;
  applyDisplayPower();
  s_forceRedraw = true;  // the clock again, at once
}

// A button while the cube is up.
static void cubeButton(bool key, ClickEvent e) {
  if (e == CLICK_LONG) {
    cubeEnd();
    return;
  }
  s_cubeTouchMs = millis();
  if (key) {
    s_cubeShove.bump(esp_random());  // off in some direction, for a moment
  } else {
    s_cube.mirrors = !s_cube.mirrors;  // the other cube, where this one is
  }
}

// What a button's gesture goes to: the cube while it is up, else what the button is for.
static void onKey(ClickEvent e) {
  if (s_cubeOn) {
    cubeButton(true, e);
  } else {
    handleKey(e);
  }
}
static void onBoot(ClickEvent e) {
  if (s_cubeOn) {
    cubeButton(false, e);
  } else {
    handleBoot(e);
  }
}

static void pollButtons(uint32_t nowMs) {
  bool active = false;
  // the two buttons are down together (or were a moment ago): what each of them reports does not count
  const bool muted = s_bothHeld.muted();
  if (buttonEdgesOn()) {
    // With an idle clock: what the interrupts recorded while the loop was busy drawing, replayed in order,
    // so that a tap which began and ended inside a slow frame still counts (button_edges.h).
    bool down;
    uint32_t at;
    while (buttonEdgeTake(BUTTON_KEY, &down, &at)) {
      active = true;
      const ClickEvent e = s_key.edge(down, at);
      if (e != CLICK_NONE && !muted) onKey(e);
    }
    while (buttonEdgeTake(BUTTON_BOOT, &down, &at)) {
      active = true;
      const ClickEvent e = s_boot.edge(down, at);
      if (e != CLICK_NONE && !muted) onBoot(e);
    }
    nowMs = millis();  // not earlier than what was just replayed
  }
  const bool keyDown = digitalRead(PIN_KEY) == LOW, bootDown = digitalRead(PIN_BOOT) == LOW;
  // (with the record, the loop's own look is one more change at "now": should the record have been full and
  // the end of a press be missing from it, the press still gets the time it lasted)
  ClickEvent k = buttonEdgesOn() ? s_key.edge(keyDown, nowMs) : s_key.update(keyDown, nowMs);
  ClickEvent b = buttonEdgesOn() ? s_boot.edge(bootDown, nowMs) : s_boot.update(bootDown, nowMs);
  // a button is down, or was a moment ago: the answer is drawn at the working clock (cpu_idle_mhz)
  if (active || keyDown || bootDown || k != CLICK_NONE || b != CLICK_NONE) s_pokeUntilMs = (nowMs + clockpolicy::kPokeMs) | 1u;
  if (k != CLICK_NONE && !muted) onKey(k);
  if (b != CLICK_NONE && !muted) onBoot(b);

  // KEY and BOOT held down together for ten seconds: the cube.  While the two are down, and until both are up
  // again, neither is a long press, a click or the hold that restarts the clock.
  if (s_bothHeld.update(s_key.isDown(), s_boot.isDown(), nowMs) == ChordHold::FIRED && !s_cubeOn) cubeBegin(nowMs);
  if (s_bothHeld.muted()) {
    s_restartHold.cancel();
    if (!strcmp(s_toast, UI_TOAST_RESTART_HOLD)) {  // KEY was held long enough to restart, and then BOOT joined it
      s_toast[0] = 0;
      s_forceRedraw = true;
    }
    return;
  }

  // KEY held for five seconds and then let go: a restart, which is when the clock reads the SD card (a settings
  // file, a firmware file).  It waits for the button to come up, because KEY held while the clock starts means
  // "keep the console" (and, on the battery-empty screen, "start anyway").
  switch (s_restartHold.update(s_key.isDown(), s_key.heldMs())) {
    case HoldRelease::ARMED:
      showToast(UI_TOAST_RESTART_HOLD, TOAST_NONE, 60000);
      break;
    case HoldRelease::FIRED:
      s_restartAsked = true;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Sensors, battery, RTC (slow work, done away from the second boundary)
// ---------------------------------------------------------------------------
static const char *chargeName(int c) {
  switch (c) {
    case CHARGE_CHARGING: return "charging";
    case CHARGE_DISCHARGING: return "discharging";
    case CHARGE_FULL: return "full";
    default: return "unknown";
  }
}

// What the battery is doing: the voltage-based guess, overruled by the charger's STAT pin when
// one is wired (PIN_CHARGE_STATUS in config.h).  The pin only knows "charging" for certain: not
// charging could be unplugged or full, so then the voltage still has to say which.
static int resolveCharge() {
  int state = CHARGE_UNKNOWN;
  switch (s_charge.state()) {
    case ChargeDetector::CHARGING: state = CHARGE_CHARGING; break;
    case ChargeDetector::DISCHARGING: state = CHARGE_DISCHARGING; break;
    case ChargeDetector::FULL: state = CHARGE_FULL; break;
    default: break;
  }
#if PIN_CHARGE_STATUS >= 0
  if (digitalRead(PIN_CHARGE_STATUS) == LOW) return CHARGE_CHARGING;
  if (state == CHARGE_CHARGING) state = s_batVolts >= ChargeDetector::kTopV ? CHARGE_FULL : CHARGE_DISCHARGING;
#endif
  return state;
}

// The screen the clock leaves behind when it switches itself off, then deep sleep; never returns.
// (No [[noreturn]] here: the IDE generates a prototype for functions in a .ino without it.)
static void shutdownForLowBattery(float volts) {
  LOGF(TAG, "battery %.2f V is under the %.2f V cut-off: switching off (restarts above %.2f V)", volts,
       g_cfg.batteryCutoffV, lowbat::kRestartV);
  u8g2_ClearBuffer(s_u8g2);
  uiDrawBatteryEmpty(s_u8g2, volts, lowbat::kRestartV, true);
  u8g2_SendBuffer(s_u8g2);
  displayRest();
  powerDeepSleep(true);
}

// The screen of the firmware update from the SD card (fw_update.cpp calls this for every step).
static void showFirmwareScreen(const UiFwScreen &screen) {
  u8g2_ClearBuffer(s_u8g2);
  uiDrawFirmwareUpdate(s_u8g2, screen);
  u8g2_SendBuffer(s_u8g2);
}

static void readSlowSensors(uint32_t nowMs) {
  if (s_lastSensorMs == 0 || nowMs - s_lastSensorMs >= SENSOR_INTERVAL_MS) {
    IndoorReading r;
    if (readIndoor(r)) {
      s_indoor = r;
      s_indoorMs = nowMs;
    }
    s_lastSensorMs = nowMs ? nowMs : 1;
  }
  if (s_lastBatMs == 0 || nowMs - s_lastBatMs >= BATTERY_INTERVAL_MS) {
    BatteryReading b = readBattery();
    if (b.present && g_cfg.hasBattery()) {
      // smooth out the dips caused by WiFi transmit bursts
      s_batVolts = s_batHave ? s_batVolts * 0.75f + b.volts * 0.25f : b.volts;
      s_batHave = true;
      s_charge.addSample(b.volts, nowMs);  // the raw reading: the detector filters it its own way
    } else {  // no cell, or "battery = none" in the settings: the gauge says USB
      s_batHave = false;
      s_charge.reset();
    }
    s_chargeState = s_batHave ? resolveCharge() : CHARGE_UNKNOWN;

    // runtime estimate: the readings taken while the clock runs on its battery as far as it can tell, which is
    // whenever no charger shows (not only once the detector says "discharging": a light load never earns that
    // word), and a fresh start whenever a charger comes or goes
    // seconds since start from the 64-bit timer: millis() / 1000 would jump back to 0 after 49.7 days
    const uint32_t nowSec = (uint32_t)(esp_timer_get_time() / 1000000);
    s_estGate.feed(s_est, s_batHave, s_chargeState == CHARGE_CHARGING || s_chargeState == CHARGE_FULL, nowSec, b.volts);

    // the cut-off: a minute under it, while not charging (low_battery.h)
    if (s_batHave && s_guard.feed(nowSec, b.volts, s_chargeState != CHARGE_CHARGING && s_chargeState != CHARGE_FULL)) {
      shutdownForLowBattery(b.volts);
    }

    if (s_batHave && (s_lastBatLogMs == 0 || nowMs - s_lastBatLogMs >= 60000UL)) {
      s_lastBatLogMs = nowMs ? nowMs : 1;
      LOGF(TAG, "battery %.3f V (reading %.3f)  %s  trend %+.2f mV/min  step %+.1f mV  %.1f min of data", s_batVolts,
           b.volts, chargeName(s_chargeState), s_charge.slopeMvPerMin(), s_charge.stepMv(), s_charge.minutesOfData());
    }
    int pct = s_batHave ? calc::batteryPercent(s_batVolts) : 0;
    if (!s_batLow && pct < BATTERY_LOW_PERCENT) {
      s_batLow = true;
    } else if (s_batLow && pct >= BATTERY_LOW_CLEAR_PERCENT) {
      s_batLow = false;
    }
    if (!s_batHave) s_batLow = false;
    s_lastBatMs = nowMs ? nowMs : 1;
  }
  if (netTakeRtcWriteRequest()) {
    time_t t = time(nullptr);
    if (t > kMinPlausibleEpoch && rtcWriteUtc(t)) LOGF(TAG, "RTC set from NTP");
  }
}

// ---------------------------------------------------------------------------
// Building the frame model
// ---------------------------------------------------------------------------
static void cityFromZone(const char *zone, char *out, size_t cap) {
  const char *slash = strrchr(zone, '/');
  copyStr(out, cap, slash ? slash + 1 : zone);
  for (char *p = out; *p; ++p) {
    if (*p == '_') *p = ' ';
  }
}

static void formatAge(uint32_t sec, char *out, size_t cap) {
  if (sec < 90) {
    snprintf(out, cap, "%us ago", (unsigned)sec);
  } else if (sec < 5400) {
    snprintf(out, cap, "%u min ago", (unsigned)(sec / 60));
  } else {
    snprintf(out, cap, "%u h ago", (unsigned)(sec / 3600));
  }
}

// "45 s", "9 min", "1 h 5 min" for a time still to come; "" when it is not known (negative).
static void formatIn(int32_t sec, char *out, size_t cap) {
  if (sec < 0) {
    if (cap) out[0] = 0;
  } else if (sec < 90) {
    snprintf(out, cap, "%d s", (int)sec);
  } else if (sec < 5400) {
    snprintf(out, cap, "%d min", (int)((sec + 30) / 60));
  } else {
    snprintf(out, cap, "%d h %d min", (int)(sec / 3600), (int)((sec / 60) % 60));
  }
}

// The firmware's build date as 2026-10-04 (from the compiler's "Oct  4 2026").
static void buildDateIso(char *out, size_t cap) {
  static const char *kMonths = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *d = __DATE__;
  int month = 0;
  for (int i = 0; i < 12; i++)
    if (!strncmp(d, kMonths + 3 * i, 3)) month = i + 1;
  const int day = atoi(d + 4);
  const int year = atoi(d + 7);
  snprintf(out, cap, "%04d-%02d-%02d", year, month, day);
}

// What the runtime estimate says, for the Info pages.
static void estimateText(char *out, size_t cap) {
  if (!s_batHave) {
    snprintf(out, cap, "%s", g_cfg.hasBattery() ? "no battery detected" : "no battery (set in the settings)");
    return;
  }
  switch (s_chargeState) {
    case CHARGE_CHARGING: snprintf(out, cap, "charging"); return;
    case CHARGE_FULL: snprintf(out, cap, "full"); return;
    default: break;  // discharging, or nothing seen of a charger: on its battery as far as the clock can tell
  }
  const battest::Estimate e = s_est.estimate(g_cfg.batteryCutoffV, (float)g_cfg.batteryCapacityMah);
  char left[24];
  switch (e.state) {
    case battest::Estimate::READY:
      if (e.pctPerHour < battest::kMinRatePctPerHour && s_chargeState != CHARGE_DISCHARGING) {
        // no fall to speak of, and no fall was ever seen: a charger that has finished looks like this for the
        // first hours after a start (charge.h), and so does a clock that draws next to nothing
        snprintf(out, cap, "no drain to measure (on a charger?)");
      } else if (e.pctPerHour < battest::kMinRatePctPerHour) {
        snprintf(out, cap, "more than a month at this rate");
      } else {
        battest::formatRemaining(e.hoursLeft, left, sizeof left);  // (">30 d" beyond a month: the rate is still worth reading)
        // two decimals under 10 %/h: with the radio mostly off the whole figure is 0.2 to 0.7, and rounded to
        // one decimal two settings that differ by a third read the same
        if (e.pctPerHour < 9.995f) {
          snprintf(out, cap, "%s (%.2f %%/h over %d min)", left, (double)e.pctPerHour, e.windowMin);
        } else {
          snprintf(out, cap, "%s (%.1f %%/h over %d min)", left, (double)e.pctPerHour, e.windowMin);
        }
      }
      break;
    case battest::Estimate::LEARNING: snprintf(out, cap, "learning: first figure in %d min", e.learnMin); break;
    default: snprintf(out, cap, "starting"); break;
  }
}

static const char *dateFormatName(int f) {
  static const char *const names[] = {"ISO date", "DD/MM/YYYY", "MM/DD/YYYY", "DD.MM.YYYY", "D Mon YYYY", "Mon D, YYYY"};
  return (f >= 0 && f < (int)(sizeof names / sizeof *names)) ? names[f] : "?";
}

// SYSTEM INFO: network, time, weather, sensors, health of the frame pipeline.
static void buildInfoSystem(UiModel &m, const SharedState &s, uint32_t nowMs, time_t shown) {
  int n = 0;
  char age[24];
  // lines are cut to the width of the info page rather than overflowing it
  auto add = [&](const char *label, const char *value) {
    if (n < UI_INFO_LINES) snprintf(m.info[n++], UI_INFO_LINE_LEN, "%-9.9s %.*s", label, UI_INFO_LINE_LEN - 12, value);
  };
  char v[128];

  char built[16];
  buildDateIso(built, sizeof built);
  // (the date is that of the day this file was compiled; the build id changes whenever the program does)
  static_assert(sizeof(APP_NAME " " APP_VERSION ", 2026-10-07, build 0123456") - 1 <= UI_INFO_LINE_LEN - 12,
                "the Firmware row of the Info page would be cut short: APP_NAME or APP_VERSION is too long for it");
  snprintf(v, sizeof v, APP_NAME " " APP_VERSION ", %s, build %s", built, fwBuildId());
  add("Firmware", v);
  uint32_t trialLeft;
  if (fwOnTrial(nowMs, &trialLeft)) {  // just installed from the SD card: a reset now puts the old one back
    if (trialLeft > 0) {
      snprintf(v, sizeof v, "new firmware on trial, kept in %u s", (unsigned)trialLeft);
    } else {
      snprintf(v, sizeof v, "new firmware NOT confirmed yet (see the log)");  // the bootloader refused it so far
    }
    add("Update", v);
  } else if (fwRolledBack()) {
    add("Update", "the last SD card update was rolled back");
  }

  if (!g_cfg.wifi) {
    snprintf(v, sizeof v, "off (wifi = off in the settings)");
  } else if (s.wifiUp) {
    snprintf(v, sizeof v, "%s%s  %d dBm", s.ssid, s.wifiBackup ? " (backup)" : "", s.rssi);
  } else if (s.radioSync && (s.radioAsleep || s.radioTrouble == RADIO_AWAY)) {  // wifi_mode = sync, between two sessions
    char in[16];
    formatIn(s.radioWakeInSec, in, sizeof in);
    const bool when = s.radioAsleep && s.radioWakeInSec >= 0;
    if (s.radioTrouble == RADIO_AWAY && !s.radioAsleep) {
      snprintf(v, sizeof v, "none in range, looking now");
    } else if (s.radioTrouble == RADIO_AWAY) {  // none of its networks answered: it looks every few minutes, and now if a button is pressed
      snprintf(v, sizeof v, "none in range%s%s", when ? ", looking again in " : "", when ? in : "");
    } else if (s.radioTrouble == RADIO_CANNOT_JOIN) {  // a network is there and will not let the clock in
      snprintf(v, sizeof v, "cannot join%s%s", when ? ", next try in " : "", when ? in : "");
    } else {
      snprintf(v, sizeof v, "asleep (sync mode)%s%s", when ? ", next in " : "", when ? in : "");
    }
  } else {
    snprintf(v, sizeof v, "not connected");
  }
  add("WiFi", v);
  snprintf(v, sizeof v, "%s  %s.local", s.ip[0] ? s.ip : "-", g_cfg.hostname);
  add("Address", v);

  if (s.ntpSynced) {
    formatAge((uint32_t)(shown > s.lastNtpSyncUtc ? shown - s.lastNtpSyncUtc : 0), age, sizeof age);
    snprintf(v, sizeof v, "NTP synced %s", age);
    if (s_timing.clockSteps) {  // the size of the last correction the clock took
      const size_t l = strlen(v);
      snprintf(v + l, sizeof v - l, ", last step %+d ms", (int)(s_timing.lastStepUs / 1000));
    }
  } else if (s.timeTrusted) {
    snprintf(v, sizeof v, "%s", g_cfg.wifi ? "from RTC, NTP pending" : "from the RTC chip, no network time");
  } else {
    snprintf(v, sizeof v, "%s", g_cfg.wifi ? "not set yet" : "not set: turn WiFi on once");
  }
  add("Time", v);
  snprintf(v, sizeof v, "%s  %s", s.tzName[0] ? s.tzName : "-", s.tzPosix[0] ? s.tzPosix : "-");
  add("Zone", v);
  if (s.locationKnown) {
    snprintf(v, sizeof v, "%.3f, %.3f", s.lat, s.lon);
  } else if (g_cfg.hasLatLon()) {
    snprintf(v, sizeof v, "%.3f, %.3f", g_cfg.latitude, g_cfg.longitude);
  } else {
    snprintf(v, sizeof v, "unknown");
  }
  add("Location", v);

  if (s.weather.valid) {
    formatAge((nowMs - s.weatherFetchedMs) / 1000, age, sizeof age);
    snprintf(v, sizeof v, "%s - Open-Meteo.com", age);  // their terms ask for credit
  } else {
    snprintf(v, sizeof v, "%s", g_cfg.wifi ? "no data yet - Open-Meteo.com" : "off (no WiFi)");
  }
  add("Weather", v);

  if (s_batHave) {
    char how[32];
    if (s_chargeState == CHARGE_UNKNOWN && s_charge.warmingUp()) {
      snprintf(how, sizeof how, "starting up");  // the first minutes after boot are not used
    } else if (s_chargeState == CHARGE_UNKNOWN && !s_charge.ready()) {
      snprintf(how, sizeof how, "learning %.0f of 8 min", s_charge.minutesOfData());
    } else if (s_charge.ready()) {
      // (neither charging nor held full, and not seen to fall either, which a light load never is: on its battery, then)
      snprintf(how, sizeof how, "%s %+.1f mV/min", s_chargeState == CHARGE_UNKNOWN ? "on battery?" : chargeName(s_chargeState),
               s_charge.slopeMvPerMin());
    } else {
      snprintf(how, sizeof how, "%s", chargeName(s_chargeState));
    }
    snprintf(v, sizeof v, "%.2f V  %d%%%s  %s", s_batVolts, calc::batteryPercent(s_batVolts), s_batLow ? "  LOW" : "", how);
  } else {
    snprintf(v, sizeof v, "%s", g_cfg.hasBattery() ? "no battery detected (USB)" : "none (battery = none): USB power");
  }
  add("Battery", v);
  estimateText(v, sizeof v);
  add("Left", v);

  if (s_indoor.valid) {
    snprintf(v, sizeof v, "%.1f C  %.0f%%   (sensor %.1f C)", s_indoor.tempC, s_indoor.rh, s_indoor.rawTempC);
  } else {
    snprintf(v, sizeof v, "sensor not answering");
  }
  add("Indoor", v);

  switch (s.spotify.status) {
    case SPOTIFY_DISABLED:
      snprintf(v, sizeof v, "%s",
               !g_cfg.wifi ? "off (no WiFi)" : (!g_cfg.spotify ? "off (spotify = off)" : "not set up (no client id)"));
      break;
    case SPOTIFY_NEEDS_LINK: snprintf(v, sizeof v, "waiting to be linked"); break;
    case SPOTIFY_ERROR: snprintf(v, sizeof v, "%s", s.spotify.message); break;
    case SPOTIFY_PLAYING: snprintf(v, sizeof v, "linked, playing"); break;
    case SPOTIFY_PAUSED: snprintf(v, sizeof v, "linked, paused"); break;
    default: snprintf(v, sizeof v, "linked, idle"); break;
  }
  if (s.spotify.linkDaysLeft >= 0 && s.spotify.status != SPOTIFY_NEEDS_LINK && s.spotify.status != SPOTIFY_DISABLED) {
    size_t l = strlen(v);
    snprintf(v + l, sizeof v - l, " (link: %d d left)", s.spotify.linkDaysLeft);
  }
  add("Spotify", v);

  uint32_t up = nowMs / 1000;
  snprintf(v, sizeof v, "%ud %uh %um, %u KB free", (unsigned)(up / 86400), (unsigned)((up / 3600) % 24),
           (unsigned)((up / 60) % 60), (unsigned)(ESP.getFreeHeap() / 1024));
  add("Uptime", v);
  // Did the seconds come out on time?  "late" is more than 30 ms behind plan; if you see a
  // stutter but this stays at zero, the delay is outside the firmware (panel, NTP offset).
  snprintf(v, sizeof v, "%u sent, %u late, worst %d ms", (unsigned)s_timing.frames, (unsigned)s_timing.late,
           (int)(s_timing.worstLateUs / 1000));
  add("Frames", v);
  m.infoCount = n;
}

// POWER AND SETTINGS: what the clock draws and how long it will last, the accuracy of its
// clock, and where each setting came from.
static void buildInfoPower(UiModel &m, const SharedState &s) {
  int n = 0;
  auto add = [&](const char *label, const char *value) {
    if (n < UI_INFO_LINES) snprintf(m.info[n++], UI_INFO_LINE_LEN, "%-9.9s %.*s", label, UI_INFO_LINE_LEN - 12, value);
  };
  char v[128];

  {  // the clock, and why it is where it is when there is an idle clock (a line holds 42 characters of value)
    const char *wifi = !g_cfg.wifi ? "WiFi off" : (g_cfg.wifiPowerSave == WIFISAVE_MAX ? "WiFi saver max" : "WiFi saver normal");
    if (g_cfg.cpuIdleMhz() > 0 && s_clockReason[0]) {
      snprintf(v, sizeof v, "%u MHz (%s), %s", (unsigned)getCpuFrequencyMhz(), s_clockReason, wifi);
    } else {
      snprintf(v, sizeof v, "%u MHz, %s", (unsigned)getCpuFrequencyMhz(), wifi);
    }
    add("Power", v);
  }
  if (s.radioSync && g_cfg.wifi) {  // wifi_mode = sync: how much of the time the radio was on
    char in[16];
    formatIn(s.radioWakeInSec, in, sizeof in);
    if (s.radioAsleep && s.radioWakeInSec >= 0) {
      snprintf(v, sizeof v, "on %.1f %%, %u sessions, next in %s", s.radioOnPermille / 10.0, (unsigned)s.radioSessions, in);
    } else {
      snprintf(v, sizeof v, "on %.1f %% of the time, %u sessions", s.radioOnPermille / 10.0, (unsigned)s.radioSessions);
    }
    add("Radio", v);
  }
  if (g_cfg.lightSleep) {  // light_sleep = on: how much of the time the chip slept, and what keeps it awake now
    char row[lightsleep::kRow];
    lightsleep::formatRow(s_sleepStats.permille(esp_timer_get_time()), s_sleepWhy, row);
    add("Sleep", row);
  }
  if (psramFound()) add("PSRAM", "ON: wastes power (Tools > PSRAM)");
  if (!g_consoleOn) {  // (a line holds 42 characters of value)
    add("Console", "off: restart with KEY held to get it back");
  } else if (s_consoleKeep && g_cfg.console != CONSOLE_ON) {
    add("Console", "kept on for this run (KEY was held)");
  } else if (g_cfg.console == CONSOLE_AUTO) {
    add("Console", "on while a computer is on the USB port");
  }

  if (s_batHave) {
    if (fabsf(g_cfg.batteryCalibration - 1.0f) > 0.0005f) {  // say so when the voltage has been corrected
      snprintf(v, sizeof v, "%.3f V (x%.4f) = %.1f %% on the curve", s_batVolts, (double)g_cfg.batteryCalibration,
               (double)calc::batteryPercentF(s_batVolts));
    } else {
      snprintf(v, sizeof v, "%.3f V = %.1f %% on the curve", s_batVolts, (double)calc::batteryPercentF(s_batVolts));
    }
  } else {
    snprintf(v, sizeof v, "no battery in use");
  }
  add("Battery", v);
  estimateText(v, sizeof v);
  add("Left", v);
  if (s_batHave && s_estGate.onBattery() && g_cfg.batteryCapacityMah > 0) {
    const battest::Estimate e = s_est.estimate(g_cfg.batteryCutoffV, (float)g_cfg.batteryCapacityMah);
    if (e.state == battest::Estimate::READY) {
      if (e.avgMa < 99.95f) {  // (one decimal while it tells something: 9.6 and 10.4 mA are both "about 10")
        snprintf(v, sizeof v, "about %.1f mA (of %d mAh)", (double)e.avgMa, (int)g_cfg.batteryCapacityMah);
      } else {
        snprintf(v, sizeof v, "about %.0f mA (of %d mAh)", (double)e.avgMa, (int)g_cfg.batteryCapacityMah);
      }
    } else {
      snprintf(v, sizeof v, "being measured (%d mAh)", (int)g_cfg.batteryCapacityMah);
    }
  } else if (g_cfg.batteryCapacityMah > 0) {
    snprintf(v, sizeof v, "%d mAh battery", (int)g_cfg.batteryCapacityMah);
  } else {
    snprintf(v, sizeof v, "unknown: set battery_capacity_mah");
  }
  add("Current", v);
  {  // the power log, when it is on (and what became of its copy to an SD card at this start)
    char rows[2][powerlog::kStatusRow];
    const int count = powerlog::statusRows(powerLogStatus((uint32_t)(esp_timer_get_time() / 1000000)), rows);
    for (int i = 0; i < count; i++) add(i == 0 ? "Log" : "", rows[i]);
  }
  if (!g_cfg.hasBattery()) {
    snprintf(v, sizeof v, "off: no battery in use");
  } else if (!g_cfg.lowBatteryShutdown) {
    snprintf(v, sizeof v, "OFF (low_battery_shutdown = off)");
  } else if (s_guardOverride) {
    snprintf(v, sizeof v, "overridden for this run (KEY held)");
  } else {
    snprintf(v, sizeof v, "at %.2f V, back on at %.2f V", (double)g_cfg.batteryCutoffV, (double)lowbat::kRestartV);
  }
  add("Shutdown", v);
  // display_power = low: say so, and whether the panel is in it now.  (On a page with room for it and the three rows
  // below: on the fullest page, with the PSRAM warning and the log's two rows, it is the one left out.)
  if (g_cfg.displayPower == DISPPOWER_LOW && n < UI_INFO_LINES - 3) {
    add("Display", s_lcd && s_lcd->lowPower() ? "low power mode, 8 Hz refresh" : "low power mode chosen, not in it");
  }

  // (a line holds 42 characters of value)
  if (!g_cfg.wifi) {
    if (s.driftKnown) {
      snprintf(v, sizeof v, "no sync; last %+.1f ppm = %+.2f s/day", (double)s.driftPpm, (double)s.driftPpm * 0.0864);
    } else {
      snprintf(v, sizeof v, "no sync (WiFi off), drift not measured");
    }
  } else if (s.driftKnown) {
    snprintf(v, sizeof v, "%+.1f ppm = %+.2f s/day over %.0f h%s", (double)s.driftPpm, (double)s.driftPpm * 0.0864,
             (double)s.driftHours, s.driftLive ? "" : " (old)");
  } else {
    snprintf(v, sizeof v, "measuring: needs 3 h of hourly syncs");
  }
  add("Drift", v);

  char sum[96];
  cfgSummary(sum, sizeof sum);
  add("Config", sum);
  // (up to three of them, and fewer on a page that is nearly full: the last row is kept for the Units line)
  for (int i = 0; i < cfgIssueCount() && i < 3 && n < UI_INFO_LINES - 1; i++) {
    cfgIssueText(i, v, sizeof v);
    add("", v);
  }
  snprintf(v, sizeof v, "%s, %s, %s%s", g_cfg.imperial() ? "imperial" : "metric", g_cfg.time12h() ? "12 h" : "24 h",
           dateFormatName(g_cfg.dateFormat), g_cfg.showWeek ? ", weeks" : "");
  add("Units", v);
  m.infoCount = n;
}

// POWER LOG: the readings of the two pages above as plain values (power_log.h makes the line of CSV), so that
// the clock writes down by itself what would otherwise be copied off its screen by hand.
static void buildPowerLogReading(powerlog::Reading &r, uint32_t nowSec) {
  bool trusted;
  {
    StateLock lock;
    trusted = g_state.timeTrusted;
    r.spotify = g_state.spotify.status;
    r.radioSync = g_state.radioSync && g_cfg.wifi;  // (as the Radio line: only with wifi_mode = sync)
    r.radioOnPermille = g_state.radioOnPermille;
    r.radioSessions = g_state.radioSessions;
  }
  r.version = APP_VERSION;
  r.buildId = fwBuildId();
  r.settings = &g_cfg;
  const time_t now = time(nullptr);
  r.timeValid = trusted && now > kMinPlausibleEpoch;
  if (r.timeValid) localtime_r(&now, &r.local);
  r.uptimeSec = nowSec;
  r.cpuMhz = (int)getCpuFrequencyMhz();
  r.clockReason = g_cfg.cpuIdleMhz() > 0 ? s_clockReason : "";  // (as the Power line: a reason only with an idle clock)
  r.asleepPermille = s_sleepStats.permille(esp_timer_get_time());  // (written only with light_sleep = on, as the Sleep line)
  r.batteryPresent = s_batHave;
  r.batteryVolts = s_batVolts;
  r.charge = s_chargeState;
  r.chargeWarmingUp = s_charge.warmingUp();
  r.chargeReady = s_charge.ready();
  if (s_batHave) r.estimate = s_est.estimate(g_cfg.batteryCutoffV, (float)g_cfg.batteryCapacityMah);
  r.freeHeapBytes = ESP.getFreeHeap();
  r.framesSent = s_timing.frames;
  r.framesLate = s_timing.late;
  r.framesWorstMs = s_timing.worstLateUs / 1000;
}

// The power log: a line a few seconds after the start and every ten minutes after that (power_log.h).
// Writing one holds the loop up for a tenth of a second or a few while the flash is erased and written, so it
// is only begun early in a second, when the frame of the next one is not due for a long while.
static void servicePowerLog(int32_t usec) {
  if (usec > powerlog::kLatestStartUs) return;
  const uint32_t nowSec = (uint32_t)(esp_timer_get_time() / 1000000);  // (from the 64-bit timer, as the estimate's time is)
  if (!powerLogDue(nowSec)) return;
  powerlog::Reading r;
  buildPowerLogReading(r, nowSec);
  powerLogWrite(r, nowSec);
}

// The moon today and on the next two days at this time of day; the series costs a millisecond
// or two of floating point, so it is worked out once a minute.
static void moonForModel(UiModel &m, time_t shown, bool southern) {
  static int64_t s_minute = -1;
  static uint8_t s_lit[3];
  static bool s_waxing[3];
  const int64_t minute = (int64_t)shown / 60;
  if (minute != s_minute) {
    s_minute = minute;
    for (int i = 0; i < 3; i++) {
      const moon::Phase p = moon::phaseAt((int64_t)shown + (int64_t)i * 86400);
      s_lit[i] = (uint8_t)lroundf(p.lit * 100.0f);
      s_waxing[i] = p.waxing;
    }
  }
  m.moonValid = true;
  for (int i = 0; i < 3; i++) {
    m.moonLit[i] = s_lit[i];
    m.moonWaxing[i] = s_waxing[i];
  }
  m.southern = southern;
}

static void buildModel(UiModel &m, time_t shown, float frac) {
  const uint32_t nowMs = millis();
  m = UiModel();

  SharedState s;  // small copy of what the frame needs; taken under the lock
  {
    StateLock lock;
    s = g_state;
  }

  // time ---------------------------------------------------------------------
  m.timeValid = s.timeTrusted && shown > kMinPlausibleEpoch;
  if (m.timeValid) {
    localtime_r(&shown, &m.local);
    m.utcOffsetMin = timeutil::utcOffsetMinutes(shown);
    strftime(m.tzAbbrev, sizeof m.tzAbbrev, "%Z", &m.local);
    m.secFrac = frac;
    const double lat = s.locationKnown ? s.lat : g_cfg.latitude;
    moonForModel(m, shown, lat < 0);
  }

  // sensors ------------------------------------------------------------------
  m.indoorValid = s_indoor.valid && (nowMs - s_indoorMs) < 3UL * SENSOR_INTERVAL_MS;
  m.indoorC = s_indoor.tempC;
  m.indoorRh = s_indoor.rh;

  m.batPresent = s_batHave;
  m.batVolts = s_batVolts;
  m.batPercent = s_batHave ? calc::batteryPercent(s_batVolts) : 0;
  m.batCharge = s_batHave ? s_chargeState : CHARGE_UNKNOWN;
  m.batLow = s_batLow && s_chargeState != CHARGE_CHARGING;  // no warning while it is being charged
  m.batBlinkOn = (shown & 1) != 0;  // 1 s on, 1 s off

  // network & data -----------------------------------------------------------
  m.wifiUp = s.wifiUp;
  m.wifiOff = !g_cfg.wifi;
  m.rssi = s.rssi;
  // the place: what the settings call it, or what was found, or the city of the saved time zone;
  // "Earth" while nothing is known at all
  if (s.place[0]) {
    copyStr(m.location, sizeof m.location, s.place);
  } else if (s.tzName[0] && s.tzName[0] != '(' && strcmp(s.tzName, "UTC") != 0) {
    cityFromZone(s.tzName, m.location, sizeof m.location);
  } else {
    copyStr(m.location, sizeof m.location, "Earth");
  }
  copyStr(m.status, sizeof m.status, s.status);

  m.weather = s.weather;
  if (m.weather.valid) m.weather.ageSec = (nowMs - s.weatherFetchedMs) / 1000;

  m.spotify = s.spotify;
  if (m.spotify.status == SPOTIFY_PLAYING) {  // the poll is seconds old: extrapolate
    uint32_t p = m.spotify.progressMs + (nowMs - s.spotifySampledMs);
    m.spotify.progressMs = (m.spotify.durationMs && p > m.spotify.durationMs) ? m.spotify.durationMs : p;
  }

  // presentation -------------------------------------------------------------
  m.useFahrenheit = g_cfg.imperial();
  m.time12h = g_cfg.time12h();
  m.dateFormat = g_cfg.dateFormat;
  m.showWeek = g_cfg.showWeek;
  m.page = s_page;
  if ((int32_t)(s_toastUntilMs - nowMs) > 0) {
    copyStr(m.toast, sizeof m.toast, s_toast);
    m.toastIcon = s_toastIcon;
  }
  if (s_page == PAGE_INFO) buildInfoSystem(m, s, nowMs, shown);
  if (s_page == PAGE_POWER) buildInfoPower(m, s);
}

// Draws the frame for second `shown` into the display buffer (nothing reaches the panel yet).
static void prepareFrame(time_t shown, float frac, bool scheduled) {
  static UiModel model;
  const uint32_t t0 = micros();
  buildModel(model, shown, frac);
  u8g2_ClearBuffer(s_u8g2);
  uiDraw(s_u8g2, model);
  s_planner.drew(shown, micros() - t0, scheduled);
}

// How late a scheduled frame went out: the transfer should end DISPLAY_LATENCY_MS before its
// second starts.
static void recordLateness() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const int64_t afterStartUs = ((int64_t)tv.tv_sec - (int64_t)s_planner.preparedSec()) * 1000000 + tv.tv_usec;
  const int32_t lateUs = (int32_t)(afterStartUs + s_planner.latencyUs());
  s_timing.frames++;
  if (lateUs > 30000) {
    s_timing.late++;
    if (lateUs > 100000) {  // rare, and the next frame is a second away, so a slow serial port costs nothing
      LOGF(TAG, "late frame: %d ms (draw %u us, send %u us)", (int)(lateUs / 1000), (unsigned)s_planner.drawCostUs(),
           (unsigned)s_planner.sendCostUs());
    }
  }
  if (lateUs > s_timing.worstLateUs) s_timing.worstLateUs = lateUs;
}

// Sends the prepared frame to the panel.
static void flushFrame() {
  const uint32_t t0 = micros();
  u8g2_SendBuffer(s_u8g2);
  const uint32_t cost = micros() - t0;
  if (s_planner.preparedScheduled()) recordLateness();
  s_planner.sent(cost);
  s_lastFrameMs = millis();
  s_forceRedraw = false;
}

// Draw and send at once (start-up, button feedback, the sweeping second hand).
static void renderFrame(time_t shown, float frac) {
  prepareFrame(shown, frac, false);
  flushFrame();
}

// Notices when the system clock jumps against the monotonic timer (an NTP correction): a jump
// of a few hundred milliseconds would show as a second that is short, long or repeated.
static void watchClock(const struct timeval &tv) {
  static int64_t lastMono = 0, lastWall = 0;
  const int64_t mono = esp_timer_get_time();
  const int64_t wall = (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
  if (lastMono != 0) {
    int64_t step = (wall - lastWall) - (mono - lastMono);
    if (step > 5000 || step < -5000) {
      if (step > 2000000000LL) step = 2000000000LL;
      if (step < -2000000000LL) step = -2000000000LL;
      s_timing.clockSteps++;
      s_timing.lastStepUs = (int32_t)step;
      LOGF(TAG, "clock stepped by %+d ms", (int)(step / 1000));
    }
  }
  lastMono = mono;
  lastWall = wall;
}

// ---------------------------------------------------------------------------
// Battery voltage calibration: "batcal 4.20" says what a LiPo tester or a meter shows right now
// ---------------------------------------------------------------------------
// The median of the raw battery voltage over about two seconds (WiFi transmit bursts pull single
// readings down).  0 if there was no battery to read.
static float measureRawBatteryVolts() {
  float v[25];
  int n = 0;
  for (int i = 0; i < 25; i++) {
    const BatteryReading b = readBattery();
    if (b.present) v[n++] = b.rawVolts;
    delay(80);
  }
  if (n < 10) return 0;
  for (int i = 1; i < n; i++) {  // insertion sort: small n
    const float x = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static void calibrateBattery(const char *arg) {
  while (*arg == ' ') arg++;
  if (!g_cfg.hasBattery()) {
    Serial.println("battery = none in the settings: there is nothing to calibrate");
    return;
  }
  if (!*arg) {
    const BatteryReading b = readBattery();
    Serial.printf("battery: measured %.3f V, corrected %.3f V (battery_calibration = %g)\n", b.rawVolts, b.volts,
                  (double)g_cfg.batteryCalibration);
    Serial.println("tell the clock what a tester or meter shows right now:  batcal 4.20   (undo it:  batcal reset)");
    return;
  }
  char value[16];
  if (!strcmp(arg, "reset")) {
    value[0] = 0;  // an empty value forgets the setting: the default from config.h is back
  } else {
    const float truth = (float)atof(arg);
    Serial.println("measuring for two seconds, leave the clock alone ...");
    const float raw = measureRawBatteryVolts();
    float factor = 1.0f;
    if (raw <= 0) {
      Serial.println("no battery reading: is a battery fitted?  nothing changed");
      return;
    }
    if (!calc::calibrationFactor(truth, raw, &factor)) {
      Serial.printf("the clock measures %.3f V and you say %.3f V: that is not one LiPo cell, or the two are more than "
                    "20 %% apart (check the wiring and the figure); nothing changed\n",
                    raw, truth);
      return;
    }
    snprintf(value, sizeof value, "%.4f", (double)factor);
    Serial.printf("the clock measures %.3f V, the tester says %.3f V: ", raw, truth);
  }
  applySetting(g_cfg, cfgBuildDefaults(), "battery_calibration", value);
  const bool saved = cfgSaveToFlash();
  Serial.printf("battery_calibration = %g%s\n", (double)g_cfg.batteryCalibration, saved ? ", saved to flash" : ", NOT saved to flash");
  if (saved) {
    Serial.println("restarting to start from a clean state ...");
    delay(300);
    fwConfirmNow();
    ESP.restart();
  }
  s_batHave = false;  // not saved: it holds until the next restart, so forget what was learnt at the old scale
  s_lastBatMs = 0;
  s_charge.reset();
  s_est.reset();
}

// "set name = value": one setting, checked like a line of the SD card file, saved to flash, then a restart
// (most settings are read once at start-up).  "set name =" forgets it, so the built-in default is back.  It
// is also the way back from "console = off" without an SD card: restart with KEY held, then "set console = on".
static void setFromConsole(const char *arg) {
  while (*arg == ' ') arg++;
  char key[48];
  size_t n = 0;
  while (arg[n] && arg[n] != '=' && arg[n] != ':' && n + 1 < sizeof key) {
    key[n] = arg[n];
    n++;
  }
  key[n] = 0;
  if (!key[0] || (arg[n] != '=' && arg[n] != ':')) {
    Serial.println("usage:  set name = value     (the names are those of the SD card file, see \"config\";  set name =  forgets one)");
    return;
  }
  char err[96] = "";
  const ApplyResult r = applySetting(g_cfg, cfgBuildDefaults(), key, arg + n + 1, err, sizeof err);
  if (r == ApplyResult::UnknownKey || r == ApplyResult::BadValue) {
    Serial.printf("not set: %s\n", err);
    return;
  }
  if (r == ApplyResult::Unchanged) {
    Serial.println("that is what it is already: nothing changed");
    return;
  }
  if (!cfgSaveToFlash()) {
    Serial.println("could NOT save it to flash: it holds until the next restart only");
    return;
  }
  Serial.println(r == ApplyResult::Cleared ? "forgotten: the built-in default is back; restarting ..." : "saved to flash; restarting ...");
  delay(300);
  fwConfirmNow();  // a restart you asked for is no failure of a new firmware
  ESP.restart();
}

// ---------------------------------------------------------------------------
// Serial console (handy while developing): type "help"
// ---------------------------------------------------------------------------
static void serviceSerial() {
  static char buf[200];  // (a "set" line with the longest name and value fits)
  static size_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      buf[len] = 0;
      len = 0;
      if (!buf[0]) continue;
      if (!strcmp(buf, "reboot")) {
        fwConfirmNow();  // a restart you asked for is no failure of a new firmware
        ESP.restart();
      } else if (!strncmp(buf, "set ", 4)) {
        setFromConsole(buf + 4);
      } else if (!strncmp(buf, "batcal", 6)) {
        calibrateBattery(buf + 6);
      } else if (!strcmp(buf, "fw")) {
        fwPrint(Serial);
      } else if (!strcmp(buf, "rollback")) {  // go back to the firmware in the other app slot
        if (fwSwitchToOtherSlot()) {
          Serial.println("the other app slot is next: restarting into it");
          delay(200);
          fwConfirmNow();
          ESP.restart();
        }
        Serial.println("no firmware in the other app slot to go back to");
      } else if (!strcmp(buf, "fwforget")) {
        fwForgetLast();
        Serial.println("forgot the last build installed from the card: it may install the same file again");
      } else if (!strcmp(buf, "refresh")) {
        netRequestWeatherRefresh();
        spotifyPost(SPOTIFY_CMD_REFRESH);
        Serial.println("refreshing weather and Spotify");
      } else if (!strcmp(buf, "invert")) {
        handleBoot(CLICK_LONG);
      } else if (!strcmp(buf, "unlink")) {
        spotifyPost(SPOTIFY_CMD_UNLINK);
        Serial.println("Spotify account forgotten");
      } else if (!strncmp(buf, "page ", 5)) {
        s_page = (UiPage)(atoi(buf + 5) % PAGE_COUNT);
        s_forceRedraw = true;
      } else if (!strcmp(buf, "sleeptest")) {  // the low-battery sleep without a flat battery: KEY or five minutes wake it
        Serial.println("deep sleep with the battery-empty screen; KEY (or five minutes) wakes the clock");
        u8g2_ClearBuffer(s_u8g2);
        uiDrawBatteryEmpty(s_u8g2, s_batVolts, lowbat::kRestartV, true);
        u8g2_SendBuffer(s_u8g2);
        displayRest();
        powerDeepSleep(false);
      } else if (!strcmp(buf, "config")) {
        char sum[96];
        cfgSummary(sum, sizeof sum);
        Serial.printf("settings (%s):\n", sum);
        cfgPrint(Serial);
      } else if (!strcmp(buf, "timing")) {
        Serial.printf("frames %u sent, %u late (>30 ms), worst %d ms; draw %u us, send %u us; clock steps %u, last %+d ms\n",
                      (unsigned)s_timing.frames, (unsigned)s_timing.late, (int)(s_timing.worstLateUs / 1000),
                      (unsigned)s_planner.drawCostUs(), (unsigned)s_planner.sendCostUs(), (unsigned)s_timing.clockSteps,
                      (int)(s_timing.lastStepUs / 1000));
      } else if (!strncmp(buf, "powerlog", 8)) {  // the power log as CSV, or "powerlog clear"
        powerLogCommand(buf + 8, Serial);
      } else if (!strcmp(buf, "power")) {
        SharedState s;
        {
          StateLock lock;
          s = g_state;
        }
        Serial.printf("cpu %u MHz (working %d, idle %d: %s), %u clock changes; USB host %s, radio %s\n", (unsigned)getCpuFrequencyMhz(),
                      g_cfg.cpuMhz(), g_cfg.cpuIdleMhz(), g_cfg.cpuIdleMhz() > 0 ? (s_clockReason[0] ? s_clockReason : "-") : "no idle clock",
                      (unsigned)s_clockSwitches, usbHostAttached() ? "attached" : "no", netRadioNeedsFastClock() ? "needs the fast clock" : "off or idle");
        Serial.printf("wifi_mode %s%s: radio on %.1f %% of the time since start, %u sessions, next in %d s%s; frames draw %u us, send %u us\n",
                      g_cfg.syncMode() ? "sync" : "always", g_cfg.wifi ? "" : " (wifi off)", s.radioOnPermille / 10.0,
                      (unsigned)s.radioSessions, (int)s.radioWakeInSec,
                      s.radioTrouble == RADIO_AWAY ? " (none of the networks in range)" : (s.radioTrouble == RADIO_CANNOT_JOIN ? " (cannot join)" : ""),
                      (unsigned)s_planner.drawCostUs(), (unsigned)s_planner.sendCostUs());
        Serial.printf("console %s%s; buttons %s; audio chips: %s\n",
                      g_cfg.console == CONSOLE_ON ? "on" : (g_cfg.console == CONSOLE_AUTO ? "auto" : "off"),
                      s_consoleKeep ? " (kept on for this run: KEY was held at start-up)" : "",
                      buttonEdgesOn() ? "read by interrupt as well (idle clock or light sleep)" : "read by the loop",
                      s_audioStandby < 0 ? "left alone (AUDIO_CHIPS_STANDBY 0)"
                                         : (s_audioStandby == (AUDIO_CODEC_ANSWERED | AUDIO_MIC_ADC_ANSWERED) ? "both powered down" : "not both answered (see the start of the log)"));
        Serial.printf("display: %s; frames timed for %d ms on the way to the glass\n",
                      s_lcd && s_lcd->lowPower() ? "low power mode, 8 Hz (display_power = low)" : "high power mode, 25.5 Hz",
                      (int)(s_planner.latencyUs() / 1000));
        if (g_cfg.lightSleep) {  // (with the console on these stay at 0: the console keeps the chip awake)
          char row[lightsleep::kRow];
          lightsleep::formatRow(s_sleepStats.permille(esp_timer_get_time()), s_sleepWhy, row);
          Serial.printf("light sleep: %s; %u naps, %u ended by a button, %u refused; time of day put right %u times, by up to %d us\n",
                        row, (unsigned)s_sleepStats.naps(), (unsigned)s_sleepStats.buttonWakes(), (unsigned)s_sleepStats.refused(),
                        (unsigned)s_wallFix.fixes(), (int)s_wallFix.largestUs());
        } else {
          Serial.println("light sleep: off (light_sleep = off)");
        }
        char rows[2][powerlog::kStatusRow];
        const int count = powerlog::statusRows(powerLogStatus((uint32_t)(esp_timer_get_time() / 1000000)), rows);
        uint32_t lastMs, worstMs;  // what a line costs in time: the flash is busy for most of it
        powerLogTimings(&lastMs, &worstMs);
        Serial.printf("power log: %s%s%s", count > 0 ? rows[0] : "off", count > 1 ? "; " : "", count > 1 ? rows[1] : "");
        if (worstMs > 0) Serial.printf("; the last line took %u ms to write, the slowest %u ms", (unsigned)lastMs, (unsigned)worstMs);
        Serial.println();
      } else if (!strcmp(buf, "battery")) {
        char est[64];
        estimateText(est, sizeof est);
        Serial.printf("battery %s: %.3f V %d%%  %s  trend %+.2f mV/min  step %+.1f mV  %.1f min of data%s\n  left: %s\n",
                      s_batHave ? "present" : "absent", s_batVolts, calc::batteryPercent(s_batVolts),
                      chargeName(s_chargeState), s_charge.slopeMvPerMin(), s_charge.stepMv(), s_charge.minutesOfData(),
                      s_charge.warmingUp() ? " (starting up: the first 3 minutes are not used)"
                                           : (s_charge.ready() ? "" : " (still learning)"),
                      est);
      } else if (!strcmp(buf, "status")) {
        SharedState s;
        {
          StateLock lock;
          s = g_state;
        }
        Serial.printf("wifi=%d rssi=%d ip=%s synced=%d zone=%s status='%s' spotify=%d '%s'\n", s.wifiUp, s.rssi,
                      s.ip, s.ntpSynced, s.tzPosix, s.status, (int)s.spotify.status, s.spotify.message);
      } else {
        Serial.println("commands: status | battery | batcal V | power | powerlog | powerlog clear | config | set name = value | timing | fw | rollback | fwforget | refresh | page N | invert | unlink | sleeptest | reboot");
      }
    } else if (len < sizeof buf - 1) {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------------------
static bool initDisplay() {
  if (s_lcd) return true;
  s_lcd = new ST7305_U8g2(PIN_LCD_SCK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST);
  if (!s_lcd->begin(0, U8G2_R1)) {
    LOGF(TAG, "display buffer allocation failed");
    for (;;) delay(1000);
  }
  s_u8g2 = s_lcd->getU8g2()->getU8g2();
  s_uiPrefs.begin("ui", false);
  s_flipPolarity = s_uiPrefs.getBool("flip", false);
  applyPolarity();
  return true;
}

// First thing after the settings are read, before the display, WiFi or anything else draws power:
// is there enough battery to start?  After a low-battery shutdown the answer stays no until the
// cell is back above lowbat::kRestartV (charging does that); a timer wake-up looks and goes back
// to sleep without touching the display.  KEY held for 3 s on the "battery empty" screen runs the
// clock anyway.
static void lowBatteryGate() {
  if (!g_cfg.hasBattery() || !g_cfg.lowBatteryShutdown) {
    powerForgetLowShutdown();
    return;
  }
  const BatteryReading b = readBattery();
  if (!b.present) return;
  const bool timerWake = powerWokeFromTimer();
  if (lowbat::bootDecision(true, b.volts, g_cfg.batteryCutoffV, powerWasLowShutdown()) == lowbat::BootAction::RUN) {
    powerForgetLowShutdown();
    return;
  }
  LOGF(TAG, "battery %.2f V: too low to start (%s)", b.volts, timerWake ? "timer check" : "button or power-up");
  if (!timerWake) {
    initDisplay();
    applyDisplayPower();
    u8g2_ClearBuffer(s_u8g2);
    uiDrawBatteryEmpty(s_u8g2, b.volts, lowbat::kRestartV, true);
    u8g2_SendBuffer(s_u8g2);
    const uint32_t start = millis();
    uint32_t heldSince = 0;
    while (millis() - start < 8000) {  // eight seconds in which KEY can say "start anyway"
      if (digitalRead(PIN_KEY) == LOW) {
        if (!heldSince) heldSince = millis();
        if (millis() - heldSince >= 3000) {
          LOGF(TAG, "KEY held: starting despite the low battery");
          s_guardOverride = true;
          powerForgetLowShutdown();
          return;
        }
      } else {
        heldSince = 0;
      }
      delay(20);
    }
  }
  displayRest();  // (nothing on a timer check: the display was not started, and it still rests from before)
  powerDeepSleep(true);
}

// The CPU clock.  The display driver holds the SPI bus for good and the core waits for every SPI bus when
// the clock changes, so the driver lets go for the moment.  A frame takes longer to draw at a lower clock,
// so the frame planner is told (the first frame after the switch is planned with the new costs).
static void applyClock(int mhz) {
  const int from = (int)getCpuFrequencyMhz();
  if (from == mhz || mhz == s_clockRefusedMhz) return;
  if (s_lcd) s_lcd->busRelease();
  powerSetCpuMhz(mhz);
  if (s_lcd) s_lcd->busAcquire();
  const int now = (int)getCpuFrequencyMhz();  // what it is now: the core refuses a clock it cannot make
  if (now == from) {
    s_clockRefusedMhz = mhz;  // (asked once: the loop would otherwise try again every few milliseconds)
    LOGF(TAG, "the CPU clock stays at %d MHz: %d MHz was refused", from, mhz);
    return;
  }
  s_planner.clockChanged((uint32_t)from, (uint32_t)now);
  s_clockSwitches++;
}

// The working clock from the settings (at start-up, and after the settings are read).
static void applyCpuSetting() { applyClock(g_cfg.cpuMhz()); }

// A restart someone asked for with KEY (held for five seconds, then let go).  The SD card is only read while
// the clock starts, so this is how a card that was put in while it ran gets read, without the battery coming out.
static void restartOnRequest() {
  LOGF(TAG, "KEY was held for %d s: restarting (the SD card is read at start-up)", (int)(KEY_RESTART_HOLD_MS / 1000));
  showToast(UI_TOAST_RESTARTING, TOAST_NONE, 10000);
  renderFrame(time(nullptr), 0);  // on the glass before the chip goes down
  delay(800);                     // (long enough to read, and to press KEY again for "keep the console")
  fwConfirmNow();  // a restart you asked for is no failure of a new firmware
  ESP.restart();
}

// Is a computer on the USB port?  (Below 80 MHz the USB console may stop, which would look like a crash.)
// Always "no" once the console was shut down: the USB transceiver is off then, and there is no console to keep.
static bool usbHostAttached() {
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  return g_consoleOn && HWCDC::isPlugged();
#else
  return false;
#endif
}

// The console: its commands while it runs, and the moment it is shut down for good ("console = auto" when no
// computer is on the USB port, "console = off" at the end of start-up; console_policy.h).
static void serviceConsole(uint32_t nowMs) {
  if (!g_consoleOn) return;
  // (with "console = on", or KEY held at start-up, the port is not even looked at: it runs as it always has)
  if (g_cfg.console != CONSOLE_ON && !s_consoleKeep && s_consoleWatch.shouldStop(g_cfg.console, false, usbHostAttached(), nowMs)) {
    LOGF(TAG, "console = %s: shutting the console and the USB port down until the next restart (KEY held at start-up keeps them)",
         g_cfg.console == CONSOLE_AUTO ? "auto, and no computer on the USB port" : "off");
    consoleShutDown();
    return;
  }
  serviceSerial();
}

// cpu_idle_mhz: the clock follows what needs it (clock_policy.h).  Called every time round the loop.
static void serviceClock(uint32_t nowMs, int32_t usec) {
  if (s_pokeUntilMs != 0 && (int32_t)(s_pokeUntilMs - nowMs) <= 0) s_pokeUntilMs = 0;  // over (a time long past must not come round again)
  const int idle = g_cfg.cpuIdleMhz();
  if (idle == 0) return;  // no idle clock: the working clock stays (start-up has set it)
  static uint32_t s_usbCheckMs = 0;
  static bool s_usb = false;
  if (s_usbCheckMs == 0 || nowMs - s_usbCheckMs >= 500) {
    s_usbCheckMs = nowMs ? nowMs : 1;
    s_usb = usbHostAttached();
  }
  clockpolicy::Needs needs;
  needs.radio = netRadioNeedsFastClock();
  needs.usbHost = s_usb;
  needs.poked = s_pokeUntilMs != 0;
  const int want = clockpolicy::target(needs, g_cfg.cpuMhz(), idle);
  s_clockReason = want == idle ? "idle" : (needs.radio ? "radio on" : (needs.usbHost ? "USB attached" : "button"));
  if (clockpolicy::mayChange((int)getCpuFrequencyMhz(), want, s_planner.quiet(usec))) applyClock(want);
}

// Is slow work due (readSlowSensors(), servicePowerLog()), and when can it run: 0 = now, lightsleep::kNever = none is due.
// Each kind has its part of the second, and each part begins at the tick.
static int32_t slowWorkInUs(uint32_t nowMs, int32_t usec) {
  const int32_t toTick = 1000000 - usec;
  int32_t in = lightsleep::kNever;
  const bool sensors = s_lastSensorMs == 0 || nowMs - s_lastSensorMs >= SENSOR_INTERVAL_MS;
  const bool battery = s_lastBatMs == 0 || nowMs - s_lastBatMs >= BATTERY_INTERVAL_MS;
  if (sensors || battery) in = s_planner.quiet(usec) ? 0 : toTick;
  if (powerLogDue((uint32_t)(esp_timer_get_time() / 1000000))) {
    const int32_t at = usec <= powerlog::kLatestStartUs ? 0 : toTick;
    if (at < in) in = at;
  }
  return in;
}

// light_sleep = on: a light sleep until the next thing is due, in place of a nap, when nothing needs the chip awake
// (light_sleep.h).  False when it did not sleep: the caller naps as before.
static bool lightSleepStep() {
  if (!g_cfg.lightSleep) return false;
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint32_t nowMs = millis();
  s_sleepStats.begin(esp_timer_get_time());
  lightsleep::Inputs in;
  in.enabled = true;
  in.radio = netRadioNeedsFastClock();
  in.console = g_consoleOn;
  // (a change the interrupts recorded while the loop was busy counts too: pollButtons() takes it next time round)
  in.button = digitalRead(PIN_KEY) == LOW || digitalRead(PIN_BOOT) == LOW || s_key.isDown() || s_boot.isDown() || buttonEdgesPending();
  in.poked = s_pokeUntilMs != 0;
  in.busy = s_forceRedraw || s_restartAsked || s_cubeOn;
  in.backOff = s_sleepRefusedMs != 0 && (uint32_t)(nowMs - s_sleepRefusedMs) < lightsleep::kRetryMs;
  if (!in.backOff) s_sleepRefusedMs = 0;  // (over: a time long past must not come round again)
  in.frameInUs = s_planner.idleUs(tv.tv_sec, (int32_t)tv.tv_usec);
  in.workInUs = slowWorkInUs(nowMs, (int32_t)tv.tv_usec);
  if (s_toast[0]) {  // the banner comes down then
    const int32_t ms = (int32_t)(s_toastUntilMs - nowMs);
    in.otherInUs = ms <= 0 ? 0 : (ms >= lightsleep::kMaxSleepUs / 1000 ? lightsleep::kMaxSleepUs : ms * 1000);
  }
  const lightsleep::Decision d = lightsleep::decide(in);
  s_sleepWhy = d.why;
  if (!d.sleep) return false;
  const lightsleep::Nap nap = powerLightSleep(d.us, s_wallFix);
  s_sleepStats.add(nap);
  if (nap.err != 0) {
    s_sleepRefusedMs = millis() | 1u;
    s_sleepWhy = lightsleep::REFUSED;
  }
  return nap.slept;
}

// A short note about what the SD card did, if anything, or about a built-in default that was refused
// (a toast holds 39 characters).
static void announceConfig() {
  char toast[40];
  // What a card's settings changed at the start before this one, if that start could not say so: it installed
  // a firmware from the card and restarted, and this start finds the same file with nothing left to change.
  const int carried = g_cfgStatus.carried;
  switch (g_cfgStatus.sd) {
    case CfgStatus::SD_FILE_APPLIED: {
      const ConfigReport &r = g_cfgStatus.report;
      uint32_t trialLeft;
      const bool justInstalled = fwOnTrial(millis(), &trialLeft);
      const bool warn = formatSdBanner(r, g_cfgStatus.savedToFlash, carried, justInstalled, toast, sizeof toast);
      const bool lost = r.touched() > 0 && !g_cfgStatus.savedToFlash;  // in force now, gone after the next restart
      showToast(toast, warn ? TOAST_WARN : TOAST_NONE, lost ? 8000 : 6000);
      break;
    }
    case CfgStatus::SD_EXAMPLE_WRITTEN: showToast("Wrote settings file to SD card", TOAST_NONE, 6000); break;
    case CfgStatus::SD_WRITE_FAILED: showToast("SD card: cannot write", TOAST_WARN, 6000); break;
    case CfgStatus::SD_READ_FAILED: showToast("SD card: cannot read file", TOAST_WARN, 6000); break;
    case CfgStatus::SD_UNREADABLE: showToast(sdProblemText(), TOAST_WARN, 10000); break;  // what the card holds instead of FAT
    default:  // no card: still say so when something in config.h / secrets.h was refused (the Info page has the details)
      if (cfgBuildProblems() > 0) {
        snprintf(toast, sizeof toast, "%d built-in default%s not used", cfgBuildProblems(), cfgBuildProblems() == 1 ? "" : "s");
        showToast(toast, TOAST_WARN, 8000);
      } else if (carried > 0) {  // the card came out during the restart: what it changed is still worth a line
        formatSdBanner(ConfigReport(), true, carried, false, toast, sizeof toast);
        showToast(toast, TOAST_NONE, 6000);
      }
      break;
  }
  cfgAnnounced();  // the count from the last start is for this banner only, whatever it said
}

// The cube that KEY and BOOT bring up when they are held down together for ten seconds: a frame of it, as often
// as the loop comes round, until a button is held (cubeButton) or nobody has touched one for two minutes.
// Nothing of the clock is drawn meanwhile; the battery is still read, because a gap in its readings would start
// the charge detector afresh.
static void cubeFrame(uint32_t nowMs) {
  if ((uint32_t)(nowMs - s_cubeTouchMs) >= kCubeIdleMs) {
    cubeEnd();
    return;
  }
  s_pokeUntilMs = (nowMs + clockpolicy::kPokeMs) | 1u;  // at the working clock, as after a button press (cpu_idle_mhz)
  const float dtMs = (float)(uint32_t)(nowMs - s_cubeFrameMs);
  s_cubeFrameMs = nowMs;
  s_cube.turnMs += dtMs;
  s_cubeShove.advance(dtMs);
  for (int i = 0; i < 3; i++) s_cube.shove[i] = s_cubeShove.angle[i];
  s_cube.elapsedMs = nowMs - s_cubeStartMs;
  u8g2_ClearBuffer(s_u8g2);
  uiDrawCube(s_u8g2, s_cube);
  u8g2_SendBuffer(s_u8g2);
  readSlowSensors(nowMs);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  consoleBegin();
  delay(100);
  powerAfterWake();
  fwBegin();  // is this firmware on trial (just installed from the SD card)?
  pinMode(PIN_KEY, INPUT_PULLUP);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  delay(2);                                      // (the pull-up needs a moment)
  s_consoleKeep = digitalRead(PIN_KEY) == LOW;   // KEY held at start-up: the console stays on for this run (console_policy.h)

  cfgLoadFlash();        // the defaults, then what the clock saved in its flash
  applyCpuSetting();     // 80 MHz unless the settings say otherwise: a clock needs no more
  lowBatteryGate();      // may put the clock straight back to sleep
  LOGF(TAG, APP_NAME " " APP_VERSION " (build %s) starting, %u MHz%s", fwBuildId(), (unsigned)getCpuFrequencyMhz(), psramFound() ? ", PSRAM on" : "");

  stateInit();
#if PIN_CHARGE_STATUS >= 0
  pinMode(PIN_CHARGE_STATUS, INPUT_PULLUP);  // the charger's STAT output pulls it low while charging
#endif

  // The time first (the RTC chip is on the I2C bus), so a file the clock writes to an SD card gets
  // the right date.
  bool shtc3 = sensorsBegin();
  LOGF(TAG, "SHTC3 %s", shtc3 ? "ok" : "not responding");
#if AUDIO_CHIPS_STANDBY
  // the audio chips share the I2C bus and are not used: power them down, and give their I2S lines a level
  s_audioStandby = audioChipsStandby();
  powerQuietAudioPins();
  LOGF(TAG, "audio chips powered down: codec %s, microphone ADC %s", (s_audioStandby & AUDIO_CODEC_ANSWERED) ? "ok" : "NO ANSWER",
       (s_audioStandby & AUDIO_MIC_ADC_ANSWERED) ? "ok" : "NO ANSWER");
#endif
  time_t rtc;
  if (rtcReadUtc(&rtc)) {  // show a plausible time straight away, before WiFi is up
    struct timeval tv = {rtc, 0};
    settimeofday(&tv, nullptr);
    {
      StateLock lock;
      g_state.timeTrusted = true;
    }
    LOGF(TAG, "clock set from RTC (%ld)", (long)rtc);
  } else {
    LOGF(TAG, "RTC not set; %s", g_cfg.wifi ? "waiting for NTP" : "WiFi is off, so it stays unset until WiFi is turned on once");
  }

  cfgImportSdCard();  // a settings file on an SD card, applied and saved to flash
  applyCpuSetting();  // ... which may have changed the clock speed
  // the buttons by interrupt too: a frame at the idle clock can outlast a tap, and in light sleep the press that wakes
  // the chip is recorded the moment it is awake (button_edges.h)
  if (g_cfg.cpuIdleMhz() > 0 || g_cfg.lightSleep) buttonEdgesBegin();
  initDisplay();
  applyDisplayPower();  // display_power, as the settings have it now (an SD card may have changed it)
  fwUpdateFromCard(showFirmwareScreen);  // a firmware file on the card: install it and restart (else carry on)
  powerLogBegin();                       // power_log = on: the flash partition the log is kept in
  s_guard.configure(g_cfg.lowBatteryShutdown && g_cfg.hasBattery() && !s_guardOverride, g_cfg.batteryCutoffV);
  announceConfig();
  char summary[96];
  cfgSummary(summary, sizeof summary);
  LOGF(TAG, "settings: %s", summary);
  if (s_consoleKeep && g_cfg.console != CONSOLE_ON) {
    LOGF(TAG, "KEY was held at start-up: the console stays on for this run (console = %s)", g_cfg.console == CONSOLE_AUTO ? "auto" : "off");
    if (!s_toast[0]) showToast("Console kept on for this run", TOAST_NONE, 4000);  // (what the SD card did comes first)
  }

  netLoadSettings();
  readSlowSensors(millis());
  renderFrame(time(nullptr), 0);
  netBegin();
}

void loop() {
  const uint32_t nowMs = millis();
  pollButtons(nowMs);
  if (s_restartAsked) restartOnRequest();
  serviceConsole(nowMs);
  fwTrialTick(nowMs);  // a new firmware is kept once it has run for a minute

  // toasts raised by the network task ("No active Spotify device", ...)
  {
    StateLock lock;
    if (g_state.noticeSeq != s_seenNoticeSeq) {
      s_seenNoticeSeq = g_state.noticeSeq;
      char text[sizeof g_state.notice];
      copyStr(text, sizeof text, g_state.notice);
      int icon = g_state.noticeIcon;
      showToast(text, icon);
    }
  }
  if (s_toast[0] && (int32_t)(s_toastUntilMs - nowMs) <= 0) {  // toast expired: repaint without it
    s_toast[0] = 0;
    s_forceRedraw = true;
  }

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  watchClock(tv);
  spotifySetPageShown(s_page == PAGE_NOW_PLAYING);  // wifi_mode = sync: the Now Playing page keeps the radio on
  serviceClock(nowMs, (int32_t)tv.tv_usec);
  if (s_cubeOn) {  // (see cubeFrame)
    cubeFrame(nowMs);
    return;
  }

#if CLOCK_SWEEP_FPS > 0
  // A sweeping second hand: frames at a steady rate, nothing scheduled.
  if (s_forceRedraw || (uint32_t)(nowMs - s_lastFrameMs) >= 1000UL / CLOCK_SWEEP_FPS) {
    renderFrame(tv.tv_sec, tv.tv_usec / 1e6f);
  } else {
    if (1000000 - (int32_t)tv.tv_usec > 300000) readSlowSensors(nowMs);
    servicePowerLog((int32_t)tv.tv_usec);
    delay(4);
  }
#else
  const FramePlanner::Step step = s_planner.next(tv.tv_sec, (int32_t)tv.tv_usec, s_forceRedraw);
  switch (step.action) {
    case FramePlanner::DRAW_NOW:
      renderFrame(step.sec, 0);
      break;
    case FramePlanner::PREPARE:
      prepareFrame(step.sec, 0, true);
      break;
    case FramePlanner::SEND:
      if (step.waitUs > 0) delayMicroseconds((uint32_t)step.waitUs);
      flushFrame();
      break;
    case FramePlanner::SLEEP:
      if (!lightSleepStep()) delay(step.waitUs > 12000 ? 4 : 1);  // short naps, so the buttons keep being polled
      break;
    default:
      if (s_planner.quiet((int32_t)tv.tv_usec)) readSlowSensors(nowMs);  // never close to a tick
      servicePowerLog((int32_t)tv.tv_usec);                               // (nor this: only early in a second)
      if (!lightSleepStep()) delay(4);
      break;
  }
#endif
}
