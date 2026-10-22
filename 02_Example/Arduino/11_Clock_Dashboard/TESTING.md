# Testing on the board

What has been written and tested on a PC but still has to be seen on the real board, as a checklist. The README's
[Not yet tried on the board](README.md#not-yet-tried-on-the-board) says, item by item, how each one is tested on the PC.

Each item has a status: **seen** (it works on the board), **partly** (some of it seen), **not yet**, or **problem**
(seen, and wrong). "The log" means the clock's own power log (`power_log = on`) of 8 to 17 October 2026: firmware 1.7
to 1.9 with `wifi_mode = sync`, `wifi_power_save = max`, `cpu_idle_mhz = 20`, `spotify_live = off`, `console = auto`.

Seen before this list was made: a card with a GUID partition table refused and a FAT32 card mounted; settings from the
card applied and kept in flash; firmware installed from the card; the processor at 20 MHz with the pages still drawn;
the analogue clock's outline (1.9) and both cubes with their buttons (1.8, 1.9).

## SD card

1. **Not yet.** The example settings file is written to a card that has none.
2. **Not yet.** Start-up with no card in the slot: any noticeable delay.

## Firmware update from the card

3. **Partly.** The trial minute after an install: the countdown and the confirmation. In the log, 1.8 and 1.9 each
   came up about two minutes after the restart that installed them, and stayed through later restarts, so the build was
   confirmed. The countdown itself has not been reported.
4. **Not yet.** The installed file is renamed on the card, so it is not installed twice.
5. **Not yet.** A rollback (only if a build ever fails its trial minute).
6. **Not yet.** The banner after an update says what the settings file did (for example *SD settings: 4 in force*).
7. **Seen.** The build id: the log's `firmware` column carries it (`1.9 5067f83`, the id of the build that was handed
   over), so the call that reads it works.

## Battery and charging

8. **Partly.** No *full* at 99 % on battery, and an estimate with %/h about 50 minutes after a start on battery: the
   estimate came at 50 minutes in every run. But on 15 October the clock said *full* for about six hours while it was on
   battery (the cell stood at 4.20 V after a charge), and showed no estimate meanwhile.
9. **Seen.** The charging and full indicators: *charging* within ten minutes of plugging in, *full* one to one and
   three-quarter hours later, *discharging* within ten minutes of unplugging (9 and 14 October).
10. **Seen.** The reading on the charger: 4.282 to 4.286 V for two days (9 to 11 October), against 4.19 to 4.22 V for
    a full cell off the charger. Above about 4.25 V the clock is on the charger (with this board's calibration).
11. **Partly.** How close the estimate comes. Just after a charge it said 8 to 16 days; the cell went from 4.20 V to
    3.71 V in about 57 hours. The voltage barely moves for the first 12 hours after a charge (4.20 to 4.15 V took 12.8
    hours, the next 50 mV took 5.8), and the battery curve counts that stretch as 5 %, so the estimate takes the slow
    start for the rate of the whole cell. 1.10 replaces the curve with the run in 12; replayed through the estimator
    on a PC, from the third hour off the charger on, 72 of 73 hourly figures were within 15 % of the time really
    left, and in the first three hours the figure is short (never long). Still to be seen on the board.
12. **Partly.** A run down to the shutdown, 17 to 20 October (firmware 1.9, the settings above): 81 hours from a full
    charge to 3.30 V, a line every ten minutes to the last (3.307 V). It gave the battery curve of 1.10. **Seen:** the
    low-battery blink, then the battery-empty screen at the shutdown. Not reported yet: the wake by timer and by KEY
    while it sleeps, and the 3-second override. What the board draws while the clock sleeps has not been measured
    (`sleeptest`).
13. **Partly.** The five battery runs (stock on WiFi, stock without WiFi, a middle ground, 20 MHz on WiFi, 20 MHz
    without WiFi). Only the fourth has been logged (the settings above): 81 hours, 3.4 days, from full to the
    shutdown.
14. **Seen.** Where the cell really is at 3.7 V: in the run in 12 it still had 39 % at 3.70 V, where the old curve
    said 12 %. The good-quality 3400 mAh cell lasted 3.4 days, not the five that the USB reading in 24 suggested: on
    the battery the clock drew roughly 40 mA from the cell (3400 mAh in 81 hours), where 105 mW from USB would make
    about 28 mA. Why is not known.

## Power saving

15. **Seen.** `wifi_mode = sync` over days: the radio on 1.7 to 3.9 % of the time, about five sessions an hour.
16. **Partly.** Late frames: 7 in three and a half days, the worst 307 ms. One frame came **3.7 s** late, on
    9 October between 06:07 and 06:18; the cause is not known.
17. **Partly.** Readings at 20 MHz: the battery voltage is steady and plausible. The indoor sensor is not in the log.
18. **Not yet.** Buttons at 20 MHz: no lost taps or double clicks.
19. **Partly.** Sync mode with the network gone: on 13 October from about 06:40 to 20:10 no session connected, while
    the radio still came on about 2.6 % of the time (looking for the network), and sessions came back by themselves.
    Was the clock away from its network then, or was the network down?
20. **Not yet.** `spotify_live = off`: the strip around the end of a track, and the *Now Playing* page waking the
    radio.
21. **Not yet.** `console = auto` and `off`: a computer noticed on USB, the KEY-held start that keeps the console, the
    `set` command. (`auto` has run for days without trouble.)
22. **Not yet.** A `cpu_mhz` change from the settings file: the start does not hang.
23. **Not yet.** The audio chips switched off: no noise, and the difference on a meter.
24. **Partly.** The current, by a USB meter (5 V) with the battery out and the settings above, console off: 21.1 to
    21.4 mA at the 20 MHz idle clock (an inverted screen makes no difference worth counting), about 29 mA at 80 MHz
    after a button press, 33 to 36 mA at 80 MHz with the radio on. With the radio on about 2 % of the time, the idle
    clock is nearly all of the energy. Not yet: the current during `sleeptest` (the board without the processor).

## Power log

25. **Not yet.** The *Log* row on *Power and settings* (size, lines this run, age).
26. **Seen.** The copy to the card as `ESP32-S3-RLCD-PowerLog.csv` at start, with the whole log in it. The file's date
    on the card is in UTC (8 hours early in Perth), and 1 January 1980 when the clock did not know the time yet.
27. **Not yet.** `powerlog`, `powerlog clear`, and the write time that `power` reports in the serial console.
28. **Seen.** A line every ten minutes for days, without the seconds stuttering (see 16).
29. **Seen.** The note that says why the clock started: `start (restart)`, `start (power on)`.

## Everything else

30. **Not yet.** The restart by KEY (held five seconds, then let go): the banner, the restart, the card read after it.
31. **Not yet.** The backup network: the switch to it when the main one cannot be joined, and back.
32. **Not yet.** The clock drift figure after real NTP syncs.
33. **Not yet.** Problem rows on *Power and settings* for a bad line in the settings file or a refused built-in default.
34. **Not yet.** The Spotify linking flow.
35. **Partly.** The newer screens on the real panel for contrast and legibility: the legend, the firmware update
    screen, the weather band with its moon and rain.
36. **Partly.** The cubes: whether the frame rate looks smooth.

## Light sleep (1.11)

37. **Not yet.** `light_sleep = on`, with the settings of 24 (`wifi_mode = sync`, `cpu_idle_mhz = 20`, `console` auto
    or off). Worth a run of its own in the log (the sheet's sixth line).
    * The meter on USB (5 V, battery out), with it on and with it off, ten seconds after the last button press: the
      estimate is about 15 mA (14 to 16) against the 21 mA of 24. The *Sleep* row of *Power and settings* should say
      some 75 to 80 % at 20 MHz once the radio has been off for a while (it counts from the start).
    * The *Frames* line after a day: no more late frames than in 16 (7 in three and a half days, the worst 307 ms).
    * Buttons: every press of KEY and BOOT answered as before, single, double and long, also the first press after a
      quiet minute (that one wakes the chip); KEY held for five seconds still restarts the clock.
    * The time overnight: the clock against a phone in the evening and in the morning, and the *Drift* line on
      *Power and settings* after a day of hourly syncs, against what it says with light sleep off. (With WiFi off
      nothing corrects the clock: try that only once the drift with sync mode is known.)
    * The low-battery shutdown at the end of a run with light sleep (12) still wakes by KEY and by the five-minute
      timer, and not by BOOT. (`sleeptest` cannot show it: the console it needs keeps the chip from light sleep.)

## The display's low power mode (1.11)

38. **Not yet.** `display_power = low`, with the settings of 37 (or of 24 if light sleep is off). Worth a run of its
    own in the log (the sheet's seventh line).
    * The meter on USB (5 V, battery out), ten seconds after the last button press, with `display_power = low` and with
      `normal`, nothing else changed (each is a restart): the difference is what it saves. The guess is about 1 mA;
      nothing published says more, so anything from nothing to 2 mA would not be a surprise. The *Power and settings*
      page should have a *Display* row saying `low power mode, 8 Hz refresh`.
    * The picture at 8 refreshes a second next to 25.5: contrast and legibility in daylight and in lamp light, and
      from the side; no flicker or shimmer, no smearing of the seconds, no ghost of the digits before.
    * The seconds against a reference (a phone's clock with seconds, or a radio time signal): they should change within
      about a twentieth of a second of it, sometimes a little before and sometimes a little after, drifting slowly
      between the two; never clearly late.
    * Buttons and page changes still feel immediate (up to 125 ms instead of 39 before they show).
    * The cubes: they run in the high power mode, so they should look as smooth as with `normal`; starting one takes
      about a third of a second longer, and afterwards the *Display* row is back.
    * After a day: the *Frames* line, no more late frames than without it.

## The display at rest during the shutdown (1.12)

39. **Not yet.** Before every deep sleep the panel goes to its slowest refresh, 0.25 Hz, whatever `display_power` says.
    `sleeptest` in the serial console shows it without a flat battery (KEY or five minutes wake it); the next run down
    to the shutdown shows it as it is meant.
    * The *Battery empty* screen at that rate: still readable, no fading or flicker between the refreshes, no ghost
      once the clock starts again.
    * It stays up through the five-minutely looks at the battery (they should leave the panel alone), over a night.
    * KEY: the screen comes back at the usual rate for its eight seconds, and the clock that starts (after charging,
      or with KEY held for three seconds) looks as before.
    * If there is a meter in the battery's lead: the sleeping current, with this and (`sleeptest` on 1.11) without it.
