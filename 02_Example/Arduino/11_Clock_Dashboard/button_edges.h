#pragma once

// The buttons by interrupt, for the idle clock (cpu_idle_mhz) and for light sleep (light_sleep).
//
// The UI loop reads the two buttons between frames.  At the working clock a frame takes some 30 ms to draw,
// shorter than any tap.  At 20 MHz it takes 120 ms, and a quick tap can begin and end inside that, unseen.
// So with an idle clock set, every change of the two pins is also recorded by an interrupt, with its time,
// and the loop replays the record to its click detectors (ClickDetector::edge) before it looks at the pins
// itself.  Without an idle clock or light sleep nothing here is used and the buttons are read as they always were.
//
// In light sleep (light_sleep.h) the two pins are what wakes the chip, by a low level.  The SDK sets that level as
// the pins' interrupt type as well, and a level interrupt would call the handler without end while a button is
// held, so their interrupts are held off for the nap (buttonEdgesSleep(true)) and put back after it
// (buttonEdgesSleep(false)), when a button that is down by then is recorded as pressed at that moment: the press
// that woke the chip, whose own change came while nothing could record it.

#include <Arduino.h>

enum ButtonId : uint8_t { BUTTON_KEY = 0, BUTTON_BOOT = 1 };

void buttonEdgesBegin();  // start recording (the pins are inputs with pull-ups already)
void buttonEdgesEnd();    // stop (before deep sleep)
bool buttonEdgesOn();

// true: KEY and BOOT wake the chip from light sleep (a low level), their interrupts off.  false: as they were, and
// a button that is down now recorded as pressed now.  Call it from the task that called buttonEdgesBegin().
void buttonEdgesSleep(bool asleep);

// A change of either button is recorded and not yet taken (a tap that came and went while the loop was busy): the
// loop has to answer it before it may sleep.
bool buttonEdgesPending();

// The oldest change of that button not yet taken: the level it changed to and when (millis()).
// False when there is none.  Call it from the task that called buttonEdgesBegin().
bool buttonEdgeTake(ButtonId button, bool *down, uint32_t *atMs);
