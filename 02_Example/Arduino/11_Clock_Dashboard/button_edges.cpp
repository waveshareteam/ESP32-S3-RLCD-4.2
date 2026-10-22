#include "button_edges.h"

#include <driver/gpio.h>

#include "config.h"

namespace {

// One writer (the interrupt) and one reader (the loop), both on the core that attached the interrupt.
const uint8_t kCap = 16;  // a power of two; a press and its release with their bounces fit several times over
struct EdgeLog {
  volatile uint8_t head = 0;  // the interrupt writes here
  volatile uint8_t tail = 0;  // the loop reads here
  uint32_t ms[kCap];
  bool down[kCap];
};
EdgeLog s_log[2];
bool s_on = false;

void ARDUINO_ISR_ATTR record(EdgeLog &q, uint8_t pin) {
  const uint8_t next = (uint8_t)((q.head + 1) & (kCap - 1));
  if (next == q.tail) return;  // full (a long burst of bounces): the loop's own look at the pin puts it right
  q.ms[q.head] = millis();
  q.down[q.head] = digitalRead(pin) == LOW;
  q.head = next;
}

void ARDUINO_ISR_ATTR onKeyChange() { record(s_log[BUTTON_KEY], PIN_KEY); }
void ARDUINO_ISR_ATTR onBootChange() { record(s_log[BUTTON_BOOT], PIN_BOOT); }

}  // namespace

void buttonEdgesBegin() {
  if (s_on) return;
  attachInterrupt(PIN_KEY, onKeyChange, CHANGE);
  attachInterrupt(PIN_BOOT, onBootChange, CHANGE);
  s_on = true;
}

void buttonEdgesEnd() {
  if (!s_on) return;
  detachInterrupt(PIN_KEY);
  detachInterrupt(PIN_BOOT);
  s_on = false;
}

bool buttonEdgesOn() { return s_on; }

bool buttonEdgesPending() { return s_log[BUTTON_KEY].head != s_log[BUTTON_KEY].tail || s_log[BUTTON_BOOT].head != s_log[BUTTON_BOOT].tail; }

void buttonEdgesSleep(bool asleep) {
  const uint8_t pins[2] = {PIN_KEY, PIN_BOOT};
  for (int b = 0; b < 2; b++) {
    const gpio_num_t pin = (gpio_num_t)pins[b];
    if (asleep) {
      gpio_intr_disable(pin);                         // first, so the level type below cannot call the handler
      gpio_wakeup_enable(pin, GPIO_INTR_LOW_LEVEL);  // (it sets the pin's interrupt type to that level too)
    } else {
      gpio_wakeup_disable(pin);
      gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);  // what attachInterrupt(CHANGE) set
      if (s_on) {
        if (digitalRead(pins[b]) == LOW) record(s_log[b], pins[b]);  // down already: the press that woke the chip
        gpio_intr_enable(pin);
      }
    }
  }
}

bool buttonEdgeTake(ButtonId button, bool *down, uint32_t *atMs) {
  EdgeLog &q = s_log[button];
  const uint8_t t = q.tail;
  if (t == q.head) return false;
  *down = q.down[t];
  *atMs = q.ms[t];
  q.tail = (uint8_t)((t + 1) & (kCap - 1));
  return true;
}
