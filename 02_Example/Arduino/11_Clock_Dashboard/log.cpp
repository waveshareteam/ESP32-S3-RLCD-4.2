#include "log.h"

#include <atomic>

#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
#include <driver/gpio.h>
#include <hal/usb_serial_jtag_ll.h>  // Serial is the chip's USB Serial/JTAG port
#include <soc/gpio_struct.h>
#include <soc/io_mux_reg.h>          // the two USB pins
#endif

volatile bool g_consoleOn = true;

namespace {
std::atomic<int> s_printing{0};  // tasks that are inside a LOGF right now
std::atomic<bool> s_hold{false};  // LOGF lines are dropped for the moment (consoleHold)
}

void consoleBegin() {
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // A console that was shut down left the two USB pins as plain outputs held low (that is how Serial.end()
  // takes the port off the bus), and a restart that is not a power-on puts no pin back: the restart by KEY,
  // the "set" command, a firmware update.  So let go of them before the port is started.  After a power-on
  // they are not outputs and nothing is touched.
  for (int pin : {USB_INT_PHY0_DM_GPIO_NUM, USB_INT_PHY0_DP_GPIO_NUM}) {
    if ((GPIO.enable >> pin) & 1u) gpio_set_direction((gpio_num_t)pin, GPIO_MODE_DISABLE);
  }
#endif
  Serial.begin(115200);  // (on the USB port this also switches the transceiver and its pull-up on)
#if ARDUINO_USB_CDC_ON_BOOT
  // Don't stall for the default 100 ms when no USB host is reading.  Not 0: with a
  // zero timeout HWCDC::write() can spin forever if the host stops draining data.
  Serial.setTxTimeoutMs(10);
#endif
}

bool logBegin() {
  s_printing.fetch_add(1);
  if (g_consoleOn && !s_hold.load()) return true;
  s_printing.fetch_sub(1);
  return false;
}

void logEnd() { s_printing.fetch_sub(1); }

void consoleHold(bool hold) {
  s_hold.store(hold);
  if (hold) {
    for (int i = 0; i < 200 && s_printing.load() > 0; i++) delay(5);  // the line another task is in the middle of
  }
}

void consoleShutDown() {
  if (!g_consoleOn) return;
  Serial.flush();
  g_consoleOn = false;  // no new lines from here on ...
  for (int i = 0; i < 200 && s_printing.load() > 0; i++) delay(5);  // ... and the one another task is in the middle of gets to finish

  // Serial.end() stops the driver: the interrupt is freed, the buffers go, and on the USB port the core turns
  // D+ and D- into plain pins held low, which also takes the USB pad and its pull-up away from the USB block.
  Serial.end();
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // Said outright as well: the transceiver off.  (The block's bus clock is left running: the SDK's check for a
  // computer on the port keeps reading its registers from the tick interrupt.)
  usb_serial_jtag_ll_phy_enable_pad(false);
#endif
}
