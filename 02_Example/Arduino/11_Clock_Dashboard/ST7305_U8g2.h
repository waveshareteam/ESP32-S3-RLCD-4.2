#pragma once

// ST7305 300x400 reflective LCD driver for U8g2 (full-buffer mode).
//
// Derived from Waveshare's 10_U8G2_Test example.  Changes: the pixel polarity
// can be inverted (so U8g2 "colour 1" can mean black ink), begin() reports
// allocation failure, the panel can be switched to the controller's low power
// mode (display_power.h), and the SPI clock / pin handling is unchanged.

#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

class ST7305_U8g2 {
private:
  int _sck;
  int _mosi;
  int _dc;
  int _cs;
  int _rst;
  SPIClass *_spi = nullptr;
  U8G2 u8g2_wrapper;
  uint8_t *_my_buf = nullptr;
  volatile bool _invert = false;
  bool _busHeld = false;  // begin() starts one long SPI transaction, see busRelease()
  bool _lowPower = false;  // the panel is driven in the low power mode (fullInit() leaves it in the high one)

  void _cmd(uint8_t cmd);
  void _data(const uint8_t *data, size_t len);
  void _cmd_data(uint8_t cmd, const uint8_t *data, size_t len);
  void _sourceVoltages();

  static uint8_t u8x8_d_st7305_custom(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
  static uint8_t u8x8_byte_custom(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

public:
  ST7305_U8g2(int sck = 11, int mosi = 12, int dc = 5, int cs = 40, int rst = 41);
  ~ST7305_U8g2();

  // tile_buf_height 0 means full-buffer mode (about 15 KB of RAM for 300x400).
  // Returns false if the frame buffer could not be allocated.
  bool begin(uint8_t tile_buf_height = 0, const u8g2_cb_t *rotation = U8G2_R0);
  void reset();
  void fullInit();

  // Flip the panel polarity: with invert = true a set U8g2 pixel is sent as 0.
  // Takes effect on the next sendBuffer().
  void setInvert(bool invert) { _invert = invert; }
  bool invert() const { return _invert; }

  // The controller's low power mode (8 Hz refresh) or its high power mode (25.5 Hz, what fullInit() leaves), by the
  // datasheet's sequence: 120 ms into the low one, 320 ms out of it, spent in delay().  Nothing is sent when the
  // panel is in that mode already.  A frame shows up later in the low one: see display_power.h.
  void setLowPower(bool low);
  bool lowPower() const { return _lowPower; }

  // For the deep-sleep shutdown: the low power mode at its slowest refresh, 0.25 Hz (display_power.h), with the
  // picture left as it is.  The rate is written in the high power mode, as fullInit() writes it, so it takes up to
  // 440 ms.  Lasts until fullInit() runs again.
  void setRestMode();

  // begin() opens one SPI transaction and keeps it open, which holds the SPI peripheral's lock
  // for good and keeps every transfer cheap.  The lock is not recursive, so whatever else needs
  // it from the same task has to let go first: changing the CPU clock (the core tells every SPI
  // bus about it and waits for the lock), or re-routing the SPI pins.  Pairs of
  //     busRelease(); ...; busAcquire();
  void busRelease();
  void busAcquire();

  U8G2 *getU8g2() { return &u8g2_wrapper; }
};
