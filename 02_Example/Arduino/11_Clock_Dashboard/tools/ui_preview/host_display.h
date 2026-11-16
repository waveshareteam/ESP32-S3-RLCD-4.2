#pragma once

// Host-side stand-in for the ST7305 panel: a 300x400 1-bit page buffer set up
// exactly like firmware (full-buffer mode, U8G2_R1 -> 400x300 landscape).
// Lets ui.cpp be rendered on a PC and dumped to an image.

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <clib/u8g2.h>

static const int kPhysW = 300;   // panel native width
static const int kPhysH = 400;   // panel native height
static const int kTileW = 38;    // buffer width in 8-px tiles (304 px)
static const int kTileH = 50;    // buffer height in 8-px tiles (400 px)

static uint8_t hostDisplayCb(u8x8_t *u8x8, uint8_t msg, uint8_t, void *) {
  static const u8x8_display_info_t info = {
      /* chip_enable_level */ 0, /* chip_disable_level */ 1,
      /* post_chip_enable_wait_ns */ 0, /* pre_chip_disable_wait_ns */ 0,
      /* reset_pulse_width_ms */ 0, /* post_reset_wait_ms */ 0,
      /* sda_setup_time_ns */ 0, /* sck_pulse_width_ns */ 0,
      /* sck_clock_hz */ 0, /* spi_mode */ 0, /* i2c_bus_clock_100kHz */ 4,
      /* data_setup_time_ns */ 0, /* write_pulse_width_ns */ 0,
      /* tile_width */ kTileW, /* tile_height */ kTileH,
      /* default_x_offset */ 0, /* flip_mode_x_offset */ 0,
      /* pixel_width */ kPhysW, /* pixel_height */ kPhysH};
  if (msg == U8X8_MSG_DISPLAY_SETUP_MEMORY) {
    u8x8_d_helper_display_setup_memory(u8x8, &info);
  }
  return 1;
}

static uint8_t hostByteCb(u8x8_t *, uint8_t, uint8_t, void *) { return 1; }
static uint8_t hostGpioCb(u8x8_t *, uint8_t, uint8_t, void *) { return 1; }

static uint8_t g_hostBuf[kTileW * 8 * kTileH];

static u8g2_t *hostDisplayInit() {
  static u8g2_t u8g2;
  u8g2_SetupDisplay(&u8g2, hostDisplayCb, u8x8_cad_empty, hostByteCb, hostGpioCb);
  u8g2_SetupBuffer(&u8g2, g_hostBuf, kTileH, u8g2_ll_hvline_vertical_top_lsb, U8G2_R1);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  return &u8g2;
}

// Writes the *logical* 400x300 image as a binary PGM-like file: "P5\n400 300\n255\n"
// with 0 = black ink and 255 = white paper.  `invert` = 1 treats set pixels as ink.
static void hostDumpPgm(const char *path) {
  // Physical pixel (px,py): byte [(py/8)*304 + px], bit (py%8), LSB = top.
  // U8G2_R1 maps logical (lx,ly) to physical (px = kPhysW-1-ly, py = lx).
  FILE *f = fopen(path, "wb");
  if (!f) return;
  fprintf(f, "P5\n400 300\n255\n");
  for (int ly = 0; ly < 300; ly++) {
    for (int lx = 0; lx < 400; lx++) {
      int px = kPhysW - 1 - ly;
      int py = lx;
      int on = (g_hostBuf[(py / 8) * (kTileW * 8) + px] >> (py % 8)) & 1;
      fputc(on ? 0 : 255, f);
    }
  }
  fclose(f);
}
