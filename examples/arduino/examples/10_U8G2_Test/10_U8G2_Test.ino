#include "ST7305_U8g2.h"

#define LCD_WIDTH 400
#define LCD_HEIGHT 300

#define RLCD_SCK_PIN 11
#define RLCD_MOSI_PIN 12
#define RLCD_DC_PIN 5
#define RLCD_CS_PIN 40
#define RLCD_RST_PIN 41

static ST7305_U8g2 lcd(RLCD_SCK_PIN, RLCD_MOSI_PIN, RLCD_DC_PIN, RLCD_CS_PIN, RLCD_RST_PIN);
static U8G2 *u8g2 = nullptr;

static uint32_t counter = 0;
static uint32_t frames = 0;
static uint32_t last_report_frames = 0;
static uint32_t last_report_ms = 0;
static uint32_t fps_x100 = 0;
static uint32_t frame_us = 0;
static uint32_t flush_us = 0;

static void drawCenteredStr(int y, const char *text)
{
  int text_width = u8g2->getStrWidth(text);
  int x = (LCD_WIDTH - text_width) / 2;
  if (x < 0) {
    x = 0;
  }
  u8g2->drawStr(x, y, text);
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  lcd.begin(0, U8G2_R1);
  u8g2 = lcd.getU8g2();

  last_report_ms = millis();
  Serial.println("ST7305 U8g2 counter demo started");
}

void loop()
{
  char text[80];
  uint32_t frame_start_us = micros();

  u8g2->clearBuffer();
  u8g2->setDrawColor(1);

  snprintf(text, sizeof(text), "%lu", (unsigned long)counter);
  u8g2->setFont(u8g2_font_logisoso50_tn);
  int number_height = u8g2->getAscent() - u8g2->getDescent();
  int number_y = ((LCD_HEIGHT - number_height) / 2) + u8g2->getAscent();
  drawCenteredStr(number_y, text);

  u8g2->drawFrame(10, 10, 380, 280);
  u8g2->drawHLine(18, 260, 364);

  u8g2->setFont(u8g2_font_6x13_tf);
  drawCenteredStr(number_y + 26, "ST7305 refresh counter");
  snprintf(text, sizeof(text), "FPS:%lu.%02lu  frame:%lums  flush:%lums",
           (unsigned long)(fps_x100 / 100),
           (unsigned long)(fps_x100 % 100),
           (unsigned long)(frame_us / 1000),
           (unsigned long)(flush_us / 1000));
  u8g2->drawStr(20, 282, text);

  uint32_t flush_start_us = micros();
  u8g2->sendBuffer();
  uint32_t now_us = micros();

  flush_us = now_us - flush_start_us;
  frame_us = now_us - frame_start_us;
  counter++;
  frames++;

  uint32_t now_ms = millis();
  uint32_t elapsed_ms = now_ms - last_report_ms;
  if (elapsed_ms >= 1000) {
    uint32_t delta_frames = frames - last_report_frames;
    fps_x100 = (uint32_t)((uint64_t)delta_frames * 100000ULL / elapsed_ms);
    Serial.printf("counter=%lu fps=%lu.%02lu frame=%lu us flush=%lu us\r\n",
                  (unsigned long)counter,
                  (unsigned long)(fps_x100 / 100),
                  (unsigned long)(fps_x100 % 100),
                  (unsigned long)frame_us,
                  (unsigned long)flush_us);
    last_report_ms = now_ms;
    last_report_frames = frames;
  }

  if ((frames % 16) == 0) {
    delay(1);
  } else {
    yield();
  }
}
