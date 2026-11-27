#include "app_state.h"

#include <stdarg.h>
#include <sys/time.h>

#include "util.h"

SharedState g_state;
static SemaphoreHandle_t s_mutex = nullptr;

void stateInit() {
  if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

StateLock::StateLock() {
  if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
}

StateLock::~StateLock() {
  if (s_mutex) xSemaphoreGive(s_mutex);
}

void stateSetStatus(const char *fmt, ...) {
  char buf[sizeof(g_state.status)];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  StateLock lock;
  copyStr(g_state.status, sizeof g_state.status, buf);
}

void stateNotice(const char *text, int icon) {
  StateLock lock;
  copyStr(g_state.notice, sizeof g_state.notice, text);
  g_state.noticeIcon = icon;
  g_state.noticeSeq++;
}

void waitForQuietPhase() {
  for (int i = 0; i < 100; i++) {  // 100 x 10 ms: gives up after a second
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    if (tv.tv_usec > 200000 && tv.tv_usec < 600000) return;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
