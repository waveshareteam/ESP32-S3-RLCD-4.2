#pragma once

// A very small test harness shared by the host-side tests: CHECK macros that count and
// print failures, and a section() heading.  Each test program defines its own main().

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

[[maybe_unused]] static int g_checks = 0, g_failed = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    g_checks++;                                                              \
    if (!(cond)) {                                                           \
      g_failed++;                                                            \
      printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);               \
    }                                                                        \
  } while (0)

#define CHECK_STR(a, b)                                                                 \
  do {                                                                                  \
    g_checks++;                                                                         \
    if (strcmp((a), (b)) != 0) {                                                        \
      g_failed++;                                                                       \
      printf("  FAIL %s:%d  \"%s\" != \"%s\"\n", __FILE__, __LINE__, (a), (b));         \
    }                                                                                   \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                              \
  do {                                                                                     \
    g_checks++;                                                                            \
    if (fabs((double)(a) - (double)(b)) > (tol)) {                                         \
      g_failed++;                                                                          \
      printf("  FAIL %s:%d  %g vs %g\n", __FILE__, __LINE__, (double)(a), (double)(b));    \
    }                                                                                      \
  } while (0)

[[maybe_unused]] static std::string readFile(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    printf("  cannot open %s\n", path);
    g_failed++;
    return "";
  }
  std::string s;
  char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) s.append(buf, n);
  fclose(f);
  return s;
}

[[maybe_unused]] static void section(const char *name) { printf("[%s]\n", name); }
