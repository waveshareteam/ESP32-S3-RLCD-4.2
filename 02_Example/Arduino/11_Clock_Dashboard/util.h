#pragma once

// Small string helpers (no Arduino dependency).

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 204 / 205 / 304 replies carry no body and no Content-Length.  Arduino's
// HTTPClient::getString() would wait for the connection to close on those, so
// callers must check this first.
inline bool httpStatusHasBody(int code) { return code >= 200 && code != 204 && code != 205 && code != 304; }

// Bounded copy that always NUL-terminates.
inline void copyStr(char *dst, size_t cap, const char *src) {
  if (cap == 0) return;
  size_t n = src ? strlen(src) : 0;
  if (n >= cap) n = cap - 1;
  if (n) memcpy(dst, src, n);
  dst[n] = 0;
}

// base64url without padding (RFC 4648 section 5), as PKCE requires.
// Returns the encoded length, or 0 if `cap` was too small.
inline size_t base64UrlEncode(const uint8_t *in, size_t len, char *out, size_t cap) {
  static const char *tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  size_t need = (len * 4 + 2) / 3;
  if (cap < need + 1) return 0;
  size_t o = 0;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < len) v |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < len) v |= in[i + 2];
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    if (i + 1 < len) out[o++] = tbl[(v >> 6) & 63];
    if (i + 2 < len) out[o++] = tbl[v & 63];
  }
  out[o] = 0;
  return o;
}

// Percent-encodes everything except RFC 3986 unreserved characters.
// Returns the encoded length, or 0 if `cap` was too small.
inline size_t urlEncode(const char *src, char *dst, size_t cap) {
  static const char *hex = "0123456789ABCDEF";
  size_t n = 0;
  for (const unsigned char *p = (const unsigned char *)src; *p; ++p) {
    bool keep = (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') ||
                *p == '-' || *p == '_' || *p == '.' || *p == '~';
    if (keep) {
      if (n + 1 >= cap) return 0;
      dst[n++] = (char)*p;
    } else {
      if (n + 3 >= cap) return 0;
      dst[n++] = '%';
      dst[n++] = hex[*p >> 4];
      dst[n++] = hex[*p & 15];
    }
  }
  if (n >= cap) return 0;
  dst[n] = 0;
  return n;
}

// Decodes %XX and '+' in place.
inline void urlDecodeInPlace(char *s) {
  char *w = s;
  for (const char *r = s; *r; ++r) {
    if (*r == '%' && r[1] && r[2]) {
      auto hv = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
      };
      int hi = hv(r[1]), lo = hv(r[2]);
      if (hi >= 0 && lo >= 0) {
        *w++ = (char)(hi * 16 + lo);
        r += 2;
        continue;
      }
    }
    *w++ = (*r == '+') ? ' ' : *r;
  }
  *w = 0;
}

// Finds `key` in a query string ("a=1&b=2" or a full URL containing one) and
// copies the decoded value into out.  Returns false when absent.
inline bool queryParam(const char *text, const char *key, char *out, size_t cap) {
  const char *q = strchr(text, '?');
  const char *p = q ? q + 1 : text;
  size_t klen = strlen(key);
  while (*p) {
    const char *end = p;
    while (*end && *end != '&' && *end != '#') ++end;
    if ((size_t)(end - p) > klen && strncmp(p, key, klen) == 0 && p[klen] == '=') {
      size_t vlen = (size_t)(end - (p + klen + 1));
      if (vlen >= cap) vlen = cap - 1;
      memcpy(out, p + klen + 1, vlen);
      out[vlen] = 0;
      urlDecodeInPlace(out);
      return true;
    }
    if (*end == '#') break;
    p = *end ? end + 1 : end;
  }
  return false;
}
