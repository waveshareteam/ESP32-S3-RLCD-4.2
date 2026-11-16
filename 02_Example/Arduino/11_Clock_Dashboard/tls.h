#pragma once

// TLS helper: verify servers against the Mozilla root CA bundle that ships
// inside the ESP32 Arduino core (the same list a browser trusts), so there is no
// need for setInsecure() or for pinning certificates that expire every month.
// The system clock must be roughly right for certificate validity checks.

#include <WiFiClientSecure.h>

extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t rootca_crt_bundle_end[] asm("_binary_x509_crt_bundle_end");

inline void useCaBundle(WiFiClientSecure &client) {
  client.setCACertBundle(rootca_crt_bundle_start, rootca_crt_bundle_end - rootca_crt_bundle_start);
  client.setHandshakeTimeout(10);  // seconds
}
