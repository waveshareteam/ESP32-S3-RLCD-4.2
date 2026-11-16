#pragma once

// JSON parsing for the Spotify Web API and accounts service.  Pure functions
// (ArduinoJson only), covered by the host-side tests in tools/tests.

#include <stddef.h>

#include "app_model.h"

// Parses a 200 reply of GET /v1/me/player into `out` (status becomes PLAYING or
// PAUSED).  Copies the active device id (used to resume after the device went
// idle) into deviceId.  Returns false if the JSON is malformed.
bool parsePlayerState(const char *json, size_t len, SpotifyInfo *out, char *deviceId,
                      size_t deviceIdCap);

struct SpotifyTokenReply {
  char accessToken[512] = "";
  char refreshToken[256] = "";  // empty when the server did not rotate it
  int expiresInSec = 0;
};

// Parses a successful reply of POST accounts.spotify.com/api/token.
bool parseTokenReply(const char *json, size_t len, SpotifyTokenReply *out);

// Extracts a short machine-readable reason and a human message from an error
// body.  Handles both shapes Spotify uses:
//   {"error":"invalid_grant","error_description":"Refresh token revoked"}
//   {"error":{"status":404,"message":"Player command failed: No active device found",
//             "reason":"NO_ACTIVE_DEVICE"}}
void parseSpotifyError(const char *json, size_t len, char *reason, size_t reasonCap, char *message,
                       size_t messageCap);

// Builds the accounts.spotify.com/authorize URL for the PKCE flow (all values are
// percent-encoded).  Returns the length, or 0 if `cap` is too small.
size_t buildAuthorizeUrl(const char *clientId, const char *redirectUri, const char *scopes,
                         const char *codeChallenge, const char *state, char *out, size_t cap);
