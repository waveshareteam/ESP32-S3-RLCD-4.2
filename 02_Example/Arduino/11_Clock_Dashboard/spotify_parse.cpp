#include "spotify_parse.h"

#include <ArduinoJson.h>
#include <string.h>

#include "util.h"

bool parsePlayerState(const char *json, size_t len, SpotifyInfo *out, char *deviceId,
                      size_t deviceIdCap) {
  // The reply can run to several KB; keep only the fields we show.
  JsonDocument filter;
  filter["is_playing"] = true;
  filter["progress_ms"] = true;
  filter["shuffle_state"] = true;
  filter["currently_playing_type"] = true;
  filter["device"]["id"] = true;
  filter["device"]["name"] = true;
  filter["device"]["volume_percent"] = true;
  filter["item"]["name"] = true;
  filter["item"]["duration_ms"] = true;
  filter["item"]["artists"][0]["name"] = true;
  filter["item"]["album"]["name"] = true;
  filter["item"]["show"]["name"] = true;

  JsonDocument doc;
  if (deserializeJson(doc, json, len, DeserializationOption::Filter(filter))) return false;

  SpotifyInfo s = *out;  // keep fields owned by the caller (message, linkUrl, ...)
  s.status = (doc["is_playing"] | false) ? SPOTIFY_PLAYING : SPOTIFY_PAUSED;
  s.progressMs = doc["progress_ms"] | 0u;
  s.shuffle = doc["shuffle_state"] | false;
  s.title[0] = s.artist[0] = s.album[0] = 0;
  s.durationMs = 0;

  const char *type = doc["currently_playing_type"] | "";
  JsonObjectConst item = doc["item"];
  if (item.isNull()) {
    // Ads and some unsupported content have no item.
    copyStr(s.title, sizeof s.title, strcmp(type, "ad") == 0 ? "Advertisement" : "Unknown");
  } else {
    copyStr(s.title, sizeof s.title, item["name"] | "");
    s.durationMs = item["duration_ms"] | 0u;

    JsonArrayConst artists = item["artists"];
    if (!artists.isNull() && artists.size() > 0) {
      size_t n = 0;
      for (JsonVariantConst a : artists) {
        const char *name = a["name"] | "";
        if (!*name) continue;
        size_t need = strlen(name) + (n ? 2 : 0);
        if (n + need + 1 > sizeof s.artist) break;
        if (n) {
          s.artist[n++] = ',';
          s.artist[n++] = ' ';
        }
        memcpy(s.artist + n, name, strlen(name));
        n += strlen(name);
        s.artist[n] = 0;
      }
    } else {
      // Podcast episode: show the show's name where the artist would be.
      copyStr(s.artist, sizeof s.artist, item["show"]["name"] | "");
    }
    copyStr(s.album, sizeof s.album, item["album"]["name"] | "");
  }
  if (!s.title[0]) copyStr(s.title, sizeof s.title, "Unknown");

  copyStr(s.device, sizeof s.device, doc["device"]["name"] | "");
  s.volume = doc["device"]["volume_percent"].isNull() ? -1 : (int8_t)(doc["device"]["volume_percent"] | -1);
  if (deviceId && deviceIdCap) copyStr(deviceId, deviceIdCap, doc["device"]["id"] | "");

  *out = s;
  return true;
}

bool parseTokenReply(const char *json, size_t len, SpotifyTokenReply *out) {
  JsonDocument doc;
  if (deserializeJson(doc, json, len)) return false;
  const char *access = doc["access_token"] | "";
  if (!*access || strlen(access) >= sizeof out->accessToken) return false;
  SpotifyTokenReply r;
  copyStr(r.accessToken, sizeof r.accessToken, access);
  const char *refresh = doc["refresh_token"] | "";
  if (strlen(refresh) >= sizeof r.refreshToken) return false;
  copyStr(r.refreshToken, sizeof r.refreshToken, refresh);
  r.expiresInSec = doc["expires_in"] | 3600;
  *out = r;
  return true;
}

void parseSpotifyError(const char *json, size_t len, char *reason, size_t reasonCap, char *message,
                       size_t messageCap) {
  if (reasonCap) reason[0] = 0;
  if (messageCap) message[0] = 0;
  JsonDocument doc;
  if (!json || !len || deserializeJson(doc, json, len)) return;
  JsonVariantConst err = doc["error"];
  if (err.is<const char *>()) {  // accounts service
    copyStr(reason, reasonCap, err.as<const char *>());
    copyStr(message, messageCap, doc["error_description"] | "");
  } else if (err.is<JsonObjectConst>()) {  // Web API
    copyStr(reason, reasonCap, err["reason"] | "");
    copyStr(message, messageCap, err["message"] | "");
  }
}

size_t buildAuthorizeUrl(const char *clientId, const char *redirectUri, const char *scopes,
                         const char *codeChallenge, const char *state, char *out, size_t cap) {
  char id[96], redirect[200], scope[240], challenge[96], st[96];
  if (!urlEncode(clientId, id, sizeof id) || !urlEncode(redirectUri, redirect, sizeof redirect) ||
      !urlEncode(scopes, scope, sizeof scope) || !urlEncode(codeChallenge, challenge, sizeof challenge) ||
      !urlEncode(state, st, sizeof st)) {
    return 0;
  }
  int n = snprintf(out, cap,
                   "https://accounts.spotify.com/authorize?client_id=%s&response_type=code&redirect_uri=%s"
                   "&scope=%s&code_challenge_method=S256&code_challenge=%s&state=%s",
                   id, redirect, scope, challenge, st);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}
