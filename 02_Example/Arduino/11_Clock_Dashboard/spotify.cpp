#include "spotify.h"

#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_random.h>
#include <freertos/queue.h>
#include <mbedtls/sha256.h>
#include <time.h>

#include "app_settings.h"
#include "app_state.h"
#include "config.h"
#include "log.h"
#include "spotify_parse.h"
#include "spotify_helper_script.h"
#include "link_page.h"
#include "radio_plan.h"
#include "timeutil.h"
#include "tls.h"
#include "util.h"

// How this works, in short:
//  * Linking uses OAuth "authorization code with PKCE", so no client secret is
//    stored on the device.  While no account is linked the device serves a small
//    page (http://<ip>/) with an authorize link; Spotify redirects the browser to
//    http://127.0.0.1:8888/callback?code=... (nothing listens there, which is
//    fine), and the user pastes that address back into the page.
//  * The refresh token lives in NVS.  Spotify expires it after six months, so the
//    time of linking is stored too and a countdown is shown.
//  * Polling is deliberately gentle: Spotify's development-mode quota is
//    unpublished and people have been locked out for hours by 3-second polling.
//    Progress is extrapolated between polls and 429 / Retry-After is honoured
//    across reboots.

namespace {

const char *const TAG = "spotify";
const char *const kApiBase = "https://api.spotify.com/v1";
const char *const kTokenUrl = "https://accounts.spotify.com/api/token";
const char *const kAuthorizeUrl = "https://accounts.spotify.com/authorize";
const char *const kRedirectUri = "http://127.0.0.1:8888/callback";
const char *const kScopes = "user-read-playback-state user-modify-playback-state";

const uint32_t kRefreshTokenLifetimeS = 180UL * 86400UL;  // Spotify: refresh tokens last 6 months
const int kRenewHintDays = 14;
const time_t kMinPlausibleEpoch = 1704067200;  // 2024-01-01
const int kIdlePollsBeforeSlowdown = 60;       // ~30 minutes at the idle rate
const uint32_t kPausedHoldMs = 5UL * 60UL * 1000UL;  // wifi_mode = sync: the radio stays on this long after music was playing or a command was sent
const uint32_t kPageHoldMs = 60UL * 1000UL;          // ... and this long after the Now Playing page was last on screen

struct Reply {
  int code = -1;  // HTTP status, or <= 0 for a transport error
  String body;
  int retryAfterSec = 0;
};

// ---- credentials & persistent state -----------------------------------------
Preferences s_prefs;
char s_refreshToken[256] = "";
char s_accessToken[512] = "";
uint32_t s_accessValidUntilMs = 0;  // millis() deadline; 0 = no valid access token
uint32_t s_authEpoch = 0;           // when the user linked (UTC seconds)
uint32_t s_blockedUntilEpoch = 0;   // rate-limit lockout (UTC seconds)
char s_deviceId[48] = "";           // last device seen playing, used to wake an idle one
bool s_linked = false;

// ---- runtime state (network task only) --------------------------------------
SpotifyInfo s_info;
uint32_t s_sampledMs = 0;
uint32_t s_nextPollMs = 0;
bool s_peekDone = false;                   // wifi_mode = sync: this radio session has looked at the player once
uint32_t s_lastActiveMs = 0;               // when the player was last seen playing, or a command was sent
volatile uint32_t s_pageHoldUntilMs = 0;   // the UI task shows the Now Playing page: the radio stays on until then (0 = not)
uint32_t s_trackLookAtMs = 0;              // spotify_live = off: when the track on screen should be over, time for the next look (0 = none)
int s_idlePolls = 0;
int s_errorStreak = 0;
int s_authFailures = 0;
uint32_t s_lastBlockMsgMs = 0;
QueueHandle_t s_queue = nullptr;

// ---- HTTP: one long-lived TLS connection to the API for keep-alive ----------
NetworkClientSecure s_apiClient;
HTTPClient s_apiHttp;

// ---- linking page -----------------------------------------------------------
WebServer *s_web = nullptr;
char s_verifier[65] = "";
char s_state[17] = "";
char s_pageMessage[160] = "";
bool s_pageMessageIsError = false;

time_t nowEpoch() {
  time_t t = time(nullptr);
  return t > kMinPlausibleEpoch ? t : 0;
}

bool timeTrusted() {
  StateLock lock;
  return g_state.timeTrusted;
}

// Does the clock follow the player all the time while music plays?  Always, except in wifi_mode = sync with
// spotify_live = off: there the radio goes off between looks and the clock looks again when the track on
// screen should be over (spotifyLookDue()).
bool followLive() { return !g_cfg.syncMode() || g_cfg.spotifyLive; }

void publish() {
  if (s_info.status == SPOTIFY_PLAYING) {
    s_lastActiveMs = millis();
  } else {
    s_trackLookAtMs = 0;  // nothing is playing: nothing ends
  }
  StateLock lock;
  g_state.spotify = s_info;
  g_state.spotifySampledMs = s_sampledMs;
}

void setStatus(SpotifyStatus st, const char *message = "") {
  s_info.status = st;
  copyStr(s_info.message, sizeof s_info.message, message);
  publish();
}

// "4h 46m", "12m", "45s"
void formatRemaining(uint32_t sec, char *out, size_t cap) {
  if (sec >= 3600) {
    snprintf(out, cap, "%uh %um", (unsigned)(sec / 3600), (unsigned)((sec / 60) % 60));
  } else if (sec >= 60) {
    snprintf(out, cap, "%um", (unsigned)(sec / 60));
  } else {
    snprintf(out, cap, "%us", (unsigned)sec);
  }
}

// ---- persistence ------------------------------------------------------------
void loadPrefs() {
  s_prefs.begin("spotify", false);
  copyStr(s_refreshToken, sizeof s_refreshToken, s_prefs.getString("rt", "").c_str());
  s_authEpoch = s_prefs.getUInt("auth", 0);
  s_blockedUntilEpoch = s_prefs.getUInt("blk", 0);
  copyStr(s_deviceId, sizeof s_deviceId, s_prefs.getString("dev", "").c_str());
  s_linked = s_refreshToken[0] != 0;
}

void forgetAccount(const char *reason) {
  LOGF(TAG, "unlinking: %s", reason);
  waitForQuietPhase();
  s_prefs.remove("rt");
  s_prefs.remove("auth");
  s_refreshToken[0] = 0;
  s_accessToken[0] = 0;
  s_accessValidUntilMs = 0;
  s_authEpoch = 0;
  s_linked = false;
  s_info.title[0] = s_info.artist[0] = s_info.album[0] = s_info.device[0] = 0;
  setStatus(SPOTIFY_NEEDS_LINK, reason);
}

// ---- HTTP helpers -----------------------------------------------------------
bool containsNoCase(const char *haystack, const char *needle) {
  size_t n = strlen(needle);
  for (; *haystack; ++haystack) {
    if (strncasecmp(haystack, needle, n) == 0) return true;
  }
  return false;
}

// Calls the Web API with the current access token, reusing the TLS connection.
Reply apiCall(const char *method, const String &path, const char *jsonBody = nullptr) {
  static const char *kHeaderKeys[] = {"Retry-After"};
  Reply r;
  for (int attempt = 0; attempt < 2; attempt++) {
    // In arduino-esp32 3.3.8 every stop() of a NetworkClientSecure (server idle
    // timeout, "Connection: close", WiFi loss, ...) wipes its CA bundle hook, and
    // the next connect() would then fail verification for good.  Attaching the
    // bundle only stores pointers, so do it before every request.
    useCaBundle(s_apiClient);
    s_apiHttp.setReuse(true);
    s_apiHttp.setConnectTimeout(8000);
    s_apiHttp.setTimeout(8000);
    s_apiHttp.collectHeaders(kHeaderKeys, 1);
    if (!s_apiHttp.begin(s_apiClient, String(kApiBase) + path)) return r;
    s_apiHttp.addHeader("Authorization", String("Bearer ") + s_accessToken);
    s_apiHttp.addHeader("Accept", "application/json");

    int code;
    if (jsonBody) {
      s_apiHttp.addHeader("Content-Type", "application/json");
      code = s_apiHttp.sendRequest(method, (uint8_t *)jsonBody, strlen(jsonBody));
    } else if (strcmp(method, "GET") == 0) {
      code = s_apiHttp.GET();
    } else {
      // Spotify answers 411 Length Required to a bodyless PUT/POST without this,
      // and HTTPClient only sends the header when there is a payload.
      s_apiHttp.addHeader("Content-Length", "0");
      code = s_apiHttp.sendRequest(method, (uint8_t *)nullptr, 0);
    }
    r.code = code;
    if (code > 0) {
      if (httpStatusHasBody(code)) r.body = s_apiHttp.getString();
      r.retryAfterSec = s_apiHttp.header("Retry-After").toInt();
    }
    s_apiHttp.end();
    if (code > 0) break;
    s_apiClient.stop();  // a stale kept-alive connection: reconnect and try once more
    // Retry only if the request cannot have been executed (never reached the
    // server) or is a read-only GET; a read timeout after a POST may mean it ran.
    bool neverSent = code == HTTPC_ERROR_CONNECTION_REFUSED || code == HTTPC_ERROR_SEND_HEADER_FAILED ||
                     code == HTTPC_ERROR_SEND_PAYLOAD_FAILED || code == HTTPC_ERROR_NOT_CONNECTED ||
                     code == HTTPC_ERROR_CONNECTION_LOST;
    if (!neverSent && strcmp(method, "GET") != 0) break;
  }
  return r;
}

// POST to the accounts service (form encoded, no auth header).
Reply tokenCall(const String &form) {
  Reply r;
  NetworkClientSecure client;
  useCaBundle(client);
  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(8000);
  if (!http.begin(client, kTokenUrl)) return r;
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  r.code = http.POST(form);
  if (r.code > 0 && httpStatusHasBody(r.code)) r.body = http.getString();
  http.end();
  return r;
}

// ---- tokens -----------------------------------------------------------------
bool storeTokens(const SpotifyTokenReply &t) {
  copyStr(s_accessToken, sizeof s_accessToken, t.accessToken);
  uint32_t life = t.expiresInSec > 120 ? (uint32_t)t.expiresInSec - 60 : 30;
  s_accessValidUntilMs = (millis() + life * 1000UL) | 1u;  // (never 0, which means "none")
  if (t.refreshToken[0] && strcmp(t.refreshToken, s_refreshToken) != 0) {
    // Spotify may rotate the refresh token; persist it before it is ever used.
    copyStr(s_refreshToken, sizeof s_refreshToken, t.refreshToken);
    waitForQuietPhase();
    s_prefs.putString("rt", s_refreshToken);
  }
  return true;
}

// Returns true when s_accessToken is usable.
bool ensureAccessToken() {
  // (0 is "no valid token" at every age of the clock: taken as a time it would look valid again after 24.8 days)
  if (s_accessToken[0] && radioplan::pending(millis(), s_accessValidUntilMs)) return true;
  if (!s_refreshToken[0]) return false;

  char enc[400];
  if (!urlEncode(s_refreshToken, enc, sizeof enc)) return false;
  String form = String("grant_type=refresh_token&refresh_token=") + enc + "&client_id=" + g_cfg.spotifyClientId;
  Reply r = tokenCall(form);

  char reason[48], msg[96];
  parseSpotifyError(r.body.c_str(), r.body.length(), reason, sizeof reason, msg, sizeof msg);
  if (r.code == 200) {
    SpotifyTokenReply t;
    if (parseTokenReply(r.body.c_str(), r.body.length(), &t)) {
      storeTokens(t);
      LOGF(TAG, "access token refreshed (valid %d s)", t.expiresInSec);
      return true;
    }
    LOGF(TAG, "token reply not understood");
    return false;
  }
  if (r.code == 400 && strcmp(reason, "invalid_grant") == 0) {
    // revoked, or past Spotify's 6 month limit: needs the user to link again
    forgetAccount("Link expired - please link again");
    return false;
  }
  LOGF(TAG, "token refresh failed: http %d %s %s", r.code, reason, msg);
  return false;
}

// ---- rate limiting ----------------------------------------------------------
void applyRateLimit(int retryAfterSec) {
  if (retryAfterSec <= 0) retryAfterSec = 60;
  time_t now = nowEpoch();
  s_blockedUntilEpoch = now ? (uint32_t)now + (uint32_t)retryAfterSec : 0;
  if (s_blockedUntilEpoch) {
    waitForQuietPhase();
    s_prefs.putUInt("blk", s_blockedUntilEpoch);
  }
  char rem[16];
  formatRemaining((uint32_t)retryAfterSec, rem, sizeof rem);
  char msg[56];
  snprintf(msg, sizeof msg, "Rate limited, retry in %s", rem);
  LOGF(TAG, "429: backing off for %d s", retryAfterSec);
  setStatus(SPOTIFY_ERROR, msg);
  stateNotice("Spotify rate limited", TOAST_WARN);
}

// Returns remaining lockout seconds (0 = not blocked).
uint32_t rateLimitRemaining() {
  time_t now = nowEpoch();
  if (!now || !s_blockedUntilEpoch) return 0;
  if ((uint32_t)now >= s_blockedUntilEpoch) {
    s_blockedUntilEpoch = 0;
    waitForQuietPhase();
    s_prefs.remove("blk");
    return 0;
  }
  return s_blockedUntilEpoch - (uint32_t)now;
}

// ---- polling ----------------------------------------------------------------
uint32_t progressNow() {
  uint32_t p = s_info.progressMs;
  if (s_info.status == SPOTIFY_PLAYING) p += millis() - s_sampledMs;
  if (s_info.durationMs && p > s_info.durationMs) p = s_info.durationMs;
  return p;
}

void schedulePoll(uint32_t afterMs) { s_nextPollMs = millis() + afterMs; }

// 10 s, 20 s, 40 s, 80 s, then 120 s between attempts while errors continue.
uint32_t errorBackoffMs() {
  int shift = s_errorStreak > 1 ? (s_errorStreak > 5 ? 4 : s_errorStreak - 1) : 0;
  uint32_t ms = 10000UL << shift;
  return ms > 120000UL ? 120000UL : ms;
}

void pollPlayer() {
  s_peekDone = true;
  if (!ensureAccessToken()) {
    if (s_linked) {  // say so instead of leaving the last track frozen on screen
      s_errorStreak++;
      setStatus(SPOTIFY_ERROR, "Spotify unreachable");
      schedulePoll(errorBackoffMs());
    }
    return;
  }
  Reply r = apiCall("GET", "/me/player?additional_types=episode");

  if (r.code == 200) {
    SpotifyInfo parsed = s_info;
    char dev[48];
    if (!parsePlayerState(r.body.c_str(), r.body.length(), &parsed, dev, sizeof dev)) {
      LOGF(TAG, "player reply not understood (%u bytes)", (unsigned)r.body.length());
      setStatus(SPOTIFY_ERROR, "Unexpected Spotify reply");
      schedulePoll(20000);
      return;
    }
    parsed.message[0] = 0;
    s_info = parsed;
    s_sampledMs = millis();
    s_errorStreak = 0;
    s_authFailures = 0;
    s_idlePolls = 0;
    if (dev[0] && strcmp(dev, s_deviceId) != 0) {
      copyStr(s_deviceId, sizeof s_deviceId, dev);
      waitForQuietPhase();
      s_prefs.putString("dev", s_deviceId);
    }
    publish();

    uint32_t interval = s_info.status == SPOTIFY_PLAYING ? SPOTIFY_POLL_PLAYING_MS : SPOTIFY_POLL_PAUSED_MS;
    if (s_info.status == SPOTIFY_PLAYING && s_info.durationMs > s_info.progressMs) {
      // look again just after the track ends so the next title shows promptly
      uint32_t untilEnd = s_info.durationMs - s_info.progressMs + 1500;
      if (untilEnd < interval) interval = untilEnd < 1500 ? 1500 : untilEnd;
    }
    schedulePoll(interval);
    // spotify_live = off: the radio may go off now, and the next look is due when this track should be over
    // (at once if it already is; never for something with no length, which the look of every session follows)
    if (s_info.status == SPOTIFY_PLAYING && s_info.durationMs > 0) {
      s_trackLookAtMs = (millis() + radioplan::trackLookAfterMs(s_info.durationMs, s_info.progressMs)) | 1u;
    } else {
      s_trackLookAtMs = 0;
    }
    return;
  }

  if (r.code == 204) {  // nothing playing and no active device
    s_info.title[0] = s_info.artist[0] = s_info.album[0] = s_info.device[0] = 0;
    s_info.durationMs = s_info.progressMs = 0;
    s_info.message[0] = 0;
    s_info.status = SPOTIFY_IDLE;
    s_sampledMs = millis();
    s_errorStreak = 0;
    s_authFailures = 0;
    publish();
    s_idlePolls++;
    schedulePoll(s_idlePolls > kIdlePollsBeforeSlowdown ? SPOTIFY_POLL_IDLE_MS * 2 : SPOTIFY_POLL_IDLE_MS);
    return;
  }

  char reason[48], msg[96];
  parseSpotifyError(r.body.c_str(), r.body.length(), reason, sizeof reason, msg, sizeof msg);
  if (r.code == 401) {  // access token rejected: refresh and try again shortly
    s_accessValidUntilMs = 0;
    if (++s_authFailures >= 3) {  // a fresh token keeps being refused: don't hammer the servers
      s_authFailures = 0;
      setStatus(SPOTIFY_ERROR, "Spotify rejected the token");
      schedulePoll(60000);
    } else {
      schedulePoll(500);
    }
    return;
  }
  if (r.code == 429) {
    applyRateLimit(r.retryAfterSec);
    return;
  }
  LOGF(TAG, "poll failed: http %d %s %s", r.code, reason, msg);
  s_errorStreak++;
  if (r.code == 403) {
    bool premium = containsNoCase(msg, "premium");
    setStatus(SPOTIFY_ERROR, premium ? "App owner needs Premium" : "Access denied (403)");
    schedulePoll(60000);
    return;
  }
  char text[56];
  if (r.code <= 0) {
    snprintf(text, sizeof text, "Spotify unreachable");
  } else {
    snprintf(text, sizeof text, "Spotify error %d", r.code);
  }
  setStatus(SPOTIFY_ERROR, text);
  schedulePoll(errorBackoffMs());
}

// ---- commands ---------------------------------------------------------------
void handleResult(const Reply &r, const char *what) {
  if (r.code >= 200 && r.code < 300) {
    schedulePoll(900);  // confirm the new state shortly (Spotify needs a moment)
    return;
  }
  char reason[48], msg[96];
  parseSpotifyError(r.body.c_str(), r.body.length(), reason, sizeof reason, msg, sizeof msg);
  LOGF(TAG, "%s failed: http %d %s %s", what, r.code, reason, msg);
  if (r.code == 429) {
    applyRateLimit(r.retryAfterSec);
    return;
  }
  schedulePoll(500);  // whatever went wrong, find out what the player is really doing
  if (r.code == 401) {
    s_accessValidUntilMs = 0;
    stateNotice("Spotify: try again", TOAST_WARN);
  } else if (r.code == 403 && strcmp(reason, "PREMIUM_REQUIRED") == 0) {
    stateNotice("Spotify Premium required", TOAST_WARN);
  } else if (r.code == 404) {
    stateNotice("No active Spotify device", TOAST_WARN);
  } else if (r.code == 403) {
    stateNotice("Spotify refused that", TOAST_WARN);
  } else {
    char text[40];
    snprintf(text, sizeof text, r.code > 0 ? "Spotify error %d" : "Spotify unreachable", r.code);
    stateNotice(text, TOAST_WARN);
  }
}

void handleCommand(SpotifyCommand cmd) {
  s_lastActiveMs = millis();  // (wifi_mode = sync: the radio stays on for a while, for the confirming poll and what comes next)
  if (cmd == SPOTIFY_CMD_UNLINK) {
    forgetAccount("Unlinked");
    return;
  }
  if (cmd == SPOTIFY_CMD_REFRESH) {
    schedulePoll(0);
    return;
  }
  if (!s_linked) {
    stateNotice("Spotify not linked", TOAST_WARN);
    return;
  }
  if (rateLimitRemaining()) {
    stateNotice("Spotify rate limited", TOAST_WARN);
    return;
  }
  if (!ensureAccessToken()) {
    stateNotice("Spotify unavailable", TOAST_WARN);
    return;
  }

  Reply r;
  switch (cmd) {
    case SPOTIFY_CMD_PLAY_PAUSE: {
      // Polling is slow while idle, so the state we hold can be 30 s old.  Look
      // first: a press right after starting music on a phone must pause it, not
      // send a useless "play".
      const bool assumedPlaying = s_info.status == SPOTIFY_PLAYING;  // what the UI based its toast on
      if ((uint32_t)(millis() - s_sampledMs) > 2500) {
        pollPlayer();
        if (rateLimitRemaining()) {
          stateNotice("Spotify rate limited", TOAST_WARN);
          return;
        }
        if (!s_linked) return;  // the refresh token turned out to be revoked
      }
      bool playing = s_info.status == SPOTIFY_PLAYING;
      if (playing != assumedPlaying) {  // the screen was out of date: correct the toast it showed
        stateNotice(playing ? "Pause" : "Play", playing ? TOAST_PAUSE : TOAST_PLAY);
      }
      r = apiCall("PUT", playing ? "/me/player/pause" : "/me/player/play");
      if (!playing && r.code == 404 && s_deviceId[0]) {
        // The device went idle (Spotify forgets it after a while): wake it.
        String body = String("{\"device_ids\":[\"") + s_deviceId + "\"],\"play\":true}";
        r = apiCall("PUT", "/me/player", body.c_str());
      }
      if (r.code >= 200 && r.code < 300) {  // flip the display now, the poll will confirm
        uint32_t p = progressNow();
        s_info.progressMs = p;
        s_sampledMs = millis();
        s_info.status = playing ? SPOTIFY_PAUSED : SPOTIFY_PLAYING;
        publish();
      }
      handleResult(r, "play/pause");
      break;
    }
    case SPOTIFY_CMD_NEXT:
      r = apiCall("POST", "/me/player/next");
      handleResult(r, "next");
      break;
    case SPOTIFY_CMD_PREVIOUS:
      r = apiCall("POST", "/me/player/previous");
      handleResult(r, "previous");
      break;
    default:
      break;
  }
}

// ---- linking page -----------------------------------------------------------
void newPkce() {
  static const char *kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-._~";  // 66 chars
  for (int i = 0; i < 64; i++) s_verifier[i] = kAlphabet[esp_random() % 66];
  s_verifier[64] = 0;
  for (int i = 0; i < 16; i++) s_state[i] = kAlphabet[esp_random() % 62];
  s_state[16] = 0;
}

String authorizeUrl() {
  uint8_t digest[32];
  mbedtls_sha256((const unsigned char *)s_verifier, strlen(s_verifier), digest, 0);
  char challenge[48];
  base64UrlEncode(digest, sizeof digest, challenge, sizeof challenge);
  char url[640];
  if (!buildAuthorizeUrl(g_cfg.spotifyClientId, kRedirectUri, kScopes, challenge, s_state, url, sizeof url)) {
    return String();
  }
  return String(url);
}

String htmlEscape(const char *s) {
  String out;
  for (; *s; ++s) {
    switch (*s) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += *s; break;
    }
  }
  return out;
}

// Wraps `body` in the page chrome shared by the setup page and the callback answer.
void sendPage(int code, const String &body) {
  const size_t cap = body.length() + 1400;
  char *doc = (char *)malloc(cap);
  if (!doc || !renderPageDocument(APP_NAME, body.c_str(), doc, cap)) {
    free(doc);
    s_web->send(500, "text/plain", "Could not build the page");
    return;
  }
  s_web->send(code, "text/html; charset=utf-8", doc);
  free(doc);
}

void sendLinkPage() {
  const String ip = WiFi.localIP().toString();
  const String authorize = authorizeUrl();
  LinkPageInfo info;
  info.appName = APP_NAME;
  info.linked = s_linked;
  info.daysLeft = s_info.linkDaysLeft;
  info.message = s_pageMessage;
  info.messageIsError = s_pageMessageIsError;
  info.redirectUri = kRedirectUri;
  info.clockAddress = ip.c_str();
  info.authorizeUrl = authorize.c_str();

  const size_t cap = 6000;
  char *body = (char *)malloc(cap);
  if (!body || !renderLinkPageBody(info, body, cap)) {
    free(body);
    s_web->send(500, "text/plain", "Could not build the page");
    return;
  }
  sendPage(200, String(body));
  free(body);
}

bool exchangeCode(const char *code, char *error, size_t errorCap) {
  char codeEnc[600], redirEnc[96];
  if (!urlEncode(code, codeEnc, sizeof codeEnc) || !urlEncode(kRedirectUri, redirEnc, sizeof redirEnc)) {
    copyStr(error, errorCap, "The code in that address is too long.");
    return false;
  }
  String form = String("grant_type=authorization_code&code=") + codeEnc + "&redirect_uri=" + redirEnc +
                "&client_id=" + g_cfg.spotifyClientId + "&code_verifier=" + s_verifier;
  Reply r = tokenCall(form);
  if (r.code != 200) {
    char reason[48], msg[96];
    parseSpotifyError(r.body.c_str(), r.body.length(), reason, sizeof reason, msg, sizeof msg);
    LOGF(TAG, "code exchange failed: http %d %s %s", r.code, reason, msg);
    if (r.code <= 0) {
      copyStr(error, errorCap, "Could not reach Spotify. Check the clock's internet connection.");
    } else {
      snprintf(error, errorCap, "Spotify refused the code (%s). Please start again.", reason[0] ? reason : "error");
    }
    return false;
  }
  SpotifyTokenReply t;
  if (!parseTokenReply(r.body.c_str(), r.body.length(), &t) || !t.refreshToken[0]) {
    copyStr(error, errorCap, "Spotify's reply was not understood.");
    return false;
  }
  storeTokens(t);
  time_t now = nowEpoch();
  s_authEpoch = (uint32_t)now;
  waitForQuietPhase();
  s_prefs.putUInt("auth", s_authEpoch);
  s_linked = true;
  s_idlePolls = 0;
  s_errorStreak = 0;
  return true;
}

// Finishes the OAuth flow with the code Spotify sent back, whichever way it
// arrived (the helper / a forwarded redirect on /callback, or the pasted address).
// `state` may be empty.  `message` always receives a sentence for the user.
bool completeLinking(const char *code, const char *state, char *message, size_t cap) {
  if (state[0] && strcmp(state, s_state) != 0) {
    copyStr(message, cap, "That redirect belongs to a different attempt. Please start again from the setup page.");
    return false;
  }
  if (!timeTrusted()) {
    copyStr(message, cap, "The clock has not got the time yet. Try again in a minute.");
    return false;
  }
  char error[120];
  if (!exchangeCode(code, error, sizeof error)) {
    copyStr(message, cap, error);
    newPkce();  // a used or mismatched code cannot be reused
    return false;
  }
  s_info.linkDaysLeft = (int16_t)(kRefreshTokenLifetimeS / 86400UL);
  setStatus(SPOTIFY_IDLE);
  schedulePoll(0);
  stateNotice("Spotify linked", TOAST_PLAY);
  copyStr(message, cap, "Linked! You can close this page; the clock will show what is playing.");
  return true;
}

// POST /link: the user pasted the address their browser ended up on.
void handleLink() {
  String url = s_web->arg("url");
  char code[512], state[40], err[48], message[160];
  s_pageMessageIsError = true;

  if (queryParam(url.c_str(), "error", err, sizeof err)) {
    snprintf(s_pageMessage, sizeof s_pageMessage, "Spotify reported: %s. Please start again from step 2.", err);
  } else if (!queryParam(url.c_str(), "code", code, sizeof code)) {
    copyStr(s_pageMessage, sizeof s_pageMessage,
            "That address does not contain a code=... part. Paste the full address Spotify redirected you to.");
  } else {
    if (!queryParam(url.c_str(), "state", state, sizeof state)) state[0] = 0;
    bool ok = completeLinking(code, state, message, sizeof message);
    copyStr(s_pageMessage, sizeof s_pageMessage, message);
    s_pageMessageIsError = !ok;
  }
  sendLinkPage();
}

// GET /callback?code=...&state=...: Spotify's redirect, handed over by the helper
// (or by any forward of 127.0.0.1:8888 to this port).  The helper shows our
// answer in the browser tab.
void handleCallback() {
  char message[160];
  bool ok = false;
  if (s_web->hasArg("error")) {
    snprintf(message, sizeof message, "Spotify reported: %s. Please start again.", s_web->arg("error").c_str());
  } else if (!s_web->hasArg("code")) {
    copyStr(message, sizeof message, "This address has no code=... part, so there is nothing to link.");
  } else {
    String code = s_web->arg("code"), state = s_web->arg("state");
    ok = completeLinking(code.c_str(), state.c_str(), message, sizeof message);
  }
  String b = ok ? "<h1>Linked</h1><p class=ok>" : "<h1>Not linked</h1><p class=err>";
  b += htmlEscape(message);
  b += "</p>";
  if (!ok) b += "<p><a href='http://" + WiFi.localIP().toString() + "/'>Back to the clock's setup page</a></p>";
  sendPage(ok ? 200 : 400, b);
}

// GET /go: straight to Spotify's approval page (used by the helper).
void handleGo() {
  s_web->sendHeader("Location", authorizeUrl(), true);
  s_web->send(302, "text/plain", "");
}

// GET /spotify_link.py: the helper, so it is one click away from the setup page.
void handleHelperScript() {
  s_web->sendHeader("Content-Disposition", "attachment; filename=\"spotify_link.py\"");
  s_web->send(200, "text/x-python; charset=utf-8", String(kSpotifyHelperScript));
}

void serveLinkPage(bool wanted, bool wifiUp) {
  if (wanted && wifiUp) {
    if (!s_web) {
      if (!s_verifier[0]) newPkce();
      s_web = new WebServer(80);
      s_web->on("/", HTTP_GET, sendLinkPage);
      s_web->on("/link", HTTP_POST, handleLink);
      s_web->on("/callback", HTTP_GET, handleCallback);
      s_web->on("/go", HTTP_GET, handleGo);
      s_web->on("/spotify_link.py", HTTP_GET, handleHelperScript);
      s_web->onNotFound([]() {
        s_web->sendHeader("Location", "/");
        s_web->send(302, "text/plain", "");
      });
      s_web->begin();
      if (MDNS.begin(g_cfg.hostname)) MDNS.addService("http", "tcp", 80);
      LOGF(TAG, "link page at http://%s/ (or http://%s.local/)", WiFi.localIP().toString().c_str(), g_cfg.hostname);
    }
    s_web->handleClient();
  } else if (s_web) {
    s_web->stop();
    delete s_web;
    s_web = nullptr;
    MDNS.end();
  }
}

// Keeps linkUrl / linkDaysLeft in the published state current.
void refreshLinkInfo(bool serving, bool wifiUp) {
  int16_t days = -1;
  time_t now = nowEpoch();
  if (s_linked && s_authEpoch && now) {
    int64_t left = (int64_t)s_authEpoch + kRefreshTokenLifetimeS - (int64_t)now;
    days = left <= 0 ? 0 : (int16_t)(left / 86400);
  }
  char url[sizeof s_info.linkUrl] = "";
  if (serving && wifiUp) snprintf(url, sizeof url, "http://%s", WiFi.localIP().toString().c_str());
  if (days != s_info.linkDaysLeft || strcmp(url, s_info.linkUrl) != 0) {
    s_info.linkDaysLeft = days;
    copyStr(s_info.linkUrl, sizeof s_info.linkUrl, url);
    publish();
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool spotifyConfigured() { return g_cfg.wifi && g_cfg.spotify && g_cfg.spotifyClientId[0] != 0; }

bool spotifyLowLatencyWanted() { return s_web != nullptr; }

void spotifyBegin() {
  if (!s_queue) s_queue = xQueueCreate(6, sizeof(uint8_t));
  if (!spotifyConfigured()) {
    s_info.status = SPOTIFY_DISABLED;
    publish();
    return;
  }
  loadPrefs();
  s_info.status = s_linked ? SPOTIFY_IDLE : SPOTIFY_NEEDS_LINK;
  publish();
  LOGF(TAG, "%s", s_linked ? "account linked" : "no account linked yet");
}

bool spotifyPost(SpotifyCommand cmd) {
  if (!spotifyConfigured() || !s_queue) return false;
  uint8_t c = (uint8_t)cmd;
  return xQueueSend(s_queue, &c, 0) == pdTRUE;
}

void spotifyTick() {
  if (!spotifyConfigured()) return;
  const bool wifiUp = WiFi.status() == WL_CONNECTED;
  // The linking page (or the renewal hint) is served all the time, except in wifi_mode = sync, where it
  // needs the radio on and is served only while the Now Playing page is open.
  const bool pageOpen = !g_cfg.syncMode() || radioplan::demandActive(millis(), s_pageHoldUntilMs);
  const bool serving = (!s_linked || (s_info.linkDaysLeft >= 0 && s_info.linkDaysLeft <= kRenewHintDays)) && pageOpen;

  refreshLinkInfo(serving && wifiUp, wifiUp);
  serveLinkPage(serving, wifiUp);

  uint8_t c;
  while (s_queue && xQueueReceive(s_queue, &c, 0) == pdTRUE) {
    if (wifiUp) {
      handleCommand((SpotifyCommand)c);
    } else {
      stateNotice("Spotify: no network", TOAST_WARN);
    }
  }
  if (!s_linked || !wifiUp || !timeTrusted()) return;

  if (uint32_t remaining = rateLimitRemaining()) {
    if ((uint32_t)(millis() - s_lastBlockMsgMs) > 30000UL || s_info.status != SPOTIFY_ERROR) {
      char rem[16], msg[56];
      formatRemaining(remaining, rem, sizeof rem);
      snprintf(msg, sizeof msg, "Rate limited, retry in %s", rem);
      setStatus(SPOTIFY_ERROR, msg);
      s_lastBlockMsgMs = millis();
    }
    return;
  }
  radioplan::keepDue(millis(), &s_nextPollMs);  // a poll that has been due for weeks (no network) stays due
  if ((int32_t)(millis() - s_nextPollMs) >= 0) pollPlayer();
}

// ---- wifi_mode = sync: the radio comes and goes (net_task.cpp, radio_plan.h) ----------------------------
void spotifySetPageShown(bool shown) {
  if (shown) s_pageHoldUntilMs = (millis() + kPageHoldMs) | 1u;  // a minute from now (the next call, while it is shown, moves it on)
}

bool spotifyWantsRadio() {
  if (!spotifyConfigured()) return false;
  const uint32_t now = millis();
  // a time long past must not come round again when millis() wraps (the UI task sets a fresh one every
  // time round its loop while the page is shown, so nothing is lost if it does so right now)
  if (s_pageHoldUntilMs != 0 && !radioplan::pending(now, s_pageHoldUntilMs)) s_pageHoldUntilMs = 0;
  radioplan::SpotifyNeeds n;
  n.commandWaiting = s_queue && uxQueueMessagesWaiting(s_queue) > 0;
  n.playing = s_info.status == SPOTIFY_PLAYING;
  n.pausedRecently = s_info.status == SPOTIFY_PAUSED && (uint32_t)(now - s_lastActiveMs) < kPausedHoldMs;
  n.pageOpen = radioplan::pending(now, s_pageHoldUntilMs);
  n.live = followLive();
  return radioplan::spotifyHolds(n);
}

void spotifyWindowBegin() {
  s_peekDone = false;
  s_nextPollMs = millis();  // look at the player as soon as the radio is up (now, not "0": that is a time too)
}

bool spotifyPeekPending() {
  return spotifyConfigured() && s_linked && !s_peekDone && timeTrusted() && rateLimitRemaining() == 0;
}

// spotify_live = off: the track on screen should be over by now, so it is time for another look.
bool spotifyLookDue() {
  if (!spotifyConfigured() || !s_linked || followLive()) return false;
  if (s_info.status != SPOTIFY_PLAYING || s_trackLookAtMs == 0) return false;
  return radioplan::due(millis(), s_trackLookAtMs) && timeTrusted() && rateLimitRemaining() == 0;
}

// The radio is about to be switched off: let go of the connections.  `keepTrack`: the session ended as it
// should, so what it saw last stays on the screen until the next look (a paused track; with spotify_live =
// off also a playing one, its progress counted on by the clock).  Otherwise the network is gone, and a track
// that nobody can follow any more is taken off.
void spotifyRadioDown(bool keepTrack) {
  s_apiHttp.end();
  s_apiClient.stop();
  if (s_web) {
    s_web->stop();
    delete s_web;
    s_web = nullptr;
    MDNS.end();
  }
  if (!keepTrack && (s_info.status == SPOTIFY_PLAYING || s_info.status == SPOTIFY_PAUSED)) {
    s_info.title[0] = s_info.artist[0] = s_info.album[0] = s_info.device[0] = 0;
    s_info.durationMs = s_info.progressMs = 0;
    s_info.status = SPOTIFY_IDLE;
  }
  s_info.linkUrl[0] = 0;
  publish();
}

void spotifyFlushQueue(const char *why) {
  if (!s_queue) return;
  uint8_t c;
  bool any = false;
  while (xQueueReceive(s_queue, &c, 0) == pdTRUE) any = true;
  if (any) stateNotice(why, TOAST_WARN);
}
