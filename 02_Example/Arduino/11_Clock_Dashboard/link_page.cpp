#include "link_page.h"

#include <stdio.h>
#include <string.h>

namespace {

// Bounded appender: once anything does not fit, the whole render is reported as failed.
class Out {
 public:
  Out(char *buf, size_t cap) : buf_(buf), cap_(cap) {
    if (cap_) buf_[0] = 0;
  }
  void add(const char *s) {
    size_t l = strlen(s);
    if (n_ + l + 1 > cap_) {
      overflow_ = true;
      return;
    }
    memcpy(buf_ + n_, s, l + 1);
    n_ += l;
  }
  void addEscaped(const char *s) {  // for text and attribute values
    for (; *s; ++s) {
      switch (*s) {
        case '&': add("&amp;"); break;
        case '<': add("&lt;"); break;
        case '>': add("&gt;"); break;
        case '"': add("&quot;"); break;
        case '\'': add("&#39;"); break;
        default: {
          char c[2] = {*s, 0};
          add(c);
        }
      }
    }
  }
  void addInt(int v) {
    char t[16];
    snprintf(t, sizeof t, "%d", v);
    add(t);
  }
  size_t size() const { return overflow_ ? 0 : n_; }

 private:
  char *buf_;
  size_t cap_;
  size_t n_ = 0;
  bool overflow_ = false;
};

}  // namespace

size_t renderLinkPageBody(const LinkPageInfo &i, char *out, size_t cap) {
  Out o(out, cap);

  o.add("<h1>");
  o.addEscaped(i.appName);
  o.add(": connect Spotify</h1><p>Status: <b>");
  o.add(i.linked ? "linked" : "not linked");
  if (i.linked && i.daysLeft >= 0) {
    o.add(" (Spotify expires the link in ");
    o.addInt(i.daysLeft);
    o.add(" days; linking again renews it)");
  }
  o.add("</b></p>");
  if (i.message[0]) {
    o.add(i.messageIsError ? "<p class=err>" : "<p class=ok>");
    o.addEscaped(i.message);
    o.add("</p>");
  }

  o.add("<h2>1. Allow the redirect</h2>"
        "<p>In the <a href='https://developer.spotify.com/dashboard' target=_blank>Spotify developer dashboard</a> "
        "open your app, choose <i>Settings</i>, add this <b>Redirect URI</b> and Save:</p>"
        "<p><input type=text readonly onclick='this.select()' value='");
  o.addEscaped(i.redirectUri);
  o.add("'></p><p><small>Why 127.0.0.1 and not the clock's own address? Spotify refuses plain-http redirects as "
        "&quot;insecure&quot;, with one exception: 127.0.0.1, which means &quot;this computer&quot;. So Spotify's "
        "answer comes back to the computer you are browsing from, and something on it has to hand it to the clock. "
        "That is step 2.</small></p>");

  o.add("<h2>2. Approve access (pick one)</h2>"
        "<h3>A. With the helper: easiest, needs Python 3 on this computer</h3><ol>"
        "<li><a href='/spotify_link.py'>Download spotify_link.py</a> (a small script served by the clock)</li>"
        "<li>Run it: <code>python spotify_link.py ");
  o.addEscaped(i.clockAddress);
  o.add("</code></li></ol><p>A browser tab opens Spotify. Press <i>Agree</i> and the clock is linked.</p>"
        "<h3>B. By hand: any browser, no helper</h3><ol><li><a href='");
  o.addEscaped(i.authorizeUrl);
  o.add("' target=_blank>Open Spotify and approve access</a></li>"
        "<li>The browser then ends on an error page for 127.0.0.1 (&quot;can't reach this page&quot;). That is "
        "expected: nothing there is listening. Copy the <b>whole address</b> from the address bar and paste it here:"
        "<form method=post action=/link><input type=text name=url required "
        "placeholder='http://127.0.0.1:8888/callback?code=...'>"
        "<button type=submit>Finish linking</button></form></li></ol>"
        "<p><small>Needs a Spotify Premium account (Spotify's rules for developer apps and for playback control). "
        "Only the permissions to read and control playback are requested.</small></p>");
  return o.size();
}

size_t renderPageDocument(const char *appName, const char *body, char *out, size_t cap) {
  Out o(out, cap);
  o.add("<!doctype html><html><head><meta charset=utf-8>"
        "<meta name=viewport content='width=device-width,initial-scale=1'><title>");
  o.addEscaped(appName);
  o.add(" - Spotify</title><style>"
        "body{font:16px/1.5 system-ui,sans-serif;max-width:44rem;margin:2rem auto;padding:0 1rem;color:#222}"
        "code,input[readonly]{background:#eee;padding:.1rem .35rem;border-radius:3px;word-break:break-all}"
        "input[type=text]{width:100%;padding:.55rem;box-sizing:border-box;font:inherit}"
        "button{padding:.55rem 1.2rem;margin-top:.6rem;font:inherit}"
        "h2{margin:1.6rem 0 .3rem}h3{margin:1.1rem 0 .2rem}"
        ".ok{color:#070}.err{color:#b00}small{color:#666}</style></head><body>");
  o.add(body);
  o.add("</body></html>");
  return o.size();
}
