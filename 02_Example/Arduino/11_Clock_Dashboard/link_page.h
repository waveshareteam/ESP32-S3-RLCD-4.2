#pragma once

// The Spotify setup page served by the clock.  Rendering is a pure function of
// plain values (no Arduino types) so the host tests can render it, check it and
// write it out as an HTML file to look at in a browser.

#include <stddef.h>

struct LinkPageInfo {
  const char *appName = "";       // e.g. "RLCD Clock"
  bool linked = false;
  int daysLeft = -1;              // days until Spotify expires the link; -1 = unknown
  const char *message = "";       // result of the last attempt ("" = none)
  bool messageIsError = false;
  const char *redirectUri = "";   // what the user must register in the Spotify app
  const char *clockAddress = "";  // the clock's IP address, used in the helper command
  const char *authorizeUrl = "";  // Spotify approval link for the manual route
};

// Renders the page body (everything between <body> and </body>) into `out`.
// Returns its length, or 0 if `cap` was too small.
size_t renderLinkPageBody(const LinkPageInfo &info, char *out, size_t cap);

// Wraps a rendered body in the complete HTML document (head, styles, tail) used by
// every page the clock serves.  Returns the length, or 0 if `cap` was too small.
size_t renderPageDocument(const char *appName, const char *body, char *out, size_t cap);
