#!/usr/bin/env python3
"""Link the RLCD clock to your Spotify account.

Spotify only accepts a plain-http redirect to 127.0.0.1 (this computer), never
to the clock's own address.  This helper listens on 127.0.0.1:8888 just long
enough to catch that redirect and hand it to the clock.

    python spotify_link.py [clock-address]

clock-address is the clock's IP address (shown on its screen and on its web
page) or its name; the default is rlcd-clock.local.  Needs Python 3.7 or newer
and nothing else.  The clock also serves this file at http://<clock>/spotify_link.py
"""
import argparse
import sys
import threading
import urllib.error
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def main():
    ap = argparse.ArgumentParser(description="Link the RLCD clock to Spotify.")
    ap.add_argument("clock", nargs="?", default="rlcd-clock.local", help="clock IP address or name")
    ap.add_argument("--port", type=int, default=8888, help="must match the Redirect URI in your Spotify app")
    ap.add_argument("--no-browser", action="store_true", help="print the link instead of opening it")
    ap.add_argument("--timeout", type=int, default=300, help="seconds to wait for you to approve")
    args = ap.parse_args()

    clock = args.clock.replace("http://", "").strip("/")
    base = "http://" + clock

    # Is the clock there, and is it offering the link page?
    try:
        urllib.request.urlopen(base + "/", timeout=8).close()
    except Exception as exc:  # noqa: BLE001 - any failure means "can't use it"
        print("Cannot reach the clock at %s/\n  (%s)" % (base, exc))
        print("Give its IP address, e.g.:  python spotify_link.py 192.168.1.50")
        print("The clock only offers this page while no account is linked (and in the last 14 days")
        print("before a link expires).")
        return 1

    done = threading.Event()
    outcome = {}

    class Catch(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802 - http.server API
            if not self.path.startswith("/callback"):
                self.send_error(404)
                return
            try:  # pass Spotify's redirect (code and state) on to the clock, show its answer
                with urllib.request.urlopen(base + self.path, timeout=40) as r:
                    status, body = r.status, r.read()
                    ctype = r.headers.get("Content-Type", "text/html; charset=utf-8")
            except urllib.error.HTTPError as exc:
                status, body = exc.code, exc.read()
                ctype = exc.headers.get("Content-Type", "text/html; charset=utf-8")
            except Exception as exc:  # noqa: BLE001
                status = 502
                body = ("Could not reach the clock at %s: %s" % (base, exc)).encode()
                ctype = "text/plain; charset=utf-8"
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            outcome["status"] = status
            done.set()

        def log_message(self, *args):  # keep the console quiet
            pass

    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Catch)
    except OSError as exc:
        print("Cannot listen on 127.0.0.1:%d (%s).\nClose whatever is using that port and try again." % (args.port, exc))
        return 1
    threading.Thread(target=server.serve_forever, daemon=True).start()

    url = base + "/go"  # the clock redirects this to Spotify's approval page
    print("Opening Spotify's approval page. Press Agree there.")
    print("If no browser opens, go to:  " + url)
    if not args.no_browser:
        webbrowser.open(url)

    finished = done.wait(args.timeout)
    server.shutdown()
    if not finished:
        print("Timed out after %d seconds without hearing from Spotify." % args.timeout)
        return 1
    if outcome.get("status") == 200:
        print("Done: the clock is linked to Spotify.")
        return 0
    print("The clock answered with status %s; see the browser tab for why." % outcome.get("status"))
    return 1


if __name__ == "__main__":
    sys.exit(main())
