"""Tests for tools/spotify_link.py against a fake clock and a simulated browser.

    python -m unittest tools/tests/test_spotify_link.py      (from the sketch folder)
    python tools/tests/test_spotify_link.py
"""
import pathlib
import socket
import subprocess
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

TOOLS = pathlib.Path(__file__).resolve().parent.parent
HELPER = TOOLS / "spotify_link.py"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakeClock:
    """Stands in for the clock's link page: /, /go and /callback."""

    def __init__(self, callback_status=200, callback_body=b"<h1>Linked!</h1>"):
        self.received = []  # request paths seen
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                outer.received.append(self.path)
                if self.path == "/":
                    self._send(200, b"setup page")
                elif self.path == "/go":
                    self.send_response(302)
                    self.send_header("Location", "https://accounts.spotify.com/authorize?client_id=fake")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                elif self.path.startswith("/callback"):
                    self._send(callback_status, callback_body)
                else:
                    self._send(404, b"nope")

            def _send(self, status, body):
                self.send_response(status)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.address = "127.0.0.1:%d" % self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def run_helper(address, port, timeout=20, *extra):
    return subprocess.Popen(
        [sys.executable, str(HELPER), address, "--port", str(port), "--no-browser", "--timeout", str(timeout), *extra],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def wait_listening(port, limit=10.0):
    end = time.time() + limit
    while time.time() < end:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.05)
    return False


class HelperTests(unittest.TestCase):
    def test_forwards_redirect_to_clock_and_relays_answer(self):
        clock, port = FakeClock(), free_port()
        try:
            proc = run_helper(clock.address, port)
            self.assertTrue(wait_listening(port), "helper never started listening")
            # the "browser" arrives after Spotify's redirect
            with urllib.request.urlopen("http://127.0.0.1:%d/callback?code=AQD%%2Bxyz&state=abc123" % port, timeout=10) as r:
                self.assertEqual(r.status, 200)
                self.assertIn(b"Linked!", r.read())
            out, _ = proc.communicate(timeout=15)
            self.assertEqual(proc.returncode, 0, out)
            self.assertIn("Done", out)
            self.assertIn("/callback?code=AQD%2Bxyz&state=abc123", clock.received)  # forwarded untouched
            self.assertIn("/", clock.received)  # it checked the clock first
            self.assertIn("/go", out)  # printed the link for the user
        finally:
            clock.close()

    def test_error_from_clock_is_relayed_and_reported(self):
        clock, port = FakeClock(callback_status=400, callback_body=b"<p>Spotify refused the code</p>"), free_port()
        try:
            proc = run_helper(clock.address, port)
            self.assertTrue(wait_listening(port))
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                urllib.request.urlopen("http://127.0.0.1:%d/callback?code=bad" % port, timeout=10)
            self.assertEqual(ctx.exception.code, 400)
            self.assertIn(b"refused", ctx.exception.read())
            out, _ = proc.communicate(timeout=15)
            self.assertEqual(proc.returncode, 1, out)
        finally:
            clock.close()

    def test_unrelated_requests_do_not_finish_it(self):
        clock, port = FakeClock(), free_port()
        try:
            proc = run_helper(clock.address, port)
            self.assertTrue(wait_listening(port))
            with self.assertRaises(urllib.error.HTTPError) as ctx:  # e.g. the browser asking for a favicon
                urllib.request.urlopen("http://127.0.0.1:%d/favicon.ico" % port, timeout=10)
            self.assertEqual(ctx.exception.code, 404)
            self.assertIsNone(proc.poll(), "helper quit on an unrelated request")
            urllib.request.urlopen("http://127.0.0.1:%d/callback?code=ok" % port, timeout=10).read()
            out, _ = proc.communicate(timeout=15)
            self.assertEqual(proc.returncode, 0, out)
        finally:
            clock.close()

    def test_idle_speculative_connection_does_not_block_the_real_request(self):
        clock, port = FakeClock(), free_port()
        idle = None
        try:
            proc = run_helper(clock.address, port)
            self.assertTrue(wait_listening(port))
            idle = socket.create_connection(("127.0.0.1", port))  # browsers open spare, silent connections
            started = time.time()
            urllib.request.urlopen("http://127.0.0.1:%d/callback?code=ok" % port, timeout=10).read()
            self.assertLess(time.time() - started, 5, "served only after the idle connection timed out")
            proc.communicate(timeout=15)
            self.assertEqual(proc.returncode, 0)
        finally:
            if idle:
                idle.close()
            clock.close()

    def test_clock_unreachable(self):
        port = free_port()
        proc = run_helper("127.0.0.1:%d" % free_port(), port)  # nothing listens on the "clock" port
        out, _ = proc.communicate(timeout=30)
        self.assertEqual(proc.returncode, 1)
        self.assertIn("Cannot reach the clock", out)

    def test_port_already_in_use(self):
        clock = FakeClock()
        busy = socket.socket()
        busy.bind(("127.0.0.1", 0))
        busy.listen(1)
        try:
            proc = run_helper(clock.address, busy.getsockname()[1])
            out, _ = proc.communicate(timeout=30)
            self.assertEqual(proc.returncode, 1)
            self.assertIn("Cannot listen on 127.0.0.1", out)
        finally:
            busy.close()
            clock.close()

    def test_times_out_without_a_redirect(self):
        clock, port = FakeClock(), free_port()
        try:
            proc = run_helper(clock.address, port, 1)
            out, _ = proc.communicate(timeout=30)
            self.assertEqual(proc.returncode, 1)
            self.assertIn("Timed out", out)
        finally:
            clock.close()

    def test_accepts_http_prefix_and_trailing_slash(self):
        clock, port = FakeClock(), free_port()
        try:
            proc = run_helper("http://" + clock.address + "/", port)
            self.assertTrue(wait_listening(port))
            urllib.request.urlopen("http://127.0.0.1:%d/callback?code=ok" % port, timeout=10).read()
            out, _ = proc.communicate(timeout=15)
            self.assertEqual(proc.returncode, 0, out)
        finally:
            clock.close()


class EmbeddedCopyTests(unittest.TestCase):
    def test_header_served_by_the_clock_matches_the_script(self):
        result = subprocess.run([sys.executable, str(TOOLS / "gen_helper_header.py"), "--check"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
