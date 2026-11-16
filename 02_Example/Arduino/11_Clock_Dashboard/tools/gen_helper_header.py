#!/usr/bin/env python3
"""Regenerate spotify_helper_script.h (the copy of tools/spotify_link.py that the
clock serves at /spotify_link.py), or verify it is current:

    python gen_helper_header.py            write ../spotify_helper_script.h
    python gen_helper_header.py --check    exit 1 if the header is out of date
"""
import pathlib
import sys

DELIM = ')PYSCRIPT"'


def build(src: str) -> str:
    if DELIM in src:
        raise SystemExit("spotify_link.py contains the raw-string terminator %s" % DELIM)
    return (
        "// GENERATED FILE - do not edit.  Regenerate with tools/gen_helper_header.py\n"
        "// Source: tools/spotify_link.py, served by the clock at /spotify_link.py\n"
        "#pragma once\n\n"
        'static const char kSpotifyHelperScript[] = R"PYSCRIPT(' + src + ')PYSCRIPT";\n'
    )


def main() -> int:
    here = pathlib.Path(__file__).resolve().parent
    src = (here / "spotify_link.py").read_text(encoding="utf-8").replace("\r\n", "\n")
    want = build(src)
    target = here.parent / "spotify_helper_script.h"
    if "--check" in sys.argv:
        have = target.read_text(encoding="utf-8").replace("\r\n", "\n") if target.exists() else ""
        if have != want:
            print("spotify_helper_script.h is out of date: run tools/gen_helper_header.py")
            return 1
        print("spotify_helper_script.h is current")
        return 0
    target.write_text(want, encoding="utf-8", newline="\n")
    print("wrote", target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
