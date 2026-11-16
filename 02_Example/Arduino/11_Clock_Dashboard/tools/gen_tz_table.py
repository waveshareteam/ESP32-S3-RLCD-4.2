#!/usr/bin/env python3
"""Generate tz_table.cpp: an IANA zone name -> POSIX TZ string lookup table.

The firmware asks Open-Meteo for the IANA zone name of your location (for
example "Australia/Perth"), then needs a POSIX TZ string ("AWST-8") to hand to
tzset() so DST changes are handled by the C library with no further network
calls.  The POSIX string is the footer of each compiled TZif file in the tz
database, so this script just reads those footers.

Usage:
    python3 gen_tz_table.py [zoneinfo_dir] > ../tz_table.cpp

zoneinfo_dir defaults to /usr/share/zoneinfo (Linux, macOS, WSL).  On Windows
you can `pip install tzdata` and point at <site-packages>/tzdata/zoneinfo.
"""
import os
import sys

SKIP_NAMES = {
    "Factory", "localtime", "posixrules", "leapseconds", "SECURITY",
}


def tzif_footer(path):
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(b"TZif") or not data.endswith(b"\n"):
        return None
    body = data[:-1]
    return body[body.rfind(b"\n") + 1:].decode("ascii")


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else "/usr/share/zoneinfo"
    version = ""
    ver_file = os.path.join(root, "+VERSION")
    if os.path.exists(ver_file):
        version = open(ver_file).read().strip()

    zones = {}
    for dirpath, _dirs, files in os.walk(root):
        rel = os.path.relpath(dirpath, root).replace("\\", "/")
        if rel.split("/")[0] in ("posix", "right"):
            continue
        for fn in files:
            name = fn if rel == "." else rel + "/" + fn
            if "." in fn or name in SKIP_NAMES:
                continue
            footer = tzif_footer(os.path.join(dirpath, fn))
            if footer:
                zones[name] = footer

    out = []
    out.append("// GENERATED FILE - do not edit.  Regenerate with tools/gen_tz_table.py")
    out.append("// Source: IANA tz database%s, footer line of each compiled TZif file." % (" " + version if version else ""))
    out.append('#include "tz_table.h"')
    out.append("")
    out.append("#include <string.h>")
    out.append("")
    out.append("namespace {")
    out.append("struct TzEntry {")
    out.append("  const char *name;")
    out.append("  const char *posix;")
    out.append("};")
    out.append("")
    out.append("// Sorted by name (strcmp order) for binary search.")
    out.append("const TzEntry kZones[] = {")
    for name in sorted(zones):
        out.append('  {"%s", "%s"},' % (name, zones[name]))
    out.append("};")
    out.append("}  // namespace")
    out.append("")
    out.append("const char *tzPosixForIana(const char *iana) {")
    out.append("  if (!iana || !*iana) return nullptr;")
    out.append("  size_t lo = 0, hi = sizeof(kZones) / sizeof(kZones[0]);")
    out.append("  while (lo < hi) {")
    out.append("    size_t mid = (lo + hi) / 2;")
    out.append("    int c = strcmp(iana, kZones[mid].name);")
    out.append("    if (c == 0) return kZones[mid].posix;")
    out.append("    if (c < 0) hi = mid; else lo = mid + 1;")
    out.append("  }")
    out.append("  return nullptr;")
    out.append("}")
    out.append("")
    out.append('const char *tzFindIanaIgnoreCase(const char *name, char *canonOut, size_t cap) {')
    out.append('  if (!name || !*name) return nullptr;')
    out.append('  for (size_t i = 0; i < sizeof(kZones) / sizeof(kZones[0]); i++) {')
    out.append('    const char *a = name, *b = kZones[i].name;')
    out.append('    while (*a && *b) {')
    out.append('      char ca = *a, cb = *b;')
    out.append("      if (ca >= 'A' && ca <= 'Z') ca = (char)(ca + 32);")
    out.append("      if (cb >= 'A' && cb <= 'Z') cb = (char)(cb + 32);")
    out.append("      if (ca == ' ') ca = '_';")
    out.append('      if (ca != cb) break;')
    out.append('      ++a;')
    out.append('      ++b;')
    out.append('    }')
    out.append('    if (!*a && !*b) {')
    out.append('      if (canonOut && cap) {')
    out.append('        strncpy(canonOut, kZones[i].name, cap - 1);')
    out.append('        canonOut[cap - 1] = 0;')
    out.append('      }')
    out.append('      return kZones[i].posix;')
    out.append('    }')
    out.append('  }')
    out.append('  return nullptr;')
    out.append('}')
    sys.stdout.write("\n".join(out) + "\n")
    sys.stderr.write("wrote %d zones\n" % len(zones))


if __name__ == "__main__":
    main()
