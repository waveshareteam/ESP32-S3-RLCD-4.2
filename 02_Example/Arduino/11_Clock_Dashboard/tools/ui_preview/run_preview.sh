#!/usr/bin/env bash
# Builds and runs the host-side UI preview, writing out/*.pgm.
# Run inside Linux or WSL, then convert with:  python to_png.py out/*.pgm
set -euo pipefail
cd "$(dirname "$0")"
SRC="${U8G2_SRC:-../../../../../01_Arduino_Libraries/U8g2/src}"
./build_u8g2.sh "$SRC" > /dev/null
g++ -std=c++17 -O1 -Wall -Wno-unused-function -Wno-unused-variable \
    -I"$SRC" -I. -I../.. \
    preview.cpp build/u8g2/*.o -o build/preview -lm
./build/preview
