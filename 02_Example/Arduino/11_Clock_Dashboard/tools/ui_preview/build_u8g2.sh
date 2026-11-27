#!/usr/bin/env bash
# Compiles the U8g2 C sources once into build/u8g2/*.o (used by the preview tool).
# Run inside Linux or WSL:  ./build_u8g2.sh [path/to/U8g2/src]
set -euo pipefail
cd "$(dirname "$0")"
SRC="${1:-../../../../../01_Arduino_Libraries/U8g2/src}"
OUT=build/u8g2
mkdir -p "$OUT"
ls "$SRC"/clib/*.c | xargs -P "$(nproc)" -I{} sh -c '
  o="'"$OUT"'/$(basename {} .c).o"
  [ "$o" -nt {} ] || gcc -O1 -w -c {} -I"'"$SRC"'"/clib -o "$o"'
echo "u8g2 objects: $(ls "$OUT" | wc -l)"
