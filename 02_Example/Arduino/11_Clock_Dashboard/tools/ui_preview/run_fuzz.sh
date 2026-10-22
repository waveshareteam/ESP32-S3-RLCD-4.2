#!/usr/bin/env bash
# Renders thousands of randomised, adversarial frames under AddressSanitizer and
# UBSan to prove ui.cpp cannot overflow a buffer.  Run inside Linux or WSL.
set -euo pipefail
cd "$(dirname "$0")"

# AddressSanitizer binaries can spin forever at startup on recent Linux/WSL2
# kernels with high ASLR entropy; run them with ASLR off where that is allowed.
run() {
  if setarch "$(uname -m)" -R true 2>/dev/null; then
    setarch "$(uname -m)" -R "$@"
  else
    "$@"
  fi
}
SRC="${U8G2_SRC:-../../../../../01_Arduino_Libraries/U8g2/src}"
./build_u8g2.sh "$SRC" > /dev/null
mkdir -p build
g++ -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-unused-variable \
    -fsanitize=address,undefined -fno-sanitize-recover=undefined \
    -I"$SRC" -I. -I../.. \
    fuzz.cpp build/u8g2/*.o -o build/fuzz -lm
run ./build/fuzz "${1:-3000}" "${2:-0}"
