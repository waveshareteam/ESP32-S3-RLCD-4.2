#!/usr/bin/env bash
# Builds and runs the host-side logic tests.  Run inside Linux or WSL.
#
# Needs the ArduinoJson headers (v7).  By default they are looked for in the
# Arduino sketchbook; set ARDUINOJSON_SRC to point at ArduinoJson/src otherwise.
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

AJ="${ARDUINOJSON_SRC:-$HOME/Arduino/libraries/ArduinoJson/src}"
if [ ! -f "$AJ/ArduinoJson.h" ]; then
  echo "ArduinoJson not found at $AJ - set ARDUINOJSON_SRC to its src directory" >&2
  exit 2
fi

FLAGS="-std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-maybe-uninitialized -fsanitize=address,undefined -fno-sanitize-recover=undefined"

mkdir -p build
g++ $FLAGS -isystem "$AJ" -I../.. \
    test_logic.cpp ../../weather.cpp ../../spotify_parse.cpp ../../link_page.cpp \
    -o build/test_logic
g++ $FLAGS -I../.. \
    test_features.cpp ../../settings.cpp ../../tz_table.cpp \
    -o build/test_features
g++ $FLAGS -I../.. test_radio.cpp -o build/test_radio
# The defaults config.h gives (build_defaults.h), four ways: on its own, then with a secrets.h that sets
# everything, one with mistakes in it and one with values on the limits (the override_*.h files play
# the secrets.h).  -Werror: a macro that config.h does not guard with #ifndef is a "redefined" error.
g++ $FLAGS -I../.. -c ../../settings.cpp -o build/settings.o
g++ $FLAGS -I../.. -c ../../tz_table.cpp -o build/tz_table.o
BD="$FLAGS -Werror -DCONFIG_NO_SECRETS -I../.."
g++ $BD test_builddefaults.cpp build/settings.o build/tz_table.o -o build/test_builddefaults_factory
g++ $BD -DTEST_ALL -include override_all.h test_builddefaults.cpp build/settings.o build/tz_table.o -o build/test_builddefaults_all
g++ $BD -DTEST_BAD -include override_bad.h test_builddefaults.cpp build/settings.o build/tz_table.o -o build/test_builddefaults_bad
g++ $BD -DTEST_EDGE -include override_edge.h test_builddefaults.cpp build/settings.o build/tz_table.o -o build/test_builddefaults_edge
# the battery simulation is a tool, not a test: build it so it cannot rot, and run one short trace
g++ -std=c++17 -O1 -Wall -Wextra -I../.. battery_sim.cpp -o build/battery_sim

run ./build/test_logic
run ./build/test_features
run ./build/test_radio
for variant in factory all bad edge; do
  echo "[build defaults: $variant]"
  run ./build/test_builddefaults_$variant
done
./build/battery_sim trace 3 > /dev/null && echo "battery_sim: builds and runs"
