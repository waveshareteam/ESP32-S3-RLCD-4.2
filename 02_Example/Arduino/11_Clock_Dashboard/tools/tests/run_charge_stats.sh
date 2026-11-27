#!/usr/bin/env bash
# Detection latency and false-event statistics for charge.h over random voltage traces
# (see charge_stats.cpp).  Run inside Linux or WSL.
#
#   bash run_charge_stats.sh [sequences]            all scenarios (300 sequences each by default)
#   bash run_charge_stats.sh 300 <n> <seed>         trace scenario [n] on sequence <seed>
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wno-unused-lambda-capture -o build/charge_stats charge_stats.cpp -lm
./build/charge_stats "$@"
