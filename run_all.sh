#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Build and test before any benchmark, fail closed on correctness errors.
echo '=== [1/3] CMake configure and build ==='
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j "${BUILD_JOBS:-2}"
echo '=== [2/3] Correctness tests ==='
ctest --test-dir build --output-on-failure
echo '=== [3/3] Verified experiments and plots ==='
python3 tools/benchmark.py "$@"
