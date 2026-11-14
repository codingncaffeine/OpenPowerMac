#!/usr/bin/env bash
# Builds the native core and the Linux app, runs the core's tests, and
# refreshes the runnable dev build in bin/ at the top of the repo:
#
#   tools/build-linux.sh            then run   bin/openpowermac
#   tools/build-linux.sh --no-test  skip the core's tests
#
# Uses a quarter of the cores at low priority unless OPM_JOBS says otherwise.
set -euo pipefail
cd "$(dirname "$0")/.."
JOBS="${OPM_JOBS:-$(( ($(nproc) + 3) / 4 ))}"
NICE=(nice -n 10)
command -v taskset >/dev/null && NICE=(taskset -c "0-$((JOBS - 1))" nice -n 10)
if ! command -v dotnet >/dev/null && [ -x "$HOME/.dotnet/dotnet" ]; then
  export DOTNET_ROOT="$HOME/.dotnet" PATH="$HOME/.dotnet:$PATH"
fi

GEN=(); command -v ninja >/dev/null && GEN=(-G Ninja)
[ -f build/CMakeCache.txt ] || cmake -S . -B build "${GEN[@]}" -DCMAKE_BUILD_TYPE=Release
"${NICE[@]}" cmake --build build --parallel "$JOBS"
[ "${1:-}" = "--no-test" ] || "${NICE[@]}" ctest --test-dir build --output-on-failure -j "$JOBS"
"${NICE[@]}" dotnet build gui -c Release -v q --nologo

# bin/app is the build output; bin/openpowermac is the one thing to run.
# Not while the app is running from it: the folder is replaced, and a running
# app that then reaches for a file it had not loaded yet finds a different
# build, or nothing — with a guest's disk image open for writing.
if pgrep -f "$PWD/bin/app/OpenPowerMac" >/dev/null 2>&1; then
  echo "build: built, but bin/ was NOT refreshed: the app is running from it. Close it and run this again." >&2
  exit 3
fi
rm -rf bin/app && mkdir -p bin/app
cp -a gui/bin/Release/net10.0/. bin/app/
cat > bin/openpowermac <<'LAUNCH'
#!/bin/sh
exec "$(dirname "$(readlink -f "$0")")/app/OpenPowerMac" "$@"
LAUNCH
chmod +x bin/openpowermac
test -f bin/app/libopmcapi.so || { echo "build: the native core is missing from bin/app" >&2; exit 1; }
echo "build: ok -> bin/openpowermac"
