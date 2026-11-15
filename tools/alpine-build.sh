#!/usr/bin/env bash
# Builds OpenPowerMac from source on Alpine (musl) in a throwaway container, the
# way an Alpine user would: Alpine's own compiler and .NET SDK, a clean copy of
# the tracked files, warnings as errors. Then runs what was built: the core's
# tests, and the app itself for a few seconds (tools/launch-check.sh).
#
#   tools/alpine-build.sh [alpine tag, default latest]
#
# Uses a quarter of the cores at low priority unless OPM_JOBS says otherwise.
set -uo pipefail
cd "$(dirname "$0")/.."
tag="${1:-latest}"
out="scratch/alpine"
JOBS="${OPM_JOBS:-$(( ($(nproc) + 3) / 4 ))}"
NICE=(nice -n 10)
command -v taskset >/dev/null && NICE=(taskset -c "0-$((JOBS - 1))" nice -n 10)
runtime=$(command -v podman || command -v docker || true)
[ -n "$runtime" ] || { echo "no podman or docker" >&2; exit 2; }

# The tracked files as they are in the working tree, and new files not ignored: what is
# about to be committed, not only what already is.
rm -rf "$out/src"; mkdir -p "$out/src"
{ git ls-files -z
  git ls-files -z --others --exclude-standard -- core machine capi gui shell tools tests packaging
} | while IFS= read -r -d '' f; do [ -f "$f" ] && printf '%s\0' "$f"; done |
    xargs -0 -r cp --parents -t "$out/src"

"${NICE[@]}" "$runtime" run --rm --network=host -e JOBS="$JOBS" -v "$PWD/$out/src:/src:Z" -w /src \
    "docker.io/library/alpine:$tag" sh -euc '
    set -o pipefail
    apk add --no-cache -q build-base cmake samurai dotnet10-sdk bash icu-libs fontconfig ttf-dejavu harfbuzz sdl3 \
        libx11 libice libsm libxi libxfixes libxcursor libxrandr libxext xvfb dbus > /dev/null
    echo "── Alpine $(cat /etc/alpine-release), $(c++ --version | head -1), .NET SDK $(dotnet --version)"
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release > /dev/null
    cmake --build build --parallel "$JOBS" | tail -1
    ctest --test-dir build --output-on-failure -j "$JOBS" | tail -8
    dotnet build gui -c Release --nologo -v q 2>&1 | grep -E "error|warn|Build succeeded" | sort -u
    bash tools/headless.sh bash tools/launch-check.sh gui/bin/Release/net10.0/OpenPowerMac
' > "$out/build.log" 2>&1
rc=$?
cat "$out/build.log"
if [ $rc -eq 0 ] && grep -q "100% tests passed" "$out/build.log" && grep -q "^LAUNCH CHECK PASSED" "$out/build.log"; then
    echo "ALPINE BUILD PASSED"
else
    echo "ALPINE BUILD FAILED"; exit 1
fi
