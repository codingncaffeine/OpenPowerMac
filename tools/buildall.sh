#!/bin/bash
# Build EVERY artifact this project ships, Release, and prove the shell got the
# right one.
#
#   build-gcc/   MinGW g++ + Ninja  -- a second compiler is a real portability
#                                      check; each rejects what the other allows
#   build/       MSVC multi-config  -- THE SHIPPING BUILD: g4run, opmcapi, tests
#   shell/       .NET net11.0-windows
#
# Two ways this has silently shipped a broken app, both guarded below:
#
#  1. The shell COPIES opmcapi.dll at its own build time, so a native-only
#     rebuild leaves the app running yesterday's machine code -- and
#     `dotnet build` prints "Build succeeded" even when the native build it
#     depends on failed.
#  2. capi/CMakeLists.txt publishes opmcapi.dll into shell/native/. That copy
#     used to be guarded by `if(WIN32)`, and MinGW IS WIN32 -- so building
#     build-gcc/ as a portability check overwrote the MSVC DLL with a gcc one,
#     the shell shipped it, and the emulator stopped booting while every
#     native build and all 4 tests still passed. The CMake guard is now
#     `WIN32 AND MSVC`; step 5 here is the backstop.
#
# ORDER IS LOAD-BEARING: gcc first, MSVC second, so the shipping compiler is
# always the last writer even if a guard regresses. And the DLL gate runs even
# when an earlier stage fails, so a bail-out can never leave a foreign DLL in
# place unreported.
#
#   tools/buildall.sh            # everything
#   tools/buildall.sh --no-gcc   # skip the second compiler (quick iteration)
#
# ⛔ NEVER PUSH AFTER --no-gcc. CI builds with gcc, and gcc rejects things MSVC
# accepts silently -- -Wunused-function is an error there, so deleting the last
# caller of a static helper breaks the Linux jobs while MSVC and all four tests
# pass locally. That is exactly how ed930d3 had to happen. --no-gcc is for
# iterating between measurements, not for the build you commit.
#
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

# cmake is not on PATH in this environment.
CMAKE="/c/Program Files/Microsoft Visual Studio/18/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
CTEST="${CMAKE%cmake.exe}ctest.exe"
[ -x "$CMAKE" ] || { echo "FAIL: cmake not found at $CMAKE"; exit 1; }

# MinGW's cc1plus lives in lib/gcc/..., not in bin/, so Windows resolves its
# libgmp/libmpc/libisl imports through PATH. Without mingw64/bin on PATH the
# compiler fails to start and g++ returns 1 having printed NOTHING AT ALL --
# which reads exactly like a source error on every file in the project.
export PATH="/c/msys64/mingw64/bin:$PATH"

SRC=build/capi/Release/opmcapi.dll
NATIVE=shell/native/opmcapi.dll
SHIPPED=shell/bin/Release/net11.0-windows/opmcapi.dll
# The RELEASE artifact — the folder you would actually run on its own or hand
# to someone. Built and gated on every pass for the same reason the shipped
# copy is: an output refreshed only "when we cut a release" is stale every
# other day, and the one time it finally gets run is the one time nobody can
# say which build it holds.
PUBLISH=shell/publish
PUBLISHED=$PUBLISH/opmcapi.dll

WANT_GCC=1
[ "${1:-}" = "--no-gcc" ] && WANT_GCC=0

RC=0
note() { echo; echo "########## $1"; RC=1; }

echo "===== 1/6  portability check: MinGW g++/Ninja Release ====="
if [ "$WANT_GCC" -eq 1 ] && [ -f build-gcc/build.ninja ]; then
    "$CMAKE" --build build-gcc 2>&1 | tail -25
    [ "${PIPESTATUS[0]}" -eq 0 ] || note "FAIL: MinGW Release build"
else
    echo "(skipped)"
fi

echo "===== 2/6  native Release (MSVC) -- the shipping build ====="
"$CMAKE" --build build --config Release 2>&1 | tail -20
[ "${PIPESTATUS[0]}" -eq 0 ] || note "FAIL: MSVC Release build"

echo "===== 3/6  ctest Release ====="
"$CTEST" --test-dir build -C Release 2>&1 | tail -8
[ "${PIPESTATUS[0]}" -eq 0 ] || note "FAIL: ctest"

echo "===== 4/6  shell Release (.NET) ====="
# --no-incremental: the csproj copies the native DLL in a target that an
# incremental build is happy to consider up to date.
(cd shell && dotnet build -c Release --no-incremental 2>&1 | tail -12)
[ "${PIPESTATUS[0]}" -eq 0 ] || note "FAIL: shell build"

echo "===== 5/6  release artifact (dotnet publish) ====="
# The runnable folder, refreshed on every pass rather than at release time.
# Framework-dependent on purpose: it is the same thing the user already runs,
# so this stays fast enough to sit in the loop after every change, which is the
# only way it is ever actually current. Add -r win-x64 --self-contained if a
# copy has to run on a machine with no .NET.
# ⚠ NO --no-incremental HERE: `dotnet publish` rejects it (MSB1001), unlike
# `dotnet build` above. Step 4 already forced a clean build, so this copies
# fresh outputs — and if it ever decides it is up to date and leaves a stale
# native DLL behind, the md5 gate below is what says so.
(cd shell && dotnet publish -c Release -o publish 2>&1 | tail -8)
[ "${PIPESTATUS[0]}" -eq 0 ] || note "FAIL: shell publish"

echo "===== 6/6  the app ships the MSVC DLL that was just built ====="
# This gate runs unconditionally: if anything above bailed, a foreign or stale
# DLL may be sitting in the shell and the user would find out by watching the
# emulator fail to boot.
for f in "$SRC" "$NATIVE" "$SHIPPED" "$PUBLISHED"; do
    [ -f "$f" ] || note "FAIL: missing $f"
done
if [ -f "$SRC" ] && [ -f "$NATIVE" ] && [ -f "$SHIPPED" ] && [ -f "$PUBLISHED" ]; then
    A=$(md5sum "$SRC"       | cut -d' ' -f1)
    B=$(md5sum "$NATIVE"    | cut -d' ' -f1)
    C=$(md5sum "$SHIPPED"   | cut -d' ' -f1)
    D=$(md5sum "$PUBLISHED" | cut -d' ' -f1)
    echo "  MSVC build   ${A:0:12}  $SRC"
    echo "  shell/native ${B:0:12}  $NATIVE"
    echo "  shipped      ${C:0:12}  $SHIPPED"
    echo "  release      ${D:0:12}  $PUBLISHED"
    [ "$A" = "$B" ] || note "FAIL: shell/native/ holds a DLL that is NOT the MSVC build (gcc build clobbered it?)"
    [ "$A" = "$C" ] || note "FAIL: the app is shipping a STALE or FOREIGN opmcapi.dll"
    [ "$A" = "$D" ] || note "FAIL: the RELEASE artifact holds a STALE or FOREIGN opmcapi.dll"

    # Independent of md5: a gcc-built DLL imports the MinGW runtime. Catches
    # the same class of mistake even if both copies were clobbered together.
    for f in "$NATIVE" "$SHIPPED" "$PUBLISHED"; do
        if grep -aqE 'libstdc\+\+-6\.dll|libgcc_s_seh-1\.dll|libwinpthread-1\.dll' "$f"; then
            note "FAIL: $f imports the MinGW runtime -- it is a gcc build, not MSVC"
        fi
    done
fi

echo
if [ "$RC" -eq 0 ]; then
    echo "ALL GREEN (MinGW Release, MSVC Release, ctest, shell Release, release artifact, DLL is MSVC + fresh everywhere)"
else
    echo "BUILD NOT CLEAN -- see the ########## lines above"
fi
exit "$RC"
