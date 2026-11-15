#!/usr/bin/env bash
# Builds the Linux release artifacts into packaging/out/:
#   OpenPowerMac-<ver>-linux-x64.tar.gz        self-contained, extract anywhere (glibc: Debian, Ubuntu, Fedora, Arch, ...)
#   OpenPowerMac-<ver>-linux-musl-x64.tar.gz   the same for musl systems (Alpine)
#   openpowermac_<ver>_amd64.deb               system install for Debian and Ubuntu (/usr/lib/openpowermac)
#
#   packaging/build-release.sh            all three
#   packaging/build-release.sh glibc      the glibc tarball and the .deb only
#   packaging/build-release.sh musl       the musl tarball only
#
# The native libraries (the emulator core and SDL3) are built in containers, never
# on this machine: against glibc 2.28 for the glibc artifacts so they load on
# mainstream distros, and in Alpine for musl. See packaging/native/build.sh.
# Then run packaging/smoke-test.sh on every artifact before it goes anywhere.
set -euo pipefail
cd "$(dirname "$0")/.."
WHAT="${1:-all}"
VER=$(grep -oPm1 '(?<=<Version>)[^<]+' gui/OpenPowerMac.Gui.csproj)
OUT=packaging/out
CACHE=packaging/cache
GLIBC_IMAGE=quay.io/pypa/manylinux_2_28_x86_64:latest
MUSL_IMAGE=docker.io/library/alpine:3.22
JOBS="${OPM_JOBS:-$(( ($(nproc) + 3) / 4 ))}"
NICE=(nice -n 10)
command -v taskset >/dev/null && NICE=(taskset -c "0-$((JOBS - 1))" nice -n 10)
if ! command -v dotnet >/dev/null && [ -x "$HOME/.dotnet/dotnet" ]; then
  export DOTNET_ROOT="$HOME/.dotnet" PATH="$HOME/.dotnet:$PATH"
fi
CONTAINER=$(command -v podman || command -v docker || true)
[ -n "$CONTAINER" ] || { echo "!! no podman or docker: the native libraries cannot be built for release" >&2; exit 4; }
for tool in dotnet dpkg-deb tar; do
  command -v "$tool" >/dev/null || { echo "!! $tool is missing" >&2; exit 4; }
done

# A full run starts from an empty folder; a one-flavor run replaces only its own artifacts.
mkdir -p "$OUT" "$CACHE"
case $WHAT in
  all)   rm -rf "$OUT" && mkdir -p "$OUT" ;;
  glibc) rm -rf "$OUT/native-glibc" "$OUT/OpenPowerMac-$VER-linux-x64" "$OUT/OpenPowerMac-$VER-linux-x64.tar.gz" "$OUT/openpowermac_${VER}_amd64.deb" ;;
  musl)  rm -rf "$OUT/native-musl" "$OUT/OpenPowerMac-$VER-linux-musl-x64" "$OUT/OpenPowerMac-$VER-linux-musl-x64.tar.gz" ;;
  *)     echo "usage: build-release.sh [all|glibc|musl]" >&2; exit 2 ;;
esac
# Stale obj/ output can publish XAML that was never compiled; every release starts from clean trees.
rm -rf gui/bin gui/obj

build_native() {   # build_native <image> <dir>
  mkdir -p "$2"
  "${NICE[@]}" "$CONTAINER" run --rm --network=host -e JOBS="$JOBS" \
      -v "$PWD":/src:ro,Z -v "$PWD/$2":/out:Z -v "$PWD/$CACHE":/cache:Z \
      "$1" /src/packaging/native/build.sh > "$2/build.log" 2>&1 || { tail -40 "$2/build.log"; return 1; }
  grep '^── ' "$2/build.log" || true
  test -f "$2/libopmcapi.so" && test -f "$2/libSDL3.so.0"
}

publish() {        # publish <rid> <native dir> <publish dir>
  "${NICE[@]}" dotnet publish gui/OpenPowerMac.Gui.csproj -c Release -r "$1" --self-contained true \
      -p:DebugType=none -o "$3" -v q --nologo
  # The publish step copies whatever core this machine last built; the release ships the container's.
  cp -f "$2/libopmcapi.so" "$2/libSDL3.so.0" "$3/"
  rm -f "$3/build.log"
  cp packaging/README.txt LICENSE THIRD-PARTY-NOTICES.md "$3/"
  cp packaging/icons/openpowermac-256.png "$3/openpowermac.png"
  grep -c CompiledAvaloniaXaml "$3/OpenPowerMac.dll" >/dev/null || { echo "!! $1: the published app has no compiled XAML" >&2; return 1; }
}

if [ "$WHAT" = all ] || [ "$WHAT" = glibc ]; then
  echo "── native libraries for glibc ($GLIBC_IMAGE)"
  build_native "$GLIBC_IMAGE" "$OUT/native-glibc"
  echo "── publish $VER, linux-x64"
  publish linux-x64 "$OUT/native-glibc" "$OUT/OpenPowerMac-$VER-linux-x64"
  tar -C "$OUT" -czf "$OUT/OpenPowerMac-$VER-linux-x64.tar.gz" "OpenPowerMac-$VER-linux-x64"

  echo "── deb"
  DEB=$OUT/debroot
  mkdir -p "$DEB/DEBIAN" "$DEB/usr/lib/openpowermac" "$DEB/usr/bin" "$DEB/usr/share/applications" \
           "$DEB/usr/share/doc/openpowermac" "$DEB/usr/share/metainfo"
  cp -a "$OUT/OpenPowerMac-$VER-linux-x64/." "$DEB/usr/lib/openpowermac/"
  rm -f "$DEB/usr/lib/openpowermac/README.txt" "$DEB/usr/lib/openpowermac/LICENSE" \
        "$DEB/usr/lib/openpowermac/THIRD-PARTY-NOTICES.md"
  cp packaging/README.txt "$DEB/usr/share/doc/openpowermac/README.txt"
  cp LICENSE "$DEB/usr/share/doc/openpowermac/copyright"
  cp THIRD-PARTY-NOTICES.md "$DEB/usr/share/doc/openpowermac/THIRD-PARTY-NOTICES.md"
  cp packaging/io.github.codingncaffeine.OpenPowerMac.metainfo.xml "$DEB/usr/share/metainfo/"
  cp packaging/io.github.codingncaffeine.OpenPowerMac.desktop "$DEB/usr/share/applications/"
  for size in 16 24 32 48 64 128 256; do
    install -Dm644 "packaging/icons/openpowermac-$size.png" "$DEB/usr/share/icons/hicolor/${size}x${size}/apps/openpowermac.png"
  done
  printf '#!/bin/sh\nexec /usr/lib/openpowermac/OpenPowerMac "$@"\n' > "$DEB/usr/bin/openpowermac"
  chmod 755 "$DEB/usr/bin/openpowermac"
  cat > "$DEB/DEBIAN/control" <<CTRL
Package: openpowermac
Version: $VER
Section: otherosfs
Priority: optional
Architecture: amd64
Installed-Size: $(du -sk "$DEB/usr" | cut -f1)
Depends: libc6 (>= 2.28), libgcc-s1, libstdc++6, libicu76 | libicu74 | libicu72 | libicu70 | libicu67, libx11-6, libx11-xcb1, libxcb1, libice6, libsm6, libxi6, libxcursor1, libxext6, libxrandr2, libxrender1, libxfixes3, libfontconfig1, libfreetype6, zlib1g
Recommends: libpulse0, libasound2t64 | libasound2, libgl1, libegl1, xdg-utils
Maintainer: OpenPowerMac <codingncaffeine@users.noreply.github.com>
Homepage: https://github.com/codingncaffeine/OpenPowerMac
Description: Power Macintosh G4 emulator written from scratch
 OpenPowerMac emulates the Power Mac G4 (AGP Graphics): a PowerPC G4 processor
 with AltiVec, the board's storage, USB and sound, and a Rage 128 Pro graphics
 card with 2D and 3D acceleration. It starts Mac OS 9 from a CD, installs it
 on a disk it creates, and runs it from there.
 .
 It contains no original Macintosh code; you supply the Boot ROM of the machine
 you own, and your own system disks.
 .
 SDL3, used for sound output, ships bundled.
CTRL
  dpkg-deb --build --root-owner-group "$DEB" "$OUT/openpowermac_${VER}_amd64.deb" > /dev/null
  rm -rf "$DEB"
fi

if [ "$WHAT" = all ] || [ "$WHAT" = musl ]; then
  echo "── native libraries for musl ($MUSL_IMAGE)"
  build_native "$MUSL_IMAGE" "$OUT/native-musl"
  echo "── publish $VER, linux-musl-x64"
  rm -rf gui/bin gui/obj
  publish linux-musl-x64 "$OUT/native-musl" "$OUT/OpenPowerMac-$VER-linux-musl-x64"
  tar -C "$OUT" -czf "$OUT/OpenPowerMac-$VER-linux-musl-x64.tar.gz" "OpenPowerMac-$VER-linux-musl-x64"
fi

# The clean above removed the dev build's intermediates; nothing else is touched (bin/ still runs).
echo "── artifacts:"
( cd "$OUT" && ls -sh1 -- *.tar.gz *.deb 2>/dev/null && sha256sum -- *.tar.gz *.deb 2>/dev/null | tee SHA256SUMS >/dev/null )
