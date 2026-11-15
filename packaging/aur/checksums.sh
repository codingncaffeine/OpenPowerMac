#!/usr/bin/env bash
# Writes the PKGBUILD's sha256sums from the files they describe: the release
# tarball in packaging/out/ and the desktop entry, metainfo and icons as they
# are in this checkout (the recipe downloads those from the release's tag).
#
#   packaging/aur/checksums.sh        after packaging/build-release.sh
set -euo pipefail
cd "$(dirname "$0")/.."
ver=$(grep -oPm1 '(?<=^pkgver=).+' aur/PKGBUILD)
tarball="out/OpenPowerMac-$ver-linux-x64.tar.gz"
[ -f "$tarball" ] || { echo "no $tarball: build the release first" >&2; exit 1; }
files=("$tarball" io.github.codingncaffeine.OpenPowerMac.desktop io.github.codingncaffeine.OpenPowerMac.metainfo.xml)
for size in 16 24 32 48 64 128 256; do files+=("icons/openpowermac-$size.png"); done
{
  first=1
  for f in "${files[@]}"; do
    sum=$(sha256sum "$f" | cut -d' ' -f1)
    if (( first )); then printf "sha256sums=('%s'" "$sum"; first=0; else printf "\n            '%s'" "$sum"; fi
  done
  printf ')\n'
} > out/sha256sums.txt
# Replace the block from "sha256sums=(" to its closing parenthesis.
awk -v block="$(cat out/sha256sums.txt)" '
  /^sha256sums=\(/ { print block; skipping = 1 }
  skipping { if (/\)[[:space:]]*$/) skipping = 0; next }
  { print }' aur/PKGBUILD > out/PKGBUILD.new
n=$(grep -c "^pkgname=" out/PKGBUILD.new)
[ "$n" = 1 ] && [ "$(grep -cE "^ {12}'[0-9a-f]{64}'" out/PKGBUILD.new)" = $(( ${#files[@]} - 1 )) ] ||
    { echo "the rewritten recipe does not look right; aur/PKGBUILD left as it was" >&2; exit 1; }
cp out/PKGBUILD.new aur/PKGBUILD && rm -f out/PKGBUILD.new out/sha256sums.txt
echo "aur/PKGBUILD: checksums written for $ver"
