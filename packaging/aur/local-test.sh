#!/usr/bin/env bash
# Builds the AUR package from the artifacts in packaging/out/ and this checkout's
# desktop entry, metainfo and icons, before the release they will be downloaded
# from exists: makepkg uses a source file it already finds under the recipe's
# name. Then launches the package's contents with packaging/smoke-test.sh.
#
#   packaging/aur/local-test.sh [FOLDER]
#     FOLDER: a folder holding an earlier version's downloaded sources (for
#             example a copy of ~/.cache/yay/openpowermac-bin). They are copied
#             in first, so the build runs the way an upgrade does.
#
# Nothing is installed. Run packaging/aur/checksums.sh first.
set -euo pipefail
cd "$(dirname "$0")/../.."
ver=$(grep -oPm1 '(?<=^pkgver=).+' packaging/aur/PKGBUILD)
work=packaging/out/aur
rm -rf "$work"; mkdir -p "$work"
if [ -n "${1:-}" ]; then
  find "$1" -maxdepth 1 -type f ! -name PKGBUILD ! -name .SRCINFO -exec cp -t "$work" {} +
  echo "── earlier sources copied in: $(ls "$work" | wc -l) files"
fi
cp packaging/aur/PKGBUILD "$work/"
cp "packaging/out/OpenPowerMac-$ver-linux-x64.tar.gz" "$work/"
cp packaging/io.github.codingncaffeine.OpenPowerMac.desktop "$work/openpowermac-$ver.desktop"
cp packaging/io.github.codingncaffeine.OpenPowerMac.metainfo.xml "$work/openpowermac-$ver.metainfo.xml"
for size in 16 24 32 48 64 128 256; do cp "packaging/icons/openpowermac-$size.png" "$work/openpowermac-$ver-$size.png"; done
# Every source name must carry the version, or an upgrade reuses the old file.
if grep -A12 '^source=' packaging/aur/PKGBUILD | sed '/^sha256sums/,$d' | grep -E '^\s*("|source=\(")' | grep -v 'pkgver' ; then
  echo "!! a source name without \$pkgver (above)" >&2; exit 1
fi
( cd "$work" && makepkg -f --nodeps > makepkg.log 2>&1 ) || { tail -20 "$work/makepkg.log"; echo "AUR LOCAL TEST FAILED: makepkg"; exit 1; }
pkg=$(ls "$work"/openpowermac-bin-"$ver"-*.pkg.tar.zst)
echo "── built $(basename "$pkg")"
tools/headless.sh packaging/smoke-test.sh "$pkg"
