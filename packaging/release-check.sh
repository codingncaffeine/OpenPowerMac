#!/usr/bin/env bash
# The whole release check in one command: builds every artifact from clean trees,
# launches the glibc ones here, and installs and launches each one in clean
# containers (the .deb on Debian 12 and 13 and Ubuntu 22.04 and 24.04, the glibc
# tarball on Fedora and Arch, the musl tarball on Alpine 3.22 and the latest
# Alpine). The container runs go three at a time. Logs land in packaging/out/logs/.
#
#   packaging/release-check.sh          exit 0 = every artifact passed everything
#
# A release is tagged only from a commit this has passed on.
set -uo pipefail
cd "$(dirname "$0")/.."
VER=$(grep -oPm1 '(?<=<Version>)[^<]+' gui/OpenPowerMac.Gui.csproj)
OUT=packaging/out
LOGS=$OUT/logs

# What gets tested must be what gets tagged: tracked files have to match a commit.
COMMIT=$(git rev-parse --short HEAD)
if [ -n "$(git status --porcelain --untracked-files=no)" ] && [ "${OPM_ALLOW_DIRTY:-}" != 1 ]; then
    echo "RELEASE CHECK REFUSED: uncommitted changes to tracked files (OPM_ALLOW_DIRTY=1 for a trial run):"
    git status --short --untracked-files=no
    exit 3
fi

packaging/build-release.sh all || { echo "RELEASE CHECK FAILED: the build"; exit 1; }
if [ "$(git rev-parse --short HEAD)" != "$COMMIT" ] || { [ -n "$(git status --porcelain --untracked-files=no)" ] && [ "${OPM_ALLOW_DIRTY:-}" != 1 ]; }; then
    echo "RELEASE CHECK FAILED: the tree changed while the artifacts were being built"; exit 1
fi
mkdir -p "$LOGS"
DEB=$OUT/openpowermac_${VER}_amd64.deb
TAR=$OUT/OpenPowerMac-$VER-linux-x64.tar.gz
MUSL=$OUT/OpenPowerMac-$VER-linux-musl-x64.tar.gz

declare -A runs
JOBS=3
start() {
    local name=$1; shift
    while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do wait -n; done
    ( "$@" > "$LOGS/$name.log" 2>&1 ) & runs[$name]=$!
}
start smoke-here            tools/headless.sh packaging/smoke-test.sh "$TAR" "$DEB"
start deb-debian-12         packaging/install-test.sh "$DEB" docker.io/library/debian:bookworm
start deb-debian-13         packaging/install-test.sh "$DEB" docker.io/library/debian:trixie
start deb-ubuntu-22.04      packaging/install-test.sh "$DEB" docker.io/library/ubuntu:22.04
start deb-ubuntu-24.04      packaging/install-test.sh "$DEB" docker.io/library/ubuntu:24.04
start tarball-fedora        packaging/install-test.sh "$TAR" registry.fedoraproject.org/fedora:latest
start tarball-arch          packaging/install-test.sh "$TAR" docker.io/library/archlinux:latest
start tarball-musl-alpine-3.22    packaging/install-test.sh "$MUSL" docker.io/library/alpine:3.22
start tarball-musl-alpine-latest  packaging/install-test.sh "$MUSL" docker.io/library/alpine:latest

failed=0
for name in $(printf '%s\n' "${!runs[@]}" | sort); do
    if wait "${runs[$name]}"; then printf '  ok    %s\n' "$name"
    else printf '  FAIL  %s   (see %s)\n' "$name" "$LOGS/$name.log"; failed=1; fi
done
echo
( cd "$OUT" && cat SHA256SUMS )
if (( failed )); then echo "RELEASE CHECK FAILED: do NOT release $VER"; exit 1; fi
echo "RELEASE CHECK PASSED: $VER at $COMMIT"
