#!/usr/bin/env bash
# Installs a release artifact in clean containers the way a user would, checks
# that every shipped library resolves there, and launches the app with
# packaging/smoke-test.sh. The in-container steps are packaging/install-check.sh.
#
#   packaging/install-test.sh <artifact> [image ...]
#
# Default images by artifact:
#   openpowermac_*.deb                       Debian 12, Debian 13, Ubuntu 22.04, Ubuntu 24.04 (apt resolves the Depends)
#   OpenPowerMac-*-linux-x64.tar.gz          Fedora, Arch
#   OpenPowerMac-*-linux-musl-x64.tar.gz     Alpine
# Exit: 0 every image passed | 1 something failed | 2 no container runtime
set -uo pipefail
cd "$(dirname "$0")/.."
artifact=$(realpath "${1:?usage: install-test.sh <artifact> [image ...]}")
shift
images=("$@")
if (( ! ${#images[@]} )); then
    case $artifact in
        *.deb) images=(docker.io/library/debian:bookworm docker.io/library/debian:trixie
                       docker.io/library/ubuntu:22.04 docker.io/library/ubuntu:24.04) ;;
        *linux-musl-x64.tar.gz) images=(docker.io/library/alpine:3.22) ;;
        *) images=(registry.fedoraproject.org/fedora:latest docker.io/library/archlinux:latest) ;;
    esac
fi
runtime=$(command -v podman || command -v docker || true)
[[ -n $runtime ]] || { echo "no podman or docker" >&2; exit 2; }

# Only what the check reads is handed to the container: the two folders of scripts.
rc=0
for image in "${images[@]}"; do
    printf '\n######## %s\n' "$image"
    "$runtime" run --rm --network=host -e DEBIAN_FRONTEND=noninteractive \
        -v "$artifact":"/tmp/$(basename "$artifact")":ro,Z \
        -v "$PWD/tools":/repo/tools:ro,Z -v "$PWD/packaging":/repo/packaging:ro,Z \
        "$image" sh /repo/packaging/install-check.sh "/tmp/$(basename "$artifact")" || rc=1
done
printf '\n'
if (( rc == 0 )); then echo "INSTALL TEST PASSED on ${#images[@]} image(s): $(basename "$artifact")"
else echo "INSTALL TEST FAILED: $(basename "$artifact")"; fi
exit $rc
