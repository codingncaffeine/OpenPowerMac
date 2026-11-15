#!/bin/sh
# Runs INSIDE a clean container (packaging/install-test.sh starts it, with the
# repository at /repo): installs what a user of this distro would, checks that
# every shipped binary finds its libraries here, then launches the app through
# tools/headless.sh and packaging/smoke-test.sh. Exit 0 = everything passed.
set -u
artifact="$1"
. /etc/os-release
if command -v apt-get >/dev/null; then
    apt-get update -qq >/dev/null
    case $artifact in
        *.deb) what="$artifact" ;;
        *) what="libicu-dev libfontconfig1 libx11-6 libice6 libsm6 libxi6 libxfixes3 libxcursor1 libxrandr2 libxext6" ;;
    esac
    # shellcheck disable=SC2086
    if ! apt-get install -y -q $what xvfb dbus > /tmp/install.log 2>&1; then tail -25 /tmp/install.log; exit 1; fi
elif command -v dnf >/dev/null; then
    if ! dnf install -y -q libicu fontconfig freetype libX11 libICE libSM libXi libXfixes libXcursor libXrandr libXext libstdc++ \
            xorg-x11-server-Xvfb dbus-daemon tar gzip findutils > /tmp/install.log 2>&1; then tail -25 /tmp/install.log; exit 1; fi
elif command -v apk >/dev/null; then
    if ! apk add --no-cache -q bash coreutils findutils tar icu-libs libstdc++ libgcc fontconfig ttf-dejavu \
            libx11 libice libsm libxi libxfixes libxcursor libxrandr libxext xvfb dbus > /tmp/install.log 2>&1; then tail -25 /tmp/install.log; exit 1; fi
fi
command -v bash >/dev/null || { echo "FAIL no bash in this image"; exit 1; }

problems=0
case $artifact in
    *.deb)
        echo "installed $(dpkg-query -W -f='${Package} ${Version}' openpowermac) on $PRETTY_NAME"
        [ -x /usr/bin/openpowermac ] || { echo "FAIL /usr/bin/openpowermac is missing"; problems=1; }
        root=/usr/lib/openpowermac ;;
    *)
        mkdir -p /opt/openpowermac && tar -xf "$artifact" -C /opt/openpowermac
        root=$(dirname "$(find /opt/openpowermac -name OpenPowerMac -type f | head -1)")
        echo "extracted $(basename "$artifact") on $PRETTY_NAME" ;;
esac

# Every shipped binary must find its libraries, symbol versions included, on this distro.
# Exception: .NET's LTTng trace provider, which the runtime loads only where LTTng is installed.
for f in "$root/OpenPowerMac" "$root"/*.so*; do
    case $(basename "$f") in libcoreclrtraceptprovider.so) continue ;; esac
    bad=$(ldd "$f" 2>&1 | grep -E "not found|Error relocating" || true)
    if [ -n "$bad" ]; then
        echo "FAIL $(basename "$f") does not load here:"; echo "$bad" | sed 's/^/        /'
        problems=1
    fi
done
[ $problems = 1 ] || echo "ok   every shipped library resolves"

bash /repo/tools/headless.sh bash /repo/packaging/smoke-test.sh "$artifact" || problems=1
exit $problems
