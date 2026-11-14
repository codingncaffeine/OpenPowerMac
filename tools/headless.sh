#!/usr/bin/env bash
# Runs a command the way every GUI test of OpenPowerMac must run: on a virtual X
# server (no window on the desktop), on its own D-Bus session that can start
# nothing, with private XDG folders (never the real ~/.config/OpenPowerMac), and with
# sound made impossible (tools/silent.sh).
#
#   tools/headless.sh COMMAND [ARGS...]
#   OPM_TEST_HOME=DIR tools/headless.sh ...   keep the scratch profile in DIR (default: a temp dir, removed)
#   OPM_TEST_SCREEN=1280x1024x24              the virtual screen
set -euo pipefail
[ $# -ge 1 ] || { sed -n '2,9p' "$0"; exit 2; }
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${OPM_TEST_HOME:-}" ]; then
  TMP="$(realpath -m "$OPM_TEST_HOME")"; mkdir -p "$TMP"
else
  TMP="$(mktemp -d "${TMPDIR:-/tmp}/opm-headless.XXXXXX")"
  trap 'rm -rf "$TMP"' EXIT
fi
mkdir -p "$TMP/config" "$TMP/data" "$TMP/state" "$TMP/cache" "$TMP/rt"; chmod 700 "$TMP/rt"
export XDG_CONFIG_HOME="$TMP/config" XDG_DATA_HOME="$TMP/data" XDG_STATE_HOME="$TMP/state" \
       XDG_CACHE_HOME="$TMP/cache" XDG_RUNTIME_DIR="$TMP/rt"
export NO_AT_BRIDGE=1 GDK_BACKEND=x11
unset WAYLAND_DISPLAY DISPLAY
# The X server picks its own free display number and reports it (-displayfd), so
# runs started at the same moment can never end up sharing one server.
export OPM_TEST_SCREEN="${OPM_TEST_SCREEN:-1280x1024x24}" OPM_TEST_TMP="$TMP" OPM_TEST_BUS="$HERE/isolated-bus.conf"
"$HERE/silent.sh" bash -c '
  rm -f "$OPM_TEST_TMP/display"
  Xvfb -displayfd 3 -screen 0 "$OPM_TEST_SCREEN" -nolisten tcp 3>"$OPM_TEST_TMP/display" 2>/dev/null &
  server=$!
  for _ in $(seq 200); do [ -s "$OPM_TEST_TMP/display" ] && break; sleep 0.05; done
  [ -s "$OPM_TEST_TMP/display" ] || { echo "headless: the virtual X server did not start" >&2; kill $server 2>/dev/null; exit 4; }
  export DISPLAY=":$(cat "$OPM_TEST_TMP/display")"
  dbus-run-session --config-file="$OPM_TEST_BUS" -- "$@"; rc=$?
  kill $server 2>/dev/null; wait $server 2>/dev/null
  exit $rc
' headless "$@"
