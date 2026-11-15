#!/usr/bin/env bash
# Starts a built app, leaves it up for a few seconds and reads its own log: it
# must still be running, have loaded the emulator core and SDL3, and have
# opened its window. A build can succeed and still not start; this is the check
# for that, on the machine (or in the container) the build was made on.
#
#   tools/headless.sh tools/launch-check.sh APP [SECONDS]
#     APP: the app's executable, e.g. gui/bin/Release/net10.0/OpenPowerMac
#
# Only through tools/headless.sh: it supplies the virtual display, the private
# profile and the wall that makes sound impossible.
# Exit: 0 passed | 1 a check failed | 2 not run through tools/headless.sh
set -uo pipefail
app=${1:?usage: tools/headless.sh tools/launch-check.sh APP [SECONDS]}
secs=${2:-12}
if [ -z "${DISPLAY:-}" ] || [ -z "${OPM_TEST_TMP:-}" ] || [ -n "$(ls -A /dev/snd 2>/dev/null)" ]; then
  echo "launch-check: run it through tools/headless.sh" >&2; exit 2
fi
[ -x "$app" ] || { echo "launch-check: no executable at $app" >&2; exit 1; }

out="$OPM_TEST_TMP/launch.out"
log="$XDG_STATE_HOME/OpenPowerMac/gui.log"
start=$SECONDS
timeout -k 5 "$secs" "$app" > "$out" 2>&1; rc=$?
failed=0
ok()   { printf '  ok    %s\n' "$*"; }
fail() { printf '  FAIL  %s\n' "$*"; failed=1; }

# timeout exits 124 when it had to stop the app (137 if TERM was ignored; BusyBox's
# timeout passes on the stopped app's own 143): the app was still up.
if { [ $rc = 124 ] || [ $rc = 137 ] || [ $rc = 143 ]; } && [ $((SECONDS - start)) -ge "$secs" ]; then ok "still running after $secs s"
else fail "exited after $((SECONDS - start)) s (code $rc)"; sed 's/^/           | /' "$out" | head -20; fi
[ -f "$log" ] || { fail "no log at $log (startup never got that far)"; exit 1; }
line() { grep -o -- "$1" "$log" | head -1; }
l=$(line 'app startup: .*');                                   [ -n "$l" ] && ok "$l" || fail "no version line in the log"
l=$(line 'core: loaded .* (machine import ok)');               [ -n "$l" ] && ok "$l" || fail "the emulator core did not load: $(line 'core: .*')"
l=$(line 'SDL3: loaded .* (audio import ok)');                 [ -n "$l" ] && ok "$l" || fail "SDL3 did not load: $(line 'SDL3: .*')"
l=$(line 'window: platform handle .*');                        [ -n "$l" ] && ok "$l" || fail "the main window never opened"
if grep -qE 'UNHANDLED|FATAL' "$log"; then fail "the app logged an error:"; grep -E 'UNHANDLED|FATAL' "$log" | sed 's/^/           | /' | head -5; fi
[ $failed = 0 ] && echo "LAUNCH CHECK PASSED" || echo "LAUNCH CHECK FAILED"
exit $failed
