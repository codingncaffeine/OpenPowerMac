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
failed=0
ok()   { printf '  ok    %s\n' "$*"; }
fail() { printf '  FAIL  %s\n' "$*"; failed=1; }

# A font the font cache has never seen, in the user's font folder, as after
# installing one: fontconfig then reads it with FreeType inside the app, and
# FreeType 2.14 and later brings the system's HarfBuzz into the process at that
# moment. The app ships its own HarfBuzz and must come up beside the system's.
font=$(find /usr/share/fonts -name '*.ttf' 2>/dev/null | head -1)
if [ -n "$font" ]; then mkdir -p "$XDG_DATA_HOME/fonts" && cp "$font" "$XDG_DATA_HOME/fonts/launch-check-unseen.ttf"; fi

"$app" > "$out" 2>&1 &
pid=$!
sleep "$secs"
if kill -0 $pid 2>/dev/null; then
  ok "still running after $secs s"
  if grep -q 'libharfbuzz\.so' "/proc/$pid/maps" 2>/dev/null; then ok "the system's HarfBuzz is in the process, beside the app's own"
  else ok "(the system's HarfBuzz was not loaded here: no such library, or a FreeType that does not open it)"; fi
  kill $pid 2>/dev/null
  wait $pid 2>/dev/null
else
  wait $pid 2>/dev/null; rc=$?
  fail "exited within $secs s (code $rc)"; sed 's/^/           | /' "$out" | head -20
fi
[ -f "$log" ] || { fail "no log at $log (startup never got that far)"; exit 1; }
line() { grep -o -- "$1" "$log" | head -1; }
l=$(line 'app startup: .*');                                   [ -n "$l" ] && ok "$l" || fail "no version line in the log"
l=$(line 'text shaper: own HarfBuzz opened first .*');           [ -n "$l" ] && ok "$l" || fail "the app's own HarfBuzz was not opened first: $(line 'text shaper: .*')"
l=$(line 'core: loaded .* (machine import ok)');               [ -n "$l" ] && ok "$l" || fail "the emulator core did not load: $(line 'core: .*')"
l=$(line 'SDL3: loaded .* (audio import ok)');                 [ -n "$l" ] && ok "$l" || fail "SDL3 did not load: $(line 'SDL3: .*')"
l=$(line 'window: platform handle .*');                        [ -n "$l" ] && ok "$l" || fail "the main window never opened"
if grep -qE 'UNHANDLED|FATAL' "$log"; then fail "the app logged an error:"; grep -E 'UNHANDLED|FATAL' "$log" | sed 's/^/           | /' | head -5; fi
[ $failed = 0 ] && echo "LAUNCH CHECK PASSED" || echo "LAUNCH CHECK FAILED"
exit $failed
