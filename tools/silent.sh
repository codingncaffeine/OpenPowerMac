#!/usr/bin/env bash
# Runs a command with sound made IMPOSSIBLE, not merely configured off: inside
# bubblewrap /dev/snd is an empty tmpfs (no ALSA device can be opened) and the
# user's runtime folder, where the PulseAudio and PipeWire sockets live, is an
# empty tmpfs too. SDL's dummy audio driver and dead server addresses are a
# second layer only. Every test run of the app goes through here.
#
#   tools/silent.sh COMMAND [ARGS...]
#   tools/silent.sh --check     the control: proves no sound device or audio socket is reachable inside
set -euo pipefail
RT="/run/user/$(id -u)"
ENVS=(PULSE_SERVER=unix:/nonexistent/opm-silent PIPEWIRE_REMOTE=/nonexistent/opm-silent
      SDL_AUDIO_DRIVER=dummy SDL_AUDIODRIVER=dummy)

if [ "${1:-}" = "--check" ]; then
  out="$("$0" sh -c 'ls -A /dev/snd; for s in "'"$RT"'"/pulse "'"$RT"'"/pipewire-0 "'"$RT"'"/pipewire-0-manager; do [ -e "$s" ] && echo "$s"; done; true')"
  if [ -n "$out" ]; then echo "silent: CONTROL FAILED, reachable inside the wall:"; echo "$out"; exit 1; fi
  # The control's own control: outside the wall the same probe sees this machine's devices, if it has any.
  if [ -n "$(ls -A /dev/snd 2>/dev/null)" ]; then echo "silent: control ok (/dev/snd has $(ls -A /dev/snd | wc -l) entries outside, none inside)"
  else echo "silent: control ok (no sound devices on this machine at all)"; fi
  exit 0
fi

if ! command -v bwrap >/dev/null || ! bwrap --dev-bind / / true 2>/dev/null; then
  # No usable bubblewrap (missing, or an image that forbids user namespaces). A
  # machine with no sound hardware has nothing to wall off; anywhere that has
  # some, nothing runs without the wall.
  if [ -n "$(ls -A /dev/snd 2>/dev/null)" ]; then echo "silent: bubblewrap cannot run and this machine has sound devices; refusing to run" >&2; exit 3; fi
  exec env "${ENVS[@]}" "$@"
fi
ARGS=(--dev-bind / / --tmpfs /dev/snd)
[ -d "$RT" ] && ARGS+=(--tmpfs "$RT")
exec bwrap "${ARGS[@]}" -- env "${ENVS[@]}" "$@"
