#!/usr/bin/env bash
# Runs a command behind a wall, so that a test run cannot reach the desktop it
# is started from. Every test run of the app goes through here.
#
# Sound is made IMPOSSIBLE, not merely configured off: inside bubblewrap
# /dev/snd is an empty tmpfs (no ALSA device can be opened) and the user's
# runtime folder, where the PulseAudio and PipeWire sockets live, is an empty
# tmpfs too. SDL's dummy audio driver and dead server addresses are a second
# layer only.
#
# The desktop's processes do not exist inside: the wall has its own process
# numbering, so nothing run here can signal one. A process whose starter has
# gone gets the wall's first process for a parent, where outside it would get
# the manager of the whole desktop session. When the wall ends, everything
# still running inside ends with it.
#
# The folder of the window servers' sockets is the wall's own as well: a
# server started inside puts its socket there, and the desktop's are not in it.
#
#   tools/silent.sh COMMAND [ARGS...]
#   tools/silent.sh --check     the control: proves each of the above from inside
set -euo pipefail
RT="/run/user/$(id -u)"
ENVS=(PULSE_SERVER=unix:/nonexistent/opm-silent PIPEWIRE_REMOTE=/nonexistent/opm-silent
      SDL_AUDIO_DRIVER=dummy SDL_AUDIODRIVER=dummy)
ARGS=(--dev-bind / / --unshare-pid --proc /proc --die-with-parent --tmpfs /dev/snd)
[ -d "$RT" ] && ARGS+=(--tmpfs "$RT")
[ -d /tmp/.X11-unix ] && ARGS+=(--perms 1777 --tmpfs /tmp/.X11-unix)
WALL=1
if ! command -v bwrap >/dev/null || ! bwrap "${ARGS[@]}" true 2>/dev/null; then WALL=0; fi

if [ "${1:-}" = "--check" ]; then
  if [ "$WALL" = 0 ]; then
    if [ -n "$(ls -A /dev/snd 2>/dev/null)" ]; then echo "silent: CONTROL FAILED, bubblewrap cannot run and this machine has sound devices"; exit 1; fi
    echo "silent: control ok (no wall on this machine, and no sound devices at all)"; exit 0
  fi
  out="$("$0" sh -c 'ls -A /dev/snd; for s in "'"$RT"'"/pulse "'"$RT"'"/pipewire-0 "'"$RT"'"/pipewire-0-manager; do [ -e "$s" ] && echo "$s"; done; true')"
  if [ -n "$out" ]; then echo "silent: CONTROL FAILED, reachable inside the wall:"; echo "$out"; exit 1; fi
  # The control's own control: outside the wall the same probe sees this machine's devices, if it has any.
  if [ -n "$(ls -A /dev/snd 2>/dev/null)" ]; then echo "silent: control ok (/dev/snd has $(ls -A /dev/snd | wc -l) entries outside, none inside)"
  else echo "silent: control ok (no sound devices on this machine at all)"; fi

  # A process left without its starter names its parent. It only prints.
  stray='( ( sleep 0.2; exec sh -c "cat /proc/\$PPID/comm" ) & ); sleep 0.6'
  inside="$("$0" sh -c "$stray")"; outside="$(sh -c "$stray")"
  if [ "$inside" != bwrap ]; then echo "silent: CONTROL FAILED, a stray process inside the wall has '$inside' for a parent"; exit 1; fi
  seen="$("$0" sh -c 'ls -d /proc/[0-9]* | wc -l')"
  if [ "$seen" -gt 8 ]; then echo "silent: CONTROL FAILED, $seen processes are visible inside the wall"; exit 1; fi
  echo "silent: control ok (a stray process's parent is the wall inside, '$outside' outside; $seen processes visible inside, $(ls -d /proc/[0-9]* | wc -l) outside)"

  servers="$("$0" sh -c 'ls -A /tmp/.X11-unix 2>/dev/null; true')"
  if [ -n "$servers" ]; then echo "silent: CONTROL FAILED, window server sockets reachable inside the wall:"; echo "$servers"; exit 1; fi
  echo "silent: control ok (no window server socket inside, $(ls -A /tmp/.X11-unix 2>/dev/null | wc -l) outside)"
  exit 0
fi

if [ "$WALL" = 0 ]; then
  # No usable bubblewrap (missing, or an image that forbids user namespaces). A
  # machine with no sound hardware has nothing to wall off; anywhere that has
  # some, nothing runs without the wall.
  if [ -n "$(ls -A /dev/snd 2>/dev/null)" ]; then echo "silent: bubblewrap cannot run and this machine has sound devices; refusing to run" >&2; exit 3; fi
  exec env "${ENVS[@]}" "$@"
fi
exec bwrap "${ARGS[@]}" -- env "${ENVS[@]}" "$@"
