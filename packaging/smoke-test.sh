#!/usr/bin/env bash
# Smoke-tests a built OpenPowerMac artifact: unpacks it, checks what is inside,
# then LAUNCHES it and requires it to stay up, load its own emulator core and
# its bundled SDL3, open its window, and write only to a scratch profile. A
# package can build and install cleanly and still not start; only running it
# shows that.
#
#   tools/headless.sh packaging/smoke-test.sh <artifact> [...]
#     <artifact>: a release .tar.gz, a .deb, or an Arch .pkg.tar.zst
#
# Run it through tools/headless.sh: that supplies the virtual display, the
# private session bus and the wall that makes sound impossible. The launch is
# refused when a sound device is reachable.
# Exit: 0 all passed | 1 a check failed | 2 the launch could not be run
set -uo pipefail

MIN_ASSEMBLIES=150
LAUNCH_SECONDS=15

fail() { printf '  FAIL  %s\n' "$*"; FAILED=1; }
ok()   { printf '  ok    %s\n' "$*"; }

smoke_one() {
    local artifact="$1" tmp payload
    FAILED=0
    printf '\n== %s ==\n' "$(basename "$artifact")"
    [[ -f $artifact ]] || { fail "no such file"; return 1; }
    tmp=$(mktemp -d) || return 1
    trap 'rm -rf "$tmp"; trap - RETURN' RETURN

    case $artifact in
        *.tar.gz|*.tgz|*.pkg.tar.zst|*.pkg.tar.xz) tar -xf "$artifact" -C "$tmp" 2>/dev/null ;;
        *.deb)
            if command -v dpkg-deb >/dev/null; then dpkg-deb -x "$artifact" "$tmp"
            else bsdtar -xOf "$artifact" 'data.tar.*' | bsdtar -xf - -C "$tmp"; fi ;;
        *)  fail "unrecognized artifact type"; return 1 ;;
    esac
    [[ $? -eq 0 ]] || { fail "could not unpack"; return 1; }

    payload=$(find "$tmp" -name OpenPowerMac -type f -perm -u+x -printf '%h\n' 2>/dev/null | head -1)
    [[ -n $payload ]] || { fail "no OpenPowerMac executable in the artifact"; return 1; }
    ok "payload at ${payload#"$tmp"}/"

    local n f
    n=$(find "$payload" -maxdepth 1 -name '*.dll' | wc -l)
    if (( n < MIN_ASSEMBLIES )); then fail "only $n managed assemblies (expected at least $MIN_ASSEMBLIES)"
    else ok "$n managed assemblies"; fi
    for f in OpenPowerMac.dll OpenPowerMac.runtimeconfig.json OpenPowerMac.deps.json System.Private.CoreLib.dll \
             Avalonia.Base.dll libcoreclr.so libhostfxr.so libhostpolicy.so libSkiaSharp.so \
             libHarfBuzzSharp.so libopmcapi.so; do
        [[ -f "$payload/$f" ]] && ok "$f" || fail "$f is missing"
    done
    # The tarballs and the .deb carry SDL3; an Arch package uses the system's.
    local bundled_sdl=1
    case $artifact in *.pkg.tar.*) bundled_sdl=0 ;; esac
    if (( bundled_sdl )); then
        [[ -f "$payload/libSDL3.so.0" ]] && ok "libSDL3.so.0 (bundled)" || fail "libSDL3.so.0 is missing"
    else
        [[ -f "$payload/libSDL3.so.0" ]] && fail "libSDL3.so.0 is in a package that depends on the system's"
    fi
    case $artifact in *.deb|*.pkg.tar.*)
        for f in usr/bin/openpowermac usr/share/applications/io.github.codingncaffeine.OpenPowerMac.desktop \
                 usr/share/icons/hicolor/256x256/apps/openpowermac.png \
                 usr/share/metainfo/io.github.codingncaffeine.OpenPowerMac.metainfo.xml; do
            [[ -e "$tmp/$f" ]] && ok "/$f" || fail "/$f is missing"
        done ;;
    esac

    # ---- launch -----------------------------------------------------------------
    if [[ -z ${DISPLAY:-} ]]; then
        printf '  SKIPPED  launch: no DISPLAY. Run through tools/headless.sh.\n'
        return 2
    fi
    if [[ -n "$(ls -A /dev/snd 2>/dev/null)" ]]; then
        printf '  SKIPPED  launch: a sound device is reachable. Run through tools/headless.sh.\n'
        return 2
    fi
    # OpenPowerMac keeps settings under $XDG_CONFIG_HOME/OpenPowerMac, its logs under
    # $XDG_STATE_HOME/OpenPowerMac and diagnostics under $XDG_DATA_HOME/OpenPowerMac.
    # Every XDG folder points into the scratch tree. Where a real one exists and
    # bubblewrap works, the launch sees an empty trap folder in its place: the real one
    # cannot be written, and anything in the trap afterwards was written by this launch
    # and by nothing else. A before/after comparison is the fallback; it cannot tell
    # this launch from an OpenPowerMac the user has open.
    local xdg=$tmp/xdg before="" i
    local -a real=("$HOME/.config/OpenPowerMac" "$HOME/.local/state/OpenPowerMac" "$HOME/.local/share/OpenPowerMac")
    local -a wall=() traps=() existed=()
    mkdir -p "$xdg/config" "$xdg/data" "$xdg/cache" "$xdg/state"
    for i in "${!real[@]}"; do
        existed[i]=0; [[ -e ${real[i]} ]] && existed[i]=1
    done
    if command -v bwrap >/dev/null && bwrap --dev-bind / / true 2>/dev/null; then
        for i in "${!real[@]}"; do
            (( existed[i] )) || continue
            traps[i]=$tmp/real-profile-trap-$i; mkdir -p "${traps[i]}"
            (( ${#wall[@]} )) || wall=(bwrap --dev-bind / /)
            wall+=(--bind "${traps[i]}" "${real[i]}")
        done
        (( ${#wall[@]} )) && wall+=(--)
    else
        before=$(for i in "${!real[@]}"; do find "${real[i]}" -type f -printf '%p %T@\n' 2>/dev/null; done | sort | md5sum)
    fi

    # A font the font cache has never seen, in the user's font folder, as after
    # installing one: fontconfig then reads it with FreeType inside the app, and
    # FreeType 2.14 and later brings the system's HarfBuzz into the process at that
    # moment. The app ships its own HarfBuzz and must come up beside the system's.
    local font
    font=$(find /usr/share/fonts -name '*.ttf' 2>/dev/null | head -1)
    [[ -n $font ]] && mkdir -p "$xdg/data/fonts" && cp "$font" "$xdg/data/fonts/smoke-test-unseen.ttf"

    local out=$tmp/launch.out rc start elapsed runner p app_pid="" system_hb=0
    start=$SECONDS
    ( cd "$payload" &&
      exec env XDG_CONFIG_HOME="$xdg/config" XDG_DATA_HOME="$xdg/data" \
               XDG_CACHE_HOME="$xdg/cache" XDG_STATE_HOME="$xdg/state" \
               "${wall[@]}" timeout -k 5 "$LAUNCH_SECONDS" ./OpenPowerMac ) > "$out" 2>&1 &
    runner=$!
    # Shortly before the launch is stopped: which libraries the running app has loaded.
    sleep $((LAUNCH_SECONDS - 4))
    for p in /proc/[0-9]*; do
        [[ $(readlink "$p/exe" 2>/dev/null) == "$payload/OpenPowerMac" ]] && { app_pid=${p#/proc/}; break; }
    done
    [[ -n $app_pid ]] && grep -q 'libharfbuzz\.so' "/proc/$app_pid/maps" 2>/dev/null && system_hb=1
    wait $runner
    rc=$?
    elapsed=$((SECONDS - start))
    # timeout exits 124 when it had to stop the app (137 if TERM was ignored; BusyBox's
    # timeout passes on the stopped app's own 143): the app was still up.
    if (( rc == 124 || rc == 137 || rc == 143 )) && (( elapsed >= LAUNCH_SECONDS )); then
        ok "still running after ${LAUNCH_SECONDS} s"
    else
        fail "exited after ${elapsed} s (code $rc)"
        sed 's/^/           | /' "$out" | head -20
    fi
    if grep -qiE 'does not exist|A fatal error|Failed to load|FileNotFoundException|XamlLoadException|DllNotFoundException' "$out"; then
        fail "host or runtime error in the output:"
        grep -iE 'does not exist|A fatal error|Failed to load|FileNotFoundException|XamlLoadException|DllNotFoundException' "$out" |
            sed 's/^/           | /' | head -5
    fi

    local log=$xdg/state/OpenPowerMac/gui.log line
    if [[ -f $log ]]; then ok "started against the scratch profile"
    else fail "no log under the scratch profile (startup never got that far)"; return 1; fi
    line=$(grep -o 'app startup: .*' "$log" | head -1)
    [[ -n $line ]] && ok "$line" || fail "no version line in the log"
    line=$(grep -o 'text shaper: .*' "$log" | head -1)
    if [[ $line == "text shaper: own HarfBuzz opened first ($payload/libHarfBuzzSharp.so)" ]]; then ok "${line/"$payload"\//<payload>/}"
    else fail "the app's own HarfBuzz was not opened first: ${line:-no text shaper line}"; fi
    line=$(grep -o 'core: .*' "$log" | head -1)
    if [[ $line == "core: loaded $payload/libopmcapi.so (machine import ok)" ]]; then ok "${line/"$payload"\//<payload>/}"
    else fail "the emulator core did not load from the package: ${line:-no core line}"; fi
    line=$(grep -o 'SDL3: .*' "$log" | head -1)
    if [[ $line != *"(audio import ok)"* ]]; then
        fail "SDL3 did not load: ${line:-no SDL3 line}"
    elif (( bundled_sdl )) && [[ $line != "SDL3: loaded $payload/libSDL3.so.0 "* ]]; then
        fail "SDL3 loaded, but not the bundled copy: $line"
    else
        ok "${line/"$payload"\//<payload>/}"
    fi
    if (( system_hb )); then ok "the system's HarfBuzz is in the process, beside the app's own"
    else ok "(the system's HarfBuzz was not loaded here: no such library, or a FreeType that does not open it)"; fi
    line=$(grep -o 'window: platform handle .*' "$log" | head -1)
    [[ -n $line ]] && ok "the main window came up ($line)" || fail "the main window never opened (no 'window: platform handle' line)"
    if grep -qE 'UNHANDLED|FATAL' "$log"; then
        fail "the app logged an error:"; grep -E 'UNHANDLED|FATAL' "$log" | sed 's/^/           | /' | head -5
    fi
    if (( ${#wall[@]} )); then
        local wrote=""
        for i in "${!traps[@]}"; do
            [[ -z "$(ls -A "${traps[i]}")" ]] || wrote+=" ${real[i]}"
        done
        [[ -z $wrote ]] && ok "nothing written to the real profile" || fail "the launch wrote to the REAL profile:$wrote"
    elif [[ -n $before ]]; then
        [[ $before == "$(for i in "${!real[@]}"; do find "${real[i]}" -type f -printf '%p %T@\n' 2>/dev/null; done | sort | md5sum)" ]] ||
            fail "the REAL profile changed during the launch"
    fi
    for i in "${!real[@]}"; do
        (( ! existed[i] )) && [[ -e ${real[i]} ]] && fail "the launch created the REAL profile folder ${real[i]}"
    done
    return $FAILED
}

(( $# )) || { echo "usage: $(basename "$0") <artifact> [...]" >&2; exit 2; }
rc=0
for a in "$@"; do
    smoke_one "$a"
    case $? in 0) ;; 2) (( rc == 0 )) && rc=2 ;; *) rc=1 ;; esac
done
printf '\n'
case $rc in
    0) echo "SMOKE TEST PASSED: every artifact launches." ;;
    2) echo "SMOKE TEST INCOMPLETE: contents checked, the app was never launched." ;;
    *) echo "SMOKE TEST FAILED: do NOT release this artifact." ;;
esac
exit $rc
