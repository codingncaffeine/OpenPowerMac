#!/usr/bin/env python3
"""Drive libopmcapi.so -- the library the APP runs -- the way the app drives it.

The Linux counterpart of tools/capicheck.ps1. g4run and the capi are two
machines assembled from the same parts, and the app adds things g4run does not
do by default: real-time pacing, the drop-box drive seated at power-on, and
its own run loop. This runs that path with no window, so a fault that only
shows in the app can be reproduced and measured from a terminal.

    tools/capicheck.py --rom FILE --ati-rom FILE [--cd FILE] [--hd FILE]
        [--ram MB] [--seconds N] [--no-realtime] [--fast-tb N] [--no-drop]
        [--mouse-at SECONDS] [--key-at SECONDS] [--press T:USAGE[,USAGE...]]...
        [--cd-insert T:FILE]... [--cd-eject T]... [--out DIR] [--lib FILE]

  --hd          copied into --out first (sparse); the original is never written
  --no-realtime pace the timebase from the instruction count (--fast-tb), as
                g4run does, instead of from the host clock
  --no-drop     do not seat the drop-box drive (the app always seats it)
  --mouse-at T  T seconds in: take the diagnostics, hand the mouse 40 reports
                of travel the way a hand would, take them again, and say
                whether the guest's cursor moved
  --key-at T    T seconds in: press and release Caps Lock (harmless), and say
                whether the keyboard report was taken
  --cd-insert   T:FILE puts an image in the CD drive T seconds in, as choosing
                one in the app does; --cd-eject T presses eject. Each prints
                the drive's state, and the last line says whether the guest
                read the disc
  --press T:U   T seconds in: press the HID usage(s) U, hold, release. The power
                key (0x66) and then R (0x15) is a Restart from the keyboard;
                the five-second lines and the final "-- pmu:" line then say
                whether the machine came back up

It opens no sound device (the codec is drained to nowhere, as the app does
with Sound off) and no display, so it needs no wall around it. Prints a line
every five seconds, and at the end the screen's distinct scanlines (a uniform
screen is 1, a desktop is hundreds) and the USB part of the diagnostics.
The last frame is written to OUT/screen.ppm and the full reports to OUT/.
"""
import argparse, ctypes, os, re, subprocess, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--rom", required=True)
ap.add_argument("--ati-rom", required=True)
ap.add_argument("--cd")
ap.add_argument("--hd")
ap.add_argument("--ram", type=int, default=256)
ap.add_argument("--seconds", type=float, default=60)
ap.add_argument("--no-realtime", action="store_true")
ap.add_argument("--fast-tb", type=int, default=4)
ap.add_argument("--no-drop", action="store_true")
ap.add_argument("--mouse-at", type=float)
ap.add_argument("--key-at", type=float)
ap.add_argument("--press", action="append", default=[], metavar="T:USAGE[,USAGE...]",
                help="T seconds in: press these HID usages in order, hold, release in reverse (repeatable). "
                     "0x66 is the power key, 0x15 is R.")
ap.add_argument("--cd-insert", action="append", default=[], metavar="T:FILE",
                help="T seconds in: put this image in the running machine's CD drive (repeatable)")
ap.add_argument("--cd-eject", action="append", default=[], type=float, metavar="T",
                help="T seconds in: press the drive's eject button (repeatable)")
ap.add_argument("--out", default=os.path.join(ROOT, "scratch", "capicheck"))
ap.add_argument("--lib", default=os.path.join(ROOT, "build", "capi", "libopmcapi.so"))
a = ap.parse_args()

for need in (a.rom, a.ati_rom, a.lib) + ((a.cd,) if a.cd else ()) + ((a.hd,) if a.hd else ()):
    if not os.path.exists(need):
        sys.exit(f"capicheck: missing {need}")
os.makedirs(a.out, exist_ok=True)
hd = None
if a.hd:
    hd = os.path.join(a.out, "hd.img")
    subprocess.run(["cp", "--sparse=always", a.hd, hd], check=True)

L = ctypes.CDLL(a.lib)
P, u32, u64, i32 = ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint64, ctypes.c_int32
def fn(name, res, *args):
    f = getattr(L, name); f.restype = res; f.argtypes = list(args); return f
opm_create = fn("opm_create", P, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, u32, u32)
opm_destroy = fn("opm_destroy", None, P)
opm_set_realtime = fn("opm_set_realtime", None, P, i32)
opm_ati_at = fn("opm_ati_at", None, P, u64)
opm_drop_seat = fn("opm_drop_seat", i32, P)
opm_run = fn("opm_run", u64, P, u64)
opm_halted = fn("opm_halted", i32, P)
opm_tb = fn("opm_tb", u64, P)
opm_idle_ns = fn("opm_idle_ns", u64, P)
opm_rt_slips = fn("opm_rt_slips", u64, P)
opm_pc = fn("opm_pc", u32, P)
opm_console = fn("opm_console", u32, P, ctypes.c_char_p, u32)
opm_audio = fn("opm_audio", u32, P, ctypes.c_char_p, u32)
opm_screen = fn("opm_screen", i32, P, ctypes.c_char_p, u32, ctypes.POINTER(u32), ctypes.POINTER(u32))
opm_diag = fn("opm_diag", u32, P, ctypes.c_char_p, u32)
opm_mouse = fn("opm_mouse", None, P, i32, i32, u32)
opm_key_event = fn("opm_key_event", None, P, u32, u32)
opm_cd_present = fn("opm_cd_present", i32, P)
opm_cd_insert = fn("opm_cd_insert", i32, P, ctypes.c_char_p)
opm_cd_eject = fn("opm_cd_eject", i32, P)
opm_cd_state = fn("opm_cd_state", i32, P)
opm_cd_error = fn("opm_cd_error", u32, P, ctypes.c_char_p, u32)
CD_STATE = {0: "no drive", 1: "tray empty", 2: "tray open, the new disc goes in", 3: "disc in, not read yet",
            4: "disc mounted", 5: "a new disc is waiting for the guest to put the current one away"}

enc = lambda s: os.path.abspath(s).encode() if s else None
m = opm_create(enc(a.rom), enc(a.cd), enc(hd), enc(a.ati_rom), a.ram, a.fast_tb)
if not m:
    sys.exit("capicheck: opm_create failed (the ROM must be a 1 MB Sawtooth boot ROM)")
opm_ati_at(m, 1)                       # the app's AtiAt: visible from reset
if not a.no_drop:
    opm_drop_seat(m)                   # the app seats the drop drive before the first instruction
if not a.no_realtime:
    opm_set_realtime(m, 1)
print(f"-- capi machine up: {a.ram} MB, "
      + ("clock paced from the host" if not a.no_realtime else f"fast-tb {a.fast_tb}")
      + (", CD attached" if a.cd and opm_cd_present(m) else ", NO CD" if not a.cd else ", CD REFUSED")
      + (", HD attached" if hd else ", NO HD") + (", drop drive seated" if not a.no_drop else ", no drop drive"),
      flush=True)

conbuf = ctypes.create_string_buffer(65536)
pcm = ctypes.create_string_buffer(44100)
shot = ctypes.create_string_buffer(2048 * 1536 * 4)
diagbuf = ctypes.create_string_buffer(262144)
w, h = u32(0), u32(0)
console = open(os.path.join(a.out, "console.log"), "wb")

def diag(name):
    n = opm_diag(m, diagbuf, len(diagbuf))
    text = diagbuf.raw[:n].decode("latin-1")
    with open(os.path.join(a.out, name), "w") as f:
        f.write(text)
    return text

def usb_part(text):
    lines, keep = [], False
    for l in text.splitlines():
        if l.startswith("-- usb"): keep = True
        elif keep and not l.startswith("--   "): keep = False   # "--   " continues a section
        if keep: lines.append(l)
    return "\n".join(lines)

def cursor(text):
    hit = re.search(r"cursor at \((\d+),(\d+)\)", text)
    return (int(hit.group(1)), int(hit.group(2))) if hit else None

def delivered(text, which):
    hit = re.search(r"-- usb\d \(%s\).*?\n.*?(\d+) reports delivered" % which, text)
    return int(hit.group(1)) if hit else -1

# The app's own loop (MachineSession.Run): chunks aimed at ~15 ms, and between
# chunks the codec and the console are drained and a frame is taken at 30 Hz.
chunk, executed = 1_000_000, 0
t0 = time.monotonic()
tb0, next_line, last_shot = opm_tb(m), 5.0, 0.0
p_time, p_tb, p_idle, p_exec = 0.0, 0, 0, 0
# Scripted input: (time, action) pairs, taken between chunks like the app's queues.
events = []
if a.mouse_at is not None:
    events.append((a.mouse_at, "mouse-before"))
    events += [(a.mouse_at + 0.5 + 0.02 * k, "mouse-move") for k in range(40)]
    events.append((a.mouse_at + 3.0, "mouse-after"))
if a.key_at is not None:
    events += [(a.key_at, "key-before"), (a.key_at + 0.5, "key-down"), (a.key_at + 0.6, "key-up"),
               (a.key_at + 2.0, "key-after")]
for spec in a.press:
    t, _, keys = spec.partition(":")
    usages = [int(k, 0) for k in keys.split(",") if k]
    events += [(float(t) + 0.05 * i, ("down", u)) for i, u in enumerate(usages)]
    events += [(float(t) + 0.4 + 0.05 * i, ("up", u)) for i, u in enumerate(reversed(usages))]
for spec in a.cd_insert:
    t, _, path = spec.partition(":")
    if not os.path.exists(path):
        sys.exit(f"capicheck: missing {path}")
    events.append((float(t), ("cd-insert", os.path.abspath(path))))
events += [(t, ("cd-eject", None)) for t in a.cd_eject]
events.sort(key=lambda e: e[0])
cd_state = None
before = {}
verdicts = []

while True:
    now = time.monotonic() - t0
    if now >= a.seconds:
        break
    c0 = time.monotonic()
    prev = executed
    executed = opm_run(m, chunk)
    dt = (time.monotonic() - c0) * 1000
    if opm_halted(m):
        print(f"-- machine HALTED @{executed:,}")
        break
    if dt < 8 and executed != prev: chunk = min(chunk * 2, 40_000_000)
    elif dt > 30: chunk = max(chunk // 2, 100_000)

    now = time.monotonic() - t0
    while events and events[0][0] <= now:
        _, what = events.pop(0)
        if isinstance(what, tuple) and what[0] == "cd-insert":
            ok = opm_cd_insert(m, what[1].encode())
            why = b""
            if not ok:
                eb = ctypes.create_string_buffer(512)
                why = eb.raw[:opm_cd_error(m, eb, len(eb))]
            print(f"   t={now:5.1f}s  cd insert {os.path.basename(what[1])}: "
                  + (CD_STATE[opm_cd_state(m)] if ok else "REFUSED: " + why.decode(errors="replace")), flush=True)
        elif isinstance(what, tuple) and what[0] == "cd-eject":
            ok = opm_cd_eject(m)
            print(f"   t={now:5.1f}s  cd eject: " + ("out" if ok else "REFUSED, the guest has the tray locked")
                  + f" ({CD_STATE[opm_cd_state(m)]})", flush=True)
        elif isinstance(what, tuple):
            opm_key_event(m, what[1], 1 if what[0] == "down" else 0)
            print(f"   t={now:5.1f}s  key {what[1]:#04x} {what[0]}", flush=True)
        elif what == "mouse-before":
            before["mouse"] = diag("diag-mouse-before.txt")
        elif what == "mouse-move":
            opm_mouse(m, 6, 4, 0)
        elif what == "mouse-after":
            after = diag("diag-mouse-after.txt")
            c0_, c1_ = cursor(before["mouse"]), cursor(after)
            took = delivered(after, "mouse") - delivered(before["mouse"], "mouse")
            verdicts.append(f"mouse: 40 reports handed over (240,160 of travel); the guest took {took}; "
                            f"cursor {c0_} -> {c1_}: " + ("MOVED" if c0_ != c1_ else "DID NOT MOVE"))
        elif what == "key-before":
            before["key"] = diag("diag-key-before.txt")
        elif what == "key-down":
            opm_key_event(m, 57, 1)
        elif what == "key-up":
            opm_key_event(m, 57, 0)
        elif what == "key-after":
            after = diag("diag-key-after.txt")
            took = delivered(after, "keyboard") - delivered(before["key"], "keyboard")
            verdicts.append(f"keyboard: Caps Lock down and up handed over; the guest took {took} report(s): "
                            + ("TAKEN" if took > 0 else "NOT TAKEN"))

    st = opm_cd_state(m)
    if st != cd_state:
        if cd_state is not None:
            print(f"   t={now:5.1f}s  cd: {CD_STATE[st]}", flush=True)
        cd_state = st
    while opm_audio(m, pcm, len(pcm)) > 0:
        pass
    while True:
        n = opm_console(m, conbuf, len(conbuf))
        if n == 0: break
        console.write(conbuf.raw[:n])
        if n + 1 < len(conbuf): break
    if now - last_shot >= 0.033:
        last_shot = now
        opm_screen(m, shot, len(shot), ctypes.byref(w), ctypes.byref(h))

    if now >= next_line:
        next_line += 5.0
        tb, idle = opm_tb(m) - tb0, opm_idle_ns(m)
        d_s, d_tb = now - p_time, tb - p_tb
        mips = (executed - p_exec) / d_s / 1e6
        idle_pct = 100 * (idle - p_idle) / 1e9 / d_s
        p_time, p_tb, p_idle, p_exec = now, tb, idle, executed
        # The guest resets its own timebase mid-boot (Open Firmware), so a
        # delta can be negative; say so instead of printing a negative rate.
        rate = "tb went BACKWARDS (the guest reset it)" if d_tb < 0 else \
               f"tb {d_tb / d_s / 1e6:6.2f} MHz ({d_tb / d_s / 25e6:.2f}x real)"
        print(f"   t={now:5.1f}s  {executed:>15,} insns  {mips:6.1f} MIPS  {rate}  idle {idle_pct:3.0f}%  "
              f"pc={opm_pc(m):08x}", flush=True)

secs = time.monotonic() - t0
final = diag("diag-final.txt")
rc = opm_screen(m, shot, len(shot), ctypes.byref(w), ctypes.byref(h))
print(f"\n-- ran {executed:,} instructions in {secs:.1f} s = {executed / secs / 1e6:.1f} MIPS; "
      f"realtime slips {opm_rt_slips(m)}")
if rc <= 0 or not w.value or not h.value:
    print(f"-- screen: NO FRAMEBUFFER (rc={rc}): the machine has no picture")
else:
    W, H = w.value, h.value
    raw = shot.raw[:W * H * 4]
    rows = {raw[y * W * 4:(y + 1) * W * 4] for y in range(H)}
    print(f"-- screen {W}x{H}: {len(rows)} distinct scanlines "
          f"({'UNIFORM: nothing is being displayed' if len(rows) <= 2 else 'structured'})")
    rgb = bytearray(W * H * 3)
    rgb[0::3], rgb[1::3], rgb[2::3] = raw[2::4], raw[1::4], raw[0::4]
    with open(os.path.join(a.out, "screen.ppm"), "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (W, H) + bytes(rgb))
print(usb_part(final))
for l in final.splitlines():
    if l.startswith("-- pmu:"): print(l)
keep = False
for l in final.splitlines():
    if l.startswith("-- cd ata cell"): keep = True
    elif keep and not l.startswith("--   "): keep = False
    if keep and ("cd ata cell" in l or "medium:" in l or "media:" in l): print(l)
print(f"-- cd drive: {CD_STATE[opm_cd_state(m)]}")
for v in verdicts:
    print("--", v)
print(f"-- reports and the last frame: {a.out}")
console.close()
opm_destroy(m)
