#!/usr/bin/env python3
"""A remote control for a headless machine: the app's library, driven by commands.

capicheck.py runs a fixed script and ends. This keeps ONE machine alive and
takes commands over a socket, so a person (or a script) can look at the
screen, decide, click, look again -- and save a snapshot when the machine is
where the work is. The machine only advances while a command runs it, so the
guest is frozen between commands and nothing is lost by thinking.

    tools/remote.py serve --rom FILE --ati-rom FILE --hd COPY [--cd FILE]
        [--ram MB] [--fast-tb N] [--resume SNAP] [--out DIR] [--lib FILE]
    tools/remote.py send [--out DIR] "COMMAND" ["COMMAND" ...]

  --hd   is WRITTEN by the guest: hand it a copy, never the user's image.
  --resume  start from a snapshot (`snap NAME` below) instead of from power-on;
         --hd must then be a copy of the NAME.hd.img saved beside it.

Commands (one per argument; times are GUEST seconds unless they say host):
  secs S            run until the guest's clock has moved S seconds
  run N             run N million instructions
  shot NAME         write OUT/NAME.png (the picture, pointer included)
  pos               where the guest's pointer is
  move X Y          walk the pointer to an absolute pixel, checking as it goes
  click | dbl       one click, or a double click, where the pointer is
  down | up         the button alone (for a drag: down, move, up)
  key U[,U...]      press HID usages together and release (0x28 Return,
                    0x29 Esc, 0x2c Space, 0xe3 Command, 0x4f-0x52 arrows)
  hold U / release U   one usage going down, or coming up
  type TEXT         text as keystrokes
  snap NAME         save OUT/NAME.snap and the disk as it is now (OUT/NAME.hd.img)
  diag NAME         write the diagnostics to OUT/NAME.txt
  watch X Y         from now on, log every write to the screen pixel (X, Y)
                    with its author: the processor and its pc, a 2D engine
                    operation, a triangle and its state
  watchlog [NAME]   print that log (and write it to OUT/NAME.txt)
  stat              instructions, guest seconds, slips
  rt on|off         pace the guest's clock from the host (the app) or from
                    the instruction count (deterministic, the default)
  perf S            S HOST seconds in the app's own loop; one line a second:
                    MIPS, the guest's clock against real time, idle share,
                    and the sound the guest produced against what a speaker
                    consumes in that second
  wav NAME | wav off   keep the sound the guest produces in OUT/NAME.wav
  drop IMAGE        hand a volume image to the drop-box drive, as the app does
                    after a drop (tools/mkhfs builds one from a folder);
                    `drop` alone says what the drive holds
  quit
"""
import argparse, ctypes, os, re, socket, struct, subprocess, sys, time, wave

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("mode", choices=["serve", "send"])
ap.add_argument("commands", nargs="*")
ap.add_argument("--rom")
ap.add_argument("--ati-rom")
ap.add_argument("--cd")
ap.add_argument("--hd")
ap.add_argument("--ram", type=int, default=1536)
ap.add_argument("--fast-tb", type=int, default=4)
ap.add_argument("--resume")
ap.add_argument("--out", default=os.path.join(ROOT, "scratch", "remote"))
ap.add_argument("--lib", default=os.path.join(ROOT, "build", "capi", "libopmcapi.so"))
a = ap.parse_args()
SOCK = os.path.join(a.out, "remote.sock")

if a.mode == "send":
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.connect(SOCK)
    except OSError as e:
        sys.exit(f"remote: no machine is serving at {SOCK} ({e})")
    s.sendall(("\n".join(a.commands) + "\n").encode())
    s.shutdown(socket.SHUT_WR)
    while True:
        b = s.recv(65536)
        if not b:
            break
        sys.stdout.write(b.decode(errors="replace"))
        sys.stdout.flush()
    sys.exit(0)

for need in (a.rom, a.ati_rom, a.hd, a.lib) + ((a.cd,) if a.cd else ()) + ((a.resume,) if a.resume else ()):
    if not need or not os.path.exists(need):
        sys.exit(f"remote: missing {need}")
os.makedirs(a.out, exist_ok=True)

L = ctypes.CDLL(a.lib)
P, u32, u64, i32 = ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint64, ctypes.c_int32
def fn(name, res, *args):
    f = getattr(L, name); f.restype = res; f.argtypes = list(args); return f
opm_create = fn("opm_create", P, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, u32, u32)
opm_destroy = fn("opm_destroy", None, P)
opm_set_realtime = fn("opm_set_realtime", None, P, i32)
opm_ati_at = fn("opm_ati_at", None, P, u64)
opm_drop_seat = fn("opm_drop_seat", i32, P)
opm_drop_swap = fn("opm_drop_swap", i32, P, ctypes.c_char_p)
opm_drop_state = fn("opm_drop_state", i32, P)
opm_drop_reads = fn("opm_drop_reads", u64, P)
opm_run = fn("opm_run", u64, P, u64)
opm_halted = fn("opm_halted", i32, P)
opm_tb = fn("opm_tb", u64, P)
opm_idle_ns = fn("opm_idle_ns", u64, P)
opm_rt_slips = fn("opm_rt_slips", u64, P)
opm_pc = fn("opm_pc", u32, P)
opm_executed = fn("opm_executed", u64, P)
opm_console = fn("opm_console", u32, P, ctypes.c_char_p, u32)
opm_audio = fn("opm_audio", u32, P, ctypes.c_char_p, u32)
opm_audio_rate = fn("opm_audio_rate", u32, P)
opm_screen = fn("opm_screen", i32, P, ctypes.c_char_p, u32, ctypes.POINTER(u32), ctypes.POINTER(u32))
opm_diag = fn("opm_diag", u32, P, ctypes.c_char_p, u32)
opm_ati_pixel_watch = fn("opm_ati_pixel_watch", i32, P, u32, u32, u32)
opm_ati_pixel_log = fn("opm_ati_pixel_log", u32, P, ctypes.c_char_p, u32)
opm_mouse = fn("opm_mouse", None, P, i32, i32, u32)
opm_key = fn("opm_key", None, P, ctypes.c_char_p)
opm_key_event = fn("opm_key_event", None, P, u32, u32)
opm_snapshot_save = fn("opm_snapshot_save", i32, P, ctypes.c_char_p, ctypes.c_char_p, u32)
opm_snapshot_load = fn("opm_snapshot_load", i32, P, ctypes.c_char_p, ctypes.c_char_p, u32)

enc = lambda s: os.path.abspath(s).encode() if s else None
m = opm_create(enc(a.rom), enc(a.cd), enc(a.hd), enc(a.ati_rom), a.ram, a.fast_tb)
if not m:
    sys.exit("remote: opm_create failed (the ROM must be a 1 MB Sawtooth boot ROM)")
opm_ati_at(m, 1)
opm_drop_seat(m)
if a.resume:
    eb = ctypes.create_string_buffer(512)
    if not opm_snapshot_load(m, enc(a.resume), eb, len(eb)):
        sys.exit("remote: the snapshot does not fit this machine: " + eb.value.decode(errors="replace"))

conbuf = ctypes.create_string_buffer(65536)
pcm = ctypes.create_string_buffer(1 << 20)
shotbuf = ctypes.create_string_buffer(2048 * 1536 * 4)
diagbuf = ctypes.create_string_buffer(262144)
errbuf = ctypes.create_string_buffer(512)
w, h = u32(0), u32(0)
console = open(os.path.join(a.out, "console.log"), "wb")
state = {"buttons": 0, "wav": None, "audio": 0, "rt": False, "chunk": 1_000_000}

def drain():
    """What the app does between chunks: the codec and the console."""
    got = 0
    while True:
        n = opm_audio(m, pcm, len(pcm))
        if n == 0:
            break
        got += n
        if state["wav"]:
            b = bytearray(pcm.raw[:n]); b[0::2], b[1::2] = b[1::2], b[0::2]   # big-endian -> little
            state["wav"].writeframes(bytes(b))
    state["audio"] += got
    while True:
        n = opm_console(m, conbuf, len(conbuf))
        if n == 0:
            break
        console.write(conbuf.raw[:n])
        if n + 1 < len(conbuf):
            break
    return got

def step(insns):
    before = opm_executed(m)
    opm_run(m, insns)
    drain()
    return opm_executed(m) - before

def run_secs(s, host_cap=600.0):
    target, t0 = opm_tb(m) + int(s * 25e6), time.monotonic()
    while opm_tb(m) < target and not opm_halted(m) and time.monotonic() - t0 < host_cap:
        step(2_000_000)

def diag_text():
    n = opm_diag(m, diagbuf, len(diagbuf))
    return diagbuf.raw[:n].decode("latin-1")

def cursor():
    hit = re.search(r"cursor at \((\d+),(\d+)\)", diag_text())
    return (int(hit.group(1)), int(hit.group(2))) if hit else None

def move(x, y):
    for _ in range(200):
        c = cursor()
        if c is None:
            return "no pointer: the card's cursor is off"
        dx, dy = x - c[0], y - c[1]
        if dx == 0 and dy == 0:
            return f"pointer at {c}"
        lim = lambda v: max(-12, min(12, v))
        opm_mouse(m, lim(dx), lim(dy), state["buttons"])
        run_secs(0.06)
    return f"pointer at {cursor()}, wanted ({x},{y}): DID NOT ARRIVE"

def button(down):
    state["buttons"] = 1 if down else 0
    opm_mouse(m, 0, 0, state["buttons"])
    run_secs(0.08)

def shot(name):
    rc = opm_screen(m, shotbuf, len(shotbuf), ctypes.byref(w), ctypes.byref(h))
    if rc <= 0 or not w.value:
        return f"no picture (rc={rc})"
    from PIL import Image
    W, H = w.value, h.value
    path = os.path.join(a.out, name + ".png")
    Image.frombuffer("RGBA", (W, H), shotbuf.raw[:W * H * 4], "raw", "BGRA", 0, 1).convert("RGB").save(path)
    return f"{path} ({W}x{H})"

def perf(secs):
    """The app's loop (MachineSession.Run): chunks aimed at ~15 ms, drained between."""
    out, chunk = [], state["chunk"]
    t0 = time.monotonic()
    p_t, p_tb, p_idle, p_ex, p_au, p_slip = 0.0, opm_tb(m), opm_idle_ns(m), opm_executed(m), state["audio"], opm_rt_slips(m)
    next_line, last_shot = 1.0, 0.0
    while True:
        now = time.monotonic() - t0
        if now >= secs or opm_halted(m):
            break
        c0 = time.monotonic()
        moved = step(chunk)
        dt = (time.monotonic() - c0) * 1000
        if dt < 8 and moved: chunk = min(chunk * 2, 40_000_000)
        elif dt > 30: chunk = max(chunk // 2, 100_000)
        now = time.monotonic() - t0
        if now - last_shot >= 0.033:
            last_shot = now
            opm_screen(m, shotbuf, len(shotbuf), ctypes.byref(w), ctypes.byref(h))
        if now >= next_line:
            next_line += 1.0
            tb, idle, ex, au, slip = opm_tb(m), opm_idle_ns(m), opm_executed(m), state["audio"], opm_rt_slips(m)
            d = now - p_t
            rate = opm_audio_rate(m) or 44100
            out.append(f"  t={now:5.1f}s {(ex - p_ex) / d / 1e6:6.1f} MIPS  clock {(tb - p_tb) / d / 25e6:5.2f}x real  "
                       f"idle {100 * (idle - p_idle) / 1e9 / d:3.0f}%  slips +{slip - p_slip}  "
                       f"sound {100 * (au - p_au) / (rate * 4 * d):5.1f}% of a second at {rate} Hz")
            p_t, p_tb, p_idle, p_ex, p_au, p_slip = now, tb, idle, ex, au, slip
    state["chunk"] = chunk
    return "\n".join(out)

def do(line):
    parts = line.split(None, 1)
    if not parts:
        return ""
    cmd, arg = parts[0], (parts[1] if len(parts) > 1 else "")
    if cmd == "secs": run_secs(float(arg)); return stat()
    if cmd == "run": step(int(float(arg) * 1e6)); return stat()
    if cmd == "shot": return shot(arg or "screen")
    if cmd == "pos": return f"pointer at {cursor()}"
    if cmd == "move":
        x, y = arg.split(); return move(int(x), int(y))
    if cmd == "click": button(True); button(False); return "clicked"
    if cmd == "dbl":
        button(True); button(False); button(True); button(False); return "double-clicked"
    if cmd == "down": button(True); return "button down"
    if cmd == "up": button(False); return "button up"
    if cmd == "key":
        us = [int(k, 0) for k in arg.split(",") if k]
        for u in us:
            opm_key_event(m, u, 1); run_secs(0.05)
        run_secs(0.1)
        for u in reversed(us):
            opm_key_event(m, u, 0); run_secs(0.05)
        return f"pressed {arg}"
    if cmd == "hold": opm_key_event(m, int(arg, 0), 1); run_secs(0.05); return f"{arg} down"
    if cmd == "release": opm_key_event(m, int(arg, 0), 0); run_secs(0.05); return f"{arg} up"
    if cmd == "type": opm_key(m, arg.encode()); run_secs(0.1 + 0.1 * len(arg)); return f"typed {arg!r}"
    if cmd == "snap":
        path = os.path.join(a.out, arg + ".snap")
        if not opm_snapshot_save(m, path.encode(), errbuf, len(errbuf)):
            return "snapshot FAILED: " + errbuf.value.decode(errors="replace")
        subprocess.run(["cp", "--reflink=auto", "--sparse=always", a.hd, os.path.join(a.out, arg + ".hd.img")], check=True)
        return f"{path} @{opm_executed(m)} (+ {arg}.hd.img)"
    if cmd == "diag":
        path = os.path.join(a.out, (arg or "diag") + ".txt")
        with open(path, "w") as f:
            f.write(diag_text())
        return path
    if cmd == "watch":
        x, y = (int(v) for v in arg.split())
        ok = opm_ati_pixel_watch(m, x, y, 400)
        return f"watching ({x},{y})" if ok else "the display is in no mode a pixel can be addressed in"
    if cmd == "watchlog":
        buf = ctypes.create_string_buffer(1 << 18)
        opm_ati_pixel_log(m, buf, len(buf))
        text = buf.value.decode("utf-8", "replace")
        if arg:
            open(os.path.join(a.out, arg + ".txt"), "w").write(text)
        return text.rstrip()
    if cmd == "stat": return stat()
    if cmd == "rt":
        state["rt"] = arg.strip() == "on"
        opm_set_realtime(m, 1 if state["rt"] else 0)
        return "clock paced from the host" if state["rt"] else f"clock paced from the instruction count (fast-tb {a.fast_tb})"
    if cmd == "perf": return perf(float(arg or 10))
    if cmd == "drop":
        names = ["no drive", "tray empty", "tray empty, a disc waiting to go in", "disc in, not read yet",
                 "disc in and read (mounted)", "disc in and read, a newer one waiting for the guest to put it away"]
        said = ""
        if arg:
            if not os.path.exists(arg):
                return f"no such image: {arg}"
            said = "handed over; " if opm_drop_swap(m, enc(arg)) else "REFUSED; "
        st = opm_drop_state(m)
        return f"{said}{names[st] if 0 <= st < len(names) else st}, {opm_drop_reads(m)} blocks read"
    if cmd == "wav":
        if state["wav"]:
            state["wav"].close(); state["wav"] = None
        if arg and arg != "off":
            wv = wave.open(os.path.join(a.out, arg + ".wav"), "wb")
            wv.setnchannels(2); wv.setsampwidth(2); wv.setframerate(opm_audio_rate(m) or 44100)
            state["wav"] = wv
            return f"recording {arg}.wav at {opm_audio_rate(m)} Hz"
        return "recording off"
    return f"unknown command: {cmd}"

def stat():
    return (f"{opm_executed(m):,} insns, guest {opm_tb(m) / 25e6:.1f} s, pc={opm_pc(m):08x}, slips {opm_rt_slips(m)}"
            + (", HALTED" if opm_halted(m) else ""))

if os.path.exists(SOCK):
    os.unlink(SOCK)
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(SOCK)
srv.listen(4)
print(f"remote: machine up ({a.ram} MB"
      + (f", resumed at {opm_executed(m):,} instructions" if a.resume else "") + f"), serving at {SOCK}", flush=True)
quit_ = False
while not quit_:
    c, _ = srv.accept()
    data = b""
    while True:
        b = c.recv(65536)
        if not b:
            break
        data += b
    for line in data.decode().splitlines():
        line = line.strip()
        if not line:
            continue
        if line == "quit":
            quit_ = True
            c.sendall(b"bye\n")
            break
        try:
            reply = do(line)
        except Exception as e:           # a bad command must not take the machine with it
            reply = f"error: {e!r}"
        try:
            c.sendall(f"[{line}] {reply}\n".encode())
        except OSError:
            break
    c.close()
if state["wav"]:
    state["wav"].close()
console.close()
srv.close()
os.unlink(SOCK)
opm_destroy(m)
