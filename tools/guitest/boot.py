#!/usr/bin/env python3
"""Boots the machine in the real window and reports what the log and the screen say.

    tools/headless.sh python3 tools/guitest/boot.py --rom FILE --ati-rom FILE
        [--hard-disk FILE] [--cd FILE] [--ram MB] [--seconds N] [--shots DIR] [--sound]
        [--mouse] [--app FILE]

Writes a scratch shell.json (never the real profile: it refuses to run outside
tools/headless.sh), copies the hard disk into the scratch profile so the
original is never written, starts the app, and walks the user's path with
synthetic keys (XTEST): Machine > Start, then File > Exit at the end, which
must stop the machine and close the window on its own. A screenshot is taken
every 15 seconds. With --mouse it then clicks the screen (which captures the
pointer), moves the mouse the way a hand would, and middle-clicks to release;
the guest's picture must change where its cursor moved. Passes when the
window shows a guest picture by the end, the mouse moved the guest's cursor
if asked, the app exits by itself after File > Exit, and its log holds no
error.
--sound plays through SDL's dummy driver: the wall makes real sound impossible.
Needs python-xlib and ImageMagick.
"""
import argparse, json, os, shutil, subprocess, sys, time
from Xlib import X, XK, display
from Xlib.ext import xtest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ap = argparse.ArgumentParser()
ap.add_argument("--rom", required=True)
ap.add_argument("--ati-rom", required=True)
ap.add_argument("--hard-disk")
ap.add_argument("--cd")
ap.add_argument("--ram", type=int, default=256)
ap.add_argument("--seconds", type=int, default=120)
ap.add_argument("--shots")
ap.add_argument("--sound", action="store_true")
ap.add_argument("--mouse", action="store_true",
                help="at the end: click the screen to capture, move, release; the guest picture must change")
ap.add_argument("--app", default=os.path.join(ROOT, "bin", "openpowermac"))
a = ap.parse_args()

config = os.environ.get("XDG_CONFIG_HOME")
if not config or os.path.realpath(config).startswith(os.path.realpath(os.path.expanduser("~/.config"))):
    sys.exit("boot test: refusing to run outside tools/headless.sh (it would use the real profile)")
profile = os.path.join(config, "OpenPowerMac")
os.makedirs(profile, exist_ok=True)
hd = ""
if a.hard_disk:
    hd = os.path.join(profile, "hd.img")
    subprocess.run(["cp", "--sparse=always", a.hard_disk, hd], check=True)
settings = {"RomPath": os.path.abspath(a.rom), "AtiRomPath": os.path.abspath(a.ati_rom),
            "HdPath": hd, "CdPath": os.path.abspath(a.cd) if a.cd else "",
            "RamMb": a.ram, "Sound": a.sound, "AutoBoot": True,
            # The default drop box is ~/Documents, which the wall does not move.
            "DropBoxPath": os.path.join(profile, "Drop Box")}
with open(os.path.join(profile, "shell.json"), "w") as f:
    json.dump(settings, f, indent=2)
shots = a.shots or os.path.join(os.environ.get("OPM_TEST_TMP", "/tmp"), "shots")
os.makedirs(shots, exist_ok=True)

d = display.Display()
def key(sym, mods=()):
    codes = [d.keysym_to_keycode(XK.string_to_keysym(m)) for m in mods]
    k = d.keysym_to_keycode(XK.string_to_keysym(sym))
    for c in codes: xtest.fake_input(d, X.KeyPress, c)
    xtest.fake_input(d, X.KeyPress, k); xtest.fake_input(d, X.KeyRelease, k)
    for c in reversed(codes): xtest.fake_input(d, X.KeyRelease, c)
    d.sync(); time.sleep(0.4)

def click(x, y, button=1):
    xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(0.2)
    xtest.fake_input(d, X.ButtonPress, button); xtest.fake_input(d, X.ButtonRelease, button)
    d.sync(); time.sleep(0.5)

def shot(name):
    out = os.path.join(shots, name + ".png")
    subprocess.run(["import", "-window", "root", out], check=False)
    return out

log_path = os.path.join(os.environ.get("XDG_STATE_HOME", ""), "OpenPowerMac", "gui.log")
app = subprocess.Popen([a.app], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(6)
shot("00-open")
key("m", ["Alt_L"]); key("s")          # Machine > Start
t0 = time.time(); n = 1
while time.time() - t0 < a.seconds:
    time.sleep(15)
    shot(f"{n:02d}-{int(time.time() - t0)}s"); n += 1
final = shot("99-final")
moved = None
if a.mouse:
    click(520, 340)                     # capture (also the guest's click, on the desktop)
    time.sleep(1.0)
    before = shot("90-captured")
    for _ in range(40):                 # relative travel, as a hand would
        xtest.fake_input(d, X.MotionNotify, detail=1, x=6, y=4); d.sync(); time.sleep(0.02)
    time.sleep(1.0)
    after = shot("91-moved")
    click(520, 340, button=2)           # middle-click releases
    time.sleep(0.5)
    diff = subprocess.run(["magick", "compare", "-metric", "AE", "-crop", "800x600+120+40",
                           before, after, "null:"], capture_output=True, text=True)
    moved = int(float((diff.stderr.strip().split() or ["0"])[0]))
    print(f"boot test: pixels changed by the mouse = {moved}")
# File > Exit by mouse, the way a user quits: while the machine runs, Alt goes
# to the guest (Command), so Alt+F is not a way into the menu. The window sits
# at the server's origin with no window manager; File is the first menu title.
click(33, 26); click(37, 299)          # File, then Exit (the menu's last item)
shot("99-exit")
try:
    rc = app.wait(timeout=30)
    exited = True
except subprocess.TimeoutExpired:
    app.kill(); rc = None; exited = False

# The picture: distinct colours inside the screen area, ignoring the chrome.
colours = int(subprocess.run(["magick", final, "-crop", "900x500+60+60", "-format", "%k", "info:"],
                             capture_output=True, text=True).stdout.strip() or 0)
log = open(log_path).read() if os.path.exists(log_path) else ""
con_path = os.path.join(os.path.dirname(log_path), "console.log")
console = open(con_path, errors="replace").read() if os.path.exists(con_path) else ""
errors = [l for l in log.splitlines() if "UNHANDLED" in l or "FATAL" in l]
print(f"boot test: exited by itself={exited} rc={rc} colours in screen area={colours} shots={shots}")
for l in log.splitlines(): print("  log:", l)
for l in console.splitlines()[-25:]: print("  console:", l)
ok = exited and colours > 40 and not errors and (moved is None or moved > 0)
print("boot test:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
