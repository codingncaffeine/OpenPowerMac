#!/usr/bin/env python3
"""Boots the machine in the real window and reports what the log and the screen say.

    tools/headless.sh python3 tools/guitest/boot.py --rom FILE --ati-rom FILE
        [--hard-disk FILE] [--cd FILE] [--ram MB] [--seconds N] [--shots DIR] [--sound]
        [--mouse] [--keys] [--cd-later FILE] [--app FILE]
        [--drop-box DIR] [--drop-later DIR]

Writes a scratch shell.json (never the real profile: it refuses to run outside
tools/headless.sh), copies the hard disk into the scratch profile so the
original is never written, starts the app, and walks the user's path with
synthetic keys (XTEST): Machine > Start, then File > Exit at the end, which
must stop the machine and close the window on its own. A screenshot is taken
every 15 seconds. With --mouse it then clicks the screen (which captures the
pointer), moves the mouse the way a hand would, and middle-clicks to release;
the guest's picture must change where its cursor moved. With --keys it also
presses a key while captured and then asks for Machine > Diagnostics, whose
USB section must show the guest taking the keyboard's reports. With --cd-later
it chooses a CD image from the File menu while the machine runs, through the
file dialog, and the console must say the guest mounted it. Passes when the
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
ap.add_argument("--cd-later", metavar="FILE",
                help="once the machine is up: File > Choose CD Image, pick FILE in the file dialog; "
                     "the console must report the disc mounted")
ap.add_argument("--drop-box", metavar="DIR",
                help="the Drop Box folder the app starts with (default: an empty one); with something in it "
                     "the console must report its volume mounted")
ap.add_argument("--drop-later", metavar="DIR",
                help="once the machine is up, choose DIR from File > Drop Box > Choose Drop Box Folder, as a "
                     "user would; the console must report its volume packed and mounted, with no new start")
ap.add_argument("--keys", action="store_true",
                help="with --mouse: while captured, press a key; the guest must take the keyboard reports")
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
            "DropBoxPath": os.path.abspath(a.drop_box) if a.drop_box else os.path.join(profile, "Drop Box")}
# The folder exists, as it does once the app has run: the folder picker opens
# in it, and opens in "Recent" (where a typed path is not taken) if it is missing.
os.makedirs(settings["DropBoxPath"], exist_ok=True)
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

def hover(x, y):
    xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(0.6)

def click(x, y, button=1):
    xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(0.2)
    xtest.fake_input(d, X.ButtonPress, button); xtest.fake_input(d, X.ButtonRelease, button)
    d.sync(); time.sleep(0.5)

SHIFTED = {"_": "minus", "~": "grave", ":": "semicolon", "+": "equal", "(": "9", ")": "0", "&": "7", "!": "1"}
PLAIN = {"/": "slash", ".": "period", "-": "minus", " ": "space", ",": "comma", "=": "equal", ";": "semicolon"}
def type_text(text):
    """Type a string as key presses, with Shift where the character needs it."""
    shift = d.keysym_to_keycode(XK.string_to_keysym("Shift_L"))
    for ch in text:
        name, shifted = ch, False
        if ch in PLAIN: name = PLAIN[ch]
        elif ch in SHIFTED: name, shifted = SHIFTED[ch], True
        elif ch.isupper(): name, shifted = ch.lower(), True
        code = d.keysym_to_keycode(XK.string_to_keysym(name))
        if not code:
            sys.exit(f"boot test: cannot type {ch!r}")
        if shifted: xtest.fake_input(d, X.KeyPress, shift)
        xtest.fake_input(d, X.KeyPress, code); xtest.fake_input(d, X.KeyRelease, code)
        if shifted: xtest.fake_input(d, X.KeyRelease, shift)
    # One burst, then one flush: a file dialog completes the path as it is
    # typed, and keys that arrive between its completions come out doubled.
    d.sync()

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
cd_ok = None
if a.cd_later:
    # File > Choose CD Image…, then the path typed into the dialog's location
    # field (Ctrl+L opens it) and Return: the way a user picks a file.
    click(33, 26); click(80, 83)
    time.sleep(4.0)
    shot("80-picker")
    key("l", ["Control_L"]); time.sleep(0.8)
    type_text(os.path.abspath(a.cd_later)); time.sleep(0.5)
    shot("81-typed")
    key("Return"); time.sleep(12.0)
    shot("82-disc")
if a.drop_later:
    # File > Drop Box > Choose Drop Box Folder…, the folder typed into the
    # dialog's location field, then the app's own notice acknowledged.
    click(33, 26); hover(80, 200); hover(300, 206); click(355, 233)
    time.sleep(4.0)
    shot("84-folder-picker")
    key("l", ["Control_L"]); time.sleep(0.8)
    type_text(os.path.abspath(a.drop_later)); time.sleep(0.5)
    shot("85-folder-typed")
    # Return takes the typed folder, and on some runs only opens it: the
    # app's saved setting says when the dialog has really answered.
    for _ in range(3):
        key("Return"); time.sleep(4.0)
        try:
            if json.load(open(os.path.join(profile, "shell.json"))).get("DropBoxPath") == os.path.abspath(a.drop_later):
                break
        except Exception:
            pass
    shot("86-notice")
    # The notice's OK button, by the pointer: a Return here goes to the menu
    # item still highlighted under it on a display with no window manager,
    # and opens the picker a second time.
    click(726, 499); time.sleep(20.0)
    shot("87-drop-box")
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
    if a.keys:
        key("u"); key("Shift_L")        # a letter and a modifier, down and up, through the grab
    click(520, 340, button=2)           # middle-click releases
    time.sleep(0.5)
    if a.keys:
        # Machine > Diagnostics, by mouse (the title, then the item). The report
        # takes two samples a chunk apart and lands on the console.
        click(89, 26); click(187, 227)
        time.sleep(3.0)
        shot("92-diagnostics")
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

# The picture: distinct colors inside the screen area, ignoring the chrome.
colors = int(subprocess.run(["magick", final, "-crop", "900x500+60+60", "-format", "%k", "info:"],
                             capture_output=True, text=True).stdout.strip() or 0)
log = open(log_path).read() if os.path.exists(log_path) else ""
con_path = os.path.join(os.path.dirname(log_path), "console.log")
console = open(con_path, errors="replace").read() if os.path.exists(con_path) else ""
errors = [l for l in log.splitlines() if "UNHANDLED" in l or "FATAL" in l]
print(f"boot test: exited by itself={exited} rc={rc} colors in screen area={colors} shots={shots}")
for l in log.splitlines(): print("  log:", l)
for l in console.splitlines()[-25:]: print("  console:", l)
if a.cd_later:
    try:
        saved = json.load(open(os.path.join(profile, "shell.json"))).get("CdPath", "")
    except Exception as ex:
        saved = f"(unreadable: {ex})"
    print(f"boot test: the app saved CdPath = {saved!r}")
    cd_lines = [l for l in console.splitlines() if l.startswith("[cd]")]
    cd_ok = any("disc mounted" in l for l in cd_lines)
    print("boot test: CD chosen while running: " + ("MOUNTED by the guest" if cd_ok else "NOT mounted")
          + "".join("\n    " + l for l in cd_lines))
drop_ok = None
drop_lines = [l for l in console.splitlines() if l.startswith("[drop box]")]
if a.drop_box:
    # From power-on: attached before the first instruction, mounted during the boot.
    drop_ok = any("volume attached from" in l for l in drop_lines) and any("volume mounted" in l for l in drop_lines)
    print("boot test: Drop Box folder set before the start: " + ("MOUNTED by the guest" if drop_ok else "NOT mounted"))
if a.drop_later:
    try:
        saved = json.load(open(os.path.join(profile, "shell.json"))).get("DropBoxPath", "")
    except Exception as ex:
        saved = f"(unreadable: {ex})"
    print(f"boot test: the app saved DropBoxPath = {saved!r}")
    at = next((i for i, l in enumerate(drop_lines) if "folder is now" in l), None)
    later_ok = at is not None and any("volume mounted" in l for l in drop_lines[at:])
    print("boot test: Drop Box folder chosen while running: "
          + ("PACKED and MOUNTED by the guest" if later_ok else "NOT mounted"))
    drop_ok = later_ok and drop_ok is not False
if a.drop_box or a.drop_later:
    for l in drop_lines: print("    " + l)
keys_ok = None
if a.keys:
    # The LAST keyboard section of the diagnostics: reports the guest took
    # against events the window handed over. Two keys, down and up, is four.
    import re
    hits = re.findall(r"-- usb0 \(keyboard\).*?\n--\s+.*?(\d+) reports delivered,\s*(\d+) injected", console)
    delivered, injected = (int(hits[-1][0]), int(hits[-1][1])) if hits else (-1, -1)
    keys_ok = injected >= 4 and delivered >= 4
    print(f"boot test: keyboard handed over {injected} event(s), the guest took {delivered} report(s): "
          + ("TAKEN" if keys_ok else "NOT TAKEN"))
ok = (exited and colors > 40 and not errors and (moved is None or moved > 0) and keys_ok is not False
      and cd_ok is not False and drop_ok is not False)
print("boot test:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
