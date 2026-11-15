#!/usr/bin/env python3
"""The Drop Box's menu, with the machine stopped: no guest is needed for it.

    tools/headless.sh python3 tools/guitest/dropbox.py [--shots DIR] [--app FILE]

Writes a scratch profile whose Drop Box folder holds a file and a folder (as
earlier drops left them) and which has a folder lent to the Drop Box, starts
the app, and walks File > Drop Box as a user would, by the pointer:
"Take the Folder Out" must set the lent folder back to none and touch nothing
in it; "Empty the Drop Box" must move the dropped copies to the Trash (the
scratch profile's own: the freedesktop.org layout, with a record for each)
and say so in a notice. Refuses to run outside tools/headless.sh.
"""
import argparse, json, os, subprocess, sys, time
from Xlib import X, display
from Xlib.ext import xtest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ap = argparse.ArgumentParser()
ap.add_argument("--shots")
ap.add_argument("--app", default=os.path.join(ROOT, "bin", "openpowermac"))
a = ap.parse_args()
config = os.environ.get("XDG_CONFIG_HOME")
if not config or os.path.realpath(config).startswith(os.path.realpath(os.path.expanduser("~/.config"))):
    sys.exit("drop box test: refusing to run outside tools/headless.sh (it would use the real profile)")
profile = os.path.join(config, "OpenPowerMac")
store = os.path.join(profile, "Drop Box")
lent = os.path.join(os.environ["OPM_TEST_TMP"], "lent folder")
os.makedirs(os.path.join(store, "A Folder"))
os.makedirs(lent)
open(os.path.join(store, "dropped file.txt"), "w").write("dropped\n")
open(os.path.join(store, "A Folder", "inside.txt"), "w").write("inside\n")
open(os.path.join(lent, "mine.txt"), "w").write("the user's own\n")
with open(os.path.join(profile, "shell.json"), "w") as f:
    json.dump({"DropStorePath": store, "DropBoxPath": lent}, f)
shots = a.shots or os.path.join(os.environ.get("OPM_TEST_TMP", "/tmp"), "shots")
os.makedirs(shots, exist_ok=True)
d = display.Display()

def hover(x, y):
    xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(0.6)

def click(x, y):
    hover(x, y)
    xtest.fake_input(d, X.ButtonPress, 1); xtest.fake_input(d, X.ButtonRelease, 1)
    d.sync(); time.sleep(0.6)

def shot(name):
    out = os.path.join(shots, name + ".png")
    subprocess.run(["import", "-window", "root", out], check=False)
    return out

def setting():
    try:
        return json.load(open(os.path.join(profile, "shell.json"))).get("DropBoxPath")
    except Exception as ex:
        return f"(unreadable: {ex})"

def submenu():
    """File, then Drop Box, then across into its submenu."""
    click(33, 26); hover(80, 200); hover(300, 206)

app = subprocess.Popen([a.app], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(6)
submenu(); shot("10-submenu")
click(355, 260)                         # Take the Folder Out
time.sleep(1.5); shot("11-folder-out")
out_ok = setting() == "" and os.listdir(lent) == ["mine.txt"] and len(os.listdir(store)) == 2
print(f"drop box test: Take the Folder Out: DropBoxPath = {setting()!r}, the folder still holds "
      f"{os.listdir(lent)}, the Drop Box folder {sorted(os.listdir(store))}: " + ("OK" if out_ok else "WRONG"))
submenu(); click(355, 297)              # Empty the Drop Box
time.sleep(3.0); notice = shot("12-emptied-notice")
trash = os.path.join(os.environ["XDG_DATA_HOME"], "Trash")
gone = os.listdir(store) == []
there = all(os.path.exists(os.path.join(trash, "files", n)) for n in ("dropped file.txt", "A Folder"))
inside = os.path.exists(os.path.join(trash, "files", "A Folder", "inside.txt"))
records = []
for n in ("dropped file.txt", "A Folder"):
    try:
        info = open(os.path.join(trash, "info", n + ".trashinfo")).read()
    except OSError:
        info = ""
    want = "Path=" + "/".join(__import__("urllib.parse").parse.quote(p, safe="") for p in os.path.join(store, n).split("/"))
    records.append(info.startswith("[Trash Info]\n") and want in info.splitlines() and "DeletionDate=" in info)
empty_ok = gone and there and inside and all(records) and os.listdir(lent) == ["mine.txt"]
print(f"drop box test: Empty the Drop Box: folder emptied={gone}, items in the Trash={there}, a folder's "
      f"content with it={inside}, records as the specification writes them={records}: "
      + ("OK" if empty_ok else "WRONG"))
click(642, 484); time.sleep(1.0)        # the notice's OK (the dialog is as wide as its one line)
shot("13-after")
click(33, 26); click(37, 299)           # File > Exit
try:
    rc = app.wait(timeout=20); exited = True
except subprocess.TimeoutExpired:
    app.kill(); rc = None; exited = False
log_path = os.path.join(os.environ.get("XDG_STATE_HOME", ""), "OpenPowerMac", "gui.log")
log = open(log_path).read() if os.path.exists(log_path) else ""
errors = [l for l in log.splitlines() if "UNHANDLED" in l or "FATAL" in l]
for l in errors: print("  log:", l)
ok = out_ok and empty_ok and exited and not errors
print(f"drop box test: exited by itself={exited} rc={rc} shots={shots}")
print("drop box test:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
