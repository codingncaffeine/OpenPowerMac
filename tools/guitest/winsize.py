#!/usr/bin/env python3
"""The window comes back the way it was left.

    tools/headless.sh python3 tools/guitest/winsize.py [--app FILE] [--shots DIR]

Starts the app on a scratch profile (it refuses to run outside
tools/headless.sh), resizes its window the way a window manager would,
drags the splitter above the console pane, and quits through File > Exit.
Then starts it again and requires the window and the console pane to be the
size they were left, read from the X server and from the picture, not from
the settings file, which only shows what was written.

Then the maximized state. The test display has no window manager, so the
test does to the window what one does when it maximizes a window: it sets
the window's state property and gives it the size of the screen, once in
each order, since a window manager may send the two either way round. The
app must remember that it was maximized AND the size it had before, ask to
be maximized when it starts again, and come back at the earlier size once
it is not maximized any more.
Needs python-xlib, Pillow and ImageMagick.
"""
import argparse, json, os, subprocess, sys, time
from PIL import Image
from Xlib import X, Xatom, display, protocol
from Xlib.ext import xtest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ap = argparse.ArgumentParser()
ap.add_argument("--app", default=os.path.join(ROOT, "bin", "openpowermac"))
ap.add_argument("--shots")
a = ap.parse_args()

config = os.environ.get("XDG_CONFIG_HOME")
if not config or os.path.realpath(config).startswith(os.path.realpath(os.path.expanduser("~/.config"))):
    sys.exit("window size test: refusing to run outside tools/headless.sh (it would use the real profile)")
profile = os.path.join(config, "OpenPowerMac")
os.makedirs(profile, exist_ok=True)
settings_file = os.path.join(profile, "shell.json")
if os.path.exists(settings_file):
    os.remove(settings_file)
shots = a.shots or os.path.join(os.environ.get("OPM_TEST_TMP", "/tmp"), "shots")
os.makedirs(shots, exist_ok=True)

d = display.Display()
root = d.screen().root
failures = []
def check(ok, what):
    print(("  ok   " if ok else "  FAIL ") + what)
    if not ok: failures.append(what)

def find_window(timeout=20):
    """The app's top-level window: the one whose name starts with the app's."""
    end = time.time() + timeout
    while time.time() < end:
        stack = [root]
        while stack:
            w = stack.pop()
            try:
                name = w.get_wm_name()
                geo = w.get_geometry()
                if name and str(name).startswith("OpenPowerMac") and geo.width > 300:
                    return w
                stack.extend(w.query_tree().children)
            except Exception:
                pass
        time.sleep(0.3)
    sys.exit("window size test: the app's window did not appear")

def origin(w):
    t = w.translate_coords(root, 0, 0)
    return -t.x, -t.y
def size(w):
    g = w.get_geometry()
    return g.width, g.height
def click(x, y, button=1):
    xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(0.2)
    xtest.fake_input(d, X.ButtonPress, button); xtest.fake_input(d, X.ButtonRelease, button)
    d.sync(); time.sleep(0.5)
def drag(x, y0, y1):
    xtest.fake_input(d, X.MotionNotify, x=x, y=y0); d.sync(); time.sleep(0.2)
    xtest.fake_input(d, X.ButtonPress, 1); d.sync(); time.sleep(0.2)
    step = 1 if y1 > y0 else -1
    for y in range(y0, y1 + step, step * 4):
        xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(0.02)
    xtest.fake_input(d, X.MotionNotify, x=x, y=y1); d.sync(); time.sleep(0.2)
    xtest.fake_input(d, X.ButtonRelease, 1); d.sync(); time.sleep(0.5)
def shot(name):
    out = os.path.join(shots, name + ".png")
    subprocess.run(["import", "-window", "root", out], check=False)
    return out
BORDER = (0x3A, 0x3A, 0x40)
def border_bands(w, png):
    """Runs of the border colour down the middle of the window, top to bottom."""
    ox, oy = origin(w); ww, wh = size(w)
    im = Image.open(png).convert("RGB")
    x, bands, y = ox + ww // 2, [], oy
    while y < oy + wh:
        if im.getpixel((x, y)) == BORDER:
            y0 = y
            while y < oy + wh and im.getpixel((x, y)) == BORDER: y += 1
            bands.append((y0, y))
        else:
            y += 1
    return bands
def splitter(w, png):
    """The splitter: the one thick band of the border colour (the rest are hairlines)."""
    thick = [b for b in border_bands(w, png) if b[1] - b[0] >= 4]
    return thick[-1] if thick else None
def console_height(w, png):
    """Measured from the picture: the console pane runs from the bottom of the
    splitter to the hairline on top of the status bar, the last one in the window."""
    sp = splitter(w, png)
    lines = [b for b in border_bands(w, png) if b[1] - b[0] == 1]
    if not sp or not lines or lines[-1][0] <= sp[1]: return None
    return lines[-1][0] - sp[1]
def settings():
    try: return json.load(open(settings_file))
    except Exception: return {}
def quit_app(w, app):
    ox, oy = origin(w)
    click(ox + 23, oy + 16)             # File
    click(ox + 27, oy + 289)            # Exit, the menu's last item
    try: return app.wait(timeout=15) == 0
    except subprocess.TimeoutExpired:
        app.kill(); return False
def net(name): return d.intern_atom(name)
MAXIMIZED = [net("_NET_WM_STATE_MAXIMIZED_VERT"), net("_NET_WM_STATE_MAXIMIZED_HORZ")]
SCREEN = (d.screen().width_in_pixels, d.screen().height_in_pixels)
def wm_state(w, on):
    """The window manager's side of a state change: it rewrites the property."""
    w.change_property(net("_NET_WM_STATE"), Xatom.ATOM, 32, MAXIMIZED if on else []); d.sync()
def wm_size(w, width, height):
    w.configure(x=0, y=0, width=width, height=height); d.sync()
def is_maximized(w):
    p = w.get_full_property(net("_NET_WM_STATE"), Xatom.ATOM)
    have = set(p.value) if p else set()
    return all(a in have for a in MAXIMIZED)
def saved():
    s = settings()
    return s.get("WindowMaximized"), s.get("WindowWidth"), s.get("WindowHeight")
def start():
    app = subprocess.Popen([a.app], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    w = find_window(); time.sleep(2.0)
    return app, w

try:
    print("first launch: the default size")
    app, w = start()
    first = size(w)
    check(first[0] >= 720 and first[1] >= 560, f"window is {first[0]}x{first[1]}")

    print("resize to 940x740 and make the console pane taller")
    w.configure(width=940, height=740); d.sync(); time.sleep(1.5)
    check(size(w) == (940, 740), f"the window took the size: {size(w)}")
    h0 = console_height(w, shot("10-resized"))
    check(h0 is not None, f"console pane measured in the picture: {h0}")
    sp = splitter(w, os.path.join(shots, "10-resized.png"))
    if sp:
        ox, oy = origin(w)
        mid = (sp[0] + sp[1]) // 2
        drag(ox + 470, mid, mid - 70)
    time.sleep(2.0)                      # past the save's delay
    h1 = console_height(w, shot("11-dragged"))
    check(h0 is not None and h1 is not None and 60 <= h1 - h0 <= 80, f"the splitter moved the pane: {h0} -> {h1}")
    s = settings()
    check(s.get("WindowWidth") == 940 and s.get("WindowHeight") == 740,
          f"saved while running: {s.get('WindowWidth')}x{s.get('WindowHeight')}, console {s.get('ConsoleHeight')}")
    check(quit_app(w, app), "File > Exit closed the app")

    print("second launch: it must come back as it was left")
    app, w = start()
    check(size(w) == (940, 740), f"window is {size(w)}")
    h2 = console_height(w, shot("20-relaunched"))
    check(h1 is not None and h2 == h1, f"console pane is {h2} (was {h1})")


    print("maximized, the state arriving before the size")
    wm_state(w, True); time.sleep(0.3); wm_size(w, *SCREEN); time.sleep(2.0)
    check(size(w) == SCREEN, f"the window has the screen's size: {size(w)}")
    check(saved() == (True, 940, 740), f"saved: maximized, and the size it had before: {saved()}")
    check(quit_app(w, app), "File > Exit closed the app")

    print("third launch: it must ask to be maximized, at the size it had before")
    app, w = start()
    check(is_maximized(w), "the window asks to be maximized")
    check(size(w) == (940, 740), f"its own size is still {size(w)}")
    wm_state(w, False); time.sleep(2.0)
    check(saved() == (False, 940, 740), f"not maximized any more: {saved()}")

    print("maximized, the size arriving before the state")
    wm_size(w, *SCREEN); time.sleep(0.3); wm_state(w, True); time.sleep(2.0)
    check(saved() == (True, 940, 740), f"saved: maximized, and the size it had before: {saved()}")
    wm_state(w, False); wm_size(w, 940, 740); time.sleep(2.0)
    check(saved() == (False, 940, 740), f"restored: {saved()}")
    check(quit_app(w, app), "File > Exit closed the app")

    print("fourth launch: not maximized, the same size")
    app, w = start()
    check(not is_maximized(w) and size(w) == (940, 740), f"window is {size(w)}, maximized: {is_maximized(w)}")
    check(quit_app(w, app), "File > Exit closed the app")
finally:
    pass

print("window size test:", "PASS" if not failures else f"FAIL ({len(failures)})")
sys.exit(0 if not failures else 1)
