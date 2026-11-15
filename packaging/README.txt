OpenPowerMac
============

A Power Macintosh G4 emulator written from scratch: the Power Mac G4 (AGP
Graphics) with its Rage 128 Pro graphics card. This is an alpha release.

OpenPowerMac contains no original Macintosh code. You supply the ROMs of the
machine you own and your own Mac OS 9 disks.


What you need
-------------

1. The Boot ROM of a Power Mac G4 (AGP Graphics): a 1 MB file. Firmware
   4.2.8f1 and 3.2.4f1 are known to start.
2. The ROM of the Rage 128 Pro AGP graphics card. Without it the machine has
   no picture.
3. A Mac OS 9 installation CD as an image file (.iso, .toast, .cdr, .dmg, or
   .cue with .bin).


Running it
----------

From the tarball: extract it anywhere and run

    ./OpenPowerMac

From the .deb: install it (sudo apt install ./openpowermac_<version>_amd64.deb)
and start OpenPowerMac from the application menu, or run "openpowermac".

First start:

1. File > Choose Boot ROM...
2. File > Choose ATI Card ROM...
3. File > Choose CD Image...               the Mac OS 9 installation CD
4. File > Hard Disk > New Disk Image       pick a size and a place for the file
5. Machine > Start

The Mac starts from the CD. Open Drive Setup on the CD and initialize the new
disk, run the installer, then restart the Mac from its Special menu: it starts
from the hard disk. File > Eject CD takes the CD out.

Click the Mac's screen to give it the mouse and keyboard. While it has them,
desktop shortcuts such as the Super key and Alt+Tab go to the Mac.
Middle-click or Esc gives them back to the desktop.

Machine > Memory sets the RAM for the next start (64 MB to 1.5 GB).


Getting files into the Mac
--------------------------

Drop files or folders on the Mac's screen: they appear in the Mac on a disc
named Drop Box, with no restart. Dragging the Drop Box to the Trash in the Mac
empties it. File > Drop Box has the rest: open its folder, lend it a folder of
this computer, or empty it.

File > Choose Shared Folder mounts a folder of this computer in the Mac,
read-only, at every start.


What it needs
-------------

A 64-bit x86 Linux desktop. The app runs as an X11 client (through XWayland
on Wayland sessions). Everything else is in the package, including SDL3 for
sound, which plays through PulseAudio, PipeWire's PulseAudio service, or ALSA.

The tarball needs these from the system, which desktop installs already have:
the C and C++ runtime, ICU, fontconfig and freetype, and the X11 client
libraries (libX11, libICE, libSM, libXi, libXfixes, libXcursor, libXrandr, libXext).
On Alpine use the linux-musl-x64 tarball and install:
    apk add icu-libs libstdc++ libgcc fontconfig libx11 libice libsm libxi libxfixes \
            libxcursor libxrandr libxext

3D games are drawn by the processor, on as many threads as the computer can
spare: more cores give more frames.


Known limits
------------

Mac OS 9 only: Mac OS X does not start yet.
Sleep (Special > Sleep) is not supported; use Shut Down.
After Special > Shut Down the window keeps the Mac's last picture and says
"shut down" in its status bar; Machine > Start starts the Mac again.


Where things are kept
---------------------

Settings:                      ~/.config/OpenPowerMac/shell.json
The app's log and the Mac's
serial console:                ~/.local/state/OpenPowerMac/gui.log, console.log

gui.log and console.log are the first things to attach to a bug report:
https://github.com/codingncaffeine/OpenPowerMac/issues
