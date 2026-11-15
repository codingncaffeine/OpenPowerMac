## What's New

The first alpha of OpenPowerMac: a Power Mac G4 (AGP Graphics) emulator written from scratch, for Linux.

- **Mac OS 9, installed from its own CD** — Start from an installation CD image, initialize a disk the app creates, install, and restart into the new system
- **A complete G4 processor** — Integer, floating point and AltiVec, with a translator to native code that is checked to behave exactly like the interpreter
- **Rage 128 Pro graphics with 2D and 3D acceleration** — A 1024×768 desktop in millions of colors, and 3D games of the era drawn on as many threads as the computer can spare
- **Sound, mouse and keyboard** — Sound plays through PulseAudio, PipeWire or ALSA; click the Mac's screen to capture the pointer and the keyboard, middle-click or Esc to release them
- **Drop Box** — Drop files or folders on the window and they appear in the Mac on their own disc, with no restart
- **CDs in and out while the Mac runs** — Choose or eject a CD image from the File menu at any time
- **Packages** — A `.deb` for Debian and Ubuntu, a tarball for any other glibc distro, a tarball for Alpine and other musl systems, and `openpowermac-bin` on the AUR

## Download

`openpowermac_0.0.3_amd64.deb` installs on Debian and Ubuntu. `OpenPowerMac-0.0.3-linux-x64.tar.gz` runs anywhere else with glibc: extract it and run `./OpenPowerMac`. `OpenPowerMac-0.0.3-linux-musl-x64.tar.gz` is the same for Alpine. OpenPowerMac contains no original Macintosh code; you supply the Boot ROM, the graphics card's ROM and your own Mac OS 9 disks. Mac OS X does not start yet.
