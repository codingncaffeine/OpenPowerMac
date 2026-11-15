<p align="center">
  <img src="docs/banner.png" alt="OpenPowerMac" width="100%">
</p>

# OpenPowerMac

A from-scratch emulator of the Power Mac G4 (AGP Graphics), "Sawtooth", with its Rage 128 Pro
graphics card. It installs Mac OS 9 from a CD onto a disk it creates, runs it from that disk,
and plays the era's 3D games. This is an alpha.

No Apple software is included or distributed. You supply the Boot ROM of the machine, the
graphics card's ROM and your own Mac OS 9 media.

## Install (Linux)

| | |
|---|---|
| Debian, Ubuntu | `sudo apt install ./openpowermac_<version>_amd64.deb` |
| Arch | [`openpowermac-bin`](https://aur.archlinux.org/packages/openpowermac-bin) from the AUR |
| Fedora and other glibc distros | extract `OpenPowerMac-<version>-linux-x64.tar.gz` and run `./OpenPowerMac` |
| Alpine and other musl systems | extract `OpenPowerMac-<version>-linux-musl-x64.tar.gz` and run `./OpenPowerMac` |

The files are on the [releases page](https://github.com/codingncaffeine/OpenPowerMac/releases).
What you need, the first start, getting files into the Mac and the known limits are in
[Installing and Running](https://github.com/codingncaffeine/OpenPowerMac/wiki/Installing-and-Running).

## Where it is

| | |
|---|---|
| Processor | a complete MPC7400 (PowerPC G4) with AltiVec, and a translator to x86-64 |
| Mac OS 9 | installs from CD and runs from disk; Restart and Shut Down work |
| Graphics | Rage 128 Pro: 2D and 3D acceleration, driven by the guest's own drivers |
| Sound and input | sound through the AWACS codec; USB keyboard and mouse |
| File exchange | files dropped on the window appear in the Mac; a host folder can be shared |
| Mac OS X | does not start yet |

The [wiki](https://github.com/codingncaffeine/OpenPowerMac/wiki) has the rest:
[the machine](https://github.com/codingncaffeine/OpenPowerMac/wiki/Power-Mac-G4),
[the processor](https://github.com/codingncaffeine/OpenPowerMac/wiki/MPC7400-Core),
[the graphics card](https://github.com/codingncaffeine/OpenPowerMac/wiki/Rage-128-Pro),
[speed](https://github.com/codingncaffeine/OpenPowerMac/wiki/Speed) and
[how it is tested](https://github.com/codingncaffeine/OpenPowerMac/wiki/Testing-and-Verification).

## Build

Needs CMake, a C++20 compiler, the .NET 10 SDK, and SDL3 at run time.

```
tools/build-linux.sh
bin/openpowermac
```

Alpine, Windows, the release packages and their checks:
[Building](https://github.com/codingncaffeine/OpenPowerMac/wiki/Building).

## How it is made

From primary documentation, never from another emulator's source. Every hardware fact the
machine relies on is in the wiki's
[Clean-Room Receipts](https://github.com/codingncaffeine/OpenPowerMac/wiki/Clean-Room-Receipts)
ledger, with its source and how it was verified.

## License

MIT. See [LICENSE](LICENSE); what the release packages carry beside the emulator is in
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
