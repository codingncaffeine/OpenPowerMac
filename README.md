# OpenPowerMac

A from-scratch, clean-room Power Macintosh emulator.

The project is being built in arcs. **Arc 1 — a complete MPC7400 (PowerPC G4) CPU core —
is done**: every instruction, register, exception, and MMU behavior, implemented from the
processor's own documentation (Motorola/NXP user's manuals, the PowerPC Programming
Environments Manual, and the AltiVec Technology manuals), validated by a self-authored
known-answer suite, a cross-compiler differential oracle, bare-metal supervisor kernel
proofs, and a byte-identical cross-compiler determinism gate. Later arcs build the machine
around it: a Power Mac G4, Mac OS 9 and Mac OS X, and GPU-accelerated 3D.

Design, methodology, and the clean-room provenance ledger live in the
[wiki](https://github.com/codingncaffeine/OpenPowerMac/wiki).

No Apple software is included or distributed. ROMs and operating system media must be
supplied by the user from their own hardware.

## Building

Requires CMake and a C++20 compiler (MSVC 2022 and GCC are both kept green).

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release
```

The Windows front end (`shell/`, WPF on .NET) builds separately once the native library
has been built — it picks `opmcapi.dll` up from `shell/native/`:

```
cd shell
dotnet build -c Release
```

## License

MIT — see [LICENSE](LICENSE).
