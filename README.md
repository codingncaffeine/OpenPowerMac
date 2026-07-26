# OpenPowerMac

A from-scratch, clean-room Power Macintosh emulator.

The project is being built in arcs. **Arc 1 (current): a complete MPC7400 (PowerPC G4) CPU
core** — every instruction, register, exception, and MMU behavior, implemented from the
processor's own documentation (Motorola/NXP user's manuals, the PowerPC Programming
Environments Manual, and the AltiVec Technology manuals), validated by a self-authored
known-answer suite and a cross-compiler differential oracle. Later arcs build the machine
around it: a New World Power Mac G4, Mac OS 9 and Mac OS X, and GPU-accelerated 3D.

No Apple software is included or distributed. ROMs and operating system media must be
supplied by the user from their own hardware.

## Building

Requires CMake and a C++20 compiler (MSVC 2022 and GCC are both kept green).

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release
```

## License

MIT — see [LICENSE](LICENSE).
