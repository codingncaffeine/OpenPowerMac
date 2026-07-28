#pragma once
// OpenPowerMac C API — the shell boundary. One opaque machine handle,
// plain C types, no callbacks: the front end owns the run thread and
// polls for pixels and console text. Mirrors the OpenMac architecture
// (WPF over a flat C surface, no SDL).

#include <stdint.h>

#ifdef _WIN32
#define OPM_API extern "C" __declspec(dllexport)
#else
#define OPM_API extern "C"
#endif

typedef struct OpmMachine OpmMachine;

// Create a Sawtooth. Any of cdPath/hdPath/atiRomPath may be null.
//
// fastTb scales the timebase. 60 was the practiced value for reaching the
// firmware, but it runs guest time about seven times fast and drives the OS
// era into a decrementer storm — measured at one 60 Hz tick per host second
// against a real sixty. 7 lands near real time at present host speed.
//
// hdPath is not optional in practice: every boot that reaches Mac OS does so
// from the hard disk, and a machine created without one only reaches the
// firmware. atiRomPath is likewise required for any picture — without the
// card's FCode there is no display node for the OS to bind a driver to.
OPM_API OpmMachine* opm_create(const char* romPath, const char* cdPath,
                               const char* hdPath, const char* atiRomPath,
                               uint32_t ramMb, uint32_t fastTb);

OPM_API void opm_destroy(OpmMachine* m);

// Defer PCI visibility of the ATI card until `insn` executed instructions
// (0 = visible from reset). The practiced boot hides the card past OF's
// console choice (~228M) so the serial console stays owned; the card node
// is then created via the Forth bridge. 236M is the practiced value.
OPM_API void opm_ati_at(OpmMachine* m, uint64_t insn);

// Execute up to `insns` more instructions; returns the machine's total.
OPM_API uint64_t opm_run(OpmMachine* m, uint64_t insns);

// Queue text for the serial console (CRs included by the caller).
OPM_API void opm_serial(OpmMachine* m, const char* text);

// Drain newly produced serial-console output into buf (returns bytes).
OPM_API uint32_t opm_console(OpmMachine* m, char* buf, uint32_t cap);

// Snapshot the display. Returns 1 and fills w/h if the CRTC is live and
// the caller's buffer (BGRA, w*h*4 bytes, cap in bytes) was big enough;
// returns 0 with w/h set when only the size query succeeded; returns -1
// when there is no picture to show.
OPM_API int32_t opm_screen(OpmMachine* m, uint8_t* bgra, uint32_t cap,
                           uint32_t* w, uint32_t* h);

// Machine state peeks for the shell's status line.
OPM_API uint64_t opm_executed(const OpmMachine* m);
OPM_API uint32_t opm_pc(const OpmMachine* m);
