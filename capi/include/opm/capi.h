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
// fastTb scales the timebase: the machine advances it (1 + fastTb)/4 ticks per
// instruction, so it fixes guest time to THIS host's throughput. ⚠⚠ THAT IS A
// TRAP AT HIGH VALUES ONCE THE GUEST'S CLOCK IS CORRECT. Mac OS spends about
// 32,000 emulated instructions on each 60 Hz tick (the 68K VBL chain, the
// Time Manager, CrsrTask), and fastTb 60 leaves it 416,666/15.25 = 27,300 —
// less than the work costs, so the machine services ticks back to back and
// never boots. Measured: fastTb 60 stops dead at 2.5 G instructions with one
// distinct scanline on screen; 30 and below reach the desktop. Prefer
// opm_set_realtime, which sizes the interval from the host clock instead of
// guessing, and leave fastTb for deterministic runs.
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

// Pace the timebase from the HOST CLOCK (25 MHz, one tick per 40 ns) instead
// of from the instruction count, and anchor it to where the machine is now.
// This is what makes the guest's clock true: its 60 Hz chain then emits 60
// Ticks per host second on any host, and the emulator has (instructions per
// second)/60 instructions to spend on each tick rather than a number fixed by
// fastTb. Call it any time; it re-anchors. fastTb is ignored while it is on.
OPM_API void opm_set_realtime(OpmMachine* m, int32_t on);

// How many times the machine could not keep up and the debt was forgiven. A
// run with many slips is not running at real time and its tick rate is a lie.
OPM_API uint64_t opm_rt_slips(const OpmMachine* m);

// The guest's timebase. /25,000,000 is its uptime in its own seconds; over
// host seconds it says whether the machine is actually running at real time.
OPM_API uint64_t opm_tb(const OpmMachine* m);

// Queue text for the serial console (CRs included by the caller).
OPM_API void opm_serial(OpmMachine* m, const char* text);

// Drain newly produced serial-console output into buf (returns bytes).
OPM_API uint32_t opm_console(OpmMachine* m, char* buf, uint32_t cap);

// USB HID input. The keyboard is usb@8 and the mouse usb@9; both speak the
// boot protocol, so the guest needs nothing beyond the driver Mac OS already
// loads. Motion is RELATIVE in the mouse's own units, so a host with an
// absolute pointer sends the delta since its last report, and `buttons` is a
// bitmask (bit 0 = left) that must be resent while a button is held.
OPM_API void opm_key(OpmMachine* m, const char* text);
// One key going down or coming up, as a HID usage code. This is what a
// person's keyboard sends; opm_key above turns text into keystrokes, which is
// right for a script and wrong for a user — Backspace, Tab, the arrows,
// Delete, the function keys and every modifier have no spelling in text.
// Usages 0xE0-0xE7 are the modifiers and fold into the report's modifier
// byte; the guest applies its own keyboard layout to everything else.
OPM_API void opm_key_event(OpmMachine* m, uint32_t usage, uint32_t down);
OPM_API void opm_mouse(OpmMachine* m, int32_t dx, int32_t dy,
                       uint32_t buttons);

// Snapshot the display. Returns 1 and fills w/h if the CRTC is live and
// the caller's buffer (BGRA, w*h*4 bytes, cap in bytes) was big enough;
// returns 0 with w/h set when only the size query succeeded; returns -1
// when there is no picture to show.
OPM_API int32_t opm_screen(OpmMachine* m, uint8_t* bgra, uint32_t cap,
                           uint32_t* w, uint32_t* h);

// Machine state peeks for the shell's status line.
OPM_API uint64_t opm_executed(const OpmMachine* m);
OPM_API uint32_t opm_pc(const OpmMachine* m);
