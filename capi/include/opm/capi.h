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

// ⛔ WHETHER THE MACHINE STOPPED, ASKED DIRECTLY. A caller cannot infer this
// from opm_run returning the same instruction count it was given before:
// under real-time pacing an IDLE machine legitimately executes nothing for a
// whole call, because the run loop spends the time waiting for the guest's
// next deadline instead of spinning. Those two states need telling apart —
// one is a Mac sitting at the Finder with nothing to do, the other is a dead
// machine — and only this can tell them apart.
OPM_API int32_t opm_halted(const OpmMachine* m);

// ⏳ HOW LONG THIS MACHINE HAS SPENT OFF THE HOST PROCESSOR, in nanoseconds,
// since it was created. Over elapsed host time it is the share of a core the
// emulator is NOT burning, which is the only way to tell an idle machine that
// waits from one that spins — they look identical from outside, and the
// difference is a whole core. Meaningful only under real-time pacing;
// instruction-paced runs never wait, by design.
OPM_API uint64_t opm_idle_ns(const OpmMachine* m);

// 🩺 WHY IS THIS MACHINE NOT DOING ANYTHING? Fills `buf` with a text report of
// the state a stall actually depends on: where the processor is and whether it
// is asleep or halted, the interrupt controller (which source is raised, which
// is in service, and how many times each has raised), and both ATA cells with
// their data-phase counters. Returns the bytes written.
//
// ⭐ It exists because a wedge that happens while someone is USING the machine
// is unreproducible from the command line — an OS install cannot be driven
// headlessly — and every instrument this project owns printed to a stdout the
// app does not have. A frozen emulator and a guest waiting forever on a device
// look identical from outside; only these counters separate them.
OPM_API uint32_t opm_diag(OpmMachine* m, char* buf, uint32_t cap);

// 📁 Build a classic HFS volume image from a host folder (UTF-8 paths),
// for attaching in the CD slot — the shared-folder transfer path. Returns 1
// on success; on failure returns 0. `err` (truncated to errCap) carries the
// refusal reason on failure, and on success the builder's WARNINGS — files
// the share had to leave out — empty when the pack was clean. The volume is
// refused above the classic 2 GB line: the guest addresses block I/O with
// signed 32-bit byte offsets, so a bigger share mounts and browses and then
// fails every copy. Pure host-side work: no machine required, callable
// before opm_create.
OPM_API int opm_hfs_build(const char* folderUtf8, const char* outPathUtf8,
                          const char* volName, char* err,
                          uint32_t errCap);

// 📥 The drop box: a second CD-class drive on the Sawtooth's third ATA
// channel (ata-3@21000 — the Zip bay on the real board), whose disc is an
// HFS volume the host builds from a folder (opm_hfs_build). Two rules from
// the hardware:
//   - the DRIVE must be on the channel before the first instruction — the
//     firmware's bus scan and the OS's ATA Manager enumerate drives exactly
//     once — so call opm_drop_seat (empty tray) or opm_drop_attach (with a
//     disc) right after opm_create, never later;
//   - the DISC changes only when the guest is not holding it. opm_drop_swap
//     is the SAFE republish: an empty tray takes the image at once (the
//     driver polls an empty drive every ~1.7 s and mounts a new disc in
//     ~0.5 s, measured); a disc the guest never read is swapped with the
//     UNIT ATTENTION a real drive raises; a disc the guest has READ stays
//     in — Mac OS locks a mounted CD, never polls it again, and (measured,
//     s45 E3) answers a swap under it by reading the NEW disc through the
//     OLD catalog: a file opened afterwards shows garbage. So the image
//     waits (state 5) until the guest puts the volume away (drag the
//     Drop Box to the Trash, or Cmd-Y), and goes in right after. Nothing
//     is injected into the guest and no restart is needed.
// Paths are opened like opm_create's (narrow fopen). Call swap/eject from
// the thread that runs the machine (like opm_key).
// opm_drop_state: 0 no drive · 1 tray empty · 2 swap staged (tray empty,
// a disc waiting to go in) · 3 disc in, the guest has not looked yet ·
// 4 disc in and read from (mounted, or being mounted) · 5 disc in and read,
// a newer image waiting for the guest to put the volume away. opm_drop_reads is
// the block count read from the current disc. opm_drop_error carries the
// last refusal (an image that would not open).
OPM_API int32_t opm_drop_seat(OpmMachine* m);
OPM_API int32_t opm_drop_attach(OpmMachine* m, const char* imgPath);
OPM_API int32_t opm_drop_swap(OpmMachine* m, const char* imgPath);
OPM_API void opm_drop_eject(OpmMachine* m);
OPM_API int32_t opm_drop_state(const OpmMachine* m);
OPM_API uint64_t opm_drop_reads(const OpmMachine* m);
OPM_API uint32_t opm_drop_error(const OpmMachine* m, char* buf, uint32_t cap);

// 💿 Whether a CD is actually in the drive, and if attach refused, why.
// opm_create attaches the CD silently — a cdPath whose image cannot be
// read (a cue naming a missing .bin, a compressed .dmg in a codec we do
// not carry) previously left the machine booting with an EMPTY drive and
// nothing saying so. Ask after create: 1 = media present. opm_cd_error
// fills `buf` with the refusal reason (empty string when none) and
// returns the bytes written.
OPM_API int32_t opm_cd_present(const OpmMachine* m);
OPM_API uint32_t opm_cd_error(const OpmMachine* m, char* buf, uint32_t cap);

// 🏭 What opm_create did to the boot ROM's system-configuration block, if
// anything: empty when the dump already described a Power Mac G4 (AGP
// Graphics); otherwise one sentence saying what the block described and
// that it was factory-configured (a ROM assembled from Apple's firmware
// updater carries a template that sends HWInit to poll hardware this
// machine does not have). Fills `buf`, returns the bytes written.
OPM_API uint32_t opm_rom_note(const OpmMachine* m, char* buf, uint32_t cap);

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

// ---- audio ----
// Drain PCM the guest has handed to the sound codec: 16-bit signed stereo
// samples, BIG-ENDIAN, interleaved left then right — the byte order Mac OS
// writes and the DBDMA channel carries unaltered. Returns bytes written
// (<= cap, always a multiple of 4). Poll it regularly: the codec queues about
// six seconds and then drops the oldest, so a front end that never drains
// hears the machine skip.
//
// The rate is not fixed — the Sound Control register selects it and the guest
// may change it — so ask opm_audio_rate() alongside, and re-open the host
// device when it changes. The boot ROM's startup chime comes out of this path
// at about 8.3 M instructions, which is the cheapest end-to-end test there is.
OPM_API uint32_t opm_audio(OpmMachine* m, uint8_t* out, uint32_t cap);
OPM_API uint32_t opm_audio_rate(const OpmMachine* m);
// Total bytes the codec has accepted since power-on. A status line can
// divide it by (rate*4) for the guest's own idea of how long it has played.
OPM_API uint64_t opm_audio_played(const OpmMachine* m);

// Snapshot the display. Returns 1 and fills w/h if the CRTC is live and
// the caller's buffer (BGRA, w*h*4 bytes, cap in bytes) was big enough;
// returns 0 with w/h set when only the size query succeeded; returns -1
// when there is no picture to show.
OPM_API int32_t opm_screen(OpmMachine* m, uint8_t* bgra, uint32_t cap,
                           uint32_t* w, uint32_t* h);

// 📸 SAVE THE WHOLE MACHINE TO A FILE, from the app, mid-run.
//
// ⭐ THIS IS THE HANDOFF FROM "the user reproduces it" TO "the harness
// reproduces it". A rendering defect that only shows up inside a running
// game costs one HUMAN launch per iteration — boot, mount, launch, play to
// the right screen, capture — and no amount of instrumentation shortens
// that loop, because the loop is a person. A snapshot taken once at the
// broken frame turns every later iteration into a headless resume that
// lands on that exact frame in seconds, with the fix under test.
//
// Returns 1 on success, 0 with a message in `err` when the file could not
// be written. Called at an instruction boundary between run chunks — the
// same place diagnostics are taken — so the resume re-enters the loop in
// exactly the state the write saw.
//
// ⚠ A snapshot pairs with ITS OWN disk image and RAM size (see
// snapshot.hpp): resuming against a different or mutated image is not
// supported and the loader says so rather than guessing.
OPM_API int32_t opm_snapshot_save(OpmMachine* m, const char* path, char* err,
                                  uint32_t errCap);

// Machine state peeks for the shell's status line.
OPM_API uint64_t opm_executed(const OpmMachine* m);
OPM_API uint32_t opm_pc(const OpmMachine* m);
