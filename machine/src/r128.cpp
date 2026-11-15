#include "opm/r128.hpp"
#include "opm/r128rec.hpp" // the frame recorder's taps
#include "opm/snapshot.hpp" // the engine's own state, for a recording

#include <vector>
#include <string>
#include <map>
#include <cstdarg>
#include <cstdio> // the CRTC mode-change report; MSVC pulls this in, gcc does not
#include "opm/bus.hpp" // SnoopSink: the bus-master fetch answers to it
#include "opm/prof.hpp" // OPM_MARK(Cce): the engine's own profile bucket
#include <cstring> // memmove, for the blitter's row copies
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace opm {

// Rage 128 register offsets the bring-up touches (register names per
// the chip's public lineage; values earn semantics as the FCode and
// the OS driver demand them).
static constexpr u32 kMmIndex = 0x0000;      // MM_INDEX / MM_DATA pair
static constexpr u32 kClockCntlIndex = 0x0008;
static constexpr u32 kClockCntlData = 0x000C;
static constexpr u32 kBiosScratch = 0x0010;  // BIOS_0_SCRATCH..
static constexpr u32 kGenReset = 0x00F0;     // GEN_RESET_CNTL
static constexpr u32 kConfigMemsize = 0x00F8;
static constexpr u32 kConfigAper0Base = 0x0100; // CONFIG_APER_0_BASE
static constexpr u32 kConfigAper1Base = 0x0104; // CONFIG_APER_1_BASE
static constexpr u32 kConfigAperSize = 0x0108;  // CONFIG_APER_SIZE
static constexpr u32 kConfigReg1Base = 0x010C;  // CONFIG_REG_1_BASE
static constexpr u32 kConfigRegAperSize = 0x0110;
// The big-endian alias of the same VRAM: BAR0 is 64 MB and the upper half
// is the byte-swapped view of the lower 32 MB.
static constexpr u32 kAper1Offset = 0x02000000;
static constexpr u32 kMemCntl = 0x0140;
static constexpr u32 kCrtcGenCntl = 0x0050;
static constexpr u32 kGpioMonid = 0x0068;    // DDC bit-bang
static constexpr u32 kGpioDvi = 0x006C;
// The I/O-aperture GPIO trio Open Firmware bit-bangs during monitor sense:
// it drives bits 22 and 23 through 0x00A0 and 0x00A8 and reads 0x00A4 back.
// Answering zero is a line held LOW, which on a two-wire bus is a stuck bus
// and not "no monitor" — undriven lines float high on their pull-ups.
static constexpr u32 kGpioA0 = 0x00A0;
static constexpr u32 kGpioA4 = 0x00A4;
static constexpr u32 kGpioA8 = 0x00A8;
// Which line is which, and which bit is the direction, are pinned by
// experiment: the address byte the slave latches is 0xFF until they are
// right, because a released SDA reads high for all eight bits.
static constexpr bool kLevelIsA0 = true; // else 0xA8 holds the levels
static constexpr u32 kSclBit = 0x00400000u;
static constexpr u32 kSdaBit = 0x00800000u;
// These pins carry two protocols, and the waveform capture shows the two
// phases using them differently. Open Firmware writes the LEVELS register
// and reads back static combinations — Apple monitor sense, not a
// transaction: 33 line states and two starts in a whole boot. Mac OS's
// driver never writes a level at all; it works purely open drain through
// the enables, 160 states and 617 starts, which is DDC/I2C.
//
// Sense only cares what regRead reports back; I2C only cares which bit the
// state machine clocks on. So the two can be decoupled, and must be: the
// earlier attempt to swap kSclBit/kSdaBit fixed the clock but changed the
// sense readback with it, which moved what the FCode publishes and made
// the CRTC fall back to Open Firmware's mode. Keep the readback layout
// exactly as it was and give the DDC state machine its own bit roles,
// which the trace names unambiguously: from state 43 the driver toggles
// bit 23 and holds bit 22, so bit 23 is the clock.
static constexpr u32 kDdcSclBit = 0x00800000u;
static constexpr u32 kDdcSdaBit = 0x00400000u;
static constexpr u32 kSenseIn = 0x00C00000u;
static constexpr u32 kGenIntCntl = 0x0040;   // bit 0 = CRTC_VBLANK_INT
static constexpr u32 kGenIntStatus = 0x0044;
static constexpr u32 kPllTest = 0x0000;
// GEN_INT_CNTL/GEN_INT_STATUS bit assignments.
static constexpr u32 kCrtcVblankInt = 0x00000001u;
// GEN_INT_STATUS is write-1-to-clear, and only the bits a source actually
// latches are writable. This is the Rage 128 PF ('PF', 1002:5046) mask; the
// later Radeon parts use a wider one, and treating every bit as writable lets
// an acknowledge clear latches the hardware would have held.
static constexpr u32 kGenIntAckMask = 0x000F040Fu;
// One blank per frame: 60 Hz against a 25 MHz timebase.
//
// This used to be 225,000,000 — 540 times too long — and its own comment
// explained why: the OS's Ticks reached 9,358 while the timebase said 6,958
// seconds had passed, so "guest-perceived time" looked 44.6x compressed and
// the constant was scaled to suit. That 44.6x was not a property of the
// machine. It was a defect: the guest calibrated its clock against a KeyLargo
// timer nothing answered, saturated its timebase frequency to 2^30, and ran
// every Duration 43x long (SawtoothBus::kKlTimerLo). With the timer answering,
// Ticks tracks the timebase at 60 Hz and the nominal period is simply correct.
//
// A constant chosen to cancel a bug outlives the bug and reads like a model.
//
// ⚠ IT MOVED WITH THE TIMER, WHICH IS THE ONLY WAY IT COULD MOVE.
// SawtoothBus::klTimerOn is now on by default, so the compression is gone and
// the nominal period is simply the right one. Measured before flipping it,
// cold boot, one variable (--ati-vbl-tb 416666 against the same run without):
// 462 distinct scanlines either way, 292 disk commands, ati paint 1,261,505,
// 62.2 against 62.7 guest Ticks per host second, 23.5 against 23.7 MIPS. The
// blank census goes from 72 raised / 1 acknowledged to 38,901 / 11 — the OS
// services a few and lets the rest expire, and SESSION27_PLAN §6's "wedges
// after two blanks" does NOT reproduce with a correct clock.
//
// The cell's vblEnabled is a CONSTRUCTOR default of true, so this is live in
// the app. --ati-vbl-tb still overrides it.
static constexpr u64 kTbPerVblank = 416666ull;
// Harness knobs, not machine state — see the notes in the header.
static u64 gVblTbPeriod = 0;
static int gVblTrace = 0;
// Census of expired (unserviced) requests. A file-static because censuses are
// deliberately not snapshotted, and because it keeps sizeof(R128Cell) fixed.
static u64 gVblDropped = 0;

// 📊 THE READS WE NEVER ACTUALLY ANSWER.
//
// `readCount` already says which registers the guest touches and how often.
// What it cannot say is whether this model ANSWERED any of them, and that is
// the difference that matters: ⚠ a register this model does not implement is
// NOT silent. The card claims its whole register aperture, so a read of an
// unmodeled offset never reaches the unclaimed-access log — it falls through
// to `default:` and reads back the last value written there, or zero. A guest
// driver polling such a register for a change waits forever, and the only
// trace it leaves is a machine that stopped making progress. Session 30 wrote
// this down for mac-io as *claiming an address and implementing it are
// different things*; it is the same trap on the other device, and it cost a
// second session.
//
// So count separately the reads that fell to the default with NOTHING ever
// written there. A register read four million times that has only ever
// returned zero is an unimplemented register being spun on — which reads
// nothing like a register that is merely busy.
//
// File-static for the same reason as the censuses above: it keeps
// sizeof(R128Cell) fixed, so an instrument cannot invalidate a snapshot.
static std::map<u32, u64> gRegReadsUnbacked;

const std::map<u32, u64>& r128RegReadsUnbacked() { return gRegReadsUnbacked; }

// 📇 THE ENGINE REGISTERS, BY NAME, FROM THE VENDOR'S OWN DOCUMENT.
//
// ATI's `RAGE 128 PRO Register Reference Guide` (RRG-G04500-C rev 1.01, Jan
// 2000) is the authority for everything in the GUI block; its OEM edition
// leaves sections 3.31–3.34 — the parameter FIFO, GUI engine control/status
// and GUI bus mastering — as `<No description>`, so the CCE/PM4 offsets below
// are marked and come from QEMU's map instead. An offset that neither names
// gets no name at all: a log that prints `+0908` and nothing else is telling
// the truth about what is known, which is the point of the instrument.
struct R128Name {
    u32 off;
    const char* name;
};
static const R128Name kEngNames[] = {
    {0x0700, "PM4_BUFFER_OFFSET"},
    {0x0704, "PM4_BUFFER_CNTL"},
    {0x0708, "PM4_BUFFER_WM_CNTL"},
    {0x070c, "PM4_BUFFER_DL_RPTR_ADDR"},
    {0x0710, "PM4_BUFFER_DL_RPTR"},
    {0x0714, "PM4_BUFFER_DL_WPTR"},
    {0x0718, "PM4_BUFFER_DL_WPTR_DELAY"},
    {0x071c, "PM4_VC_FPU_SETUP"},
    {0x0720, "PM4_FPU_CNTL"},
    {0x0724, "PM4_VC_FORMAT"},
    {0x0728, "PM4_VC_CNTL"},
    {0x072c, "PM4_VC_I01"},
    {0x0730, "PM4_VC_VLOFF"},
    {0x0734, "PM4_VC_VLSIZE"},
    {0x0738, "PM4_IW_INDOFF"},
    {0x073c, "PM4_IW_INDSIZE"},
    {0x0740, "CRC_CMDFIFO_ADDR"},
    {0x0744, "CRC_CMDFIFO_DOUT"},
    {0x0748, "PM4_FPU_FPX1"},
    {0x074c, "PM4_FPU_FPY1"},
    {0x0750, "PM4_FPU_FPX2"},
    {0x0754, "PM4_FPU_FPY2"},
    {0x0758, "PM4_FPU_FPY3"},
    {0x075c, "PM4_FPU_FPY4"},
    {0x0760, "PM4_FPU_FPY5"},
    {0x0764, "PM4_FPU_FPY6"},
    {0x0768, "PM4_FPU_FPR"},
    {0x076c, "PM4_FPU_FPG"},
    {0x0770, "PM4_FPU_FPB"},
    {0x0774, "PM4_FPU_FPA"},
    {0x0780, "PM4_FPU_INTXY0"},
    {0x0784, "PM4_FPU_INTXY1"},
    {0x0788, "PM4_FPU_INTXY2"},
    {0x078c, "PM4_FPU_INTARGB"},
    {0x0790, "PM4_FPU_FPTWICEAREA"},
    {0x0794, "PM4_FPU_DMAJOR01"},
    {0x0798, "PM4_FPU_DMAJOR12"},
    {0x079c, "PM4_FPU_DMAJOR02"},
    {0x07a0, "PM4_FPU_STAT"},
    {0x07a4, "PM4_VC_DEBUG_CONFIG"},
    {0x07a8, "PM4_VC_STAT"},
    {0x07b0, "PM4_VC_TIMESTAMP0"},
    {0x07b4, "PM4_VC_TIMESTAMP1"},
    {0x07b8, "PM4_STAT"},
    {0x07d0, "PM4_TEST_CNTL"},
    {0x07d4, "PM4_MICROCODE_ADDR"},
    {0x07d8, "PM4_MICROCODE_RADDR"},
    {0x07dc, "PM4_MICROCODE_DATAH"},
    {0x07e0, "PM4_MICROCODE_DATAL"},
    {0x07e4, "PM4_CMDFIFO_ADDR"},
    {0x07e8, "PM4_CMDFIFO_DATAH"},
    {0x07ec, "PM4_CMDFIFO_DATAL"},
    {0x07f0, "PM4_BUFFER_ADDR"},
    {0x07f4, "PM4_BUFFER_DATAH"},
    {0x07f8, "PM4_BUFFER_DATAL"},
    {0x07fc, "PM4_MICRO_CNTL"},
    {0x0900, "VID_BUFFER_CONTROL"},
    {0x0950, "CAP0_TRIG_CNTL"},
    {0x09c0, "CAP1_TRIG_CNTL"},
    {0x0a14, "BM_QUEUE_FREE_STATUS"},
    {0x0a88, "BM_ABORT"},
    {0x0b00, "SURFACE_DELAY"},
    {0x0b04, "SURFACE0_LOWER_BOUND"},
    {0x0b08, "SURFACE0_UPPER_BOUND"},
    {0x0b0c, "SURFACE0_INFO"},
    {0x0b14, "SURFACE1_LOWER_BOUND"},
    {0x0b18, "SURFACE1_UPPER_BOUND"},
    {0x0b1c, "SURFACE1_INFO"},
    {0x0b24, "SURFACE2_LOWER_BOUND"},
    {0x0b28, "SURFACE2_UPPER_BOUND"},
    {0x0b2c, "SURFACE2_INFO"},
    {0x0b34, "SURFACE3_LOWER_BOUND"},
    {0x0b38, "SURFACE3_UPPER_BOUND"},
    {0x0b3c, "SURFACE3_INFO"},
    {0x0b44, "AGP_CNTL_B"},
    {0x0c00, "VIPH_CH0_DATA"},
    {0x0c04, "VIPH_CH1_DATA"},
    {0x0c08, "VIPH_CH2_DATA"},
    {0x0c0c, "VIPH_CH3_DATA"},
    {0x0e40, "RBBM_STATUS"},
    {0x1000, "PM4_FIFO_DATA_EVEN"},
    {0x1004, "PM4_FIFO_DATA_ODD"},
    {0x1404, "DST_OFFSET"},
    {0x1408, "DST_PITCH"},
    {0x140c, "DST_WIDTH"},
    {0x1410, "DST_HEIGHT"},
    {0x1414, "SRC_X"},
    {0x1418, "SRC_Y"},
    {0x141c, "DST_X"},
    {0x1420, "DST_Y"},
    {0x1428, "SRC_PITCH_OFFSET"},
    {0x142c, "DST_PITCH_OFFSET"},
    {0x1434, "SRC_Y_X"},
    {0x1438, "DST_Y_X"},
    {0x143c, "DST_HEIGHT_WIDTH"},
    {0x146c, "DP_GUI_MASTER_CNTL"},
    {0x1470, "BRUSH_SCALE"},
    {0x1474, "BRUSH_Y_X"},
    {0x1478, "DP_BRUSH_BKGD_CLR"},
    {0x147c, "DP_BRUSH_FRGD_CLR"},
    {0x1588, "DST_WIDTH_X"},
    {0x158c, "DST_HEIGHT_WIDTH_8"},
    {0x1590, "SRC_X_Y"},
    {0x1594, "DST_X_Y"},
    {0x1598, "DST_WIDTH_HEIGHT"},
    {0x159c, "DST_WIDTH_X_INCY"},
    {0x15a0, "DST_HEIGHT_Y"},
    {0x15a4, "DST_X_SUB"},
    {0x15a8, "DST_Y_SUB"},
    {0x15ac, "SRC_OFFSET"},
    {0x15b0, "SRC_PITCH"},
    {0x15b4, "DST_WIDTH_BW"},
    {0x15c0, "CLR_CMP_CNTL"},
    {0x15c4, "CLR_CMP_CLR_SRC"},
    {0x15c8, "CLR_CMP_CLR_DST"},
    {0x15cc, "CLR_CMP_MSK"},
    {0x15d8, "DP_SRC_FRGD_CLR"},
    {0x15dc, "DP_SRC_BKGD_CLR"},
    {0x15e0, "GUI_SCRATCH_REG0"},
    {0x15e4, "GUI_SCRATCH_REG1"},
    {0x15e8, "GUI_SCRATCH_REG2"},
    {0x15ec, "GUI_SCRATCH_REG3"},
    {0x15f0, "GUI_SCRATCH_REG4"},
    {0x15f4, "GUI_SCRATCH_REG5"},
    {0x1600, "LEAD_BRES_ERR"},
    {0x1604, "LEAD_BRES_INC"},
    {0x1608, "LEAD_BRES_DEC"},
    {0x160c, "TRAIL_BRES_ERR"},
    {0x1610, "TRAIL_BRES_INC"},
    {0x1614, "TRAIL_BRES_DEC"},
    {0x1618, "TRAIL_X"},
    {0x161c, "LEAD_BRES_LNTH"},
    {0x1620, "TRAIL_X_SUB"},
    {0x1624, "LEAD_BRES_LNTH_SUB"},
    {0x1628, "DST_BRES_ERR"},
    {0x162c, "DST_BRES_INC"},
    {0x1630, "DST_BRES_DEC"},
    {0x1634, "DST_BRES_LNTH"},
    {0x1638, "DST_BRES_LNTH_SUB"},
    {0x1640, "SC_LEFT"},
    {0x1644, "SC_RIGHT"},
    {0x1648, "SC_TOP"},
    {0x164c, "SC_BOTTOM"},
    {0x1654, "SRC_SC_RIGHT"},
    {0x165c, "SRC_SC_BOTTOM"},
    {0x1660, "AUX_SC_CNTL"},
    {0x1664, "AUX1_SC_LEFT"},
    {0x1668, "AUX1_SC_RIGHT"},
    {0x166c, "AUX1_SC_TOP"},
    {0x1670, "AUX1_SC_BOTTOM"},
    {0x1674, "AUX2_SC_LEFT"},
    {0x1678, "AUX2_SC_RIGHT"},
    {0x167c, "AUX2_SC_TOP"},
    {0x1680, "AUX2_SC_BOTTOM"},
    {0x1684, "AUX3_SC_LEFT"},
    {0x1688, "AUX3_SC_RIGHT"},
    {0x168c, "AUX3_SC_TOP"},
    {0x1690, "AUX3_SC_BOTTOM"},
    {0x16a0, "GUI_DEBUG0"},
    {0x16a4, "GUI_DEBUG1"},
    {0x16a8, "GUI_DEBUG2"},
    {0x16ac, "GUI_DEBUG3"},
    {0x16b0, "GUI_DEBUG4"},
    {0x16b4, "GUI_DEBUG5"},
    {0x16b8, "GUI_DEBUG6"},
    {0x16bc, "GUI_PROBE"},
    {0x16c0, "DP_CNTL"},
    {0x16c4, "DP_DATATYPE"},
    {0x16c8, "DP_MIX"},
    {0x16cc, "DP_WRITE_MSK"},
    {0x16d0, "DP_CNTL_XDIR_YDIR_YMAJOR"},
    {0x16e0, "DEFAULT_OFFSET"},
    {0x16e4, "DEFAULT_PITCH"},
    {0x16e8, "DEFAULT_SC_BOTTOM_RIGHT"},
    {0x16ec, "SC_TOP_LEFT"},
    {0x16f0, "SC_BOTTOM_RIGHT"},
    {0x16f4, "SRC_SC_BOTTOM_RIGHT"},
    {0x1700, "DST_TILE"},
    {0x1704, "FLUSH_1"},
    {0x1708, "FLUSH_2"},
    {0x170c, "FLUSH_3"},
    {0x1710, "FLUSH_4"},
    {0x1714, "FLUSH_5"},
    {0x1718, "FLUSH_6"},
    {0x171c, "FLUSH_7"},
    {0x1720, "WAIT_UNTIL"},
    {0x1724, "CACHE_CNTL"},
    {0x1740, "GUI_STAT"},
    {0x1744, "PC_GUI_MODE"},
    {0x1748, "PC_GUI_CTLSTAT"},
    {0x1760, "PC_DEBUG_MODE"},
    {0x1780, "BRES_DST_ERR_DEC"},
    {0x1784, "TRAIL_BRES_T12_ERR_DEC"},
    {0x1788, "TRAIL_BRES_T12_INC"},
    {0x178c, "DP_T12_CNTL"},
    {0x1790, "DST_BRES_T1_LNTH"},
    {0x1794, "DST_BRES_T2_LNTH"},
    {0x17c0, "HOST_DATA0"},
    {0x17c4, "HOST_DATA1"},
    {0x17c8, "HOST_DATA2"},
    {0x17cc, "HOST_DATA3"},
    {0x17d0, "HOST_DATA4"},
    {0x17d4, "HOST_DATA5"},
    {0x17d8, "HOST_DATA6"},
    {0x17dc, "HOST_DATA7"},
    {0x17e0, "HOST_DATA_LAST"},
    {0x1800, "TEX_CNTL"},
    {0x18cc, "W_START"},
    {0x1980, "SECONDARY_SCALE_PITCH"},
    {0x1984, "SECONDARY_SCALE_X_INC"},
    {0x1988, "SECONDARY_SCALE_Y_INC"},
    {0x198c, "SECONDARY_SCALE_HACC"},
    {0x1990, "SECONDARY_SCALE_VACC"},
    {0x1994, "SCALE_SRC_HEIGHT_WIDTH"},
    {0x1998, "SCALE_OFFSET_0"},
    {0x199c, "SCALE_PITCH"},
    {0x19a0, "SCALE_X_INC"},
    {0x19a4, "SCALE_Y_INC"},
    {0x19a8, "SCALE_HACC"},
    {0x19ac, "SCALE_VACC"},
    {0x19b0, "SCALE_DST_X_Y"},
    {0x19b4, "SCALE_DST_HEIGHT_WIDTH"},
    {0x19d4, "MC_SRC2_CNTL"},
    {0x19d8, "MC_SRC1_CNTL"},
    {0x19dc, "MC_DST_CNTL"},
    {0x19e0, "MC_START_CNTL"},
    {0x1a00, "SCALE_3D_CNTL"},
    {0x1a0c, "COMPOSITE_SHADOW_ID"},
    {0x1a14, "FOG_TABLE_INDEX"},
    {0x1a18, "FOG_TABLE_DATA"},
    {0x1a20, "SCALE_3D_DATATYPE"},
    {0x1a24, "CLR_CMP_CLR_3D"},
    {0x1a28, "CLR_CMP_MSK_3D"},
    {0x1a30, "CONSTANT_COLOR"},
    {0x1ad4, "Z_VIS"},
    {0x1bc4, "SETUP_CNTL"},
    {0x1bc8, "SOLID_COLOR"},
    {0x1bcc, "WINDOW_XY_OFFSET"},
    {0x1bd0, "DRAW_LINE_POINT"},
    {0x1bd4, "SETUP_CNTL_PM4"},
    {0x1c80, "DST_PITCH_OFFSET_C"},
    {0x1c84, "DP_GUI_MASTER_CNTL_C"},
    {0x1c88, "SC_TOP_LEFT_C"},
    {0x1c8c, "SC_BOTTOM_RIGHT_C"},
    {0x1c90, "Z_OFFSET_C"},
    {0x1c94, "Z_PITCH_C"},
    {0x1c98, "Z_STEN_CNTL_C"},
    {0x1c9c, "TEX_CNTL_C"},
    {0x1ca0, "MISC_3D_STATE_CNTL_REG"},
    {0x1ca4, "TEXTURE_CLR_CMP_CLR_C"},
    {0x1ca8, "TEXTURE_CLR_CMP_MSK_C"},
    {0x1cac, "FOG_COLOR_C"},
    {0x1cb0, "PRIM_TEX_CNTL_C"},
    {0x1cb4, "PRIM_TEXTURE_COMBINE_CNTL_C"},
    {0x1cb8, "TEX_SIZE_PITCH_C"},
    {0x1cbc, "PRIM_TEX_0_OFFSET_C"},
    {0x1cc0, "PRIM_TEX_1_OFFSET_C"},
    {0x1cc4, "PRIM_TEX_2_OFFSET_C"},
    {0x1cc8, "PRIM_TEX_3_OFFSET_C"},
    {0x1ccc, "PRIM_TEX_4_OFFSET_C"},
    {0x1cd0, "PRIM_TEX_5_OFFSET_C"},
    {0x1cd4, "PRIM_TEX_6_OFFSET_C"},
    {0x1cd8, "PRIM_TEX_7_OFFSET_C"},
    {0x1cdc, "PRIM_TEX_8_OFFSET_C"},
    {0x1ce0, "PRIM_TEX_9_OFFSET_C"},
    {0x1ce4, "PRIM_TEX_10_OFFSET_C"},
    {0x1d00, "SEC_TEX_CNTL_C"},
    {0x1d04, "SEC_TEX_COMBINE_CNTL_C"},
    {0x1d08, "SEC_TEX_0_OFFSET_C"},
    {0x1d0c, "SEC_TEX_1_OFFSET_C"},
    {0x1d10, "SEC_TEX_2_OFFSET_C"},
    {0x1d14, "SEC_TEX_3_OFFSET_C"},
    {0x1d18, "SEC_TEX_4_OFFSET_C"},
    {0x1d1c, "SEC_TEX_5_OFFSET_C"},
    {0x1d20, "SEC_TEX_6_OFFSET_C"},
    {0x1d24, "SEC_TEX_7_OFFSET_C"},
    {0x1d28, "SEC_TEX_8_OFFSET_C"},
    {0x1d2c, "SEC_TEX_9_OFFSET_C"},
    {0x1d30, "SEC_TEX_10_OFFSET_C"},
    {0x1d34, "CONSTANT_COLOR_C"},
    {0x1d38, "PRIM_TEXTURE_BORDER_COLOR_C"},
    {0x1d3c, "SEC_TEXTURE_BORDER_COLOR_C"},
    {0x1d40, "STEN_REF_MASK_C"},
    {0x1d44, "PLANE_3D_MASK_C"},
};

const char* r128RegName(u32 off)
{
    for (const auto& n : kEngNames)
        if (n.off == off)
            return n.name;
    return nullptr;
}

// The engine half of the card: CCE/PM4 at 0x07xx, video at 0x09xx, bus
// mastering at 0x0Axx, the surface apertures at 0x0Bxx, the PM4 FIFO windows
// at 0x1000/0x1004, and the whole GUI block from 0x1400 up. The display half
// below 0x0700 is modeled and understood, and logging it would bury the
// engine traffic under the driver's DDC bit-banging — which is exactly what
// happened to the existing ring.
static constexpr u32 kEngLo = 0x0700, kEngHi = 0x2000;
// ---- bits ever written as one (see the header) ------------------------------
namespace {
u32 gRegEverSet[0x1000] = {}; // one per register of the 16 KB block
} // namespace
u32 r128RegEverSet(u32 off) { return gRegEverSet[(off >> 2) & 0xFFFu]; }
void r128RegEverReset() { std::memset(gRegEverSet, 0, sizeof gRegEverSet); }

bool r128Reg3dDecodedBits(u32 off, u32& mask); // r128_3d.cpp
bool r128RegDecodedBits(u32 off, u32& mask)
{
    if (r128Reg3dDecodedBits(off, mask))
        return true;
    switch (off) {
    // CLR_CMP_CNTL: the two functions and the source select. What the
    // engine does with each value is in ClrCmp, and an unapplied value is
    // counted there; this only says the fields are read.
    case 0x15C0u: mask = 0x03000707u; return true;
    case 0x15C4u: case 0x15C8u: case 0x15CCu: mask = 0xFFFFFFFFu; return true;
    // DP_GUI_MASTER_CNTL and its context copy (guide 3-171): the four
    // "leave alone" controls, the brush, destination and source datatypes,
    // the ROP, the source select, and the three clear-on-write controls.
    // Bit 27 set means "leave the 3D function alone", which is what happens;
    // its zero is an action not taken, and the value census has it. Bit 14
    // is the pixel order within a pattern's byte; bit 31 says a packet
    // carries the brush origin, which the packet parser then loads. Not
    // read: 15 (color temperature).
    case 0x146Cu: case 0x1C84u: mask = 0xFFFF7FFFu; return true;
    // DP_CNTL: the two directions, the major axis and the sign of a zero
    // error term for lines, and the raster stall (an operation here is
    // complete before the store returns). Not read: the rectangle tiling
    // bits (the value census has them), the last-pel bit, the poly-line and
    // trapezoid fields.
    case 0x16C0u: mask = 0x00010107u; return true;
    case 0x16D0u: mask = 0x80008004u; return true;
    // DP_DATATYPE: destination, brush and source types, the pixel order
    // within a pattern's byte. Not read: 29 (big-endian host data) and 31
    // (as bit 15 above).
    case 0x16C4u: mask = 0x40030F0Fu; return true;
    // DST_PITCH_OFFSET_C: offset and pitch. Bit 31 tiles the destination.
    case 0x1C80u: mask = 0x7FFFFFFFu; return true;
    case 0x1C88u: case 0x1C8Cu: mask = 0x3FFF3FFFu; return true;
    default: return false;
    }
}

// ---- the pixel watch (see the header) --------------------------------------
u32 gR128PwLo = 0, gR128PwHi = 0;
namespace {
std::mutex gPwMu; // the engine's thread and the processor's both report
// The first half of the budget in order, then a ring of the most recent:
// "who drew what is there now" is the tail's question, "what happened when
// it first went wrong" the head's, and a pixel redrawn every frame would
// spend a head-only log on the first second.
std::vector<std::string> gPwHead, gPwTail;
size_t gPwTailAt = 0;
u64 gPwTotal = 0;
std::map<std::string, u64> gPwCount;
u32 gPwBudget = 0;
} // namespace

void r128SetPixelWatch(u32 vramOff, u32 bytes, u32 budget)
{
    std::lock_guard<std::mutex> lk(gPwMu);
    gPwHead.clear();
    gPwTail.clear();
    gPwTailAt = 0;
    gPwTotal = 0;
    gPwCount.clear();
    gPwBudget = budget;
    gR128PwLo = vramOff;
    gR128PwHi = bytes ? vramOff + bytes : 0u;
}

bool r128SetPixelWatchXY(const R128Cell& c, u32 x, u32 y, u32 budget)
{
    const R128Scan s = r128ScanDecode(c);
    if (!s.enabled || !s.bypp || x >= s.w || y >= s.h)
        return false;
    r128SetPixelWatch(s.offset + y * s.rowBytes + x * s.bypp, s.bypp, budget);
    return true;
}

void r128PixelWatchNote(const char* who, const char* fmt, ...)
{
    char text[320];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    std::lock_guard<std::mutex> lk(gPwMu);
    ++gPwCount[who];
    ++gPwTotal;
    const size_t half = gPwBudget / 2u;
    if (!half)
        return;
    std::string line = "#" + std::to_string(gPwTotal) + " " + who + ": " + text;
    if (gPwHead.size() < half) {
        gPwHead.push_back(std::move(line));
    } else if (gPwTail.size() < half) {
        gPwTail.push_back(std::move(line));
    } else {
        gPwTail[gPwTailAt] = std::move(line);
        gPwTailAt = (gPwTailAt + 1u) % half;
    }
}

std::string r128PixelWatchReport()
{
    std::lock_guard<std::mutex> lk(gPwMu);
    if (!gR128PwHi)
        return "-- pixel watch: off\n";
    char b[160];
    snprintf(b, sizeof b,
             "-- pixel watch on video memory %06x..%06x, writes in order:\n",
             gR128PwLo, gR128PwHi - 1u);
    std::string s = b;
    for (const std::string& l : gPwHead)
        s += "--   " + l + "\n";
    if (gPwTotal > gPwHead.size() + gPwTail.size()) {
        snprintf(b, sizeof b, "--   ... %llu not kept ...\n",
                 static_cast<unsigned long long>(gPwTotal - gPwHead.size() -
                                                 gPwTail.size()));
        s += b;
    }
    for (size_t k = 0; k < gPwTail.size(); ++k)
        s += "--   " + gPwTail[(gPwTailAt + k) % gPwTail.size()] + "\n";
    s += "--   by author:";
    if (gPwCount.empty())
        s += " nothing wrote it";
    for (const auto& [who, n] : gPwCount) {
        snprintf(b, sizeof b, " %s %llu", who.c_str(),
                 static_cast<unsigned long long>(n));
        s += b;
    }
    return s + "\n";
}

static size_t gEngLogMax = 0;
static std::vector<R128EngEv> gEngHead, gEngTail, gEngStitched;
static size_t gEngTailAt = 0;
static u64 gEngDropped = 0;

void r128SetEngineLog(size_t maxEntries) { gEngLogMax = maxEntries; }
u64 r128EngineLogDropped() { return gEngDropped; }

static void noteEngine(u32 off, u32 val, u32 pc, u64 at, bool wr)
{
    if (!gEngLogMax || off < kEngLo || off >= kEngHi)
        return;
    const size_t half = gEngLogMax > 1 ? gEngLogMax / 2 : 1;
    if (gEngHead.size() < half) {
        gEngHead.push_back({at, off, val, pc, wr});
        return;
    }
    if (gEngTail.size() < half) {
        gEngTail.push_back({at, off, val, pc, wr});
        return;
    }
    // The ring is full: one event leaves for each that arrives, and the count
    // of them is part of the report.
    ++gEngDropped;
    gEngTail[gEngTailAt] = {at, off, val, pc, wr};
    gEngTailAt = (gEngTailAt + 1) % half;
}

const std::vector<R128EngEv>& r128EngineLog()
{
    gEngStitched = gEngHead;
    for (size_t k = 0; k < gEngTail.size(); ++k)
        gEngStitched.push_back(gEngTail[(gEngTailAt + k) % gEngTail.size()]);
    return gEngStitched;
}

// =========================== THE 2D ENGINE ===============================
//
// Offsets, field positions and semantics are ATI's own, from the `RAGE 128
// PRO Register Reference Guide` (RRG-G04500-C rev 1.01); QEMU's ati_2d.c is
// the behavioral oracle for how the pieces combine into a blit. Names,
// offsets, bit meanings and observed behavior are facts; the code below is
// ours.
//
// ⚠ dingusppc's atirage.md is the MACH64 Rage Pro — a different register map
// entirely (SRC_OFF_PITCH 0x180). It must not be mixed in here.
static constexpr u32 kDstOffset = 0x1404, kDstPitch = 0x1408;
static constexpr u32 kDstWidth = 0x140C, kDstHeight = 0x1410;
static constexpr u32 kSrcX = 0x1414, kSrcY = 0x1418;
static constexpr u32 kDstX = 0x141C, kDstY = 0x1420;
static constexpr u32 kSrcPitchOffset = 0x1428, kDstPitchOffset = 0x142C;
static constexpr u32 kSrcYX = 0x1434, kDstYX = 0x1438;
static constexpr u32 kDstHeightWidth = 0x143C;
static constexpr u32 kDpGuiMasterCntl = 0x146C;
static constexpr u32 kDpBrushFrgdClr = 0x147C;
static constexpr u32 kDpBrushBkgdClr = 0x1478, kBrushYX = 0x1474;
static constexpr u32 kDpSrcFrgdClr = 0x15D8, kDpSrcBkgdClr = 0x15DC;
static constexpr u32 kBrushData0 = 0x1480, kBrushData1 = 0x1484;
static constexpr u32 kDstWidthX = 0x1588;
static constexpr u32 kSrcXY = 0x1590, kDstXY = 0x1594;
static constexpr u32 kDstWidthHeight = 0x1598, kDstWidthXIncy = 0x159C;
static constexpr u32 kDstHeightY = 0x15A0;
static constexpr u32 kSrcOffset = 0x15AC, kSrcPitch = 0x15B0;
static constexpr u32 kDstWidthBw = 0x15B4;
static constexpr u32 kClrCmpCntl = 0x15C0, kClrCmpMsk = 0x15CC;
static constexpr u32 kScLeft = 0x1640, kScRight = 0x1644;
static constexpr u32 kScTop = 0x1648, kScBottom = 0x164C;
static constexpr u32 kSrcScRight = 0x1654, kSrcScBottom = 0x165C;
static constexpr u32 kAuxScCntl = 0x1660;
static constexpr u32 kDpCntl = 0x16C0, kDpDatatype = 0x16C4;
static constexpr u32 kDpMix = 0x16C8, kDpWriteMsk = 0x16CC;
static constexpr u32 kDefaultOffset = 0x16E0, kDefaultPitch = 0x16E4;
static constexpr u32 kDefaultScBr = 0x16E8;
static constexpr u32 kScTopLeft = 0x16EC, kScBottomRight = 0x16F0;
static constexpr u32 kSrcScBottomRight = 0x16F4;
static constexpr u32 kWaitUntil = 0x1720;
static constexpr u32 kGuiStat = 0x1740;
static constexpr u32 kPcGuiCtlstat = 0x1748;
// PM4_STAT sits in the CCE block, but it carries the SAME status flip-flops as
// GUI_STAT — free-FIFO count in 11:0, PM4_BUSY at 16, GUI_ACTIVE at 31 — read
// through the command engine's own address. Answering zero says "no FIFO
// entries free", which is the opposite of idle.
static constexpr u32 kPm4Stat = 0x07B8;
static constexpr u32 kScTopLeftC = 0x1C88, kScBottomRightC = 0x1C8C;
static constexpr u32 kDstPitchOffsetC = 0x1C80, kDpGuiMasterCntlC = 0x1C84;
// GUI_STAT: GUI_FIFOCNT is bits 11:0 and its RESET DEFAULT IS 0x40 — the
// number of free CMDFIFO entries. Every busy bit (16..29) and GUI_ACTIVE
// (31, the OR of them) reads clear here, which is the truth for an engine
// that finishes each operation inside the store that triggered it.
static constexpr u32 kGuiStatIdle = 0x00000040u;
// The engine's virtual address space is 64 MB: the low 32 MB is the frame
// buffer and the high 32 MB is AGP_BASE + offset(24:0) — system memory this
// cell cannot reach. DST_OFFSET/SRC_OFFSET are 26-bit with the low 4 bits
// hardwired to zero.
static constexpr u32 kEngAddrMask = 0x03FFFFF0u;
static constexpr u32 kVramSpan = 32u << 20;

static R128EngStats gEng;
static std::map<u32, u64> gRopUnimpl;
// ON by default, and that default is load-bearing: the capi never touches
// these knobs, so the shipping app takes whatever is here. `--no-ati-2d`
// restores the pre-engine card bit for bit, which is what proves a boot that
// never uses the engine is unchanged by its arrival.
static bool gEng2dOn = true;
void r128SetEngine2d(bool on) { gEng2dOn = on; }

// The bus-master block. 0x0A10 carries a BUSY bit at 27 that the accelerator
// polls; 0x0A50 takes the physical address of the transfer. Neither is named
// in either ATI register guide — the guides document exactly two registers of
// this block, BM_QUEUE_FREE_STATUS 0x0A14 and BM_ABORT 0x0A88 — so these are
// recorded as OBSERVED BEHAVIOR and nothing more.
static constexpr u32 kBmChunk0 = 0x0A18;
static constexpr u32 kBmStatus = 0x0A10;
static constexpr u32 kBmAddr = 0x0A50;
static constexpr u32 kGuiScratch0 = 0x15E0, kGuiScratch1 = 0x15E4;

void r128BusMasterFetch(R128Cell& c, const u8* ram, u32 ramSize,
                        SnoopSink* snoop)
{
    if (!gEng2dOn || !ram)
        return;
    // The address is 16-byte aligned by the driver (`clrrwi r3,r31,4`) and is
    // PHYSICAL: the accelerator has just set BM_PTR_FORCE_TO_PCI in
    // BM_CHUNK_0_VAL, which is what "address this as plain PCI rather than
    // through the AGP aperture" means.
    const u32 pa = c.peek(kBmAddr) & ~0xFu;
    if (!pa || static_cast<u64>(pa) + 8u > ramSize) {
        ++gEng.bmBadAddr;
        return;
    }
    // ⚠ SNOOP THE READ. The guest wrote those bytes with ordinary stores, so
    // they may still be sitting in a dirty cache line; a master that reads
    // around the cache gets stale memory and the comparison fails for a
    // reason that looks nothing like a coherency bug. Session 30 paid for the
    // write direction of exactly this.
    if (snoop)
        snoop->snoopRead(pa, 8);
    if (r128RecOn())
        r128RecTouch(ram, ramSize, pa, 8, snoop);
    // Little-endian throughout: the guest builds this with stwbrx and reads
    // the registers back with lwbrx.
    auto le32 = [&](u32 at) {
        return u32(ram[at]) | (u32(ram[at + 1]) << 8) |
               (u32(ram[at + 2]) << 16) | (u32(ram[at + 3]) << 24);
    };
    // ⭐ 0x0A50 POINTS AT A DESCRIPTOR, NOT AT THE DATA.
    //
    // Measured: the first fetch this model ever performed read
    // {0x000015E0, 0x005EC000} — a TARGET REGISTER OFFSET (0x15E0 is
    // GUI_SCRATCH_REG0 exactly) followed by a SOURCE ADDRESS, and that
    // address is precisely `buffer + 4096`, which is where the accelerator's
    // own code plants its timestamp (`addi r4,r29,4096`). Treating the
    // descriptor as the data is what surfaced both values, so the version of
    // this function that was wrong is what named the format.
    const u32 dstReg = le32(pa) & 0x3FFCu;
    const u32 src = le32(pa + 4) & ~3u;
    if (!dstReg || !src || static_cast<u64>(src) + 8u > ramSize) {
        ++gEng.bmBadAddr;
        return;
    }
    if (snoop)
        snoop->snoopRead(src, 8);
    if (r128RecOn())
        r128RecTouch(ram, ramSize, src, 8, snoop);
    // ⚠ TWO WORDS, because two is all that has ever been observed: the
    // self-test compares exactly GUI_SCRATCH_REG0 and REG1 against the pair
    // it planted. Nothing establishes where a longer transfer would take its
    // length from, so a longer one is not invented here.
    c.setReg(dstReg, le32(src));
    c.setReg(dstReg + 4u, le32(src + 4u));
    (void)kGuiScratch0;
    (void)kGuiScratch1;
    // The transfer is complete before the store that started it returns, so
    // BM busy (0x0A10 bit 27) is already clear — which is what the register
    // reads back as, and the accelerator's wait loop exits on the first pass.
    c.setReg(kBmStatus, 0);
    ++gEng.bmFetches;
    (void)kBmChunk0;
}

const R128EngStats& r128EngStats() { return gEng; }
// The 3D unit (r128_3d.cpp) counts into the same table, so the report stays
// one table. Not in the header: the mutable view is the engine's own.
R128EngStats& r128EngStatsMut() { return gEng; }
const std::map<u32, u64>& r128RopUnimplemented() { return gRopUnimpl; }

// ── The CCE's PIO packet path ──────────────────────────────────────────────
//
// Measured 2026-08-01, the first boot with a working .AGP driver: the ARM
// sets PM4_BUFFER_CNTL mode 7 (64 CCE packet PIO / 64 vertex BM / 64
// indirect BM), uploads the microcode, and submits through the FIFO. The
// first submission ever seen was six DWORDs:
//
//   000101ce 00101000 00000018   Type-0, 2 regs @0x0738: PM4_IW_INDOFF =
//                                0x101000 (an AGP address), PM4_IW_INDSIZE =
//                                0x18 DWORDs -> dispatch an indirect buffer
//   00010578 d9c4e0d7 00000001   Type-0, 2 regs @0x15E0: GUI_SCRATCH_REG0/1
//                                <- a 64-bit UpTime fence
//
// and the ARM then polls GUI_SCRATCH_REG0/1 (43.7 M reads each) for the
// fence to land — the software-fence idiom, working because a FIFO'd write
// executes in command order. Executing the packets IS the fence mechanism;
// nothing here free-runs.
//
// Packet formats are SDK-G04000 App F (r128sdk.txt 12575-12800): Type-0
// header = TYPE 31:30 | COUNT 29:16 (N-1) | ONE_REG_WR 15 | BASE_INDEX 10:0
// in DWORDs; Type-1 = two indexed registers; Type-2 = filler; Type-3 =
// TYPE | COUNT (N-1) | IT_OPCODE 15:8 with an N-DWord body.
//
// ⚠ The staging FIFO is a file-static: a snapshot taken between the words
// of one packet loses the partial packet on resume. Snapshots are minted at
// quiescent points; the word counter makes a truncation visible.
static std::vector<u32> gCceFifo;
static size_t gCceHead = 0;
static int gCceDepth = 0;
static std::map<u32, u64> gCceP3;
const std::map<u32, u64>& r128CceP3Skipped() { return gCceP3; }

static constexpr u32 kIwIndOff = 0x0738, kIwIndSize = 0x073C;

static u32 cceBswap(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}

// Who is running the packet loop, which decides what it may do.
enum class CceMode {
    Sync,     // the engine's thread is off: execute everything, here
    EmuFront, // the machine's thread, engine thread on: execute what is cheap
              // or needs system memory, hand a long stream over
    EmuDrain, // the machine's thread, and the guest is about to look:
              // execute everything, here
    Worker,   // the engine's thread: no system memory
};
// How a run of the packet loop ended.
enum class CceRun {
    Empty,    // nothing complete is left staged
    HandOff,  // EmuFront: the rest is for the engine's thread
    NeedsEmu, // Worker: the packet at the head needs system memory
    Yield,    // Worker: the machine's thread asked for the engine
};
static CceRun cceRun(R128Cell& c, const CceMem& m, CceMode mode);

namespace {
void asyncSubmit(R128Cell& c, const CceMem& m);
void asyncRelease(R128Cell& c, const CceMem& m);
}

// An INDSIZE write seen while a packet is still executing must not fetch on
// the spot: the fetched body has to land BETWEEN this packet and the next
// one, and a parse re-entered mid-packet would re-read the in-flight header
// and desynchronize the whole stream — the unit test caught exactly that as
// a garbage register spray. So execution only RECORDS the dispatch, and the
// parse loop performs it at the packet boundary.
static std::vector<u32> gCcePendingInd;

// A rectangle whose pixels are arriving through the HOST_DATA registers:
// see "Host data through the registers", by engHostBegin.
namespace {
struct HostOp {
    bool active = false, byteAligned = false, lsbFirst = false;
    int x = 0, y = 0;
    u32 w = 0, h = 0, srcType = 3, rop = 0xCC, bypp = 4;
    u64 need = 0; // DWORDs the rectangle takes
    std::vector<u32> words;
} gHostOp;
} // namespace

// ── The command engine on its own thread (see opm/r128.hpp) ───────────────
//
// Three locks, always taken in this order when more than one is held:
//
//   engMu   whoever is executing packets holds it. The engine's thread
//           holds it while it runs and gives it up between two packets when
//           the machine's thread asks (drainReq); the machine's thread holds
//           it for as long as it executes. It is what "the engine is mine"
//           means while work is queued.
//   regMu   the cell's register store, its logs and its censuses, for one
//           access (the machine's thread) or one packet's register work (the
//           engine's thread, which lets it go while it rasterizes).
//   inboxMu the FIFO words that arrived behind a running engine — and the
//           busy flag's fall, so "queue it" and "nothing is running, execute
//           it yourself" cannot both be decided about the same word.
//
// gR128AsyncBusy is only ever RAISED by the machine's thread (a hand-off) and
// only falls when a run finds nothing left. So when the machine's thread
// reads it false, nothing else is touching the card and nothing will until
// that same thread submits again: the idle case takes no lock at all.
std::atomic<bool> gR128AsyncBusy{false};
std::atomic<bool> gR128AsyncNeedsEmu{false};
namespace {
// A stream shorter than this is executed where it was submitted: waking a
// thread costs more than a few small packets do.
constexpr size_t kHandOffWords = 256;
void asyncWorkerMain();
struct Async {
    std::mutex engMu;
    std::condition_variable engCv;
    std::mutex regMu;
    std::mutex inboxMu;
    std::vector<u32> inbox;
    std::atomic<bool> on{false};
    std::atomic<bool> strict{false};
    std::atomic<bool> drainReq{false};
    std::atomic<bool> hold{false};
    R128Cell* cell = nullptr; // under engMu
    CceMem mem;               // under engMu: system memory as the engine's
                              // thread may read it (asyncRelease)
    bool quit = false;        // under engMu
    bool started = false;     // the machine's thread only
    std::thread th;
    R128AsyncStats st;
    // When the busy flag last rose (under engMu, as its rise and fall are).
    std::chrono::steady_clock::time_point busySince{};
    bool busyTimed = false;
    u32 cause = kR128DrainOther; // the machine's thread only
    std::map<u32, R128DrainCause> causes;
    ~Async()
    {
        if (!started)
            return;
        {
            std::lock_guard<std::mutex> lk(engMu);
            quit = true;
        }
        engCv.notify_all();
        th.join();
        started = false; // a card destroyed after this has nothing to abandon
    }
};
// ⚠ Defined after every engine static above it on purpose: statics are torn
// down in reverse, so the thread is joined before the staging vectors go.
Async gAsync;
thread_local int tRegDepth = 0;
} // namespace

void r128AsyncEnable(bool on, bool strict)
{
    gAsync.on.store(on);
    gAsync.strict.store(on && strict);
}
bool r128AsyncEnabled() { return gAsync.on.load(); }
bool r128AsyncStrict() { return gAsync.strict.load(std::memory_order_relaxed); }
const R128AsyncStats& r128AsyncStats() { return gAsync.st; }
void r128AsyncHold(bool hold)
{
    {
        std::lock_guard<std::mutex> lk(gAsync.engMu);
        gAsync.hold.store(hold);
    }
    gAsync.engCv.notify_one();
}
void r128AsyncDrainCause(u32 cause) { gAsync.cause = cause; }
u32 r128AsyncFifoQueued()
{
    std::lock_guard<std::mutex> ib(gAsync.inboxMu);
    return static_cast<u32>(gAsync.inbox.size());
}
const std::map<u32, R128DrainCause>& r128AsyncDrainCauses()
{
    return gAsync.causes;
}
void r128AsyncRegRelease() { gAsync.regMu.unlock(); }
void r128AsyncRegTake() { gAsync.regMu.lock(); }

R128RegLock::R128RegLock()
{
    if (!r128AsyncBusy())
        return;
    if (tRegDepth++ == 0)
        gAsync.regMu.lock();
    held_ = true;
}
R128RegLock::~R128RegLock()
{
    if (held_ && --tRegDepth == 0)
        gAsync.regMu.unlock();
}

// A machine is being created: no packet stream exists yet, whatever a
// previous machine in this process left staged. See the header note — a
// stale half-packet here desynchronizes every subsequent submission, and
// it presented as "the desktop drew for a while after a Stop/Start and
// then froze mid-redraw".
void r128CceReset()
{
    r128AsyncAbandon(nullptr); // the engine's thread must not be mid-stream
    gCceFifo.clear();
    gCceHead = 0;
    gCceDepth = 0;
    gCcePendingInd.clear();
    gHostOp = HostOp{};
    r128Cce3dReset();
}

// The engine's state that lives outside the cell, for a frame recording
// (opm/r128rec.hpp): the words of a packet still arriving, the indirect
// dispatches waiting for a packet boundary, and the 3D unit's tables. Machine
// snapshots do not carry these (see the note on the staging FIFO); a
// recording starts wherever the guest happens to be, so it must.
void r128EngineStateSave(SnapWriter& w)
{
    w.u64v(gCceFifo.size() - gCceHead);
    for (size_t k = gCceHead; k < gCceFifo.size(); ++k)
        w.u32v(gCceFifo[k]);
    w.u64v(gCcePendingInd.size());
    for (u32 v : gCcePendingInd)
        w.u32v(v);
    r128Cce3dStateSave(w);
    // A rectangle whose pixels are still arriving through HOST_DATA.
    w.b(gHostOp.active);
    w.b(gHostOp.byteAligned);
    w.b(gHostOp.lsbFirst);
    w.i32v(gHostOp.x);
    w.i32v(gHostOp.y);
    w.u32v(gHostOp.w);
    w.u32v(gHostOp.h);
    w.u32v(gHostOp.srcType);
    w.u32v(gHostOp.rop);
    w.u32v(gHostOp.bypp);
    w.u64v(gHostOp.need);
    w.u64v(gHostOp.words.size());
    for (u32 v : gHostOp.words)
        w.u32v(v);
}
void r128EngineStateLoad(SnapReader& r)
{
    gCceFifo.clear();
    gCceHead = 0;
    gCcePendingInd.clear();
    const u64 staged = r.u64v();
    for (u64 k = 0; k < staged && r.ok; ++k)
        gCceFifo.push_back(r.u32v());
    const u64 pending = r.u64v();
    for (u64 k = 0; k < pending && r.ok; ++k)
        gCcePendingInd.push_back(r.u32v());
    r128Cce3dStateLoad(r);
    // A recording made before host data went through the registers ends
    // here.
    gHostOp = HostOp{};
    if (r.ok && r.p < r.end) {
        gHostOp.active = r.b();
        gHostOp.byteAligned = r.b();
        gHostOp.lsbFirst = r.b();
        gHostOp.x = r.i32v();
        gHostOp.y = r.i32v();
        gHostOp.w = r.u32v();
        gHostOp.h = r.u32v();
        gHostOp.srcType = r.u32v();
        gHostOp.rop = r.u32v();
        gHostOp.bypp = r.u32v();
        gHostOp.need = r.u64v();
        const u64 have = r.u64v();
        for (u64 k = 0; k < have && r.ok; ++k)
            gHostOp.words.push_back(r.u32v());
    }
}

static void cceRegWrite(R128Cell& c, u32 off, u32 native)
{
    // Through the front door: the same path a guest MMIO store takes, so a
    // packet's register write triggers the same engine side effects, lands
    // in the same logs, and needs no second dispatch table. write() swaps a
    // 4-byte store, so hand it the pre-swapped image.
    c.write(off & 0x3FFCu, cceBswap(native), 4);
    if ((off & 0x3FFCu) == kIwIndSize)
        gCcePendingInd.push_back(native);
}

// One GART-translated LE32 read out of AGP space. The Uni-N GART table is in
// system RAM where the .AGP driver built it (base = 106b:0020 config +0x8C);
// entries are little-endian, bit 0 valid, 31:12 the physical page — the
// layout Linux's uninorth-agp documents.
//
// ⚠⚠ THE BASE REGISTER'S LOW BITS ARE FLAGS, NOT ADDRESS — the table is
// PAGE-ALIGNED, always. Observed values: 0x03105D00 (64 MB machine) and
// 0x00B77008 (1.5 GB). An earlier version probed the RAW base first and
// let a valid bit decide; at 1.5 GB the off-by-8 read landed on the
// NEIGHBORING entries, whose valid bits are legitimately set, so every
// walk "succeeded" onto the wrong physical pages and fed 0xDDDDDD00 fill
// garbage into the parser as packets — one absurd Type-3 header then
// swallowed 3,895 real words and the desktop froze mid-redraw with zero
// recorded misses. A self-deciding probe whose discriminator can be
// satisfied by garbage is not an instrument; the diag capture that showed
// STAGED words behind a garbage header is what named this.
static bool cceGartRead(const u8* ram, u32 ramSize, u32 gartBase,
                        u32 aperBase, u32 agp, u32& out, SnoopSink* snoop)
{
    auto le32 = [&](u32 at) {
        return u32(ram[at]) | (u32(ram[at + 1]) << 8) |
               (u32(ram[at + 2]) << 16) | (u32(ram[at + 3]) << 24);
    };
    static int walkLog = 0;
    // The GART indexes APERTURE-RELATIVE pages: subtract the AGP aperture
    // base (bridge config +0x90) first. On a 64 MB machine the driver
    // programmed base 0 and this line was invisible; at 1.5 GB the
    // aperture sits above RAM, and an unsubtracted address indexed pages
    // far past the table — the fetched garbage then parsed as packets,
    // one absurd header swallowed every later real submission, and the
    // desktop froze mid-redraw with the fence never landing.
    const u32 aper = aperBase & 0xFFFFF000u;
    if (aper) {
        if (agp < aper) {
            ++gEng.cceGartMiss;
            if (walkLog < 8) {
                ++walkLog;
                printf("-- cce gart MISS: agp=%08x below aperture %08x\n",
                       agp, aper);
            }
            return false;
        }
        agp -= aper;
    }
    const u32 idx = agp >> 12;
    const u32 base = gartBase & ~0xFFFu;
    const u64 ea = static_cast<u64>(base) + static_cast<u64>(idx) * 4u;
    if (base && ea + 4u <= ramSize) {
        if (snoop)
            snoop->snoopRead(static_cast<u32>(ea), 4);
        if (r128RecOn())
            r128RecTouch(ram, ramSize, static_cast<u32>(ea), 4, snoop);
        const u32 entry = le32(static_cast<u32>(ea));
        const u32 pa = (entry & 0xFFFFF000u) | (agp & 0xFFFu);
        if ((entry & 1u) && static_cast<u64>(pa) + 4u <= ramSize) {
            if (walkLog < 8) {
                ++walkLog;
                printf("-- cce gart: agp=%08x idx=%x entry@%08llx=%08x -> "
                       "pa=%08x\n",
                       agp, idx, static_cast<unsigned long long>(ea), entry,
                       pa);
            }
            if (snoop)
                snoop->snoopRead(pa, 4);
            if (r128RecOn())
                r128RecTouch(ram, ramSize, pa, 4, snoop);
            out = le32(pa);
            return true;
        }
    }
    ++gEng.cceGartMiss;
    if (walkLog < 8) {
        ++walkLog;
        printf("-- cce gart MISS: agp=%08x idx=%x base=%08x\n", agp, idx,
               gartBase);
    }
    return false;
}

// The exported walk, for the 3D unit's vertex and texture fetches — the same
// translation, the same miss counter, the same first-eight log.
bool r128CceGartRead(const CceMem& m, u32 agp, u32& out)
{
    if (!m.ram) { // the engine's thread: counted as a miss, never a read
        ++gEng.cceGartMiss;
        return false;
    }
    return cceGartRead(m.ram, m.ramSize, m.gartBase, m.aperBase, agp, out,
                       m.snoop);
}

// ── The 2D operation packets (SDK App F.7) ────────────────────────────────
//
// Every 2D packet's body opens with the same SETTINGS structure:
// [GUI_CONTROL] — bit-compatible with DP_GUI_MASTER_CNTL, and its low flag
// bits complement the register's default-load semantics exactly: packet bit
// set = "the datum follows in SETUP_BODY" = register bit set = "leave the
// current value alone", so GUI_CONTROL is written to the GMC VERBATIM after
// the supplied data land in their registers — then the ordered optional
// SETUP_BODY: [SRC_PITCH_OFFSET b0] [DST_PITCH_OFFSET b1]
// [SRC_SC_BOT_RITE b2] [SC_TOP_LEFT + SC_BOT_RITE b3]
// {BRUSH_PACKET per bits 7:4} [BRUSH_Y_X b31].
//
// Consumes from `body` (bounded by the Type-3 COUNT, so a malformed body
// can be declined without desynchronizing the stream) and returns the index
// past SETTINGS, or n+1 to signal "decline".
static u32 cceSettings(R128Cell& c, const u32* body, u32 n)
{
    if (!n)
        return 1;
    u32 i = 0;
    const u32 gc = body[i++];
    if (gc & 1u) { // SRC_PITCH_OFFSET
        if (i >= n) return n + 1;
        cceRegWrite(c, 0x1428u, body[i++]);
    }
    if (gc & 2u) { // DST_PITCH_OFFSET
        if (i >= n) return n + 1;
        cceRegWrite(c, 0x142Cu, body[i++]);
    }
    if (gc & 4u) { // SRC_SC_BOT_RITE
        if (i >= n) return n + 1;
        cceRegWrite(c, 0x16F4u, body[i++]);
    }
    if (gc & 8u) { // SC_TOP_LEFT + SC_BOT_RITE
        if (i + 1u >= n) return n + 1;
        cceRegWrite(c, 0x16ECu, body[i++]);
        cceRegWrite(c, 0x16F0u, body[i++]);
    }
    // The brush packet, sized by BRUSH_TYPE (Table F-2). Only the solid
    // brush draws today; pattern data still lands in the brush registers so
    // nothing is silently dropped, and an op that needs the pattern is the
    // blitter's brushUnimpl decline, visible in the report.
    const u32 brush = (gc >> 4) & 0xFu;
    static const int kBrushDwords[16] = {4,  3,  3,  2, 3, 2, 3, 2,
                                         34, 33, 33, -1, -1, 1, -1, 0};
    int bn = kBrushDwords[brush];
    if (bn < 0) {
        // 8x8 / 8x1 / 1x8 COLOR patterns: 16*N / 2*N / 2*N DWORDs where N
        // is bytes-per-pixel from DST_TYPE (Table F-3).
        static const int kBpp[16] = {0, 0, 1, 2, 2, 3, 4, 1,
                                     0, 0, 0, 0, 0, 0, 0, 0};
        const int bpp = kBpp[(gc >> 8) & 0xFu];
        if (!bpp)
            return n + 1;
        bn = (brush == 11u ? 16 : 2) * bpp;
    }
    if (i + static_cast<u32>(bn) > n)
        return n + 1;
    switch (brush) {
    case 0: // [BKGD][FRGD][BMP1][BMP2]
        cceRegWrite(c, 0x1478u, body[i]);
        cceRegWrite(c, 0x147Cu, body[i + 1u]);
        cceRegWrite(c, kBrushData0, body[i + 2u]);
        cceRegWrite(c, kBrushData1, body[i + 3u]);
        break;
    case 1: // [FRGD][BMP1][BMP2]: the 8x8 pattern over what is there
        cceRegWrite(c, 0x147Cu, body[i]);
        cceRegWrite(c, kBrushData0, body[i + 1u]);
        cceRegWrite(c, kBrushData1, body[i + 2u]);
        break;
    case 3:
    case 5:
    case 7:
    case 9:
    case 10:
    case 13: // solid: [FRGD] alone
        cceRegWrite(c, 0x147Cu, body[i]);
        break;
    case 2:
    case 4:
    case 6:
    case 8:
        cceRegWrite(c, 0x1478u, body[i]);
        if (bn > 1)
            cceRegWrite(c, 0x147Cu, body[i + 1u]);
        break;
    default:
        break;
    }
    i += static_cast<u32>(bn);
    if (gc >> 31) { // BRUSH_Y_X follows
        if (i >= n) return n + 1;
        cceRegWrite(c, 0x1474u, body[i++]);
    }
    // Now the control word itself, verbatim — see the header comment.
    cceRegWrite(c, 0x146Cu, gc);
    return i;
}

// PAINT (0x91): rectangles [TOP|LEFT][BOTM|RITE] after SETTINGS. The engine
// initiates on DST_HEIGHT_WIDTH, exactly as the register path does.
// ⚠ Bottom-right is taken as EXCLUSIVE (width = RITE-LEFT), the GDI
// convention the packet's own WIN31_ROP field points at; if the desktop
// ever draws one pixel short everywhere, this is the bit to revisit.
static bool cceOpPaint(R128Cell& c, const u32* body, u32 n)
{
    u32 i = cceSettings(c, body, n);
    if (i > n)
        return false;
    while (i + 2u <= n) {
        const u32 tl = body[i], br = body[i + 1u];
        i += 2u;
        const u32 left = tl & 0xFFFFu, top = tl >> 16;
        const u32 rite = br & 0xFFFFu, botm = br >> 16;
        const u32 w = (rite - left) & 0x3FFFu;
        const u32 h = (botm - top) & 0x3FFFu;
        if (!w || !h)
            continue;
        cceRegWrite(c, 0x1438u, tl);              // DST_Y_X
        cceRegWrite(c, 0x143Cu, (h << 16) | w);   // DST_HEIGHT_WIDTH: draw
    }
    return true;
}

// PAINT_MULTI (0x9A): same fill as PAINT, different parameter shape — the
// SDK's own words. Pairs of [DST_X|DST_Y] (⚠ x HIGH, y LOW — inverted from
// both PAINT and the DST_Y_X register) and [DST_W|DST_H] (w HIGH, h LOW).
// Both are half-swaps of the register layouts, and width/height are given
// directly, so no corner arithmetic and no inclusivity question.
static bool cceOpPaintMulti(R128Cell& c, const u32* body, u32 n)
{
    u32 i = cceSettings(c, body, n);
    if (i > n)
        return false;
    while (i + 2u <= n) {
        const u32 xy = body[i], wh = body[i + 1u];
        i += 2u;
        const u32 w = wh >> 16, h = wh & 0xFFFFu;
        if (!w || !h)
            continue;
        cceRegWrite(c, 0x1438u, (xy >> 16) | (xy << 16)); // DST_Y_X
        cceRegWrite(c, 0x143Cu, (wh >> 16) | (wh << 16)); // H_W: draw
    }
    return true;
}

// ---- color compare (CLR_CMP_CNTL, 0x15C0) ----------------------------------
//
// The 2D engine's transparency: a pixel is drawn or left alone by comparing
// the SOURCE pixel, the DESTINATION pixel or both against a key, through a
// mask. A sprite with a black surround is pushed with "source, function 5,
// key black" and only its own pixels land.
//
// ⚠ THE TABLE READS TWO WAYS AND MEANS ONE. Register guide 3-178 gives each
// function an expression and an action — "5 = SRC_CLR = CLR_CMP_CLR_SRC
// (CMP_NEQ_COLOR, draw when neq)" — and the two look like opposites. They are
// the same statement: the expression is what SUPPRESSES the pixel, exactly as
// "1 = True" is "never draw", and the action is its complement. This engine
// used to count the setting and draw every pixel, on the grounds that the
// table contradicted itself; a driver settles it. For a sprite on a black
// surround the Mac driver programs 0x01000005 with a key of zero, which is
// transparency only under "draw when not equal".
//
//   0 always draw   1 never draw   4 draw when equal   5 draw when not equal
//
// Function 7 (flip through SRC_FRGD_CLR when equal), the reserved values and
// the HILITE source are not applied; they are counted in colorCompare, which
// is now "a compare this engine did not apply" and nothing else.
static u32 rop3(u32 rop, u32 P, u32 S, u32 D);
static bool ropUsesSrc(u32 rop);
static bool ropUsesPat(u32 rop);
static bool ropUsesDst(u32 rop);

namespace {
struct ClrCmp {
    u32 fnSrc = 0, fnDst = 0; // only 1, 4 and 5 ever suppress a pixel
    u32 keySrc = 0, keyDst = 0, mask = 0;
    bool any = false;     // some pixel could be suppressed
    bool unknown = false; // …or the setting is one this engine does not apply

    static bool one(u32 fn, u32 pix, u32 key, u32 mask)
    {
        switch (fn) {
        case 1u: return false;
        case 4u: return ((pix ^ key) & mask) == 0u;
        case 5u: return ((pix ^ key) & mask) != 0u;
        default: return true;
        }
    }
    bool draws(u32 S, u32 D) const
    {
        return one(fnSrc, S, keySrc, mask) && one(fnDst, D, keyDst, mask);
    }
};

// `bypp` is the pixel's width: the key and the mask are compared over the
// pixel's own bits and no further.
ClrCmp clrCmpDecode(u32 cntl, u32 keySrc, u32 keyDst, u32 mask, u32 bypp)
{
    ClrCmp k;
    const u32 which = (cntl >> 24) & 3u; // 0 destination, 1 source, 2 both
    const u32 fs = cntl & 7u, fd = (cntl >> 8) & 7u;
    const u32 bits = bypp >= 4u ? 0xFFFFFFFFu : (1u << (bypp * 8u)) - 1u;
    if (which == 1u || which == 2u)
        k.fnSrc = fs;
    if (which == 0u || which == 2u)
        k.fnDst = fd;
    auto applied = [](u32 fn) { return fn == 1u || fn == 4u || fn == 5u; };
    k.unknown = which == 3u || (k.fnSrc && !applied(k.fnSrc)) ||
                (k.fnDst && !applied(k.fnDst));
    k.any = applied(k.fnSrc) || applied(k.fnDst);
    k.keySrc = keySrc & bits;
    k.keyDst = keyDst & bits;
    k.mask = mask & bits;
    return k;
}
} // namespace

// HOSTDATA_BLT (0x94): bit-packed mono bitmaps expanded through
// foreground/background — the large-glyph text path (SDK F.11, mirroring
// the Windows LARGEBITGLYPH). The engine's register-path mono expansion is
// not built (monoSrc is a decline counter), so the expansion happens here,
// honoring the engine's CURRENT destination state: pitch/offset from the
// DST registers, the scissor, and the destination type from GUI_CONTROL.
// FRGD/BKGD arrive as RGBQUAD and are converted to the destination format.
static u32 ccePixelBytes(u32 dstType);
static bool cceOpHostBlt(R128Cell& c, const u32* body, u32 n)
{
    const u32 i0 = cceSettings(c, body, n);
    if (i0 > n || i0 + 2u > n)
        return false;
    const u32 gc = body[0];
    const u32 dstType = (gc >> 8) & 0xFu;
    const u32 srcType = (gc >> 12) & 3u;   // 0 mono opaque, 1 transparent,
                                           // 3 color ("same as DST")
    const bool lsbFirst = (gc >> 14) & 1u; // PIX_ORDER
    if (srcType == 2u) {
        ++gEng.hostData; // reserved encoding — decline loudly
        return false;
    }
    const u32 bpp = ccePixelBytes(dstType);
    if (!bpp) {
        ++gEng.badBpp;
        return false;
    }
    auto conv = [&](u32 rgbq) -> u32 {
        const u32 rr = (rgbq >> 16) & 0xFFu, gg = (rgbq >> 8) & 0xFFu,
                  bb = rgbq & 0xFFu;
        if (dstType == 4u) // RGB565
            return ((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3);
        if (dstType == 3u) // aRGB1555
            return ((rr >> 3) << 10) | ((gg >> 3) << 5) | (bb >> 3);
        return rgbq;
    };
    const u32 fg = conv(body[i0]), bk = conv(body[i0 + 1u]);
    const u32 dstOff = c.peek(0x1404u);
    const u32 pitchBytes = c.peek(0x1408u) * 8u * bpp;
    // Color pixels go through the raster op and the color compare, as any
    // other source does: a sprite arrives this way with its surround keyed
    // out. The 1-bit forms are expanded as before — what "the source color"
    // is for a compare against an expanded bit is not something the guide
    // says — and a compare left on across one is counted instead.
    const u32 rop = (gc >> 16) & 0xFFu;
    const ClrCmp key = clrCmpDecode(c.peek(0x15C0u), c.peek(0x15C4u),
                                    c.peek(0x15C8u), c.peek(0x15CCu), bpp);
    if (key.unknown || (key.any && srcType != 3u))
        ++gEng.colorCompare;
    const bool plainCopy = rop == 0xCCu && !key.any;
    const u32 brush = ropUsesPat(rop) ? c.peek(0x147Cu) : 0u;
    // The engine normalizes SC_TOP_LEFT/SC_BOTTOM_RIGHT into the split
    // canonical cells — read those, not the alias offsets.
    const int scL = static_cast<int>(c.peek(0x1640u) & 0x3FFFu),
              scR = static_cast<int>(c.peek(0x1644u) & 0x3FFFu),
              scT = static_cast<int>(c.peek(0x1648u) & 0x3FFFu),
              scB = static_cast<int>(c.peek(0x164Cu) & 0x3FFFu);
    u32 i = i0 + 2u;
    while (i + 3u <= n) {
        const u32 baseYX = body[i], hw = body[i + 1u], m = body[i + 2u];
        i += 3u;
        if (i + m > n)
            return false; // malformed: header count disagrees
        const int bx = static_cast<int>(baseYX & 0xFFFFu);
        const int by = static_cast<int>(baseYX >> 16);
        const u32 wPx = hw & 0xFFFFu, hPx = hw >> 16;
        const u32* raster = &body[i];
        i += m;
        if (!wPx || !hPx || !m)
            continue;
        if (srcType == 3u) {
            // COLOR host data: the raster is packed destination-format
            // pixels, rows continuous — an icon/pixmap push, copied
            // byte-for-byte. ⚠ THE COUNT IS NOT THE LAYOUT. For a width
            // whose row is not a whole number of DWORDs the Mac driver
            // counts every row rounded up (a 490x33 push at one byte a
            // pixel says 4059 = 33 x 123 DWORDs) and still sends the pixels
            // end to end, in the first 4043 of them; the rest is slack.
            // Starting each row on its own DWORD, as that count suggests,
            // shears the picture two pixels a row, and did.
            // ⚠ The stream's DWORDs are LITTLE-ENDIAN
            // (both the FIFO ingest and the GART fetch compose them LE),
            // so stream byte s is lane (s & 3), LSB FIRST. Extracting
            // MSB-first reverses every pixel: a white menu bar's
            // 00 FF FF FF lands as FF FF FF 00 and scans out YELLOW —
            // the packet path's copy of the fill engine's old olive-
            // desktop byte-order bug, caught by the user's eyes in the
            // first interactive session.
            const u64 availB = static_cast<u64>(m) * 4u;
            // The two counts seen: the raster's bytes rounded up to a DWORD,
            // and each row rounded up. Any other is a packing this does not
            // know; it is counted, so the report says so.
            const u64 rowB = static_cast<u64>(wPx) * bpp;
            if (m != (rowB * hPx + 3u) / 4u && m != (rowB + 3u) / 4u * hPx &&
                ++gEng.hostShape <= 2)
                printf("-- cce host data %ux%u, %u bytes a pixel, in %u "
                       "words: a count this engine has no layout for\n",
                       wPx, hPx, bpp, m);
            u64 sb = 0;
            for (u32 yy = 0; yy < hPx; ++yy) {
                for (u32 xx = 0; xx < wPx; ++xx, sb += bpp) {
                    if (sb + bpp > availB)
                        break;
                    const int px = bx + static_cast<int>(xx);
                    const int py = by + static_cast<int>(yy);
                    if (px < scL || px > scR || py < scT || py > scB)
                        continue;
                    const u64 at = static_cast<u64>(dstOff) +
                                   static_cast<u64>(py) * pitchBytes +
                                   static_cast<u64>(px) * bpp;
                    if (at + bpp > c.vram.size())
                        continue;
                    if (!plainCopy) {
                        // The pixel as a number, most significant byte
                        // first, the way every reader of video memory sees
                        // it — the key is compared against that.
                        u32 S = 0, D = 0;
                        for (u32 k = 0; k < bpp; ++k) {
                            const u64 s = sb + k;
                            S = (S << 8) |
                                ((raster[s >> 2] >> (8u * (s & 3u))) & 0xFFu);
                            D = (D << 8) | c.vram[at + k];
                        }
                        if (!key.draws(S, D))
                            continue;
                        const u32 v = rop3(rop, brush, S, D);
                        if (r128PixelWatched(at, bpp))
                            r128PixelWatchNote(
                                "2d host data",
                                "color pixels pushed to (%d,%d %ux%u), rop "
                                "%02x, color compare %08x key %08x mask "
                                "%08x, wrote %x",
                                bx, by, wPx, hPx, rop, c.peek(0x15C0u),
                                c.peek(0x15C4u), c.peek(0x15CCu), v);
                        for (u32 k = 0; k < bpp; ++k)
                            c.vram[at + k] =
                                static_cast<u8>(v >> (8u * (bpp - 1u - k)));
                        continue;
                    }
                    if (r128PixelWatched(at, bpp))
                        r128PixelWatchNote(
                            "2d host data",
                            "color pixels pushed to (%d,%d %ux%u), "
                            "GUI_CONTROL %08x (rop %02x), no color compare",
                            bx, by, wPx, hPx, gc, (gc >> 16) & 0xFFu);
                    for (u32 k = 0; k < bpp; ++k) {
                        const u64 s = sb + k;
                        c.vram[at + k] = static_cast<u8>(
                            raster[s >> 2] >> (8u * (s & 3u)));
                    }
                    ++gEng.pixels;
                }
            }
            ++gEng.blits;
            continue;
        }
        // Mono: bits consumed row-major and CONTINUOUSLY across the DWORDs
        // (the linear HOSTDATA trajectory); PIX_ORDER picks the end each
        // DWORD is consumed from.
        u64 bit = 0;
        const u64 avail = static_cast<u64>(m) * 32u;
        for (u32 yy = 0; yy < hPx; ++yy) {
            for (u32 xx = 0; xx < wPx; ++xx, ++bit) {
                if (bit >= avail)
                    break;
                const u32 wIdx = static_cast<u32>(bit >> 5);
                const u32 bIdx = static_cast<u32>(bit & 31u);
                const u32 word = raster[wIdx];
                const u32 on = lsbFirst ? (word >> bIdx) & 1u
                                        : (word >> (31u - bIdx)) & 1u;
                if (!on && srcType == 1u)
                    continue; // transparent background
                const int px = bx + static_cast<int>(xx);
                const int py = by + static_cast<int>(yy);
                if (px < scL || px > scR || py < scT || py > scB)
                    continue;
                const u64 at = static_cast<u64>(dstOff) +
                               static_cast<u64>(py) * pitchBytes +
                               static_cast<u64>(px) * bpp;
                if (at + bpp > c.vram.size())
                    continue;
                const u32 v = on ? fg : bk;
                if (r128PixelWatched(at, bpp))
                    r128PixelWatchNote(
                        "2d host data",
                        "1-bit pixels expanded to (%d,%d %ux%u), source type "
                        "%u, wrote %x",
                        bx, by, wPx, hPx, srcType, v);
                for (u32 k = 0; k < bpp; ++k)
                    c.vram[at + k] =
                        static_cast<u8>(v >> (8u * (bpp - 1u - k)));
                ++gEng.pixels;
            }
        }
        ++gEng.blits;
    }
    return true;
}

// BITBLT_MULTI (0x9B), and BITBLT (0x92) which is its one-rect form: N
// screen-to-screen copies, per rect three words — [X|Y source] [X|Y dest]
// [W|H], X and W in the high halves. The spec's prose calls the second
// word the destination's "bottom-right corner"; the SDK's own worked
// example (§6.4, y2|x2<<16 "destination location") and a live window
// drag's stream (dst = src + exactly the commanded drag delta) both say
// TOP-LEFT, and the stream outranks the prose. These are the window-move
// and scroll ops.
static bool cceOpBitbltMulti(R128Cell& c, const u32* body, u32 n)
{
    u32 i = cceSettings(c, body, n);
    if (i > n)
        return false;
    while (i + 3u <= n) {
        const u32 s = body[i], d = body[i + 1u], wh = body[i + 2u];
        i += 3u;
        const u32 w = wh >> 16, h = wh & 0x3FFFu;
        if (!w || !h)
            continue;
        u32 sx = s >> 16, sy = s & 0xFFFFu; // packet: X high, Y low
        u32 dx = d >> 16, dy = d & 0xFFFFu;
        // ⚠ THE WALK ORDER IS THIS OP'S JOB, NOT THE DRIVER'S. The window
        // drag submits an OVERLAPPING screen-to-screen copy with the GMC's
        // direction bits at plain forward — and a forward walk into an
        // overlap reads rows it has already overwritten. Measured: the
        // drag's own blit, src=(117,152) dst=(186,241) 417x239, repeated
        // the window image every dy=89 rows down the screen. The engine's
        // register path honors DP_CNTL and treats a reversed origin as
        // the far corner, so give it exactly what a safe driver would:
        // direction away from the overlap, far-corner coordinates when
        // reversed.
        const bool l2r = dx <= sx, t2b = dy <= sy;
        if (!l2r) {
            sx += w - 1u;
            dx += w - 1u;
        }
        if (!t2b) {
            sy += h - 1u;
            dy += h - 1u;
        }
        cceRegWrite(c, kDpCntl, (c.peek(kDpCntl) & ~3u) |
                                    (l2r ? 1u : 0u) | (t2b ? 2u : 0u));
        cceRegWrite(c, 0x1434u, (sy << 16) | sx); // SRC_Y_X
        cceRegWrite(c, 0x1438u, (dy << 16) | dx); // DST_Y_X
        cceRegWrite(c, 0x143Cu, (h << 16) | w);   // H_W: draw
    }
    return true;
}

// Destination state for the ops that write VRAM directly, gathered from the
// engine's CANONICAL registers (the 0x16EC aliases are normalized away).
struct CceDst {
    u32 bpp = 0, dstOff = 0, pitchBytes = 0;
    int scL = 0, scR = 0, scT = 0, scB = 0;
};
// ── Scanlines (SDK F.15 POLYSCANLINES, F.20 PLY_NEXTSCAN) ────────────────
//
// A scanline is [HEIGHT | TOP] and then one [END | START] pair for each run
// along it: a band HEIGHT pixels thick, painted from START to END with the
// brush and ROP in force, which is a rectangle to this engine. The manual
// does not say whether END is the last pixel painted or the one after it;
// it is taken as the one after, the way PAINT's right and bottom edges are
// in the same family of packets. Returns the index past the scanline, or
// n + 1 if the body ends inside it.
static u32 cceScanline(R128Cell& c, const u32* body, u32 i, u32 n, u32 runs)
{
    if (i >= n)
        return n + 1u;
    const u32 ht = body[i++];
    const u32 top = ht & 0xFFFFu, height = (ht >> 16) & 0x3FFFu;
    for (u32 k = 0; k < runs; ++k) {
        if (i >= n)
            return n + 1u;
        const u32 se = body[i++];
        const u32 start = se & 0xFFFFu, end = se >> 16;
        const u32 w = (end - start) & 0x3FFFu;
        if (!w || !height || end < start)
            continue;
        cceRegWrite(c, 0x1438u, (top << 16) | start); // DST_Y_X
        cceRegWrite(c, 0x143Cu, (height << 16) | w);  // DST_HEIGHT_WIDTH: draw
    }
    return i;
}

// POLYLINE (0x95, SDK F.12): SETTINGS, then the corners [Y | X] of a line
// drawn through them in order. Each segment gets the terms the SDK's own
// line sample computes and goes through the engine's line registers. A
// segment draws from its first corner up to, not onto, its second: the next
// segment starts there, so a corner is drawn once, which is what an XOR
// needs. The manual does not say whether the polyline's last corner is
// drawn; it is left out, the convention of the Windows drivers this packet
// family was made for (its HOSTDATA_BLT is their glyph call).
static bool cceOpPolyline(R128Cell& c, const u32* body, u32 n)
{
    u32 i = cceSettings(c, body, n);
    if (i > n || i + 2u > n)
        return false;
    for (; i + 1u < n; ++i) {
        const int x0 = static_cast<int16_t>(body[i] & 0xFFFFu);
        const int y0 = static_cast<int16_t>(body[i] >> 16);
        const int x1 = static_cast<int16_t>(body[i + 1u] & 0xFFFFu);
        const int y1 = static_cast<int16_t>(body[i + 1u] >> 16);
        const int dx = x1 > x0 ? x1 - x0 : x0 - x1;
        const int dy = y1 > y0 ? y1 - y0 : y0 - y1;
        const int large = dx < dy ? dy : dx, small = dx < dy ? dx : dy;
        if (!large)
            continue;
        cceRegWrite(c, 0x16D0u,
                    (x0 < x1 ? 1u << 31 : 0u) | (y0 < y1 ? 1u << 15 : 0u) |
                        (dx < dy ? 4u : 0u)); // DP_CNTL_XDIR_YDIR_YMAJOR
        cceRegWrite(c, 0x1438u, (static_cast<u32>(y0 & 0x3FFF) << 16) |
                                    static_cast<u32>(x0 & 0x3FFF)); // DST_Y_X
        cceRegWrite(c, 0x1628u, static_cast<u32>(-large) & 0xFFFFFu);
        cceRegWrite(c, 0x162Cu, static_cast<u32>(2 * small) & 0xFFFFFu);
        cceRegWrite(c, 0x1630u, static_cast<u32>(-2 * large) & 0xFFFFFu);
        cceRegWrite(c, 0x1634u, static_cast<u32>(large)); // DST_BRES_LNTH: draw
    }
    return true;
}

// POLYSCANLINES (0x98): SETTINGS, [SCAN_COUNT], then that many scanlines,
// each led by its own [NUM_LINE], the number of runs.
static bool cceOpPolyScanlines(R128Cell& c, const u32* body, u32 n)
{
    u32 i = cceSettings(c, body, n);
    if (i >= n)
        return false;
    const u32 scans = body[i++];
    for (u32 s = 0; s < scans; ++s) {
        if (i >= n)
            return false;
        const u32 runs = body[i++];
        i = cceScanline(c, body, i, n, runs);
        if (i > n)
            return false;
    }
    return true;
}

// PLY_NEXTSCAN (0x1D): one more scanline "using the current settings": no
// SETTINGS, no counts; the runs are whatever the packet has left.
static bool cceOpPlyNextScan(R128Cell& c, const u32* body, u32 n)
{
    return n >= 1u && cceScanline(c, body, 0, n, n - 1u) <= n;
}

static u32 cceConvColor(u32 rgbq, u32 dstType);

// TRANS_BITBLT (0x9C, SDK F.19): a copy from video memory with a color
// compare. After SETTINGS: [CLR_CMP_CNTL], the source and destination
// reference colors as RGBQUADs, then [SRC_Y | SRC_X], [DST_Y | DST_X] and
// [DST_H | DST_W], each with its first-named half in the high word. The
// compare word has CLR_CMP_CNTL's own layout (functions in 2:0 and 10:8,
// which of them in 25:24), so the packet is the three compare registers and
// one blit. They are left as the packet set them: the manual says nothing
// of putting them back, and a driver that wants the compare off afterwards
// has GMC_CLR_CMP_CNTL_DIS in its next GUI_CONTROL.
static bool cceOpTransBitblt(R128Cell& c, const u32* body, u32 n)
{
    const u32 i = cceSettings(c, body, n);
    if (i > n || i + 6u > n)
        return false;
    const u32 dstType = c.peek(kDpDatatype) & 0xFu;
    cceRegWrite(c, 0x15C4u, cceConvColor(body[i + 1u], dstType));
    cceRegWrite(c, 0x15C8u, cceConvColor(body[i + 2u], dstType));
    cceRegWrite(c, kClrCmpCntl, body[i]);
    const u32 s = body[i + 3u], d = body[i + 4u], hw = body[i + 5u];
    const u32 w = hw & 0x3FFFu, h = (hw >> 16) & 0x3FFFu;
    if (!w || !h)
        return true;
    u32 sx = s & 0xFFFFu, sy = s >> 16;
    u32 dx = d & 0xFFFFu, dy = d >> 16;
    // Walk away from the source, as BITBLT_MULTI does, so an overlapping
    // copy reads each pixel before it is written.
    const bool l2r = dx <= sx, t2b = dy <= sy;
    if (!l2r) {
        sx += w - 1u;
        dx += w - 1u;
    }
    if (!t2b) {
        sy += h - 1u;
        dy += h - 1u;
    }
    cceRegWrite(c, kDpCntl, (c.peek(kDpCntl) & ~3u) | (l2r ? 1u : 0u) |
                                (t2b ? 2u : 0u));
    cceRegWrite(c, 0x1434u, (sy << 16) | sx); // SRC_Y_X
    cceRegWrite(c, 0x1438u, (dy << 16) | dx); // DST_Y_X
    cceRegWrite(c, 0x143Cu, (h << 16) | w);   // DST_HEIGHT_WIDTH: draw
    return true;
}

// How many bytes a pixel takes for the packets that write video memory
// themselves (SDK Table F-3, by GUI_CONTROL's destination type): 2 and 7 one
// byte, 3 and 4 two, 6 four. A one-byte pixel is a palette index (type 2) or
// RGB 3:3:2 (type 7), and a color a packet carries for one is that byte: no
// conversion from an RGBQUAD can make an index. Type 5, three bytes, stays
// declined: nothing says how its host data is packed into DWORDs.
static u32 ccePixelBytes(u32 dstType)
{
    switch (dstType) {
    case 2u:
    case 7u: return 1;
    case 3u:
    case 4u: return 2;
    case 6u: return 4;
    default: return 0;
    }
}

static bool cceDstState(R128Cell& c, u32 dstType, CceDst& d)
{
    d.bpp = ccePixelBytes(dstType);
    if (!d.bpp) {
        ++gEng.badBpp;
        return false;
    }
    d.dstOff = c.peek(0x1404u);
    d.pitchBytes = c.peek(0x1408u) * 8u * d.bpp;
    d.scL = static_cast<int>(c.peek(0x1640u) & 0x3FFFu);
    d.scR = static_cast<int>(c.peek(0x1644u) & 0x3FFFu);
    d.scT = static_cast<int>(c.peek(0x1648u) & 0x3FFFu);
    d.scB = static_cast<int>(c.peek(0x164Cu) & 0x3FFFu);
    return true;
}
static u32 cceConvColor(u32 rgbq, u32 dstType)
{
    const u32 rr = (rgbq >> 16) & 0xFFu, gg = (rgbq >> 8) & 0xFFu,
              bb = rgbq & 0xFFu;
    if (dstType == 4u)
        return ((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3);
    if (dstType == 3u)
        return ((rr >> 3) << 10) | ((gg >> 3) << 5) | (bb >> 3);
    return rgbq;
}
// One TRANSPARENT mono glyph: set bits draw the foreground, clear bits
// leave the destination — the text ops' rendering. Bit consumption matches
// the HOSTDATA mono path (row-major, continuous across DWORDs, PIX_ORDER
// picks the end).
static void cceDrawMonoGlyph(R128Cell& c, const CceDst& d, int bx, int by,
                             u32 wPx, u32 hPx, const u32* raster, u32 mWords,
                             u32 fg, bool lsbFirst)
{
    u64 bit = 0;
    const u64 avail = static_cast<u64>(mWords) * 32u;
    for (u32 yy = 0; yy < hPx; ++yy)
        for (u32 xx = 0; xx < wPx; ++xx, ++bit) {
            if (bit >= avail)
                return;
            const u32 word = raster[bit >> 5];
            const u32 bIdx = static_cast<u32>(bit & 31u);
            const u32 on =
                lsbFirst ? (word >> bIdx) & 1u : (word >> (31u - bIdx)) & 1u;
            if (!on)
                continue;
            const int px = bx + static_cast<int>(xx);
            const int py = by + static_cast<int>(yy);
            if (px < d.scL || px > d.scR || py < d.scT || py > d.scB)
                continue;
            const u64 at = static_cast<u64>(d.dstOff) +
                           static_cast<u64>(py) * d.pitchBytes +
                           static_cast<u64>(px) * d.bpp;
            if (at + d.bpp > c.vram.size())
                continue;
            if (r128PixelWatched(at, d.bpp))
                r128PixelWatchNote("2d glyph", "a 1-bit glyph at (%d,%d), "
                                               "wrote %x",
                                   bx, by, fg);
            for (u32 k = 0; k < d.bpp; ++k)
                c.vram[at + k] =
                    static_cast<u8>(fg >> (8u * (d.bpp - 1u - k)));
            ++gEng.pixels;
        }
}

// SMALL_TEXT (0x93): a run of packed mono glyphs. Body after SETTINGS:
// [FRGD_COLOUR (RGBQUAD)] [BAS_Y|BAS_X] then per glyph
// [H 31:25 | W 23:16 | Ydev 15:8 | Xdev 7:0] + ceil(W*H/32) raster DWORDs.
// The pen ADVANCES by each glyph's Xdev (the SDK's worked example: x1=0 for
// the first glyph), and Ydev is the glyph's rise above the baseline. The
// foreground RGBQUAD is remembered in DP_SRC_FRGD_CLR for NEXTCHAR.
static bool cceOpSmallText(R128Cell& c, const u32* body, u32 n)
{
    const u32 i = cceSettings(c, body, n);
    if (i > n || i + 2u > n)
        return false;
    const u32 gc = body[0];
    const bool lsbFirst = (gc >> 14) & 1u;
    CceDst d;
    if (!cceDstState(c, (gc >> 8) & 0xFu, d))
        return false;
    const u32 fgq = body[i];
    const u32 fg = cceConvColor(fgq, (gc >> 8) & 0xFu);
    cceRegWrite(c, 0x15D8u, fgq); // NEXTCHAR's "default foreground"
    int penX = static_cast<short>(body[i + 1u] & 0xFFFFu);
    const int basY = static_cast<short>(body[i + 1u] >> 16);
    u32 j = i + 2u;
    while (j < n) {
        const u32 g = body[j];
        const u32 dx = g & 0xFFu, dy = (g >> 8) & 0xFFu;
        const u32 w = (g >> 16) & 0xFFu, h = g >> 25;
        const u32 m = (w * h + 31u) / 32u;
        if (j + 1u + m > n)
            return false; // malformed against the header count
        penX += static_cast<int>(dx);
        if (w && h)
            cceDrawMonoGlyph(c, d, penX, basY - static_cast<int>(dy), w, h,
                             &body[j + 1u], m, fg, lsbFirst);
        j += 1u + m;
    }
    ++gEng.blits;
    return true;
}

// NEXTCHAR (0x19): one glyph, no SETTINGS — the current GMC supplies the
// destination type and pixel order, DP_SRC_FRGD_CLR (set by SMALL_TEXT)
// the color. Body: [DST_Y|DST_X] [DST_H|DST_W] [raster...], both words in
// the ENGINE's own layout for once.
static bool cceOpNextChar(R128Cell& c, const u32* body, u32 n)
{
    if (n < 3u)
        return false;
    const u32 gmc = c.peek(0x146Cu);
    CceDst d;
    if (!cceDstState(c, (gmc >> 8) & 0xFu, d))
        return false;
    const u32 fg = cceConvColor(c.peek(0x15D8u), (gmc >> 8) & 0xFu);
    const u32 yx = body[0], hw = body[1];
    cceDrawMonoGlyph(c, d, static_cast<short>(yx & 0xFFFFu),
                     static_cast<short>(yx >> 16), hw & 0xFFFFu, hw >> 16,
                     &body[2], n - 2u, fg, (gmc >> 14) & 1u);
    ++gEng.blits;
    return true;
}

// SET_SCISSORS (0x1E): [TOP_LEFT][BOTTOM_RIGHT], the same field layout the
// SETTINGS block uses. Written to the LIVE scissor aliases AND to
// DEFAULT_SC_BOTTOM_RIGHT: a later packet whose GUI_CONTROL says "use the
// default clipping" reloads TL=(0,0), BR=DEFAULT_SC_BOTTOM_RIGHT, so a
// SET_SCISSORS that skipped the default register would be undone by the
// very next op's GMC write.
static bool cceOpSetScissors(R128Cell& c, const u32* body, u32 n)
{
    if (n < 2u)
        return false;
    cceRegWrite(c, 0x16ECu, body[0]);
    cceRegWrite(c, 0x16F0u, body[1]);
    cceRegWrite(c, 0x16E8u, body[1]);
    return true;
}

// Fetch a pending indirect body and SPLICE IT AT THE PARSE HEAD, so the
// body executes after the packet that dispatched it and before whatever the
// guest queued next — the order the hardware gives.
static void cceDrainPending(R128Cell& c, const u8* ram, u32 ramSize,
                            u32 gartBase, u32 aperBase, SnoopSink* snoop)
{
    while (!gCcePendingInd.empty()) {
        const u32 sizeDwords = gCcePendingInd.front();
        gCcePendingInd.erase(gCcePendingInd.begin());
        if (gCceDepth <= 0) {
            // A chain of indirect dispatches longer than any real frame's
            // submission is a cycle — a body re-dispatching itself would
            // hang the EMULATOR, not the guest. The budget is set at each
            // guest-visible entry; overruns are counted, not silent.
            ++gEng.cceIndNested;
            continue;
        }
        --gCceDepth;
        const u32 indOff = c.peek(kIwIndOff);
        const u32 n = sizeDwords & 0xFFFFu;
        if (!n || n > 0x4000u)
            continue;
        ++gEng.cceIndFetch;
        std::vector<u32> body;
        body.reserve(n);
        for (u32 i = 0; i < n; ++i) {
            u32 w = 0;
            if (!cceGartRead(ram, ramSize, gartBase, aperBase,
                             indOff + 4u * i, w, snoop))
                break;
            ++gEng.cceIndWords;
            body.push_back(w);
        }
        gCceFifo.insert(gCceFifo.begin() + static_cast<long>(gCceHead),
                        body.begin(), body.end());
    }
}

void r128CceIndirect(R128Cell& c, u32 sizeDwords, const u8* ram, u32 ramSize,
                     u32 gartBase, u32 aperBase, SnoopSink* snoop)
{
    if (!gEng2dOn || !ram)
        return;
    OPM_MARK(Cce);
    const CceMem m{ram, ramSize, gartBase, aperBase, snoop};
    r128AsyncDrain(c, m); // a direct dispatch goes behind what is queued
    gCceDepth = 64; // dispatch budget for this guest-visible entry
    gCcePendingInd.push_back(sizeDwords);
    asyncSubmit(c, m);
}

// A register a stream may write while triangles are still waiting to be
// drawn (opm/r128.hpp, "Triangles waiting to be drawn"): its write stores
// state that the next primitive is gathered from and does nothing else. The
// 3D context block with its four 2D-shared cells, SCALE_3D_CNTL, the two
// pixel cache flushes a driver sends with every block of state, which this
// model has no cache to act on, and the pair that dispatches an indirect
// buffer, which is more stream and no drawing. Anything else is settled for
// first: a 2D operation's trigger, the fog table, a scratch register the
// guest is polling for.
static bool cceWriteIsState(u32 off)
{
    off &= 0x3FFCu;
    return (off >= 0x1C80u && off < 0x1D48u) || off == 0x1A00u ||
           off == kPcGuiCtlstat || off == 0x171Cu || off == kIwIndOff ||
           off == kIwIndSize;
}

static CceRun cceRunLoop(R128Cell& c, const CceMem& m, CceMode mode);

// The packet loop. Every CCE packet — 2D blits and the whole 3D rasterizer —
// executes inside it, on whichever thread `mode` says is running it. However
// it is left, no triangle is still waiting to be drawn.
static CceRun cceRun(R128Cell& c, const CceMem& m, CceMode mode)
{
    const CceRun how = cceRunLoop(c, m, mode);
    r128RowsSettle();
    return how;
}

static CceRun cceRunLoop(R128Cell& c, const CceMem& m, CceMode mode)
{
    const bool worker = mode == CceMode::Worker;
    for (;;) {
        if (worker && gAsync.drainReq.load(std::memory_order_relaxed))
            return CceRun::Yield; // between packets: the engine can be taken
        // Drop what has been consumed, here between packets where nothing
        // points into the vector. Not only when the stream runs dry: with the
        // engine on its own thread a busy title keeps it fed and it may not
        // run dry for a long time.
        if (gCceHead > 0x10000u) {
            gCceFifo.erase(gCceFifo.begin(),
                           gCceFifo.begin() + static_cast<long>(gCceHead));
            gCceHead = 0;
        }
        // A dispatched indirect buffer is fetched out of system memory.
        if (!gCcePendingInd.empty()) {
            // On the engine's thread memory is as it stood at the hand-off,
            // and this dispatch may be younger than that.
            if (worker)
                return CceRun::NeedsEmu;
            cceDrainPending(c, m.ram, m.ramSize, m.gartBase, m.aperBase,
                            m.snoop);
        }
        const size_t avail = gCceFifo.size() - gCceHead;
        u32 w = 0, type = 2u, n = 0;
        bool complete = false;
        if (avail) {
            w = gCceFifo[gCceHead];
            type = w >> 30;
            n = ((w >> 16) & 0x3FFFu) + 1u;
            complete = type == 2u || (type == 1u ? avail >= 3u
                                                 : avail >= 1u + n);
        }
        if (!complete) {
            if (mode == CceMode::Sync)
                break;
            // Nothing runnable is staged. Words the guest wrote while the
            // engine was running come in now; and if there are none, this is
            // where the engine goes idle — decided under the inbox lock, so a
            // word arriving at this moment is either taken here or executed
            // by its own writer, never left behind.
            std::lock_guard<std::mutex> ib(gAsync.inboxMu);
            if (!gAsync.inbox.empty()) {
                // ...by the machine's thread only. What these words refer to
                // — a texture, a vertex buffer, the body of a dispatch — was
                // written after the hand-off, and the engine's thread sees
                // memory as it stood then.
                if (worker)
                    return CceRun::NeedsEmu;
                gCceFifo.insert(gCceFifo.end(), gAsync.inbox.begin(),
                                gAsync.inbox.end());
                gAsync.inbox.clear();
                gCceDepth = 64; // each of these was a guest-visible entry
                continue;
            }
            // Idle means drawn: the guest may touch video memory, and read
            // its own markers, the moment the flag falls.
            r128RowsSettle();
            if (gAsync.busyTimed) {
                gAsync.busyTimed = false;
                gAsync.st.busyNs += static_cast<u64>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - gAsync.busySince)
                        .count());
            }
            gR128AsyncBusy.store(false, std::memory_order_release);
            break;
        }
        if (type == 2u) { // filler
            ++gCceHead;
            ++gEng.ccePkt2;
            continue;
        }
        // Triangles still waiting to be drawn are drawn before any packet
        // but a 3D primitive (which waits behind them, or settles for itself)
        // and a write of plain 3D state. Here, before the register lock: the
        // wait is the drawing threads', and a guest polling a register
        // should not stand behind it.
        if (gR128RowsPending) {
            bool state;
            if (type == 0u) {
                const u32 base = (w & 0x7FFu) << 2;
                state = true;
                if ((w >> 15) & 1u)
                    state = cceWriteIsState(base);
                else
                    for (u32 i = 0; i < n && state; ++i)
                        state = cceWriteIsState(base + 4u * i);
            } else if (type == 1u) {
                state = cceWriteIsState((w & 0x7FFu) << 2) &&
                        cceWriteIsState(((w >> 11) & 0x7FFu) << 2);
            } else {
                const u32 op = (w >> 8) & 0xFFu;
                state = op == 0x25u || op == 0x23u || op == 0x2Eu;
            }
            if (!state)
                r128RowsSettle();
        }
        // The engine's thread holds the register lock for a packet's register
        // work; the 3D unit lets it go while it rasterizes.
        std::unique_lock<std::mutex> rl(gAsync.regMu, std::defer_lock);
        if (worker)
            rl.lock();
        if (type == 0u) {
            const u32 base = (w & 0x7FFu) << 2;
            const bool oneReg = (w >> 15) & 1u;
            gCceHead += 1u; // consume the header BEFORE executing: a body
                            // word may queue an indirect dispatch, and the
                            // splice point is wherever the head stands.
            for (u32 i = 0; i < n; ++i)
                cceRegWrite(c, oneReg ? base : base + 4u * i,
                            gCceFifo[gCceHead + i]);
            gCceHead += n;
            ++gEng.ccePkt0;
            continue;
        }
        if (type == 1u) {
            cceRegWrite(c, (w & 0x7FFu) << 2, gCceFifo[gCceHead + 1u]);
            cceRegWrite(c, ((w >> 11) & 0x7FFu) << 2,
                        gCceFifo[gCceHead + 2u]);
            gCceHead += 3u;
            ++gEng.ccePkt1;
            continue;
        }
        // Type-3: an operation packet. COUNT = N-1 body DWORDs; the body is
        // consumed exactly by the HEADER count whatever the op does, so a
        // half-understood operation can never desynchronize the stream.
        const u32 op = (w >> 8) & 0xFFu;
        const bool op3d =
            op == 0x25u || op == 0x23u || op == 0x2Eu || op == 0x2Cu;
        if (mode != CceMode::Sync) {
            // An operation at the head of a long stream is the engine
            // thread's; a short one costs less here than waking it.
            if (mode == CceMode::EmuFront && avail >= kHandOffWords)
                return CceRun::HandOff;
            if (worker)
                ++gAsync.st.workerPkts;
            else if (mode == CceMode::EmuDrain)
                ++gAsync.st.drainPkts;
            else
                ++gAsync.st.inlinePkts;
        }
        const u32* body = &gCceFifo[gCceHead + 1u];
        bool done = false;
        if (op == 0x91u) // PAINT — SDK F.9
            done = cceOpPaint(c, body, n);
        else if (op == 0x9Au) // PAINT_MULTI — SDK F.17
            done = cceOpPaintMulti(c, body, n);
        else if (op == 0x94u) // HOSTDATA_BLT — SDK F.11
            done = cceOpHostBlt(c, body, n);
        else if (op == 0x9Bu) // BITBLT_MULTI — SDK F.18
            done = cceOpBitbltMulti(c, body, n);
        else if (op == 0x92u)
            // BITBLT, the single-rectangle form — absent from the SDK's own
            // opcode table. Decoded from a live window drag: GUI_CONTROL
            // (no brush, src-from-VRAM, ROP CC) then one [SRC][DST][H_W]
            // rect, byte-identical to a one-rect BITBLT_MULTI — dst arrived
            // at src plus exactly the commanded drag delta. Mac OS moves a
            // window's pixels with THIS op, so skipping it left every
            // dragged window's content behind — "can't really see it any
            // more" until a close/reopen forced a full redraw.
            done = cceOpBitbltMulti(c, body, n);
        else if (op == 0x93u) // SMALL_TEXT — SDK F.10
            done = cceOpSmallText(c, body, n);
        else if (op == 0x19u) // NEXTCHAR — SDK F.16
            done = cceOpNextChar(c, body, n);
        else if (op == 0x1Eu) // SET_SCISSORS — SDK F.22
            done = cceOpSetScissors(c, body, n);
        else if (op == 0x95u) // POLYLINE — SDK F.12
            done = cceOpPolyline(c, body, n);
        else if (op == 0x98u) // POLYSCANLINES — SDK F.15
            done = cceOpPolyScanlines(c, body, n);
        else if (op == 0x1Du) // PLY_NEXTSCAN — SDK F.20
            done = cceOpPlyNextScan(c, body, n);
        else if (op == 0x9Cu) // TRANS_BITBLT — SDK F.19
            done = cceOpTransBitblt(c, body, n);
        else if (op == 0x1Fu)
            // SET_MODE_24BPP — SDK F.23: "the 24bpp flag in the microcode
            // engine", one word. The manual gives the flag no effect a
            // packet's result could show, and the packets here take the
            // pixel width from GUI_CONTROL's destination type, 24 bpp
            // included. Taken and counted, so the stream stays in step.
            done = n == 1u;
        else if (op3d) { // the 3D packets — SDK F.24/F.26/F.27/F.21
            CceMem pm = m;
            pm.worker = worker;
            done = r128Cce3dOp(c, op, body, n, pm);
        } else if (op == 0x2Du) {
            // PURGE — flush and purge the pixel cache. Every operation here
            // is resident in VRAM the moment its packet executes, so the
            // purge is complete when asked for, same as PC_GUI_CTLSTAT.
            ++gEng.cacheFlushes;
            done = true;
        }
        if (!done) {
            // The map says WHICH op was skipped; the first few skipped
            // packets say WHAT IT CARRIED — which is the whole question
            // when the op is one the SDK's own table does not list
            // (opcode 0x92 came off a live window drag exactly so).
            if (++gCceP3[op] <= 2) {
                printf("-- cce pkt3 op %02x SKIPPED, %u words:", op, n);
                for (u32 k = 0; k < n && k < 12; ++k)
                    printf(" %08x",
                           gCceFifo[gCceHead + 1u + k]);
                printf("%s\n", n > 12 ? " ..." : "");
            }
        }
        gCceHead += 1u + n;
        ++gEng.ccePkt3;
    }
    // Compact once drained so the staging vector cannot grow without bound
    // across a long session.
    if (gCceHead == gCceFifo.size()) {
        gCceFifo.clear();
        gCceHead = 0;
    }
    return CceRun::Empty;
}

namespace {

// The engine's thread: run packets whenever there are some and nobody on the
// machine's thread has asked for the engine.
void asyncWorkerMain()
{
    std::unique_lock<std::mutex> lk(gAsync.engMu);
    for (;;) {
        gAsync.engCv.wait(lk, [] {
            return gAsync.quit ||
                   (gR128AsyncBusy.load() && !gAsync.drainReq.load() &&
                    !gR128AsyncNeedsEmu.load() && !gAsync.hold.load());
        });
        if (gAsync.quit)
            return;
        const auto t0 = std::chrono::steady_clock::now();
        const CceRun how = cceRun(*gAsync.cell, gAsync.mem, CceMode::Worker);
        gAsync.st.workNs += static_cast<u64>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - t0)
                .count());
        if (how == CceRun::NeedsEmu)
            gR128AsyncNeedsEmu.store(true);
        // Empty: the run let the busy flag fall itself. Yield: the wait
        // above gives the lock up until the request is withdrawn.
    }
}

// Take the engine from its thread: it stops at the next packet boundary and
// stays stopped until the request is withdrawn.
struct EngineTaken {
    std::unique_lock<std::mutex> lk;
    EngineTaken()
    {
        gAsync.drainReq.store(true);
        lk = std::unique_lock<std::mutex>(gAsync.engMu);
    }
    ~EngineTaken()
    {
        gAsync.drainReq.store(false);
        lk.unlock();
        gAsync.engCv.notify_one();
    }
};

// What the engine's thread is about to be given, under engMu: the cell, and
// system memory. A card-initiated read of memory is snooped against the
// processor's caches, and those are the machine thread's; so the modified
// lines are copied out to memory HERE (SnoopSink::publish — nothing else
// about the caches changes), and the engine's thread then reads memory
// directly, as it stood at this moment. Everything the stream refers to — its
// textures in AGP space, a vertex buffer, the GART table itself — was written
// by the guest before it submitted the stream, so it is all there.
void asyncRelease(R128Cell& c, const CceMem& m)
{
    if (m.snoop && m.ram)
        m.snoop->publish(const_cast<u8*>(m.ram), m.ramSize);
    gAsync.cell = &c;
    gAsync.mem = m;
    gAsync.mem.snoop = nullptr;
    gAsync.mem.worker = true;
    ++gAsync.st.handOffs;
}

// The machine's thread has staged something and owns the engine (it was idle
// on entry): execute it, or the part of it that belongs here, and hand a long
// stream over.
void asyncSubmit(R128Cell& c, const CceMem& m)
{
    if (!gAsync.on.load()) {
        cceRun(c, m, CceMode::Sync);
        return;
    }
    if (cceRun(c, m, CceMode::EmuFront) != CceRun::HandOff)
        return;
    if (!gAsync.started) {
        gAsync.started = true;
        gAsync.th = std::thread(asyncWorkerMain);
    }
    {
        std::lock_guard<std::mutex> lk(gAsync.engMu);
        asyncRelease(c, m);
        gAsync.busySince = std::chrono::steady_clock::now();
        gAsync.busyTimed = true;
        gR128AsyncBusy.store(true, std::memory_order_release);
    }
    gAsync.engCv.notify_one();
}

} // namespace

void r128AsyncDrain(R128Cell& c, const CceMem& m)
{
    if (!r128AsyncBusy())
        return;
    OPM_MARK(Cce);
    const auto t0 = std::chrono::steady_clock::now();
    const u64 pkts0 = gAsync.st.drainPkts;
    {
        EngineTaken taken;
        if (r128AsyncBusy())
            cceRun(c, m, CceMode::EmuDrain); // ends by letting busy fall
        gR128AsyncNeedsEmu.store(false);
    }
    const u64 ns = static_cast<u64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());
    ++gAsync.st.drains;
    gAsync.st.drainNs += ns;
    R128DrainCause& dc = gAsync.causes[gAsync.cause];
    ++dc.n;
    dc.ns += ns;
    dc.pkts += gAsync.st.drainPkts - pkts0;
    gAsync.cause = kR128DrainOther;
}

void r128AsyncService(R128Cell& c, const CceMem& m)
{
    if (!r128AsyncNeedsService())
        return;
    OPM_MARK(Cce);
    // The engine's thread is parked on the flag, so the lock is free.
    std::unique_lock<std::mutex> lk(gAsync.engMu);
    if (!gR128AsyncNeedsEmu.load())
        return;
    ++gAsync.st.services;
    gCceDepth = 64; // dispatch budget, as at any entry that fetches
    const CceRun r = cceRun(c, m, CceMode::EmuFront);
    if (r == CceRun::HandOff)
        asyncRelease(c, m); // still busy: the rest is the engine's
    gR128AsyncNeedsEmu.store(false);
    lk.unlock();
    if (r == CceRun::HandOff)
        gAsync.engCv.notify_one();
}

void r128AsyncAbandon(const R128Cell* c)
{
    if (!gAsync.started)
        return;
    EngineTaken taken;
    if (c && gAsync.cell != c)
        return;
    gCceFifo.clear();
    gCceHead = 0;
    gCcePendingInd.clear();
    {
        std::lock_guard<std::mutex> ib(gAsync.inboxMu);
        gAsync.inbox.clear();
        gAsync.busyTimed = false;
        gR128AsyncBusy.store(false, std::memory_order_release);
    }
    gR128AsyncNeedsEmu.store(false);
    gAsync.cell = nullptr;
}

void r128CceFifoWord(R128Cell& c, u32 raw, u32 len, const u8* ram,
                     u32 ramSize, u32 gartBase, u32 aperBase,
                     SnoopSink* snoop)
{
    if (!gEng2dOn || !ram)
        return;
    if (len != 4u) {
        ++gEng.cceBadWidth;
        return;
    }
    OPM_MARK(Cce); // staging is engine cost too, not the guest's store
    ++gEng.cceWords;
    if (gAsync.started) {
        // The engine is running: this word goes behind what it has, in the
        // order it was written. Decided under the inbox lock — see cceRun.
        std::lock_guard<std::mutex> ib(gAsync.inboxMu);
        if (r128AsyncBusy()) {
            gAsync.inbox.push_back(cceBswap(raw));
            ++gAsync.st.inboxWords;
            return;
        }
    }
    gCceDepth = 64; // dispatch budget for this guest-visible entry
    gCceFifo.push_back(cceBswap(raw));
    asyncSubmit(c, CceMem{ram, ramSize, gartBase, aperBase, snoop});
}

size_t r128CceStaged(u32& headWord)
{
    const size_t staged = gCceFifo.size() - gCceHead;
    headWord = staged ? gCceFifo[gCceHead] : 0;
    return staged;
}

// ── The CCE ring (SDK §5.3) ───────────────────────────────────────────────
//
// In the bus-mastered-packet modes the driver builds packets in a ring in
// AGP space, advances PM4_BUFFER_DL_WPTR, and the card fetches and executes
// everything between DL_RPTR and DL_WPTR on its own. The ring size is log2
// QUADWORDS in PM4_BUFFER_CNTL's low bits (the SDK's worked example writes
// 0x…0A and calls it "8 K"), the pointers are DWORD indices, and after a
// fetch the card BUS-MASTERS the new read pointer into system memory at
// PM4_BUFFER_DL_RPTR_ADDR — the documented hardware-writes-to-RAM
// completion mechanism, and the one register no open-source driver ever
// programmed. The 2D queue measured on this machine is PIO mode 7, so this
// path stays cold until a 3D-era driver arms it; it exists so that arming
// it is an execution, not a hang.
void r128CceRingKick(R128Cell& c, u8* ram, u32 ramSize, u32 gartBase,
                     u32 aperBase, SnoopSink* snoop)
{
    if (!gEng2dOn || !ram)
        return;
    const u32 cntl = c.peek(0x0704u);
    const u32 mode = cntl >> 28;
    // Packet fetch out of the ring exists only in the modes whose CCE
    // packet source is bus-mastered (192BM, 128BM+64IND, 64BM+128IND,
    // 64BM+64VC+64IND). The PIO modes feed PM4_FIFO_DATA and leave the
    // ring registers inert.
    if (mode != 2u && mode != 4u && mode != 6u && mode != 8u)
        return;
    if (!(c.peek(0x07FCu) & 0x40000000u)) // PM4_MICRO_CNTL FREERUN off:
        return;                           // the microengine is not running
    const u32 l2qw = cntl & 0x3Fu;
    if (l2qw > 20u) // 8 MB of ring is already absurd; refuse to index wilder
        return;
    const u32 sizeDw = 2u << l2qw;
    const u32 base = c.peek(0x0700u); // PM4_BUFFER_OFFSET, AGP space
    u32 rptr = c.peek(0x0710u) & (sizeDw - 1u);
    const u32 wptr = c.peek(0x0714u) & (sizeDw - 1u);
    if (rptr == wptr)
        return;
    OPM_MARK(Cce); // ring fetch + execute: the engine's cost, named
    const CceMem m{ram, ramSize, gartBase, aperBase, snoop};
    r128AsyncDrain(c, m); // behind whatever is queued
    ++gEng.cceRingKicks;
    gCceDepth = 64; // indirect budget, per guest-visible entry
    while (rptr != wptr) {
        u32 w = 0;
        if (!cceGartRead(ram, ramSize, gartBase, aperBase,
                         base + 4u * rptr, w, snoop)) {
            // A miss mid-ring: abandon the kick with the pointer parked at
            // the failure, so the report shows a stalled ring instead of a
            // stream spliced around a hole.
            ++gEng.cceRingStall;
            break;
        }
        gCceFifo.push_back(w);
        ++gEng.cceRingWords;
        rptr = (rptr + 1u) & (sizeDw - 1u);
    }
    c.setReg(0x0710u, rptr);
    // Executed here whatever the engine's thread is for: the write-back
    // below says the ring was CONSUMED, and a driver believes it.
    cceRun(c, m, CceMode::Sync);
    // The read-pointer write-back. Snooped: a status word landing under a
    // live stale cache line is invisible to the driver polling it —
    // session 30's DBDMA lesson, applied to the one mechanism the SDK
    // documents for exactly this purpose.
    const u32 rpa = c.peek(0x070Cu) & ~3u;
    if (rpa && static_cast<u64>(rpa) + 4u <= ramSize) {
        if (snoop)
            snoop->snoopWrite(rpa, 4);
        ram[rpa] = static_cast<u8>(rptr);
        ram[rpa + 1] = static_cast<u8>(rptr >> 8);
        ram[rpa + 2] = static_cast<u8>(rptr >> 16);
        ram[rpa + 3] = static_cast<u8>(rptr >> 24);
    }
}


u32 R128Cell::regRead(u32 idx)
{
    const u32 off = idx << 2;
    // GUI_STAT and PM4_STAT: engine idle, command FIFO empty. Ahead of the
    // switch so that --no-ati-2d falls through to exactly the register model
    // that was here before the engine existed, unbacked-read census included.
    //
    // ⚠⚠ ZERO IS NOT IDLE HERE, IT IS THE WORST ANSWER AVAILABLE. Bits 11:0
    // are the count of FREE command-FIFO entries, and the reset default is
    // 0x40. Answering zero tells a driver the FIFO is completely full, so a
    // driver that waits for room before writing waits forever — and it never
    // appears in the unclaimed-access log, because the card claims its whole
    // aperture. Session 30 wrote this down for mac-io as *claiming an address
    // and implementing it are different things*; this is the same trap, on
    // the register a command engine's bring-up reads first.
    if (gEng2dOn && (off == kGuiStat || off == kPm4Stat)) {
        if (!r128AsyncBusy())
            return kGuiStatIdle;
        // The engine is running on its own thread and the guest is asking
        // how it stands (the bus does not finish the queue for this read —
        // SawtoothBus::atiBeforeRead). The free count is the 64-entry FIFO
        // less the words written behind the engine; busy is PM4_BUSY at 16
        // and GUI_ACTIVE at 31, the same flip-flops in both registers.
        const u32 queued = r128AsyncFifoQueued();
        const u32 room = queued < kGuiStatIdle ? kGuiStatIdle - queued : 0u;
        return room | 0x00010000u | 0x80000000u;
    }
    switch (off) {
    case kConfigAper0Base: {
        // The framebuffer aperture BASE, not its size. Returning 32 MB here
        // handed the driver 0x02000000 as the VRAM address, so it painted
        // into system RAM and the framebuffer counter stayed at zero in
        // every run. CONFIG_APER_SIZE is 0x0108.
        return fbBase;
    }
    case kConfigAper1Base:
        // The SECOND aperture: the same 32 MB of VRAM seen through the
        // big-endian alias, which is the one a Mac driver paints through
        // and which the card's own FCode publishes in its `address`
        // property (BAR0 + 0x02000000 + the CRTC offset).
        //
        // This register was not modeled and read back as zero. Measured
        // consequence: the OS display driver stores it as its framebuffer
        // base (device record +0x8c), adds the CRTC offset it programmed
        // (+0x138 = 0x8000), and fills 640x480x32 from address 0x00008000 —
        // 1.2 MB straight over low memory. That wipes ExpandMem, the next
        // dereference of the poisoned cell raises a bus error, and the ROM
        // sad-Macs into `bra.s *` at ffc046ee. The screen staying blank was
        // the mildest symptom of it.
        return fbBase + kAper1Offset;
    case kConfigReg1Base:
        // The base of register aperture ONE, which is not the BAR. The chip
        // has two identical copies of its register aperture so that "for the
        // PowerMac environment ... one aperture [can] be marked as cacheable"
        // (RAGE 128 PRO Register Reference Guide 2.2): aperture 0 sits at
        // REG_BASE from PCI config space, aperture 1 at this register, whose
        // field is bits 31:13 with its lowest bit HARDWIRED TO ONE — that is,
        // the BAR plus 0x2000, the upper half of the 16 KB window.
        //
        // ⚠ Answering with the BAR itself, and 16 KB for the size below, told
        // the driver that aperture 1 WAS the whole window. Mac OS 9.1's
        // driver takes the guide at its word — SetProcessorCacheMode(this
        // base, CONFIG_REG_APER_SIZE, copy-back) — so it made every register
        // it owns cacheable, aperture 0 included. Its interrupt acknowledges
        // and cursor-position stores then sat in the L1 instead of reaching
        // the chip: 20,397 of 20,407 vertical blanks expired unanswered, the
        // slot-VBL task never ran, and the pointer never moved.
        return (regBase & 0xFFFFC000u) | 0x00002000u;
    case kConfigRegAperSize:
        // The size of EACH register aperture, not of the BAR that holds
        // both: REG_APER_SIZE 13:0 reads 0x2000 (bits 12:0 hardwired zero).
        return 0x00002000u;
    case kConfigMemsize:
    case kConfigAperSize: {
        // Power-on default 32 MB; if the init code programs the field,
        // read back what it wrote (verify loops depend on it).
        auto it = regs_.find(off);
        return it != regs_.end() ? it->second : (32u << 20);
    }
    case kGenIntStatus: {
        // Vertical blank. A display driver arms CRTC_VBLANK_INT in
        // GEN_INT_CNTL and then waits on this status bit before it will
        // paint, so with nothing here the wait never ends and the
        // framebuffer stays untouched however correct the modeset was.
        //
        // The latch is now real: tick() sets it once per 1/60 s of guest
        // time and an acknowledge clears it, which is what lets the pin
        // fall and rise again. Reporting the blank as permanently pending
        // was fine for a polling loop and is actively wrong for a handler
        // — an interrupt service routine that re-reads this register would
        // see a blank that never goes away.
        auto it = regs_.find(kGenIntStatus);
        const u32 v = it != regs_.end() ? it->second : 0;
        if (vblEnabled)
            return v;
        // --no-ati-vbl: the pre-interrupt machine, bit for bit.
        auto ic = regs_.find(kGenIntCntl);
        const bool armed = ic != regs_.end() && (ic->second & kCrtcVblankInt);
        return armed ? (v | kCrtcVblankInt) : v;
    }
    case kClockCntlIndex:
        return pllAddr_;
    case kClockCntlData: {
        auto it = pll_.find(pllAddr_ & 0x3Fu);
        // PLL_TEST_CNTL-style status: report locked/ready bits set so
        // spin-until-lock loops fall through.
        return it != pll_.end() ? it->second : 0x00000000u;
    }
    case kGpioA0:
    case kGpioA4:
    case kGpioA8: {
        // Bit 22 is the direction, bit 23 the level, and the lines are open
        // drain: undriven reads high, and either side can pull down. 0x00A4
        // reads the bus back.
        const u32 lvl = kLevelIsA0 ? gpioSda_ : gpioScl_;
        const u32 en = kLevelIsA0 ? gpioScl_ : gpioSda_;
        // Report per PIN, and apply the slave's pull-down to the lane the
        // DDC state machine actually uses. It was applied to kSdaBit
        // (bit 23) while the state machine drives and reads SDA on
        // kDdcSdaBit (bit 22), so the acknowledge landed on the wrong lane
        // and the master never saw it — which is why three corrections to
        // the slave in a row produced byte-identical counters.
        auto pin = [&](u32 bit) {
            const bool masterHigh = !(en & bit) || (lvl & bit);
            return masterHigh && (bit == kDdcSdaBit ? ddcSda() : true);
        };
        auto it = regs_.find(off);
        const u32 v = it != regs_.end() ? it->second : 0;
        return (v & 0xFF3FFFFFu) |
               (pin(0x00400000u) ? 0x00400000u : 0u) |
               (pin(0x00800000u) ? 0x00800000u : 0u);
    }
    case kGpioMonid:
    case kGpioDvi: {
        // DDC lines read back as driven; pulled-up (idle high) when
        // released. Bits 24-27 are the input reflections of 8-11.
        auto it = regs_.find(off);
        const u32 v = it != regs_.end() ? it->second : 0;
        u32 in = 0x0F000000u; // all lines high (no monitor yet)
        return (v & 0x00FFFFFFu) | in;
    }
    default: {
        auto it = regs_.find(off);
        if (it == regs_.end()) {
            // Never written, not modeled: this read is answering 0 and will
            // keep answering 0 forever. If a count here is large, the guest
            // is waiting on something this card does not have.
            ++gRegReadsUnbacked[off];
            return 0;
        }
        return it->second;
    }
    }
}

// A 14-bit signed coordinate. The scissors and the destination origin run
// -8192..8191, and a rectangle placed at a negative X against a scissor of 0
// is ordinary: it is how a window clipped at the screen edge is drawn.
static int sx14(u32 v)
{
    const int t = static_cast<int>(v & 0x3FFFu);
    return t >= 0x2000 ? t - 0x4000 : t;
}

// 🎨 ROP3, ALL 256 OF THEM, AS ONE FUNCTION.
//
// A Windows ternary raster op is a TRUTH TABLE, not an opcode: the code's bit
// at index (P<<2)|(S<<1)|D gives the result for that combination of pattern,
// source and destination. So there is nothing to special-case — evaluate the
// table across all the bits of a pixel at once and every op is exact.
//
// Enumerating a handful of named ops instead (SRCCOPY, PATCOPY, …) is what
// this engine did first, and it left 252 of them as "declined": correct, but
// it means a guest doing something as ordinary as XOR-ing a selection outline
// gets nothing drawn. Checked against ATI's own table in r128_reg.h —
// ROP3_S 0xCC, ROP3_P 0xF0, ROP3_D 0xAA, ROP3_Dn 0x55, ROP3_DSx 0x66,
// ROP3_DSa 0x88, ROP3_DSo 0xEE, ROP3_DPx 0x5A, ROP3_Pn 0x0F — all reproduced
// by this one expression, and the unit tests assert exactly that.
static u32 rop3(u32 rop, u32 P, u32 S, u32 D)
{
    u32 r = 0;
    if (rop & 0x01u) r |= ~P & ~S & ~D;
    if (rop & 0x02u) r |= ~P & ~S &  D;
    if (rop & 0x04u) r |= ~P &  S & ~D;
    if (rop & 0x08u) r |= ~P &  S &  D;
    if (rop & 0x10u) r |=  P & ~S & ~D;
    if (rop & 0x20u) r |=  P & ~S &  D;
    if (rop & 0x40u) r |=  P &  S & ~D;
    if (rop & 0x80u) r |=  P &  S &  D;
    return r;
}
// Which inputs the op actually reads. Two minterms that differ only in one
// input, and agree in the table, mean the op ignores that input — so an op
// that never looks at the source must not be refused for wanting a source
// surface it was never going to read.
static bool ropUsesSrc(u32 rop) { return (rop & 0x33u) != ((rop >> 2) & 0x33u); }
static bool ropUsesPat(u32 rop) { return (rop & 0x0Fu) != ((rop >> 4) & 0x0Fu); }
static bool ropUsesDst(u32 rop) { return (rop & 0x55u) != ((rop >> 1) & 0x55u); }

void R128Cell::engBlit()
{
    // --- how wide is a pixel ------------------------------------------
    const u32 dt = rd(kDpDatatype);
    r128SettingUsed(R128Setting::Dst2d, dt & 0xFu);
    u32 bpp = 0;
    switch (dt & 0xFu) {
    case 2: case 7: case 8: case 9: bpp = 8; break;  // pseudocolor, 332, Y8
    case 3: case 4: bpp = 16; break;                 // aRGB1555, RGB565
    case 5: bpp = 24; break;
    case 6: bpp = 32; break;
    default: break;                                  // YUV and 3D-only types
    }
    if (!bpp) {
        ++gEng.badBpp;
        return;
    }
    const u32 bypp = bpp / 8u;

    // --- what operation -----------------------------------------------
    const u32 mix = rd(kDpMix);
    const u32 rop = (mix >> 16) & 0xFFu;
    const u32 srcSource = (mix >> 8) & 0x7u;
    r128SettingUsed(R128Setting::SrcFrom2d, srcSource);
    r128SettingUsed(R128Setting::Tile2d, (rd(kDpCntl) >> 3) & 3u);
    // 3 and 4 are "loaded thru hostdata": the rectangle's pixels come from
    // the processor, through HOST_DATA0..7, after this trigger. See
    // engHostBegin.
    if (srcSource == 3u || srcSource == 4u) {
        engHostBegin(rop, bypp, srcSource == 4u);
        return;
    }
    const bool usesSrc = ropUsesSrc(rop);
    const bool usesPat = ropUsesPat(rop);
    const bool usesDst = ropUsesDst(rop);

    // The pattern operand. Brush datatype 13 is a solid color taken from
    // DP_BRUSH_FRGD_CLR, and 15 the manual says to "treat as 13".
    //
    // 0 and 1 are the 8x8 MONO PATTERN: 64 bits in BRUSH_DATA0 and 1, a set
    // bit drawing the foreground color and a clear one the background (0)
    // or nothing at all (1, "leave alone"). The books give the registers no
    // description; the layout is the XFree86 driver's use of them (its XAA
    // 8x8 pattern fill, r128_accel.c): the first register holds rows 0 to 3
    // and the second rows 4 to 7, a row a byte, the first row in the low
    // byte; the pixel order within a byte is DP_BYTE_PIX_ORDER's (most
    // significant bit leftmost, or least); and BRUSH_Y_X is where on the
    // SURFACE the pattern's first pixel sits, so a pattern stays put under
    // the rectangles drawn with it.
    //
    // The other pattern brushes (the 8x1, 1x8 and 32x1 mono ones, which are
    // for lines, the 32x32, the color ones) have no layout in anything on
    // the shelf: an operation that would READ one is declined rather than
    // painted with a solid guess where a chequer belongs.
    u32 pat = 0, patBkgd = 0, patX = 0, patY = 0;
    u64 patBits = 0;
    bool patMono = false, patLeave = false, patLsbFirst = false;
    if (usesPat) {
        const u32 brush = (dt >> 8) & 0xFu;
        r128SettingUsed(R128Setting::Brush2d, brush);
        if (brush == 0u || brush == 1u) {
            patMono = true;
            patLeave = brush == 1u;
            patBits = rd(kBrushData0) |
                      (static_cast<u64>(rd(kBrushData1)) << 32);
            patBkgd = rd(kDpBrushBkgdClr);
            patX = rd(kBrushYX) & 7u;
            patY = (rd(kBrushYX) >> 8) & 7u;
            patLsbFirst = (dt >> 30) & 1u;
        } else if (brush != 13u && brush != 15u) {
            ++gEng.brushUnimpl;
            return;
        }
        pat = rd(kDpBrushFrgdClr);
    }
    // Whether the pattern has its foreground at this pixel of the surface.
    const auto patOn = [&](int x, int y) {
        const u32 row = static_cast<u32>(
            (patBits >> (8u * ((static_cast<u32>(y) - patY) & 7u))) & 0xFFu);
        const u32 col = (static_cast<u32>(x) - patX) & 7u;
        return ((row >> (patLsbFirst ? col : 7u - col)) & 1u) != 0;
    };
    // The source operand: color pixels of the destination's own format
    // (3), or a ONE-BIT bitmap (guide, GMC_SRC_DATATYPE: 0 "mono, expanded
    // to frgd, bkgd", 1 "mono, expanded to frgd, leave alone"): a set bit is
    // DP_SRC_FRGD_CLR, a clear one DP_SRC_BKGD_CLR or, for type 1, no pixel
    // at all. A one-bit bitmap in video memory has a row every pitch bytes,
    // the pitch being in units of eight pixels, and DP_BYTE_PIX_ORDER says
    // which end of a byte is its left pixel. Type 2 is reserved.
    const u32 srcType = (dt >> 16) & 0x3u;
    if (usesSrc)
        r128SettingUsed(R128Setting::SrcFormat2d, srcType);
    if (usesSrc && srcType == 2u) {
        ++gEng.monoSrc;
        return;
    }
    const bool srcMono = usesSrc && srcType != 3u;
    const bool srcLsbFirst = (dt >> 30) & 1u;
    const u32 srcFrgd = rd(kDpSrcFrgdClr), srcBkgd = rd(kDpSrcBkgdClr);

    // --- pitches, in bytes ---------------------------------------------
    // DST_PITCH/SRC_PITCH bits 9:0 are the pitch in units of EIGHT PIXELS,
    // so the byte stride is pitch*8*(bpp/8) = pitch*bpp. ⚠ 24 bpp is the
    // exception the manual calls out: there the field is programmed in
    // bytes*8, so the stride is pitch*8 and the general formula would be
    // three times too large.
    auto strideOf = [&](u32 v) -> u32 {
        const u32 p = v & 0x3FFu;
        return bpp == 24u ? p * 8u : p * bpp;
    };
    const u32 dstStride = strideOf(rd(kDstPitch));
    const u32 srcStride = srcMono ? rd(kSrcPitch) & 0x3FFu
                                  : strideOf(rd(kSrcPitch));
    if (!dstStride || (usesSrc && !srcStride)) {
        ++gEng.zeroPitch;
        return;
    }

    const u32 dstOff = rd(kDstOffset) & kEngAddrMask;
    const u32 srcOff = rd(kSrcOffset) & kEngAddrMask;
    if (dstOff >= kVramSpan || (usesSrc && srcOff >= kVramSpan)) {
        // The upper half of the engine's address space is AGP: system RAM
        // reached by bus mastering, which this card does not model. Drawing
        // into VRAM instead would corrupt the screen; say so and stop.
        ++gEng.agpTarget;
        return;
    }

    // --- the rectangle --------------------------------------------------
    const u32 dpc = rd(kDpCntl);
    const bool l2r = (dpc & 0x1u) != 0;  // DST_X_DIR: 1 = left to right
    const bool t2b = (dpc & 0x2u) != 0;  // DST_Y_DIR: 1 = top to bottom
    // Only bits 12:0 carry a rectangle extent; 15:13 alias the Bresenham
    // length and belong to trapezoids, which this engine does not draw.
    const int w = static_cast<int>(rd(kDstWidth) & 0x1FFFu);
    const int h = static_cast<int>(rd(kDstHeight) & 0x1FFFu);
    if (w <= 0 || h <= 0)
        return;
    // Running right-to-left or bottom-to-top, the programmed origin is the
    // FAR corner: the rectangle extends back from it.
    const int dx0 = l2r ? sx14(rd(kDstX)) : sx14(rd(kDstX)) + 1 - w;
    const int dy0 = t2b ? sx14(rd(kDstY)) : sx14(rd(kDstY)) + 1 - h;
    const int sx0 = l2r ? sx14(rd(kSrcX)) : sx14(rd(kSrcX)) + 1 - w;
    const int sy0 = t2b ? sx14(rd(kSrcY)) : sx14(rd(kSrcY)) + 1 - h;

    // --- the scissor, inclusive on all four edges -----------------------
    const int scl = sx14(rd(kScLeft)), scr = sx14(rd(kScRight));
    const int sct = sx14(rd(kScTop)), scb = sx14(rd(kScBottom));
    int x0 = dx0 > scl ? dx0 : scl;
    int y0 = dy0 > sct ? dy0 : sct;
    int x1 = (dx0 + w - 1) < scr ? (dx0 + w - 1) : scr;
    int y1 = (dy0 + h - 1) < scb ? (dy0 + h - 1) : scb;
    if (x1 < x0 || y1 < y0 || x0 < 0 || y0 < 0) {
        // Wholly clipped, or clipped to a region that starts off the left or
        // top of the surface — nothing addressable is left.
        ++gEng.clippedOut;
        return;
    }
    const u32 cw = static_cast<u32>(x1 - x0 + 1);
    const u32 ch = static_cast<u32>(y1 - y0 + 1);
    // The source follows the destination's clip, so clipping the top or left
    // of the destination CLIPS the image rather than sliding it.
    const int csx = sx0 + (x0 - dx0);
    const int csy = sy0 + (y0 - dy0);
    if (usesSrc && (csx < 0 || csy < 0)) {
        ++gEng.clippedOut;
        return;
    }

    // --- does it fit in VRAM -------------------------------------------
    auto lastByte = [&](u32 off, u32 stride, int px, int py, u32 rows) {
        return static_cast<u64>(off) +
               static_cast<u64>(py + static_cast<int>(rows) - 1) * stride +
               static_cast<u64>(px) * bypp + bypp;
    };
    // The last byte a one-bit source reads is the one holding the bottom
    // right pixel's bit.
    const u64 srcEnd =
        srcMono ? static_cast<u64>(srcOff) +
                      static_cast<u64>(csy + static_cast<int>(ch) - 1) *
                          srcStride +
                      static_cast<u64>((csx + static_cast<int>(cw) - 1) >> 3) +
                      1u
                : lastByte(srcOff, srcStride, csx, csy, ch);
    if (lastByte(dstOff, dstStride, x0, y0, ch) > vram.size() ||
        (usesSrc && srcEnd > vram.size())) {
        ++gEng.offVram;
        return;
    }

    // The color compare: see ClrCmp. A source compare needs a source, so a
    // fill only ever takes the destination half.
    const u32 cmp = rd(kClrCmpCntl);
    ClrCmp key = clrCmpDecode(cmp, rd(0x15C4u), rd(0x15C8u), rd(kClrCmpMsk),
                              bypp);
    if ((!usesSrc || srcMono) && key.fnSrc) {
        // What "the source color" is for a compare against an expanded bit
        // the guide does not say: such a compare is counted, not applied.
        if (srcMono)
            ++gEng.colorCompare;
        key.fnSrc = 0;
        key.any = key.fnDst == 1u || key.fnDst == 4u || key.fnDst == 5u;
    }
    if (key.unknown)
        ++gEng.colorCompare;

    // A recorder or a replayer is listening for frame edges (opm/r128rec.hpp).
    // An operation on the surface being displayed that covers at least half
    // of it is a frame being presented (a copy onto it, reported once it is
    // drawn) or begun (a fill of it, reported before it wipes the last one).
    bool frameEdge = false;
    if (gR128FrameEdge) {
        const R128Scan shown = r128ScanDecode(*this);
        frameEdge = shown.enabled && shown.rowBytes == dstStride &&
                    shown.offset == dstOff &&
                    static_cast<u64>(cw) * ch * 2u >=
                        static_cast<u64>(shown.w) * shown.h;
        if (frameEdge && !usesSrc)
            gR128FrameEdge(*this, 'C');
    }

    // --- draw -----------------------------------------------------------
    //
    // ⚠ BYTE ORDER. This model's VRAM holds pixels exactly as the guest
    // wrote them through the big-endian aperture, and the scanout in
    // opm_screen reads a 32-bpp pixel as byte0=unused, byte1=R, byte2=G,
    // byte3=B. A fill color arriving in DP_BRUSH_FRGD_CLR is a plain 32-bit
    // number, so it has to be stored MOST SIGNIFICANT BYTE FIRST for the
    // three readers of this memory — the CPU, the scanout and this engine —
    // to agree. Storing it the other way round is exactly the bug that once
    // made a gray desktop come out olive.
    const u32 wmask = rd(kDpWriteMsk);
    // A write mask of zero means "mask nothing" here rather than "write
    // nothing": the reset value is 0 and GMC_WR_MSK_DIS sets it to all ones,
    // so a driver that never programs it must still be able to draw.
    const u32 mask = wmask ? wmask : 0xFFFFFFFFu;
    auto pixAt = [&](size_t o) {
        u32 v = 0;
        for (u32 k = 0; k < bypp; ++k)
            v = (v << 8) | vram[o + k];
        return v;
    };
    auto putPix = [&](size_t o, u32 v) {
        if (mask != 0xFFFFFFFFu)
            v = (pixAt(o) & ~mask) | (v & mask);
        if (r128PixelWatched(o, bypp))
            r128PixelWatchNote(
                "2d blit",
                "rop %02x %s to (%d,%d %dx%d), color compare %08x key %08x, "
                "wrote %x",
                rop, usesSrc ? "from video memory" : "fill", dx0, dy0, w, h,
                cmp, rd(0x15C4u), v);
        for (u32 k = 0; k < bypp; ++k)
            vram[o + k] = static_cast<u8>(v >> (8 * (bypp - 1 - k)));
    };

    const bool plain = mask == 0xFFFFFFFFu;
    if (!usesSrc && !usesDst && !key.any && !patMono) {
        // The result does not depend on anything that varies across the
        // rectangle, so evaluate the op ONCE and fill. This is PATCOPY,
        // BLACKNESS and WHITENESS, and every other op that reduces to a
        // constant — no special cases, they just land here.
        ++gEng.fills;
        const u32 fill = rop3(rop, pat, 0, 0);
        for (u32 r = 0; r < ch; ++r) {
            const size_t d = dstOff + (static_cast<size_t>(y0) + r) * dstStride +
                             static_cast<size_t>(x0) * bypp;
            for (u32 c = 0; c < cw; ++c)
                putPix(d + static_cast<size_t>(c) * bypp, fill);
        }
    } else if (rop == 0xCCu && plain && !key.any && !srcMono) {
        // Straight SRCCOPY with no write mask: whole rows at a time.
        ++gEng.copies;
        for (u32 r = 0; r < ch; ++r) {
            // Overlapping copies: the row ORDER is what makes a downward
            // scroll correct, and memmove makes each row correct however the
            // spans overlap horizontally.
            const u32 rr = t2b ? r : ch - 1u - r;
            const size_t d = dstOff + (static_cast<size_t>(y0) + rr) * dstStride +
                             static_cast<size_t>(x0) * bypp;
            const size_t s = srcOff + (static_cast<size_t>(csy) + rr) * srcStride +
                             static_cast<size_t>(csx) * bypp;
            memmove(&vram[d], &vram[s], static_cast<size_t>(cw) * bypp);
        }
    } else {
        // The general case: evaluate the truth table per pixel.
        ++(usesSrc ? gEng.copies : gEng.fills);
        for (u32 r = 0; r < ch; ++r) {
            const u32 rr = t2b ? r : ch - 1u - r;
            const size_t d = dstOff + (static_cast<size_t>(y0) + rr) * dstStride +
                             static_cast<size_t>(x0) * bypp;
            const size_t s = srcOff + (static_cast<size_t>(csy) + rr) * srcStride +
                             static_cast<size_t>(csx) * bypp;
            // ⚠ Direction matters here for the same reason the row order
            // does. memmove sorts out a horizontal overlap on its own; a hand
            // loop does not, and walking the wrong way across a row that
            // overlaps its own source reads pixels it has already written.
            // DST_X_DIR is the driver saying which way is safe — follow it.
            for (u32 c = 0; c < cw; ++c) {
                const u32 cc = l2r ? c : cw - 1u - c;
                const size_t dp = d + static_cast<size_t>(cc) * bypp;
                u32 S = 0;
                if (srcMono) {
                    const size_t sx = static_cast<size_t>(csx) + cc;
                    const u8 bits = vram[srcOff +
                                         (static_cast<size_t>(csy) + rr) *
                                             srcStride +
                                         (sx >> 3)];
                    const bool on =
                        (bits >> (srcLsbFirst ? sx & 7u : 7u - (sx & 7u))) & 1u;
                    if (!on && srcType == 1u)
                        continue; // "leave alone": no background pixel
                    S = on ? srcFrgd : srcBkgd;
                } else if (usesSrc) {
                    S = pixAt(s + static_cast<size_t>(cc) * bypp);
                }
                const u32 D = (usesDst || key.fnDst) ? pixAt(dp) : 0u;
                if (key.any && !key.draws(S, D))
                    continue; // keyed out: the destination keeps its pixel
                u32 P = pat;
                if (patMono &&
                    !patOn(x0 + static_cast<int>(cc),
                           y0 + static_cast<int>(rr))) {
                    if (patLeave)
                        continue; // "leave alone": no background is drawn
                    P = patBkgd;
                }
                putPix(dp, rop3(rop, P, S, D));
            }
        }
    }
    ++gEng.blits;
    gEng.pixels += static_cast<u64>(cw) * ch;
    if (frameEdge && usesSrc)
        gR128FrameEdge(*this, 'P');
    // Keep the painted span honest: the screen dump and the "did anything
    // reach the framebuffer" report both read it, and a desktop drawn
    // entirely by the engine would otherwise look like a machine that never
    // painted at all.
    const u32 lo = dstOff + static_cast<u32>(y0) * dstStride +
                   static_cast<u32>(x0) * bypp;
    const u32 hi = static_cast<u32>(lastByte(dstOff, dstStride, x1, y1, 1) - 1);
    if (lo < fbLo)
        fbLo = lo;
    if (hi > fbHi)
        fbHi = hi;
}

// Absorb a write into the engine block. Returns true when the write has been
// fully handled and must not also fall through to the plain register store.
// ── Host data through the registers ──────────────────────────────────────
//
// With DP_SRC_SOURCE 3 or 4 a rectangle's pixels come from the processor:
// the store that would have drawn (DST_HEIGHT_WIDTH and its kin) only sets
// the rectangle up, and the pixels follow as DWORDs stored to HOST_DATA0-7,
// the last of them to HOST_DATA_LAST (guide: "indicates the last host data
// register write for operation"). The XFree86 driver's image writes and its
// text (a one-bit source expanded through DP_SRC_FRGD_CLR / _BKGD_CLR) are
// this protocol.
//
// The stream is read as the HOSTDATA_BLT packet's raster is, since that
// packet is this protocol with the DWORDs in a buffer: color pixels are
// bytes taken from each DWORD low byte first, rows running on; a one-bit
// source is bits taken across the DWORDs, from each DWORD's top bit or its
// bottom one as DP_BYTE_PIX_ORDER says. Source 3 is the "linear trajectory":
// a row begins where the last ended. Source 4 is "linear trajectory and
// byte-aligned": each row of a one-bit source begins on a byte.
//
// The DWORDs are kept until the rectangle's worth has arrived (or
// HOST_DATA_LAST says there are no more) and drawn then, so a guest that
// reads video memory in the middle of a stream sees none of it rather than
// part: the one thing here the chip does differently.
void R128Cell::engHostBegin(u32 rop, u32 bypp, bool byteAligned)
{
    const u32 dt = rd(kDpDatatype);
    const u32 srcType = (dt >> 16) & 3u;
    r128SettingUsed(R128Setting::SrcFormat2d, srcType);
    const u32 w = rd(kDstWidth) & 0x1FFFu, h = rd(kDstHeight) & 0x1FFFu;
    gHostOp = HostOp{};
    if (srcType == 2u || bypp == 3u) {
        // A reserved source type; and 24 bpp, whose packing in the stream
        // nothing describes.
        ++gEng.hostData;
        return;
    }
    if (!w || !h)
        return;
    const u32 dpc = rd(kDpCntl);
    gHostOp.active = true;
    gHostOp.byteAligned = byteAligned;
    gHostOp.lsbFirst = (dt >> 30) & 1u;
    // As for any rectangle, the programmed corner is the far one when the
    // direction runs backward.
    gHostOp.x = (dpc & 1u) ? sx14(rd(kDstX))
                           : sx14(rd(kDstX)) + 1 - static_cast<int>(w);
    gHostOp.y = (dpc & 2u) ? sx14(rd(kDstY))
                           : sx14(rd(kDstY)) + 1 - static_cast<int>(h);
    gHostOp.w = w;
    gHostOp.h = h;
    gHostOp.srcType = srcType;
    gHostOp.rop = rop;
    gHostOp.bypp = bypp;
    const u64 bits = srcType == 3u
                         ? static_cast<u64>(w) * h * bypp * 8u
                         : byteAligned ? static_cast<u64>((w + 7u) & ~7u) * h
                                       : static_cast<u64>(w) * h;
    gHostOp.need = (bits + 31u) / 32u;
}

void R128Cell::engHostWord(u32 v, bool last)
{
    HostOp& op = gHostOp;
    if (!op.active)
        return; // a stray word, with no rectangle waiting for it
    op.words.push_back(v);
    if (!last && op.words.size() < op.need)
        return;
    op.active = false;
    const u32 bypp = op.bypp;
    const u32 pitch = rd(kDstPitch) & 0x3FFu;
    const u32 stride = pitch * 8u * bypp;
    const u32 dstOff = rd(kDstOffset) & kEngAddrMask;
    if (!stride) {
        ++gEng.zeroPitch;
        return;
    }
    if (dstOff >= kVramSpan) {
        ++gEng.agpTarget;
        return;
    }
    const int scl = sx14(rd(kScLeft)), scr = sx14(rd(kScRight));
    const int sct = sx14(rd(kScTop)), scb = sx14(rd(kScBottom));
    const u32 wmask = rd(kDpWriteMsk);
    const u32 mask = wmask ? wmask : 0xFFFFFFFFu;
    const u32 pat = rd(kDpBrushFrgdClr);
    const u32 fg = rd(kDpSrcFrgdClr), bk = rd(kDpSrcBkgdClr);
    const bool mono = op.srcType != 3u;
    ClrCmp key = clrCmpDecode(rd(kClrCmpCntl), rd(0x15C4u), rd(0x15C8u),
                              rd(kClrCmpMsk), bypp);
    if (key.unknown || (mono && key.fnSrc))
        ++gEng.colorCompare;
    if (mono) // as in engBlit: not applied across an expanded bit
        key.fnSrc = 0;
    const u64 haveBits = static_cast<u64>(op.words.size()) * 32u;
    const u64 rowBits = mono ? (op.byteAligned ? (op.w + 7u) & ~7u : op.w)
                             : static_cast<u64>(op.w) * bypp * 8u;
    for (u32 yy = 0; yy < op.h; ++yy) {
        for (u32 xx = 0; xx < op.w; ++xx) {
            const u64 bit = yy * rowBits + (mono ? xx : xx * bypp * 8ull);
            if (bit + (mono ? 1u : bypp * 8u) > haveBits)
                break; // the stream ended short of the rectangle
            u32 S = 0;
            if (mono) {
                const u32 word = op.words[bit >> 5];
                const u32 b = static_cast<u32>(bit & 31u);
                const bool on =
                    ((word >> (op.lsbFirst ? b : 31u - b)) & 1u) != 0;
                if (!on && op.srcType == 1u)
                    continue;
                S = on ? fg : bk;
            } else {
                for (u32 k = 0; k < bypp; ++k) {
                    const u64 s = (bit >> 3) + k;
                    S = (S << 8) |
                        ((op.words[s >> 2] >> (8u * (s & 3u))) & 0xFFu);
                }
            }
            const int px = op.x + static_cast<int>(xx);
            const int py = op.y + static_cast<int>(yy);
            if (px < scl || px > scr || py < sct || py > scb || px < 0 ||
                py < 0)
                continue;
            const u64 at = static_cast<u64>(dstOff) +
                           static_cast<u64>(py) * stride +
                           static_cast<u64>(px) * bypp;
            if (at + bypp > vram.size())
                continue;
            u32 D = 0;
            for (u32 k = 0; k < bypp; ++k)
                D = (D << 8) | vram[at + k];
            if (key.any && !key.draws(S, D))
                continue;
            u32 out = rop3(op.rop, pat, S, D);
            if (mask != 0xFFFFFFFFu)
                out = (D & ~mask) | (out & mask);
            if (r128PixelWatched(at, bypp))
                r128PixelWatchNote(
                    "2d host data",
                    "%s pushed through the registers to (%d,%d %ux%u), rop "
                    "%02x, wrote %x",
                    mono ? "1-bit pixels" : "color pixels", op.x, op.y, op.w,
                    op.h, op.rop, out);
            for (u32 k = 0; k < bypp; ++k)
                vram[at + k] = static_cast<u8>(out >> (8u * (bypp - 1u - k)));
            ++gEng.pixels;
        }
    }
    ++gEng.blits;
    ++gEng.copies;
    op.words.clear();
}

// ── Lines ────────────────────────────────────────────────────────────────
//
// The engine draws a line by Bresenham's algorithm from the terms a driver
// computes: the pen starts at DST_X, DST_Y; DP_CNTL has the two directions
// and which axis is the major one; DST_BRES_ERR, _INC and _DEC are the error
// term and its two steps (20 bits, signed); and the store to DST_BRES_LNTH,
// the number of pixels, draws.
//
// The guide names the registers and stops. The stepping rule is the one the
// SDK's own sample (4.3.2, R128_DrawLine) and the XFree86 driver's lines
// both program for: err = -major, inc = 2 * minor, dec = -2 * major, length
// = the pixel count. Those terms draw the line they are meant to only if,
// after each pixel, the engine adds INC and, when the result is not
// negative, steps the minor axis and adds DEC. DP_CNTL's BRES_SIGN says
// which way a zero falls ("0 = zero error term is positive").
//
// The pen is left where the line ended and the error term as it stood, so a
// polyline continues from one segment into the next. The brush must be the
// solid one: the line patterns (8x1, 32x1) have no layout in any source.
// DST_LAST_PEL is not acted on: the XFree86 driver leaves it clear and gets
// every pixel of its count, and nothing says what setting it changes.
static constexpr u32 kDstBresErr = 0x1628, kDstBresInc = 0x162C;
static constexpr u32 kDstBresDec = 0x1630, kDstBresLnth = 0x1634;
static constexpr u32 kDpCntlXdirYdirYmajor = 0x16D0;

void R128Cell::engLine(u32 length)
{
    const u32 dt = rd(kDpDatatype);
    r128SettingUsed(R128Setting::Dst2d, dt & 0xFu);
    u32 bypp = 0;
    switch (dt & 0xFu) {
    case 2: case 7: case 8: case 9: bypp = 1; break;
    case 3: case 4: bypp = 2; break;
    case 5: bypp = 3; break;
    case 6: bypp = 4; break;
    default: break;
    }
    if (!bypp) {
        ++gEng.badBpp;
        return;
    }
    const u32 rop = (rd(kDpMix) >> 16) & 0xFFu;
    const u32 brush = (dt >> 8) & 0xFu;
    if (ropUsesPat(rop))
        r128SettingUsed(R128Setting::Brush2d, brush);
    // A line has no source rectangle, and only the solid brush is known.
    if (ropUsesSrc(rop) ||
        (ropUsesPat(rop) && brush != 13u && brush != 15u)) {
        ++gEng.lineDeclined;
        return;
    }
    const u32 pitch = rd(kDstPitch) & 0x3FFu;
    const u32 stride = bypp == 3u ? pitch * 8u : pitch * 8u * bypp;
    const u32 dstOff = rd(kDstOffset) & kEngAddrMask;
    if (!stride) {
        ++gEng.zeroPitch;
        return;
    }
    if (dstOff >= kVramSpan) {
        ++gEng.agpTarget;
        return;
    }
    const auto s20 = [](u32 v) {
        return static_cast<int>(v & 0xFFFFFu) -
               ((v & 0x80000u) ? 0x100000 : 0);
    };
    int err = s20(rd(kDstBresErr));
    const int inc = s20(rd(kDstBresInc)), dec = s20(rd(kDstBresDec));
    const u32 dpc = rd(kDpCntl);
    const int stepX = (dpc & 1u) ? 1 : -1, stepY = (dpc & 2u) ? 1 : -1;
    const bool yMajor = (dpc & 4u) != 0;
    const bool zeroSteps = !(dpc & 0x100u); // BRES_SIGN clear: zero is positive
    const int scl = sx14(rd(kScLeft)), scr = sx14(rd(kScRight));
    const int sct = sx14(rd(kScTop)), scb = sx14(rd(kScBottom));
    const u32 wmask = rd(kDpWriteMsk);
    const u32 mask = wmask ? wmask : 0xFFFFFFFFu; // as engBlit: zero masks nothing
    const u32 pat = rd(kDpBrushFrgdClr);
    int x = sx14(rd(kDstX)), y = sx14(rd(kDstY));
    for (u32 k = 0; k < length; ++k) {
        const u64 at = static_cast<u64>(dstOff) +
                       static_cast<u64>(y < 0 ? 0 : y) * stride +
                       static_cast<u64>(x < 0 ? 0 : x) * bypp;
        if (x >= scl && x <= scr && y >= sct && y <= scb && x >= 0 && y >= 0 &&
            at + bypp <= vram.size()) {
            u32 d = 0;
            for (u32 b = 0; b < bypp; ++b)
                d = (d << 8) | vram[at + b];
            u32 v = rop3(rop, pat, 0, d);
            if (mask != 0xFFFFFFFFu)
                v = (d & ~mask) | (v & mask);
            if (r128PixelWatched(at, bypp))
                r128PixelWatchNote("2d line", "rop %02x at (%d,%d), wrote %x",
                                   rop, x, y, v);
            for (u32 b = 0; b < bypp; ++b)
                vram[at + b] = static_cast<u8>(v >> (8u * (bypp - 1u - b)));
            ++gEng.linePixels;
            const u32 lo = static_cast<u32>(at);
            if (lo < fbLo)
                fbLo = lo;
            if (lo + bypp - 1u > fbHi)
                fbHi = lo + bypp - 1u;
        }
        err += inc;
        if (err > 0 || (err == 0 && zeroSteps)) {
            (yMajor ? x : y) += yMajor ? stepX : stepY;
            err += dec;
        }
        (yMajor ? y : x) += yMajor ? stepY : stepX;
    }
    regs_[kDstX] = static_cast<u32>(x) & 0x3FFFu;
    regs_[kDstY] = static_cast<u32>(y) & 0x3FFFu;
    regs_[kDstBresErr] = static_cast<u32>(err) & 0xFFFFFu;
    ++gEng.lines2d;
}

bool R128Cell::engWrite(u32 off, u32 v)
{
    // The 3D unit's registers first: the fog-table pair and the write-gated
    // context block live with the rasterizer in r128_3d.cpp.
    if (off >= 0x1A00u && r128Eng3dWrite(*this, off, v))
        return true;
    switch (off) {
    // --- composite registers: normalize into the canonical pair ---------
    //
    // Every one of these is marked [W] in the reference guide — they are
    // write-only aliases, so nothing is lost by decomposing them, and
    // decomposing is what lets the blit read ONE canonical set.
    case kDstPitchOffset:
    case kDstPitchOffsetC: // the CCE context name for the same state — the
                           // DRI's 3D path programs the destination through
                           // this offset and the 2D path through 0x142C;
                           // one engine, one canonical pair.
        regs_[kDstOffset] = (v & 0x1FFFFFu) << 5;
        regs_[kDstPitch] = (v >> 21) & 0x3FFu;
        return true;
    case kSrcPitchOffset:
        regs_[kSrcOffset] = (v & 0x1FFFFFu) << 5;
        regs_[kSrcPitch] = (v >> 21) & 0x3FFu;
        return true;
    case kSrcYX:
        regs_[kSrcX] = v & 0x3FFFu;
        regs_[kSrcY] = (v >> 16) & 0x3FFFu;
        return true;
    case kDstYX:
        regs_[kDstX] = v & 0x3FFFu;
        regs_[kDstY] = (v >> 16) & 0x3FFFu;
        return true;
    case kSrcXY:
        regs_[kSrcY] = v & 0x3FFFu;
        regs_[kSrcX] = (v >> 16) & 0x3FFFu;
        return true;
    case kDstXY:
        regs_[kDstY] = v & 0x3FFFu;
        regs_[kDstX] = (v >> 16) & 0x3FFFu;
        return true;
    case kDstHeightY:
        regs_[kDstY] = v & 0x3FFFu;
        regs_[kDstHeight] = (v >> 16) & 0x3FFFu;
        return true;
    case kScTopLeft:
    case kScTopLeftC: // "Aliased to SC_TOP_LEFT"
        regs_[kScLeft] = v & 0x3FFFu;
        regs_[kScTop] = (v >> 16) & 0x3FFFu;
        return true;
    case kScBottomRight:
    case kScBottomRightC: // "Aliased to SC_BOTTOM_RIGHT"
        regs_[kScRight] = v & 0x3FFFu;
        regs_[kScBottom] = (v >> 16) & 0x3FFFu;
        return true;
    case kSrcScBottomRight:
        regs_[kSrcScRight] = v & 0x3FFFu;
        regs_[kSrcScBottom] = (v >> 16) & 0x3FFFu;
        return true;

    // --- the master control, and its side effects -----------------------
    case kDpGuiMasterCntlC: // CCE context alias: same control, same effects
    case kDpGuiMasterCntl: {
        r128SettingUsed(R128Setting::Gmc3dFn, (v >> 27) & 1u);
        regs_[kDpGuiMasterCntl] = v;
        // The GMC word CARRIES the datapath fields; writing it is how a
        // driver sets datatype and mix in one store. Field positions are the
        // reference guide's: brush 7:4, dst 11:8, src 13:12, byte-pixel
        // order 14, ROP3 23:16, source select 26:24.
        regs_[kDpDatatype] = ((v & 0x0F00u) >> 8) |    // dst  -> 3:0
                             ((v & 0x30F0u) << 4) |    // brush -> 11:8, src -> 17:16
                             ((v & 0x4000u) << 16);    // byte order -> 30
        regs_[kDpMix] = (v & 0x00FF0000u) |            // ROP3 stays at 23:16
                        ((v & 0x07000000u) >> 16);     // source -> 10:8
        // ⭐ A GMC WRITE RESETS THE DIRECTION. The reference guide says so
        // twice, under DP_CNTL's DST_X_DIR and DST_Y_DIR: "This bit is set
        // to '1' by a GUI_MASTER_CNTL write." QEMU does not model it, and
        // without it a blit that follows a reversed-direction blit would
        // inherit a direction the hardware had already cleared — which shows
        // up as one torn rectangle after a scroll and nothing else.
        regs_[kDpCntl] = rd(kDpCntl) | 0x3u;
        if (!(v & 0x1u)) { // GMC_SRC_PITCH_OFFSET_CNTL
            regs_[kSrcOffset] = rd(kDefaultOffset);
            regs_[kSrcPitch] = rd(kDefaultPitch);
        }
        if (!(v & 0x2u)) { // GMC_DST_PITCH_OFFSET_CNTL
            regs_[kDstOffset] = rd(kDefaultOffset);
            regs_[kDstPitch] = rd(kDefaultPitch);
        }
        if (!(v & 0x4u)) { // GMC_SRC_CLIPPING
            regs_[kSrcScRight] = rd(kDefaultScBr) & 0x3FFFu;
            regs_[kSrcScBottom] = (rd(kDefaultScBr) >> 16) & 0x3FFFu;
        }
        if (!(v & 0x8u)) { // GMC_DST_CLIPPING
            regs_[kScLeft] = 0;
            regs_[kScTop] = 0;
            regs_[kScRight] = rd(kDefaultScBr) & 0x3FFFu;
            regs_[kScBottom] = (rd(kDefaultScBr) >> 16) & 0x3FFFu;
        }
        if (v & 0x10000000u) // GMC_CLR_CMP_CNTL_DIS
            regs_[kClrCmpCntl] = 0;
        if (v & 0x20000000u) // GMC_AUX_CLIP_DIS: clear every AUXn_SC_ENB,
            regs_[kAuxScCntl] = rd(kAuxScCntl) & ~0x15u; // bits 0, 2 and 4
        if (v & 0x40000000u) { // GMC_WR_MSK_DIS
            regs_[kDpWriteMsk] = 0xFFFFFFFFu;
            regs_[kClrCmpMsk] = 0xFFFFFFFFu;
        }
        return true;
    }

    // --- the initiators -------------------------------------------------
    //
    // "Trajectory registers ... set up the source and destination
    // trajectories and initiate draw operations" (§2.1.5). The write that
    // supplies the WIDTH is the one that starts the engine; DST_WIDTH_BW is
    // the only one the guide names outright ("this is an initiator
    // register"), and QEMU's oracle supplies the rest of the set.
    case kDstWidth:
        regs_[kDstWidth] = v & 0x3FFFu;
        engBlit();
        return true;
    case kDstWidthBw:
        regs_[kDstWidth] = v & 0x3FFFu;
        engBlit();
        return true;
    case kDstHeightWidth:
        regs_[kDstWidth] = v & 0x3FFFu;
        regs_[kDstHeight] = (v >> 16) & 0x3FFFu;
        engBlit();
        return true;
    case kDstWidthHeight:
        regs_[kDstHeight] = v & 0x3FFFu;
        regs_[kDstWidth] = (v >> 16) & 0x3FFFu;
        engBlit();
        return true;
    case kDstWidthX:
        regs_[kDstX] = v & 0x3FFFu;
        regs_[kDstWidth] = (v >> 16) & 0x3FFFu;
        engBlit();
        return true;
    case kDstWidthXIncy:
        regs_[kDstX] = v & 0x3FFFu;
        regs_[kDstWidth] = (v >> 16) & 0x3FFFu;
        engBlit();
        // ...and then step down one scanline, which is what the INCY in the
        // name is for: it draws a run of single rows without reprogramming Y.
        regs_[kDstY] = (rd(kDstY) + 1u) & 0x3FFFu;
        return true;

    // --- host data --------------------------------------------------------
    case 0x17C0u: case 0x17C4u: case 0x17C8u: case 0x17CCu:
    case 0x17D0u: case 0x17D4u: case 0x17D8u: case 0x17DCu:
        engHostWord(v, false);
        return true;
    case 0x17E0u: // HOST_DATA_LAST
        engHostWord(v, true);
        return true;

    // --- lines ------------------------------------------------------------
    case kDstBresLnth:
        engLine(v & 0x3FFFu);
        return true;
    case kDpCntlXdirYdirYmajor:
        // The three line bits of DP_CNTL under another address: X direction
        // in bit 31, Y direction in bit 15, the major axis in bit 2.
        regs_[kDpCntlXdirYdirYmajor] = v;
        regs_[kDpCntl] = (rd(kDpCntl) & ~7u) | (v >> 31) |
                         (((v >> 15) & 1u) << 1) | (v & 4u);
        return true;

    // --- the pixel cache -------------------------------------------------
    case kPcGuiCtlstat:
        // ⭐ THESE ARE SELF-CLEARING COMMAND PULSES, NOT STORAGE. ATI's own
        // wording, on every one of bits 0..9: "Once a bit in the field is set,
        // it will remain set until the flush is complete, then the Pixel Cache
        // will automatically clear it."
        //
        // The accelerator writes 0x000000FF here — `R128_PC_FLUSH_ALL`, flush
        // and read-invalidate both the GUI and non-GUI sides — and then reads
        // the register back. Storing the value and answering it back forever
        // reports a cache flush that NEVER COMPLETES, and it does it silently,
        // because the offset is inside an aperture this card claims.
        //
        // This model has no pixel cache to flush and every blit is already
        // resident in VRAM by the time the triggering store returns, so the
        // flush is complete the instant it is asked for. Answer accordingly:
        // command bits clear, and the read-only status bits 24..31 (PC_DIRTY,
        // PC_BUSY_FLUSH, PC_BUSY_GUI, PC_BUSY) all idle. That is the whole
        // register, so it reads back zero.
        ++gEng.cacheFlushes;
        regs_[kPcGuiCtlstat] = 0;
        return true;

    // --- stalls ----------------------------------------------------------
    case kWaitUntil:
        // Every WAIT_UNTIL event asks the command FIFO to stall until some
        // engine reaches a milestone. This engine completes each operation
        // inside the store that triggered it, so every milestone is already
        // behind us and the stall is a no-op — which is the honest answer
        // here, not a shortcut. Counted, because a driver leaning on
        // EVENT_CRTC_OFFSET for page flips would need the CRTC side too.
        ++gEng.waitUntil;
        return true;
    default:
        break;
    }
    return false;
}

void R128Cell::note(u32 off, u32 val, bool wr)
{
    // The card's own FCode bring-up issues thousands of accesses around
    // 330M and fills the log, so by the time the OS driver touches a
    // register at 3.9G there is no room left and the accesses that matter
    // are invisible. logFrom opens the window late; the first-touch set is
    // cleared as it opens so the OS era gets its own first touches rather
    // than inheriting the firmware's.
    const u64 at = stamp ? *stamp : 0;
    if (at < logFrom)
        return;
    if (logFrom && !gateOpened_) {
        gateOpened_ = true;
        seen_.clear();
        log.clear();
    }
    if (!wr) {
        if (seen_.count(off))
            return;
        seen_[off] = 1;
    }
    if (log.size() < 4096)
        log.push_back({at, off, val, pcRef ? *pcRef : 0, wr});
}

u32 R128Cell::read(u32 off, u32 len)
{
    readCount[off & ~3u]++;
    const u32 native = regRead(off >> 2);
    note(off & ~3u, native, false);
    noteEngine(off & ~3u, native, pcRef ? *pcRef : 0, stamp ? *stamp : 0,
               false);
    if (len == 4)
        return swap32(native);
    u32 r = 0;
    for (u32 k = 0; k < len; ++k)
        r = (r << 8) | ((native >> (8 * ((off + k) & 3u))) & 0xFFu);
    return r;
}

void R128Cell::write(u32 off, u32 v, u32 len)
{
    writeCount[off & ~3u]++;
    u32 native;
    if (len == 4)
        native = swap32(v);
    else {
        native = regRead(off >> 2);
        for (u32 k = 0; k < len; ++k) {
            const u32 lane = (off + k) & 3u;
            native = (native & ~(0xFFu << (8 * lane))) |
                     (((v >> (8 * (len - 1 - k))) & 0xFFu) << (8 * lane));
        }
    }
    gRegEverSet[(off >> 2) & 0xFFFu] |= native;
    const u64 at = stamp ? *stamp : 0;
    note(off & ~3u, native, true);
    noteEngine(off & ~3u, native, pcRef ? *pcRef : 0, at, true);
    const u32 aligned = off & ~3u;
    // The GUI block first: a write there may be an alias to normalize or the
    // initiator of a draw, and in either case it must not also land in the
    // plain register store under its own offset.
    if (gEng2dOn && aligned >= 0x1400u && aligned < 0x2000u &&
        engWrite(aligned, native))
        return;
    if (aligned == kClockCntlIndex) {
        pllAddr_ = native & 0xFFu;
        // write-enable bit 7: data writes land in the PLL file
    } else if (aligned == kGpioA0 || aligned == kGpioA8) {
        // Every write to either line register can move the bus, so step the
        // DDC slave from both.
        if (aligned == kGpioA0)
            gpioSda_ = native; // holds levels when kLevelIsA0
        else
            gpioScl_ = native; // holds enables when kLevelIsA0
        regs_[aligned] = native;
        const u32 lvl2 = kLevelIsA0 ? gpioSda_ : gpioScl_;
        const u32 en2 = kLevelIsA0 ? gpioScl_ : gpioSda_;
        // Gated with the register log, and for the same reason: the FCode
        // fills 160 states long before the OS driver says anything.
        if (at >= logFrom && ddcWave.size() < 160)
            ddcWave.push_back({gpioSda_, gpioScl_});
        // Pass the MASTER-driven SDA. Feeding the combined bus level in
        // made our own slave generate edges: asserting the acknowledge
        // pulls SDA low, and with SCL high the detector read that as a
        // START and reset the shift register mid-byte. Start and stop are
        // defined by what the master does; the slave.s pull-down only
        // matters when a data bit is sampled.
        ddcStep(!(en2 & kDdcSclBit) || (lvl2 & kDdcSclBit),
                !(en2 & kDdcSdaBit) || (lvl2 & kDdcSdaBit));
    } else if (aligned == kClockCntlData) {
        pll_[pllAddr_ & 0x3Fu] = native;
    } else if (aligned == 0x00B0u) { // PALETTE_INDEX
        palIdx_ = native & 0xFFu;
    } else if (aligned == 0x00B4u) { // PALETTE_DATA, auto-increment
        pal_[palIdx_ & 0xFFu] = native & 0x00FFFFFFu;
        palIdx_ = (palIdx_ + 1u) & 0xFFu;
    } else if (aligned == kGenIntCntl && vblEnabled) {
        // Arming CRTC_VBLANK_INT starts the blank clock; disarming parks it.
        // The first blank is scheduled a WHOLE period out rather than fired
        // from inside the store: a device event delivered before the driver
        // has finished the write that asked for it is as broken as one that
        // never arrives, which is the lesson the root-hub connect taught
        // three times over.
        const bool arm = (native & kCrtcVblankInt) != 0;
        if (arm && !vblNextTb_)
            vblNextTb_ = tbNow_ + vblPeriod();
        else if (!arm)
            vblNextTb_ = 0;
    } else if (aligned == kGenIntStatus && vblEnabled) {
        // WRITE-1-TO-CLEAR, and only over the bits this part latches. The
        // acknowledge is what drops the pin, so it has to reach the stored
        // register rather than overwrite it: fall through to the plain store
        // below and the driver's first ack would LATCH every bit it wrote.
        auto it = regs_.find(kGenIntStatus);
        const u32 cur = it != regs_.end() ? it->second : 0;
        const u32 cleared = cur & ~(native & kGenIntAckMask);
        if (cleared != cur)
            ++vblAcks;
        regs_[kGenIntStatus] = cleared;
        if (gVblTrace > 0) {
            --gVblTrace;
            printf("VBL ack    wrote=%08x cur=%08x -> %08x %s pc=%08x @%llu\n",
                   native, cur, cleared,
                   cleared != cur ? "CLEARED" : "no-op", pcRef ? *pcRef : 0,
                   static_cast<unsigned long long>(at));
            fflush(stdout);
        }
        return;
    }
    // Mode changes, called out by name. The register log is a 4096-entry
    // ring and the OS driver bit-bangs DDC through the GPIO registers
    // thousands of times, so the one write that decides the pixel format
    // is always trimmed away — and getting the format wrong is
    // indistinguishable from the guest painting garbage. Print the CRTC
    // registers whenever they CHANGE, with the WIDTH of the access that
    // did it: a read-modify-write through the wrong lane is exactly how a
    // depth field ends up holding two depths at once (2 | 4 = 6).
    if ((aligned == kCrtcGenCntl || aligned == 0x0224u ||
         aligned == 0x022Cu || aligned == 0x0200u || aligned == 0x0208u) &&
        regs_[aligned] != native && crtcShown_ < 48) {
        ++crtcShown_;
        printf("-- ati crtc %s +%04x %08x -> %08x (len %u) @%llu pc=%08x\n",
               aligned == kCrtcGenCntl ? "GEN_CNTL"
               : aligned == 0x0224u    ? "OFFSET  "
               : aligned == 0x022Cu    ? "PITCH   "
               : aligned == 0x0200u    ? "H_TOTAL "
                                       : "V_TOTAL ",
               aligned, regs_[aligned], native, len,
               static_cast<unsigned long long>(at), pcRef ? *pcRef : 0);
        fflush(stdout);
    }
    regs_[aligned] = native;
    (void)kMmIndex;
    (void)kBiosScratch;
    (void)kGenReset;
    (void)kMemCntl;
    (void)kCrtcGenCntl;
    (void)kPllTest;
}

void R128Cell::setVblTbPeriod(u64 n) { gVblTbPeriod = n; }
u64 R128Cell::vblTbPeriod() { return gVblTbPeriod; }
void R128Cell::setVblTrace(int n) { gVblTrace = n; }
u64 R128Cell::vblDropped() { return gVblDropped; }
u64 R128Cell::vblPeriodEffective()
{
    return gVblTbPeriod ? gVblTbPeriod : kTbPerVblank;
}

u64 R128Cell::vblPeriod() const
{
    return gVblTbPeriod ? gVblTbPeriod : kTbPerVblank;
}

// The earliest timebase at which tick() could change anything: the blank
// itself, or the point at which an unacknowledged one expires. Deliberately
// mirrors tick()'s own two conditions including the status-bit test — leaving
// the test out would return a deadline already in the past forever once a
// blank had expired, and the machine loop would stop gating at all.
u64 R128Cell::nextTickTb() const
{
    if (!vblEnabled || !vblNextTb_)
        return ~0ull;
    const u64 per = vblPeriod();
    u64 due = vblNextTb_;
    const auto st = regs_.find(kGenIntStatus);
    if (st != regs_.end() && (st->second & kCrtcVblankInt) && vblNextTb_ >= per) {
        const u64 expiry = (vblNextTb_ - per) + per / 4u + 1u; // tick tests >
        if (expiry < due)
            due = expiry;
    }
    return due;
}

void R128Cell::tick(u64 tb)
{
    tbNow_ = tb;
    if (!vblEnabled || !vblNextTb_)
        return;
    // A REQUEST NOBODY SERVICED HAS TO EXPIRE. Real silicon holds the latch
    // until the driver acknowledges, and that is fine while a driver is
    // listening — but there are long stretches where none is: Open Firmware
    // arms the interrupt during its display bring-up, and a snapshot can carry a
    // latched blank across a resume. With the line held forever the OS spins
    // iack -> handler -> eoi -> iack about every 5,000 instructions and never
    // makes progress, because `cpuLine()` re-offers the same level the instant
    // the previous one retires. Expiring an unanswered request turns that total
    // paralysis into a dropped frame, which is the right failure mode; when a
    // driver IS listening it acknowledges in ~6,100 instructions, four orders of
    // magnitude inside this window, so serviced behavior is unchanged.
    const u64 per = vblPeriod();
    auto st = regs_.find(kGenIntStatus);
    if (st != regs_.end() && (st->second & kCrtcVblankInt) &&
        vblNextTb_ >= per && tb > (vblNextTb_ - per) + per / 4) {
        st->second &= ~kCrtcVblankInt;
        ++gVblDropped;
    }
    if (tb < vblNextTb_)
        return;
    // Re-base rather than replay. A resume, or an arm that landed before this
    // cell had ever been handed the timebase, leaves the due time billions of
    // ticks in the past; firing one blank per missed period would be a burst
    // that models nothing and would bury the driver in interrupts.
    vblNextTb_ = tb + vblPeriod();
    ++vblanks;
    auto ic = regs_.find(kGenIntCntl);
    if (ic == regs_.end() || !(ic->second & kCrtcVblankInt))
        return; // the panel still retraces; nobody asked to hear about it
    ++vblIrqs;
    regs_[kGenIntStatus] |= kCrtcVblankInt;
    if (gVblTrace > 0) {
        --gVblTrace;
        printf("VBL latch  status=%08x cntl=%08x @%llu\n", regs_[kGenIntStatus],
               regs_[kGenIntCntl],
               static_cast<unsigned long long>(stamp ? *stamp : 0));
        fflush(stdout);
    }
}

bool R128Cell::irqLine() const
{
    if (!vblEnabled)
        return false;
    auto ic = regs_.find(kGenIntCntl);
    auto is = regs_.find(kGenIntStatus);
    const u32 en = ic != regs_.end() ? ic->second : 0;
    const u32 st = is != regs_.end() ? is->second : 0;
    // A level, held while an ENABLED source stands latched. The driver's
    // write-1-to-clear acknowledge is what releases it.
    return (en & st & kGenIntAckMask) != 0;
}


// A 128-byte EDID for a plain multiscan monitor that can do 640x480 and
// 800x600 at 60 Hz. Generated rather than stored: the fields a display
// driver reads are few, and a table of magic bytes teaches nobody which
// ones matter.
u8 R128Cell::edidByte(u8 at) const
{
    switch (at) {
    case 0: case 7: return 0x00;
    case 1: case 2: case 3: case 4: case 5: case 6: return 0xFF; // header
    case 8: return 0x06; case 9: return 0x10;   // manufacturer "APP"
    case 10: return 0x01; case 11: return 0x9D; // product
    case 16: return 0x0A; case 17: return 0x0B; // week 10, year 2001
    case 18: return 0x01; case 19: return 0x03; // EDID 1.3
    case 20: return 0x0E;                       // analog input
    case 21: return 0x22; case 22: return 0x1B; // 34 cm x 27 cm
    case 23: return 0x78;                       // gamma 2.2
    case 24: return 0x0D;                       // RGB, preferred timing
    case 35: return 0x21;  // established: 640x480@60, 800x600@60
    // The mode ceiling used to be HERE, not in the card: the ndrv carries
    // 49 modes up to 1920x1440 but offers only what the monitor claims,
    // and this EDID claimed two. Established II + the standard-timing
    // block below raise the claim; the boot default stays 640x480 (the
    // established/preferred story is unchanged) so every boot gate holds.
    case 36: return 0x08;  // established II: 1024x768@60
    case 37: return 0x80;  // manufacturer band: 1152x870@75 (Apple 21")
    // Standard timings: (X/8 - 31), then aspect<<6 | (refresh - 60).
    case 38: return 0x61; case 39: return 0x40; // 1024x768@60  (4:3)
    case 40: return 0x81; case 41: return 0x40; // 1280x960@60  (4:3)
    case 42: return 0x81; case 43: return 0x80; // 1280x1024@60 (5:4)
    case 44: return 0xC9; case 45: return 0x40; // 1600x1200@60 (4:3)
    case 46: return 0xD1; case 47: return 0xC0; // 1920x1080@60 (16:9)
    case 48: return 0xD1; case 49: return 0x40; // 1920x1440@60 (4:3)
    case 50: case 51: case 52: case 53: return 0x01; // unused slots
    case 54: return 0x31; case 55: return 0x40; // pixel clock 16.4 MHz
    case 56: return 0x80; case 57: return 0xE0; // 640 x 480 active
    case 58: return 0x00; case 59: return 0x18;
    case 60: return 0x10; case 61: return 0x20;
    default: break;
    }
    if (at == 127) {
        // Checksum: the whole 128 bytes must sum to zero mod 256.
        u32 s = 0;
        for (u32 k = 0; k < 127; ++k)
            s += edidByte(static_cast<u8>(k));
        return static_cast<u8>(0x100u - (s & 0xFFu));
    }
    return 0x00;
}

// One step of the DDC I2C bus, called whenever either line could have
// moved. START is SDA falling while SCL is high, STOP is SDA rising while
// SCL is high, and every other bit is sampled on SCL's rising edge.
void R128Cell::ddcStep(bool scl, bool sda)
{
    if (ddc_.lastScl && scl && sda != ddc_.lastSda) {
        if (!sda) { // START
            ++ddcStarts;
            ddc_.state = 1;
            ddc_.shift = 0;
            ddc_.bits = 0;
            ddc_.sdaOut = true;
        } else { // STOP
            ddc_.state = 0;
            ddc_.sdaOut = true;
        }
        ddc_.lastSda = sda;
        return;
    }
    if (scl && !ddc_.lastScl) { // rising edge: sample
        switch (ddc_.state) {
        case 1: // address byte
            ddc_.shift = static_cast<u8>((ddc_.shift << 1) | ((sda && ddcSda()) ? 1 : 0));
            if (++ddc_.bits == 8) {
                ddc_.addr = ddc_.shift;
                ddcLastAddr = ddc_.addr;
                ddc_.bits = 0;
                // 0xA0/0xA1 is the EDID slave at 7-bit address 0x50.
                ddc_.state = (ddc_.addr & 0xFEu) == 0xA0u ? 2 : 0;
                if (ddc_.state == 2)
                    ++ddcMatches;
            }
            break;
        case 3: // data byte from the host (the EDID offset)
            ddc_.shift = static_cast<u8>((ddc_.shift << 1) | ((sda && ddcSda()) ? 1 : 0));
            if (++ddc_.bits == 8) {
                ddc_.ptr = ddc_.shift;
                ddc_.bits = 0;
                ddc_.state = 4;
            }
            break;
        // The acknowledge bit occupies a full clock of its own. Driving it
        // on the falling edge and moving straight on made the very next
        // rising edge — the edge that samples the ACK — get counted as the
        // first bit of the following byte, so every byte after the address
        // was shifted by one. Apple's own master (ddc2-send-byte in the
        // boot ROM, ff92c990) shifts eight bits and then polls the ACK in
        // a retry loop, so the slave has to hold SDA low across that whole
        // clock and consume its rising edge.
        case 7: // address ACK sampled: now the data phase begins
            ddc_.state = (ddc_.addr & 1u) ? 5 : 3;
            ddc_.shift = (ddc_.addr & 1u) ? edidByte(ddc_.ptr) : 0;
            ddc_.bits = 0;
            break;
        case 8: // offset-byte ACK sampled
            ddc_.state = 3;
            ddc_.bits = 0;
            break;
        case 5: // data byte to the host
            if (++ddc_.bits == 8) {
                ddc_.bits = 0;
                ddc_.state = 6;
                ++ddcBytes;
                ++ddc_.ptr;
            }
            break;
        default: break;
        }
    } else if (!scl && ddc_.lastScl) { // falling edge: drive
        switch (ddc_.state) {
        case 2: // acknowledge the address — the ACK owns a whole clock
            ddc_.sdaOut = false;
            ddc_.state = 7; // eat the rising edge that samples it
            break;
        case 4: // acknowledge the offset byte, likewise
            ddc_.sdaOut = false;
            ddc_.state = 8;
            break;
        case 5: // present the next bit, MSB first
            ddc_.sdaOut = (ddc_.shift & (0x80u >> ddc_.bits)) != 0;
            break;
        case 6: // release for the host's acknowledge
            ddc_.sdaOut = true;
            ddc_.state = 5;
            ddc_.shift = edidByte(ddc_.ptr);
            ddc_.bits = 0;
            break;
        default:
            ddc_.sdaOut = true;
            break;
        }
    }
    ddc_.lastScl = scl;
    ddc_.lastSda = sda;
}

// --- The scan-out (see the header for why this is the ONE copy) ------------

// A tube of gamma 2.6 shown on an sRGB screen: entry t is
// round(255 * sRGB((t / 255) ^ 2.6)), sRGB being IEC 61966-2-1's encoding
// (12.92 L below 0.0031308, else 1.055 L^(1/2.4) - 0.055). Kept as numbers,
// not computed: the picture is then the same on every host's maths library.
static const u8 kMonitorCurve[256] = {
      0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   2,   2,
      2,   3,   3,   4,   4,   5,   6,   6,   7,   8,   9,  10,  11,  12,  12,  13,
     14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,
     30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,
     46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  61,  62,
     63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,
     79,  80,  81,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,
     96,  97,  98, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
    113, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 128, 129, 130,
    131, 132, 133, 134, 135, 136, 137, 138, 140, 141, 142, 143, 144, 145, 146, 147,
    148, 149, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 162, 163, 164, 165,
    166, 167, 168, 169, 170, 172, 173, 174, 175, 176, 177, 178, 179, 180, 182, 183,
    184, 185, 186, 187, 188, 189, 191, 192, 193, 194, 195, 196, 197, 198, 199, 201,
    202, 203, 204, 205, 206, 207, 208, 210, 211, 212, 213, 214, 215, 216, 217, 219,
    220, 221, 222, 223, 224, 225, 227, 228, 229, 230, 231, 232, 233, 234, 236, 237,
    238, 239, 240, 241, 242, 244, 245, 246, 247, 248, 249, 250, 252, 253, 254, 255,
};
const u8* r128MonitorCurve() { return kMonitorCurve; }

R128Scan r128ScanDecode(const R128Cell& c)
{
    R128Scan s;
    const u32 gen = c.peek(0x0050);
    s.enabled = (gen & 0x02000000u) != 0;
    s.fmt = (gen >> 8) & 0xFu;
    s.w = (((c.peek(0x0200) >> 16) & 0x3FFu) + 1u) * 8u;
    s.h = ((c.peek(0x0208) >> 16) & 0xFFFu) + 1u;
    s.pitch8 = c.peek(0x022C) & 0xFFFFu;
    s.offset = c.peek(0x0224);
    s.tiled = (c.peek(0x0228) & (1u << 15)) != 0; // CRTC_OFFSET_CNTL TILE_EN
    // CRTC_PIX_WIDTH (RRG-G04500 §CRTC_GEN_CNTL): 2 = 8 bpp CLUT, 3 = 15 bpp
    // ARGB1555, 4 = 16 bpp RGB565, 5 = 24 bpp RGB888, 6 = 32 bpp xRGB. The
    // others (4 bpp planar and friends) have never been seen from this
    // guest; bypp = 0 keeps them a visible refusal rather than a garbled
    // picture.
    switch (s.fmt) {
    case 2u: s.bypp = 1; break;
    case 3u: case 4u: s.bypp = 2; break;
    case 5u: s.bypp = 3; break;
    case 6u: s.bypp = 4; break;
    default: s.bypp = 0; break;
    }
    s.rowBytes = s.pitch8 * 8u * s.bypp;
    for (u32 i = 0; i < 256u; ++i) {
        const u32 e = c.pal(i);
        s.lut[0][i] = kMonitorCurve[(e >> 16) & 0xFFu];
        s.lut[1][i] = kMonitorCurve[(e >> 8) & 0xFFu];
        s.lut[2][i] = kMonitorCurve[e & 0xFFu];
    }
    return s;
}

const char* r128ScanFmtName(u32 fmt)
{
    switch (fmt) {
    case 2u: return "8bpp CLUT";
    case 3u: return "15bpp 1555";
    case 4u: return "16bpp 565";
    case 5u: return "24bpp 888";
    case 6u: return "32bpp xRGB";
    default: return "UNSCANNABLE";
    }
}

void r128ScanRow(const R128Cell& c, const R128Scan& s, u32 y, u8* out)
{
    const auto& vr = c.vram;
    const size_t row = s.offset + size_t(y) * s.rowBytes;
    for (u32 x = 0; x < s.w; ++x) {
        u8 b = 0, g = 0, r = 0;
        const size_t o = row + size_t(x) * s.bypp;
        if (s.bypp && o + s.bypp <= vr.size()) {
            switch (s.fmt) {
            // In every format the three values are TABLE INDICES, looked up
            // once the switch is done (see "The DAC's table" in the header):
            // the pixel itself in 8 bpp, a channel with zeros below it in
            // the narrow formats.
            case 2u:
                r = g = b = vr[o];
                break;
            // ⭐ THE BYTE ORDER IS A LESSON ALREADY PAID FOR. A Mac 32-bpp
            // pixel is big-endian xRGB: byte 0 is the unused/alpha lane and
            // R, G, B follow. Reading bytes 0,1,2 as B,G,R put the unused
            // lane in blue and shifted the other two, so a 50% gray desktop
            // (00 80 80 80) came out as r=80 g=80 b=00 — olive. The whole
            // screen was yellow, which is what named the bug. The 15/16-bpp
            // pixel is therefore the BIG-ENDIAN halfword (vr[o] << 8) |
            // vr[o+1] before any field is extracted, and 24 bpp drops the
            // unused lane: R,G,B in bytes 0,1,2.
            //
            // ⚠ If colors ever come out wrong in these modes, check
            // SURFACE_CNTL / DAC swap state before inventing a new table —
            // the card has per-aperture byte swappers and the guest may
            // program them.
            case 3u: { // ARGB1555, QuickDraw's 16-bit (555 + unused bit)
                const u32 v = (u32(vr[o]) << 8) | vr[o + 1];
                r = static_cast<u8>(((v >> 10) & 0x1Fu) << 3);
                g = static_cast<u8>(((v >> 5) & 0x1Fu) << 3);
                b = static_cast<u8>((v & 0x1Fu) << 3);
                // Zeros below, as the chip indexes its table: full scale is
                // entry 0xF8, and what that entry holds is the driver's
                // business (Mac OS's ramp has 250 there).
                break;
            }
            case 4u: { // RGB565
                const u32 v = (u32(vr[o]) << 8) | vr[o + 1];
                r = static_cast<u8>(((v >> 11) & 0x1Fu) << 3);
                g = static_cast<u8>(((v >> 5) & 0x3Fu) << 2);
                b = static_cast<u8>((v & 0x1Fu) << 3);
                break;
            }
            case 5u: // RGB888, big-endian: R,G,B
                r = vr[o];
                g = vr[o + 1];
                b = vr[o + 2];
                break;
            case 6u: // xRGB8888, big-endian: byte 0 is the unused lane
                r = vr[o + 1];
                g = vr[o + 2];
                b = vr[o + 3];
                break;
            default: break;
            }
        }
        out[x * 4 + 0] = s.lut[2][b];
        out[x * 4 + 1] = s.lut[1][g];
        out[x * 4 + 2] = s.lut[0][r];
        out[x * 4 + 3] = 0xFF;
    }
}

// 64x64 at two bits per pixel, 16 bytes per row: eight bytes of AND bits
// then eight of XOR bits, each big-endian with the most significant bit
// leftmost. AND=0 selects color 0 or 1 from XOR; AND=1 with XOR=0 is
// transparent, and with XOR=1 complements what is underneath — which is
// how the classic arrow keeps its black outline over any background.
// CUR_HORZ_VERT_OFF is how many cursor rows/columns to skip, which is how
// the driver clips the pointer against the top and left edges.
void r128ScanCursor(const R128Cell& c, const R128Scan& s, u8* bgra)
{
    if (!(c.peek(0x0050) & 0x00010000u)) // CRTC_GEN_CNTL: cursor enable
        return;
    const auto& vr = c.vram;
    const u32 curOff = c.peek(0x0260) & 0x07FFFFFFu;
    const u32 posn = c.peek(0x0264);
    const u32 hvoff = c.peek(0x0268);
    const u32 clr[2] = {c.peek(0x026C), c.peek(0x0270)};
    const u32 cx = (posn >> 16) & 0xFFFFu, cy = posn & 0xFFFFu;
    const u32 ox = (hvoff >> 16) & 0x3Fu, oy = hvoff & 0x3Fu;
    for (u32 row = oy; row < 64u; ++row) {
        const u32 sy = cy + (row - oy);
        if (sy >= s.h)
            break;
        const size_t so = size_t(curOff) + size_t(row) * 16u;
        if (so + 16u > vr.size())
            break;
        u64 abits = 0, xbits = 0;
        for (u32 k = 0; k < 8; ++k) {
            abits = (abits << 8) | vr[so + k];
            xbits = (xbits << 8) | vr[so + 8u + k];
        }
        for (u32 col = ox; col < 64u; ++col) {
            const u32 sx = cx + (col - ox);
            if (sx >= s.w)
                break;
            const u64 bit = u64(1) << (63u - col);
            u8* px = bgra + (size_t(sy) * s.w + sx) * 4u;
            if (abits & bit) {
                if (!(xbits & bit))
                    continue; // transparent
                px[0] = static_cast<u8>(~px[0]);
                px[1] = static_cast<u8>(~px[1]);
                px[2] = static_cast<u8>(~px[2]);
            } else {
                // Past the table, through the monitor: see the header.
                const u32 p = clr[(xbits & bit) ? 1 : 0];
                px[0] = kMonitorCurve[p & 0xFFu];
                px[1] = kMonitorCurve[(p >> 8) & 0xFFu];
                px[2] = kMonitorCurve[(p >> 16) & 0xFFu];
            }
            px[3] = 0xFF;
        }
    }
}

} // namespace opm
