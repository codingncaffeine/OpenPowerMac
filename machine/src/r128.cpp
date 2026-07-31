#include "opm/r128.hpp"

#include <cstdio> // the CRTC mode-change report; MSVC pulls this in, gcc does not
#include <cstring> // memmove, for the blitter's row copies

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
// unmodelled offset never reaches the unclaimed-access log — it falls through
// to `default:` and reads back the last value written there, or zero. A guest
// driver polling such a register for a change waits for ever, and the only
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
    {0x1ca0, "MISC_3D_STATE_CNTL_REG"},
    {0x1d34, "CONSTANT_COLOR_C"},
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
// below 0x0700 is modelled and understood, and logging it would bury the
// engine traffic under the driver's DDC bit-banging — which is exactly what
// happened to the existing ring.
static constexpr u32 kEngLo = 0x0700, kEngHi = 0x2000;
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
// the behavioural oracle for how the pieces combine into a blit. Names,
// offsets, bit meanings and observed behaviour are facts; the code below is
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

const R128EngStats& r128EngStats() { return gEng; }
const std::map<u32, u64>& r128RopUnimplemented() { return gRopUnimpl; }


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
    // driver that waits for room before writing waits for ever — and it never
    // appears in the unclaimed-access log, because the card claims its whole
    // aperture. Session 30 wrote this down for mac-io as *claiming an address
    // and implementing it are different things*; this is the same trap, on
    // the register a command engine's bring-up reads first.
    if (gEng2dOn && (off == kGuiStat || off == kPm4Stat))
        return kGuiStatIdle;
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
        // This register was not modelled and read back as zero. Measured
        // consequence: the OS display driver stores it as its framebuffer
        // base (device record +0x8c), adds the CRTC offset it programmed
        // (+0x138 = 0x8000), and fills 640x480x32 from address 0x00008000 —
        // 1.2 MB straight over low memory. That wipes ExpandMem, the next
        // dereference of the poisoned cell raises a bus error, and the ROM
        // sad-Macs into `bra.s *` at ffc046ee. The screen staying blank was
        // the mildest symptom of it.
        return fbBase + kAper1Offset;
    case kConfigReg1Base:
        // Same shape, for the register aperture: a driver that asks where
        // its own registers are must not be told zero.
        return regBase;
    case kConfigRegAperSize:
        return 0x00004000u; // 16 KB, matching the BAR we advertise
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
            // Never written, not modelled: this read is answering 0 and will
            // keep answering 0 for ever. If a count here is large, the guest
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
    // 3 and 4 are "loaded thru hostdata": the CPU feeds pixels through
    // HOST_DATA0..7 after the trigger, which is a stream, not a rectangle
    // copy. Stage 3 of the plan. Counted rather than half-drawn.
    if (srcSource == 3u || srcSource == 4u) {
        ++gEng.hostData;
        return;
    }
    const bool usesSrc = ropUsesSrc(rop);
    const bool usesPat = ropUsesPat(rop);
    const bool usesDst = ropUsesDst(rop);

    // The pattern operand. Brush datatype 13 is a solid colour taken from
    // DP_BRUSH_FRGD_CLR, and 15 the manual says to "treat as 13". The mono
    // and colour brushes (0..12) need the pattern registers, which this
    // engine does not model — so if the op would actually READ the pattern,
    // decline rather than paint a solid guess where a chequer belongs.
    u32 pat = 0;
    if (usesPat) {
        const u32 brush = (dt >> 8) & 0xFu;
        if (brush != 13u && brush != 15u) {
            ++gEng.brushUnimpl;
            return;
        }
        pat = rd(kDpBrushFrgdClr);
    }
    // The source operand must be colour pixels from memory. A MONO source is
    // a 1-bit mask expanded through DP_SRC_FRGD_CLR/BKGD_CLR, which is its
    // own path and belongs with host-data blits.
    if (usesSrc && ((dt >> 16) & 0x3u) != 3u) {
        ++gEng.monoSrc;
        return;
    }

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
    const u32 srcStride = strideOf(rd(kSrcPitch));
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
    if (lastByte(dstOff, dstStride, x0, y0, ch) > vram.size() ||
        (usesSrc && lastByte(srcOff, srcStride, csx, csy, ch) > vram.size())) {
        ++gEng.offVram;
        return;
    }

    // ⚠ A COLOUR-KEY FUNCTION THIS ENGINE DOES NOT APPLY. The manual's own
    // two renderings of CLR_CMP_FN_SRC contradict each other about which
    // sense of the comparison suppresses the pixel, so implementing it from
    // the document would be a guess in the one place a guess is invisible —
    // transparent pixels drawn opaque look like an ordinary drawing bug. The
    // blit still runs; the count says how often the omission could matter,
    // and a non-zero count is the instruction to go and settle the semantics.
    const u32 cmp = rd(kClrCmpCntl);
    if ((cmp & 0x7u) || ((cmp >> 8) & 0x7u))
        ++gEng.colorCompare;

    // --- draw -----------------------------------------------------------
    //
    // ⚠ BYTE ORDER. This model's VRAM holds pixels exactly as the guest
    // wrote them through the big-endian aperture, and the scanout in
    // opm_screen reads a 32-bpp pixel as byte0=unused, byte1=R, byte2=G,
    // byte3=B. A fill colour arriving in DP_BRUSH_FRGD_CLR is a plain 32-bit
    // number, so it has to be stored MOST SIGNIFICANT BYTE FIRST for the
    // three readers of this memory — the CPU, the scanout and this engine —
    // to agree. Storing it the other way round is exactly the bug that once
    // made a grey desktop come out olive.
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
        for (u32 k = 0; k < bypp; ++k)
            vram[o + k] = static_cast<u8>(v >> (8 * (bypp - 1 - k)));
    };

    const bool plain = mask == 0xFFFFFFFFu;
    if (!usesSrc && !usesDst) {
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
    } else if (rop == 0xCCu && plain) {
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
        ++gEng.copies;
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
                const u32 S = usesSrc ? pixAt(s + static_cast<size_t>(cc) * bypp)
                                      : 0u;
                const u32 D = usesDst ? pixAt(dp) : 0u;
                putPix(dp, rop3(rop, pat, S, D));
            }
        }
    }
    ++gEng.blits;
    gEng.pixels += static_cast<u64>(cw) * ch;
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
bool R128Cell::engWrite(u32 off, u32 v)
{
    switch (off) {
    // --- composite registers: normalise into the canonical pair ---------
    //
    // Every one of these is marked [W] in the reference guide — they are
    // write-only aliases, so nothing is lost by decomposing them, and
    // decomposing is what lets the blit read ONE canonical set.
    case kDstPitchOffset:
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
    case kDpGuiMasterCntl: {
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
        if (v & 0x20000000u) // GMC_AUX_CLIP_DIS: clear every AUXn_SC_ENB
            regs_[kAuxScCntl] = rd(kAuxScCntl) & ~0x00010101u;
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

    // --- the pixel cache -------------------------------------------------
    case kPcGuiCtlstat:
        // ⭐ THESE ARE SELF-CLEARING COMMAND PULSES, NOT STORAGE. ATI's own
        // wording, on every one of bits 0..9: "Once a bit in the field is set,
        // it will remain set until the flush is complete, then the Pixel Cache
        // will automatically clear it."
        //
        // The accelerator writes 0x000000FF here — `R128_PC_FLUSH_ALL`, flush
        // and read-invalidate both the GUI and non-GUI sides — and then reads
        // the register back. Storing the value and answering it back for ever
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
    const u64 at = stamp ? *stamp : 0;
    note(off & ~3u, native, true);
    noteEngine(off & ~3u, native, pcRef ? *pcRef : 0, at, true);
    const u32 aligned = off & ~3u;
    // The GUI block first: a write there may be an alias to normalise or the
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
    // magnitude inside this window, so serviced behaviour is unchanged.
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
    case 20: return 0x0E;                       // analogue input
    case 21: return 0x22; case 22: return 0x1B; // 34 cm x 27 cm
    case 23: return 0x78;                       // gamma 2.2
    case 24: return 0x0D;                       // RGB, preferred timing
    case 35: return 0x21;  // established: 640x480@60, 800x600@60
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

} // namespace opm
