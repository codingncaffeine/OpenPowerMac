#include "opm/r128.hpp"

#include <cstdio> // the CRTC mode-change report; MSVC pulls this in, gcc does not

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
// ⚠ BUT IT GOES BACK WHEN THE TIMER DOES. SawtoothBus::klTimerOn is off by
// default — a correct clock stops this machine booting until the pacing work
// lands — so the 44.6x compression is still there, and the nominal period
// would hand the guest ~2,550 blanks per second of its own time. The cell's
// vblEnabled is a CONSTRUCTOR default of true, so this is live in the app.
// Move the two together, or not at all. --ati-vbl-tb still overrides it.
static constexpr u64 kTbPerVblank = 225000000ull;
// Harness knobs, not machine state — see the notes in the header.
static u64 gVblTbPeriod = 0;
static int gVblTrace = 0;
// Census of expired (unserviced) requests. A file-static because censuses are
// deliberately not snapshotted, and because it keeps sizeof(R128Cell) fixed.
static u64 gVblDropped = 0;

u32 R128Cell::regRead(u32 idx)
{
    const u32 off = idx << 2;
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
        return it != regs_.end() ? it->second : 0;
    }
    }
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
    const u32 aligned = off & ~3u;
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
