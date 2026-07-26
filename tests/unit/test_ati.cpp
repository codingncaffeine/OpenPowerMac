// Arc 2 M3: the ATI Rage Pro register core — beam counter and VBLANK
// latching, the CLOCK_CNTL PLL address/data window, the RAMDAC palette
// autoincrement, and the aperture-top register file routing. All behavior
// is deterministic; these pin the contracts the boot ROM's driver relies on.

#include "doctest.h"
#include "opm/ati.hpp"

using namespace opm;

namespace {

// Little-endian dword helpers over the byte-lane interface.
u32 ioRd32(AtiRage& a, u32 off)
{
    return u32(a.ioRead8(off)) | (u32(a.ioRead8(off + 1)) << 8) |
           (u32(a.ioRead8(off + 2)) << 16) | (u32(a.ioRead8(off + 3)) << 24);
}
void ioWr32(AtiRage& a, u32 off, u32 v)
{
    a.ioWrite8(off, static_cast<u8>(v));
    a.ioWrite8(off + 1, static_cast<u8>(v >> 8));
    a.ioWrite8(off + 2, static_cast<u8>(v >> 16));
    a.ioWrite8(off + 3, static_cast<u8>(v >> 24));
}

void programCrtc640x480x8(AtiRage& a)
{
    // 640x480: h_disp = 80-1 in 8-px units, v fields in lines (count-1).
    ioWr32(a, 0x00, (79u << 16) | 99u);        // CRTC_H_TOTAL_DISP
    ioWr32(a, 0x08, (479u << 16) | 524u);      // CRTC_V_TOTAL_DISP
    ioWr32(a, 0x14, (80u << 22) | 0u);         // CRTC_OFF_PITCH: 640 px pitch
    ioWr32(a, 0x1C, 0x02000200u);              // CRTC_GEN_CNTL: 8bpp + enable
}

} // namespace

TEST_CASE("ati: beam advances, VBLANK live + latched, W1C ack")
{
    AtiRage a;
    programCrtc640x480x8(a);

    // Read the beam at two points; it must advance.
    const u32 l0 = (ioRd32(a, 0x10) >> 16) & 0x7FFu;
    for (u32 i = 0; i < AtiRage::kTicksPerLine * 3; ++i)
        a.tick();
    const u32 l1 = (ioRd32(a, 0x10) >> 16) & 0x7FFu;
    CHECK(l1 == l0 + 3);

    // Run into the blank region: live bit up, latch set.
    while ((((ioRd32(a, 0x10) >> 16) & 0x7FFu) != 480u))
        a.tick();
    const u32 ic = ioRd32(a, 0x18);
    CHECK((ic & 0x1u) != 0);   // live VBLANK
    CHECK((ic & 0x4u) != 0);   // latched VBLANK_INT

    // Ack: write-1-to-clear the latch; live status unaffected by the ack.
    ioWr32(a, 0x18, 0x4u);
    const u32 ic2 = ioRd32(a, 0x18);
    CHECK((ic2 & 0x4u) == 0);
    CHECK((ic2 & 0x1u) != 0);

    // Wrap: beam returns to line 0 and the live bit drops.
    while ((((ioRd32(a, 0x10) >> 16) & 0x7FFu) != 0u))
        a.tick();
    CHECK((ioRd32(a, 0x18) & 0x1u) == 0);
}

TEST_CASE("ati: VLINE latch fires on the programmed line")
{
    AtiRage a;
    programCrtc640x480x8(a);
    ioWr32(a, 0x10, 100u); // compare line
    ioWr32(a, 0x18, 0x10u); // clear any latch
    while ((((ioRd32(a, 0x10) >> 16) & 0x7FFu) != 101u))
        a.tick();
    CHECK((ioRd32(a, 0x18) & 0x10u) != 0);
}

TEST_CASE("ati: CLOCK_CNTL PLL address/data window")
{
    AtiRage a;
    // Write PLL reg 5 = 0xAB: addr|WR_EN, then data.
    a.ioWrite8(0x91, static_cast<u8>((5u << 2) | 2u));
    a.ioWrite8(0x92, 0xAB);
    // Write PLL reg 6 = 0x33.
    a.ioWrite8(0x91, static_cast<u8>((6u << 2) | 2u));
    a.ioWrite8(0x92, 0x33);
    // Read back through the window with WR_EN clear.
    a.ioWrite8(0x91, static_cast<u8>(5u << 2));
    CHECK(a.ioRead8(0x92) == 0xAB);
    a.ioWrite8(0x91, static_cast<u8>(6u << 2));
    CHECK(a.ioRead8(0x92) == 0x33);
    // WR_EN clear: data writes must not land.
    a.ioWrite8(0x92, 0x77);
    CHECK(a.ioRead8(0x92) == 0x33);
}

TEST_CASE("ati: RAMDAC palette autoincrement, write and read paths")
{
    AtiRage a;
    a.ioWrite8(0xC0, 3); // write index
    const u8 rgb[2][3] = {{0x11, 0x22, 0x33}, {0x44, 0x55, 0x66}};
    for (const auto& c : rgb)
        for (u8 comp : c)
            a.ioWrite8(0xC1, comp);
    u8 r, g, b;
    a.palette(3, r, g, b);
    CHECK(r == 0x11);
    CHECK(g == 0x22);
    CHECK(b == 0x33);
    a.palette(4, r, g, b);
    CHECK(r == 0x44);

    a.ioWrite8(0xC3, 3); // read index, then six autoincrementing reads
    CHECK(a.ioRead8(0xC1) == 0x11);
    CHECK(a.ioRead8(0xC1) == 0x22);
    CHECK(a.ioRead8(0xC1) == 0x33);
    CHECK(a.ioRead8(0xC1) == 0x44);
    CHECK(a.ioRead8(0xC1) == 0x55);
    CHECK(a.ioRead8(0xC1) == 0x66);
}

TEST_CASE("ati: register file serves both aperture halves and the io map")
{
    AtiRage a;
    // Write SCRATCH_REG0 via io; read it via both memory-mapped halves.
    ioWr32(a, 0x80, 0xDEADBEEFu);
    const u32 lo = AtiRage::kRegTop + 0x80u;         // +0x7FF800 half
    const u32 hi = AtiRage::kRegTop + 0x400u + 0x80u; // +0x7FFC00 half
    u32 v0 = 0, v1 = 0;
    for (int i = 3; i >= 0; --i) {
        v0 = (v0 << 8) | a.apRead8(lo + u32(i));
        v1 = (v1 << 8) | a.apRead8(hi + u32(i));
    }
    CHECK(v0 == 0xDEADBEEFu);
    CHECK(v1 == 0xDEADBEEFu);

    // VRAM below the register file stays VRAM.
    a.apWrite8(0x1234, 0x5A);
    CHECK(a.vram()[0x1234] == 0x5A);
    CHECK(a.apRead8(0x1234) == 0x5A);

    // CONFIG_CHIP_ID reads the Rage Pro identity and ignores writes.
    CHECK(ioRd32(a, 0xE0) == 0x5C004750u);
    ioWr32(a, 0xE0, 0);
    CHECK(ioRd32(a, 0xE0) == 0x5C004750u);
}

TEST_CASE("ati: GP_IO monitor sense presents a 13\" RGB (code 6, ext 0x2B)")
{
    AtiRage a;
    // Tristate all lines: primary sense = A,B pulled up, C grounded.
    ioWr32(a, 0x78, 0);
    u32 v = ioRd32(a, 0x78);
    CHECK((v & (1u << 13)) != 0);
    CHECK((v & (1u << 12)) != 0);
    CHECK((v & (1u << 8)) == 0);

    // Drive all high: every line reads its driven value.
    ioWr32(a, 0x78, 0x31003100u);
    v = ioRd32(a, 0x78);
    CHECK((v & (1u << 13)) != 0);
    CHECK((v & (1u << 12)) != 0);
    CHECK((v & (1u << 8)) != 0);

    // The driver's extended-sense matrix, exactly as it runs it:
    u32 sense;
    ioWr32(a, 0x78, 0);
    v = ioRd32(a, 0x78);
    sense = ((v & 0x3000u) >> 3) | (v & 0x100u);
    ioWr32(a, 0x78, 0x20000000u); // A low
    v = ioRd32(a, 0x78);
    sense |= ((v & 0x1000u) >> 7) | ((v & 0x100u) >> 4);
    ioWr32(a, 0x78, 0x10000000u); // B low
    v = ioRd32(a, 0x78);
    sense |= ((v & 0x2000u) >> 10) | ((v & 0x100u) >> 6);
    ioWr32(a, 0x78, 0x01000000u); // C low
    v = ioRd32(a, 0x78);
    sense |= (v & 0x3000u) >> 12;
    CHECK(sense == 0x62Bu); // primary 6, extended 0x2B
}

namespace {

// A little I2C master over GP_IO, mimicking the ROM FCode driver's wiring:
// SDA = bit 13 (enable 29), SCL = bit 12 (enable 28). SDA changes happen
// while SCL is low, except the deliberate START/STOP transitions.
struct DdcMaster {
    AtiRage& a;
    bool sclDrive = false, sclVal = true;
    bool sdaDrive = false, sdaVal = true;
    void apply()
    {
        const u32 en = (sclDrive ? 1u << 28 : 0) | (sdaDrive ? 1u << 29 : 0);
        const u32 da = (sclVal ? 1u << 12 : 0) | (sdaVal ? 1u << 13 : 0);
        ioWr32(a, 0x78, en | da);
    }
    bool sampleSda() { return (ioRd32(a, 0x78) & (1u << 13)) != 0; }
    void scl(bool v)
    {
        sclDrive = true;
        sclVal = v;
        apply();
    }
    void sda(bool drive, bool v)
    {
        sdaDrive = drive;
        sdaVal = v;
        apply();
    }
    void start()
    {
        sda(true, true);
        scl(true);
        sda(true, false); // SDA falls while SCL high
        scl(false);
    }
    void stop()
    {
        sda(true, false);
        scl(true);
        sda(true, true); // SDA rises while SCL high
    }
    bool writeByte(u8 b) // returns true on slave ACK
    {
        for (int i = 7; i >= 0; --i) {
            sda(true, (b >> i) & 1);
            scl(true);
            scl(false);
        }
        sda(false, true); // release for the ACK bit
        scl(true);
        const bool ack = !sampleSda();
        scl(false);
        return ack;
    }
    u8 readByte(bool ack)
    {
        u8 v = 0;
        sda(false, true);
        for (int i = 7; i >= 0; --i) {
            scl(true);
            v = static_cast<u8>((v << 1) | (sampleSda() ? 1 : 0));
            scl(false);
        }
        sda(true, !ack); // master ACK (low) or NACK (high)
        scl(true);
        scl(false);
        sda(false, true);
        return v;
    }
};

} // namespace

TEST_CASE("ati: DDC slave serves a checksum-valid EDID over I2C")
{
    AtiRage a;
    DdcMaster m{a};

    // Set the register pointer to 0, then read the whole block.
    m.start();
    CHECK(m.writeByte(0xA0));
    CHECK(m.writeByte(0x00));
    m.start(); // repeated START
    CHECK(m.writeByte(0xA1));
    u8 blk[128];
    for (int i = 0; i < 128; ++i)
        blk[i] = m.readByte(i != 127);
    m.stop();

    CHECK(blk[0] == 0x00);
    CHECK(blk[1] == 0xFF);
    CHECK(blk[6] == 0xFF);
    CHECK(blk[7] == 0x00);
    u32 sum = 0;
    for (u8 b : blk)
        sum += b;
    CHECK((sum & 0xFFu) == 0);

    // A non-slave address must be ignored (no ACK).
    m.start();
    CHECK_FALSE(m.writeByte(0x42));
    m.stop();

    // Legacy sense is unaffected: tristate reads code 6 lines.
    ioWr32(a, 0x78, 0);
    const u32 v = ioRd32(a, 0x78);
    CHECK((v & (1u << 13)) != 0);
    CHECK((v & (1u << 12)) != 0);
    CHECK((v & (1u << 8)) == 0);
}

TEST_CASE("ati: mode readout describes the programmed CRTC")
{
    AtiRage a;
    CHECK_FALSE(a.mode().enabled);
    programCrtc640x480x8(a);
    const AtiRage::Mode m = a.mode();
    CHECK(m.enabled);
    CHECK(m.width == 640);
    CHECK(m.height == 480);
    CHECK(m.bpp == 8);
    CHECK(m.pitchPixels == 640);
    CHECK(m.offsetBytes == 0);
}
