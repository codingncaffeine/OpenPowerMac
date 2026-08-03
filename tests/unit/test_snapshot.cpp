// Snapshot / restore. The property under test is completeness: a snapshot
// that captures MOST of the machine is worse than none, because the restored
// run diverges silently and every observation made afterwards is fiction.
//
// Three separate proofs, because each catches a different failure:
//   1. round trip — save, mutate everything, restore, and the serialization
//      must be byte-identical to the original, with every device answering
//      its pre-mutation values again;
//   2. execution — run, snapshot, run a window, restore over the machine the
//      window left behind, run the window again: the instruction stream must
//      be identical step for step. A field that is never saved is still
//      holding its post-window value when the second leg starts, so this is
//      what catches an omission the round trip cannot see;
//   3. refusal — a truncated, mis-tagged or wrong-version file must fail
//      loudly instead of being half-read.

#include "doctest.h"
#include "opm/cpu.hpp"
#include "opm/sawtooth.hpp"
#include "opm/snapshot.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace opm;

namespace {

// 8 MB: enough for the exerciser below, and it keeps the serialized image
// small. The sizing-window aperture aliases modulo a 64 MB DIMM, so nothing
// here touches 0x78000000+.
constexpr size_t kRam = 8u << 20;

std::vector<u8> makeRom()
{
    std::vector<u8> rom(SawtoothBus::kRomSize, 0);
    for (size_t k = 0; k < rom.size(); ++k)
        rom[k] = static_cast<u8>(0xA5u ^ (k * 7u));
    // A deterministic exerciser at the reset vector: store a run of words
    // through the data cache, then spin. Real-mode stores are cacheable
    // (WIMG 0b0011), so this leaves dirty L1D lines holding values RAM does
    // not — exactly the state a naive snapshot drops on the floor.
    const u32 prog[] = {
        0x3C600001u, // lis   r3,1          ; r3 = 0x00010000
        0x38800000u, // li    r4,0
        0x90830000u, // stw   r4,0(r3)
        0x38840001u, // addi  r4,r4,1
        0x38630004u, // addi  r3,r3,4
        0x2C040064u, // cmpwi r4,100
        0x4180FFF0u, // blt   -16
        0x48000000u, // b     .
    };
    for (size_t k = 0; k < sizeof prog / sizeof prog[0]; ++k)
        for (u32 b = 0; b < 4; ++b)
            rom[0x100 + k * 4 + b] =
                static_cast<u8>(prog[k] >> (24 - 8 * b));
    return rom;
}

// A small ATAPI image so the CD cell can be driven into a real mid-command
// state rather than left at its power-on defaults.
const char* kIsoPath = "opm_snapshot_test.iso";

bool makeIso()
{
    FILE* f = fopen(kIsoPath, "wb");
    if (!f)
        return false;
    std::vector<u8> buf(256u * 1024u);
    for (size_t k = 0; k < buf.size(); ++k)
        buf[k] = static_cast<u8>(k * 31u + 7u);
    const bool ok = fwrite(buf.data(), 1, buf.size(), f) == buf.size();
    fclose(f);
    return ok;
}

// Put every subsystem into a distinctive state. Called twice with different
// seeds: the second call must leave a machine the first snapshot can undo.
void driveCpu(Cpu& cpu, u32 seed)
{
    CpuState& s = cpu.st;
    for (u32 k = 0; k < 32; ++k) {
        s.gpr[k] = 0x1000u * seed + k;
        s.fpr[k] = 0x4000000000000000ull + k * 0x101u * seed;
        for (u32 b = 0; b < 16; ++b)
            s.vr[k].b[b] = static_cast<u8>(k * 16u + b + seed);
    }
    s.pc = 0x00100000u + seed * 4u;
    s.cr = 0x12345678u ^ seed;
    s.xer = 0x20000000u | seed;
    s.lr = 0xFFC00000u + seed;
    s.ctr = 0x00ABCDEFu + seed;
    s.msr = 0x00003030u | seed;
    s.fpscr = 0x82000000u ^ seed;
    s.vscr = 0x00010000u ^ seed;
    s.vrsave = 0xF0F0F0F0u ^ seed;
    s.srr0 = 0x00050000u + seed;
    s.srr1 = 0x00009032u ^ seed;
    for (u32 k = 0; k < 4; ++k)
        s.sprg[k] = 0x5000u * seed + k;
    s.dar = 0x00040000u + seed;
    s.dsisr = 0x42000000u ^ seed;
    s.sdr1 = 0x00300000u | seed;
    s.ear = seed;
    s.pir = seed;
    for (u32 k = 0; k < 16; ++k)
        s.sr[k] = 0x00100000u * seed + k;
    for (u32 k = 0; k < 4; ++k) {
        s.ibatu[k] = 0x80000002u + k * seed;
        s.ibatl[k] = 0x80000012u + k * seed;
        s.dbatu[k] = 0x90000002u + k * seed;
        s.dbatl[k] = 0x90000012u + k * seed;
    }
    s.hid0 = 0x0000C000u ^ seed;
    s.hid1 = 0x00001234u ^ seed;
    s.msscr0 = 0x00400000u ^ seed;
    s.msscr1 = seed;
    s.l2cr = 0xA0000000u ^ seed;
    s.ictc = seed;
    for (u32 k = 0; k < 3; ++k)
        s.thrm[k] = 0x1000u * seed + k;
    s.iabr = 0x00060000u + seed;
    s.dabr = 0x00070000u + seed;
    s.bamr = 0xFFFF0000u ^ seed;
    s.mmcr0 = 0x00000080u ^ seed;
    s.mmcr1 = seed;
    for (u32 k = 0; k < 4; ++k)
        s.pmc[k] = 0x2000u * seed + k;
    s.siar = 0x00080000u + seed;
    s.sdar = 0x00090000u + seed;
    s.tb = 0x0000000123456789ull + seed;
    s.dec = 0x7FFF0000u ^ seed;
    s.pvr = 0x000C0209u;
    s.resvValid = (seed & 1u) != 0;
    s.resvAddr = 0x000A0000u + seed * 32u;

    cpu.extIrqLine = (seed & 1u) != 0;
    cpu.smiPending = (seed & 2u) != 0;
    cpu.decPending = (seed & 1u) == 0;
    cpu.pmPending = (seed & 2u) == 0;
    cpu.raisedThisStep = (seed & 1u) != 0;
    cpu.napping = false;
    cpu.curInsn = 0x60000000u ^ seed;
    cpu.cycleAccum = seed;
    cpu.cyclesPerTbTick = 4u + (seed & 1u);
    cpu.halted = false;
    cpu.haltReason = std::string("reason-") + std::to_string(seed);
    cpu.mmuProbe = false;
    cpu.realModeInhibitBase = 0xF0000000u ^ seed;

    for (u32 set = 0; set < 64; ++set)
        for (u32 way = 0; way < 2; ++way) {
            Cpu::TlbEntry& i = cpu.itlb[set][way];
            i = {true, (seed & 1u) != 0, 0x1000u * seed + set, set + way,
                 0x100u + set, 2u, 1u};
            Cpu::TlbEntry& d = cpu.dtlb[set][way];
            d = {true, (seed & 2u) != 0, 0x2000u * seed + set, set * 2u + way,
                 0x200u + set, 3u, 2u};
            cpu.itlbLru[set] = static_cast<u8>(way ^ seed);
            cpu.dtlbLru[set] = static_cast<u8>(set ^ seed);
        }

    cpu.l1dClock = 0x1234u + seed;
    for (u32 set = 0; set < 128; ++set)
        for (u32 way = 0; way < 8; ++way) {
            Cpu::DLine& l = cpu.l1d[set][way];
            const bool v = ((set + way + seed) & 3u) != 0;
            l.d = ((set + way + seed) & 1u) != 0;
            cpu.l1x[set].tv[way] =
                ((0x10u + set * 8u + way + seed) << 1) | (v ? 1u : 0u);
            cpu.l1x[set].age[way] = set + way * seed;
            for (u32 b = 0; b < 32; ++b)
                l.b[b] = static_cast<u8>(set + way * 8u + b + seed);
        }
    cpu.l2Sets = 64u + seed;
    cpu.l2Clock = 0x5678u + seed;
    cpu.l2.assign(128u + seed, Cpu::L2Line{});
    for (size_t k = 0; k < cpu.l2.size(); ++k) {
        Cpu::L2Line& l = cpu.l2[k];
        l.v = (k & 1u) != 0;
        l.d = (k & 2u) != 0;
        l.tag = static_cast<u32>(k) + seed;
        l.age = static_cast<u32>(k) * seed;
        for (u32 b = 0; b < 32; ++b)
            l.b[b] = static_cast<u8>(k + b + seed);
    }
}

void driveBus(SawtoothBus& bus, u32 seed)
{
    // RAM, the mac-io register store, the Uni-North block, an unclaimed
    // address (which only exists in the access log), and the boot flash
    // write path (logged, never applied).
    bus.write32(0x00020000u, 0xDEAD0000u | seed);
    bus.write32(0x00700000u, 0xBEEF0000u | seed);
    bus.write32(0xF3000038u, 0x00000010u | seed); // KeyLargo FCR
    bus.write32(0xF8000070u, 0x00000001u | seed); // Uni-N HWINIT_STATE
    bus.write32(0xF9000000u, 0x11110000u | seed); // unclaimed
    bus.write32(0xFFF00000u, 0x22220000u | seed); // flash

    // PCI config: latch + data on all three bridges.
    for (u32 b = 0; b < 3; ++b) {
        bus.write32(0xF0800000u + (b * 0x02000000u), 0x00000804u);
        bus.write32(0xF0C00000u + (b * 0x02000000u), 0x00000006u | seed);
    }
    // Give the OHCI functions and the ATI register block real BARs, which
    // also proves the derived-BAR state survives.
    bus.write32(0xF2800000u, 0x01000010u);
    bus.write32(0xF2C00000u, 0x80081000u);
    bus.write32(0xF2800000u, 0x02000010u);
    bus.write32(0xF2C00000u, 0x80082000u);

    // Keywest I2C on the Uni-North SPD bus: address the DIMM and launch.
    bus.write8(0xF8001053u, 0xA1u);
    bus.write8(0xF8001063u, static_cast<u8>(0x10u + seed));
    bus.write8(0xF8001013u, 0x02u);

    // SCC: console output, and a paced injection sitting in the RX queue.
    bus.write8(0xF3013030u, static_cast<u8>('A' + seed));
    bus.write8(0xF3013030u, static_cast<u8>('B' + seed));
    bus.injectSerial(std::string("mac-boot") + std::to_string(seed));

    // OpenPIC: a source's vector/priority and destination, plus a live line.
    bus.write32(0xF3050000u + 20u * 0x20u, 0x00A0u + seed); // src 20 vp
    bus.write32(0xF3050010u + 20u * 0x20u, 0x00000001u);    // src 20 dest
    bus.write32(0xF3060080u, 0x00000000u);                  // task priority
    bus.pic().setLine(20, true);

    // The two OHCI cells (register file is LE on the PCI side).
    for (u32 f = 0; f < 2; ++f) {
        OhciCell& o = bus.ohci(f);
        o.write(0x04u, 0x80u + seed, 4); // HcControl
        o.write(0x18u, 0x00030000u + seed, 4);
        o.write(0x34u, 0x2EDFu + seed, 4);
    }

    // DBDMA: a command pointer and a channel-control write.
    bus.write32(0xF3008B0Cu, 0x00030000u + seed);
    bus.write32(0xF3008B00u, 0x30003000u);

    // VIA / PMU: direction registers, the auxiliary control register, and a
    // timer 1 latch pair.
    bus.pmu().write(2u * 0x200u, static_cast<u8>(0x10u | seed), 1000);
    bus.pmu().write(3u * 0x200u, static_cast<u8>(0x20u | seed), 1000);
    bus.pmu().write(11u * 0x200u, static_cast<u8>(0x1Cu ^ seed), 1000);
    bus.pmu().write(6u * 0x200u, static_cast<u8>(0x34u + seed), 1000);
    bus.pmu().write(7u * 0x200u, static_cast<u8>(0x12u + seed), 1000);

    // Rage 128: a register, the PLL file, and the DAC palette.
    R128Cell& g = bus.ati();
    g.write(0x0050u, 0x02000200u + seed, 4);
    g.write(0x0008u, 0x00000002u + seed, 4); // CLOCK_CNTL_INDEX
    g.write(0x000Cu, 0x11223344u + seed, 4); // CLOCK_CNTL_DATA
    g.write(0x00B0u, seed & 0xFFu, 4);       // PALETTE_INDEX
    for (u32 k = 0; k < 8; ++k)
        g.write(0x00B4u, 0x00102030u + k + seed, 4);
    g.vram[0x1000u + seed] = static_cast<u8>(0x77u + seed);

    // The ATAPI cell: reset, task file, then a real PACKET command so the
    // CDB buffer, the PIO buffer and the interrupt line are all mid-flight.
    bus.write8(0xF3020160u, 0x04u); // SRST -> ATAPI signature
    bus.write8(0xF3020060u, 0x00u); // device 0
    bus.write8(0xF3020040u, static_cast<u8>(0xFEu - seed));
    bus.write8(0xF3020050u, 0x00u);
    bus.write8(0xF3020070u, 0xA0u); // PACKET
    const u8 cdb[12] = {0x12, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0}; // INQUIRY
    for (u32 k = 0; k < 12; ++k)
        bus.write8(0xF3020000u, cdb[k]);
    for (u32 k = 0; k <= seed; ++k)
        (void)bus.read16(0xF3020000u); // a seed-dependent PIO position
    bus.write8(0xF3020020u, static_cast<u8>(0x5Au + seed)); // nsect
    bus.write8(0xF3020030u, static_cast<u8>(0x77u + seed)); // lba0
}

// Values a restored machine must answer with again. Deliberately made of
// side-effect-free reads: nothing here pops a queue or acknowledges a line.
std::vector<u32> probe(SawtoothBus& bus, const Cpu& cpu)
{
    std::vector<u32> v;
    v.push_back(cpu.st.gpr[5]);
    v.push_back(static_cast<u32>(cpu.st.tb));
    v.push_back(cpu.st.sr[3]);
    v.push_back(cpu.l1x[7].tv[2] >> 1);
    v.push_back(cpu.l1d[7][2].b[9]);
    v.push_back(cpu.itlb[9][1].rpn);
    v.push_back(cpu.l2Sets);
    v.push_back(static_cast<u32>(cpu.l2.size()));
    v.push_back(cpu.l2.empty() ? 0u : cpu.l2[5].tag);
    v.push_back(bus.read32(0x00020000u));
    v.push_back(bus.read32(0x00700000u));
    v.push_back(bus.read32(0xF3000038u));
    v.push_back(bus.read32(0xF8000070u));
    v.push_back(bus.pic().read(0x10000u + 20u * 0x20u, 4));
    v.push_back(bus.pic().cpuLine() ? 1u : 0u);
    for (u32 f = 0; f < 2; ++f) {
        v.push_back(bus.ohci(f).read(0x04u, 4));
        v.push_back(bus.ohci(f).read(0x18u, 4));
        v.push_back(bus.ohci(f).read(0x34u, 4));
    }
    v.push_back(bus.ataDma().read(0x0Cu, 4));
    v.push_back(bus.pmu().read(2u * 0x200u, 1000));
    v.push_back(bus.pmu().read(3u * 0x200u, 1000));
    v.push_back(bus.pmu().read(11u * 0x200u, 1000));
    v.push_back(bus.pmu().read(6u * 0x200u, 1000));
    v.push_back(bus.ati().peek(0x0050u));
    v.push_back(bus.ati().pal(2));
    v.push_back(bus.ati().vram[0x1000u]);
    v.push_back(bus.ati().vram[0x1001u]);
    v.push_back(bus.read8(0xF3020160u)); // alt status: no INTRQ side effect
    v.push_back(bus.read8(0xF3020040u));
    v.push_back(bus.read8(0xF3020020u));
    v.push_back(bus.read8(0xF3020030u));
    v.push_back(static_cast<u32>(bus.console().size()));
    v.push_back(bus.console().empty()
                    ? 0u
                    : static_cast<u8>(bus.console().back()));
    return v;
}

} // namespace

TEST_CASE("snapshot round-trips every device")
{
    REQUIRE(makeIso());
    SawtoothBus bus(kRam, makeRom());
    REQUIRE(bus.attachCd(kIsoPath));
    Cpu cpu;
    cpu.attach(bus);
    cpu.reset();

    driveCpu(cpu, 1);
    driveBus(bus, 1);
    const HarnessState h1{123456789ull, 60u, 999ull, 7u, true, true, false};
    const std::vector<u32> before = probe(bus, cpu);

    SnapWriter w0;
    saveSnapshot(cpu, bus, h1, w0);
    const u64 fp0 = snapshotFingerprint(cpu, bus, h1);

    // A different machine in every subsystem, so the restore has something
    // real to undo.
    driveCpu(cpu, 2);
    driveBus(bus, 2);
    const HarnessState h2{987654321ull, 0u, 1ull, 1u, false, false, true};
    CHECK(snapshotFingerprint(cpu, bus, h2) != fp0);

    HarnessState hr;
    SnapReader r(w0.buf.data(), w0.buf.size());
    const bool loaded = loadSnapshot(cpu, bus, hr, r);
    INFO("load error: " << r.err);
    REQUIRE(loaded);

    CHECK(hr.executed == h1.executed);
    CHECK(hr.fastTb == h1.fastTb);
    CHECK(hr.fastTbUntil == h1.fastTbUntil);
    CHECK(hr.parkSeen == h1.parkSeen);
    CHECK(hr.parkArmed == h1.parkArmed);
    CHECK(hr.ataPoked == h1.ataPoked);
    CHECK(hr.emPoked == h1.emPoked);

    // Byte-identical re-serialization: saved-but-mis-restored fields show up
    // here even when nothing ever reads them.
    SnapWriter w1;
    saveSnapshot(cpu, bus, hr, w1);
    CHECK(w1.buf.size() == w0.buf.size());
    CHECK(w1.buf == w0.buf);

    // And the devices themselves answer their pre-mutation values.
    CHECK(probe(bus, cpu) == before);

    remove(kIsoPath);
}

TEST_CASE("a restored machine executes identically")
{
    SawtoothBus bus(kRam, makeRom());
    Cpu cpu;
    cpu.attach(bus);
    cpu.reset();
    cpu.st.hid0 |= 0x00004000u; // DCE: run the exerciser through the L1D

    u64 executed = 0;
    bus.stamp = &executed;
    bus.pcRef = &cpu.st.pc;

    auto advance = [&](u64 n, std::vector<u32>* rec) {
        for (u64 k = 0; k < n; ++k) {
            const u32 pc = cpu.st.pc;
            cpu.step();
            if (rec) {
                rec->push_back(pc);
                rec->push_back(cpu.curInsn);
            }
            cpu.tick(1);
            bus.ohciTick(cpu.st.tb);
            bus.syncIrqs();
            cpu.setExternalIrq(bus.pic().cpuLine());
            ++executed;
        }
    };

    advance(400, nullptr); // mid-loop: dirty cache lines, live registers
    const HarnessState h{executed, 0u, ~0ull, 0u, false, false, false};
    SnapWriter w;
    saveSnapshot(cpu, bus, h, w);
    const u64 fpAt = snapshotFingerprint(cpu, bus, h);

    std::vector<u32> legA;
    advance(600, &legA);
    const HarnessState hA{executed, 0u, ~0ull, 0u, false, false, false};
    const u64 fpA = snapshotFingerprint(cpu, bus, hA);

    // Restore ON TOP of the machine leg A left behind: anything the
    // serializer misses is still holding its post-window value here.
    HarnessState hr;
    SnapReader r(w.buf.data(), w.buf.size());
    const bool loaded = loadSnapshot(cpu, bus, hr, r);
    INFO("load error: " << r.err);
    REQUIRE(loaded);
    executed = hr.executed;
    CHECK(snapshotFingerprint(cpu, bus, hr) == fpAt);

    std::vector<u32> legB;
    advance(600, &legB);
    const HarnessState hB{executed, 0u, ~0ull, 0u, false, false, false};

    REQUIRE(legA.size() == legB.size());
    size_t firstBad = legA.size();
    for (size_t k = 0; k < legA.size(); ++k)
        if (legA[k] != legB[k]) {
            firstBad = k;
            break;
        }
    INFO("first divergence at step " << firstBad / 2);
    CHECK(firstBad == legA.size());
    CHECK(snapshotFingerprint(cpu, bus, hB) == fpA);
}

TEST_CASE("a damaged snapshot is refused, not misread")
{
    SawtoothBus bus(kRam, makeRom());
    Cpu cpu;
    cpu.attach(bus);
    cpu.reset();
    driveCpu(cpu, 3);
    const HarnessState h{1u, 0u, ~0ull, 0u, false, false, false};
    SnapWriter w;
    saveSnapshot(cpu, bus, h, w);

    auto refuses = [&](std::vector<u8> blob) {
        HarnessState hr;
        SnapReader r(blob.data(), blob.size());
        const bool ok = loadSnapshot(cpu, bus, hr, r);
        return !ok && !r.err.empty();
    };

    CHECK(refuses({})); // empty
    {
        std::vector<u8> bad = w.buf;
        bad[0] ^= 0xFFu; // magic
        CHECK(refuses(bad));
    }
    {
        std::vector<u8> bad = w.buf;
        bad[4] = static_cast<u8>(bad[4] + 1u); // version
        CHECK(refuses(bad));
    }
    {
        std::vector<u8> bad = w.buf;
        bad[8] ^= 0x01u; // layout digest
        CHECK(refuses(bad));
    }
    {
        std::vector<u8> bad = w.buf;
        bad.resize(bad.size() / 2); // truncated mid-machine
        CHECK(refuses(bad));
    }
    {
        std::vector<u8> bad = w.buf;
        bad.push_back(0x42u); // trailing garbage after the end marker
        CHECK(refuses(bad));
    }
}
