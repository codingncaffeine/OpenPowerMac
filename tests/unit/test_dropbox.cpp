// The drop box: a CD-class drive whose DISC comes and goes while the DRIVE
// stays on the channel — and the protocol a swapped disc speaks to the
// guest's own CD driver, with nothing injected into guest memory.
//
// The drive answers every media command NOT READY / MEDIUM NOT PRESENT
// while the tray is empty, announces a new disc with UNIT ATTENTION /
// MEDIUM MAY HAVE CHANGED on the first command that could act on it, and a
// host-requested swap waits for the guest to SEE the empty tray before the
// new disc goes in (a guest that never looks gets it after a fallback).
// Each of those is a claim a driver relies on, so each is asserted here
// against the register interface the guest uses, not against the cell's
// own accessors alone.
//
// The snapshot half: the third channel and the media-change state travel
// in an OPTIONAL section after everything that existed before it, so a
// snapshot written without it loads exactly as it always did. That is
// proved both ways — a stream with the section, and the SAME stream with
// the section spliced out.

#include "doctest.h"
#include "opm/ata.hpp"
#include "opm/cpu.hpp"
#include "opm/sawtooth.hpp"
#include "opm/snapshot.hpp"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace opm;

namespace {

const char* kDiscA = "opm_drop_a.iso";
const char* kDiscB = "opm_drop_b.iso";

// A raw 2048-byte-block image whose every block begins with its own tag
// byte, so a read says WHICH disc served it.
bool makeDisc(const char* path, u8 tag, u32 blocks)
{
    FILE* f = fopen(path, "wb");
    if (!f)
        return false;
    std::vector<u8> blk(2048);
    bool ok = true;
    for (u32 b = 0; b < blocks && ok; ++b) {
        for (size_t k = 0; k < blk.size(); ++k)
            blk[k] = static_cast<u8>(tag + k * 3u + b);
        blk[0] = tag;
        ok = fwrite(blk.data(), 1, blk.size(), f) == blk.size();
    }
    fclose(f);
    return ok;
}

// Status register bits, as a driver reads them.
constexpr u32 kBsy = 0x80, kDrdy = 0x40, kDrq = 0x08, kErr = 0x01;

// Issue an ATAPI PACKET command the way the guest does: PACKET through the
// command register, then the 12 CDB bytes through the data register two at
// a time. With a clock attached the command write is deferred behind BSY
// (see the ATA BSY test), so the clock is run past the delay before the
// CDB goes in; without one it runs as written.
void packet(AtaCell& c, const u8* cdb, u64* clock = nullptr)
{
    c.write(0x060, 0x00, 1);
    c.write(0x070, 0xA0, 1);
    if (clock) {
        CHECK((c.read(0x160, 1) & kBsy) != 0);
        *clock += c.cmdDelayTb_;
        c.tick();
    }
    REQUIRE((c.read(0x160, 1) & kDrq) != 0); // command phase open
    for (u32 k = 0; k < 12; k += 2)
        c.write(0x000, (u32(cdb[k]) << 8) | cdb[k + 1], 2);
}

void testUnitReady(AtaCell& c, u64* clock = nullptr)
{
    const u8 cdb[12] = {0};
    packet(c, cdb, clock);
}

// Drain a data phase one 16-bit word at a time into a byte vector.
std::vector<u8> drain(AtaCell& c, u32 bytes)
{
    std::vector<u8> out;
    for (u32 k = 0; k < bytes; k += 2) {
        const u32 v = c.read(0x000, 2);
        out.push_back(static_cast<u8>(v >> 8));
        out.push_back(static_cast<u8>(v));
    }
    return out;
}

struct Sense {
    u8 key, asc, ascq;
};

// REQUEST SENSE, the way a driver learns WHY a command was refused.
Sense requestSense(AtaCell& c, u64* clock = nullptr)
{
    const u8 cdb[12] = {0x03, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0};
    packet(c, cdb, clock);
    REQUIRE((c.read(0x070, 1) & kDrq) != 0);
    const std::vector<u8> d = drain(c, 18);
    return {static_cast<u8>(d[2] & 0x0F), d[12], d[13]};
}

// READ(12) of one block; returns its first byte (the disc's tag).
u8 readBlockTag(AtaCell& c, u32 lba, u64* clock = nullptr)
{
    const u8 cdb[12] = {0xA8, 0, static_cast<u8>(lba >> 24),
                        static_cast<u8>(lba >> 16), static_cast<u8>(lba >> 8),
                        static_cast<u8>(lba), 0, 0, 0, 1, 0, 0};
    c.write(0x040, 0x00, 1); // byte-count limit 0x0800
    c.write(0x050, 0x08, 1);
    packet(c, cdb, clock);
    REQUIRE((c.read(0x070, 1) & kDrq) != 0);
    return drain(c, 2048)[0];
}

bool refused(AtaCell& c) { return (c.read(0x070, 1) & kErr) != 0; }

} // namespace

TEST_CASE("an empty drop-box drive is a drive, and says so honestly")
{
    AtaCell d;
    CHECK(!d.present()); // nothing on the channel until it is seated
    d.seat();
    CHECK(d.present());
    CHECK(!d.mediaPresent());

    // The firmware's probe and the OS's enumeration work without a disc:
    // IDENTIFY PACKET DEVICE and INQUIRY both answer.
    d.write(0x060, 0x00, 1);
    d.write(0x070, 0xA1, 1);
    CHECK((d.read(0x070, 1) & kDrq) != 0);
    const std::vector<u8> id = drain(d, 512);
    CHECK(id[1] == 0x85); // word 0 = 0x8580: ATAPI, CD-ROM, removable
    const u8 inq[12] = {0x12, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0};
    packet(d, inq);
    CHECK((d.read(0x070, 1) & kDrq) != 0);
    CHECK(drain(d, 36)[0] == 0x05); // a CD-ROM device

    // But anything that touches the disc is NOT READY, MEDIUM NOT PRESENT —
    // the sense a driver polls for while it waits for a disc.
    testUnitReady(d);
    CHECK(refused(d));
    CHECK((d.read(0x010, 1) >> 4) == 0x02); // sense key in the error register
    const Sense s = requestSense(d);
    CHECK(s.key == 0x02);
    CHECK(s.asc == 0x3A);
    CHECK(s.ascq == 0x00);
    // REQUEST SENSE itself never needs a disc, and a second one reports
    // nothing pending: the sense was consumed.
    const Sense again = requestSense(d);
    CHECK(again.key == 0x00);
    CHECK(again.asc == 0x00);

    // The pre-existing refusal path is untouched: an unknown opcode that
    // needs no disc is still ILLEGAL REQUEST with ASC 0, exactly as before
    // the media state existed.
    const u8 bogus[12] = {0xD0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    packet(d, bogus);
    CHECK(refused(d));
    const Sense ill = requestSense(d);
    CHECK(ill.key == 0x05);
    CHECK(ill.asc == 0x00);
}

TEST_CASE("a disc that arrives is announced once, with UNIT ATTENTION")
{
    REQUIRE(makeDisc(kDiscA, 0xAA, 4));
    AtaCell d;
    d.seat();
    REQUIRE(d.hostInsert(kDiscA));
    CHECK(d.mediaPresent());
    CHECK(d.unitAttentionPending());
    CHECK(d.insertions() == 1u);

    // INQUIRY does not consume it: a driver identifying the drive must not
    // eat the event the media poll is waiting for.
    const u8 inq[12] = {0x12, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0};
    packet(d, inq);
    CHECK(!refused(d));
    drain(d, 36);
    CHECK(d.unitAttentionPending());

    // The first command that could act on the disc hears about it...
    testUnitReady(d);
    CHECK(refused(d));
    const Sense s = requestSense(d);
    CHECK(s.key == 0x06);
    CHECK(s.asc == 0x28);
    CHECK(!d.unitAttentionPending());

    // ...and only that one. The drive is then simply ready, and serves the
    // disc that was inserted.
    testUnitReady(d);
    CHECK(!refused(d));
    CHECK(readBlockTag(d, 0) == 0xAA);
    CHECK(readBlockTag(d, 3) == 0xAA);
    CHECK(d.readsSinceInsert() == 2u);

    remove(kDiscA);
}

TEST_CASE("a swap waits for the guest to see the empty tray, then a dwell")
{
    REQUIRE(makeDisc(kDiscA, 0xAA, 4));
    REQUIRE(makeDisc(kDiscB, 0xBB, 4));
    AtaCell d;
    u64 clock = 1000;
    d.tbRef = &clock;
    d.cmdDelayTb_ = 100;
    REQUIRE(d.attachIso(kDiscA));
    CHECK(d.present());
    CHECK(d.mediaPresent());
    CHECK(!d.unitAttentionPending()); // the disc the machine powered on with

    testUnitReady(d, &clock);
    CHECK(!refused(d));
    CHECK(readBlockTag(d, 1, &clock) == 0xAA);

    // The host asks for B. The tray opens NOW; B is staged, not inserted.
    const u64 t0 = clock;
    REQUIRE(d.hostSwap(kDiscB));
    CHECK(d.swapPending());
    CHECK(!d.mediaPresent());
    CHECK(d.present()); // the drive never leaves
    // The machine loop must not sleep through the fallback.
    CHECK(d.pendingTb() == t0 + AtaCell::kSwapFallbackTb);

    // Nothing happens on its own before the guest has looked.
    clock = t0 + AtaCell::kSwapDwellTb / 2;
    CHECK(!d.tick());
    CHECK(d.swapPending());
    CHECK(!d.mediaPresent());

    // The guest polls: NOT READY, MEDIUM NOT PRESENT — the absence is seen,
    // and the deadline moves up to the dwell.
    testUnitReady(d, &clock);
    CHECK(refused(d));
    CHECK(d.pendingTb() == t0 + AtaCell::kSwapDwellTb);
    const Sense s = requestSense(d, &clock);
    CHECK(s.key == 0x02);
    CHECK(s.asc == 0x3A);
    CHECK(d.swapPending()); // seen, but the dwell has not passed

    // The dwell is measured from the EJECT.
    clock = t0 + AtaCell::kSwapDwellTb - 1;
    d.tick();
    CHECK(d.swapPending());
    clock = t0 + AtaCell::kSwapDwellTb;
    d.tick();
    CHECK(!d.swapPending());
    CHECK(d.mediaPresent());
    CHECK(d.mediaPath() == kDiscB);
    CHECK(d.insertions() == 1u);
    CHECK(d.unitAttentionPending());
    CHECK(d.pendingTb() == ~0ull); // nothing deferred any more

    // The guest's next poll is told the medium changed, then reads B.
    testUnitReady(d, &clock);
    CHECK(refused(d));
    const Sense ua = requestSense(d, &clock);
    CHECK(ua.key == 0x06);
    CHECK(ua.asc == 0x28);
    testUnitReady(d, &clock);
    CHECK(!refused(d));
    CHECK(readBlockTag(d, 1, &clock) == 0xBB);
    CHECK(d.readsSinceInsert() == 1u);

    remove(kDiscA);
    remove(kDiscB);
}

TEST_CASE("a swap the guest never looks at lands after the fallback")
{
    REQUIRE(makeDisc(kDiscA, 0xAA, 2));
    REQUIRE(makeDisc(kDiscB, 0xBB, 2));
    AtaCell d;
    u64 clock = 5000;
    d.tbRef = &clock;
    REQUIRE(d.attachIso(kDiscA));
    const u64 t0 = clock;
    REQUIRE(d.hostSwap(kDiscB));

    clock = t0 + AtaCell::kSwapFallbackTb - 1;
    d.tick();
    CHECK(d.swapPending());
    CHECK(!d.mediaPresent());

    clock = t0 + AtaCell::kSwapFallbackTb;
    d.tick();
    CHECK(!d.swapPending());
    CHECK(d.mediaPresent());
    CHECK(d.mediaPath() == kDiscB);
    CHECK(d.unitAttentionPending());

    // A swap to an image that will not open leaves the tray EMPTY and says
    // why, rather than quietly keeping the old disc: the guest was already
    // told the tray opened.
    const u64 t1 = clock;
    REQUIRE(d.hostSwap("opm_drop_does_not_exist.iso"));
    clock = t1 + AtaCell::kSwapFallbackTb;
    d.tick();
    CHECK(!d.swapPending());
    CHECK(!d.mediaPresent());
    CHECK(d.present());
    CHECK(std::strlen(d.mediaError()) > 0);
    // ...and the tray is still a tray: the next insert works.
    REQUIRE(d.hostInsert(kDiscA));
    CHECK(d.mediaPresent());

    remove(kDiscA);
    remove(kDiscB);
}

TEST_CASE("the guest's own eject and close work the tray too")
{
    REQUIRE(makeDisc(kDiscA, 0xAA, 2));
    AtaCell d;
    REQUIRE(d.attachIso(kDiscA));

    // PREVENT is recorded, but the disc stays the host's to take.
    const u8 prevent[12] = {0x1E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    packet(d, prevent);
    CHECK(!refused(d));
    CHECK(d.lockedByGuest());
    const u8 allow[12] = {0x1E, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    packet(d, allow);
    CHECK(!d.lockedByGuest());

    // START STOP UNIT, LoEj with Start clear: the OS's Put Away.
    const u8 eject[12] = {0x1B, 0, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0};
    packet(d, eject);
    CHECK(!refused(d));
    CHECK(!d.mediaPresent());
    CHECK(d.guestEjects() == 1u);
    testUnitReady(d);
    CHECK(refused(d));
    CHECK(requestSense(d).asc == 0x3A);

    // LoEj with Start set closes the tray on the disc that was in it, and
    // that is a media change like any other.
    const u8 close[12] = {0x1B, 0, 0, 0, 0x03, 0, 0, 0, 0, 0, 0, 0};
    packet(d, close);
    CHECK(!refused(d));
    CHECK(d.mediaPresent());
    CHECK(d.unitAttentionPending());
    testUnitReady(d);
    CHECK(refused(d));
    CHECK(requestSense(d).asc == 0x28);
    testUnitReady(d);
    CHECK(!refused(d));
    CHECK(readBlockTag(d, 0) == 0xAA);

    // The host can still take the disc from a locked drive — the pinhole.
    packet(d, prevent);
    d.hostEject();
    CHECK(!d.mediaPresent());
    CHECK(d.lockedByGuest());

    remove(kDiscA);
}

TEST_CASE("a staged republish never pulls a disc the guest has read")
{
    REQUIRE(makeDisc(kDiscA, 0xAA, 4));
    REQUIRE(makeDisc(kDiscB, 0xBB, 4));
    AtaCell d;
    u64 clock = 1000;
    d.tbRef = &clock;
    d.cmdDelayTb_ = 100;

    // An empty tray takes the image at once.
    d.seat();
    REQUIRE(d.hostStage(kDiscA));
    CHECK(d.mediaPresent());
    CHECK(!d.swapPending());
    CHECK(d.mediaPath() == kDiscA);

    // A disc the guest never read is swapped the ordinary way: the tray
    // opens now.
    REQUIRE(d.hostStage(kDiscB));
    CHECK(d.swapPending());
    CHECK(!d.mediaPresent());
    CHECK(!d.awaitingGuestEject());
    clock += AtaCell::kSwapFallbackTb;
    d.tick();
    CHECK(d.mediaPresent());
    CHECK(d.mediaPath() == kDiscB);

    // The guest mounts B: UNIT ATTENTION, then a read, then the lock.
    testUnitReady(d, &clock);
    CHECK(refused(d));
    CHECK(requestSense(d, &clock).asc == 0x28);
    CHECK(readBlockTag(d, 1, &clock) == 0xBB);
    const u8 prevent[12] = {0x1E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    packet(d, prevent, &clock);

    // Now a republish must NOT open the tray: the guest's catalog of B is
    // cached, and a swap under it would serve A through B's catalog
    // (measured on the desktop, s45 E3). The image waits.
    REQUIRE(d.hostStage(kDiscA));
    CHECK(d.awaitingGuestEject());
    CHECK(d.swapPending());
    CHECK(d.mediaPresent());
    CHECK(d.mediaPath() == kDiscB);
    CHECK(d.pendingTb() == ~0ull); // nothing deferred: only the guest ends this
    clock += 10 * AtaCell::kSwapFallbackTb;
    CHECK(!d.tick());
    CHECK(d.mediaPresent()); // the fallback does not apply to a disc in use
    CHECK(readBlockTag(d, 2, &clock) == 0xBB);

    // A newer image replaces the waiting one; still nothing moves.
    REQUIRE(d.hostStage(kDiscA));
    CHECK(d.awaitingGuestEject());

    // The guest puts the volume away (ALLOW, then LoEj). The tray is open
    // and the staged disc goes in after the dwell — the E5 path.
    const u8 allow[12] = {0x1E, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    packet(d, allow, &clock);
    const u8 eject[12] = {0x1B, 0, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0};
    packet(d, eject, &clock);
    CHECK(!d.mediaPresent());
    CHECK(d.guestEjects() == 1u);
    CHECK(!d.awaitingGuestEject());
    CHECK(d.swapPending());
    const u64 t0 = clock;
    CHECK(d.pendingTb() == t0 + AtaCell::kSwapFallbackTb);
    testUnitReady(d, &clock); // the driver's poll sees the empty tray
    CHECK(refused(d));
    CHECK(requestSense(d, &clock).asc == 0x3A);
    clock = t0 + AtaCell::kSwapDwellTb;
    d.tick();
    CHECK(!d.swapPending());
    CHECK(d.mediaPresent());
    CHECK(d.mediaPath() == kDiscA);
    testUnitReady(d, &clock);
    CHECK(refused(d));
    CHECK(requestSense(d, &clock).asc == 0x28);
    CHECK(readBlockTag(d, 1, &clock) == 0xAA);

    remove(kDiscA);
    remove(kDiscB);
}

// --- the snapshot section ----------------------------------------------

namespace {

constexpr size_t kRam = 8u << 20;

std::vector<u8> makeRom()
{
    std::vector<u8> rom(SawtoothBus::kRomSize, 0);
    for (size_t k = 0; k < rom.size(); ++k)
        rom[k] = static_cast<u8>(0x5Au ^ (k * 3u));
    rom[0x100] = 0x48; // b . at the reset vector: a machine that idles
    rom[0x101] = 0x00;
    rom[0x102] = 0x00;
    rom[0x103] = 0x00;
    return rom;
}

// Locate a section by tag, scanning from the END so that the tag bytes
// appearing inside RAM or a log cannot be mistaken for framing. Returns the
// offset of the tag, or npos.
size_t findSectionFromEnd(const std::vector<u8>& buf, const char* tag4,
                          size_t before)
{
    for (size_t at = before; at-- > 12;) {
        if (std::memcmp(buf.data() + at, tag4, 4) != 0)
            continue;
        u64 len = 0;
        std::memcpy(&len, buf.data() + at + 4, 8);
        if (at + 12 + len == before)
            return at;
    }
    return std::string::npos;
}

// A bus and its processor are far too big for the stack once a test
// holds more than one machine, so they live on the heap.
struct Machine {
    std::unique_ptr<SawtoothBus> bus;
    std::unique_ptr<Cpu> cpu;
    Machine()
        : bus(std::make_unique<SawtoothBus>(kRam, makeRom())),
          cpu(std::make_unique<Cpu>())
    {
        cpu->attach(*bus);
        cpu->reset();
    }
    AtaCell& drop() { return bus->drop(); }
};

} // namespace

TEST_CASE("the drop drive travels in an optional snapshot section, both ways")
{
    REQUIRE(makeDisc(kDiscA, 0xAA, 4));

    // A machine with the drop drive seated AND a disc in it that the guest
    // has read from; the CD cell has been locked by the guest.
    Machine m;
    REQUIRE(m.bus->attachDrop(kDiscA));
    CHECK(m.drop().present());
    // The bus wires its cells to the machine clock, so commands here are
    // deferred behind BSY; a clock of our own stands in for it.
    u64 clock0 = 0;
    m.drop().tbRef = &clock0;
    m.drop().cmdDelayTb_ = 10;
    testUnitReady(m.drop(), &clock0);
    CHECK(readBlockTag(m.drop(), 2, &clock0) == 0xAA);
    const u8 prevent[12] = {0x1E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    packet(m.drop(), prevent, &clock0);
    CHECK(m.drop().lockedByGuest());

    SnapWriter w;
    const HarnessState h{42ull, 4u, 0ull, 0u, false, false, false};
    saveSnapshot(*m.cpu, *m.bus, h, w);

    // The section sits between R128 and HARN. Find HARN from the end (it
    // is followed only by END), then DROP as the section ending exactly
    // where HARN begins.
    const size_t endAt = w.buf.size() - 12; // "END " + its zero length
    const size_t harnAt = findSectionFromEnd(w.buf, "HARN", endAt);
    REQUIRE(harnAt != std::string::npos);
    const size_t dropAt = findSectionFromEnd(w.buf, "DROP", harnAt);
    REQUIRE(dropAt != std::string::npos);

    SUBCASE("a stream WITH the section: the drive is seated by the loader, "
            "the tray is emptied and the host's disc re-staged")
    {
        // The resuming run seats nothing itself — the section says a drive
        // was enumerated at that boot, so the loader seats one.
        Machine m2;
        REQUIRE(m2.bus->attachDrop(kDiscA)); // the host's disc, as the app would
        HarnessState hr;
        SnapReader r(w.buf.data(), w.buf.size());
        const bool loaded = loadSnapshot(*m2.cpu, *m2.bus, hr, r);
        INFO("load error: " << r.err);
        REQUIRE(loaded);
        CHECK(hr.executed == 42ull);
        CHECK(m2.drop().present());
        // The guest's cached catalog describes whatever was in the tray at
        // snapshot time; the honest resume is an empty tray with the host's
        // disc staged, so the guest remounts it cleanly on its next poll.
        CHECK(!m2.drop().mediaPresent());
        CHECK(m2.drop().swapPending());
        CHECK(m2.drop().lockedByGuest()); // the guest's lock travelled
        CHECK(m2.drop().readsSinceInsert() == 1u);
        // A guest that polls sees the absence, then the disc, then the
        // unit attention — the swap protocol from the top.
        u64 clock = 0;
        m2.drop().tbRef = &clock;
        m2.drop().cmdDelayTb_ = 10;
        testUnitReady(m2.drop(), &clock);
        CHECK(refused(m2.drop()));
        CHECK(requestSense(m2.drop(), &clock).asc == 0x3A);
        clock += AtaCell::kSwapDwellTb;
        m2.drop().tick();
        CHECK(m2.drop().mediaPresent());
        CHECK(m2.drop().unitAttentionPending());
    }

    SUBCASE("a stream WITH the section, loaded into a run that seated no "
            "drive and attached no disc: the drive is still there, empty")
    {
        Machine m2;
        CHECK(!m2.drop().present());
        HarnessState hr;
        SnapReader r(w.buf.data(), w.buf.size());
        const bool loaded = loadSnapshot(*m2.cpu, *m2.bus, hr, r);
        INFO("load error: " << r.err);
        REQUIRE(loaded);
        CHECK(m2.drop().present()); // the guest enumerated it; it must answer
        CHECK(!m2.drop().mediaPresent());
        CHECK(!m2.drop().swapPending());
        u64 clock = 0;
        m2.drop().tbRef = &clock;
        testUnitReady(m2.drop(), &clock);
        CHECK(refused(m2.drop()));
        CHECK(requestSense(m2.drop(), &clock).asc == 0x3A);
    }

    SUBCASE("the SAME stream with the section spliced out — what every "
            "snapshot written before it looks like — loads unchanged")
    {
        std::vector<u8> old(w.buf.begin(), w.buf.begin() + dropAt);
        old.insert(old.end(), w.buf.begin() + harnAt, w.buf.end());
        REQUIRE(old.size() < w.buf.size());
        REQUIRE(findSectionFromEnd(old, "DROP", old.size() - 12) ==
                std::string::npos);

        Machine m2;
        HarnessState hr;
        SnapReader r(old.data(), old.size());
        const bool loaded = loadSnapshot(*m2.cpu, *m2.bus, hr, r);
        INFO("load error: " << r.err);
        REQUIRE(loaded);
        CHECK(hr.executed == 42ull);
        CHECK(!m2.drop().present()); // no third drive in that machine

        // And a stream that is neither — the section tag followed by
        // garbage — is refused, not misread.
        std::vector<u8> bad(w.buf);
        bad[dropAt + 4] ^= 0x01; // corrupt the section length
        Machine m3;
        SnapReader rb(bad.data(), bad.size());
        CHECK(!loadSnapshot(*m3.cpu, *m3.bus, hr, rb));
    }

    remove(kDiscA);
}
