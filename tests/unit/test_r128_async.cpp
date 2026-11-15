// The card's command engine on its own thread (opm/r128.hpp): a stream handed
// to it must leave the card exactly as the same stream executed where it was
// submitted, whatever the guest does while it runs.
//
// Every case builds one stream and runs it twice — the engine's thread off,
// then on — and compares all of video memory and the registers the stream
// wrote. r128AsyncHold makes "queued, and none of it executed yet" a state the
// case stands in, so nothing here depends on which thread gets there first.

#include "doctest.h"
#include "opm/cpu.hpp"
#include "opm/r128.hpp"

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using namespace opm;

namespace {

u32 be(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}
u32 f2u(float f)
{
    u32 u;
    std::memcpy(&u, &f, 4);
    return u;
}

// 8 MB of "system RAM" with a GART table at 1 MB; indirect bodies live in AGP
// space at kBody, which the GART maps onto RAM at kBodyPa.
struct Rig {
    R128Cell c;
    std::vector<u8> ram;
    static constexpr u32 kGart = 0x00100000u;
    static constexpr u32 kBody = 0x00010000u, kBodyPa = 0x00200000u;

    Rig() : ram(8u << 20, 0)
    {
        r128CceReset();
        for (u32 page = 0; page < 64u; ++page)
            put32(kGart + ((kBody >> 12) + page) * 4u,
                  ((kBodyPa + (page << 12)) & 0xFFFFF000u) | 1u);
        // 64x64 at 32 bpp from VRAM 0, scissor open, pipe on, solid shading.
        reg(0x146Cu, (1u << 30) | (1u << 28) | 0x0Fu | (6u << 8));
        reg(0x142Cu, (64u / 8u) << 21);
        reg(0x16ECu, 0);
        reg(0x16F0u, (63u << 16) | 63u);
        reg(0x1CA0u, 2u << 8);
        reg(0x071Cu, (3u << 1) | (3u << 3) | (2u << 5));
    }
    CceMem mem() { return CceMem{ram.data(), static_cast<u32>(ram.size()), kGart, 0, nullptr}; }
    void put32(u32 at, u32 v)
    {
        ram[at] = static_cast<u8>(v);
        ram[at + 1] = static_cast<u8>(v >> 8);
        ram[at + 2] = static_cast<u8>(v >> 16);
        ram[at + 3] = static_cast<u8>(v >> 24);
    }
    void reg(u32 off, u32 v) { c.write(off, be(v), 4); }
    void fifo(u32 w)
    {
        r128CceFifoWord(c, be(w), 4, ram.data(), static_cast<u32>(ram.size()),
                        kGart, 0, nullptr);
    }
    // Put `words` in AGP space at `slot` and dispatch them the way the
    // driver does: one Type-0 packet over PM4_IW_INDOFF / PM4_IW_INDSIZE.
    void dispatch(u32 slot, const std::vector<u32>& words)
    {
        const u32 agp = kBody + slot * 0x8000u;
        for (size_t i = 0; i < words.size(); ++i)
            put32(kBodyPa + slot * 0x8000u + static_cast<u32>(i) * 4u, words[i]);
        fifo(0x00010000u | (0x0738u >> 2));
        fifo(agp);
        fifo(static_cast<u32>(words.size()));
    }
    void drain() { r128AsyncDrain(c, mem()); }
};

// A stream of `n` Gouraud triangles in changing colors and places, with a
// fence in GUI_SCRATCH_REG0 written after every eighth, and the fence value
// `mark` at its very end. ~18 words a triangle.
std::vector<u32> stream(u32 n, u32 mark)
{
    std::vector<u32> w;
    auto vtx = [&](float x, float y, u32 argb) {
        w.push_back(f2u(x));
        w.push_back(f2u(y));
        w.push_back(f2u(0.5f));
        w.push_back(f2u(1.0f));
        w.push_back(argb);
    };
    for (u32 i = 0; i < n; ++i) {
        w.push_back(0xC0002500u | ((2u + 15u - 1u) << 16)); // 3D_RNDR_GEN_PRIM
        w.push_back(0x9u);                                  // RHW | DIFFUSE_ARGB
        w.push_back(4u | (3u << 4) | (3u << 16));           // list, ring walk
        const float ox = static_cast<float>((i * 7u) % 40u);
        const float oy = static_cast<float>((i * 11u) % 40u);
        vtx(ox, oy, 0xFF000000u | (i * 0x030507u));
        vtx(ox + 24, oy, 0xFF000000u | (i * 0x070305u));
        vtx(ox, oy + 24, 0xFF000000u | (i * 0x050703u));
        if ((i & 7u) == 7u) {
            w.push_back(0x15E0u >> 2); // Type-0, one register
            w.push_back(0x1000u + i);
        }
    }
    w.push_back(0x15E0u >> 2);
    w.push_back(mark);
    return w;
}

// What a run leaves behind that a guest could read.
struct Result {
    std::vector<u8> vram;
    u32 scratch = 0;
    u64 tris = 0, pixels = 0;
};
Result finish(Rig& r, const R128EngStats& before)
{
    Result out;
    out.vram.assign(r.c.vram.begin(), r.c.vram.begin() + 64 * 64 * 4);
    out.scratch = r.c.peek(0x15E0u);
    out.tris = r128EngStats().tris - before.tris;
    out.pixels = r128EngStats().triPixels - before.triPixels;
    return out;
}

// The reference: the same streams, the engine's thread off.
Result reference(const std::vector<std::vector<u32>>& streams)
{
    r128AsyncEnable(false);
    Rig r;
    const R128EngStats before = r128EngStats();
    for (u32 k = 0; k < streams.size(); ++k)
        r.dispatch(k, streams[k]);
    return finish(r, before);
}

bool waitFor(bool (*cond)(), int ms = 5000)
{
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (!cond()) {
        if (std::chrono::steady_clock::now() > until)
            return false;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
}

struct AsyncOff { // every case leaves the process the way the other tests expect it
    ~AsyncOff()
    {
        r128AsyncHold(false);
        r128AsyncEnable(false);
    }
};

} // namespace

TEST_CASE("engine thread: a handed-over stream leaves the card as the "
          "synchronous engine does")
{
    AsyncOff restore;
    const std::vector<std::vector<u32>> s = {stream(120, 0xFEED0001u)};
    const Result want = reference(s);
    REQUIRE(want.tris == 120);
    REQUIRE(want.scratch == 0xFEED0001u);

    r128AsyncEnable(true);
    Rig r;
    const R128EngStats before = r128EngStats();
    const R128AsyncStats st0 = r128AsyncStats();
    r128AsyncHold(true);
    r.dispatch(0, s[0]);
    // Handed over, and with the thread held nothing of it has run.
    CHECK(r128AsyncStats().handOffs == st0.handOffs + 1);
    CHECK(r128AsyncBusy());
    CHECK(r128EngStats().tris == before.tris);
    r128AsyncHold(false);
    REQUIRE(waitFor([] { return !r128AsyncBusy(); }));
    CHECK(r128AsyncStats().workerPkts - st0.workerPkts == 120);
    const Result got = finish(r, before);
    CHECK(got.tris == want.tris);
    CHECK(got.pixels == want.pixels);
    CHECK(got.scratch == want.scratch);
    CHECK(got.vram == want.vram);
}

TEST_CASE("engine thread: a marker in the stream is not there to be read "
          "before the triangles ahead of it are drawn")
{
    // A driver ends a buffer with a write to a scratch register and polls
    // for it: when the value appears, the card is done with the buffer, and
    // the driver goes on to change the textures and vertices the buffer
    // used. Large triangles are queued for the drawing threads and the
    // packet loop runs ahead of them, so the marker's write has to wait for
    // the queue. Here the guest's side polls as a driver does, and at the
    // moment it reads the marker the picture must be the finished one.
    AsyncOff restore;
    constexpr u32 kW = 512, kH = 256, kMark = 0xD0C0FFEEu;
    const auto target = [](Rig& r) {
        r.reg(0x142Cu, (kW / 8u) << 21);
        r.reg(0x16F0u, ((kH - 1u) << 16) | (kW - 1u));
        r.reg(0x1C9Cu, 1u << 9); // blended: every triangle reads what is there
        r.reg(0x1CA0u, (2u << 8) | (4u << 16) | (5u << 20));
    };
    std::vector<u32> w;
    for (u32 i = 0; i < 90u; ++i) {
        w.push_back(0xC0002500u | ((2u + 15u - 1u) << 16));
        w.push_back(0x9u);
        w.push_back(4u | (3u << 4) | (3u << 16));
        const float ox = static_cast<float>((i * 37u) % 120u);
        const float oy = static_cast<float>((i * 23u) % 60u);
        const auto vtx = [&](float x, float y, u32 argb) {
            w.push_back(f2u(x));
            w.push_back(f2u(y));
            w.push_back(f2u(0.5f));
            w.push_back(f2u(1.0f));
            w.push_back(argb);
        };
        vtx(ox, oy, 0x60000000u | (i * 0x030507u));
        vtx(ox + 380.0f, oy + 10.0f, 0x90000000u | (i * 0x070305u));
        vtx(ox + 20.0f, oy + 190.0f, 0xC0000000u | (i * 0x050703u));
    }
    w.push_back(0x15E0u >> 2);
    w.push_back(kMark);

    r128AsyncEnable(false);
    r128SetRasterThreads(1);
    std::vector<u8> want;
    {
        Rig r;
        target(r);
        r.dispatch(0, w);
        REQUIRE(r.c.peek(0x15E0u) == kMark);
        want.assign(r.c.vram.begin(), r.c.vram.begin() + kW * kH * 4u);
    }
    for (int round = 0; round < 3; ++round) {
        r128AsyncEnable(true);
        r128SetRasterThreads(4);
        Rig r;
        target(r);
        const R128AsyncStats st0 = r128AsyncStats();
        const u64 queued0 = r128EngStats().crewTris;
        r.dispatch(0, w);
        REQUIRE(r128AsyncStats().handOffs == st0.handOffs + 1);
        const auto until =
            std::chrono::steady_clock::now() + std::chrono::seconds(20);
        u32 seen = 0;
        while (seen != kMark && std::chrono::steady_clock::now() < until) {
            R128RegLock lock; // a guest's read of the register
            seen = r.c.peek(0x15E0u);
        }
        REQUIRE(seen == kMark);
        // The marker is the last thing in the stream: nothing draws now.
        const std::vector<u8> got(r.c.vram.begin(),
                                  r.c.vram.begin() + kW * kH * 4u);
        CHECK(got == want);
        REQUIRE(waitFor([] { return !r128AsyncBusy(); }));
        CHECK(r128EngStats().crewTris - queued0 >= 80u);
    }
    r128SetRasterThreads(0);
}

TEST_CASE("engine thread: a short stream is executed where it is submitted")
{
    AsyncOff restore;
    r128AsyncEnable(true);
    Rig r;
    const R128AsyncStats st0 = r128AsyncStats();
    const R128EngStats before = r128EngStats();
    r.dispatch(0, stream(4, 0xFEED0002u)); // ~80 words: under the hand-off size
    CHECK(r128AsyncStats().handOffs == st0.handOffs);
    CHECK(!r128AsyncBusy());
    CHECK(r128EngStats().tris - before.tris == 4);
    CHECK(r.c.peek(0x15E0u) == 0xFEED0002u);
}

TEST_CASE("engine thread: draining executes what is queued, here, in order")
{
    AsyncOff restore;
    const std::vector<std::vector<u32>> s = {stream(120, 0xFEED0003u)};
    const Result want = reference(s);

    r128AsyncEnable(true);
    Rig r;
    const R128EngStats before = r128EngStats();
    const R128AsyncStats st0 = r128AsyncStats();
    r128AsyncHold(true);
    r.dispatch(0, s[0]);
    REQUIRE(r128AsyncBusy());
    r.drain(); // the thread is held: every packet runs on this one
    CHECK(!r128AsyncBusy());
    CHECK(r128AsyncStats().workerPkts == st0.workerPkts);
    CHECK(r128AsyncStats().drainPkts - st0.drainPkts == 120);
    const Result got = finish(r, before);
    CHECK(got.tris == want.tris);
    CHECK(got.scratch == want.scratch);
    CHECK(got.vram == want.vram);
}

TEST_CASE("engine thread: words written behind a running engine wait their "
          "turn, and the dispatch among them is fetched by this thread")
{
    AsyncOff restore;
    // Two streams back to back. The second paints over the first, so the
    // result says which order they ran in.
    const std::vector<std::vector<u32>> s = {stream(100, 0xFEED0004u),
                                             stream(90, 0xFEED0005u)};
    const Result want = reference(s);
    REQUIRE(want.scratch == 0xFEED0005u);

    r128AsyncEnable(true);
    Rig r;
    const R128EngStats before = r128EngStats();
    const R128AsyncStats st0 = r128AsyncStats();
    r128AsyncHold(true);
    r.dispatch(0, s[0]);
    REQUIRE(r128AsyncBusy());
    r.dispatch(1, s[1]); // three FIFO words, behind the first stream
    CHECK(r128AsyncStats().inboxWords - st0.inboxWords == 3);
    CHECK(r128AsyncFifoQueued() == 3);
    CHECK(r128AsyncStats().handOffs == st0.handOffs + 1);
    CHECK(r.c.peek(0x15E0u) == 0); // nothing has run
    r128AsyncHold(false);
    // The engine runs the first stream and stops at the words behind it:
    // what they dispatch is in memory this thread has to read.
    REQUIRE(waitFor([] { return r128AsyncNeedsService(); }));
    CHECK(r128EngStats().tris - before.tris == 100);
    r128AsyncService(r.c, r.mem());
    CHECK(r128AsyncStats().services == st0.services + 1);
    CHECK(r128AsyncStats().handOffs == st0.handOffs + 2);
    REQUIRE(waitFor([] { return !r128AsyncBusy(); }));
    const Result got = finish(r, before);
    CHECK(got.tris == want.tris);
    CHECK(got.pixels == want.pixels);
    CHECK(got.scratch == want.scratch);
    CHECK(got.vram == want.vram);
}

TEST_CASE("engine thread: the status words say busy and how much FIFO is "
          "left, and the other registers are reachable meanwhile")
{
    AsyncOff restore;
    r128AsyncEnable(true);
    Rig r;
    r.reg(0x0050u, 0x12345678u); // a CRTC register: nothing to do with the engine
    CHECK(be(r.c.read(0x07B8u, 4)) == 0x40u); // idle: 64 free, no busy bit
    r128AsyncHold(true);
    r.dispatch(0, stream(60, 0xFEED0006u));
    REQUIRE(r128AsyncBusy());
    r.fifo(0x80000000u); // two fillers behind it
    r.fifo(0x80000000u);
    {
        R128RegLock lock; // what the bus takes for a read that does not drain
        const u32 st = be(r.c.read(0x07B8u, 4));
        CHECK((st & 0xFFFu) == 0x40u - 2u);
        CHECK((st & 0x00010000u) != 0);
        CHECK((st & 0x80000000u) != 0);
        CHECK(be(r.c.read(0x1740u, 4)) == st);
        CHECK(be(r.c.read(0x0050u, 4)) == 0x12345678u);
        // The fence the stream ends with has not been reached.
        CHECK(be(r.c.read(0x15E0u, 4)) == 0);
    }
    r128AsyncHold(false);
    r.drain();
    CHECK(be(r.c.read(0x07B8u, 4)) == 0x40u);
    CHECK(be(r.c.read(0x15E0u, 4)) == 0xFEED0006u);
}

TEST_CASE("engine thread: a card that goes away takes its queue with it")
{
    AsyncOff restore;
    r128AsyncEnable(true);
    const R128EngStats before = r128EngStats();
    {
        Rig r;
        r128AsyncHold(true);
        r.dispatch(0, stream(60, 0xFEED0007u));
        REQUIRE(r128AsyncBusy());
        r128AsyncAbandon(&r.c);
        CHECK(!r128AsyncBusy());
    }
    r128AsyncHold(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    CHECK(r128EngStats().tris == before.tris); // and none of it ever ran
    // The next machine starts clean.
    Rig r2;
    r2.dispatch(0, stream(60, 0xFEED0008u));
    r2.drain();
    CHECK(r2.c.peek(0x15E0u) == 0xFEED0008u);
}

TEST_CASE("publish: modified cache lines reach memory and the caches do not "
          "change")
{
    Cpu c;
    std::vector<u8> ram(1u << 20, 0xEE);
    auto fill = [](u8* b, u8 v) {
        for (u32 k = 0; k < 32; ++k)
            b[k] = static_cast<u8>(v + k);
    };
    // L1: a modified line at 0x00012340 (set 0x1A, tag 0x12), a clean one at
    // 0x00023360, and a modified one past the end of this RAM.
    fill(c.l1d[0x1A][2].b, 0x10);
    c.l1d[0x1A][2].d = true;
    c.l1x[0x1A].tv[2] = (0x12u << 1) | 1u;
    fill(c.l1d[0x1B][0].b, 0x40);
    c.l1d[0x1B][0].d = false;
    c.l1x[0x1B].tv[0] = (0x23u << 1) | 1u;
    fill(c.l1d[0x05][7].b, 0x70);
    c.l1d[0x05][7].d = true;
    c.l1x[0x05].tv[7] = (0x7FFFFu << 1) | 1u;
    // L2: a modified line of its own at 0x00040000, and an OLDER modified
    // copy of the L1's block, which must not be the one left standing.
    c.l2.assign(4, Cpu::L2Line{});
    c.l2[0].v = c.l2[0].d = true;
    c.l2[0].tag = 0x00040000u >> 5;
    fill(c.l2[0].b, 0xA0);
    c.l2[1].v = c.l2[1].d = true;
    c.l2[1].tag = 0x00012340u >> 5;
    fill(c.l2[1].b, 0xC0);
    c.l2[2].v = true; // valid, not modified
    c.l2[2].tag = 0x00050000u >> 5;
    fill(c.l2[2].b, 0xD0);

    c.publishModified(ram.data(), static_cast<u32>(ram.size()));

    CHECK(ram[0x00012340] == 0x10); // the L1's bytes, not the L2's 0xC0
    CHECK(ram[0x0001235F] == 0x10 + 31);
    CHECK(ram[0x00040000] == 0xA0);
    CHECK(ram[0x00023360] == 0xEE); // clean lines are already in memory
    CHECK(ram[0x00050000] == 0xEE);
    CHECK(ram[0x0001233F] == 0xEE); // and nothing either side of a line
    CHECK(ram[0x00012360] == 0xEE);
    // Still modified, still valid: a publish is not a flush.
    CHECK(c.l1d[0x1A][2].d);
    CHECK(c.l1x[0x1A].tv[2] == ((0x12u << 1) | 1u));
    CHECK(c.l2[0].v);
    CHECK(c.l2[0].d);
    CHECK(c.l2[1].d);
}
