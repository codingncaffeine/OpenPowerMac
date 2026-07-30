#include "opm/capi.h"

#include "opm/cpu.hpp"
#include "opm/sawtooth.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace opm;

struct OpmMachine {
    SawtoothBus* bus = nullptr;
    Cpu cpu;
    // 60x bus snooping for every DMA engine in the machine. Wired before
    // the first instruction: a DBDMA channel with no snoop responder reads
    // its descriptor list out of RAM while the firmware's stores are still
    // sitting dirty in the L1, and transfers into memory the processor is
    // still caching.
    CpuSnoop snoop;
    uint64_t executed = 0;
    uint32_t fastTb = 0;
    size_t consoleAt = 0; // drained-up-to mark

    ~OpmMachine() { delete bus; }
};

static std::vector<u8> slurp(const char* path)
{
    std::vector<u8> v;
    FILE* f = fopen(path, "rb");
    if (!f)
        return v;
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    v.assign(static_cast<size_t>(n > 0 ? n : 0), 0);
    if (!v.empty() && fread(v.data(), 1, v.size(), f) != v.size())
        v.clear();
    fclose(f);
    return v;
}

OPM_API OpmMachine* opm_create(const char* romPath, const char* cdPath,
                               const char* hdPath, const char* atiRomPath,
                               uint32_t ramMb, uint32_t fastTb)
{
    std::vector<u8> rom = slurp(romPath);
    if (rom.size() != SawtoothBus::kRomSize)
        return nullptr;
    OpmMachine* m = new OpmMachine();
    m->bus = new SawtoothBus(size_t(ramMb ? ramMb : 256) * 1024 * 1024,
                             std::move(rom));
    if (cdPath && *cdPath)
        m->bus->attachCd(cdPath);
    if (hdPath && *hdPath)
        m->bus->attachHd(hdPath);
    if (atiRomPath && *atiRomPath)
        m->bus->attachAtiRom(atiRomPath);
    m->fastTb = fastTb;
    m->cpu.attach(*m->bus);
    m->snoop.cpu = &m->cpu;
    m->bus->attachSnoop(&m->snoop);
    m->cpu.reset();
    m->bus->pcRef = &m->cpu.st.pc;
    m->bus->stamp = &m->executed;
    m->bus->cd().stamp = &m->executed;
    m->bus->pic().stamp = &m->executed;
    m->bus->pmu().tbRef = &m->cpu.st.tb;
    m->bus->ataDma().stamp = &m->executed;
    m->bus->ataDma().pcRef = &m->cpu.st.pc;
    for (u32 f = 0; f < 2; ++f) {
        m->bus->ohci(f).stamp = &m->executed;
        m->bus->ohci(f).pcRef = &m->cpu.st.pc;
    }
    m->bus->ati().stamp = &m->executed;
    m->bus->ati().pcRef = &m->cpu.st.pc;
    return m;
}

OPM_API void opm_destroy(OpmMachine* m) { delete m; }

OPM_API void opm_ati_at(OpmMachine* m, uint64_t insn)
{
    m->bus->atiVisibleAt = insn;
}

OPM_API uint64_t opm_run(OpmMachine* m, uint64_t insns)
{
    const uint64_t until = m->executed + insns;
    Cpu& cpu = m->cpu;
    SawtoothBus& bus = *m->bus;
    while (m->executed < until && !cpu.halted) {
        cpu.step();
        if (m->fastTb)
            cpu.tick(m->fastTb);
        bus.ohciTick(cpu.st.tb);
        bus.syncIrqs();
        cpu.setExternalIrq(bus.pic().cpuLine());
        ++m->executed;
    }
    return m->executed;
}

OPM_API void opm_serial(OpmMachine* m, const char* text)
{
    if (text && *text)
        m->bus->injectSerial(text);
}

OPM_API void opm_key(OpmMachine* m, const char* text)
{
    if (text && *text)
        m->bus->ohci(0).typeAscii(text);
}

OPM_API void opm_key_event(OpmMachine* m, uint32_t usage, uint32_t down)
{
    m->bus->ohci(0).keyEvent(static_cast<uint8_t>(usage), down != 0u);
}

OPM_API void opm_mouse(OpmMachine* m, int32_t dx, int32_t dy,
                       uint32_t buttons)
{
    m->bus->ohci(1).moveMouse(dx, dy, static_cast<uint8_t>(buttons));
}

OPM_API uint32_t opm_console(OpmMachine* m, char* buf, uint32_t cap)
{
    const std::string& con = m->bus->console();
    if (m->consoleAt > con.size())
        m->consoleAt = con.size(); // defensive
    uint32_t n = 0;
    while (m->consoleAt < con.size() && n + 1 < cap)
        buf[n++] = con[m->consoleAt++];
    if (cap)
        buf[n] = 0;
    return n;
}

OPM_API int32_t opm_screen(OpmMachine* m, uint8_t* bgra, uint32_t cap,
                           uint32_t* w, uint32_t* h)
{
    R128Cell& ati = m->bus->ati();
    const u32 gen = ati.peek(0x0050);
    if (!(gen & 0x02000000u))
        return -1;
    const u32 ht = ati.peek(0x0200);
    const u32 vt = ati.peek(0x0208);
    const u32 pitch8 = ati.peek(0x022C) & 0xFFFFu;
    const u32 offset = ati.peek(0x0224);
    const u32 fmt = (gen >> 8) & 0xFu;
    const u32 sw = (((ht >> 16) & 0x3FFu) + 1u) * 8u;
    const u32 sh = ((vt >> 16) & 0xFFFu) + 1u;
    if (sw < 64 || sw > 2048 || sh < 64 || sh > 1536 ||
        (fmt != 2u && fmt != 6u))
        return -1;
    if (w)
        *w = sw;
    if (h)
        *h = sh;
    if (!bgra || cap < sw * sh * 4u)
        return 0;
    const u32 bypp = fmt == 2u ? 1u : 4u;
    const u32 rowBytes = pitch8 * 8u * bypp;
    const auto& vr = ati.vram;
    for (u32 y = 0; y < sh; ++y) {
        uint8_t* out = bgra + size_t(y) * sw * 4u;
        const size_t row = offset + size_t(y) * rowBytes;
        for (u32 x = 0; x < sw; ++x) {
            uint8_t b = 0, g = 0, r = 0;
            const size_t o = row + size_t(x) * bypp;
            if (o + bypp <= vr.size()) {
                if (fmt == 2u) {
                    const u32 c = ati.pal(vr[o]);
                    r = static_cast<uint8_t>(c >> 16);
                    g = static_cast<uint8_t>(c >> 8);
                    b = static_cast<uint8_t>(c);
                } else {
                    // A Mac 32-bpp pixel is big-endian xRGB: byte 0 is the
                    // unused/alpha lane and R, G, B follow. Reading bytes
                    // 0,1,2 as B,G,R put the unused lane in blue and shifted
                    // the other two, so a 50% grey desktop (00 80 80 80) came
                    // out as r=80 g=80 b=00 -- olive. The whole screen was
                    // yellow, which is what named the bug.
                    r = vr[o + 1];
                    g = vr[o + 2];
                    b = vr[o + 3];
                }
            }
            out[x * 4 + 0] = b;
            out[x * 4 + 1] = g;
            out[x * 4 + 2] = r;
            out[x * 4 + 3] = 0xFF;
        }
    }

    // The hardware cursor. Mac OS draws its pointer with the card's cursor
    // engine, not into the framebuffer, so a scanout that reads VRAM alone
    // shows no pointer at all however well the USB side works -- which is
    // exactly what it looked like from the outside.
    //
    // 64x64 at two bits per pixel, 16 bytes per row: eight bytes of AND bits
    // then eight of XOR bits, each big-endian with the most significant bit
    // leftmost. AND=0 selects colour 0 or 1 from XOR; AND=1 with XOR=0 is
    // transparent, and with XOR=1 complements what is underneath -- which is
    // how the classic arrow keeps its black outline over any background.
    // CUR_HORZ_VERT_OFF is how many cursor rows/columns to skip, which is how
    // the driver clips the pointer against the top and left edges.
    if (ati.peek(0x0050) & 0x00010000u) { // CRTC_GEN_CNTL: cursor enable
        const u32 curOff = ati.peek(0x0260) & 0x07FFFFFFu;
        const u32 posn = ati.peek(0x0264);
        const u32 hvoff = ati.peek(0x0268);
        const u32 clr[2] = {ati.peek(0x026C), ati.peek(0x0270)};
        const u32 cx = (posn >> 16) & 0xFFFFu, cy = posn & 0xFFFFu;
        const u32 ox = (hvoff >> 16) & 0x3Fu, oy = hvoff & 0x3Fu;
        for (u32 row = oy; row < 64u; ++row) {
            const u32 sy = cy + (row - oy);
            if (sy >= sh)
                break;
            const size_t so = size_t(curOff) + size_t(row) * 16u;
            if (so + 16u > vr.size())
                break;
            uint64_t abits = 0, xbits = 0;
            for (u32 k = 0; k < 8; ++k) {
                abits = (abits << 8) | vr[so + k];
                xbits = (xbits << 8) | vr[so + 8u + k];
            }
            for (u32 col = ox; col < 64u; ++col) {
                const u32 sx = cx + (col - ox);
                if (sx >= sw)
                    break;
                const uint64_t bit = uint64_t(1) << (63u - col);
                uint8_t* px = bgra + (size_t(sy) * sw + sx) * 4u;
                if (abits & bit) {
                    if (!(xbits & bit))
                        continue; // transparent
                    px[0] = static_cast<uint8_t>(~px[0]);
                    px[1] = static_cast<uint8_t>(~px[1]);
                    px[2] = static_cast<uint8_t>(~px[2]);
                } else {
                    const u32 c = clr[(xbits & bit) ? 1 : 0];
                    px[0] = static_cast<uint8_t>(c);
                    px[1] = static_cast<uint8_t>(c >> 8);
                    px[2] = static_cast<uint8_t>(c >> 16);
                }
                px[3] = 0xFF;
            }
        }
    }
    return 1;
}

OPM_API uint64_t opm_executed(const OpmMachine* m) { return m->executed; }
OPM_API uint32_t opm_pc(const OpmMachine* m) { return m->cpu.st.pc; }
