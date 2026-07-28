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
                    b = vr[o + 0];
                    g = vr[o + 1];
                    r = vr[o + 2];
                }
            }
            out[x * 4 + 0] = b;
            out[x * 4 + 1] = g;
            out[x * 4 + 2] = r;
            out[x * 4 + 3] = 0xFF;
        }
    }
    return 1;
}

OPM_API uint64_t opm_executed(const OpmMachine* m) { return m->executed; }
OPM_API uint32_t opm_pc(const OpmMachine* m) { return m->cpu.st.pc; }
