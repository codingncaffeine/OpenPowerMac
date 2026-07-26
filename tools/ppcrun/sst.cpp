// SST-PPC chapter runner: replays generated single-instruction vectors and
// compares the full final state. The JSON parser is deliberately minimal —
// it reads exactly the schema sstgen emits (objects, arrays, unsigned
// decimal numbers, strings).

#include "sst.hpp"
#include "opm/cpu.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace opm {

namespace {

// ---- micro JSON ------------------------------------------------------------

struct JVal {
    enum Kind { Num, Str, Arr, Obj } kind = Num;
    u64 num = 0;
    std::string str;
    std::vector<JVal> arr;
    std::vector<std::pair<std::string, JVal>> obj;

    const JVal* get(const std::string& k) const
    {
        for (const auto& [key, v] : obj)
            if (key == k)
                return &v;
        return nullptr;
    }
};

struct JParser {
    const char* p;
    const char* end;

    void ws()
    {
        while (p < end && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t'))
            ++p;
    }
    bool eat(char c)
    {
        ws();
        if (p < end && *p == c) {
            ++p;
            return true;
        }
        return false;
    }
    JVal parse()
    {
        ws();
        JVal v;
        if (p >= end)
            return v;
        if (*p == '{') {
            ++p;
            v.kind = JVal::Obj;
            if (!eat('}')) {
                do {
                    ws();
                    JVal key = parse(); // string
                    eat(':');
                    v.obj.emplace_back(key.str, parse());
                } while (eat(','));
                eat('}');
            }
        } else if (*p == '[') {
            ++p;
            v.kind = JVal::Arr;
            if (!eat(']')) {
                do {
                    v.arr.push_back(parse());
                } while (eat(','));
                eat(']');
            }
        } else if (*p == '"') {
            ++p;
            v.kind = JVal::Str;
            while (p < end && *p != '"')
                v.str.push_back(*p++);
            if (p < end)
                ++p;
        } else {
            v.kind = JVal::Num;
            while (p < end && (*p == '-' || (*p >= '0' && *p <= '9')))
                v.num = v.num * 10 + static_cast<u64>(*p++ - '0');
        }
        return v;
    }
};

// ---- sparse bus ------------------------------------------------------------

class MapBus final : public Bus {
public:
    std::map<u32, u8> mem;

    u8 rd(u32 a)
    {
        auto it = mem.find(a);
        return it == mem.end() ? 0 : it->second;
    }
    u8 read8(u32 pa) override { return rd(pa); }
    u16 read16(u32 pa) override { return static_cast<u16>((rd(pa) << 8) | rd(pa + 1)); }
    u32 read32(u32 pa) override
    {
        return (u32(rd(pa)) << 24) | (u32(rd(pa + 1)) << 16) | (u32(rd(pa + 2)) << 8) |
               u32(rd(pa + 3));
    }
    u64 read64(u32 pa) override { return (u64(read32(pa)) << 32) | read32(pa + 4); }
    void write8(u32 pa, u8 v) override { mem[pa] = v; }
    void write16(u32 pa, u16 v) override
    {
        write8(pa, static_cast<u8>(v >> 8));
        write8(pa + 1, static_cast<u8>(v));
    }
    void write32(u32 pa, u32 v) override
    {
        write8(pa, static_cast<u8>(v >> 24));
        write8(pa + 1, static_cast<u8>(v >> 16));
        write8(pa + 2, static_cast<u8>(v >> 8));
        write8(pa + 3, static_cast<u8>(v));
    }
    void write64(u32 pa, u64 v) override
    {
        write32(pa, static_cast<u32>(v >> 32));
        write32(pa + 4, static_cast<u32>(v));
    }
};

void applyState(Cpu& c, MapBus& bus, const JVal& s)
{
    c.st.pc = static_cast<u32>(s.get("pc")->num);
    const JVal* g = s.get("gprs");
    for (int i = 0; i < 32; ++i)
        c.st.gpr[i] = static_cast<u32>(g->arr[static_cast<size_t>(i)].num);
    c.st.cr = static_cast<u32>(s.get("cr")->num);
    c.st.xer = static_cast<u32>(s.get("xer")->num);
    c.st.lr = static_cast<u32>(s.get("lr")->num);
    c.st.ctr = static_cast<u32>(s.get("ctr")->num);
    c.st.msr = static_cast<u32>(s.get("msr")->num);
    c.st.srr0 = static_cast<u32>(s.get("srr0")->num);
    c.st.srr1 = static_cast<u32>(s.get("srr1")->num);
    c.st.dec = static_cast<u32>(s.get("dec")->num);
    c.st.tb = s.get("tb")->num;
    const JVal* rv = s.get("resv");
    c.st.resvValid = rv->arr[0].num != 0;
    c.st.resvAddr = static_cast<u32>(rv->arr[1].num);
    for (const JVal& pair : s.get("ram")->arr)
        bus.mem[static_cast<u32>(pair.arr[0].num)] =
            static_cast<u8>(pair.arr[1].num);
}

bool checkState(Cpu& c, MapBus& bus, const JVal& s, std::string& diff)
{
    bool ok = true;
    auto chk = [&](const char* name, u32 want, u32 got) {
        if (want != got) {
            char b[96];
            snprintf(b, sizeof b, "  %s: want %08x got %08x\n", name, want, got);
            diff += b;
            ok = false;
        }
    };
    chk("pc", static_cast<u32>(s.get("pc")->num), c.st.pc);
    const JVal* g = s.get("gprs");
    for (int i = 0; i < 32; ++i) {
        char nm[8];
        snprintf(nm, sizeof nm, "r%d", i);
        chk(nm, static_cast<u32>(g->arr[static_cast<size_t>(i)].num), c.st.gpr[i]);
    }
    chk("cr", static_cast<u32>(s.get("cr")->num), c.st.cr);
    chk("xer", static_cast<u32>(s.get("xer")->num), c.st.xer);
    chk("lr", static_cast<u32>(s.get("lr")->num), c.st.lr);
    chk("ctr", static_cast<u32>(s.get("ctr")->num), c.st.ctr);
    chk("msr", static_cast<u32>(s.get("msr")->num), c.st.msr);
    chk("srr0", static_cast<u32>(s.get("srr0")->num), c.st.srr0);
    chk("srr1", static_cast<u32>(s.get("srr1")->num), c.st.srr1);
    chk("dec", static_cast<u32>(s.get("dec")->num), c.st.dec);
    chk("tb.lo", static_cast<u32>(s.get("tb")->num), static_cast<u32>(c.st.tb));
    const JVal* rv = s.get("resv");
    chk("resv.valid", static_cast<u32>(rv->arr[0].num), c.st.resvValid ? 1u : 0u);
    if (rv->arr[0].num)
        chk("resv.addr", static_cast<u32>(rv->arr[1].num), c.st.resvAddr);
    for (const JVal& pair : s.get("ram")->arr) {
        const u32 a = static_cast<u32>(pair.arr[0].num);
        chk("ram", static_cast<u32>(pair.arr[1].num), bus.rd(a));
    }
    return ok;
}

int runFile(const fs::path& file, int& total, int shown, int& failures)
{
    std::ifstream in(file, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    JParser jp{text.c_str(), text.c_str() + text.size()};
    const JVal doc = jp.parse();
    const JVal* tests = doc.get("tests");
    if (!tests) {
        fprintf(stderr, "sst: %s: no tests\n", file.string().c_str());
        return 1;
    }

    int fileFails = 0;
    for (const JVal& t : tests->arr) {
        ++total;
        MapBus bus;
        Cpu cpu;
        cpu.attach(bus);
        cpu.reset();
        cpu.st.msr = 0;
        applyState(cpu, bus, *t.get("initial"));
        cpu.step();
        std::string diff;
        bool ok = !cpu.halted && checkState(cpu, bus, *t.get("final"), diff);
        if (cpu.halted)
            diff += "  halted: " + cpu.haltReason + "\n";
        if (!ok) {
            ++failures;
            ++fileFails;
            if (failures <= shown)
                fprintf(stderr, "FAIL %s: %s\n%s", file.filename().string().c_str(),
                        t.get("name")->str.c_str(), diff.c_str());
        }
    }
    return fileFails;
}

} // namespace

int runSst(const char* path)
{
    std::vector<fs::path> files;
    fs::path p(path);
    if (fs::is_directory(p)) {
        for (const auto& e : fs::directory_iterator(p))
            if (e.path().extension() == ".json")
                files.push_back(e.path());
    } else {
        files.push_back(p);
    }

    int total = 0, failures = 0;
    for (const auto& f : files)
        runFile(f, total, 20, failures);
    printf("sst: %d tests, %d failed (%zu chapters)\n", total, failures,
           files.size());
    return failures;
}

} // namespace opm
