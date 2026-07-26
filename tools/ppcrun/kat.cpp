#include "kat.hpp"
#include "opm/bits.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace opm {

namespace {

class KatBus final : public Bus {
public:
    explicit KatBus(size_t n) : ram_(n, 0) {}
    std::vector<u8> ram_;

    u8 read8(u32 pa) override { return ok(pa, 1) ? ram_[pa] : 0; }
    u16 read16(u32 pa) override
    {
        return ok(pa, 2) ? static_cast<u16>((ram_[pa] << 8) | ram_[pa + 1]) : 0;
    }
    u32 read32(u32 pa) override
    {
        if (!ok(pa, 4))
            return 0;
        return (u32(ram_[pa]) << 24) | (u32(ram_[pa + 1]) << 16) |
               (u32(ram_[pa + 2]) << 8) | u32(ram_[pa + 3]);
    }
    u64 read64(u32 pa) override { return (u64(read32(pa)) << 32) | read32(pa + 4); }
    void write8(u32 pa, u8 v) override { if (ok(pa, 1)) ram_[pa] = v; }
    void write16(u32 pa, u16 v) override
    {
        if (!ok(pa, 2)) return;
        ram_[pa] = static_cast<u8>(v >> 8);
        ram_[pa + 1] = static_cast<u8>(v);
    }
    void write32(u32 pa, u32 v) override
    {
        if (!ok(pa, 4)) return;
        ram_[pa] = static_cast<u8>(v >> 24);
        ram_[pa + 1] = static_cast<u8>(v >> 16);
        ram_[pa + 2] = static_cast<u8>(v >> 8);
        ram_[pa + 3] = static_cast<u8>(v);
    }
    void write64(u32 pa, u64 v) override
    {
        write32(pa, static_cast<u32>(v >> 32));
        write32(pa + 4, static_cast<u32>(v));
    }

private:
    bool ok(u32 pa, u32 n) const { return u64(pa) + n <= ram_.size(); }
};

struct MemExpect {
    u32 addr;
    std::vector<u8> bytes;
};

struct Assign {
    std::string key;
    std::string value;
};

u64 hexval(const std::string& s) { return std::stoull(s, nullptr, 16); }

bool applyKey(Cpu& c, KatBus& bus, const std::string& key, const std::string& val)
{
    if (key.size() >= 2 && key[0] == 'r' && isdigit(static_cast<unsigned char>(key[1]))) {
        c.st.gpr[std::stoul(key.substr(1))] = static_cast<u32>(hexval(val));
        return true;
    }
    if (key.size() >= 2 && key[0] == 'f' && isdigit(static_cast<unsigned char>(key[1]))) {
        c.st.fpr[std::stoul(key.substr(1)) & 31u] = hexval(val);
        return true;
    }
    if (key == "fpscr") { c.st.fpscr = static_cast<u32>(hexval(val)); return true; }
    if (key == "xer") { c.st.xer = static_cast<u32>(hexval(val)); return true; }
    if (key == "cr")  { c.st.cr = static_cast<u32>(hexval(val)); return true; }
    if (key == "lr")  { c.st.lr = static_cast<u32>(hexval(val)); return true; }
    if (key == "ctr") { c.st.ctr = static_cast<u32>(hexval(val)); return true; }
    if (key == "pc")  { c.st.pc = static_cast<u32>(hexval(val)); return true; }
    if (key == "msr") { c.st.msr = static_cast<u32>(hexval(val)); return true; }
    if (key == "srr0") { c.st.srr0 = static_cast<u32>(hexval(val)); return true; }
    if (key == "srr1") { c.st.srr1 = static_cast<u32>(hexval(val)); return true; }
    if (key == "dec") { c.st.dec = static_cast<u32>(hexval(val)); return true; }
    if (key == "dar") { c.st.dar = static_cast<u32>(hexval(val)); return true; }
    if (key == "dsisr") { c.st.dsisr = static_cast<u32>(hexval(val)); return true; }
    if (key == "sdr1") { c.st.sdr1 = static_cast<u32>(hexval(val)); return true; }
    if (key.size() >= 3 && key.compare(0, 2, "sr") == 0 &&
        isdigit(static_cast<unsigned char>(key[2]))) {
        c.st.sr[std::stoul(key.substr(2)) & 15u] = static_cast<u32>(hexval(val));
        return true;
    }
    if (key.size() == 6 && key.compare(1, 3, "bat") == 0 &&
        (key[0] == 'i' || key[0] == 'd') && (key[4] == 'u' || key[4] == 'l') &&
        key[5] >= '0' && key[5] <= '3') { // [id]bat[ul][0-3]
        const u32 n = static_cast<u32>(key[5] - '0');
        u32* reg = key[0] == 'i' ? (key[4] == 'u' ? c.st.ibatu : c.st.ibatl)
                                 : (key[4] == 'u' ? c.st.dbatu : c.st.dbatl);
        reg[n] = static_cast<u32>(hexval(val));
        return true;
    }
    if (key == "resv") {
        c.st.resvValid = true;
        c.st.resvAddr = static_cast<u32>(hexval(val)) & ~31u;
        return true;
    }
    if (key.rfind("mem@", 0) == 0) {
        u32 addr = static_cast<u32>(hexval(key.substr(4)));
        for (size_t i = 0; i + 1 < val.size(); i += 2)
            bus.write8(addr++, static_cast<u8>(std::stoul(val.substr(i, 2), nullptr, 16)));
        return true;
    }
    return false;
}

bool checkKey(Cpu& c, KatBus& bus, const std::string& key, const std::string& val,
              std::string& diff)
{
    auto expectU32 = [&](u32 actual) {
        const u32 want = static_cast<u32>(hexval(val));
        if (actual == want)
            return true;
        char b[96];
        snprintf(b, sizeof b, "  %s: want %08x got %08x\n", key.c_str(), want, actual);
        diff += b;
        return false;
    };
    auto expectU64 = [&](u64 actual) {
        const u64 want = hexval(val);
        if (actual == want)
            return true;
        char b[128];
        snprintf(b, sizeof b, "  %s: want %016llx got %016llx\n", key.c_str(),
                 static_cast<unsigned long long>(want),
                 static_cast<unsigned long long>(actual));
        diff += b;
        return false;
    };
    if (key.size() >= 2 && key[0] == 'r' && isdigit(static_cast<unsigned char>(key[1])))
        return expectU32(c.st.gpr[std::stoul(key.substr(1))]);
    if (key.size() >= 2 && key[0] == 'f' && isdigit(static_cast<unsigned char>(key[1])))
        return expectU64(c.st.fpr[std::stoul(key.substr(1)) & 31u]);
    if (key == "fpscr") return expectU32(c.st.fpscr);
    if (key == "xer") return expectU32(c.st.xer);
    if (key == "cr")  return expectU32(c.st.cr);
    if (key == "lr")  return expectU32(c.st.lr);
    if (key == "ctr") return expectU32(c.st.ctr);
    if (key == "pc")  return expectU32(c.st.pc);
    if (key == "msr") return expectU32(c.st.msr);
    if (key == "srr0") return expectU32(c.st.srr0);
    if (key == "srr1") return expectU32(c.st.srr1);
    if (key == "dec") return expectU32(c.st.dec);
    if (key == "dar") return expectU32(c.st.dar);
    if (key == "dsisr") return expectU32(c.st.dsisr);
    if (key == "resv") {
        const bool want = hexval(val) != 0;
        if (c.st.resvValid == want)
            return true;
        diff += "  resv: want " + val + " got " + (c.st.resvValid ? "1" : "0") + "\n";
        return false;
    }
    if (key == "halt") {
        const bool want = val != "0";
        if (c.halted == want)
            return true;
        diff += "  halt: want " + val + " got " + (c.halted ? "1" : "0") +
                (c.halted ? (" (" + c.haltReason + ")") : "") + "\n";
        return false;
    }
    if (key.rfind("mem@", 0) == 0) {
        u32 addr = static_cast<u32>(hexval(key.substr(4)));
        bool okAll = true;
        for (size_t i = 0; i + 1 < val.size(); i += 2, ++addr) {
            const u8 want = static_cast<u8>(std::stoul(val.substr(i, 2), nullptr, 16));
            const u8 got = bus.read8(addr);
            if (want != got) {
                char b[96];
                snprintf(b, sizeof b, "  mem@%08x: want %02x got %02x\n", addr, want, got);
                diff += b;
                okAll = false;
            }
        }
        return okAll;
    }
    diff += "  (unknown expect key " + key + ")\n";
    return false;
}

int runFile(const fs::path& file, int& totalTests)
{
    std::ifstream in(file);
    if (!in) {
        fprintf(stderr, "kat: cannot open %s\n", file.string().c_str());
        return 1;
    }

    int failures = 0;
    std::string line, name;
    std::vector<Assign> inits, expects;
    u32 insn = 0;
    bool haveInsn = false;

    auto runOne = [&]() {
        ++totalTests;
        KatBus bus(1u << 20);
        Cpu cpu;
        cpu.attach(bus);
        cpu.reset();
        cpu.st.pc = 0x1000;
        cpu.st.msr = 0; // flat supervisor-off state for UISA tests

        for (const Assign& a : inits)
            if (!applyKey(cpu, bus, a.key, a.value))
                fprintf(stderr, "kat: %s: bad init key %s\n", name.c_str(), a.key.c_str());
        bus.write32(cpu.st.pc, insn);

        cpu.step();

        std::string diff;
        bool pass = true;
        bool expectsHalt = false;
        for (const Assign& a : expects) {
            if (a.key == "halt" && a.value != "0")
                expectsHalt = true;
            if (!checkKey(cpu, bus, a.key, a.value, diff))
                pass = false;
        }
        if (cpu.halted && !expectsHalt) {
            pass = false;
            diff += "  unexpected halt: " + cpu.haltReason + "\n";
        }
        if (!pass) {
            ++failures;
            fprintf(stderr, "FAIL %s [%s] insn=%08x\n%s", name.c_str(),
                    file.filename().string().c_str(), insn, diff.c_str());
        }
    };

    while (std::getline(in, line)) {
        if (const size_t h = line.find('#'); h != std::string::npos)
            line.resize(h);
        std::istringstream ss(line);
        std::string tok;
        if (!(ss >> tok))
            continue;
        if (tok == "test") {
            ss >> name;
            inits.clear();
            expects.clear();
            haveInsn = false;
        } else if (tok == "insn") {
            std::string w;
            ss >> w;
            insn = static_cast<u32>(hexval(w));
            haveInsn = true;
        } else if (tok == "expect") {
            std::string kv;
            while (ss >> kv) {
                const size_t eq = kv.find('=');
                expects.push_back({kv.substr(0, eq), kv.substr(eq + 1)});
            }
        } else if (tok == "end") {
            if (haveInsn)
                runOne();
        } else {
            // init line: first token is already a key=value
            std::string kv = tok;
            do {
                const size_t eq = kv.find('=');
                if (eq != std::string::npos)
                    inits.push_back({kv.substr(0, eq), kv.substr(eq + 1)});
            } while (ss >> kv);
        }
    }
    return failures;
}

} // namespace

int runKats(const char* path)
{
    std::vector<fs::path> files;
    fs::path p(path);
    if (fs::is_directory(p)) {
        for (const auto& e : fs::directory_iterator(p))
            if (e.path().extension() == ".kat")
                files.push_back(e.path());
    } else {
        files.push_back(p);
    }

    int failures = 0, total = 0;
    for (const auto& f : files)
        failures += runFile(f, total);
    printf("kat: %d tests, %d failed (%zu files)\n", total, failures, files.size());
    return failures;
}

} // namespace opm
