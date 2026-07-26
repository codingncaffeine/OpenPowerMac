#pragma once
// Arc 2 M1: PCI configuration space behind the Grackle's CONFIG_ADDR /
// CONFIG_DATA pair. Three devices populate the Gossamer's bus 0:
//   dev 0x00  MPC106 "Grackle" host bridge (1057:0002) — config registers
//             stored with read-back so the ROM's PICR-style programming
//             sticks;
//   dev 0x10  "Heathrow" mac-io (106B:0010), BAR0 preset to 0xF3000000;
//   dev 0x12  ATI 3D Rage Pro (1002:4750), a 16 MB memory aperture BAR the
//             firmware sizes and assigns — the route to first light.
// Config registers are little-endian bytes; CONFIG_DATA is a 4-byte window
// (the CPU reads it with byte-reversed loads). BAR sizing implements the
// write-ones-read-mask protocol. RECEIPT: device numbers follow the common
// Gossamer IDSEL wiring; the ROM's probe traffic is logged to re-pin them
// if its expectations differ.

#include "opm/types.hpp"

#include <map>
#include <vector>

namespace opm {

class PciConfig {
public:
    PciConfig();

    // CONFIG_ADDR as written by the CPU (value already byte-swapped to LE
    // meaning by the caller).
    void setAddr(u32 le) { addr_ = le; }
    u32 addr() const { return addr_; }

    u8 readData(u32 lane);          // lane = low 2 bits of the data address
    void writeData(u32 lane, u8 v);

    // The ATI memory aperture as currently programmed (0 = not assigned).
    u32 atiBase() const;
    u32 atiIoBase() const; // BAR1 (I/O), 0 = not assigned
    static constexpr u32 kAtiAperture = 0x01000000u; // 16 MB

    struct Probe {
        u64 reads = 0, writes = 0;
    };
    const std::map<u32, Probe>& probeLog() const { return probeLog_; }

private:
    struct Dev {
        u8 cfg[256] = {};
        u32 barMask[6] = {};
    };
    Dev* find(u32 devNum);
    static void put16(Dev& d, u32 off, u16 v);
    static void put32(Dev& d, u32 off, u32 v);

    std::map<u32, Dev> devs_;
    u32 addr_ = 0;
    std::map<u32, Probe> probeLog_;
};

} // namespace opm
