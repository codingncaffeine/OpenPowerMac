#pragma once
#include <cstdint>
#include <cstddef>

namespace opm {

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i8  = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

// AltiVec vector register. b[0] is the byte at the LOWEST effective address of a
// 16-byte aligned load, i.e. the most-significant end: element 0 of any element
// size starts at b[0] (big-endian element numbering per the AltiVec PEM).
struct V128 {
    u8 b[16]{};
};

} // namespace opm
