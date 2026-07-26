#include "doctest.h"
#include "opm/bits.hpp"

using namespace opm;

TEST_CASE("ppcbits extracts PPC-numbered fields (bit 0 = MSB)")
{
    CHECK(ppcbits(0xFFFFFFFFu, 0, 5) == 63u);
    CHECK(ppcbits(0x80000000u, 0, 0) == 1u);
    CHECK(ppcbits(0x00000001u, 31, 31) == 1u);
    CHECK(ppcbits(0x7C221A14u, 0, 5) == 31u);   // primary opcode of add
    CHECK(ppcbits(0x7C221A14u, 21, 30) == 266u); // XO of add
    CHECK(ppcbits(0x7C221A14u, 6, 10) == 1u);
    CHECK(ppcbits(0x7C221A14u, 11, 15) == 2u);
    CHECK(ppcbits(0x7C221A14u, 16, 20) == 3u);
}

TEST_CASE("ppcmask handles normal and wrapped ranges")
{
    CHECK(ppcmask(0, 31) == 0xFFFFFFFFu);
    CHECK(ppcmask(25, 31) == 0x7Fu);
    CHECK(ppcmask(0, 0) == 0x80000000u);
    CHECK(ppcmask(31, 31) == 0x00000001u);
    CHECK(ppcmask(29, 2) == 0xE0000007u); // wrapped (mb > me)
}

TEST_CASE("sign extension helpers")
{
    CHECK(sext16(0x8000u) == -32768);
    CHECK(sext16(0x7FFFu) == 32767);
    CHECK(sext26(0x2000000u) == -33554432);
    CHECK(sext14(0x2000u) == -8192);
    CHECK(sext5(0x10u) == -16);
    CHECK(sext5(0x0Fu) == 15);
}

TEST_CASE("mfspr split field")
{
    CHECK(f_spr(0x7C0802A6u) == 8u); // mflr r0
}
