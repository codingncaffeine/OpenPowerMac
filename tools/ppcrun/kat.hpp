#pragma once
#include "opm/cpu.hpp"

namespace opm {

// Run every *.kat file at `path` (file or directory). Returns number of
// failures; prints a per-file summary and a diff for each failing test.
//
// Format (hand-authorable; '#' comments):
//   test <name>
//   [init lines]  key=HEXVALUE ...   keys: rN, xer, cr, lr, ctr, pc,
//                                    resv (reservation address), mem@ADDR=HEXBYTES
//   insn HEXWORD
//   expect key=HEXVALUE ...          same keys; only listed keys are checked;
//                                    halt=1 expects the instruction to halt
//   end
int runKats(const char* path);

} // namespace opm
