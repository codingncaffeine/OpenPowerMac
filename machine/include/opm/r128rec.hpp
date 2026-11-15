#pragma once
#include "opm/bus.hpp"
#include "opm/r128.hpp"

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

// 🎞 The frame recorder: the card's inputs over a few frames, kept in a file,
// and replayed into a card on its own.
//
// A change to the rasteriser is judged by drawing the SAME thing before and
// after, and a running guest cannot draw the same thing twice: under host
// pacing no two runs take the same path, and some programs do not draw at
// all when paced by the instruction count. A recording is the input itself.
// It holds the card as it stood when the recording began (registers, video
// memory, palettes, and the engine's state outside the cell) and then, in
// order, everything that reached the card from outside:
//
//   W  a store to a register, and what the card does as a bus master because
//      of it (a FIFO word, an indirect dispatch, a ring doorbell);
//   w  a store to a register through the indexed I/O window (no bus-master
//      consequence);
//   R  a read of a register (for the reads that change something);
//   F  a store by the processor into video memory;
//   P  a page of system memory, as it stood the first time the engine read it
//      since the processor could last have changed it;
//   G  the GART and aperture bases, when they change.
//
// Replay applies them to a fresh cell with the engine on the calling thread.
// Nothing of the machine is needed: system memory is a zeroed buffer the P
// events fill in.
//
// FRAME EDGES. The engine tells whoever listens (gR128FrameEdge) when a frame
// is complete: after a copy that covers at least half of the surface being
// displayed (a back buffer presented, 'P'), before a fill that does (a
// single-buffered frame about to be wiped, 'C'), and when the display is
// pointed at another surface ('F'). While recording, each edge's video-memory
// hash goes into the file (E) and the recording ends with the store that
// produced the last frame asked for; the hash at the very end is Z. A replay
// hears the same edges from the same code, so its hashes can be compared with
// the file's: equal means the replayed card drew exactly what the live one
// did, and after a change to the rasteriser the two pictures at one edge are
// the before and after of one input.
//
// While a recording runs, the engine's own thread is off (the caller drains it
// first): the engine then reads system memory at the moment of the store that
// asks, which is the only moment a recording can name.

extern bool gR128RecOn;
inline bool r128RecOn() { return gR128RecOn; }

// Begin recording `frames` frames into `path`. The engine must be idle (the
// bus drains it). False with `err` when the file cannot be written or a
// recording is already running.
bool r128RecStart(R128Cell& c, const CceMem& m, u32 frames,
                  const std::string& path, std::string& err);
// End a recording early; what was captured so far is a valid file.
void r128RecStop(R128Cell& c, const char* why);

struct R128RecStatus {
    bool on = false;       // recording now
    bool finished = false; // a recording has ended since the last start
    u32 framesWanted = 0, frames = 0;
    u64 events = 0, pages = 0, bytes = 0;
    std::string path, note;
};
R128RecStatus r128RecStatus();

// The bus's side. A register store is bracketed: the pages the engine reads
// while carrying it out are written to the file ahead of it.
void r128RecRegWrite(R128Cell& c, char kind, u32 off, u32 v, u32 len,
                     const CceMem* m);
void r128RecRegWriteDone(R128Cell& c);
void r128RecRegRead(u32 off, u32 len);
void r128RecFbWrite(u32 off, u32 v, u32 len);
// The engine's side: it is about to read [pa, pa+n) of system memory. The
// page is kept whole, and the processor's caches may still hold newer bytes
// of it than memory does (the engine snoops only the line it reads), so the
// first read under each store has every modified line copied out to memory
// first (SnoopSink::publish, which changes nothing in the caches).
void r128RecTouch(const u8* ram, u32 ramSize, u32 pa, u32 n, SnoopSink* snoop);
// Frame edges, to whoever listens (a recording, a replay). Null otherwise.
extern void (*gR128FrameEdge)(R128Cell& c, char kind);

// A guest store to the card's register block, whole: the cell's write, what
// the card does as a bus master because of it, the recorder told. The bus
// and a replay both go through here, so they cannot drift apart.
void r128GuestRegWrite(R128Cell& c, u32 off, u32 v, u32 len, const CceMem& m);

// The engine's state outside the cell (r128.cpp, r128_3d.cpp).
void r128EngineStateSave(SnapWriter& w);
void r128EngineStateLoad(SnapReader& r);
void r128Cce3dStateSave(SnapWriter& w);
void r128Cce3dStateLoad(SnapReader& r);

u64 r128VramHash(const R128Cell& c);

// Replay a recording into `c`. `onEdge` is called at every frame edge, in
// order, with the cell as it stands there. The report says how the replay's
// edges and end compare with the recording's.
struct R128ReplayReport {
    u64 events = 0, pages = 0;
    u32 edges = 0;        // edges the replay heard
    u32 edgesWanted = 0;  // edges in the recording
    u32 edgesEqual = 0;   // ...whose kind and video memory matched, in order
    bool endRecorded = false, endEqual = false;
    u64 endHash = 0;
    // Each edge as (kind, video-memory hash): the replay's, the recording's.
    std::vector<std::pair<char, u64>> heard, wanted;
};
using R128EdgeFn = std::function<void(u32 index, char kind, const R128Cell& c)>;
bool r128Replay(const std::string& path, R128Cell& c, const R128EdgeFn& onEdge,
                R128ReplayReport& rep, std::string& err);

} // namespace opm
