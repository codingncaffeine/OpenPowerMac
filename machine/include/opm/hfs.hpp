#pragma once
#include <string>

namespace opm {

// 📁 → 💿  Build a classic HFS volume image from a host folder.
//
// The file-transfer story for a Mac OS 9 guest: the user points the shell at
// a folder, and at machine start the folder's contents are packed into an
// HFS volume and attached in the CD slot — the guest mounts it like any
// inserted disc, read-only, and files come across by drag-and-drop in the
// Finder. Read-only is the honest shape of the feature: the image is built
// fresh from the folder at every start, so the folder is the single source
// of truth and nothing ever has to merge two writable views of it.
//
// Classic HFS rather than HFS+, deliberately: the format is fully documented
// (Inside Macintosh: Files), a cleanly-built read-only volume needs no
// journal and no wrapper, and every Mac OS from 7 up mounts it. The 65,535
// allocation-block ceiling sets the size law: block size scales up with
// content, 512 bytes at the small end.
//
// What a host file becomes:
// - `name.bin` in valid MacBinary II dress decodes fully: both forks, the
//   real Mac name, type/creator and Finder flags. The lossless container —
//   what download archives use — so it round-trips exactly.
// - any other file: the bytes are the data fork; an NTFS alternate stream
//   `name:rsrc` (what 7-Zip writes when it extracts Mac media) becomes the
//   resource fork; type/creator come from a small extension map, with a PEF
//   sniff ('Joy!peff') promoting executables to APPL so a bare-copied
//   PowerPC app still double-clicks.
// - folders nest; dot-prefixed entries and Windows litter (desktop.ini,
//   Thumbs.db) are skipped.
//
// The recommended transfer form for anything fork-critical remains a .sit
// archive expanded in the guest — StuffIt carries the forks internally and
// the guest's own Expander restores them bit-exact.
//
// Returns true and writes `outPath` (padded to a 2048 multiple, ready for
// the ATAPI slot); false with `err` naming the first thing that refused.
bool hfsBuildImage(const std::string& folder, const std::string& outPath,
                   const std::string& volName, std::string& err);

} // namespace opm
