// mkhfs — pack a host folder into a classic HFS volume image.
//
// The command-line face of the shared-folder builder (opm/hfs.hpp): the
// shell calls the same code through the capi at machine start; this exists
// for headless runs and for inspecting what the guest will be handed.
//
//   mkhfs <folder> <out.img> [volume name]
//
// ⚠ On Windows the narrow argv arrives in the ANSI code page, and a name
// like "Nanosaur™" turns into bytes that are not valid UTF-8 — which the
// path layer rejects by throwing, which a naive main turns into a 0xC0000409
// fail-fast with no message at all. The wide entry point plus an explicit
// UTF-8 conversion is the whole fix.

#include "opm/hfs.hpp"

#include <cstdio>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>

static std::string utf8(const wchar_t* w)
{
    const int n =
        WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    std::string s(n > 0 ? n - 1 : 0, 0);
    if (n > 1)
        WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr,
                            nullptr);
    return s;
}

int wmain(int argc, wchar_t** argv)
{
    std::vector<std::string> a;
    for (int i = 0; i < argc; ++i)
        a.push_back(utf8(argv[i]));
#else
int main(int argc, char** argv)
{
    std::vector<std::string> a(argv, argv + argc);
#endif
    if (a.size() < 3) {
        std::fprintf(stderr, "usage: mkhfs <folder> <out.img> [name]\n");
        return 2;
    }
    const std::string name = a.size() > 3 ? a[3] : "Shared";
    std::string err, warn;
    const bool ok = opm::hfsBuildImage(a[1], a[2], name, err, &warn);
    if (!ok)
        std::fprintf(stderr, "mkhfs: %s\n", err.c_str());
    // Files the share had to leave out (oversized forks): on success,
    // silence would read as "everything is in"; on failure, the refusal's
    // arithmetic only adds up next to what was already left out.
    if (!warn.empty())
        std::fprintf(stderr, "mkhfs: %s", warn.c_str());
    return ok ? 0 : 1;
}
