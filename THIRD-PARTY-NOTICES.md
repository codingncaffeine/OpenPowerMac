# Third-party notices

OpenPowerMac is MIT licensed (see `LICENSE`). The emulator itself, in `core/`,
`machine/` and `capi/`, contains no third-party code. The Linux packages carry
the following components beside it, each under its own license.

## In the release packages

| Component | Used for | License |
|---|---|---|
| .NET runtime (self-contained publish) | runs the app | MIT, Copyright (c) .NET Foundation and Contributors |
| Avalonia 12, with MicroCom.Runtime and Tmds.DBus.Protocol | the window, menus and dialogs | MIT, Copyright (c) AvaloniaUI OÜ and contributors |
| SkiaSharp | drawing | MIT, Copyright (c) Xamarin, Inc. and Microsoft Corporation |
| Skia (inside `libSkiaSharp.so`) | drawing | BSD-3-Clause, Copyright (c) Google Inc. |
| HarfBuzzSharp | text shaping | MIT, Copyright (c) Xamarin, Inc. and Microsoft Corporation |
| HarfBuzz (inside `libHarfBuzzSharp.so`) | text shaping | MIT ("Old MIT"), Copyright (c) the HarfBuzz authors |
| Inter typeface | the app's font | SIL Open Font License 1.1, Copyright (c) The Inter Project Authors |
| SDL 3 (`libSDL3.so.0`, in the tarballs and the `.deb`) | sound output | zlib license, below |
| GCC runtime (`libstdc++`, `libgcc`, linked into `libopmcapi.so`) | the core's C++ runtime | GPL-3.0 with the GCC Runtime Library Exception |

Each project's full license text is in its own repository:
<https://github.com/dotnet/runtime>, <https://github.com/AvaloniaUI/Avalonia>,
<https://github.com/mono/SkiaSharp>, <https://skia.org>,
<https://github.com/harfbuzz/harfbuzz>, <https://github.com/rsms/inter>,
<https://github.com/libsdl-org/SDL>, <https://gcc.gnu.org/onlinedocs/libstdc++/manual/license.html>.

### SDL 3 (zlib license)

```
Copyright (C) 1997-2026 Sam Lantinga <slouken@libsdl.org>

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
```

The SDL 3 in the packages is built from the unmodified 3.4.16 release archive
by `packaging/native/build.sh`.

## In the source tree only

- doctest (`tests/vendor/doctest.h`), the unit tests' framework: MIT license,
  Copyright (c) 2016-2023 Viktor Kirilov. It is not part of any package.
