# BSPDecompiler

This is the native C++17 command-line replacement for the Java VMEX GUI.

## Build on Windows

Install CMake and a C++17 compiler, then run these commands from this folder:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

The executable statically links the C/C++ runtime (`/MT` with MSVC, or static
GCC and C++ standard libraries with MinGW). Windows system libraries remain
Windows DLL dependencies; Windows does not support statically linking those
operating-system components into an application.

Drag one or more `.bsp` files onto the resulting `bspdecompiler.exe`, or run:

```powershell
.\build\Release\bspdecompiler.exe "C:\maps\example.bsp"
```

The program writes `example_decompiled.vmf` beside the input map. Its default
brush-based VMF writer emits world and entity brushes, displacements, entity
keyvalues/connections, detail brushes, overlays, static props, and cubemaps.
It accepts uncompressed BSP versions 17, 19, and 20; compressed BSP lumps and
the legacy face-based decompilation modes are not supported.
