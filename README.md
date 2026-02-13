# PhysKit
3D general purpose physics engine / simulation library

# Dependencies
Depends on:
- `mp-units`

# Build
With Conan, run:  
```sh
conan remote add conan-mpusz https://mpusz.jfrog.io/artifactory/api/conan/conan-oss
```

Ensure you have a compliant C++26 conan profile, and then run:
```sh
conan build -pr <myprof|default> . -s build_type=<Release|Debug> --build=missing -u
```

## Windows (MSYS2)

Install [MSYS2](https://www.msys2.org/) and a C++26-capable GCC from one of its environments (e.g. ucrt64):

```sh
pacman -S mingw-w64-ucrt-x86_64-gcc
```

Create or edit your Conan profile (`conan profile path default`):

```ini
[settings]
arch=x86_64
build_type=Release
compiler=gcc
compiler.cppstd=26
compiler.version=15
compiler.libcxx=libstdc++11
os=Windows

[conf]
tools.cmake.cmaketoolchain:generator=Ninja
tools.build:compiler_executables={"c":"YOUR_PATH_TO_MSYS2/ucrt64/bin/gcc.exe","cpp":"YOUR_PATH_TO_MSYS2/ucrt64/bin/g++.exe"}
```

Adjust the compiler paths to match your MSYS2 installation.

**Important notes:**
- `compiler.libcxx` must be `libstdc++11` (the new ABI). The old `libstdc++` ABI is not compatible with mp-units.
- If you have multiple MSYS2 environments installed (e.g. both mingw64 and ucrt64), make sure the environment you are building with appears **before** the others in your system PATH. Mismatched runtime DLLs will cause silent build failures.

# Usage

> **See the [Graphics & Demo Framework documentation](tests/README.md)** for a comprehensive guide covering the configuration API (JSON + CLI + builder), the `graphics_app` base class, camera controls, animation tracks, scene objects, and mesh helpers.

### Complete Example

See [tests/graphics_demo/main.cpp](tests/graphics_demo/main.cpp) for a working demonstration of camera tracks in action, featuring a smooth camera path that visits multiple points of interest while maintaining focus on different targets.

# Contributing

## Setup

After cloning, enable the repo's Git hooks:
```sh
git config core.hooksPath hooks/
```

This installs a pre-commit hook that auto-formats all source files before each commit.

## References
See the following pages for dependency documentation:
- [mp-units](https://mpusz.github.io/mp-units/latest/)
- [Eigen](https://devdocs.io/eigen3/)
