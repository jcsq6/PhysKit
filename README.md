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

## Camera Animation Tracks

The graphics testing framework includes a camera animation system for creating smooth camera movements through keyframes.

### Basic Example

```cpp
#include "camera.h"
using namespace mp_units::si::unit_symbols;
using namespace graphics;

// Build a track from keyframes, then configure interpolation/extrapolation.
cam.set_move_track(
    graphics::camera_track({
        graphics::kf::make_pos({0.0f * m, 1.0f * m, -5.0f * m})
            .look_at({0.0f * m, 0.0f * m, 0.0f * m})
            .transition(2.0f * s),
        graphics::kf::make_pos({5.0f * m, 2.0f * m, -3.0f * m})
            .transition(1.5f * s),
        graphics::kf::make_pos({0.0f * m, 1.0f * m, 5.0f * m})
            .dir({0.0f, 0.0f, 1.0f})
            .transition(2.0f * s),
    })
        .with_interp(graphics::camera_track::spline)
        .with_extrap(graphics::camera_track::release));
```

### Keyframe Options

Each keyframe can specify:
- **Position**: `.pos(vec3)` - Camera position in world space
- **Orientation** (choose one):
  - `.look_at(vec3)` - Point camera toward a target position
  - `.dir(vec3)` - Face camera in a specific direction
  - `.orient(Quaternion)` - Explicit quaternion orientation
- **Transition**: `.transition(duration)` - Time to interpolate to next keyframe

You can use the factory helpers:
- `kf::make_pos(position)`
- `kf::make_look_at(target)`
- `kf::make_dir(direction)`
- `kf::make_orient(quaternion)`

### Interpolation Modes

```cpp
// Step function (no interpolation)
cam.set_move_track(
    camera_track({...keyframes...}).with_interp(camera_track::constant));

// Linear interpolation
cam.set_move_track(
    camera_track({...keyframes...}).with_interp(camera_track::linear));

// Smooth cubic spline (recommended, default)
cam.set_move_track(camera_track({...keyframes...}));
// or explicitly:
cam.set_move_track(
    camera_track({...keyframes...}).with_interp(camera_track::spline));
```

### Extrapolation Modes

```cpp
// Default: release control when track duration is exceeded
cam.set_move_track(
    camera_track({...keyframes...}).with_extrap(camera_track::release));

// Loop from end back to beginning
cam.set_move_track(
    camera_track({...keyframes...}).with_extrap(camera_track::loop));

// Ping-pong between endpoints
cam.set_move_track(
    camera_track({...keyframes...}).with_extrap(camera_track::reverse));
```

### Manual Track Sampling

If you need to manually sample the track (without applying it to a camera), create a named track:

```cpp
// Create a track for manual sampling
camera_track track({
    kf::make_pos({0.0f * m, 0.0f * m, 0.0f * m}).transition(1.0f * s),
    kf::make_pos({5.0f * m, 0.0f * m, 0.0f * m}).transition(1.0f * s)
});

// Sample at any time
auto [pos, rot] = track.at(current_time);
// Use pos (Vector3) and rot (Quaternion) as needed
```

### Requirements

- At least **two keyframes** with position data are required
- Keyframes without explicit orientation derive it from movement direction
- If no orientation keyframes are provided at all, the track enters **freelook** mode (position is animated, rotation remains user-controlled)
- Track durations are cumulative based on transition times

### Complete Example

See [tests/graphics_demo/main.cpp](tests/graphics_demo/main.cpp) for a working demonstration of camera tracks in action, featuring a smooth camera path that visits multiple points of interest while maintaining focus on different targets.

# Contributing
See the following pages for dependency documentation:
- [mp-units](https://mpusz.github.io/mp-units/latest/)
- [Eigen](https://devdocs.io/eigen3/)
