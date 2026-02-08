# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PhysKit is a 3D general purpose physics engine/simulation library written in modern C++26. It uses compile-time unit checking via `mp-units` and Eigen for linear algebra, with optional Magnum graphics for visualization.

## Build System

### Prerequisites
- C++26-compliant Clang compiler (required - see cmake/EnsureClang.cmake)
- Conan package manager
- CMake 3.40+

### Initial Setup
```sh
# Add mp-units remote
conan remote add conan-mpusz https://mpusz.jfrog.io/artifactory/api/conan/conan-oss

# Build with Conan (Debug or Release)
conan build -pr <profile|default> . -s build_type=<Release|Debug> --build=missing -u
```

### Common Commands
```sh
# Configure with compile commands (for IDE support)
cmake --preset conan-debug-compile-commands

# Build the project
cmake --build build/Debug

# Generate API documentation (requires Doxygen)
cmake --build build/Debug --target docs

# Format code
clang-format -i <files>
```

### Build Options
- `PHYSKIT_BUILD_GRAPHICS`: Include graphical tests/demos (enabled by default)
- `ENABLE_DOCS`: Enable Doxygen documentation generation (enabled by default)

## Architecture

### Core Physics Library

**Location**: `include/physkit/`, `src/physkit.cpp`

The core library provides unit-aware physics primitives:

- **`physkit::particle`**: Represents a physical particle with position, velocity, acceleration, and mass. All quantities use compile-time unit checking via `mp-units`.

- **`physkit::integrator`**: Abstract base class for numerical integration schemes. Implementations include:
  - `forward_euler`: Basic forward Euler integration
  - `semi_implicit_euler`: Semi-implicit (symplectic) Euler integration

- **Unit-Aware Types**: The library defines custom type aliases that combine Eigen matrices with mp-units:
  - `vec3<unit>`: 3D vector with compile-time units
  - `mat3<unit>`, `mat4<unit>`: 3×3 and 4×4 matrices with units
  - Implementation via `unit_mat` wrapper (see `include/physkit/detail/lin_alg.h`)

### Graphics Testing Framework

**Location**: `tests/include/`, `tests/demo/`, `tests/graphics_demo/`

A Magnum-based graphics framework for visual testing and demonstrations:

- **`graphics::graphics_app`**: Base application class providing:
  - Scene graph management via Magnum's SceneGraph
  - Camera control with WASD movement, mouse look
  - Instanced rendering for efficient drawing of many objects
  - Frame timing and vsync control

- **`graphics::camera`**: First-person camera with:
  - Quaternion-based rotation
  - Track-based animation support (position interpolation)
  - Unit-aware movement speed

- **Object Hierarchy**:
  - `gfx_obj`: Base graphics object with transformation
  - `physics_obj`: Combines graphics with `physkit::object`
  - `instanced_drawable`: Efficiently renders multiple instances
  - `colored_drawable`: Drawable with configurable color

- **Conversion Utilities** (`tests/include/convert.h`): Bridge between physkit and Magnum types

### Type System Philosophy

PhysKit uses **compile-time dimensional analysis** throughout:
- All physical quantities are strongly typed with their units (e.g., `quantity<si::metre>`)
- Unit mismatches cause compilation errors, not runtime bugs
- `using namespace mp_units;` and `using namespace si::unit_symbols;` are common in implementation files
- The `vec3`, `mat3`, `mat4` templates embed unit information in the matrix type itself

## Code Style

- **Formatting**: Uses clang-format with LLVM base style (see `.clang-format`)
  - 4-space indentation, Allman braces
  - 100 character line limit
- **Naming**: Snake_case for functions/variables, PascalCase for types
- **Member Prefix**: Private/protected members prefixed with `M_`
- **Includes**: Sorted with IWYU pragmas where needed

## Key Dependencies

- **mp-units**: Type-safe physical units library ([docs](https://mpusz.github.io/mp-units/latest/))
- **Eigen3**: Linear algebra library ([docs](https://devdocs.io/eigen3/))
- **Magnum**: Graphics/visualization library (optional, for tests only)
- **GLFW**: Window/input handling (optional, for graphics)

## Development Notes

### Working with Units
```cpp
// Good: explicit units
using namespace mp_units::si::unit_symbols;
physkit::vec3<si::metre> pos = {1.0 * m, 2.0 * m, 3.0 * m};
auto speed = 5.0 * m / s;

// Unit conversions are explicit
float value_in_meters = pos.x().numerical_value_in(si::metre);
```

### Extending Integrators
Derive from `physkit::integrator` and implement `integrate(particle&, quantity<si::second>)`:
```cpp
class my_integrator : public physkit::integrator {
    void integrate(particle& p, quantity<si::second> dt) override {
        // Your integration logic
    }
};
```

### Graphics Demos
Subclass `graphics::graphics_app` and override:
- `update(quantity<si::second> dt)`: Update physics/logic each frame
- `key_press(KeyEvent&, bool)`: Handle keyboard input
- `pointer_move/press(...)`: Handle mouse/pointer input

Objects are added via `add_object(...)` which handles mesh registration and instanced rendering automatically.

## Important Constraints

- **Compiler**: Must use Clang with C++26 support (enforced by EnsureClang.cmake on macOS)
- **Unit Safety**: Never bypass the unit system - all physical quantities must have units
- **Graphics Optional**: Core physics library has no graphics dependencies; graphics is test-only
