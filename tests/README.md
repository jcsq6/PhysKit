# PhysKit Graphics & Demo Framework

This directory contains a self-contained graphics and configuration framework for building PhysKit demos and visual tests. It is built on top of [Magnum](https://magnum.graphics/) (OpenGL, GLFW) and provides camera animation, JSON+CLI configuration, instanced rendering, and a ready-made application base class.

**Headers:** [`tests/include/`](include/)

| Header | Purpose |
|--------|---------|
| [`graphics.h`](include/graphics.h) | Application base class, scene objects, config loader, mesh helpers |
| [`camera.h`](include/camera.h) | FPS-style camera, keyframe animation tracks |
| [`convert.h`](include/convert.h) | Conversion utilities between PhysKit and Magnum types |
| [`test.h`](include/test.h) | Lightweight test runner, assertion macros, approximate comparisons |

---

## Table of Contents

- [Quick Start](#quick-start)
- [Configuration (`g_config`)](#configuration-g_config)
  - [Builder API](#builder-api)
  - [JSON Configuration File](#json-configuration-file)
  - [CLI Arguments](#cli-arguments)
  - [Priority Order](#priority-order)
  - [Defaults](#defaults)
- [Application Base Class (`graphics_app`)](#application-base-class-graphics_app)
  - [Subclassing](#subclassing)
  - [Adding Objects](#adding-objects)
  - [Input Handling](#input-handling)
  - [Frame Timing](#frame-timing)
- [Camera (`camera`)](#camera)
  - [Construction](#construction)
  - [Movement & Rotation](#movement--rotation)
  - [Camera Animation Tracks](#camera-animation-tracks)
- [Keyframes (`kf`)](#keyframes-kf)
  - [Factory Methods](#factory-methods)
  - [Builder Methods](#builder-methods)
- [Camera Track (`camera_track`)](#camera-track-camera_track)
  - [Interpolation](#interpolation)
  - [Extrapolation](#extrapolation)
  - [Freelook Mode](#freelook-mode)
  - [Manual Sampling](#manual-sampling)
- [Scene Objects](#scene-objects)
- [Mesh Helpers (`mesh_objs`)](#mesh-helpers-mesh_objs)
- [Type Conversions (`convert.h`)](#type-conversions-converth)
- [Testing Framework](#testing-framework)
  - [Writing Tests](#writing-tests)
  - [Assertion Macros](#assertion-macros)
  - [Approximate Comparison](#approximate-comparison)
  - [CMake Integration](#cmake-integration)
  - [Graphical Tests (`--testing` mode)](#graphical-tests---testing-mode)

---

## Quick Start

The minimal demo is a subclass of `graphics_app` with an `update()` override:

```cpp
#include <graphics.h>
#include <mp-units/systems/si/unit_symbols.h>

using namespace graphics;
using namespace physkit;
using namespace si::unit_symbols;

class app : public graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics_app{g_config{arguments}.title_or("My Demo")}
    {
        // Set up objects, camera, etc.
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        // Called every frame before drawing.
    }
};

MAGNUM_APPLICATION_MAIN(app)
```

Run with:
```sh
./my_demo                              # all defaults
./my_demo -c scene.json                # load from JSON
./my_demo --fov 60 --cam-pos 0,10,-20  # CLI overrides
```

---

## Configuration (`g_config`)

`g_config` is a unified configuration object that merges three sources: a **builder API** (programmatic defaults), a **JSON config file**, and **CLI arguments**.

### Builder API

All setters use a builder pattern and return `*this` for chaining. Every property has two variants:

| Method | Behavior |
|--------|----------|
| `.fov(val)` | Always sets the value (unconditional) |
| `.fov_or(val)` | Sets the value only if not already set (fallback) |

The `_or` variants are the key mechanism for layering: a JSON file sets values first, then `_or` in code provides fallback defaults without overwriting what the file already specified.

```cpp
g_config{arguments}
    .cam_pos_or(vec3{0, 0, -25} * m)   // fallback if not set by JSON/CLI
    .cam_dir_or(vec3{0, 0, 1} * one)
    .title_or("My Demo")
    .window_size_or({1280, 720})
```

**Available properties:**

| Property | Type | Setter / Setter-or | Description |
|----------|------|-------------------|-------------|
| `fov` | `quantity<degree>` | `.fov()` / `.fov_or()` | Camera field of view |
| `cam_pos` | `fvec3<metre>` | `.cam_pos()` / `.cam_pos_or()` | Camera position |
| `cam_dir` | `fvec3<one>` | `.cam_dir()` / `.cam_dir_or()` | Camera direction vector |
| `look_at` | `fvec3<metre>` | `.look_at()` / `.look_at_or()` | Point camera toward target (overrides `cam_dir`) |
| `title` | `string_view` | `.title()` / `.title_or()` | Window title |
| `window_size` | `Vector2i` | `.window_size()` / `.window_size_or()` | Window dimensions |
| `drag` | `bool` | `.drag()` / `.drag_or()` | Mouse drag mode (see [Input Handling](#input-handling)) |
| `vsync` | `bool` | `.vsync()` / `.vsync_or()` | Vertical sync |
| `gravity` | `vec3<m/s²>` | `.gravity()` / `.gravity_or()` | World gravity vector |
| `integrator` | `world_desc::integ_t` | `.integrator()` / `.integrator_or()` | Physics integrator type |
| `solver_iterations` | `size_t` | `.solver_iterations()` / `.solver_iterations_or()` | Constraint solver iterations |
| `time_step` | `quantity<second>` | `.time_step()` / `.time_step_or()` | Fixed physics time step |

The constructor signature is:
```cpp
g_config(Magnum::Platform::Application::Arguments args, bool read_config = true);
```

Pass `read_config = false` to disable JSON/CLI config file parsing (the `--config` argument will not be registered). All other CLI arguments are still parsed.

### JSON Configuration File

Pass a JSON file with `--config` / `-c`. The file is deserialized with [Glaze](https://github.com/stephenberry/glaze).

**Full schema:**

```jsonc
{
    // World physics
    "gravity": [0.0, -9.81, 0.0],         // m/s², default [0, -9.81, 0]
    "integrator": "semi_implicit_euler",   // "forward_euler" | "semi_implicit_euler" | "rk4"
    "solver_iterations": 10,               // unsigned integer

    // Camera & window (all optional)
    "fov": 45.0,                           // degrees
    "cam_pos": [0.0, 5.0, -10.0],         // metres
    "cam_dir": [0.0, 0.0, 1.0],           // unit direction
    "look_at": [0.0, 0.0, 0.0],           // metres (overrides cam_dir)
    "title": "My Scene",
    "window_size": [1280, 720],
    "drag": false,
    "vsync": true,

    // Objects array
    "objects": [
        {
            "pos": [0.0, 5.0, 0.0],                    // metres (required)
            "vel": [0.0, 0.0, 0.0],                    // m/s, default [0,0,0]
            "orientation": [1.0, 0.0, 0.0, 0.0],       // quaternion [w,x,y,z], default identity
            "angular_velocity": [0.0, 0.0, 1.0],       // rad/s, default [0,0,0]
            "inertia_tensor": [1.0, 1.0, 1.0],         // diagonal [Ixx,Iyy,Izz] kg·m², default [1,1,1]
            "mass": 1.0,                                // kg (required)
            "type": "dynam",                            // "dynam" | "stat"
            "color": [1.0, 0.0, 0.0, 1.0],             // RGBA floats 0-1, default white
            "mesh": {                                   // mesh descriptor (see below)
                "name": "sphere",
                "radius": 0.5
            }
        }
    ]
}
```

**Mesh types:**

| Type | Fields | Example |
|------|--------|---------|
| `box` | `half_extents: [x, y, z]` | `{"name": "box", "half_extents": [0.5, 0.5, 0.5]}` |
| `sphere` | `radius`, optional `stacks` (16), `sectors` (32) | `{"name": "sphere", "radius": 1.0}` |
| `pyramid` | `height`, `base_size` | `{"name": "pyramid", "height": 1.0, "base_size": 1.0}` |

**Example JSON file** (see [`tests/configs/ball_box_pyr.json`](configs/ball_box_pyr.json)):

```json
{
    "cam_pos": [-25.0, -10.0, -25.0],
    "look_at": [0.0, 10.0, 0.0],
    "drag": true,
    "objects": [
        {
            "mesh": {"name": "sphere", "radius": 0.5},
            "pos": [0.0, 0.0, 0.0],
            "vel": [1.0, 20.0, 0.0],
            "mass": 1.0,
            "color": [1.0, 0.0, 0.0]
        },
        {
            "mesh": {"name": "box", "half_extents": [0.5, 0.5, 0.5]},
            "pos": [5.0, 0.0, 0.0],
            "vel": [-1.0, 20.0, 0.0],
            "mass": 2.0,
            "color": [0.0, 1.0, 0.0]
        }
    ]
}
```

### CLI Arguments

All CLI arguments are parsed with [argparse](https://github.com/p-ranav/argparse). When `read_config = true`, the `--config` argument is also available.

| Flag | Short | Type | Default | Description |
|------|-------|------|---------|-------------|
| `--fov` | `-f` | `double` | `45` | FOV in degrees |
| `--cam-pos` | `-p` | `x,y,z` | `0,0,0` | Camera position (metres) |
| `--cam-dir` | `-d` | `x,y,z` | `0,0,1` | Camera direction |
| `--look-at` | `-l` | `x,y,z` | *(none)* | Look-at target (overrides `--cam-dir`) |
| `--title` | `-t` | `string` | `PhysKit Graphics Demo` | Window title |
| `--window-size` | `-w` | `w,h` | `1280,720` | Window size |
| `--drag` | | `bool` | `false` | Enable drag-to-rotate mode |
| `--vsync` | `-v` | `bool` | `true` | Enable VSync |
| `--time-step` | `-s` | `double` | `0.01667` | Physics time step in seconds |
| `--config` | `-c` | `path` | *(none)* | JSON config file path |

### Priority Order

Values are resolved with a clear precedence:

1. **CLI arguments** (highest priority — always override everything)
2. **JSON config file** (loaded via `--config`)
3. **Programmatic `_or` fallbacks** (`cam_pos_or(...)`, etc.)
4. **Built-in defaults** (lowest priority)

The unconditional setters (`.fov()`, `.cam_pos()`, etc.) bypass this layering and always overwrite.

### Defaults

| Property | Default Value |
|----------|---------------|
| FOV | 45° |
| Camera position | (0, 0, 0) m |
| Camera direction | (0, 0, 1) |
| Window title | `"PhysKit Graphics Demo"` |
| Window size | 1280×720 |
| Drag mode | `false` (FPS-style mouse look) |
| VSync | `true` |
| Time step | 1/60 s |
| Gravity | (0, −9.81, 0) m/s² |
| Integrator | `semi_implicit_euler` |
| Solver iterations | 10 |

---

## Application Base Class (`graphics_app`)

`graphics_app` extends `Magnum::Platform::Application` and manages the render loop, physics stepping, camera controls, and input dispatch.

### Subclassing

Override these `protected virtual` methods:

| Method | Called | Required |
|--------|--------|----------|
| `update(dt)` | Every frame, before physics step and draw | **Yes** |
| `key_press(event, pressed)` | On key press/release | No |
| `pointer_move(event)` | On mouse movement | No |
| `pointer_press(event, pressed)` | On mouse button press/release | No |

```cpp
class my_app : public graphics_app
{
public:
    explicit my_app(const Arguments &args)
        : graphics_app{g_config{args}.title("My App")}
    {
        // Initialization: add objects, configure camera, etc.
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        // Per-frame logic. Physics step happens automatically after this.
    }

    void key_press(KeyEvent &event, bool pressed) override
    {
        if (pressed && event.key() == Key::R) { /* reset */ }
    }
};
```

### Adding Objects

Three `add_object` overloads are available:

```cpp
// 1. Physics object (auto-creates mesh from physkit::object's mesh)
physics_obj *obj = add_object(world_handle, Color3{1, 0, 0});

// 2. Physics object with explicit shared mesh
auto mesh = get_mesh(phys_mesh);
physics_obj *obj = add_object(world_handle, mesh, Color3{0, 1, 0});

// 3. Pure graphics object (no physics)
auto mesh = mesh_objs::sphere();
gfx_obj *obj = add_object(mesh, Color3{0, 0, 1});
obj->scale({2.0f, 2.0f, 2.0f});
obj->translate({5.0f, 0.0f, 0.0f});
```

Objects sharing the same `GL::Mesh` are automatically instanced for efficient rendering.

Physics objects (`physics_obj`) are automatically synced with their rigid body's position and orientation each frame. For graphics-only objects, you manipulate their transforms directly using Magnum's scene graph API (`translate`, `rotate`, `scale`, `resetTransformation`).

### Input Handling

**Mouse modes:**

| Mode | `drag = false` (default) | `drag = true` |
|------|--------------------------|---------------|
| Camera rotation | Moving the mouse rotates the camera (FPS-style, cursor hidden) | Click-and-drag to rotate |
| Toggle | Press `Esc` to switch modes | Press `Esc` to switch modes |

**Keyboard state** is available via `keys()`:

```cpp
if (keys()[Key::Space].is_pressed())      { /* held down */ }
if (keys()[Key::Space].is_initial_press()) { /* first frame only */ }
```

**Built-in movement** (always active):

| Key | Action |
|-----|--------|
| W / S | Move forward / backward |
| A / D | Strafe left / right |
| Space / Shift | Move up / down |
| Esc | Toggle drag mode |

### Frame Timing

```cpp
dt()            // Time since last frame (quantity<second>)
current_time()  // Time since application start (quantity<second>)
set_vsync(bool) // Enable/disable vsync at runtime
frame_limit(hz) // Set a frame rate cap (0 Hz = unlimited)
```

---

## Camera (`camera`)

The `camera` class provides a first-person-style camera with perspective projection, movement, mouse look, and animation track support.

### Construction

`graphics_app` constructs the camera automatically from `g_config`. Access it with `cam()`.

**Key parameters:**

| Parameter | Type | Default |
|-----------|------|---------|
| FOV | `quantity<degree, float>` | From config |
| Near plane | `0.01f` | Fixed |
| Far plane | `1000.0f` | Fixed |
| Speed | `quantity<m/s, float>` | `4.0 m/s` |

### Movement & Rotation

```cpp
cam().move(forward, right, up, dt);          // Relative movement along camera axes
cam().move(vec3{1, 0, 0} * m);              // Absolute position delta
cam().rotate(yaw_rad, pitch_rad);            // Rotate by yaw/pitch
cam().look_at(vec3{0, 0, 0} * m);           // Point at a world position
cam().set_view(vec3{0, 0, 1} * one);        // Set facing direction
cam().speed(10 * m / s);                     // Change movement speed
```

---

## Keyframes (`kf`)

A `kf` represents a single point in a camera animation. Keyframes specify a position and/or orientation, plus a transition duration to the next keyframe.

### Factory Methods

Each factory creates a `kf` with one primary property set:

```cpp
kf::make_pos(vec3{0, 5, -10} * m)           // Position keyframe
kf::make_look_at(vec3{0, 0, 0} * m)         // Orientation: look at target
kf::make_dir(vec3{0, 0, 1} * one)           // Orientation: face direction
kf::make_orient(quaternion)                  // Orientation: explicit quaternion
```

### Builder Methods

Chain additional properties onto any keyframe:

```cpp
kf::make_pos(vec3{5, 2, 0} * m)
    .look_at(vec3{0, 0, 0} * m)    // Orient toward origin
    .transition(2.0f * s)           // Take 2s to reach the next keyframe
```

| Method | Description |
|--------|-------------|
| `.pos(vec3)` | Set/override position |
| `.look_at(vec3)` | Look at target (requires position for direction calculation) |
| `.dir(vec3)` | Face a direction |
| `.orient(quat)` | Explicit quaternion orientation |
| `.transition(duration)` | Time to interpolate *to the next* keyframe |

A keyframe can have **at most one** orientation type (`.look_at`, `.dir`, or `.orient`). Setting a new one replaces the previous.

---

## Camera Track (`camera_track`)

`camera_track` takes a sequence of keyframes and produces smooth interpolated camera poses over time.

```cpp
cam().set_move_track(
    camera_track({
        kf::make_pos(vec3{0, 1, -5} * m)
            .look_at(vec3{0, 0, 0} * m)
            .transition(2.0f * s),
        kf::make_pos(vec3{5, 2, -3} * m)
            .transition(1.5f * s),
        kf::make_pos(vec3{0, 1, 5} * m)
            .dir(vec3{0, 0, 1} * one)
            .transition(2.0f * s),
    })
        .with_interp(camera_track::spline)
        .with_extrap(camera_track::release));
```

**Requirements:**
- At least **two keyframes** must have position data.
- The track always starts at t = 0.
- Transition times are cumulative (each keyframe's `transition()` is the duration from **it** to the next).

### Interpolation

Set with `.with_interp(mode)`. Default is `spline`.

| Mode | Enum | Description |
|------|------|-------------|
| Constant | `camera_track::constant` | Step function, no interpolation |
| Linear | `camera_track::linear` | Linear interpolation between keyframes |
| Spline | `camera_track::spline` | Cubic Hermite spline (smooth, recommended) |

### Extrapolation

Set with `.with_extrap(mode)`. Controls what happens when the track's duration is exceeded. Default is `release`.

| Mode | Enum | Description |
|------|------|-------------|
| Release | `camera_track::release` | Camera reverts to manual control |
| Loop | `camera_track::loop` | Track loops back to the start seamlessly |
| Reverse | `camera_track::reverse` | Track ping-pongs back and forth |

### Freelook Mode

If **no keyframes** specify an orientation (no `.look_at()`, `.dir()`, or `.orient()`), the track enters **freelook mode**: position is animated, but the user retains mouse control over rotation.

```cpp
// Freelook: position follows the track, user rotates freely
cam().set_move_track(camera_track({
    kf::make_pos(vec3{0, 8, -18} * m).transition(2.0f * s),
    kf::make_pos(vec3{0, 20, 0} * m).transition(4.0f * s),
}));
```

### Manual Sampling

To sample a track without assigning it to a camera:

```cpp
camera_track track({
    kf::make_pos(vec3{0, 0, 0} * m).transition(1.0f * s),
    kf::make_pos(vec3{5, 0, 0} * m)
});

auto [pos, rot] = track.at(0.5f * s);
// pos: physkit::vec3<metre, float>
// rot: physkit::quat<one, float>
```

---

## Scene Objects

| Class | Inherits | Description |
|-------|----------|-------------|
| `gfx_obj` | Magnum `Object<MatrixTransformation3D>` | Base graphics object with standard transform methods |
| `physics_obj` | `gfx_obj` | Linked to a `physkit::world::handle`; auto-syncs position/orientation each frame |
| `instanced_drawable` | `gfx_obj` + `Drawable3D` | Internal: objects sharing the same mesh are batched for instanced draw |
| `colored_drawable` | `instanced_drawable` | Adds per-instance color; created automatically by `add_object()` |

`gfx_obj` exposes all of Magnum's scene graph transforms:

```cpp
obj->resetTransformation()
    .scale({2.0f, 1.0f, 2.0f})
    .rotateY(Deg(45.0f))
    .translate({10.0f, 0.0f, 0.0f});
```

`physics_obj` gives access to the underlying physics object:

```cpp
physics_obj *p = add_object(handle, color);
p->obj().pos();     // physkit position
p->obj().vel();     // physkit velocity
p->handle();        // world handle
```

---

## Mesh Helpers (`mesh_objs`)

The `graphics::mesh_objs` namespace provides factory functions for common GPU meshes (returned as `std::shared_ptr<GL::Mesh>`). These are **graphics-only** primitives for decoration; physics collision meshes come from `physkit::mesh`.

```cpp
auto sphere = mesh_objs::sphere(3);                // icosphere, 3 subdivisions (default)
auto box    = mesh_objs::cube();                    // unit cube
auto cyl    = mesh_objs::cylinder(1, 48, 0.5f);    // rings, segments, half_length
auto cn     = mesh_objs::cone(4, 32, 0.5f);        // rings, segments, half_length
auto pl     = mesh_objs::plane();                   // unit plane
```

Use `get_mesh(physkit_mesh)` to create a GPU mesh from a physics mesh:

```cpp
auto phys_mesh = physkit::mesh::box(vec3{1, 1, 1} * m);
auto gl_mesh = get_mesh(*phys_mesh);
```

---

## Type Conversions (`convert.h`)

Conversion utilities between PhysKit (`physkit::vec`, `physkit::quat`) and Magnum (`Math::Vector`, `Math::Quaternion`) types. These are used internally but are also available for user code.

```cpp
using namespace graphics;

// PhysKit → Magnum
Magnum::Vector3 v = to_magnum_vector<physkit::si::metre, float>(physkit_vec);
Magnum::Quaternion q = to_magnum_quaternion<physkit::one, float>(physkit_quat);

// Magnum → PhysKit
auto pv = to_physkit_vector<physkit::si::metre, float>(magnum_vec);
auto pq = to_physkit_quaternion<physkit::one, float>(magnum_quat);

// PhysKit mesh → Magnum GL::Mesh
Magnum::GL::Mesh m = to_magnum_mesh(phys_mesh);
```

---

## Testing Framework

PhysKit includes a lightweight, header-only testing framework in [`test.h`](include/test.h). It provides a simple test runner with grouped test suites, approximate comparisons for quantities and geometric types, and clear PASS/FAIL output.

### Headers

| Header | Purpose |
|--------|---------|
| [`test.h`](include/test.h) | Test runner, assertion macros, approximate comparison utilities |

### Writing Tests

Tests are plain `void` functions registered on a `testing::suite`. Group related tests under named sections:

```cpp
#include "test.h"

using namespace testing;

void test_something()
{
    CHECK(1 + 1 == 2);
}

void test_approx_value()
{
    auto v = vec3{1.0, 2.0, 3.0} * m;
    CHECK_APPROX(v.x(), 1.0 * m);
}

int main()
{
    suite tests;

    tests.group("My Group")
        .test("something", test_something)
        .test("approx value", test_approx_value);

    return tests.run();
}
```

If no `.group()` is called before `.test()`, tests are placed in a default `"Tests"` group.

### Assertion Macros

| Macro | Description |
|-------|-------------|
| `CHECK(expr)` | Asserts that `expr` is truthy. Throws `check_failure` on failure with file/line info. |
| `CHECK_APPROX(a, b)` | Asserts that `a` and `b` are approximately equal (within `eps = 1e-9`). |
| `FAIL(msg)` | Unconditionally fails with the given message. |

All macros capture the source location and report it on failure.

### Approximate Comparison

`CHECK_APPROX` dispatches to overloaded `approx()` functions that support:

| Type | Comparison |
|------|------------|
| `float` / `double` | `std::abs(a - b) < eps` |
| `mp_units::Quantity` | Compares numerical values in the quantity's reference unit |
| `unit_mat<Q, R, C>` | Norm of the difference matrix < `eps` |
| `unit_quat<Q>` | Angular distance < `eps` |
| `aabb` | Component-wise approximate comparison of `min` and `max` |
| `bounding_sphere` | Approximate comparison of `center` and `radius` |
| `mesh::ray_hit` | Approximate comparison of `pos`, `normal`, and `distance` |

### Test Output

The runner prints grouped results to stdout/stderr:

```
=== AABB Tests ===
  PASS: from_points
  PASS: size/center/extent
  FAIL: volume - approx(box.volume(), 25.0 * m * m * m) (main.cpp:85)

=== Mesh Tests ===
  PASS: make
  PASS: bounds
  ...

5 out of 6 tests passed!
```

The process exits with code `0` if all tests pass, `1` otherwise. This makes it compatible with CTest and CI pipelines.

### CMake Integration

Test executables link against `phys_testing` (for non-graphical tests) or `phys_graphics` (for graphical tests) and are registered with `add_test()`:

```cmake
# Non-graphical tests (test.h only)
add_executable(mesh_tests main.cpp)
target_link_libraries(mesh_tests PRIVATE phys_testing)
add_test(NAME mesh_tests COMMAND mesh_tests)
```

Run all tests with:

```sh
cd build/Debug && ctest
```

### Graphical Tests (`--testing` mode)

Graphical demos built with `graphics_app` can also be registered as CTest tests by passing the `--testing` flag. When `--testing` is set:

1. The demo launches and renders normally, allowing visual inspection.
2. When the application window is closed, a **platform-native dialog** pops up asking "Success?" with Yes/No buttons.
3. The process exits with code `0` (Yes) or `1` (No), which CTest interprets as pass/fail.

This enables manual visual verification of rendering correctness within an automated test harness.

```cmake
# Graphical test with --testing flag
add_executable(bounce_demo bounce_demo.cpp)
target_link_libraries(bounce_demo PRIVATE phys_graphics)
add_test(NAME bounce_demo COMMAND bounce_demo --testing)
```

The dialog is implemented per-platform: `osascript` on macOS, `MessageBox` on Windows, and `zenity` on Linux.

| Flag | Purpose |
|------|---------|
| `--testing` | Enables the exit-dialog prompt after the window closes |

---

## Demo Programs

| Demo | Source | Description |
|------|--------|-------------|
| Config demo | [`demo/config_demo.cpp`](demo/config_demo.cpp) | Minimal app; loads scene entirely from JSON + CLI |
| Bounce demo | [`demo/bounce_demo.cpp`](demo/bounce_demo.cpp) | Falling cube with manual collision response |
| Solar system | [`graphics_demo/solar_system.cpp`](graphics_demo/solar_system.cpp) | Animated solar system with camera track tour |
| Track test | [`graphics_demo/track_demo.cpp`](graphics_demo/track_demo.cpp) | Camera track in freelook mode, visiting waypoints |
