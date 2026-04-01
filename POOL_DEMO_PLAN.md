# Pool Demo Plan

Developer note: this document summarizes engine capabilities discovered and the design for the Pool/Billiards demo.

Engine capabilities (found in repository)
- Rigid body dynamics via `physkit::world` and `particle`/`object` types.
- Collision detection: broad-phase BVH + narrow-phase manifold generation (`detail::broad_phase`, `narrow_phase`).
- Contact resolution: impulse-based constraint solver with contact normal + two tangents, friction, restitution, warm-starting and configurable solver iterations (`impulse::constraint_solver`).
- Primitive meshes: `physkit::mesh::sphere`, `mesh::box`, `mesh::pyramid`; mesh BVH for raycasts and queries.
- Object properties: mass, inertia tensor, restitution, friction, orientation, angular velocity; impulses/forces API available.
- Integrators: `semi_implicit_euler` (implemented), RK4 scaffold (not implemented). Substepping / fixed timestep via `physkit::stepper` used by `graphics_app`.
- Constraints / joints: distance, ball-socket, hinge, slider, weld (available in `constraint.h`).
- Ray/misc queries: BVH-accelerated ray intersection on `mesh` (useful for trajectory previews).
- Graphics & demo framework: `graphics_app` (Magnum-based) with `g_config` JSON loader, camera, instanced rendering, input hooks, and `add_object()` helper.
- Input handling: keyboard and pointer events exposed via `graphics_app` overrides; `keys()` and `mouse_pos()` helpers provided.
- Debugging / overlays: `graphics_app` has a `M_debug` flag and rendering layers; debug drawing of physics internals is supported at the app layer.

Limitations discovered
- No explicit continuous collision detection (CCD) subsystem found — high-speed tunnelling must be handled by tuning time step and solver iterations.
- No explicit sleeping system for bodies (no automatic sleep/wake detected in headers). Objects will be simulated continuously.
- No built-in audio system or particle systems in the repo.
- Integrator: only semi-implicit Euler is implemented; high-precision integration limited by that integrator and solver iterations.

Design decisions for pool demo
- Use `graphics_app` subclass to implement pool controls and camera composition (frees us from windowing/GL setup and config parsing).
- Build the table, rails, pockets and balls using native `physkit::mesh` factories and `object_desc` + `world.create_rigid()` to add bodies.
- Use spheres for balls and boxes for table/rails (simple, performant). Use a planar table with pockets detected by simple distance tests (pocket centers + radius) — avoids adding sensors to the physics engine.
- Implement aiming by mapping horizontal mouse motion to an azimuth angle around the cue ball (simple, robust, and camera-independent). Show a cue stick graphic aligned to the aim vector.
- Charge-and-release shooting: hold left mouse to charge (power bar), on release compute a target speed and apply an impulse to the cue ball using the object's mass times desired delta-v (units handled with mp_units).
- Prevent shooting while any ball exceeds a small velocity threshold (enforces turn stability and showcases solver stability during breaks).
- Pocket detection: when a ball's horizontal distance to a pocket center is less than pocket radius and its vertical position falls below pocket lip threshold, remove it from the world and sink it visually below the table.
- Reset/rerack: store initial dynamic-object transforms at startup and restore on `R` key.

Polish and tuning
- Expose tuning constants at the top of the demo source for: ball radius, mass, friction, restitution, damping factor (local), cue impulse multiplier, pocket radius.
- Camera: use `g_config` defaults with a slighty elevated camera looking toward table; `graphics_app` camera supports nice framing and animation tracks if needed.

Files added
- `tests/pool/pool_demo.cpp` — the demo app and gameplay logic.
- `tests/pool/CMakeLists.txt` — build recipe for demo.
- `tests/configs/pool_demo.json` — default scene config used by the demo (fallback values).
- `POOL_DEMO_CONTROLS.md`, `POOL_DEMO_TUNING.md` — run instructions and tuning notes.

Known workarounds
- Sinking balls: we remove the rigid body from the physics `world` and then keep the graphics instance transformed below the table (the `graphics_app` stops syncing a removed body). This avoids modifying engine internals.

Next steps
- Build and run `tests/pool` demo. Iterate tuning constants and camera framing for best visual presentation.
