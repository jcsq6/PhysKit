# Orchestrating Simulations with Coroutines

PhysKit provides a powerful coroutine-based task system (`physkit::task<T>`) to orchestrate complex simulation events. If you've ever tried to script a sequence of events in a physics engine (e.g., "spawn an object, wait 3 seconds, make it explode, wait for debris to hit the ground"), you know it usually requires building complex state machines or using difficult-to-track callback chains.

Coroutines in PhysKit allow you to write asynchronous simulation logic linearly, exactly as you would write synchronous, blocking code. Behind the scenes, the simulation pauses the coroutine, runs the physics world, and seamlessly resumes your coroutine when its requested condition is met.

## The Problem: State Machines

Without coroutines, scripting a temporal sequence of events requires manually maintaining state across frames. Imagine we want to throw a grenade and have it explode after 2 seconds:

### Manually Managing State (Without Coroutines)

```cpp
class MySimulation {
    bool grenade_thrown = false;
    double time_since_thrown = 0.0;
    object_handle grenade_handle;

    void update(double dt) {
        if (!grenade_thrown) {
            grenade_handle = world.create_rigid(...);
            grenade_thrown = true;
            time_since_thrown = 0.0;
        } else if (grenade_handle != object_handle::null) {
            time_since_thrown += dt;
            if (time_since_thrown >= 2.0) {
                // Explode grenade
                explode(grenade_handle);
                world.destroy_rigid(grenade_handle);
                grenade_handle = object_handle::null;
            }
        }
    }
};
```

This gets messy very quickly. Adding a second grenade, or waiting for a collision instead of a timer, drastically increases the complexity of our `update` function and litters our class with tracking variables (`grenade_thrown`, `time_since_thrown`, etc.).

## The Solution: C++ Coroutines

Using PhysKit's `task<>`, we can write the exact same logic sequentially using `co_await`. The engine's task handler will automatically suspend our function and resume it later.

### With Coroutines

```cpp
task<vec3<si::metre>> throw_grenade(app* self, vec3<si::metre> pos, vec3<si::metre> vel, quantity<si::second> fuse_time) {
    // All co_await operations return a std::expected<T, physkit::error>.
    // Use * or .value() to extract the result when success is guaranteed.
    
    // 1. Create the grenade
    auto gh = *co_await create_rigid(object_desc::dynam().with_pos(pos).with_lin_vel(vel)...);
    
    // 2. Wait for the fuse time. The underlying physics simulation
    // will continue stepping normally while this function waits!
    co_await wait_for(fuse_time);
    
    // 3. Explode
    co_await get_rigid(gh); // Will abort if grenade was already destroyed early
    auto& w = co_await get_world();
    auto grenade_pos = w.get_rigid(gh)->pos();
    co_await destroy_rigid(gh);
    spawn_explosion_debris(self, grenade_pos);
    co_return grenade_pos;
}
```

Notice how we have **zero member variables** tracking state or timers. The function reads exactly like what we want to happen.

## Key Awaitables

PhysKit provides several `co_await`-able operations out of the box in `<physkit/core/awaitables.h>`:

### Temporal
- `co_await wait_for(duration)`: Suspend until a certain amount of simulation time has passed.
- `co_await wait_until_time(time)`: Suspend until the simulation reaches a specific absolute time.
- `co_await next_frame()`: Suspend the coroutine until the start of the next logical frame.
- `co_await next_physics_tick()`: Suspend until the next physics step completes.
- `co_await wait_until(condition)`: Suspend the coroutine until a custom predicate returns `true` (evaluated every frame).

### Physics & Events
- `co_await wait_for_collision({handle, [with]})`: Suspends until the specified object collides with something (or an optional specific `with` object). Returns a structure containing the `other` object's handle and a `contact_manifold`.
- `co_await wait_for_separation({handle, [with]})`: Suspends until an existing collision ends. Returns the handle of the object it separated from.
- `co_await wait_until_destroyed({handle})`: Suspends until the specified object is destroyed.
- `co_await wait_for_event(setup_fn, destroy_fn)`: Primitive to build custom event suspensions by injecting arbitrary setup and teardown lambdas.

### Commands
- `co_await create_rigid(desc)`: Creates a new rigid body in the simulation world.
- `co_await destroy_rigid(handle)`: Destroys a given rigid body.
- `co_await get_rigid(handle)`: Safely tests if a rigid body exists, failing if the handle is stale.
- `co_await get_world()`: Retrieves a reference to the `world` (does not return `std::expected`).
- `co_await raycast(ray, max_dist)`: Performs a raycast and yields a generator of hit results. 
- `co_await add_constraint(desc)`: Adds a constraint to the world.
- `co_await remove_constraint(handle)`: Removes a constraint.
- `co_await add_task(my_task())`: Starts a child coroutine task. Use `<policy::no_wait>` to fire-and-forget without waiting for initialization.
- `co_await cancel_task(task_id)`: Cancels a running task.

### Orchestration
- `co_await wait_for_all(awaitable1, awaitable2, ...)`: Suspend until *all* the provided awaitables complete. Returns a `std::tuple` of the results.
- `co_await wait_for_any(awaitable1, awaitable2, ...)`: Suspend until *any* of the provided awaitables complete. Returns a `std::variant` of the results.

## Error Handling (`std::expected`)

Because physical events can fail (e.g. an object might be destroyed while waiting for it to collide, or a handle becomes stale), *every* wait-style awaitable returns a `std::expected<T, physkit::error>`. 

```cpp
auto result = co_await get_rigid(gh);
if (!result) {
    // Handle error (e.g. result.error() == physkit::error::stale_handle)
    co_return;
}
auto& w = co_await get_world();
auto* obj = w.get_rigid(gh);
```

## Advanced Composition

Because tasks can return values and be `co_await`ed by other tasks, they become highly composable building blocks. Here is an excerpt adapted from the `missile_demo.cpp` test that builds up a cascading attack using sub-tasks:

```cpp
task<vec3<si::metre>> launch_guided(vec3<si::metre> origin, vec3<si::metre> target) {
    // Spawn missile box
    auto mh = *co_await create_rigid(object_desc::dynam().with_pos(origin));

    // Delegate the actual per-frame steering logic to another sub-task
    // which eventually returns the impact position.
    co_return *co_await guide_to_target(mh, target);
}

task<> main_scene() {
    using namespace mp_units::si::unit_symbols;

    // Wait for the grenade to blow up on the ground and get its actual detonation position
    auto detonation_pos = *co_await throw_grenade(self, origin, vel, 2.0 * s);

    // Launch a guided missile at the exact point where the grenade detonated
    auto impact_pos = *co_await launch_guided(missile_silo_pos, detonation_pos);

    // After the missile hits that spot, trigger secondary explosions
    spawn_debris(self, impact_pos, ...);
}
```

In the example above, `throw_grenade` and `launch_guided` are standalone, reusable action bundles. The "Director" routine (`main_scene`) weaves them together to form complex choreography effortlessly.
