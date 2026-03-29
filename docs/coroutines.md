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
task<> throw_grenade(app* self) {
    using namespace mp_units::si::unit_symbols;
    
    // 1. Create the grenade
    auto gh = co_await create_rigid(object_desc::dynam().with_mass(0.5 * kg)...);
    
    // 2. Wait exactly 2 seconds. The underlying physics simulation
    // will continue stepping normally while this function waits!
    co_await wait_for(2.0 * s);
    
    // 3. Explode
    auto grenade_pos = (*co_await get_rigid(gh))->pos();
    co_await destroy_rigid(gh);
    spawn_explosion_debris(self, grenade_pos);
}
```

Notice how we have **zero member variables** tracking state or timers. The function reads exactly like what we want to happen.

## Key Awaitables

PhysKit provides several `co_await`-able operations out of the box in `<physkit/core/awaitables.h>`:

### Temporal
- `co_await wait_for(time)`: Suspend until a certain amount of simulation time has passed.
- `co_await wait_until(time)`: Suspend until the simulation reaches a specific absolute time.
- `co_await next_frame()`: Suspend the coroutine until the start of the next logical frame.
- `co_await next_physics_tick()`: Suspend until the next physics step.

### Physics & Events
- `co_await wait_for_collision(handle)`: Suspends until the specified object collides with something. Returns a structure containing the other object's handle and the contact manifold!
- `co_await wait_for_separation(handle)`: Suspends until an existing collision ends.

### Orchestration
- `co_await wait_for_all(task1, task2, ...)`: Suspend until *all* the provided tasks complete.
- `co_await wait_for_any(task1, task2, ...)`: Suspend until *any* of the provided tasks complete.
- `co_await add_task(my_task())`: Spin up a child coroutine as an independent background task (fire-and-forget).

## Advanced Composition

Because tasks can return values and be `co_await`ed by other tasks, they become highly composable building blocks. Here is an excerpt adapted from the `missile_demo.cpp` test that builds up a cascading attack using sub-tasks:

```cpp
task<vec3<si::metre>> launch_guided(vec3<si::metre> origin, vec3<si::metre> target) {
    // Spawn missile box
    auto mh = co_await create_rigid(object_desc::dynam().with_pos(origin));

    // Delegate the actual per-frame steering logic to another sub-task
    // which eventually returns the impact position.
    co_return co_await guide_to_target(mh, target);
}

task<> main_scene() {
    using namespace mp_units::si::unit_symbols;

    // Wait for the grenade to blow up on the ground and get its actual detonation position
    auto detonation_pos = co_await throw_grenade(origin, vel, 2.0 * s);

    // Launch a guided missile at the exact point where the grenade detonated
    auto impact_pos = co_await launch_guided(missile_silo_pos, detonation_pos);

    // After the missile hits that spot, trigger secondary explosions
    co_await spawn_debris(impact_pos, ...);
}
```

In the example above, `throw_grenade` and `launch_guided` are standalone, reusable action bundles. The "Director" routine (`main_scene`) weaves them together to form complex choreography effortlessly.
