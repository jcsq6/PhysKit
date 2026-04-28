#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <coroutine> // IWYU pragma: keep
#endif

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#else
#include <physkit/physkit.h>
#endif

#ifdef PHYSKIT_GRAPHICS_MODULES
import graphics;
#else
#include <graphics/graphics.h>
#endif

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace graphics;

// Inertia Tensor Visualization Test
// Demonstrates correct inertia tensor computation for different mesh shapes
// and their rotational dynamics behavior
class inertia_tensor_app : public graphics_app
{
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    using typed_world = physkit::world<physkit::semi_implicit_euler>;

public:
    explicit inertia_tensor_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .window_size({1280, 720})
                           .title("Inertia Tensor Test - Mesh Dynamics Demo")
                           .cam_pos(fvec3{15, 12, -20} * m)
                           .look_at(fvec3{0, 3, 0} * m)
                           .gravity(vec3{0, -9.81, 0} * m / s / s)
                           .solver_iterations(32)
                           .time_step(1.0 / 120.0 * s)}
    {
        auto &w = dynamic_cast<physkit::world<physkit::semi_implicit_euler> &>(world());

        // Create floor
        auto floor = w.create_rigid(object_desc::stat()
                                        .with_pos({0 * m, -2.5 * m, 0 * m})
                                        .with_mesh(mesh::box(vec3<m>{50 * m, 1 * m, 50 * m})));
        add_object(floor, {0.25f, 0.25f, 0.25f});

        // Test 1: Box with computed inertia tensor
        // Shows stable rotation with anisotropic inertia
        {
            auto box_mesh = mesh::box(vec3<m>{1.0 * m, 1.0 * m, 1.0 * m});
            auto vol = box_mesh->volume();
            auto mass = 2.0 * kg;
            auto density = mass / vol;
            auto inertia = box_mesh->inertia_tensor(density);

            auto box_handle = w.create_rigid(object_desc::dynam()
                                                 .with_pos({-8 * m, 5 * m, 0 * m})
                                                 .with_mass(mass)
                                                 .with_inertia_tensor(inertia)
                                                 .with_ang_vel(vec3{10, 15, 5} * rad / s)
                                                 .with_mesh(box_mesh)
                                                 .with_restitution(0.4)
                                                 .with_friction(0.6));
            add_object(box_handle, {0.2f, 0.6f, 0.9f});
        }

        // Test 2: Sphere with computed inertia tensor
        // Should rotate uniformly due to isotropic inertia
        {
            auto sphere_mesh = mesh::sphere(1.0 * m);
            auto vol = sphere_mesh->volume();
            auto mass = 3.0 * kg;
            auto density = mass / vol;
            auto inertia = sphere_mesh->inertia_tensor(density);

            auto sphere_handle = w.create_rigid(object_desc::dynam()
                                                    .with_pos({-4 * m, 6 * m, 0 * m})
                                                    .with_mass(mass)
                                                    .with_inertia_tensor(inertia)
                                                    .with_ang_vel(vec3{20, 0, 0} * rad / s)
                                                    .with_mesh(sphere_mesh)
                                                    .with_restitution(0.5)
                                                    .with_friction(0.5));
            add_object(sphere_handle, {0.9f, 0.3f, 0.3f});
        }

        // Test 3: Pyramid with computed inertia tensor
        // Demonstrates asymmetric rotational behavior
        {
            auto pyramid_mesh = mesh::pyramid(1.5 * m, 3.0 * m);
            auto vol = pyramid_mesh->volume();
            auto mass = 4.0 * kg;
            auto density = mass / vol;
            auto inertia = pyramid_mesh->inertia_tensor(density);

            auto tilt = quat<one>::from_angle_axis(20.0 * deg, vec3<one>{0, 0, 1});
            auto pyramid_handle = w.create_rigid(object_desc::dynam()
                                                     .with_pos({0 * m, 5 * m, 0 * m})
                                                     .with_mass(mass)
                                                     .with_orientation(tilt)
                                                     .with_inertia_tensor(inertia)
                                                     .with_ang_vel(vec3{0, 25, 10} * rad / s)
                                                     .with_mesh(pyramid_mesh)
                                                     .with_restitution(0.3)
                                                     .with_friction(0.7));
            add_object(pyramid_handle, {0.9f, 0.7f, 0.2f});
        }

        // Test 4: Tall box (anisotropic inertia - different behavior)
        // Shows pronounced wobble due to height/mass distribution
        {
            auto tall_box_mesh = mesh::box(vec3<m>{0.5 * m, 2.5 * m, 0.5 * m});
            auto vol = tall_box_mesh->volume();
            auto mass = 5.0 * kg;
            auto density = mass / vol;
            auto inertia = tall_box_mesh->inertia_tensor(density);

            auto tilt = quat<one>::from_angle_axis(10.0 * deg, vec3<one>{1, 0, 0});
            auto tall_box_handle = w.create_rigid(object_desc::dynam()
                                                      .with_pos({4 * m, 6 * m, 0 * m})
                                                      .with_mass(mass)
                                                      .with_orientation(tilt)
                                                      .with_inertia_tensor(inertia)
                                                      .with_ang_vel(vec3{5, 20, 0} * rad / s)
                                                      .with_mesh(tall_box_mesh)
                                                      .with_restitution(0.35)
                                                      .with_friction(0.65));
            add_object(tall_box_handle, {0.3f, 0.85f, 0.4f});
        }

        // Test 5: Flat box (disc-like inertia properties)
        // Demonstrates gyroscopic stabilization
        {
            auto flat_box_mesh = mesh::box(vec3<m>{2.0 * m, 0.3 * m, 2.0 * m});
            auto vol = flat_box_mesh->volume();
            auto mass = 3.0 * kg;
            auto density = mass / vol;
            auto inertia = flat_box_mesh->inertia_tensor(density);

            auto flat_box_handle = w.create_rigid(object_desc::dynam()
                                                      .with_pos({8 * m, 5 * m, 0 * m})
                                                      .with_mass(mass)
                                                      .with_inertia_tensor(inertia)
                                                      .with_ang_vel(vec3{0, 35, 0} * rad / s)
                                                      .with_mesh(flat_box_mesh)
                                                      .with_restitution(0.4)
                                                      .with_friction(0.5));
            add_object(flat_box_handle, {0.7f, 0.3f, 0.8f});
        }

        // Test 6: Small sphere for comparison
        {
            auto small_sphere_mesh = mesh::sphere(0.5 * m);
            auto vol = small_sphere_mesh->volume();
            auto mass = 1.0 * kg;
            auto density = mass / vol;
            auto inertia = small_sphere_mesh->inertia_tensor(density);

            auto small_sphere_handle = w.create_rigid(object_desc::dynam()
                                                          .with_pos({-12 * m, 4 * m, 0 * m})
                                                          .with_mass(mass)
                                                          .with_inertia_tensor(inertia)
                                                          .with_ang_vel(vec3{15, 15, 15} * rad / s)
                                                          .with_mesh(small_sphere_mesh)
                                                          .with_restitution(0.6)
                                                          .with_friction(0.4));
            add_object(small_sphere_handle, {0.4f, 0.8f, 0.9f});
        }
    }

    void update(quantity<si::second> dt) override
    {
        // Optional: Add per-frame logic here if needed
        // For this test, we let the physics engine demonstrate inertia behavior
    }
};

MAGNUM_APPLICATION_MAIN(inertia_tensor_app)
