#pragma once
#include "detail/types.h"
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{
class particle
{
public:
    particle(const vec3<si::metre> &pos, const vec3<si::metre / si::second> &vel,
             quantity<si::kilogram> mass, const quat<one> &orientation = quat<one>::identity(),
             const vec3<si::radian / si::second> &ang_vel = vec3<si::radian / si::second>::zero(),
             const mat3<si::kilogram * si::metre * si::metre> &inertia =
                 mat3<si::kilogram * si::metre * si::metre>::identity())
        : M_pos(pos), M_vel(vel), M_mass(mass), M_inv_mass(1.0 / mass), M_orientation(orientation),
          M_ang_vel(ang_vel), M_inertia_tensor_local(inertia),
          M_inv_inertia_tensor_local(inertia.inverse())
    {
        update_derived_state();
    }

    [[nodiscard]] const auto &pos() const { return M_pos; }
    [[nodiscard]] auto &pos() { return M_pos; }

    [[nodiscard]] const auto &vel() const { return M_vel; }
    [[nodiscard]] auto &vel() { return M_vel; }

    [[nodiscard]] const auto &ang_vel() const { return M_ang_vel; }
    [[nodiscard]] auto &ang_vel() { return M_ang_vel; }

    [[nodiscard]] const auto &orientation() const { return M_orientation; }
    [[nodiscard]] auto &orientation() { return M_orientation; }
    void orientation(const quat<one> &orient)
    {
        M_orientation = orient;
        update_derived_state();
    }

    [[nodiscard]] const auto &acc() const { return M_acc; }
    [[nodiscard]] const auto &torque_acc() const { return M_torque_acc; }

    [[nodiscard]] auto mass() const { return M_mass; }
    [[nodiscard]] auto inv_mass() const { return M_inv_mass; }

    void mass(quantity<si::kilogram> m)
    {
        M_mass = m;
        M_inv_mass = 1.0 / m;
    }

    [[nodiscard]] const auto &local_inertia_tensor() const { return M_inertia_tensor_local; }
    [[nodiscard]] const auto &local_inv_inertia_tensor() const
    {
        return M_inv_inertia_tensor_local;
    }
    [[nodiscard]] const auto &world_inertia_tensor() const { return M_inertia_tensor_world; }
    [[nodiscard]] const auto &world_inv_inertia_tensor() const
    {
        return M_inv_inertia_tensor_world;
    }

    void inertia_tensor(const mat3<si::kilogram * si::metre * si::metre> &tensor)
    {
        M_inertia_tensor_local = tensor;
        M_inv_inertia_tensor_local = tensor.inverse();
        update_derived_state();
    }

    [[nodiscard]] auto angular_accel() const
    {
        return M_inv_inertia_tensor_world *
               (M_torque_acc - M_ang_vel.cross(M_inertia_tensor_world * M_ang_vel));
    }

    void apply_force(const vec3<si::kilogram * si::metre / si::second / si::second> &force)
    {
        M_acc += force / M_mass;
    }

    void apply_force(const vec3<si::kilogram * si::metre / si::second / si::second> &force,
                     const vec3<si::metre> &application_point)
    {
        M_acc += force / M_mass;
        M_torque_acc += (application_point - M_pos).cross(force);
    }

    void apply_impulse(const vec3<si::kilogram * si::metre / si::second> &impulse)
    {
        M_vel += impulse / M_mass;
    }

    void
    apply_angular_impulse(const vec3<si::kilogram * si::metre * si::metre / si::second> &impulse)
    {
        M_ang_vel += M_inv_inertia_tensor_world * impulse;
    }

    void apply_acceleration(const vec3<si::metre / si::second / si::second> &a) { M_acc += a; }

    void
    apply_torque(const vec3<si::kilogram * si::metre * si::metre / si::second / si::second> &torque)
    {
        M_torque_acc += torque;
    }

    void clear_forces()
    {
        M_acc = vec3<si::metre / si::second / si::second>::zero();
        M_torque_acc = vec3<si::kilogram * si::metre * si::metre / si::second / si::second>::zero();
    }

    [[nodiscard]] auto project_to_local(const vec3<si::metre> &world_point) const
    {
        return M_orientation.conjugate() * (world_point - M_pos);
    }

    [[nodiscard]] auto project_to_world(const vec3<si::metre> &local_point) const
    {
        return M_orientation * local_point + M_pos;
    }

private:
    // Core State
    vec3<si::metre> M_pos = vec3<si::metre>::zero();
    vec3<si::metre / si::second> M_vel = vec3<si::metre / si::second>::zero();
    vec3<si::metre / si::second / si::second> M_acc =
        vec3<si::metre / si::second / si::second>::zero();
    quantity<si::kilogram> M_mass = 1.0 * si::kilogram;
    quantity<one / si::kilogram> M_inv_mass = 1.0 / M_mass;

    vec3<si::radian / si::second> M_ang_vel = vec3<si::radian / si::second>::zero();
    vec3<si::kilogram * si::metre * si::metre / si::second / si::second> M_torque_acc =
        vec3<si::kilogram * si::metre * si::metre / si::second / si::second>::zero();
    quat<one> M_orientation = quat<one>::identity();

    // Local-Space Constants
    mat3<si::kilogram * si::metre * si::metre> M_inertia_tensor_local =
        mat3<si::kilogram * si::metre * si::metre>::identity();
    mat3<one / (si::kilogram * si::metre * si::metre)> M_inv_inertia_tensor_local =
        mat3<one / (si::kilogram * si::metre * si::metre)>::identity();

    // Cached State
    mat3<one> M_rotation_matrix = mat3<one>::identity();
    mat3<si::kilogram * si::metre * si::metre> M_inertia_tensor_world =
        mat3<si::kilogram * si::metre * si::metre>::identity();
    mat3<one / (si::kilogram * si::metre * si::metre)> M_inv_inertia_tensor_world =
        mat3<one / (si::kilogram * si::metre * si::metre)>::identity();

    void update_derived_state()
    {
        M_rotation_matrix = M_orientation.to_rotation_matrix();
        auto r_t = M_rotation_matrix.transpose();
        M_inertia_tensor_world = M_rotation_matrix * M_inertia_tensor_local * r_t;
        M_inv_inertia_tensor_world = M_rotation_matrix * M_inv_inertia_tensor_local * r_t;
    }
};
} // namespace physkit
