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
        : M_pos(pos), M_vel(vel), M_mass(mass), M_orientation(orientation), M_ang_vel(ang_vel),
          M_inertia_tensor(inertia), M_inv_inertia_tensor(inertia.inverse())
    {
    }

    [[nodiscard]] const auto &pos() const { return M_pos; }
    [[nodiscard]] auto &pos() { return M_pos; }

    [[nodiscard]] const auto &vel() const { return M_vel; }
    [[nodiscard]] auto &vel() { return M_vel; }

    [[nodiscard]] const auto &ang_vel() const { return M_ang_vel; }
    [[nodiscard]] auto &ang_vel() { return M_ang_vel; }

    [[nodiscard]] const auto &orientation() const { return M_orientation; }
    [[nodiscard]] auto &orientation() { return M_orientation; }

    [[nodiscard]] const auto &acc() const { return M_acc; }
    [[nodiscard]] const auto &torque_acc() const { return M_torque_acc; }

    [[nodiscard]] auto mass() const { return M_mass; }

    void mass(quantity<si::kilogram> m) { M_mass = m; }

    [[nodiscard]] const auto &inertia_tensor() const { return M_inertia_tensor; }

    [[nodiscard]] const auto &inv_inertia_tensor() const { return M_inv_inertia_tensor; }

    void inertia_tensor(const mat3<si::kilogram * si::metre * si::metre> &tensor)
    {
        M_inertia_tensor = tensor;
        M_inv_inertia_tensor = tensor.inverse();
    }

    [[nodiscard]] auto angular_accel() const
    {
        auto r = M_orientation.to_rotation_matrix();
        auto r_t = r.transpose();
        auto i_inv_world = r * M_inv_inertia_tensor * r_t;
        auto i_world = r * M_inertia_tensor * r_t;
        return i_inv_world * (M_torque_acc - M_ang_vel.cross(i_world * M_ang_vel));
    }

    void apply_force(const vec3<si::kilogram * si::metre / si::second / si::second> &force)
    { M_acc += force / M_mass; }

    void apply_force(const vec3<si::kilogram * si::metre / si::second / si::second> &force,
                     const vec3<si::metre> &application_point)
    {
        M_acc += force / M_mass;
        M_torque_acc += (application_point - M_pos).cross(force);
    }

    void apply_acceleration(const vec3<si::metre / si::second / si::second> &a) { M_acc += a; }

    void
    apply_torque(const vec3<si::kilogram * si::metre * si::metre / si::second / si::second> &torque)
    { M_torque_acc += torque; }

    void clear_forces()
    {
        M_acc = vec3<si::metre / si::second / si::second>::zero();
        M_torque_acc = vec3<si::kilogram * si::metre * si::metre / si::second / si::second>::zero();
    }

    [[nodiscard]] auto project_local(const vec3<si::metre> &point) const
    {
        auto r = M_orientation.to_rotation_matrix();
        return r.transpose() * (point - M_pos);
    }

private:
    vec3<si::metre> M_pos = vec3<si::metre>::zero();
    vec3<si::metre / si::second> M_vel = vec3<si::metre / si::second>::zero();
    vec3<si::metre / si::second / si::second> M_acc =
        vec3<si::metre / si::second / si::second>::zero();
    quantity<si::kilogram> M_mass = 1.0 * si::kilogram;

    vec3<si::radian / si::second> M_ang_vel = vec3<si::radian / si::second>::zero();
    mat3<si::kilogram * si::metre * si::metre> M_inertia_tensor =
        mat3<si::kilogram * si::metre * si::metre>::identity();
    mat3<one / (si::kilogram * si::metre * si::metre)> M_inv_inertia_tensor =
        mat3<one / (si::kilogram * si::metre * si::metre)>::identity();
    vec3<si::kilogram * si::metre * si::metre / si::second / si::second> M_torque_acc =
        vec3<si::kilogram * si::metre * si::metre / si::second / si::second>::zero();

    quat<one> M_orientation = quat<one>::identity();
};
} // namespace physkit
