#pragma once
#include "../collision/mesh.h"
#include "../collision/shape.h"
#include "particle.h"

PHYSKIT_EXPORT
namespace physkit
{

enum class body_type : std::uint8_t
{
    dynam,
    stat
};

class object_desc
{
public:
    [[nodiscard]] static object_desc dynam() { return {body_type::dynam}; }

    [[nodiscard]] static object_desc stat() { return {body_type::stat}; }

    object_desc(body_type type) : M_type(type) {}

    auto &&with_pos(this auto &&self, const vec3<si::metre> &pos)
    {
        self.M_pos = pos;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_vel(this auto &&self, const vec3<si::metre / si::second> &vel)
    {
        self.M_vel = vel;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_mass(this auto &&self, quantity<si::kilogram> mass)
    {
        self.M_mass = mass;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_orientation(this auto &&self, const quat<one> &orient)
    {
        self.M_orientation = orient;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_ang_vel(this auto &&self, const vec3<si::radian / si::second> &ang_vel)
    {
        self.M_ang_vel = ang_vel;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_inertia_tensor(this auto &&self,
                               const mat3<si::kilogram * si::metre * si::metre> &inertia)
    {
        self.M_inertia = inertia;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_mesh(this auto &&self, std::shared_ptr<const physkit::mesh> msh)
    {
        self.M_shape = std::move(msh);
        return std::forward<decltype(self)>(self);
    }
    auto &&with_shape(this auto &&self, physkit::shape shp)
    {
        self.M_shape = std::move(shp);
        return std::forward<decltype(self)>(self);
    }

    auto &&with_restitution(this auto &&self, double restitution)
    {
        self.M_restitution = restitution;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_friction(this auto &&self, double friction)
    {
        self.M_friction = friction;
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] const vec3<si::metre> &pos() const { return M_pos; }
    [[nodiscard]] const vec3<si::metre / si::second> &vel() const { return M_vel; }
    [[nodiscard]] quantity<si::kilogram> mass() const
    {
        return M_type == body_type::dynam ? M_mass
                                          : std::numeric_limits<quantity<si::kilogram>>::infinity();
    }
    [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }
    [[nodiscard]] const vec3<si::radian / si::second> &ang_vel() const { return M_ang_vel; }
    [[nodiscard]] mat3<si::kilogram * si::metre * si::metre> inertia_tensor() const
    {
        if (M_inertia.has_value()) return *M_inertia;

        if (M_type == body_type::stat)
            return vec3{
                std::numeric_limits<quantity<si::kilogram * si::metre * si::metre>>::infinity(),
                std::numeric_limits<quantity<si::kilogram * si::metre * si::metre>>::infinity(),
                std::numeric_limits<quantity<si::kilogram * si::metre * si::metre>>::infinity()}
                .as_diagonal();

        auto density = M_mass / M_shape.volume();
        return M_shape.inertia_tensor(density);
    }
    [[nodiscard]] double restitution() const { return M_restitution; }
    [[nodiscard]] double friction() const { return M_friction; }

    [[nodiscard]] auto &&mesh(this auto &&self)
    { return std::forward_like<decltype(self)>(self.M_shape.mesh()); }
    [[nodiscard]] auto &&shape(this auto &&self)
    { return std::forward_like<decltype(self)>(self.M_shape); }
    [[nodiscard]] body_type type() const { return M_type; }

private:
    body_type M_type;
    vec3<si::metre> M_pos = vec3<si::metre>::zero();
    vec3<si::metre / si::second> M_vel = vec3<si::metre / si::second>::zero();
    quantity<si::kilogram> M_mass = 1.0 * si::kilogram;
    quat<one> M_orientation = quat<one>::identity();
    vec3<si::radian / si::second> M_ang_vel = vec3<si::radian / si::second>::zero();
    std::optional<mat3<si::kilogram * si::metre * si::metre>> M_inertia;
    physkit::shape M_shape;
    double M_restitution = 0.5;
    double M_friction = 0.5;
};

class object : public rigid_body
{
public:
    explicit object(object_desc desc)
        : rigid_body(desc.pos(), desc.vel(), desc.mass(), desc.orientation(), desc.ang_vel(),
                     desc.inertia_tensor()),
          M_type(desc.type()), M_shape(std::move(desc).shape()), M_restitution(desc.restitution()),
          M_friction(desc.friction())
    {
    }

    [[nodiscard]] const struct shape &shape() const { return M_shape; }

    void shape(physkit::shape new_shape)
    {
        M_shape = std::move(new_shape);
        if (is_dynamic())
            inertia_tensor(M_shape.inertia_tensor(rigid_body::mass() / M_shape.volume()));
    }

    /// @brief Update the mass and automatically recalculate the physics inertia tensor
    void mass(quantity<si::kilogram> new_mass)
    {
        rigid_body::mass(new_mass);
        if (is_dynamic()) inertia_tensor(M_shape.inertia_tensor(new_mass / M_shape.volume()));
    }

    [[nodiscard]] auto mass() const { return rigid_body::mass(); }

    [[nodiscard]] body_type type() const { return M_type; }
    [[nodiscard]] bool is_dynamic() const { return M_type == body_type::dynam; }
    [[nodiscard]] bool is_static() const { return M_type == body_type::stat; }
    [[nodiscard]] double restitution() const { return M_restitution; }
    [[nodiscard]] double friction() const { return M_friction; }

    void restitution(double r) { M_restitution = r; }
    void friction(double f) { M_friction = f; }

    /// @brief Return a lightweight view for world-space queries.
    [[nodiscard]] physkit::instance instance() const { return {M_shape, pos(), orientation()}; }

private:
    physkit::shape M_shape;
    body_type M_type;
    double M_restitution;
    double M_friction;
};

} // namespace physkit