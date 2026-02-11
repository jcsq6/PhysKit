#pragma once
#include "mesh.h"
#include "particle.h"

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

    auto &&with_mesh(this auto &&self, std::shared_ptr<const physkit::mesh> msh)
    {
        self.M_mesh = std::move(msh);
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] const vec3<si::metre> &pos() const { return M_pos; }
    [[nodiscard]] const vec3<si::metre / si::second> &vel() const { return M_vel; }
    [[nodiscard]] quantity<si::kilogram> mass() const { return M_mass; }
    [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }
    [[nodiscard]] auto &&mesh(this auto &&self)
    {
        return std::forward_like<decltype(self)>(self.M_mesh);
    }
    [[nodiscard]] body_type type() const { return M_type; }

private:
    object_desc(body_type type) : M_type(type) {}

    body_type M_type;
    vec3<si::metre> M_pos = vec3<si::metre>::zero();
    vec3<si::metre / si::second> M_vel = vec3<si::metre / si::second>::zero();
    quantity<si::kilogram> M_mass = 1.0 * si::kilogram;
    quat<one> M_orientation = quat<one>::identity();
    std::shared_ptr<const physkit::mesh> M_mesh;
};

class object : public particle
{
public:
    explicit object(object_desc desc)
        : particle(desc.pos(), desc.vel(), desc.mass(), desc.orientation()), M_type(desc.type()),
          M_mesh(std::move(desc).mesh())
    {
    }

    [[nodiscard]] const struct mesh &mesh() const
    {
        assert(M_mesh != nullptr);
        return *M_mesh;
    }
    [[nodiscard]] body_type type() const { return M_type; }
    [[nodiscard]] bool is_dynamic() const { return M_type == body_type::dynam; }
    [[nodiscard]] bool is_static() const { return M_type == body_type::stat; }

    /// @brief Return a lightweight view for world-space queries.
    [[nodiscard]] struct mesh::instance instance() const { return {*M_mesh, pos(), orientation()}; }

private:
    std::shared_ptr<const struct mesh> M_mesh;
    body_type M_type;
};

} // namespace physkit