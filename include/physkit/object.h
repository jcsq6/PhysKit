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
        self.M_data.pos = pos;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_vel(this auto &&self, const vec3<si::metre / si::second> &vel)
    {
        self.M_data.vel = vel;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_mass(this auto &&self, quantity<si::kilogram> mass)
    {
        self.M_data.mass = mass;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_orientation(this auto &&self, const mat3<one> &orient)
    {
        self.M_orientation = orient;
        return std::forward<decltype(self)>(self);
    }

    auto &&with_mesh(this auto &&self, std::shared_ptr<const physkit::mesh> msh)
    {
        self.M_mesh = std::move(msh);
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] const particle &data() const { return M_data; }
    [[nodiscard]] const mat3<one> &orientation() const { return M_orientation; }
    [[nodiscard]] auto &&mesh(this auto &&self)
    {
        return std::forward_like<decltype(self)>(self.M_mesh);
    }
    [[nodiscard]] body_type type() const { return M_type; }

private:
    object_desc(body_type type) : M_type(type) {}

    body_type M_type;
    particle M_data;
    mat3<one> M_orientation = mat3<one>::identity();
    std::shared_ptr<const physkit::mesh> M_mesh;
};

class object
{
public:
    object(object_desc desc)
        : M_mesh(std::move(desc).mesh()), M_info(desc.data()), M_orientation(desc.orientation()),
          M_type(desc.type())
    {
    }

    [[nodiscard]] const struct particle &particle() const { return M_info; }
    [[nodiscard]] struct particle &particle() { return M_info; }
    [[nodiscard]] const struct mesh &mesh() const { return *M_mesh; }
    [[nodiscard]] const mat3<one> &orientation() const { return M_orientation; }
    [[nodiscard]] mat3<one> &orientation() { return M_orientation; }
    [[nodiscard]] body_type type() const { return M_type; }
    [[nodiscard]] bool is_dynamic() const { return M_type == body_type::dynam; }
    [[nodiscard]] bool is_static() const { return M_type == body_type::stat; }

    void set_pos(const vec3<si::metre> &pos) { M_info.pos = pos; }
    void set_vel(const vec3<si::metre / si::second> &vel) { M_info.vel = vel; }
    void set_orientation(const mat3<one> &orient) { M_orientation = orient; }

    void apply_force(const vec3<si::kilogram * si::metre / si::second / si::second> &force)
    {
        M_info.apply_force(force);
    }

    /// @brief Return a lightweight view for world-space queries.
    [[nodiscard]] struct mesh::instance instance() const
    {
        return {*M_mesh, M_info.pos, M_orientation};
    }

private:
    std::shared_ptr<const struct mesh> M_mesh;
    struct particle M_info;
    mat3<one> M_orientation;
    body_type M_type;
};

} // namespace physkit