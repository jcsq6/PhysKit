#pragma once
#include "mesh.h"
#include "sphere.h"
namespace physkit
{
enum shape_type {shape_sphere=0, shape_box=1, shape_cylinder=2, shape_pill=3, shape_convex_hull=4, shape_mesh=5};

class shape
{
public:
    shape(const std::shared_ptr<mesh> &m) : M_type{shape_mesh}
    {
        M_item.mesh = m;
    }
    shape (const std::shared_ptr<sphere> &s) :M_type{shape_sphere}
    {
        M_item.sphere = s;
    }
    [[nodiscard]] const aabb &bounds() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->bounds();
            break;
        case shape_type::shape_sphere:
            return  M_item.sphere->bounds();
            break;
        }
    }

    [[nodiscard]] const bounding_sphere &bsphere() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->bsphere();
            break;
        case shape_sphere:
            return M_item.sphere->bsphere();
        }
    }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->volume();
            break;
        case shape_sphere:
            return M_item.sphere->volume();
            break;
        }

    }

    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->mass_center();
            break;
        case shape_sphere:
            return M_item.sphere->mass_center();
            break;
        }
    }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->inertia_tensor(density);
            break;
        case shape_sphere:
            return M_item.sphere->inertia_tensor(density);
            break;
        }
    }

    /// @brief Ray intersection
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->ray_intersect(r, max_distance);
            break;
        case shape_sphere:
            return M_item.sphere->ray_intersect(r, max_distance);
            break;
        }
    }
    /// @brief Closest point on the surface
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->closest_point(point);
            break;
        case shape_sphere:
            return M_item.sphere->closest_point(point);
            break;
        }
    }
    /// @brief Point containment test in local space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->contains(point);
            break;
        case shape_sphere:
            return M_item.sphere->contains(point);
            break;
        }
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given sphere.
    std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->overlap_sphere(sphere);
            break;
        }
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->support(direction);
            break;
        case shape_sphere:
            return M_item.sphere->support(direction);
            break;
        }
    }
    [[nodiscard]] bool is_convex() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_item.mesh->is_convex();
            break;
        case shape_sphere:
            return true;
            break;
        }
    }
private:
    shape_type M_type;
    union
    {
        std::shared_ptr<physkit::mesh> mesh;
        std::shared_ptr<physkit::sphere> sphere;
    } M_item;
};
}
