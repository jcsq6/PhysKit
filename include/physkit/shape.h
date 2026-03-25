#pragma once
#include "mesh.h"
#include "sphere.h"
#include "detail/bounds.h"
#include "detail/bvh.h"
#include "detail/types.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <cstdio>
#include <cassert>
#include <memory>
#include <span>
#include <vector>

#include <cstdio>

namespace physkit
{

enum shape_type {shape_sphere=0, shape_box=1, shape_cylinder=2, shape_pill=3, shape_convex_hull=4, shape_mesh=5};

class shape : public std::enable_shared_from_this<shape>
{
public:

    shape(const shape &) = default;
    shape &operator=(const shape &) = default;
    shape(shape &&) = default;
    shape &operator=(shape &&) = default;
    ~shape() = default;

    shape(const std::shared_ptr<const mesh> &m) : M_type(shape_mesh), M_mesh(m) {}
    shape(const std::shared_ptr<const sphere> &s) : M_type(shape_sphere), M_sphere(s) {}

    static std::shared_ptr<shape> make(const std::shared_ptr<const mesh> &m) { return std::make_shared<shape>(m);}
    static std::shared_ptr<shape> make(const std::shared_ptr<const sphere> &s) { return std::make_shared<shape>(s);}


    [[nodiscard]] std::shared_ptr<const struct mesh> get_mesh() const
    {
        assert(M_type == shape_mesh);
        return M_mesh;
    }
    [[nodiscard]] std::shared_ptr<const struct sphere> get_sphere() const
    {
        assert(M_type == shape_sphere);
        return M_sphere;
    }

    [[nodiscard]] const aabb &bounds() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->bounds();
        case shape_type::shape_sphere:
            return  M_sphere->bounds();
        }
    }

    [[nodiscard]] const bounding_sphere &bsphere() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->bsphere();
        case shape_sphere:
            return M_sphere->bsphere();
        }
    }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->volume();
        case shape_sphere:
            return M_sphere->volume();
        }

    }

    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->mass_center();
        case shape_sphere:
            return M_sphere->mass_center();
        }
    }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->inertia_tensor(density);
            break;
        case shape_sphere:
            return M_sphere->inertia_tensor(density);
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
            return M_mesh->ray_intersect(r, max_distance);
        case shape_sphere:
            return M_sphere->ray_intersect(r, max_distance);
        }
    }
    /// @brief Closest point on the surface
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->closest_point(point);
        case shape_sphere:
            return M_sphere->closest_point(point);
        }
    }
    /// @brief Point containment test in local space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->contains(point);
        case shape_sphere:
            return M_sphere->contains(point);
        }
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given sphere.
    std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        assert(M_type == shape_mesh);
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->overlap_sphere(sphere);
        }
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->support(direction);
        case shape_sphere:
            return M_sphere->support(direction);
        }
    }
    [[nodiscard]] bool is_convex() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_mesh->is_convex();
        case shape_sphere:
            return true;
        }
    }

    [[nodiscard]] shape_type type() const { return M_type;};

    //mesh only methods for compatibility
    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const
    {
        assert(M_type == shape_mesh);
        return M_mesh->vertices();
    }
    [[nodiscard]] std::span<const triangle_t> triangles() const
    {
        assert(M_type == shape_mesh);
        return M_mesh->triangles();
    }
    [[nodiscard]] const vec3<si::metre> &vertex(unsigned int index) const
    {
        assert(M_type == shape_mesh);
        assert(index < M_mesh->vertices().size());
        return M_mesh->vertices()[index];
    }

private:
    shape_type M_type;
    std::shared_ptr<const mesh> M_mesh;
    std::shared_ptr<const sphere> M_sphere;
};

/// @brief A view of a collision shape placed in world space
/// Provides world-space collision queries by transforming into local space and back.
/// Instances are meant for temporary use in queries and should not be stored long-term by
/// users. Watch out for dangling references.
class instance
{
public:
    instance(const shape &shp, const vec3<si::metre> &position,
             const quat<one> &orientation = quat<one>::identity());

    [[nodiscard]] const shape &geometry() const { return *M_shape; }
    [[nodiscard]] const vec3<si::metre> &position() const { return M_position; }
    [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }

    [[nodiscard]] vec3<si::metre> vertex(unsigned int index) const
    {
        assert(M_shape->type() == shape_mesh);
        assert(index < M_shape->vertices().size());
        return M_orientation * M_shape->vertices()[index] + M_position;
    }

    /// @brief Compute the world-space AABB by rotating the local AABB and translating.
    [[nodiscard]] aabb bounds() const { return M_shape->bounds() * M_orientation + M_position; }

    /// @brief Compute the world-space bounding sphere. Rotation-invariant — only the
    /// center is translated.
    [[nodiscard]] bounding_sphere bsphere() const
    {
        auto local = M_shape->bsphere();
        return {.center = M_orientation * local.center + M_position, .radius = local.radius};
    }

    /// @brief Compute the world-space ray intersection by transforming the ray into local space
    /// and back.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        // Transform ray into local space.
        auto inv_orient = M_orientation.conjugate();
        auto local_origin = inv_orient * (r.origin() - M_position);
        auto local_dir = inv_orient * r.direction();
        ray local_ray{local_origin, local_dir};

        auto hit = M_shape->ray_intersect(local_ray, max_distance);
        if (hit)
        {
            hit->pos = M_orientation * hit->pos + M_position;
            hit->normal = M_orientation * hit->normal;
        }
        return hit;
    }

    /// @brief Compute the closest point on the mesh surface to the given world-space point.
    /// O(log N) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_point = inv_orient * (point - M_position);
        auto local_closest = M_shape->closest_point(local_point);
        return M_orientation * local_closest + M_position;
    }

    /// @brief Point containment test in world space. O(log N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_point = inv_orient * (point - M_position);
        return M_shape->contains(local_point);
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given world-space sphere.
    [[nodiscard]] std::vector<std::uint32_t>
    overlap_sphere(const bounding_sphere &sphere) const
    {
        auto inv_orient = M_orientation.conjugate();
        bounding_sphere local_sphere{
            .center = inv_orient * (sphere.center - M_position),
            .radius = sphere.radius,
        };
        return M_shape->overlap_sphere(local_sphere);
    }

    /// @brief GJK support function in world space. Rotates the direction into
    /// local frame, queries the mesh, and transforms the result back.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_dir = inv_orient * direction;
        auto local_support = M_shape->support(local_dir);
        return M_orientation * local_support + M_position;
    }

    /// @brief Compute the inertia tensor rotated into the world frame and shifted to the
    /// instance's position via the parallel axis theorem.
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        throw std::runtime_error("not implemented");
        if (M_shape->type() == shape_mesh)
            throw std::runtime_error("mesh::instance::inertia_tensor not yet implemented");

        //TODO is this correct??? no but i'll get back to this later
        //return M_orientation * M_shape->inertia_tensor(density);
    }

private:
    const shape *M_shape; // NOLINT
    vec3<si::metre> M_position;
    quat<one> M_orientation;
};

inline instance::instance(const shape &shp, const vec3<si::metre> &position,
                                const quat<one> &orientation)
    : M_shape{&shp}, M_position{position}, M_orientation{orientation}
{
}

}
