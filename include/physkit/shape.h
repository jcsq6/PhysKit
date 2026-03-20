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

    [[nodiscard]] shape_type type() const { return M_type;};

    //mesh only methods
    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const
    {
        assert(M_type == shape_mesh);
        return M_item.mesh->vertices();
    }
    [[nodiscard]] std::span<const triangle_t> triangles() const
    {
        assert(M_type == shape_mesh);
        return M_item.mesh->triangles();
    }
    [[nodiscard]] const vec3<si::metre> &vertex(unsigned int index) const
    {
        assert(M_type == shape_mesh);
        assert(index < M_item.mesh->vertices().size());
        return M_item.mesh->vertices()[index];
    }

private:
    shape_type M_type;
    union
    {
        std::shared_ptr<physkit::mesh> mesh;
        std::shared_ptr<physkit::sphere> sphere;
    } M_item;
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
        if (M_shape->type() == shape_mesh)
            throw std::runtime_error("mesh::instance::inertia_tensor not yet implemented");

        //TODO is this correct???
        return M_orientation * M_shape->inertia_tensor(density);
    }

private:
    const shape *M_shape; // NOLINT
    vec3<si::metre> M_position;
    quat<one> M_orientation;
};
}
