#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "../detail/macros.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <cassert>
#include <memory>
#include <span>
#include <variant>
#include <vector>
#endif

#include "../algebra/types.h"
#include "bounds.h"
#include "bvh.h"
#include "mesh.h"

PHYSKIT_EXPORT
namespace physkit
{

class box
{
public:
    box(const box &) = default;
    box &operator=(const box &) = default;
    box(box &&) = default;
    box &operator=(box &&) = default;
    ~box() = default;

    box(const vec3<si::metre> &half_extents) : M_half_extents(half_extents)
    {
        M_aabb = aabb::from_points({half_extents, -half_extents});

        M_bsphere = bounding_sphere({0.0f * si::metre, 0.0f * si::metre, 0.0f * si::metre},
                                    half_extents.norm());
    }

    [[nodiscard]] const vec3<si::metre> &half_extents() const { return M_half_extents; }
    [[nodiscard]] const aabb &bounds() const;
    [[nodiscard]] const bounding_sphere &bsphere() const;

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const;
    [[nodiscard]] vec3<si::metre> mass_center() const;
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const;

    /// @brief Ray intersection in local (model) space. O(log N) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the box surface in local space. O(N) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space. O(N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const;

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

private:
    vec3<si::metre> M_half_extents;
    bounding_sphere M_bsphere;
    aabb M_aabb;
};

class sphere
{
public:
    /// @brief A view of a sphere placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.

    sphere(const sphere &) = default;
    sphere &operator=(const sphere &) = default;
    sphere(sphere &&) = default;
    sphere &operator=(sphere &&) = default;
    ~sphere() = default;

    sphere(const quantity<si::metre> radius) : M_radius{radius}
    {
        M_aabb = aabb::from_points({vec3<si::metre>{M_radius, M_radius, M_radius},
                                    vec3<si::metre>{-M_radius, -M_radius, -M_radius}});

        M_bsphere =
            bounding_sphere({0.0f * si::metre, 0.0f * si::metre, 0.0f * si::metre}, M_radius);
    }

    [[nodiscard]] const quantity<si::metre> &radius() const { return M_radius; }
    [[nodiscard]] const aabb &bounds() const;
    [[nodiscard]] const bounding_sphere &bsphere() const;

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const;
    [[nodiscard]] vec3<si::metre> mass_center() const;
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const;

    /// @brief Ray intersection in local (model) space. O(log N) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the sphere surface in local space. O(N) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space. O(N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const;

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

private:
    quantity<si::metre> M_radius;
    bounding_sphere M_bsphere;
    aabb M_aabb;
};

enum shape_type
{
    shape_sphere = 0,
    shape_box = 1,
    shape_cylinder = 2,
    shape_pill = 3,
    shape_convex_hull = 4,
    shape_mesh = 5
};

class shape
{
public:
    shape(const shape &other) : M_type(other.M_type) { copy_from(other); }

    shape &operator=(const shape &other)
    {
        if (this != &other)
        {
            destroy_active();
            M_type = other.M_type;
            copy_from(other);
        }
        return *this;
    }

    shape(shape &&other) noexcept : M_type(other.M_type) { move_from(std::move(other)); }

    shape &operator=(shape &&other) noexcept
    {
        if (this != &other)
        {
            destroy_active();
            M_type = other.M_type;
            move_from(std::move(other));
        }
        return *this;
    }
    ~shape() { destroy_active(); }

    shape() : M_type(shape_box) { construct_box(vec3{0.5f, 0.5f, 0.5f} * si::metre); }

    shape(std::shared_ptr<const physkit::mesh> m) : M_type(shape_mesh)
    { construct_mesh(std::move(m)); }

    shape(const physkit::sphere &s) : M_type(shape_sphere) { construct_sphere(s); }

    shape(const physkit::box &b) : M_type(shape_box) { construct_box(b); }

    [[nodiscard]] const std::shared_ptr<const physkit::mesh> &mesh() const
    {
        assert(M_type == shape_mesh);
        return M_storage.msh;
    }
    [[nodiscard]] const physkit::sphere &sphere() const
    {
        assert(M_type == shape_sphere);
        return M_storage.sph;
    }
    [[nodiscard]] const physkit::box &box() const
    {
        assert(M_type == shape_box);
        return M_storage.bx;
    }

    shape &operator=(std::shared_ptr<const physkit::mesh> m)
    {
        destroy_active();
        M_type = shape_mesh;
        construct_mesh(std::move(m));
        return *this;
    }

private:
    /// @brief Dispatch a callable over the active primitive, dereferencing the mesh pointer
    /// so the visitor always receives a reference to the underlying shape.
    template <typename F> decltype(auto) visit(F &&f) const
    {
        switch (M_type)
        {
        case shape_sphere:
            return std::forward<F>(f)(M_storage.sph);
        case shape_box:
            return std::forward<F>(f)(M_storage.bx);
        case shape_mesh:
            return std::forward<F>(f)(*M_storage.msh);
        default:
            std::unreachable();
        }
    }

public:
    [[nodiscard]] const aabb &bounds() const
    {
        return visit([](const auto &s) -> const aabb & { return s.bounds(); });
    }

    [[nodiscard]] const bounding_sphere &bsphere() const
    {
        return visit([](const auto &s) -> const bounding_sphere & { return s.bsphere(); });
    }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    {
        return visit([](const auto &s) { return s.volume(); });
    }

    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        return visit([](const auto &s) { return s.mass_center(); });
    }

    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        return visit([&](const auto &s) { return s.inertia_tensor(density); });
    }

    /// @brief Ray intersection
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        return visit([&](const auto &s) { return s.ray_intersect(r, max_distance); });
    }

    /// @brief Closest point on the surface
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        return visit([&](const auto &s) { return s.closest_point(point); });
    }

    /// @brief Point containment test in local space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        return visit([&](const auto &s) { return s.contains(point); });
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given sphere.
    std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        assert(M_type == shape_mesh);
        return M_storage.msh->overlap_sphere(sphere);
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        return visit([&](const auto &s) { return s.support(direction); });
    }

    [[nodiscard]] bool is_convex() const
    {
        switch (M_type)
        {
        default:
        case shape_mesh:
            return M_storage.msh->is_convex();
        case shape_sphere:
        case shape_box:
            return true;
        }
    }

    [[nodiscard]] shape_type type() const { return M_type; };

    // mesh only methods for compatibility
    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const
    {
        assert(M_type == shape_mesh);
        return M_storage.msh->vertices();
    }
    [[nodiscard]] std::span<const triangle_t> triangles() const
    {
        assert(M_type == shape_mesh);
        return M_storage.msh->triangles();
    }
    [[nodiscard]] const vec3<si::metre> &vertex(unsigned int index) const
    {
        assert(M_type == shape_mesh);
        const auto &msh = M_storage.msh;
        assert(index < msh->vertices().size());
        return msh->vertices()[index];
    }

private:
    shape_type M_type;
    union storage_t
    {
        std::shared_ptr<const physkit::mesh> msh;
        physkit::sphere sph;
        physkit::box bx;
        storage_t() {}
        ~storage_t() {}
    } M_storage;

    void destroy_active()
    {
        switch (M_type)
        {
        case shape_mesh:
            M_storage.msh.~shared_ptr();
            break;
        case shape_sphere:
            M_storage.sph.~sphere();
            break;
        case shape_box:
            M_storage.bx.~box();
            break;
        }
    }

    template <typename... Args> void construct_mesh(Args &&...args)
    { std::construct_at(&M_storage.msh, std::forward<Args>(args)...); }

    template <typename... Args> void construct_sphere(Args &&...args)
    { std::construct_at(&M_storage.sph, std::forward<Args>(args)...); }

    template <typename... Args> void construct_box(Args &&...args)
    { std::construct_at(&M_storage.bx, std::forward<Args>(args)...); }

    void copy_from(const shape &other)
    {
        switch (M_type)
        {
        case shape_mesh:
            construct_mesh(other.M_storage.msh);
            break;
        case shape_sphere:
            construct_sphere(other.M_storage.sph);
            break;
        case shape_box:
            construct_box(other.M_storage.bx);
            break;
        default:
            std::unreachable();
        }
    }

    void move_from(shape &&other)
    {
        switch (M_type)
        {
        case shape_mesh:
            construct_mesh(std::move(other.M_storage.msh));
            break;
        case shape_sphere:
            construct_sphere(std::move(other.M_storage.sph));
            break;
        case shape_box:
            construct_box(std::move(other.M_storage.bx));
            break;
        default:
            std::unreachable();
        }
    }
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
    [[nodiscard]] aabb bounds() const
    {
        auto ret = M_shape->bounds() * M_orientation + M_position;
        return M_shape->bounds() * M_orientation + M_position;
    }

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
        auto ret = M_orientation * local_closest + M_position;
        return M_orientation * local_closest + M_position;
    }

    /// @brief Point containment test in world space. O(log N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_point = inv_orient * (point - M_position);
        auto ret = M_shape->contains(local_point);
        return M_shape->contains(local_point);
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given world-space sphere.
    [[nodiscard]] std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        auto inv_orient = M_orientation.conjugate();
        bounding_sphere local_sphere{
            .center = inv_orient * (sphere.center - M_position),
            .radius = sphere.radius,
        };
        auto ret = M_shape->overlap_sphere(local_sphere);
        return M_shape->overlap_sphere(local_sphere);
    }

    /// @brief GJK support function in world space. Rotates the direction into
    /// local frame, queries the mesh, and transforms the result back.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_dir = inv_orient * direction;
        auto local_support = M_shape->support(local_dir);
        auto ret = M_orientation * local_support + M_position;
        return M_orientation * local_support + M_position;
    }

    /// @brief Compute the inertia tensor rotated into the world frame and shifted to the
    /// instance's position via the parallel axis theorem.
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        // TODO is this correct???
        return M_shape->inertia_tensor(density) * M_orientation.to_rotation_matrix();
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

} // namespace physkit
