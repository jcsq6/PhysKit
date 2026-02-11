#pragma once
#include "detail/types.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <algorithm>
#include <cassert>
#include <memory>
#include <numbers>
#include <span>
#include <vector>

namespace physkit
{
/// @brief An axis-aligned bounding box defined by minimum and maximum corners.
class aabb
{
public:
    vec3<si::metre> min;
    vec3<si::metre> max;

    static aabb from_points(std::span<const vec3<si::metre>> points)
    {
        assert(!points.empty());
        aabb box;
        box.min = points[0];
        box.max = points[0];
        for (std::size_t i = 1; i < points.size(); ++i)
        {
            box.min.x(std::min(box.min.x(), points[i].x()));
            box.min.y(std::min(box.min.y(), points[i].y()));
            box.min.z(std::min(box.min.z(), points[i].z()));
            box.max.x(std::max(box.max.x(), points[i].x()));
            box.max.y(std::max(box.max.y(), points[i].y()));
            box.max.z(std::max(box.max.z(), points[i].z()));
        }
        return box;
    }

    [[nodiscard]] constexpr auto size() const { return max - min; }
    [[nodiscard]] constexpr auto center() const { return (min + max) / 2.0; }
    [[nodiscard]] constexpr auto extent() const { return (max - min) / 2.0; }
    [[nodiscard]] constexpr auto volume() const
    {
        auto s = size();
        return s.x() * s.y() * s.z();
    }

    [[nodiscard]] constexpr auto point(unsigned int index) const
    {
        assert(index < 8);
        return vec3<si::metre>{((index & 1) != 0u) ? max.x() : min.x(),
                               ((index & 2) != 0u) ? max.y() : min.y(),
                               ((index & 4) != 0u) ? max.z() : min.z()};
    }

    [[nodiscard]] constexpr bool contains(const vec3<si::metre> &point) const
    {
        return (point.x() >= min.x() && point.x() <= max.x()) &&
               (point.y() >= min.y() && point.y() <= max.y()) &&
               (point.z() >= min.z() && point.z() <= max.z());
    }

    [[nodiscard]] constexpr bool intersects(const aabb &other) const
    {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }

    [[nodiscard]] constexpr aabb operator+(const vec3<si::metre> &offset) const
    {
        return {.min = min + offset, .max = max + offset};
    }

    [[nodiscard]] constexpr aabb operator-(const vec3<si::metre> &offset) const
    {
        return {.min = min - offset, .max = max - offset};
    }

    [[nodiscard]] constexpr aabb operator*(float_t scale) const
    {
        auto center = this->center();
        auto half_size = size() * (scale / 2.0f);
        return {.min = center - half_size, .max = center + half_size};
    }

    [[nodiscard]] constexpr aabb operator*(QuantityOf<dimensionless> auto scale) const
    {
        auto center = this->center();
        auto half_size = size() * (scale / 2.0f);
        return {.min = center - half_size, .max = center + half_size};
    }

    template <Quantity Q>
        requires(QuantityOf<Q, dimensionless>)
    [[nodiscard]] aabb operator*(const unit_mat<Q, 3, 3> &transform) const
    {
        aabb result{.min = transform * min, .max = transform * min};
        for (unsigned int i = 1; i < 8; ++i)
        {
            auto pt = transform * point(i);
            result.min.x(std::min(result.min.x(), pt.x()));
            result.min.y(std::min(result.min.y(), pt.y()));
            result.min.z(std::min(result.min.z(), pt.z()));
            result.max.x(std::max(result.max.x(), pt.x()));
            result.max.y(std::max(result.max.y(), pt.y()));
            result.max.z(std::max(result.max.z(), pt.z()));
        }
        return result;
    }

    template <Quantity Q>
        requires(QuantityOf<Q, dimensionless>)
    [[nodiscard]] aabb operator*(const unit_quat<Q> &transform) const
    {
        aabb result{.min = transform * min, .max = transform * min};
        for (unsigned int i = 1; i < 8; ++i)
        {
            auto pt = transform * point(i);
            result.min.x(std::min(result.min.x(), pt.x()));
            result.min.y(std::min(result.min.y(), pt.y()));
            result.min.z(std::min(result.min.z(), pt.z()));
            result.max.x(std::max(result.max.x(), pt.x()));
            result.max.y(std::max(result.max.y(), pt.y()));
            result.max.z(std::max(result.max.z(), pt.z()));
        }
        return result;
    }

    [[nodiscard]] constexpr aabb operator/(float_t scale) const
    {
        auto center = this->center();
        auto half_size = size() * (.5f / scale);
        return {.min = center - half_size, .max = center + half_size};
    }

    [[nodiscard]] constexpr aabb operator/(QuantityOf<dimensionless> auto scale) const
    {
        auto center = this->center();
        auto half_size = size() * (.5f / scale);
        return {.min = center - half_size, .max = center + half_size};
    }

    constexpr auto &operator+=(const vec3<si::metre> &offset)
    {
        min += offset;
        max += offset;
        return *this;
    }

    constexpr auto &operator-=(const vec3<si::metre> &offset)
    {
        min -= offset;
        max -= offset;
        return *this;
    }

    constexpr auto &operator*=(float_t scale)
    {
        auto c = center();
        auto h = size() * (scale / 2.0f);
        min = c - h;
        max = c + h;
        return *this;
    }

    constexpr auto &operator*=(QuantityOf<dimensionless> auto scale)
    {
        auto c = center();
        auto h = size() * (scale / 2.0f);
        min = c - h;
        max = c + h;
        return *this;
    }

    constexpr auto &operator/=(float_t scale)
    {
        auto c = center();
        auto h = size() * (.5f / scale);
        min = c - h;
        max = c + h;
        return *this;
    }

    constexpr auto &operator/=(QuantityOf<dimensionless> auto scale)
    {
        auto c = center();
        auto h = size() * (.5f / scale);
        min = c - h;
        max = c + h;
        return *this;
    }
};

/// @brief A bounding sphere defined by a center and radius.
class bounding_sphere
{
public:
    vec3<si::metre> center;
    quantity<si::metre> radius;

    /// @brief Ritter's bounding-sphere algorithm.
    [[nodiscard]] static bounding_sphere from_points(std::span<const vec3<si::metre>> points)
    {
        assert(!points.empty());

        // --- Pass 1: find the two most-separated points along each axis ---
        std::size_t min_x = 0;
        std::size_t max_x = 0;
        std::size_t min_y = 0;
        std::size_t max_y = 0;
        std::size_t min_z = 0;
        std::size_t max_z = 0;

        for (std::size_t i = 1; i < points.size(); ++i)
        {
            if (points[i].x() < points[min_x].x()) min_x = i;
            if (points[i].x() > points[max_x].x()) max_x = i;
            if (points[i].y() < points[min_y].y()) min_y = i;
            if (points[i].y() > points[max_y].y()) max_y = i;
            if (points[i].z() < points[min_z].z()) min_z = i;
            if (points[i].z() > points[max_z].z()) max_z = i;
        }

        auto span_x = (points[max_x] - points[min_x]).squared_norm();
        auto span_y = (points[max_y] - points[min_y]).squared_norm();
        auto span_z = (points[max_z] - points[min_z]).squared_norm();

        auto lo = min_x;
        auto hi = max_x;
        auto best = span_x;
        if (span_y > best)
        {
            lo = min_y;
            hi = max_y;
            best = span_y;
        }
        if (span_z > best)
        {
            lo = min_z;
            hi = max_z;
        }

        auto c = (points[lo] + points[hi]) / 2.0;
        auto r = (points[hi] - c).norm();

        for (const auto &pt : points)
        {
            auto dist = (pt - c).norm();
            if (dist > r)
            {
                auto new_r = (r + dist) / 2.0;
                auto shift = dist - r;
                c = c + (pt - c) * (shift / (2.0 * dist));
                r = new_r;
            }
        }

        return {.center = c, .radius = r};
    }

    [[nodiscard]] static bounding_sphere from_aabb(const aabb &box)
    {
        auto c = box.center();
        auto r = (box.max - c).norm();
        return {.center = c, .radius = r};
    }

    /// @brief Compute the smallest sphere enclosing two spheres.
    [[nodiscard]] static bounding_sphere merge(const bounding_sphere &a, const bounding_sphere &b)
    {
        auto d_vec = b.center - a.center;
        auto dist = d_vec.norm();

        if (dist + b.radius <= a.radius) return a;
        if (dist + a.radius <= b.radius) return b;

        auto new_r = (dist + a.radius + b.radius) / 2.0;
        auto new_c = a.center + d_vec * ((new_r - a.radius) / dist);
        return {.center = new_c, .radius = new_r};
    }

    [[nodiscard]] constexpr auto surface_area() const
    {
        return 4.0 * std::numbers::pi * radius * radius;
    }

    [[nodiscard]] constexpr auto volume() const
    {
        return (4.0 / 3.0) * std::numbers::pi * radius * radius * radius;
    }

    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        return (point - center).squared_norm() <= radius * radius;
    }

    [[nodiscard]] bool intersects(const bounding_sphere &other) const
    {
        auto combined = radius + other.radius;
        return (other.center - center).squared_norm() <= combined * combined;
    }

    [[nodiscard]] bool intersects(const aabb &box) const
    {
        auto closest = closest_point_on_aabb(box);
        return (closest - center).squared_norm() <= radius * radius;
    }

    [[nodiscard]] constexpr bounding_sphere operator+(const vec3<si::metre> &offset) const
    {
        return {.center = center + offset, .radius = radius};
    }

    [[nodiscard]] constexpr bounding_sphere operator-(const vec3<si::metre> &offset) const
    {
        return {.center = center - offset, .radius = radius};
    }

    [[nodiscard]] constexpr bounding_sphere operator*(float_t scale) const
    {
        return {.center = center, .radius = radius * scale};
    }

    [[nodiscard]] constexpr bounding_sphere operator*(QuantityOf<dimensionless> auto scale) const
    {
        return {.center = center, .radius = radius * scale};
    }

    constexpr auto &operator+=(const vec3<si::metre> &offset)
    {
        center += offset;
        return *this;
    }

    constexpr auto &operator-=(const vec3<si::metre> &offset)
    {
        center -= offset;
        return *this;
    }

    constexpr auto &operator*=(float_t scale)
    {
        radius = radius * scale;
        return *this;
    }

    constexpr auto &operator*=(QuantityOf<dimensionless> auto scale)
    {
        radius = radius * scale;
        return *this;
    }

private:
    [[nodiscard]] vec3<si::metre> closest_point_on_aabb(const aabb &box) const
    {
        auto cx = std::clamp(center.x(), box.min.x(), box.max.x());
        auto cy = std::clamp(center.y(), box.min.y(), box.max.y());
        auto cz = std::clamp(center.z(), box.min.z(), box.max.z());
        return {cx, cy, cz};
    }
};

class mesh
{
public:
    struct instance;

    struct triangle_t : std::array<unsigned int, 3>
    {
        [[nodiscard]] auto normal(const mesh &m) const
        {
            auto v0 = m.M_vertices[(*this)[0]];
            auto v1 = m.M_vertices[(*this)[1]];
            auto v2 = m.M_vertices[(*this)[2]];
            return (v1 - v0).cross(v2 - v0).normalized();
        }

        [[nodiscard]] vec3<one> normal(const instance &inst) const;
    };

    struct ray_hit
    {
        vec3<si::metre> pos;
        vec3<one> normal;
        quantity<si::metre> distance;
    };

    struct ray
    {
        vec3<si::metre> origin;
        vec3<one> direction;
    };

    /// @brief A lightweight view of a mesh placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.
    /// Not intended for long-term storage. Create one, query it, discard it. Avoid dangling
    /// references.
    class instance
    {
    public:
        instance(const mesh &msh, const vec3<si::metre> &position,
                 const quat<one> &orientation = quat<one>::identity())
            : M_mesh{msh}, M_position{position}, M_orientation{orientation}
        {
        }

        [[nodiscard]] const mesh &geometry() const { return M_mesh; }
        [[nodiscard]] const vec3<si::metre> &position() const { return M_position; }
        [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }

        [[nodiscard]] auto vertex(unsigned int index) const
        {
            assert(index < M_mesh.vertices().size());
            return M_orientation * M_mesh.vertices()[index] + M_position;
        }

        /// @brief Compute the world-space AABB by rotating the local AABB and translating.
        [[nodiscard]] aabb bounds() const;

        /// @brief Compute the world-space bounding sphere. Rotation-invariant — only the
        /// center is translated.
        [[nodiscard]] bounding_sphere bsphere() const
        {
            auto local = M_mesh.bsphere();
            return {.center = M_orientation * local.center + M_position, .radius = local.radius};
        }

        [[nodiscard]] std::optional<ray_hit>
        ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                        std::numeric_limits<quantity<si::metre>>::infinity()) const;

        [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;

        /// @brief Point containment test in world space.
        [[nodiscard]] bool contains(const vec3<si::metre> &point) const;

        /// @brief GJK support function in world space. Rotates the direction into
        /// local frame, queries the mesh, and transforms the result back.
        [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;

        /// @brief Compute the inertia tensor rotated into the world frame and shifted to the
        /// instance's position via the parallel axis theorem.
        [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
        inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const;

    private:
        const mesh &M_mesh; // NOLINT
        vec3<si::metre> M_position;
        quat<one> M_orientation;
    };

    mesh() = default;
    mesh(const mesh &) = default;
    mesh &operator=(const mesh &) = default;
    mesh(mesh &&) = default;
    mesh &operator=(mesh &&) = default;
    ~mesh() = default;

    static std::shared_ptr<mesh> make(std::span<const vec3<si::metre>> vertices,
                                      std::span<const triangle_t> triangles);

    /// @brief Create a box mesh centered at the origin with the given half-extents.
    static std::shared_ptr<mesh> box(const vec3<si::metre> &half_extents);

    /// @brief Create a UV-sphere mesh centered at the origin.
    /// @param radius Sphere radius.
    /// @param stacks Number of horizontal divisions (latitude, >= 2).
    /// @param sectors Number of vertical divisions (longitude, >= 3).
    static std::shared_ptr<mesh> sphere(quantity<si::metre> radius, unsigned int stacks = 16,
                                        unsigned int sectors = 32);

    /// @brief Create a square-base pyramid with base centered at the origin and apex at
    /// (0, height, 0).
    /// @param base_half Half the side length of the square base.
    /// @param height Height from base to apex.
    static std::shared_ptr<mesh> pyramid(quantity<si::metre> base_half, quantity<si::metre> height);

    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const { return M_vertices; }
    [[nodiscard]] std::span<const triangle_t> triangles() const { return M_triangles; }
    [[nodiscard]] const auto &vertex(unsigned int index) const
    {
        assert(index < M_vertices.size());
        return M_vertices[index];
    }

    [[nodiscard]] const aabb &bounds() const { return M_bounds; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const;
    [[nodiscard]] vec3<si::metre> mass_center() const;
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const;

    /// @brief Ray intersection in local (model) space.
    [[nodiscard]] std::optional<ray_hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the mesh surface in local space.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const;

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;
    [[nodiscard]] bool is_convex() const;

    /// @brief Create an instance view of this mesh at the given position and orientation.
    [[nodiscard]] instance at(const vec3<si::metre> &position,
                              const quat<one> &orientation = quat<one>::identity()) const
    {
        return {*this, position, orientation};
    }

private:
    aabb M_bounds;
    bounding_sphere M_bsphere;
    std::vector<vec3<si::metre>> M_vertices;
    std::vector<triangle_t> M_triangles;
};

inline vec3<one> mesh::triangle_t::normal(const mesh::instance &inst) const
{
    return inst.orientation() * this->normal(inst.geometry());
}
} // namespace physkit
