#pragma once
#include "lin_alg.h"
#include "types.h"

#include <algorithm>
#include <numbers>
#include <span>

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
    [[nodiscard]] constexpr auto surface_area() const
    {
        auto s = size();
        return 2.0 * (s.x() * s.y() + s.y() * s.z() + s.z() * s.x());
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

    [[nodiscard]] constexpr bool contains(const aabb &other) const
    {
        return (other.min.x() >= min.x() && other.max.x() <= max.x()) &&
               (other.min.y() >= min.y() && other.max.y() <= max.y()) &&
               (other.min.z() >= min.z() && other.max.z() <= max.z());
    }

    [[nodiscard]] constexpr bool intersects(const aabb &other) const
    {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }

    [[nodiscard]] friend aabb aabb_union(const aabb &a, const aabb &b)
    {
        return aabb{
            .min = {std::min(a.min.x(), b.min.x()), std::min(a.min.y(), b.min.y()),
                    std::min(a.min.z(), b.min.z())},
            .max = {std::max(a.max.x(), b.max.x()), std::max(a.max.y(), b.max.y()),
                    std::max(a.max.z(), b.max.z())},
        };
    }

    [[nodiscard]] constexpr auto squared_distance_to(const vec3<si::metre> &point) const
    {
        auto dx = std::max({min.x() - point.x(), 0.0 * si::metre, point.x() - max.x()});
        auto dy = std::max({min.y() - point.y(), 0.0 * si::metre, point.y() - max.y()});
        auto dz = std::max({min.z() - point.z(), 0.0 * si::metre, point.z() - max.z()});
        return dx * dx + dy * dy + dz * dz;
    }

    [[nodiscard]] constexpr aabb operator+(const vec3<si::metre> &offset) const
    { return {.min = min + offset, .max = max + offset}; }

    [[nodiscard]] constexpr aabb operator-(const vec3<si::metre> &offset) const
    { return {.min = min - offset, .max = max - offset}; }

    [[nodiscard]] constexpr aabb operator*(quantity<one> scale) const
    {
        assert(scale >= 0.0);
        return {.min = min * scale, .max = max * scale};
    }

    template <Quantity Q>
        requires(QuantityOf<Q, dimensionless>)
    [[nodiscard]] aabb operator*(const unit_mat<Q, 3, 3> &transform) const
    {
        aabb result{.min = transform * min, .max = transform * min}; // initialize to point(0)
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
        aabb result{.min = transform * min, .max = transform * min}; // initialize to point(0)
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

    [[nodiscard]] constexpr aabb operator/(quantity<one> scale) const
    { return {.min = min / scale, .max = max / scale}; }

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

    constexpr auto &operator*=(quantity<one> scale)
    {
        assert(scale >= 0.0);
        min *= scale;
        max *= scale;
        return *this;
    }

    constexpr auto &operator/=(quantity<one> scale)
    {
        assert(scale >= 0.0);
        min /= scale;
        max /= scale;
        return *this;
    }
};

/// @brief A bounding sphere defined by a center and radius.
class bounding_sphere
{
public:
    vec3<si::metre> center = vec3<si::metre>::zero();
    quantity<si::metre> radius{};

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
    { return 4.0 * std::numbers::pi * radius * radius; }

    [[nodiscard]] constexpr auto volume() const
    { return (4.0 / 3.0) * std::numbers::pi * radius * radius * radius; }

    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    { return (point - center).squared_norm() <= radius * radius; }

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
    { return {.center = center + offset, .radius = radius}; }

    [[nodiscard]] constexpr bounding_sphere operator-(const vec3<si::metre> &offset) const
    { return {.center = center - offset, .radius = radius}; }

    [[nodiscard]] constexpr bounding_sphere operator*(quantity<one> scale) const
    {
        assert(scale >= 0.0);
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

    constexpr auto &operator*=(quantity<one> scale)
    {
        assert(scale >= 0.0);
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
} // namespace physkit