#pragma once
#include "../algebra/types.h"
#include "bounds.h"
#include "bvh.h"

#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

#include <cassert>
#include <cstdio>
#include <memory>
#include <span>

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

    static std::shared_ptr<box> make(const vec3<si::metre> &half_extents)
    { return std::make_shared<box>(half_extents); }

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

} // namespace physkit