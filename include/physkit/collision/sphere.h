#pragma once
#include "bounds.h"
#include "bvh.h"
#include "../algebra/types.h"

#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

#include <cassert>
#include <memory>
#include <span>
#include <cstdio>

namespace physkit
{

class sphere : public std::enable_shared_from_this<sphere>
{
    struct key
    {
    };

public:
    /// @brief A view of a sphere placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.

    sphere (const sphere &) = default;
    sphere &operator=(const sphere &) = default;
    sphere (sphere &&) = default;
    sphere &operator=(sphere &&) = default;
    ~sphere() = default;

    sphere(const quantity<si::metre> radius) :
        M_radius{radius}
    {
        M_aabb = aabb::from_points({
            vec3<si::metre>{M_radius, M_radius, M_radius},
            vec3<si::metre>{-M_radius, -M_radius, -M_radius}});

        M_bsphere =  bounding_sphere({0.0f*si::metre,0.0f*si::metre,0.0f*si::metre}, M_radius);
    }

    static std::shared_ptr<sphere> make(const quantity<si::metre> radius)
    {
        return std::make_shared<sphere>(radius);
        auto a = std::make_shared<sphere>(radius);
    }

    [[nodiscard]] const quantity<si::metre> &radius() const { return M_radius;}
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

} // namespace physkit
