// resolve and combine SAT and GJK skeletons together and keep working.

#pragma once

#include "detail/types.h"
#include "mesh.h" // for aabb
#include "obb.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <array>
#include <optional>
#include <utility>

// NOTE: nothing for SAT is explictly implemented, however we intend to combine both headers
// together.

namespace physkit
{
/// @brief Result of a GJK intersection/distance test
/// @brief should be able to work on both AABB and OBB type collision object
struct gjk_result
{
    bool intersects = false;
    std::optional<std::pair<vec3<si::metre>, vec3<si::metre>>>
        closest_points{};           // Closest points on each shape, if not intersecting
    quantity<si::metre> distance{}; // Minimum distance between shapes
};

/// @brief implement algorithm for the following possible collision possibilities.
/// obb - obb | aabb - aabb | aabb - obb |
[[nodiscard]] gjk_result gjk_obb_obb(const obb &a, const obb &b);

[[nodiscard]] gjk_result gjk_obb_aabb(const obb &obb_obj, const aabb &aabb_obj);

[[nodiscard]] gjk_result gjk_aabb_obb(const aabb &aabb_obj, const obb &obb_obj);

/// @brief GJK algorithm implementation for AABB-AABB intersection/distance
[[nodiscard]] gjk_result gjk_aabb_aabb(const aabb &a, const aabb &b);

/// TODO -> implement generic test for any obb abbb combination: future goal idk if we need
/// immediately.
template <typename ShapeA, typename ShapeB>
[[nodiscard]] gjk_result gjk_test(const ShapeA &shape_a, const ShapeB &shape_b);

/// @brief Support function for obb & aabb - finds furthest point in given direction
[[nodiscard]] vec3<si::metre> support_obb(const obb &obj, const vec3<one> &direction);

[[nodiscard]] vec3<si::metre> support_aabb(const aabb &obj, const vec3<one> &direction);

/// @brief Compute Minkowski difference of shapes A and B
template <typename ShapeA, typename ShapeB>
[[nodiscard]] vec3<si::metre> minkowski_difference_support(const ShapeA &shape_a,
                                                           const ShapeB &shape_b,
                                                           const vec3<one> &direction);
} // namespace physkit