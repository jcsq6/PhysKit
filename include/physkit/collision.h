#pragma once

#include "detail/bounds.h"
#include "detail/types.h"
#include "mesh.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>
#include <optional>
#include <utility>

namespace physkit
{
// moved struct of GJK result back into collision.h
struct gjk_result
{
    bool intersects{};
    std::optional<std::pair<vec3<si::metre>, vec3<si::metre>>> closest_points_data{};
    quantity<si::metre> distance_data{};

    [[nodiscard]] auto closest_points() const { return closest_points_data; }
    [[nodiscard]] auto distance() const { return distance_data; }
};

/// @brief - not a true convex check but just checking for valid furthest point
template <typename T>
concept SupportShape = requires(const T &shape, const vec3<one> &dir) {
    { shape.support_point(dir) } -> std::same_as<vec3<si::metre>>;
};

/// @brief - fuckass support function definition
template <typename ShapeA, typename ShapeB>
    requires SupportShape<ShapeA> && SupportShape<ShapeB>
[[nodiscard]] vec3<si::metre> minkowski_support(const ShapeA &a, const ShapeB &b,
                                                const vec3<one> &direction);

class collision_info // TODO: time of impact for continuous collision detection
{
public:
    vec3<one> normal = vec3<one>::zero();
    vec3<si::metre> local_a = vec3<si::metre>::zero();
    vec3<si::metre> local_b = vec3<si::metre>::zero();
    quantity<si::metre> depth{};
};

// moved gjk tests back into collision.h
[[nodiscard]] gjk_result gjk_obb_obb(const obb &a, const obb &b);
[[nodiscard]] gjk_result gjk_aabb_aabb(const aabb &a, const aabb &b);
[[nodiscard]] gjk_result gjk_obb_aabb(const obb &obb_obj, const aabb &aabb_obj);
[[nodiscard]] gjk_result gjk_aabb_obb(const aabb &aabb_obj, const obb &obb_obj);

std::optional<collision_info> gjk_epa(const mesh::instance &a, const mesh::instance &b);
std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b);
} // namespace physkit
