#pragma once

#include "detail/bounds.h"
#include "detail/types.h"
#include "mesh.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>
#include <optional>

namespace physkit
{

namespace detail
{

/// @brief - not a true convex check but just checking for valid furthest point
template <typename T>
concept SupportShape = requires(const T &shape, const vec3<one> &dir) {
    { shape.support(dir) } -> std::same_as<vec3<si::metre>>;
};

/// @brief minkowski differenec support for convex shapes
template <typename ShapeA, typename ShapeB>
    requires SupportShape<ShapeA> && SupportShape<ShapeB>
[[nodiscard]] inline vec3<si::metre> minkowski_support(const ShapeA &a, const ShapeB &b,
                                                       const vec3<one> &direction)
{ return a.support(direction) - b.support(-direction); }
} // namespace detail

class collision_info // TODO: time of impact for continuous collision detection
{
public:
    vec3<one> normal = vec3<one>::zero();
    vec3<si::metre> local_a = vec3<si::metre>::zero();
    vec3<si::metre> local_b = vec3<si::metre>::zero();
    quantity<si::metre> depth{};
};

// moved gjk tests back into collision.h
[[nodiscard]] std::optional<collision_info> gjk_epa(const obb &a, const obb &b);
[[nodiscard]] std::optional<collision_info> gjk_epa(const aabb &a, const aabb &b);
[[nodiscard]] std::optional<collision_info> gjk_epa(const obb &obb_obj, const aabb &aabb_obj);
[[nodiscard]] std::optional<collision_info> gjk_epa(const aabb &aabb_obj, const obb &obb_obj);

std::optional<collision_info> gjk_epa(const mesh::instance &a, const mesh::instance &b);
std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b);
} // namespace physkit
