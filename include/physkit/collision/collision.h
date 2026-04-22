#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#else
#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>
#include <optional>
#endif

#include "../algebra/types.h"
#include "mesh.h"
#include "shape.h"

PHYSKIT_EXPORT
namespace physkit
{

namespace detail
{

struct support_pt
{
    vec3<si::metre> p;  // minkowski point
    vec3<si::metre> pa; // point on A
    vec3<si::metre> pb; // point on b
};

/// @brief - not a true convex check but just checking for valid furthest point
template <typename T>
concept SupportShape = requires(const T &shape, const vec3<one> &dir) {
    { shape.support(dir) } -> std::same_as<vec3<si::metre>>;
};

/// @brief minkowski difference support for convex shapes
template <typename ShapeA, typename ShapeB>
    requires SupportShape<ShapeA> && SupportShape<ShapeB>
[[nodiscard]] inline support_pt minkowski_support(const ShapeA &a, const ShapeB &b,
                                                  const vec3<one> &direction)
{
    auto pa = a.support(direction);
    auto pb = b.support(-direction);
    return support_pt{.p = pa - pb, .pa = pa, .pb = pb};
}
} // namespace detail

class collision_info
{
public:
    vec3<one> normal = vec3<one>::zero();
    vec3<si::metre> world_a = vec3<si::metre>::zero();
    vec3<si::metre> world_b = vec3<si::metre>::zero();
    quantity<si::metre> depth{};
};

std::optional<collision_info> gjk_epa(detail::SupportShape auto const &a,
                                      detail::SupportShape auto const &b);
// std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b);
} // namespace physkit
