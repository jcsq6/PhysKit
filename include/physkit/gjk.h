#pragma once

#include "detail/types.h"
#include "mesh.h"
#include "obb.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <array>
#include <utility>

namespace physkit
{
/// @brief Result of a GJK intersection/distance test
struct gjk_result
{
    bool intersects = false;
    std::pair<vec3<si::metre>, vec3<si::metre>> closest_points{}; // Closest points on each shape
    quantity<si::metre> distance{};                               // Minimum distance between shapes
};

} // namespace physkit