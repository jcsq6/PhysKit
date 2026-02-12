#include "physkit/sat.h"
#include "physkit/math_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace physkit
{
// Specialization for OBB projection
template <>
std::pair<quantity<si::metre>, quantity<si::metre>>
project_shape_onto_axis<obb>(const obb &box, const vec3<one> &axis)
{
    // Project the center of the OBB onto the axis
    auto center_projection =
        dot_product(box.center_pos(), vec3<si::metre>{axis.x(), axis.y(), axis.z()});

    // Calculate the projection radius based on the half-extents and orientation
    auto axes = box.axes();
    auto radius = std::abs(dot_product(vec3<one>{axes[0].x(), axes[0].y(), axes[0].z()}, axis)) *
                      box.half_extents.x() +
                  std::abs(dot_product(vec3<one>{axes[1].x(), axes[1].y(), axes[1].z()}, axis)) *
                      box.half_extents.y() +
                  std::abs(dot_product(vec3<one>{axes[2].x(), axes[2].y(), axes[2].z()}, axis)) *
                      box.half_extents.z();

    return std::make_pair(center_projection - radius, center_projection + radius);
}

// Specialization for AABB projection
template <>
std::pair<quantity<si::metre>, quantity<si::metre>>
project_shape_onto_axis<aabb>(const aabb &box, const vec3<one> &axis)
{
    // Project the center of the AABB onto the axis
    auto center = box.center();
    auto center_projection = dot_product(center, vec3<si::metre>{axis.x(), axis.y(), axis.z()});

    // Calculate the projection radius based on the extent
    auto extent = box.extent();
    auto radius = std::abs(axis.x()) * extent.x() + std::abs(axis.y()) * extent.y() +
                  std::abs(axis.z()) * extent.z();

    return std::make_pair(center_projection - radius, center_projection + radius);
}

sat_intersection_result sat_test_obb_obb(const obb &a, const obb &b)
{
    sat_intersection_result result;
    result.intersects = true;
    result.penetration_depth = 0.0 * si::metre;

    // Get the axes of both OBBs
    auto axes_a = a.axes();
    auto axes_b = b.axes();

    // Potential separating axes for OBB-OBB:
    // 1. Axes of OBB A (3 axes)
    // 2. Axes of OBB B (3 axes)
    // 3. Cross products of axes from A and B (9 axes, but only 9 unique)

    std::vector<vec3<one>> potential_axes;

    // Add axes of OBB A
    potential_axes.push_back(axes_a[0]);
    potential_axes.push_back(axes_a[1]);
    potential_axes.push_back(axes_a[2]);

    // Add axes of OBB B
    potential_axes.push_back(axes_b[0]);
    potential_axes.push_back(axes_b[1]);
    potential_axes.push_back(axes_b[2]);

    // Add cross products of axes
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            // Cross product of axes_a[i] and axes_b[j]
            vec3<one> cross_axis =
                cross_product(vec3<one>{axes_a[i].x(), axes_a[i].y(), axes_a[i].z()},
                              vec3<one>{axes_b[j].x(), axes_b[j].y(), axes_b[j].z()});

            // Normalize the cross product
            auto length =
                std::sqrt(cross_axis.x() * cross_axis.x() + cross_axis.y() * cross_axis.y() +
                          cross_axis.z() * cross_axis.z());
            if (length > 1e-9)
            {
                cross_axis = vec3<one>{cross_axis.x() / length, cross_axis.y() / length,
                                       cross_axis.z() / length};
                potential_axes.push_back(cross_axis);
            }
        }
    }

    quantity<si::metre> min_penetration = std::numeric_limits<double>::max() * si::metre;
    vec3<one> best_separating_axis{0, 0, 0};
    bool found_separating_axis = false;

    for (const auto &axis : potential_axes)
    {
        auto proj_a = project_shape_onto_axis(a, axis);
        auto proj_b = project_shape_onto_axis(b, axis);

        // Check for overlap
        if (proj_a.second < proj_b.first || proj_b.second < proj_a.first)
        {
            // Found a separating axis
            result.intersects = false;
            result.separating_axis = axis;
            return result;
        }

        // Calculate penetration depth
        auto penetration = std::min(proj_a.second - proj_b.first, proj_b.second - proj_a.first);
        if (penetration < min_penetration)
        {
            min_penetration = penetration;
            best_separating_axis = axis;
            found_separating_axis = true;
        }
    }

    if (found_separating_axis)
    {
        result.penetration_depth = min_penetration;
        result.separating_axis = best_separating_axis;
        result.intersects = true;
    }

    return result;
}

sat_intersection_result sat_test_obb_aabb(const obb &obb_obj, const aabb &aabb_obj)
{
    // Convert AABB to OBB with identity orientation for easier computation
    obb aabb_as_obb{aabb_obj.center(),
                    quat<one>::identity(), // Identity orientation for AABB
                    aabb_obj.extent()};

    return sat_test_obb_obb(obb_obj, aabb_as_obb);
}

sat_intersection_result sat_test_aabb_obb(const aabb &aabb_obj, const obb &obb_obj)
{
    // This is symmetric to OBB-AABB
    return sat_test_obb_aabb(obb_obj, aabb_obj);
}

sat_intersection_result sat_test_aabb_aabb(const aabb &a, const aabb &b)
{
    sat_intersection_result result;
    result.intersects = true;

    // For AABB-AABB, we only need to check the 3 cardinal axes
    std::array<vec3<one>, 3> axes = {
        vec3<one>{1.0, 0.0, 0.0}, // X-axis
        vec3<one>{0.0, 1.0, 0.0}, // Y-axis
        vec3<one>{0.0, 0.0, 1.0}  // Z-axis
    };

    quantity<si::metre> min_penetration = std::numeric_limits<double>::max() * si::metre;
    vec3<one> best_separating_axis{0, 0, 0};
    bool found_separating_axis = false;

    for (const auto &axis : axes)
    {
        auto proj_a = project_shape_onto_axis(a, axis);
        auto proj_b = project_shape_onto_axis(b, axis);

        // Check for overlap
        if (proj_a.second < proj_b.first || proj_b.second < proj_a.first)
        {
            // Found a separating axis
            result.intersects = false;
            result.separating_axis = axis;
            return result;
        }

        // Calculate penetration depth
        auto penetration = std::min(proj_a.second - proj_b.first, proj_b.second - proj_a.first);
        if (penetration < min_penetration)
        {
            min_penetration = penetration;
            best_separating_axis = axis;
            found_separating_axis = true;
        }
    }

    if (found_separating_axis)
    {
        result.penetration_depth = min_penetration;
        result.separating_axis = best_separating_axis;
    }

    return result;
}

// Template specializations for the generic SAT test
template <> sat_intersection_result sat_test<obb, obb>(const obb &shape_a, const obb &shape_b)
{
    return sat_test_obb_obb(shape_a, shape_b);
}

template <> sat_intersection_result sat_test<obb, aabb>(const obb &shape_a, const aabb &shape_b)
{
    return sat_test_obb_aabb(shape_a, shape_b);
}

template <> sat_intersection_result sat_test<aabb, obb>(const aabb &shape_a, const obb &shape_b)
{
    return sat_test_aabb_obb(shape_a, shape_b);
}

template <> sat_intersection_result sat_test<aabb, aabb>(const aabb &shape_a, const aabb &shape_b)
{
    return sat_test_aabb_aabb(shape_a, shape_b);
}

template <typename ShapeA, typename ShapeB>
std::vector<vec3<si::metre>> compute_contact_points(const ShapeA &shape_a, const ShapeB &shape_b,
                                                    const vec3<one> &separating_axis)
{
    // This is a simplified implementation - a full implementation would use
    // clipping algorithms like Sutherland-Hodgman to find contact points
    std::vector<vec3<si::metre>> contacts;

    // For now, return an empty vector - a more sophisticated implementation
    // would compute actual contact points based on the intersection
    return contacts;
}
} // namespace physkit