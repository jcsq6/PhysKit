#include "physkit/gjk.h"
#include "physkit/math_utils.h"

#include <mp-units/systems/si/unit_symbols.h>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;

#include <algorithm>
#include <cmath>
#include <limits>

namespace physkit
{

// Support function for OBB - finds furthest point in given direction
vec3<si::metre> support_obb(const obb &obj, const vec3<one> &direction)
{
    // Get the three axis vectors of the OBB in world space
    auto axes = obj.axes();

    // Find the support point by taking the center and adding the signed half-extents
    // in the direction of the support
    vec3<si::metre> result = obj.center_pos(); // Using the renamed method

    // Project the direction onto each local axis and determine which direction to go
    auto dir_proj_x = dot_product(direction, axes[0]);
    auto dir_proj_y = dot_product(direction, axes[1]);
    auto dir_proj_z = dot_product(direction, axes[2]);

    // Add or subtract half-extents based on the direction
    if (dir_proj_x >= 0) { result = result + axes[0] * obj.half_extents.x(); }
    else
    {
        result = result - axes[0] * obj.half_extents.x();
    }

    if (dir_proj_y >= 0) { result = result + axes[1] * obj.half_extents.y(); }
    else
    {
        result = result - axes[1] * obj.half_extents.y();
    }

    if (dir_proj_z >= 0) { result = result + axes[2] * obj.half_extents.z(); }
    else
    {
        result = result - axes[2] * obj.half_extents.z();
    }

    return result;
}

// Support function for AABB - finds furthest point in given direction
vec3<si::metre> support_aabb(const aabb &obj, const vec3<one> &direction)
{
    // For AABB, the support point is simply the corner in the direction of the support vector
    vec3<si::metre> result;

    if (direction.x() >= 0) { result.x(obj.max.x()); }
    else
    {
        result.x(obj.min.x());
    }

    if (direction.y() >= 0) { result.y(obj.max.y()); }
    else
    {
        result.y(obj.min.y());
    }

    if (direction.z() >= 0) { result.z(obj.max.z()); }
    else
    {
        result.z(obj.min.z());
    }

    return result;
}

// Compute the support point of the Minkowski difference A - B
template <typename ShapeA, typename ShapeB>
vec3<si::metre> minkowski_difference_support(const ShapeA &shape_a, const ShapeB &shape_b,
                                             const vec3<one> &direction)
{
    // Support of A - B in direction d is: support(A, d) - support(B, -d)
    auto support_a = support_obb(shape_a, direction);
    auto neg_direction = vec3<one>{-direction.x(), -direction.y(), -direction.z()};
    auto support_b =
        support_obb(shape_b, neg_direction); // This will be resolved by template specialization

    return support_a - support_b;
}

// Explicit template specializations for different shape combinations
template <>
vec3<si::metre> minkowski_difference_support<obb, obb>(const obb &shape_a, const obb &shape_b,
                                                       const vec3<one> &direction)
{
    auto support_a = support_obb(shape_a, direction);
    auto neg_direction = vec3<one>{-direction.x(), -direction.y(), -direction.z()};
    auto support_b = support_obb(shape_b, neg_direction);

    return support_a - support_b;
}

template <>
vec3<si::metre> minkowski_difference_support<obb, aabb>(const obb &shape_a, const aabb &shape_b,
                                                        const vec3<one> &direction)
{
    auto support_a = support_obb(shape_a, direction);
    auto neg_direction = vec3<one>{-direction.x(), -direction.y(), -direction.z()};
    auto support_b = support_aabb(shape_b, neg_direction);

    return support_a - support_b;
}

template <>
vec3<si::metre> minkowski_difference_support<aabb, obb>(const aabb &shape_a, const obb &shape_b,
                                                        const vec3<one> &direction)
{
    auto support_a = support_aabb(shape_a, direction);
    auto neg_direction = vec3<one>{-direction.x(), -direction.y(), -direction.z()};
    auto support_b = support_obb(shape_b, neg_direction);

    return support_a - support_b;
}

template <>
vec3<si::metre> minkowski_difference_support<aabb, aabb>(const aabb &shape_a, const aabb &shape_b,
                                                         const vec3<one> &direction)
{
    auto support_a = support_aabb(shape_a, direction);
    auto neg_direction = vec3<one>{-direction.x(), -direction.y(), -direction.z()};
    auto support_b = support_aabb(shape_b, neg_direction);

    return support_a - support_b;
}

// Simplex class to manage the simplex in GJK algorithm
class simplex
{
private:
    std::array<vec3<si::metre>, 4> points_;
    size_t count_;

public:
    simplex() : count_(0) {}

    void add_point(const vec3<si::metre> &point)
    {
        if (count_ < 4)
        {
            points_[count_] = point;
            count_++;
        }
    }

    size_t size() const { return count_; }

    const vec3<si::metre> &operator[](size_t i) const { return points_[i]; }
    vec3<si::metre> &operator[](size_t i) { return points_[i]; }

    // Remove point at index i, shifting remaining points
    void remove_point(size_t i)
    {
        for (size_t j = i; j < count_ - 1; ++j) { points_[j] = points_[j + 1]; }
        count_--;
    }

    // Get the last point
    const vec3<si::metre> &last() const { return points_[count_ - 1]; }
};

// Closest point on line segment AB to origin
vec3<si::metre> closest_point_on_segment(const vec3<si::metre> &a, const vec3<si::metre> &b)
{
    auto ab = b - a;
    auto ao = vec3<si::metre>::zero() - a;

    auto ab_dot_ab = dot_product(ab, ab);
    if (ab_dot_ab.numerical_value_in(si::metre * si::metre) < 1e-9)
    {
        // Degenerate case: A and B are the same point
        return a;
    }

    auto ab_dot_ao = dot_product(ab, ao);

    if (ab_dot_ao.numerical_value_in(si::metre * si::metre) <= 0)
    {
        // Closest point is A
        return a;
    }

    if (ab_dot_ab.numerical_value_in(si::metre * si::metre) <=
        ab_dot_ao.numerical_value_in(si::metre * si::metre))
    {
        // Closest point is B
        return b;
    }

    // Closest point is inside the segment
    auto t = ab_dot_ao / ab_dot_ab;
    return a + ab * t.numerical_value_in(one);
}

// Closest point on triangle ABC to origin
vec3<si::metre> closest_point_on_triangle(const vec3<si::metre> &a, const vec3<si::metre> &b,
                                          const vec3<si::metre> &c)
{
    // Calculate edges
    auto ab = b - a;
    auto ac = c - a;
    auto ao = vec3<si::metre>::zero() - a;

    // Calculate dot products needed
    auto ab_dot_ab = dot_product(ab, ab);
    auto ac_dot_ac = dot_product(ac, ac);
    auto ab_dot_ac = dot_product(ab, ac);
    auto ab_dot_ao = dot_product(ab, ao);
    auto ac_dot_ao = dot_product(ac, ao);

    // Check region R4 (inside triangle)
    auto d1 = ab_dot_ac * ac_dot_ao - ac_dot_ac * ab_dot_ao;
    if (d1 <= 0 && ab_dot_ao <= 0)
    {
        // Closest point is on edge AB
        return closest_point_on_segment(a, b);
    }

    auto d2 = ab_dot_ac * ab_dot_ao - ab_dot_ab * ac_dot_ao;
    if (d2 <= 0 && ac_dot_ao <= 0)
    {
        // Closest point is on edge AC
        return closest_point_on_segment(a, c);
    }

    if (d1 + d2 <= ab_dot_ab * ac_dot_ac - ab_dot_ac * ab_dot_ac)
    {
        // Inside triangle
        auto w = d1 / (d1 + d2);
        return a + ab * w.numerical_value_in(one) + ac * (1.0 - w).numerical_value_in(one);
    }

    // Closest point is on edge BC
    return closest_point_on_segment(b, c);
}

// Determine the new search direction and update the simplex
bool update_simplex(simplex &s, vec3<one> &direction)
{
    switch (s.size())
    {
    case 2:
        return update_line_simplex(s, direction);
    case 3:
        return update_triangle_simplex(s, direction);
    case 4:
        return update_tetrahedron_simplex(s, direction);
    default:
        return false;
    }
}

// Update simplex when it has 2 points (line)
bool update_line_simplex(simplex &s, vec3<one> &direction)
{
    auto a = s[1]; // newest point
    auto b = s[0]; // previous point

    auto ab = b - a;
    auto ao = vec3<si::metre>::zero() - a;

    if (dot_product(ab, ao) >= 0)
    {
        // Origin is in the direction of AB, keep both points
        direction = normalize(vec3<one>{ab.y() * ao.z() - ab.z() * ao.y(),
                                        ab.z() * ao.x() - ab.x() * ao.z(),
                                        ab.x() * ao.y() - ab.y() * ao.x()});
        return false; // Still need more points
    }
    else
    {
        // Origin is not in direction of AB, replace B with A
        s[0] = a;
        s.remove_point(1);
        direction = normalize(vec3<one>{-ao.x().numerical_value_in(si::metre),
                                        -ao.y().numerical_value_in(si::metre),
                                        -ao.z().numerical_value_in(si::metre)});
        return false;
    }
}

// Update simplex when it has 3 points (triangle)
bool update_triangle_simplex(simplex &s, vec3<one> &direction)
{
    auto c = s[2]; // newest point
    auto b = s[1];
    auto a = s[0];

    auto ab = b - a;
    auto ac = c - a;
    auto ao = vec3<si::metre>::zero() - a;

    auto abc = vec3<si::metre>{ab.y() * ac.z() - ab.z() * ac.y(), ab.z() * ac.x() - ab.x() * ac.z(),
                               ab.x() * ac.y() - ab.y() * ac.x()};

    // Check if origin is above or below triangle
    if (dot_product(abc, ao) >= 0)
    {
        // Origin is above triangle, ABC is wound counter-clockwise
        if (dot_product(vec3<si::metre>{abc.y() * ab.z() - abc.z() * ab.y(),
                                        abc.z() * ab.x() - abc.x() * ab.z(),
                                        abc.x() * ab.y() - abc.y() * ab.x()},
                        ao) < 0)
        {
            // Origin is in region of edge AB
            s.remove_point(2); // Remove C
            direction = normalize(vec3<one>{ab.y() * ao.z() - ab.z() * ao.y(),
                                            ab.z() * ao.x() - ab.x() * ao.z(),
                                            ab.x() * ao.y() - ab.y() * ao.x()});
            return false;
        }
        if (dot_product(vec3<si::metre>{ac.y() * abc.z() - ac.z() * abc.y(),
                                        ac.z() * abc.x() - ac.x() * abc.z(),
                                        ac.x() * abc.y() - ac.y() * abc.x()},
                        ao) >= 0)
        {
            // Origin is in region of edge AC
            s[1] = c;          // Move C to B's position
            s.remove_point(2); // Remove old C
            direction = normalize(vec3<one>{ac.y() * ao.z() - ac.z() * ao.y(),
                                            ac.z() * ao.x() - ac.x() * ao.z(),
                                            ac.x() * ao.y() - ac.y() * ao.x()});
            return false;
        }
        // Origin is above triangle
        direction = normalize(vec3<one>{abc.x().numerical_value_in(si::metre * si::metre),
                                        abc.y().numerical_value_in(si::metre * si::metre),
                                        abc.z().numerical_value_in(si::metre * si::metre)});
        return false;
    }
    else
    {
        // Origin is below triangle, reverse ABC winding
        if (dot_product(vec3<si::metre>{abc.y() * ac.z() - abc.z() * ac.y(),
                                        abc.z() * ac.x() - abc.x() * ac.z(),
                                        abc.x() * ac.y() - abc.y() * ac.x()},
                        ao) >= 0)
        {
            // Origin is in region of edge AC
            s[1] = c;          // Move C to B's position
            s.remove_point(2); // Remove old C
            direction = normalize(vec3<one>{ac.y() * ao.z() - ac.z() * ao.y(),
                                            ac.z() * ao.x() - ac.x() * ao.z(),
                                            ac.x() * ao.y() - ac.y() * ao.x()});
            return false;
        }
        if (dot_product(vec3<si::metre>{ab.y() * abc.z() - ab.z() * abc.y(),
                                        ab.z() * abc.x() - ab.x() * abc.z(),
                                        ab.x() * abc.y() - ab.y() * abc.x()},
                        ao) < 0)
        {
            // Origin is in region of edge AB
            s.remove_point(2); // Remove C
            direction = normalize(vec3<one>{ab.y() * ao.z() - ab.z() * ao.y(),
                                            ab.z() * ao.x() - ab.x() * ao.z(),
                                            ab.x() * ao.y() - ab.y() * ao.x()});
            return false;
        }
        // Origin is below triangle, swap B and C to maintain consistent winding
        auto temp = s[1];
        s[1] = s[2];
        s[2] = temp;
        direction = normalize(vec3<one>{-abc.x().numerical_value_in(si::metre * si::metre),
                                        -abc.y().numerical_value_in(si::metre * si::metre),
                                        -abc.z().numerical_value_in(si::metre * si::metre)});
        return false;
    }
}

// Update simplex when it has 4 points (tetrahedron)
bool update_tetrahedron_simplex(simplex &s, vec3<one> &direction)
{
    auto d = s[3]; // newest point
    auto c = s[2];
    auto b = s[1];
    auto a = s[0];

    auto ao = vec3<si::metre>::zero() - a;
    auto ab = b - a;
    auto ac = c - a;
    auto ad = d - a;

    // Calculate face normals
    auto abc_normal =
        vec3<si::metre>{ab.y() * ac.z() - ab.z() * ac.y(), ab.z() * ac.x() - ab.x() * ac.z(),
                        ab.x() * ac.y() - ab.y() * ac.x()};
    auto acd_normal =
        vec3<si::metre>{ac.y() * ad.z() - ac.z() * ad.y(), ac.z() * ad.x() - ac.x() * ad.z(),
                        ac.x() * ad.y() - ac.y() * ad.x()};
    auto adb_normal =
        vec3<si::metre>{ad.y() * ab.z() - ad.z() * ab.y(), ad.z() * ab.x() - ad.x() * ab.z(),
                        ad.x() * ab.y() - ad.y() * ab.x()};

    // Check if origin is in front of any face
    if (dot_product(abc_normal, ao) > 0)
    {
        // Origin is in front of ABC face, remove D
        s.remove_point(3);
        direction = normalize(vec3<one>{abc_normal.x().numerical_value_in(si::metre * si::metre),
                                        abc_normal.y().numerical_value_in(si::metre * si::metre),
                                        abc_normal.z().numerical_value_in(si::metre * si::metre)});
        return false;
    }

    if (dot_product(acd_normal, ao) > 0)
    {
        // Origin is in front of ACD face, remove B
        s[1] = s[2]; // Move C to B's position
        s[2] = s[3]; // Move D to C's position
        s.remove_point(3);
        direction = normalize(vec3<one>{acd_normal.x().numerical_value_in(si::metre * si::metre),
                                        acd_normal.y().numerical_value_in(si::metre * si::metre),
                                        acd_normal.z().numerical_value_in(si::metre * si::metre)});
        return false;
    }

    if (dot_product(adb_normal, ao) > 0)
    {
        // Origin is in front of ADB face, remove C
        s[2] = s[3]; // Move D to C's position
        s.remove_point(3);
        direction = normalize(vec3<one>{adb_normal.x().numerical_value_in(si::metre * si::metre),
                                        adb_normal.y().numerical_value_in(si::metre * si::metre),
                                        adb_normal.z().numerical_value_in(si::metre * si::metre)});
        return false;
    }

    // Origin is inside the tetrahedron - intersection detected!
    return true;
}

// GJK algorithm implementation
template <typename ShapeA, typename ShapeB>
gjk_result gjk_algorithm(const ShapeA &shape_a, const ShapeB &shape_b)
{
    gjk_result result;
    result.intersects = false;
    result.closest_points = std::nullopt;
    result.distance = 0.0 * si::metre;

    // Initial direction - towards origin from the first point
    vec3<one> direction = normalize(vec3<one>{1.0, 1.0, 1.0}); // Start with diagonal direction

    // Get first support point in the initial direction
    auto support = minkowski_difference_support(shape_a, shape_b, direction);

    // Initialize the simplex with the first point
    simplex s;
    s.add_point(support);

    // Reverse the direction for the next iteration
    direction = normalize(vec3<one>{-support.x().numerical_value_in(si::metre),
                                    -support.y().numerical_value_in(si::metre),
                                    -support.z().numerical_value_in(si::metre)});

    // Maximum iterations to prevent infinite loop
    const int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // Get the next support point
        support = minkowski_difference_support(shape_a, shape_b, direction);

        // Check if the support point is past the origin in the opposite direction
        auto dot_supp_dir = dot_product(support, direction);
        if (dot_supp_dir.numerical_value_in(si::metre * one) < 0)
        {
            // The shapes are not intersecting
            result.intersects = false;
            result.distance = length(closest_point_on_simplex_to_origin(s));
            return result;
        }

        // Add the new point to the simplex
        s.add_point(support);

        // Check if the origin is inside the simplex
        if (update_simplex(s, direction))
        {
            // The shapes are intersecting
            result.intersects = true;
            result.distance = 0.0 * si::metre;
            return result;
        }
    }

    // If we reach max iterations, assume no intersection
    result.intersects = false;
    result.distance = length(closest_point_on_simplex_to_origin(s));
    return result;
}

// Helper function to get the closest point on the simplex to the origin
vec3<si::metre> closest_point_on_simplex_to_origin(const simplex &s)
{
    switch (s.size())
    {
    case 1:
        return s[0];
    case 2:
        return closest_point_on_segment(s[0], s[1]);
    case 3:
        return closest_point_on_triangle(s[0], s[1], s[2]);
    case 4:
    default:
        // For tetrahedron, we would need to check all faces
        // For now, return the closest point among the vertices
        auto closest = s[0];
        auto min_dist_sq = squared_length(closest);
        for (size_t i = 1; i < s.size(); ++i)
        {
            auto dist_sq = squared_length(s[i]);
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest = s[i];
            }
        }
        return closest;
    }
}

// Specific implementations for different shape combinations
gjk_result gjk_obb_obb(const obb &a, const obb &b) { return gjk_algorithm(a, b); }

gjk_result gjk_obb_aabb(const obb &obb_obj, const aabb &aabb_obj)
{
    return gjk_algorithm(obb_obj, aabb_obj);
}

gjk_result gjk_aabb_obb(const aabb &aabb_obj, const obb &obb_obj)
{
    return gjk_algorithm(aabb_obj, obb_obj);
}

gjk_result gjk_aabb_aabb(const aabb &a, const aabb &b) { return gjk_algorithm(a, b); }

// Explicit template instantiations for the common shape combinations
template gjk_result gjk_algorithm<obb, obb>(const obb &, const obb &);
template gjk_result gjk_algorithm<obb, aabb>(const obb &, const aabb &);
template gjk_result gjk_algorithm<aabb, obb>(const aabb &, const obb &);
template gjk_result gjk_algorithm<aabb, aabb>(const aabb &, const aabb &);

// Generic GJK test function
template <typename ShapeA, typename ShapeB>
[[nodiscard]] gjk_result gjk_test(const ShapeA &shape_a, const ShapeB &shape_b)
{
    return gjk_algorithm(shape_a, shape_b);
}
} // namespace physkit