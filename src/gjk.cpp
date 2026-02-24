#include "detail/bounds.h"
#include "detail/types.h"
#include "physkit/mesh.h"

#include <array>
#include <mp-units/systems/si/unit_symbols.h>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;

#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>
#include <optional>
namespace physkit
{
///@brief support function for GJK to take two collides and a direction - returns minkowski
/// difference
template <typename ShapeA, typename ShapeB>
vec3<si::metre> minkowski_difference_support(const ShapeA &shape_a, const ShapeB &shape_b,
                                             const vec3<one> &direction)
{
    // support A - B in direction d
    // Note: This assumes support_function is defined elsewhere for each shape type
    // For now, we'll leave this as a placeholder that needs implementation
    return vec3<si::metre>{0 * si::metre, 0 * si::metre, 0 * si::metre}; // Placeholder
}

// include support
template <typename T>
concept ConvexShape = requires(const T &shape, const vec3<one> &dir) {
    { shape.support(dir) } -> std::same_as<vec3<si::metre>>;
};

// add in forward declarations for helper functions
bool handle_line(Simplex &s, vec3<si::metre> &dir);
bool handle_triangle(Simplex &s, vec3<si::metre> &dir);
bool handle_tetrahedron(Simplex &s, vec3<si::metre> &dir, quantity<si::metre> tolerance);

///@brief construct simplex class
class Simplex
{
public:
    Simplex() = default;
    void add_point(const vec3<si::metre> &point)
    {
        if (M_count < 4)
        {
            M_points[M_count] = point;
            M_count++;
        }
    }

    [[nodiscard]] size_t size() const { return M_count; }

    vec3<si::metre> &operator[](size_t i) { return M_points[i]; }
    const vec3<si::metre> &operator[](size_t i) const
    {
        return M_points[i];
    } // Fixed: was returning pointer, should return reference

    // remove points at index i and shift all points after
    void remove_point(std::size_t i)
    {
        for (std::size_t j = i; j < M_count - 1; ++j)
        { // Fixed: extra space in expression
            M_points[j] = M_points[j + 1];
        }
        M_count--;
    }

    // find the last point in the simplex
    [[nodiscard]] const vec3<si::metre> &last() const
    {
        return M_points[M_count - 1];
    } // Fixed: return const reference

    void clear() { M_count = 0; }

    // add in begin and end for simplex structure
    [[nodiscard]] auto begin() const { return M_points.begin(); }
    [[nodiscard]] auto end() const { return M_points.end() - (4 - M_count); }

private:
    std::array<vec3<si::metre>, 4> M_points;
    [[nodiscard]] size_t M_count{};

    // Create support functions before creating the iteration loop
};

// from class simplex, we can make shapes - line, triangle, tetrahedron, etc...
/// @brief compute support point for minkowski difference A - B in given direction
template <typename ShapeA, typename ShapeB>
[[nodiscard]] vec3<si::metre> minkowksi_difference_support(const ShapeA &shape_a,
                                                           const ShapeB &shape_b,
                                                           const vec3<one> &direction)
{
    return shape_a.support(direction) - shape_b.support(-direction);
}

// explicit instatiation for mesh::instance - WIP
template <typename>
[[nodiscard]] vec3<si::metre>
minkowski_difference_support(const mesh::instance &, const mesh::instance &, const vec3<one> &);

/// @brief support function for a single convex shape
vec3<si::metre> support(const ConvexShape &shape, const vec3<si::metre> &direction);

vec3<si::metre> minkowski_support(const ConvexShape &A, const ConvexShape &B,
                                  const vec3<si::metre> &direction)
{
    return support(A, direction) - support(B, -direction);
}

bool gjk_collision(const ConvexShape &A, ConvexShape &B,
                   const quantity<si::metre> tolerance = quantity<si::metre>{1e-6})
{
    Simplex simplex;
    vec3<si::metre> direction = {1 * si::metre, 0 * si::metre,
                                 0 * si::metre}; // initial search direction in x axis

    // set the support point
    auto point = minkowski_support(A, B, direction);
    direction = -point; // point towards origin

    constexpr int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = minkowski_support(A, B, direction);

        // termination: if new point doesn't pass the origin in search direction
        // dot returns quantity<metre> -> compare against same type

        if (dot(new_point, direction) < quantity<si::metre>{0}) { return false; }

        simplex.add_point(new_point);
        if (next_simplex(simplex, direction, tolerance)) { return true; }
        // continue into updated direction
    }
    return false; // max iterations reached
}

bool next_simplex(Simplex &simplex, vec3<si::metre> &direction, const quantity<si::metre> tolerance)
{
    switch (simplex.size())
    {
    case 2:
        return handle_line(simplex, direction);
    case 3:
        return handle_triangle(simplex, direction);
    case 4:
        return handle_tetrahedron(simplex, direction, tolerance);
    default:
        return false;
    }
}
bool handle_line(Simplex &s, vec3<si::metre> &dir)
{
    const auto A = s[1];
    const auto B = s[0];

    auto AB = B - A;
    auto A0 = -A; // vector from A to origin

    if (dot(AB, A0) > quantity<si::metre * si::metre>{0})
    {
        dir = A0;
        s.remove_point(0);
    }
    else
    {
        dir = cross(cross(AB, A0), AB);
    }
    return false;
}

bool handle_triangle(Simplex &s, vec3<si::metre> &dir)
{
    auto A = s[2];
    auto B = s[1];
    auto C = s[0];
    auto AB = B - A;
    auto AC = C - A;
    auto A0 = -A;

    auto ABC = cross(AB, AC);

    // region test using voroni features
    // logic:

    /*

    if origin outside of edge Ac, keep A,C; dir = perpendicular to AC; return false;
    else if: origin outside edge AB, keep A, B; dir = perp to AB; return false;
    else if origin on same side as norm. {dir = ABC; return false;}
    else {dir = -ABC; s = {A,B,C}; reutrn false} will flip winding
    */
    dir = cross(cross(AB, A0), AB);
    return false;
}

bool handle_tetrahedron(Simplex &s, vec3<si::metre> &dir, quantity<si::metre> tolerance)
{
    // check if origin is inside tetrahedron using barycentric or plane tests
    // if inside: return true is collision
    // if outside: reduce to closest face and update direction

    void(toleranec);
    return false;
}

} // namespace physkit
