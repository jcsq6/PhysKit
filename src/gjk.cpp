#include "physkit/collision.h"
#include "physkit/detail/bounds.h"
#include "physkit/detail/types.h"

#include <array>
#include <concepts>
#include <mp-units/systems/si/unit_symbols.h>
#include <optional>
#include <utility>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

namespace physkit
{
/// based off winter dev gjk algorithm implementation

///@brief define a compile type shape for the GJK algorithm
// to be a valid convex shape, type T must have furthest point method

// takes dir vector and returns a point in metres.
template <typename T>
concept ConvexShape = requires(const T &shape, const vec3<one> &dir) {
    { shape.furthest_point(dir) } -> std::same_as<vec3<si::metre>>;
};

// move gjk struct for results into this code
struct gjk_result
{
    bool intersects;
    std::optional<std::pair<vec3<si::metre>, vec3<si::metre>>> closest_points_data;
    quantity<si::metre> distance_data;
};

template <typename ShapeA, typename ShapeB>
[[nodiscard]] gjk_result gjk_test(const ShapeA &shape_a, const ShapeB &shape_b)
{
    if constexpr (std::same_as<ShapeA, obb> && std::same_as<ShapeB, obb>)
    {
        return gjk_obb_obb(shape_a, shape_b);
    }
    else if constexpr (std::same_as<ShapeA, obb> && std::same_as<ShapeB, aabb>)
    {
        return gjk_obb_aabb(shape_a, shape_b);
    }
    else if constexpr (std::same_as<ShapeA, aabb> && std::same_as<ShapeB, obb>)
    {
        return gjk_aabb_obb(shape_a, shape_b);
    }

    else if constexpr (std::same_as<ShapeA, aabb> && std::same_as<ShapeB, aabb>)
    {
        return gjk_aabb_aabb(shape_a, shape_b);
    }
    else
    {
        static_assert(std::same_as<ShapeA, void>, "GJK ONLY APPLIES TO OBB AND AABB OBJECTS ONLY");
    }
}

class Simplex
{
public:
    void add_point(const vec3<si::metre> &point)
    {
        if (M_count < 4)
        {
            M_points[M_count] = point;
            ++M_count;
        }
    }

    [[nodiscard]] std::size_t size() const { return M_count; }

    vec3<si::metre> &operator[](std::size_t i) { return M_points[i]; }
    [[nodiscard]] const vec3<si::metre> &operator[](std::size_t i) const { return M_points[i]; }

    void remove_point(std::size_t i)
    {
        if (i >= M_count) return;
        for (std::size_t j = i; j + 1 < M_count; ++j) { M_points[j] = M_points[j + 1]; }
        --M_count;
    }

private:
    std::array<vec3<si::metre>, 4> M_points{};
    std::size_t M_count{};
};

bool handle_simplex(Simplex &simplex, vec3<one> &direction);
bool handle_line(Simplex &simplex, vec3<one> &direction);
bool handle_triangle(Simplex &simplex, vec3<one> &direction);
bool handle_tetrahedron(Simplex &simplex, vec3<one> &direction);

template <typename ShapeA, typename ShapeB>
    requires ConvexShape<ShapeA> && ConvexShape<ShapeB>
[[nodiscard]] vec3<si::metre> minkowski_support(const ShapeA &a, const ShapeB &b,
                                                const vec3<one> &direction)
{
    return a.furthest_point(direction) - b.furthest_point(-direction);
}

template <typename ShapeA, typename ShapeB>
    requires ConvexShape<ShapeA> && ConvexShape<ShapeB>

bool gjk_collision(const ShapeA &a, const ShapeB &b,
                   quantity<si::metre> tolerance = 1e-6 * si::metre)
{
    Simplex simplex;
    vec3<one> direction = {1.0, 0.0, 0.0};

    auto point = minkowski_support(a, b, direction);
    if (point.squared_norm() == decltype(point.squared_norm()){}) return true;
    simplex.add_point(point);
    direction = -point.normalized();

    constexpr int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = minkowski_support(a, b, direction);
        auto progress = new_point.dot(direction);
        if (progress < tolerance) return false;

        simplex.add_point(new_point);
        if (handle_simplex(simplex, direction)) return true;
    }

    return false;
}

bool handle_simplex(Simplex &simplex, vec3<one> &direction)
{
    switch (simplex.size())
    {
    case 2:
        return handle_line(simplex, direction);
    case 3:
        return handle_triangle(simplex, direction);
    case 4:
        return handle_tetrahedron(simplex, direction);
    default:
        return false;
    }
}

bool handle_line(Simplex &simplex, vec3<one> &direction)
{
    const auto a = simplex[1];
    const auto b = simplex[0];
    const auto ab = b - a;
    const auto ao = -a;
    auto ab_dot_ao = ab.dot(ao);

    if (ab_dot_ao > decltype(ab_dot_ao){})
    {
        auto triple = ab.cross(ao).cross(ab);
        if (triple.squared_norm() == decltype(triple.squared_norm()){})
            direction = ao.normalized();
        else
            direction = triple.normalized();
    }
    else
    {
        simplex.remove_point(0); // keep A only
        direction = ao.normalized();
    }

    return false;
}

bool handle_triangle(Simplex &simplex, vec3<one> &direction)
{
    const auto a = simplex[2];
    const auto b = simplex[1];
    const auto c = simplex[0];

    const auto ab = b - a;
    const auto ac = c - a;
    const auto ao = -a;
    const auto abc = ab.cross(ac);

    const auto ab_perp = abc.cross(ab);
    const auto ab_side = ab_perp.dot(ao);
    if (ab_side > decltype(ab_side){})
    {
        simplex.remove_point(0); // remove C
        auto triple = ab.cross(ao).cross(ab);
        direction = (triple.squared_norm() == decltype(triple.squared_norm()){})
                        ? ao.normalized()
                        : triple.normalized();
        return false;
    }

    const auto ac_perp = ac.cross(abc);
    const auto ac_side = ac_perp.dot(ao);
    if (ac_side > decltype(ac_side){})
    {
        simplex.remove_point(1); // remove B
        auto triple = ac.cross(ao).cross(ac);
        direction = (triple.squared_norm() == decltype(triple.squared_norm()){})
                        ? ao.normalized()
                        : triple.normalized();
        return false;
    }

    const auto abc_side = abc.dot(ao);
    if (abc_side > decltype(abc_side){}) { direction = abc.normalized(); }
    else
    {
        std::swap(simplex[0], simplex[1]);
        direction = (-abc).normalized();
    }

    return false;
}

bool handle_tetrahedron(Simplex &simplex, vec3<one> &direction)
{
    const auto a = simplex[3];
    const auto b = simplex[2];
    const auto c = simplex[1];
    const auto d = simplex[0];
    const auto ao = -a;

    auto abc = (b - a).cross(c - a);
    auto acd = (c - a).cross(d - a);
    auto adb = (d - a).cross(b - a);

    auto orient_face_outward = [](auto &normal, const auto &face_a, const auto &opposite)
    {
        auto toward_opposite = opposite - face_a;
        auto side = normal.dot(toward_opposite);
        if (side > decltype(side){}) normal = -normal;
    };

    orient_face_outward(abc, a, d);
    orient_face_outward(acd, a, b);
    orient_face_outward(adb, a, c);

    const auto side_abc = abc.dot(ao);
    if (side_abc > decltype(side_abc){})
    {
        simplex = Simplex{};
        simplex.add_point(c);
        simplex.add_point(b);
        simplex.add_point(a);
        direction = abc.normalized();
        return handle_triangle(simplex, direction);
    }

    const auto side_acd = acd.dot(ao);
    if (side_acd > decltype(side_acd){})
    {
        simplex = Simplex{};
        simplex.add_point(d);
        simplex.add_point(c);
        simplex.add_point(a);
        direction = acd.normalized();
        return handle_triangle(simplex, direction);
    }

    const auto side_adb = adb.dot(ao);
    if (side_adb > decltype(side_adb){})
    {
        simplex = Simplex{};
        simplex.add_point(b);
        simplex.add_point(d);
        simplex.add_point(a);
        direction = adb.normalized();
        return handle_triangle(simplex, direction);
    }

    return true;
}

/// @brief return information that intersection occured
[[nodiscard]] gjk_result make_intersection_result()
{
    gjk_result result;
    result.intersects = true;
    result.closest_points_data = std::nullopt; // better way of showing disengaged.
    result.distance_data = quantity<si::metre>{};
    return result;
}

/// @brief return information that intersection did not occur
[[nodiscard]] gjk_result make_separated_result(const vec3<si::metre> &point_a,
                                               const vec3<si::metre> &point_b)
{
    gjk_result result;
    result.intersects = false;
    result.closest_points_data = std::pair{point_a, point_b}; // better way of showing disengaged.
    result.distance_data = (point_b - point_a).norm();
    return result;
}

vec3<si::metre> support_obb(const obb &obj, const vec3<one> &direction)
{
    return obj.furthest_point(direction);
}

vec3<si::metre> support_aabb(const aabb &obj, const vec3<one> &direction)
{
    return obj.furthest_point(direction);
}

/// @brief moved minkowski difference support into this file to keep self-contained
template <typename ShapeA, typename ShapeB>
vec3<si::metre> minkowski_difference_support(const ShapeA &shape_a, const ShapeB &shape_b,
                                             const vec3<one> &direction)
{
    return shape_a.furthest_point(direction) - shape_b.furthest_point(-direction);
}

/// @brief return data for obb obb collision
gjk_result gjk_obb_obb(const obb &a, const obb &b)
{
    if (gjk_collision(a, b)) return make_intersection_result();
    return make_separated_result(a.furthest_point({1, 0, 0}), b.furthest_point({-1, 0, 0}));
}

gjk_result gjk_aabb_aabb(const aabb &a, const aabb &b)
{
    if (gjk_collision(a, b)) return make_intersection_result();
    return make_separated_result(a.furthest_point({1, 0, 0}), b.furthest_point({-1, 0, 0}));
}

gjk_result gjk_obb_aabb(const obb &obb_obj, const aabb &aabb_obj)
{
    if (gjk_collision(obb_obj, aabb_obj)) return make_intersection_result();
    return make_separated_result(obb_obj.furthest_point({1, 0, 0}),
                                 aabb_obj.furthest_point({-1, 0, 0}));
}

gjk_result gjk_aabb_obb(const aabb &aabb_obj, const obb &obb_obj)
{
    if (gjk_collision(aabb_obj, obb_obj)) return make_intersection_result();
    return make_separated_result(aabb_obj.furthest_point({1, 0, 0}),
                                 obb_obj.furthest_point({-1, 0, 0}));
}

} // namespace physkit
