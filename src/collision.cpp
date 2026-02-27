#include "physkit/collision.h"
#include "physkit/detail/types.h"

#include <cassert>
#include <mp-units/systems/si/unit_symbols.h>
#include <optional>

#include <absl/container/inlined_vector.h>

namespace physkit
{
/// based off winter dev gjk algorithm implementation

/// @brief add in support vertex - TODO -> Add in support points in the future
struct support_pt
{
    vec3<si::metre> p;  // minkowski point
    vec3<si::metre> pa; // point on A
    vec3<si::metre> pb; // point on b
};

using simplex = absl::InlinedVector<vec3<si::metre>, 4>;

inline bool handle_line(simplex &simplex, vec3<one> &direction)
{
    const auto a = simplex[1];
    const auto b = simplex[0];
    const auto ab = b - a;
    const auto ao = -a;
    auto ab_dot_ao = ab.dot(ao);

    if (ab_dot_ao > 0.0 * pow<2>(si::metre))
    {
        auto triple = ab.cross(ao).cross(ab);
        if (triple.squared_norm() == 0.0 * pow<6>(si::metre))
        {
            auto ab_hat = ab.normalized();
            auto perp = ab_hat.cross(vec3<one>{0.0, 1.0, 0.0});
            if (perp.squared_norm() == 0) perp = ab_hat.cross(vec3<one>{0.0, 0.0, 1.0});
            direction = perp.normalized();
        }
        else
            direction = triple.normalized();
    }
    else
    {
        simplex.erase(simplex.begin()); // Keep A only
        direction = ao.normalized();
    }

    return false;
}

inline bool handle_triangle(simplex &simplex, vec3<one> &direction)
{
    const auto a = simplex[2];
    const auto b = simplex[1];
    const auto c = simplex[0];

    const auto ab = b - a;
    const auto ac = c - a;
    const auto ao = -a;
    const auto abc = ab.cross(ac);

    const auto ab_perp = ab.cross(abc);
    const auto ab_side = ab_perp.dot(ao);
    if (ab_side > 0.0 * pow<4>(si::metre))
    {
        simplex.erase(simplex.begin()); // remove C
        auto triple = ab.cross(ao).cross(ab);
        direction = (triple.squared_norm() == 0.0 * pow<6>(si::metre)) ? ao.normalized()
                                                                       : triple.normalized();
        return false;
    }

    const auto ac_perp = abc.cross(ac);
    const auto ac_side = ac_perp.dot(ao);
    if (ac_side > 0.0 * pow<4>(si::metre))
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        simplex.erase(simplex.begin() + 1); // remove B
        auto triple = ac.cross(ao).cross(ac);
        direction = (triple.squared_norm() == 0.0 * pow<6>(si::metre)) ? ao.normalized()
                                                                       : triple.normalized();
        return false;
    }

    const auto abc_side = abc.dot(ao);
    if (abc_side <= 0.0 * pow<3>(si::metre))
    {
        std::swap(simplex[0], simplex[1]);
        direction = (-abc).normalized();
    }
    else
        direction = abc.normalized();

    return false;
}

inline bool handle_tetrahedron(simplex &simplex, vec3<one> &direction)
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
        if (side > 0.0 * pow<3>(si::metre)) normal = -normal;
    };

    orient_face_outward(abc, a, d);
    orient_face_outward(acd, a, b);
    orient_face_outward(adb, a, c);

    const auto side_abc = abc.dot(ao);
    if (side_abc > 0.0 * pow<3>(si::metre))
    {
        simplex = physkit::simplex{};
        simplex.push_back(c);
        simplex.push_back(b);
        simplex.push_back(a);
        direction = abc.normalized();
        return handle_triangle(simplex, direction);
    }

    const auto side_acd = acd.dot(ao);
    if (side_acd > 0.0 * pow<3>(si::metre))
    {
        simplex = physkit::simplex{};
        simplex.push_back(d);
        simplex.push_back(c);
        simplex.push_back(a);
        direction = acd.normalized();
        return handle_triangle(simplex, direction);
    }

    const auto side_adb = adb.dot(ao);
    if (side_adb > 0.0 * pow<3>(si::metre))
    {
        simplex = physkit::simplex{};
        simplex.push_back(b);
        simplex.push_back(d);
        simplex.push_back(a);
        direction = adb.normalized();
        return handle_triangle(simplex, direction);
    }

    return true;
}

inline bool handle_simplex(simplex &simplex, vec3<one> &direction)
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

/// @brief modify collision loop to check on separation
bool gjk_collision(const detail::SupportShape auto &a, const detail::SupportShape auto &b)
{
    simplex simplex;
    vec3<one> direction = {1.0, 0.0, 0.0};

    auto point = detail::minkowski_support(a, b, direction);
    if (point.squared_norm() == 0 * pow<2>(si::metre)) return true;
    simplex.push_back(point);
    direction = -point.normalized();

    constexpr int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = detail::minkowski_support(a, b, direction);
        auto progress = new_point.dot(direction);
        if (progress <= 0 * si::metre) return false;

        simplex.push_back(new_point);
        if (handle_simplex(simplex, direction)) return true;
    }

    return false;
}

/// @brief return information that intersection occured
inline collision_info epa(const detail::SupportShape auto &a, const detail::SupportShape auto &b)
{ return {}; }

/// @brief return data for obb obb collision
std::optional<collision_info> gjk_epa(detail::SupportShape auto const &a,
                                      detail::SupportShape auto const &b)
{
    if (gjk_collision(a, b)) return epa(a, b);
    return std::nullopt;
}

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define INSTANTIATE_GJK_EPA(ShapeA, ShapeB)                                                        \
    template std::optional<collision_info> gjk_epa(const ShapeA &a, const ShapeB &b);

INSTANTIATE_GJK_EPA(obb, obb)
INSTANTIATE_GJK_EPA(aabb, aabb)
INSTANTIATE_GJK_EPA(obb, aabb)
INSTANTIATE_GJK_EPA(aabb, obb)
INSTANTIATE_GJK_EPA(mesh::instance, mesh::instance)
INSTANTIATE_GJK_EPA(aabb, mesh::instance)
INSTANTIATE_GJK_EPA(mesh::instance, aabb)
INSTANTIATE_GJK_EPA(obb, mesh::instance)
INSTANTIATE_GJK_EPA(mesh::instance, obb)

#undef INSTANTIATE_GJK_EPA

std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b)
{ assert(false && "SAT not implemented"); }
} // namespace physkit