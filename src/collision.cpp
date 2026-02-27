#include "physkit/collision.h"
#include "physkit/detail/bounds.h"
#include "physkit/detail/types.h"

#include <array>
#include <cassert>
#include <mp-units/systems/si/unit_symbols.h>
#include <optional>

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

template <std::size_t N> class simplex
{
public:
    void add_point(const vec3<si::metre> &point)
    {
        if (M_count == N) throw std::runtime_error("simplex is full");

        M_points[M_count] = point;
        ++M_count;
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
    std::array<vec3<si::metre>, N> M_points{};
    std::size_t M_count{};
};

template <std::size_t N> inline bool handle_line(simplex<N> &simplex, vec3<one> &direction)
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
        simplex.remove_point(0); // keep A only
        direction = ao.normalized();
    }

    return false;
}

template <std::size_t N>
inline bool handle_triangle(simplex<N> &simplex, vec3<one> &direction)
    requires(N >= 3)
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
        simplex.remove_point(0); // remove C
        auto triple = ab.cross(ao).cross(ab);
        direction = (triple.squared_norm() == 0.0 * pow<6>(si::metre)) ? ao.normalized()
                                                                       : triple.normalized();
        return false;
    }

    const auto ac_perp = abc.cross(ac);
    const auto ac_side = ac_perp.dot(ao);
    if (ac_side > 0.0 * pow<4>(si::metre))
    {
        simplex.remove_point(1); // remove B
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

template <std::size_t N>
inline bool handle_tetrahedron(simplex<N> &simplex, vec3<one> &direction)
    requires(N >= 4)
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
        simplex = physkit::simplex<N>{};
        simplex.add_point(c);
        simplex.add_point(b);
        simplex.add_point(a);
        direction = abc.normalized();
        return handle_triangle(simplex, direction);
    }

    const auto side_acd = acd.dot(ao);
    if (side_acd > 0.0 * pow<3>(si::metre))
    {
        simplex = physkit::simplex<N>{};
        simplex.add_point(d);
        simplex.add_point(c);
        simplex.add_point(a);
        direction = acd.normalized();
        return handle_triangle(simplex, direction);
    }

    const auto side_adb = adb.dot(ao);
    if (side_adb > 0.0 * pow<3>(si::metre))
    {
        simplex = physkit::simplex<N>{};
        simplex.add_point(b);
        simplex.add_point(d);
        simplex.add_point(a);
        direction = adb.normalized();
        return handle_triangle(simplex, direction);
    }

    return true;
}

template <std::size_t N> inline bool handle_simplex(simplex<N> &simplex, vec3<one> &direction)
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
template <detail::SupportShape ShapeA, detail::SupportShape ShapeB>
bool gjk_collision(const ShapeA &a, const ShapeB &b)
{
    simplex<4> simplex;
    vec3<one> direction = {1.0, 0.0, 0.0};

    auto point = detail::minkowski_support(a, b, direction);
    if (point.squared_norm() == 0 * pow<2>(si::metre)) return true;
    simplex.add_point(point);
    direction = -point.normalized();

    constexpr int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = detail::minkowski_support(a, b, direction);
        auto progress = new_point.dot(direction);
        if (progress <= 0 * si::metre) return false;

        simplex.add_point(new_point);
        if (handle_simplex(simplex, direction)) return true;
    }

    return false;
}

/// @brief return information that intersection occured
inline std::optional<collision_info> epa(const detail::SupportShape auto &a,
                                         const detail::SupportShape auto &b)
{
    collision_info info;
    info.normal = vec3<one>::zero();
    info.local_a = vec3<si::metre>::zero();
    info.local_b = vec3<si::metre>::zero();
    info.depth = quantity<si::metre>{};

    return info;
}

/// @brief return data for obb obb collision
std::optional<collision_info> gjk_epa(const obb &a, const obb &b)
{
    if (gjk_collision(a, b)) return epa(a, b);
    return std::nullopt;
}

std::optional<collision_info> gjk_epa(const aabb &a, const aabb &b)
{
    if (gjk_collision(a, b)) return epa(a, b);
    return std::nullopt;
}

std::optional<collision_info> gjk_epa(const obb &obb_obj, const aabb &aabb_obj)
{
    if (gjk_collision(obb_obj, aabb_obj)) return epa(obb_obj, aabb_obj);
    return std::nullopt;
}

std::optional<collision_info> gjk_epa(const aabb &aabb_obj, const obb &obb_obj)
{
    if (gjk_collision(aabb_obj, obb_obj)) return epa(aabb_obj, obb_obj);
    return std::nullopt;
}

std::optional<collision_info> gjk_epa(const mesh::instance &a, const mesh::instance &b)
{
    if (gjk_collision(a, b)) return epa(a, b);
    return std::nullopt;
}

/// EPA to be implemented separately.

std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b)
{ assert(false && "SAT not implemented"); }
} // namespace physkit