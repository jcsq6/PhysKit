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
    constexpr auto eps = 1e-12;
    const auto a = simplex[1];
    const auto b = simplex[0];
    const auto ab = b - a;
    const auto ao = -a;
    auto ab_dot_ao = ab.dot(ao);

    if (ab_dot_ao > 0.0 * pow<2>(si::metre))
    {
        auto triple = ab.cross(ao).cross(ab);
        if (triple.squared_norm() < eps * pow<6>(si::metre))
        {
            auto ab_hat = ab.normalized();
            auto perp = ab_hat.cross(vec3<one>{0.0, 1.0, 0.0});
            if (perp.squared_norm() < eps) perp = ab_hat.cross(vec3<one>{0.0, 0.0, 1.0});
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
    constexpr auto eps = 1e-12;
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
        direction = (triple.squared_norm() < 1e-12 * pow<6>(si::metre)) ? ao.normalized()
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
        direction = (triple.squared_norm() < 1e-12 * pow<6>(si::metre)) ? ao.normalized()
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
std::optional<simplex> gjk_collision(const detail::SupportShape auto &a,
                                     const detail::SupportShape auto &b)
{
    constexpr auto eps = 1e-12;
    simplex simplex;
    vec3<one> direction = {1.0, 0.0, 0.0};

    auto point = detail::minkowski_support(a, b, direction);
    if (point.squared_norm() < eps * pow<2>(si::metre)) return simplex;
    simplex.push_back(point);
    direction = -point.normalized();

    constexpr int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = detail::minkowski_support(a, b, direction);
        auto progress = new_point.dot(direction);
        if (progress <= 0 * si::metre) return std::nullopt;

        simplex.push_back(new_point);
        if (handle_simplex(simplex, direction)) return simplex;
    }

    return std::nullopt;
}

inline bool pad_simplex(const detail::SupportShape auto &a, const detail::SupportShape auto &b,
                        simplex &simplex)
{
    using a_type = std::decay_t<decltype(a)>;
    using b_type = std::decay_t<decltype(b)>;

    auto on_1 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto dir = vec3{1, 0, 0};
        auto p2 = detail::minkowski_support(a, b, dir);

        if ((p2 - simplex[0]).squared_norm() < 1e-6 * pow<2>(si::metre))
            p2 = detail::minkowski_support(a, b, -dir);
        simplex.push_back(p2);
    };
    auto on_2 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto line = (simplex[1] - simplex[0]);
        auto dir = line.normalized().cross(vec3<one>{0.0, 1.0, 0.0});
        if (dir.squared_norm() < 1e-6) dir = line.normalized().cross(vec3<one>{0.0, 0.0, 1.0});
        dir.normalize();
        auto p3 = detail::minkowski_support(a, b, dir);
        if (line.cross(p3 - simplex[0]).squared_norm() < 1e-6 * pow<4>(si::metre))
            p3 = detail::minkowski_support(a, b, -dir);
        simplex.push_back(p3);
    };
    auto on_3 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto ab = simplex[1] - simplex[0];
        auto ac = simplex[2] - simplex[0];
        auto dir = ab.cross(ac).normalized();
        auto p4 = detail::minkowski_support(a, b, dir);
        if (abs(ab.cross(p4 - simplex[0]).dot(dir)) < 1e-6 * pow<2>(si::metre))
            p4 = detail::minkowski_support(a, b, -dir);
        simplex.push_back(p4);
    };

    auto ons = std::array<void (*)(const a_type &, const b_type &, physkit::simplex &), 3>{
        on_1, on_2, on_3};

    for (auto i = simplex.size(); i < 4; ++i) ons[i](a, b, simplex);

    auto ad = simplex[0] - simplex[3];
    auto bd = simplex[1] - simplex[3];
    auto cd = simplex[2] - simplex[3];
    auto triple = ad.dot(bd.cross(cd));

    return abs(triple) > 1e-12 * pow<3>(si::metre);
}

/// @brief return information that intersection occured
inline std::optional<collision_info> epa(const detail::SupportShape auto &a,
                                         const detail::SupportShape auto &b, simplex &simplex)
{
    if (simplex.size() < 4 && !pad_simplex(a, b, simplex))
        return std::nullopt; // Degenerate case, treat as no collision

    absl::InlinedVector<vec3<si::metre>, 64> polytope(simplex.begin(), simplex.end());

    struct face
    {
        std::size_t a{}, b{}, c{};
        vec3<one> normal;
        quantity<si::metre> distance{};
    };

    auto compute_face = [&](std::size_t i, std::size_t j, std::size_t k) -> face
    {
        auto ab = polytope[j] - polytope[i];
        auto ac = polytope[k] - polytope[i];
        face f{.a = i, .b = j, .c = k, .normal = ab.cross(ac) / pow<2>(si::metre)};

        if (f.normal.squared_norm() < 1e-12)
            f.normal = vec3<one>::zero();
        else
            f.normal.normalize();

        f.distance = f.normal.dot(polytope[i]);
        if (f.distance < 0.0 * si::metre)
        {
            f.normal = -f.normal;
            f.distance = -f.distance;
            std::swap(f.b, f.c);
        }

        return f;
    };

    absl::InlinedVector<face, 64> faces;
    faces.push_back(compute_face(0, 1, 2));
    faces.push_back(compute_face(0, 2, 3));
    faces.push_back(compute_face(0, 3, 1));
    faces.push_back(compute_face(1, 3, 2));

    constexpr int max_iterations = 32;
    constexpr auto tolerance = 1e-6 * si::metre;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto min_face = *std::ranges::min_element(faces, [](const face &f1, const face &f2)
                                                  { return f1.distance < f2.distance; });

        auto p = detail::minkowski_support(a, b, min_face.normal);
        auto p_dist = min_face.normal.dot(p);

        if (p_dist - min_face.distance < tolerance) // convergence
            return collision_info{
                .normal = -min_face.normal,
                .depth = min_face.distance,
            };

        absl::InlinedVector<std::pair<std::size_t, std::size_t>, 64> edges;
        auto add_edge = [&](std::size_t a, std::size_t b)
        {
            auto it = std::ranges::find_if(edges, [&](const auto &e)
                                           { return (e.first == b && e.second == a); });
            if (it != edges.end())
                edges.erase(it);
            else
                edges.emplace_back(a, b);
        };
        for (auto it = faces.begin(); it != faces.end();)
        {
            if (it->normal.dot(p) > it->distance + tolerance)
            {
                add_edge(it->a, it->b);
                add_edge(it->b, it->c);
                add_edge(it->c, it->a);
                it = faces.erase(it);
            }
            else
                ++it; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }

        polytope.push_back(p);
        for (const auto &[i, j] : edges) faces.push_back(compute_face(i, j, polytope.size() - 1));
    }

    auto min_face = *std::ranges::min_element(faces, [](const face &f1, const face &f2)
                                              { return f1.distance < f2.distance; });
    return collision_info{
        .normal = -min_face.normal, // negate: point from B toward A
        .depth = min_face.distance,
    };
}

/// @brief return data for obb obb collision
std::optional<collision_info> gjk_epa(detail::SupportShape auto const &a,
                                      detail::SupportShape auto const &b)
{
    auto simplex = gjk_collision(a, b);
    if (simplex)
    {
        auto info = epa(a, b, *simplex);
        return info;
    }
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
{
    assert(false && "SAT not implemented");
}
} // namespace physkit