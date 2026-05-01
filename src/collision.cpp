#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/collision/collision.h"
#include <absl/container/inlined_vector.h>
#endif

namespace physkit
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
inline support_pt minkowski_support(const ShapeA &a, const ShapeB &b, const vec3<one> &direction)
{
    auto pa = a.support(direction);
    auto pb = b.support(-direction);
    return support_pt{.p = pa - pb, .pa = pa, .pb = pb};
}

/// based off winter dev gjk algorithm implementation

using simplex = absl::InlinedVector<support_pt, 4>;

inline bool handle_line(simplex &simplex, vec3<one> &direction)
{
    constexpr auto eps = 1e-12;
    const auto a = simplex[1];
    const auto b = simplex[0];
    const auto ab = b.p - a.p;
    const auto ao = -a.p;
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

    const auto ab = b.p - a.p;
    const auto ac = c.p - a.p;
    const auto ao = -a.p;
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
    const auto ao = -a.p;

    auto abc = (b.p - a.p).cross(c.p - a.p);
    auto acd = (c.p - a.p).cross(d.p - a.p);
    auto adb = (d.p - a.p).cross(b.p - a.p);

    auto orient_face_outward = [](auto &normal, const auto &face_a, const auto &opposite)
    {
        auto toward_opposite = opposite.p - face_a.p;
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
std::optional<simplex> gjk_collision(const SupportShape auto &a, const SupportShape auto &b)
{
    constexpr auto eps = 1e-12;
    simplex simplex;
    vec3<one> direction = {1.0, 0.0, 0.0};

    auto point = minkowski_support(a, b, direction);
    simplex.push_back(point);
    if (point.p.squared_norm() < eps * pow<2>(si::metre)) return simplex;
    direction = -point.p.normalized();

    constexpr int max_iterations = 100;
    constexpr auto progress_tol = 1e-9 * si::metre; // accept boundary contact (within ~nm)
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = minkowski_support(a, b, direction);
        auto progress = new_point.p.dot(direction);
        if (progress < -progress_tol) return std::nullopt;

        simplex.push_back(new_point);
        if (handle_simplex(simplex, direction)) return simplex;
    }

    return std::nullopt;
}

inline bool pad_simplex(const SupportShape auto &a, const SupportShape auto &b, simplex &simplex)
{
    using a_type = std::decay_t<decltype(a)>;
    using b_type = std::decay_t<decltype(b)>;

    auto on_1 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto dir = vec3{1, 0, 0};
        auto p2 = minkowski_support(a, b, dir);

        if ((p2.p - simplex[0].p).squared_norm() < 1e-6 * pow<2>(si::metre))
            p2 = minkowski_support(a, b, -dir);
        simplex.push_back(p2);
    };
    auto on_2 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto line = (simplex[1].p - simplex[0].p);
        auto dir = line.normalized().cross(vec3<one>{0.0, 1.0, 0.0});
        if (dir.squared_norm() < 1e-6) dir = line.normalized().cross(vec3<one>{0.0, 0.0, 1.0});
        dir.normalize();
        auto p3 = minkowski_support(a, b, dir);
        if (line.cross(p3.p - simplex[0].p).squared_norm() < 1e-6 * pow<4>(si::metre))
            p3 = minkowski_support(a, b, -dir);
        simplex.push_back(p3);
    };
    auto on_3 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto ab = simplex[1].p - simplex[0].p;
        auto ac = simplex[2].p - simplex[0].p;
        auto dir = ab.cross(ac).normalized();
        auto p4 = minkowski_support(a, b, dir);
        if (abs((p4.p - simplex[0].p).dot(dir)) < 1e-6 * si::metre)
            p4 = minkowski_support(a, b, -dir);
        simplex.push_back(p4);
    };

    switch (simplex.size())
    {
    case 1:
        on_1(a, b, simplex);
        [[fallthrough]];
    case 2:
        on_2(a, b, simplex);
        [[fallthrough]];
    case 3:
        on_3(a, b, simplex);
    default:
        break;
    }

    auto ad = simplex[0].p - simplex[3].p;
    auto bd = simplex[1].p - simplex[3].p;
    auto cd = simplex[2].p - simplex[3].p;
    auto triple = ad.dot(bd.cross(cd));

    return abs(triple) > 1e-12 * pow<3>(si::metre);
}

// TODO: hill climbing to find support point in O(\sqrt(n))
struct epa_solver
{
    using index_t = std::uint16_t;
    static constexpr auto null_index = static_cast<index_t>(-1);
    static constexpr auto buffer_size = 128;
    struct face
    {
        std::array<index_t, 3> vertices{};
        std::array<index_t, 3> adj = {null_index, null_index, null_index};

        vec3<one> normal;
        quantity<si::metre> distance{};
        bool obsolete = false;
    };

    struct silhouette_edge
    {
        index_t start_vertex;
        index_t end_vertex;
        index_t adjacent_face;
    };

    void init_face(std::size_t f_idx, std::size_t i, std::size_t j, std::size_t k,
                   index_t opposite_index = null_index)
    {
        auto &face = faces[f_idx];
        face.vertices = {static_cast<epa_solver::index_t>(i), static_cast<epa_solver::index_t>(j),
                         static_cast<epa_solver::index_t>(k)};

        auto ab = polytope[j].p - polytope[i].p;
        auto ac = polytope[k].p - polytope[i].p;
        face.normal = ab.cross(ac) / pow<2>(si::metre);

        if (face.normal.squared_norm() < 1e-24)
        {
            face.normal = vec3<one>::zero();
            face.distance = quantity<si::metre>::max();
        }
        else
        {
            face.normal.normalize();

            if (opposite_index != null_index &&
                face.normal.dot(polytope[opposite_index].p - polytope[i].p) > 0.0 * si::metre)
            {
                std::swap(face.vertices[1], face.vertices[2]);
                face.normal = -face.normal;
            }

            face.distance = face.normal.dot(polytope[i].p);
        }
    }

    std::size_t allocate_face()
    {
        faces.emplace_back();
        return faces.size() - 1;
    }

    void link_faces(std::size_t f1, std::size_t f2, std::size_t v_a, std::size_t v_b)
    {
        auto &a = faces[f1];
        auto &b = faces[f2];
        auto edge1 = (a.vertices[0] == v_a) ? 0 : (a.vertices[1] == v_a ? 1 : 2);
        auto edge2 = (b.vertices[0] == v_b) ? 0 : (b.vertices[1] == v_b ? 1 : 2);
        a.adj[edge1] = static_cast<index_t>(f2);
        b.adj[edge2] = static_cast<index_t>(f1);
    }

    absl::InlinedVector<silhouette_edge, 32> find_silhouette(index_t face_idx,
                                                             const vec3<si::metre> &p)
    {
        constexpr auto tolerance = 1e-6;
        absl::InlinedVector<std::size_t, 32> stack{face_idx};
        faces[face_idx].obsolete = true;

        absl::InlinedVector<silhouette_edge, 32> horizon;

        while (!stack.empty())
        {
            std::size_t cur = stack.back();
            stack.pop_back();
            auto &cur_face = faces[cur];

            for (std::size_t i = 0; i < 3; ++i)
            {
                std::size_t n_idx = cur_face.adj[i];
                if (n_idx == null_index) continue;

                auto &neighbor = faces[n_idx];
                if (neighbor.obsolete) continue;

                if (neighbor.normal.dot(p) > neighbor.distance + tolerance * si::metre)
                {
                    neighbor.obsolete = true;
                    stack.push_back(n_idx);
                }
                else
                    horizon.push_back({
                        .start_vertex = cur_face.vertices[i],
                        .end_vertex = cur_face.vertices[(i + 1) % 3],
                        .adjacent_face = static_cast<index_t>(n_idx),
                    });
            }
        }

        return horizon;
    }

    void build_initial_tetrahedron()
    {
        auto alloc_init_push =
            [&](std::size_t i, std::size_t j, std::size_t k, index_t opposite_index)
        {
            auto f_idx = allocate_face();
            init_face(f_idx, i, j, k, opposite_index);
            push_face(f_idx);
            return f_idx;
        };
        auto f0 = alloc_init_push(0, 1, 2, 3);
        auto f1 = alloc_init_push(0, 2, 3, 1);
        auto f2 = alloc_init_push(0, 3, 1, 2);
        auto f3 = alloc_init_push(1, 3, 2, 0);

        // Brute force initial 4 faces
        for (std::size_t i = 0; i < 4; ++i)
            for (std::size_t j = i + 1; j < 4; ++j)
                for (std::size_t e1 = 0; e1 < 3; ++e1)
                {
                    auto u1 = faces[i].vertices[e1];
                    auto v1 = faces[i].vertices[(e1 + 1) % 3];
                    for (std::size_t e2 = 0; e2 < 3; ++e2)
                    {
                        auto u2 = faces[j].vertices[e2];
                        auto v2 = faces[j].vertices[(e2 + 1) % 3];
                        if (u1 == v2 && v1 == u2)
                        {
                            faces[i].adj[e1] = static_cast<index_t>(j);
                            faces[j].adj[e2] = static_cast<index_t>(i);
                        }
                    }
                }
    }

    void push_face(std::size_t f_idx)
    {
        face_heap.push_back(f_idx);
        std::ranges::push_heap(face_heap, [&](std::size_t f1, std::size_t f2)
                               { return faces[f1].distance > faces[f2].distance; });
    }

    auto pop_face()
    {
        while (!face_heap.empty())
        {
            std::ranges::pop_heap(face_heap, [&](std::size_t f1, std::size_t f2)
                                  { return faces[f1].distance > faces[f2].distance; });
            auto f_idx = face_heap.back();
            face_heap.pop_back();
            if (!faces[f_idx].obsolete) return f_idx;
        }
        return static_cast<std::size_t>(null_index);
    }

    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    static std::optional<collision_info> solve(const SupportShape auto &a,
                                               const SupportShape auto &b, simplex &simplex)
    {
        if (simplex.size() < 4 && !pad_simplex(a, b, simplex))
            return std::nullopt; // Degenerate case, treat as no collision

        epa_solver solver{.polytope = {simplex.begin(), simplex.end()}};

        solver.build_initial_tetrahedron();

        constexpr int max_iterations = 64;
        constexpr auto tolerance = 1e-8 * si::metre;

        auto get_barycentric = [&](const face &f)
        {
            auto p0 = solver.polytope[f.vertices[0]];
            auto p1 = solver.polytope[f.vertices[1]];
            auto p2 = solver.polytope[f.vertices[2]];

            auto p_minkowski = f.normal * f.distance;

            auto v0 = p1.p - p0.p;
            auto v1 = p2.p - p0.p;
            auto v2 = p_minkowski - p0.p;

            auto d00 = v0.dot(v0);
            auto d01 = v0.dot(v1);
            auto d11 = v1.dot(v1);
            auto d20 = v2.dot(v0);
            auto d21 = v2.dot(v1);

            auto denom = d00 * d11 - d01 * d01;

            auto v = (d11 * d20 - d01 * d21) / denom;
            auto w = (d00 * d21 - d01 * d20) / denom;
            auto u = 1.0 - v - w;

            return collision_info{
                .normal = -f.normal,
                .world_a = u * p0.pa + v * p1.pa + w * p2.pa,
                .world_b = u * p0.pb + v * p1.pb + w * p2.pb,
                .depth = f.distance,
            };
        };

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            std::size_t min_face_idx = solver.pop_face();
            if (min_face_idx == null_index) break;

            const auto min_face_snapshot = solver.faces[min_face_idx];

            auto p = minkowski_support(a, b, min_face_snapshot.normal);
            auto p_dist = min_face_snapshot.normal.dot(p.p);
            if (p_dist - min_face_snapshot.distance < tolerance) // convergence
                return get_barycentric(min_face_snapshot);

            auto horizon = solver.find_silhouette(min_face_idx, p.p);
            // degenerate.
            if (horizon.empty()) return get_barycentric(min_face_snapshot);

            solver.polytope.push_back(p);
            auto p_idx = static_cast<index_t>(solver.polytope.size() - 1);

            absl::InlinedVector<std::size_t, 32> new_faces;
            for (const auto &[start, end, adj_face] : horizon)
            {
                auto f = solver.allocate_face();
                solver.init_face(f, start, end, p_idx);
                solver.link_faces(f, adj_face, start, end);
                solver.push_face(f);
                new_faces.push_back(f);
            }

            std::size_t n = new_faces.size();
            for (std::size_t i = 0; i < n; ++i)
            {
                for (std::size_t j = i + 1; j < n; ++j)
                {
                    auto i_idx = new_faces[i];
                    auto j_idx = new_faces[j];

                    if (horizon[i].end_vertex == horizon[j].start_vertex)
                        solver.link_faces(i_idx, j_idx, horizon[i].end_vertex, p_idx);
                    else if (horizon[i].start_vertex == horizon[j].end_vertex)
                        solver.link_faces(j_idx, i_idx, horizon[j].end_vertex, p_idx);
                }
            }
        }

        std::size_t min_face_idx = solver.pop_face();
        if (min_face_idx == null_index) return std::nullopt;
        return get_barycentric(solver.faces[min_face_idx]);
    }

    absl::InlinedVector<face, buffer_size> faces;
    absl::InlinedVector<support_pt, buffer_size> polytope;
    absl::InlinedVector<std::size_t, buffer_size> face_heap;
};

// NOLINTNEXTLINE
#define DEFINE_MIRROR_IMPL(a_t, b_t)                                                               \
    std::optional<collision_info> b_t##_##a_t(const instance &a, const instance &b)                \
    {                                                                                              \
        if (auto info = a_t##_##b_t(b, a))                                                         \
        {                                                                                          \
            info->normal = -info->normal;                                                          \
            std::swap(info->world_a, info->world_b);                                               \
            return info;                                                                           \
        }                                                                                          \
        return std::nullopt;                                                                       \
    }

inline std::optional<collision_info> gjk_epa(const instance &a, const instance &b)
{
    auto simplex = gjk_collision(a, b);
    if (!simplex) return std::nullopt;
    return epa_solver::solve(a, b, *simplex);
}

std::optional<collision_info> sphere_sphere(const instance &a, const instance &b)
{
    using namespace mp_units::si::unit_symbols;
    const auto ra = a.geometry().sphere().radius();
    const auto rb = b.geometry().sphere().radius();
    const auto diff = a.position() - b.position();
    const auto dist2 = diff.squared_norm();
    const auto rsum = ra + rb;

    if (dist2 > rsum * rsum) return std::nullopt;

    vec3<one> normal;
    quantity<si::metre> dist; // NOLINT
    if (dist2 < 1e-24 * pow<2>(si::metre))
    {
        normal = vec3<one>{1.0, 0.0, 0.0};
        dist = 0.0 * m;
    }
    else
    {
        dist = mp_units::sqrt(dist2);
        normal = diff / dist;
    }

    return collision_info{
        .normal = normal,
        .world_a = a.position() - ra * normal,
        .world_b = b.position() + rb * normal,
        .depth = rsum - dist,
    };
}

std::optional<collision_info> box_sphere(const instance &a, const instance &b)
{
    using namespace mp_units::si::unit_symbols;
    const auto &bx = a.geometry().box();
    const auto he = bx.half_extents();
    const auto box_q = a.orientation();
    const auto sph_center = b.position();
    const auto sph_radius = b.geometry().sphere().radius();

    const auto local_center = box_q.conjugate() * (sph_center - a.position());

    const auto clamp = [](quantity<si::metre> v, quantity<si::metre> lim)
    { return v < -lim ? -lim : (v > lim ? lim : v); };

    vec3<si::metre> cp_local{clamp(local_center.x(), he.x()), clamp(local_center.y(), he.y()),
                             clamp(local_center.z(), he.z())};

    const auto diff_local = cp_local - local_center;
    const auto dist2 = diff_local.squared_norm();

    if (dist2 < 1e-24 * pow<2>(si::metre))
    {
        const auto dx = he.x() - mp_units::abs(local_center.x());
        const auto dy = he.y() - mp_units::abs(local_center.y());
        const auto dz = he.z() - mp_units::abs(local_center.z());

        vec3<one> local_normal;
        quantity<si::metre> penetration; // NOLINT
        vec3<si::metre> cp_face;
        if (dx <= dy && dx <= dz)
        {
            const auto sign = local_center.x() >= 0.0 * m ? 1.0 : -1.0;
            local_normal = vec3<one>{-sign, 0.0, 0.0};
            penetration = dx;
            cp_face = vec3<si::metre>{sign * he.x(), local_center.y(), local_center.z()};
        }
        else if (dy <= dz)
        {
            const auto sign = local_center.y() >= 0.0 * m ? 1.0 : -1.0;
            local_normal = vec3<one>{0.0, -sign, 0.0};
            penetration = dy;
            cp_face = vec3<si::metre>{local_center.x(), sign * he.y(), local_center.z()};
        }
        else
        {
            const auto sign = local_center.z() >= 0.0 * m ? 1.0 : -1.0;
            local_normal = vec3<one>{0.0, 0.0, -sign};
            penetration = dz;
            cp_face = vec3<si::metre>{local_center.x(), local_center.y(), sign * he.z()};
        }

        const auto world_normal = box_q * local_normal;
        return collision_info{
            .normal = world_normal,
            .world_a = box_q * cp_face + a.position(),
            .world_b = sph_center + sph_radius * world_normal,
            .depth = penetration + sph_radius,
        };
    }

    if (dist2 > sph_radius * sph_radius) return std::nullopt;

    const auto dist = mp_units::sqrt(dist2);
    const auto local_normal = diff_local / dist;
    const auto world_normal = box_q * local_normal;

    return collision_info{
        .normal = world_normal,
        .world_a = box_q * cp_local + a.position(),
        .world_b = sph_center + sph_radius * world_normal,
        .depth = sph_radius - dist,
    };
}

struct sat_face
{
    absl::InlinedVector<std::uint8_t, 4> vertices;
    vec3<one> normal;
};

struct sat_edge
{
    std::uint8_t start{};
    std::uint8_t end{};
    vec3<one> direction;
};

struct sat_polyhedron
{
    absl::InlinedVector<vec3<si::metre>, 8> vertices;
    absl::InlinedVector<sat_face, 6> faces;
    absl::InlinedVector<sat_edge, 12> edges;
    absl::InlinedVector<vec3<one>, 8> edge_directions;
    vec3<si::metre> center;
};

inline void sat_add_face(sat_polyhedron &poly, std::initializer_list<std::uint8_t> indices)
{
    sat_face face;
    face.vertices.assign(indices);

    auto normal = (poly.vertices[face.vertices[1]] - poly.vertices[face.vertices[0]])
                      .cross(poly.vertices[face.vertices[2]] - poly.vertices[face.vertices[0]]) /
                  pow<2>(si::metre);
    if (normal.squared_norm() < 1e-24) return;
    normal.normalize();

    if (normal.dot(poly.center - poly.vertices[face.vertices[0]]) > 0.0 * si::metre)
    {
        normal = -normal;
        std::ranges::reverse(face.vertices);
    }

    face.normal = normal;
    poly.faces.push_back(face);
}

inline void sat_add_edge(sat_polyhedron &poly, std::uint8_t start, std::uint8_t end)
{
    static constexpr auto parallel_dot = 1.0 - 1e-9;

    auto edge = poly.vertices[end] - poly.vertices[start];
    if (edge.squared_norm() < 1e-24 * pow<2>(si::metre)) return;

    auto direction = edge.normalized();
    poly.edges.push_back({.start = start, .end = end, .direction = direction});

    for (const auto &existing : poly.edge_directions)
        if (abs(existing.dot(direction)) > parallel_dot) return;
    poly.edge_directions.push_back(direction);
}

inline sat_polyhedron make_box_polyhedron(const instance &inst)
{
    const auto he = inst.geometry().box().half_extents();
    const auto &q = inst.orientation();

    sat_polyhedron poly{.center = inst.position()};
    const std::array local_vertices = {
        vec3<si::metre>{-he.x(), -he.y(), -he.z()}, vec3<si::metre>{he.x(), -he.y(), -he.z()},
        vec3<si::metre>{he.x(), he.y(), -he.z()},   vec3<si::metre>{-he.x(), he.y(), -he.z()},
        vec3<si::metre>{-he.x(), -he.y(), he.z()},  vec3<si::metre>{he.x(), -he.y(), he.z()},
        vec3<si::metre>{he.x(), he.y(), he.z()},    vec3<si::metre>{-he.x(), he.y(), he.z()},
    };

    for (const auto &v : local_vertices) poly.vertices.push_back(q * v + inst.position());

    sat_add_face(poly, {0, 3, 2, 1});
    sat_add_face(poly, {4, 5, 6, 7});
    sat_add_face(poly, {0, 1, 5, 4});
    sat_add_face(poly, {3, 7, 6, 2});
    sat_add_face(poly, {0, 4, 7, 3});
    sat_add_face(poly, {1, 2, 6, 5});

    sat_add_edge(poly, 0, 1);
    sat_add_edge(poly, 1, 2);
    sat_add_edge(poly, 2, 3);
    sat_add_edge(poly, 3, 0);
    sat_add_edge(poly, 4, 5);
    sat_add_edge(poly, 5, 6);
    sat_add_edge(poly, 6, 7);
    sat_add_edge(poly, 7, 4);
    sat_add_edge(poly, 0, 4);
    sat_add_edge(poly, 1, 5);
    sat_add_edge(poly, 2, 6);
    sat_add_edge(poly, 3, 7);

    return poly;
}

inline sat_polyhedron make_pyramid_polyhedron(const instance &inst)
{
    using namespace mp_units::si::unit_symbols;

    const auto &p = inst.geometry().pyramid();
    const auto b = p.base_half();
    const auto h = p.height();
    const auto &q = inst.orientation();

    sat_polyhedron poly{.center = q * p.mass_center() + inst.position()};
    const std::array local_vertices = {
        vec3{b, 0.0 * m, b},  vec3{-b, 0.0 * m, b},      vec3{-b, 0.0 * m, -b},
        vec3{b, 0.0 * m, -b}, vec3{0.0 * m, h, 0.0 * m},
    };

    for (const auto &v : local_vertices) poly.vertices.push_back(q * v + inst.position());

    sat_add_face(poly, {0, 4, 1});
    sat_add_face(poly, {1, 4, 2});
    sat_add_face(poly, {2, 4, 3});
    sat_add_face(poly, {3, 4, 0});
    sat_add_face(poly, {0, 1, 2, 3});

    sat_add_edge(poly, 0, 1);
    sat_add_edge(poly, 1, 2);
    sat_add_edge(poly, 2, 3);
    sat_add_edge(poly, 3, 0);
    sat_add_edge(poly, 0, 4);
    sat_add_edge(poly, 1, 4);
    sat_add_edge(poly, 2, 4);
    sat_add_edge(poly, 3, 4);

    return poly;
}

enum class sat_feature : uint8_t
{
    face_a,
    face_b,
    edge
};

struct sat_axis
{
    std::size_t a_index = 0;
    std::size_t b_index = 0;
    sat_feature feature = sat_feature::face_a;
};

inline std::pair<quantity<si::metre>, quantity<si::metre>> sat_project(const sat_polyhedron &poly,
                                                                       const vec3<one> &axis)
{
    auto min = poly.vertices[0].dot(axis);
    auto max = min;
    for (const auto &v : poly.vertices)
    {
        const auto projected = v.dot(axis);
        min = std::min(min, projected);
        max = std::max(max, projected);
    }
    return {min, max};
}

inline std::size_t sat_face_supporting(const sat_polyhedron &poly, const vec3<one> &outward)
{
    std::size_t best = 0;
    auto best_dot = poly.faces[0].normal.dot(outward);
    for (std::size_t i = 1; i < poly.faces.size(); ++i)
        if (auto dot = poly.faces[i].normal.dot(outward); dot > best_dot)
        {
            best_dot = dot;
            best = i;
        }
    return best;
}

using sat_contact_polygon = absl::InlinedVector<vec3<si::metre>, 8>;

inline sat_contact_polygon sat_face_vertices(const sat_polyhedron &poly, const sat_face &face)
{
    sat_contact_polygon polygon;
    for (auto index : face.vertices) polygon.push_back(poly.vertices[index]);
    return polygon;
}

inline sat_contact_polygon sat_clip_polygon(const sat_contact_polygon &polygon,
                                            const vec3<one> &plane_normal,
                                            quantity<si::metre> offset)
{
    static constexpr auto clip_tol = 1e-9 * si::metre;
    sat_contact_polygon clipped;
    if (polygon.empty()) return clipped;

    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
        const auto &a_pt = polygon[i];
        const auto &b_pt = polygon[(i + 1) % polygon.size()];
        const auto da = plane_normal.dot(a_pt) - offset;
        const auto db = plane_normal.dot(b_pt) - offset;
        const auto a_inside = da <= clip_tol;
        const auto b_inside = db <= clip_tol;

        if (a_inside) clipped.push_back(a_pt);
        if (a_inside != b_inside)
        {
            const auto t = da / (da - db);
            clipped.push_back(a_pt + (b_pt - a_pt) * t);
        }
    }
    return clipped;
}

inline vec3<si::metre> sat_project_to_plane(const vec3<si::metre> &point,
                                            const vec3<si::metre> &plane_point,
                                            const vec3<one> &plane_normal)
{ return point - plane_normal * (point - plane_point).dot(plane_normal); }

// TODO: replace with smarter point selection

inline bool sat_point_in_polygon(const sat_contact_polygon &polygon, const vec3<si::metre> &point,
                                 const vec3<one> &plane_normal)
{
    if (polygon.size() < 3) return false;

    static constexpr auto side_tol = 1e-9 * pow<2>(si::metre);
    int sign = 0;

    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
        const auto &a = polygon[i];
        const auto &b = polygon[(i + 1) % polygon.size()];
        const auto side = (b - a).cross(point - a).dot(plane_normal);
        const int current = side > side_tol ? 1 : (side < -side_tol ? -1 : 0);

        if (current == 0) continue;
        if (sign == 0)
            sign = current;
        else if (sign != current)
            return false;
    }

    return true;
}

inline vec3<si::metre> sat_closest_point_on_segment(const vec3<si::metre> &point,
                                                    const vec3<si::metre> &a,
                                                    const vec3<si::metre> &b)
{
    const auto ab = b - a;
    const auto ab_len2 = ab.squared_norm();
    static constexpr auto eps = 1e-24 * pow<2>(si::metre);
    if (ab_len2 <= eps) return a;

    return a + ab * std::clamp(static_cast<double>((point - a).dot(ab) / ab_len2), 0.0, 1.0);
}

inline vec3<si::metre> sat_closest_point_on_polygon(const sat_contact_polygon &polygon,
                                                    const vec3<si::metre> &point,
                                                    const vec3<one> &plane_normal)
{
    const auto projected = sat_project_to_plane(point, polygon[0], plane_normal);
    if (sat_point_in_polygon(polygon, projected, plane_normal)) return projected;

    auto best = polygon.front();
    auto best_dist2 = (best - point).squared_norm();
    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
        const auto candidate =
            sat_closest_point_on_segment(point, polygon[i], polygon[(i + 1) % polygon.size()]);
        if (const auto dist2 = (candidate - point).squared_norm(); dist2 < best_dist2)
        {
            best = candidate;
            best_dist2 = dist2;
        }
    }

    return best;
}

inline vec3<si::metre> sat_representative_face_point(const sat_contact_polygon &polygon,
                                                     const vec3<one> &plane_normal,
                                                     const vec3<si::metre> &center_a,
                                                     const vec3<si::metre> &center_b)
{
    auto projected_inside = [&](const vec3<si::metre> &center) -> std::optional<vec3<si::metre>>
    {
        const auto projected = sat_project_to_plane(center, polygon[0], plane_normal);
        if (sat_point_in_polygon(polygon, projected, plane_normal)) return projected;
        return std::nullopt;
    };

    const auto projected_a = projected_inside(center_a);
    const auto projected_b = projected_inside(center_b);

    if (projected_a && projected_b) return (*projected_a + *projected_b) * 0.5;
    if (projected_a) return *projected_a;
    if (projected_b) return *projected_b;

    const auto closest_a = sat_closest_point_on_polygon(polygon, center_a, plane_normal);
    const auto closest_b = sat_closest_point_on_polygon(polygon, center_b, plane_normal);
    return (closest_a + closest_b) * 0.5;
}

inline collision_info sat_clipped_face_contact(const sat_polyhedron &a, const sat_polyhedron &b,
                                               const vec3<one> &normal,
                                               quantity<si::metre> fallback_depth, bool ref_is_a)
{
    static constexpr auto contact_tol = 1e-7 * si::metre;

    const auto &ref = ref_is_a ? a : b;
    const auto &inc = ref_is_a ? b : a;
    const auto ref_outward = ref_is_a ? -normal : normal;
    const auto ref_face_idx = sat_face_supporting(ref, ref_outward);
    const auto inc_face_idx = sat_face_supporting(inc, -ref_outward);
    const auto &ref_face = ref.faces[ref_face_idx];
    const auto &inc_face = inc.faces[inc_face_idx];

    auto polygon = sat_face_vertices(inc, inc_face);
    for (std::size_t i = 0; i < ref_face.vertices.size(); ++i)
    {
        const auto p0 = ref.vertices[ref_face.vertices[i]];
        const auto p1 = ref.vertices[ref_face.vertices[(i + 1) % ref_face.vertices.size()]];
        auto side_normal = (p1 - p0).cross(ref_outward) / si::metre;
        if (side_normal.squared_norm() < 1e-24) continue;
        side_normal.normalize();
        polygon = sat_clip_polygon(polygon, side_normal, side_normal.dot(p0));
    }

    const auto ref_offset = ref_outward.dot(ref.vertices[ref_face.vertices[0]]);
    auto best_depth = -std::numeric_limits<quantity<si::metre>>::infinity();
    auto shallowest_depth = std::numeric_limits<quantity<si::metre>>::infinity();
    auto sum_a = vec3<si::metre>::zero();
    auto sum_b = vec3<si::metre>::zero();
    std::size_t contact_count = 0;
    sat_contact_polygon valid_polygon;

    for (const auto &p : polygon)
    {
        const auto depth = ref_offset - ref_outward.dot(p);
        if (depth < -contact_tol) continue;

        valid_polygon.push_back(p);
        shallowest_depth = std::min(shallowest_depth, depth);

        const auto ref_point = p + ref_outward * depth;
        const auto world_a = ref_is_a ? ref_point : p;
        const auto world_b = ref_is_a ? p : ref_point;

        if (depth > best_depth + contact_tol)
        {
            best_depth = depth;
            sum_a = vec3<si::metre>::zero();
            sum_b = vec3<si::metre>::zero();
            contact_count = 0;
        }
        if (mp_units::abs(depth - best_depth) <= contact_tol)
        {
            sum_a += world_a;
            sum_b += world_b;
            ++contact_count;
        }
    }

    if (valid_polygon.empty())
    {
        const auto world_a = a.vertices.front();
        return collision_info{
            .normal = normal,
            .world_a = world_a,
            .world_b = world_a + normal * fallback_depth,
            .depth = fallback_depth,
        };
    }

    if (best_depth - shallowest_depth <= contact_tol)
    {
        const auto p =
            sat_representative_face_point(valid_polygon, inc_face.normal, a.center, b.center);
        const auto depth = ref_offset - ref_outward.dot(p);
        const auto ref_point = p + ref_outward * depth;

        return collision_info{
            .normal = normal,
            .world_a = ref_is_a ? ref_point : p,
            .world_b = ref_is_a ? p : ref_point,
            .depth = depth,
        };
    }

    const auto inv_count = 1.0 / static_cast<double>(contact_count);
    return collision_info{
        .normal = normal,
        .world_a = sum_a * inv_count,
        .world_b = sum_b * inv_count,
        .depth = best_depth,
    };
}

inline sat_edge sat_supporting_edge(const sat_polyhedron &poly, std::size_t direction_idx,
                                    const vec3<one> &outward)
{
    constexpr auto parallel_dot = 1.0 - 1e-8;
    const auto direction = poly.edge_directions[direction_idx];
    std::size_t best = 0;
    auto best_score = -std::numeric_limits<quantity<si::metre>>::infinity();

    for (std::size_t i = 0; i < poly.edges.size(); ++i)
    {
        const auto &edge = poly.edges[i];
        if (abs(edge.direction.dot(direction)) < parallel_dot) continue;

        const auto score = (poly.vertices[edge.start] + poly.vertices[edge.end]).dot(outward);
        if (score > best_score)
        {
            best_score = score;
            best = i;
        }
    }

    return poly.edges[best];
}

inline std::pair<vec3<si::metre>, vec3<si::metre>>
sat_closest_segment_points(const vec3<si::metre> &p1, const vec3<si::metre> &q1,
                           const vec3<si::metre> &p2, const vec3<si::metre> &q2)
{
    const auto d1 = q1 - p1;
    const auto d2 = q2 - p2;
    const auto r = p1 - p2;
    const auto a_len = d1.dot(d1);
    const auto e_len = d2.dot(d2);
    const auto f = d2.dot(r);
    static constexpr auto eps = 1e-24 * pow<2>(si::metre);

    double s = 0.0;
    double t = 0.0;
    auto clamp01 = [](auto v) { return std::clamp(static_cast<double>(v), 0.0, 1.0); };

    if (a_len <= eps && e_len <= eps) return {p1, p2};
    if (a_len <= eps)
        t = clamp01(f / e_len);
    else
    {
        const auto c = d1.dot(r);
        if (e_len <= eps)
            s = clamp01(-c / a_len);
        else
        {
            const auto b = (d1.dot(d2));
            if (auto denom = a_len * e_len - b * b; abs(denom) > eps * pow<2>(si::metre))
                s = clamp01((b * f - c * e_len) / denom);

            t = static_cast<double>((b * s + f) / e_len);
            if (t < 0.0)
            {
                t = 0.0;
                s = clamp01(-c / a_len);
            }
            else if (t > 1.0)
            {
                t = 1.0;
                s = clamp01((b - c) / a_len);
            }
        }
    }

    return {p1 + d1 * s, p2 + d2 * t};
}

inline collision_info sat_edge_contact(const sat_polyhedron &a, const sat_polyhedron &b,
                                       const vec3<one> &normal, quantity<si::metre> fallback_depth,
                                       std::size_t a_direction, std::size_t b_direction)
{
    const auto edge_a = sat_supporting_edge(a, a_direction, -normal);
    const auto edge_b = sat_supporting_edge(b, b_direction, normal);
    auto [world_a, world_b] =
        sat_closest_segment_points(a.vertices[edge_a.start], a.vertices[edge_a.end],
                                   b.vertices[edge_b.start], b.vertices[edge_b.end]);
    auto depth = (world_b - world_a).dot(normal);

    if (depth < 0.0 * si::metre)
    {
        world_b = world_a + normal * fallback_depth;
        depth = fallback_depth;
    }

    return collision_info{
        .normal = normal,
        .world_a = world_a,
        .world_b = world_b,
        .depth = depth,
    };
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
inline std::optional<collision_info> sat_polyhedron_collision(const sat_polyhedron &a,
                                                              const sat_polyhedron &b)
{
    constexpr auto eps_axis2 = 1e-12;
    auto min_overlap = std::numeric_limits<quantity<si::metre>>::infinity();
    vec3<one> best_axis{1.0, 0.0, 0.0};
    sat_axis best{};
    const auto t_world = b.center - a.center;

    auto test_axis = [&](const vec3<one> &axis_world, sat_axis candidate) -> bool
    {
        const auto sn = axis_world.squared_norm();
        if (sn < eps_axis2) return true;
        const auto n = axis_world / mp_units::sqrt(sn);

        const auto [min_a, max_a] = sat_project(a, n);
        const auto [min_b, max_b] = sat_project(b, n);
        const auto overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
        if (overlap < 0.0 * si::metre) return false;

        if (overlap < min_overlap)
        {
            min_overlap = overlap;
            best_axis = (t_world.dot(n) > 0.0 * si::metre) ? -n : n;
            best = candidate;
        }
        return true;
    };

    for (std::size_t i = 0; i < a.faces.size(); ++i)
        if (!test_axis(a.faces[i].normal, {.a_index = i, .feature = sat_feature::face_a}))
            return std::nullopt;
    for (std::size_t i = 0; i < b.faces.size(); ++i)
        if (!test_axis(b.faces[i].normal, {.b_index = i, .feature = sat_feature::face_b}))
            return std::nullopt;
    for (std::size_t i = 0; i < a.edge_directions.size(); ++i)
        for (std::size_t j = 0; j < b.edge_directions.size(); ++j)
            if (!test_axis(a.edge_directions[i].cross(b.edge_directions[j]),
                           {.a_index = i, .b_index = j, .feature = sat_feature::edge}))
                return std::nullopt;

    switch (best.feature)
    {
    case sat_feature::face_a:
        return sat_clipped_face_contact(a, b, best_axis, min_overlap, true);
    case sat_feature::face_b:
        return sat_clipped_face_contact(a, b, best_axis, min_overlap, false);
    case sat_feature::edge:
        return sat_edge_contact(a, b, best_axis, min_overlap, best.a_index, best.b_index);
    }

    std::unreachable();
}

std::optional<collision_info> box_box(const instance &a, const instance &b)
{ return sat_polyhedron_collision(make_box_polyhedron(a), make_box_polyhedron(b)); }
std::optional<collision_info> box_cylinder(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> box_cone(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> box_pyramid(const instance &a, const instance &b)
{ return sat_polyhedron_collision(make_box_polyhedron(a), make_pyramid_polyhedron(b)); }
std::optional<collision_info> box_mesh(const instance &a, const instance &b)
{ return gjk_epa(a, b); }

std::optional<collision_info> sphere_cylinder(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> sphere_cone(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> sphere_pyramid(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> sphere_mesh(const instance &a, const instance &b)
{ return gjk_epa(a, b); }

std::optional<collision_info> cylinder_cylinder(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> cylinder_cone(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> cylinder_pyramid(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> cylinder_mesh(const instance &a, const instance &b)
{ return gjk_epa(a, b); }

std::optional<collision_info> cone_cone(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> cone_pyramid(const instance &a, const instance &b)
{ return gjk_epa(a, b); }
std::optional<collision_info> cone_mesh(const instance &a, const instance &b)
{ return gjk_epa(a, b); }

std::optional<collision_info> pyramid_pyramid(const instance &a, const instance &b)
{ return sat_polyhedron_collision(make_pyramid_polyhedron(a), make_pyramid_polyhedron(b)); }
std::optional<collision_info> pyramid_mesh(const instance &a, const instance &b)
{ return gjk_epa(a, b); }

std::optional<collision_info> mesh_mesh(const instance &a, const instance &b)
{ return gjk_epa(a, b); }

DEFINE_MIRROR_IMPL(box, sphere)
DEFINE_MIRROR_IMPL(box, cylinder)
DEFINE_MIRROR_IMPL(box, cone)
DEFINE_MIRROR_IMPL(box, pyramid)
DEFINE_MIRROR_IMPL(box, mesh)

DEFINE_MIRROR_IMPL(sphere, cylinder)
DEFINE_MIRROR_IMPL(sphere, cone)
DEFINE_MIRROR_IMPL(sphere, pyramid)
DEFINE_MIRROR_IMPL(sphere, mesh)

DEFINE_MIRROR_IMPL(cylinder, cone)
DEFINE_MIRROR_IMPL(cylinder, pyramid)
DEFINE_MIRROR_IMPL(cylinder, mesh)

DEFINE_MIRROR_IMPL(cone, pyramid)
DEFINE_MIRROR_IMPL(cone, mesh)

DEFINE_MIRROR_IMPL(pyramid, mesh)

#undef DEFINE_MIRROR_IMPL

static constexpr auto collision_map = std::array{
    std::array{box_box, box_sphere, box_cylinder, box_cone, box_pyramid, box_mesh},
    std::array{sphere_box, sphere_sphere, sphere_cylinder, sphere_cone, sphere_pyramid,
               sphere_mesh},
    std::array{cylinder_box, cylinder_sphere, cylinder_cylinder, cylinder_cone, cylinder_pyramid,
               cylinder_mesh},
    std::array{cone_box, cone_sphere, cone_cylinder, cone_cone, cone_pyramid, cone_mesh},
    std::array{pyramid_box, pyramid_sphere, pyramid_cylinder, pyramid_cone, pyramid_pyramid,
               pyramid_mesh},
    std::array{mesh_box, mesh_sphere, mesh_cylinder, mesh_cone, mesh_pyramid, mesh_mesh},
};

std::optional<collision_info> collision(const physkit::instance &a, const physkit::instance &b)
{
    return collision_map[static_cast<std::size_t>(a.geometry().stored_type())]
                        [static_cast<std::size_t>(b.geometry().stored_type())](a, b);
}
} // namespace physkit
