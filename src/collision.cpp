#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/collision/collision.h"
#include <absl/container/inlined_vector.h>
#endif

namespace physkit
{
/// based off winter dev gjk algorithm implementation

using simplex = absl::InlinedVector<detail::support_pt, 4>;

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
std::optional<simplex> gjk_collision(const detail::SupportShape auto &a,
                                     const detail::SupportShape auto &b)
{
    constexpr auto eps = 1e-12;
    simplex simplex;
    vec3<one> direction = {1.0, 0.0, 0.0};

    auto point = detail::minkowski_support(a, b, direction);
    simplex.push_back(point);
    if (point.p.squared_norm() < eps * pow<2>(si::metre)) return simplex;
    direction = -point.p.normalized();

    constexpr int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        auto new_point = detail::minkowski_support(a, b, direction);
        auto progress = new_point.p.dot(direction);
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

        if ((p2.p - simplex[0].p).squared_norm() < 1e-6 * pow<2>(si::metre))
            p2 = detail::minkowski_support(a, b, -dir);
        simplex.push_back(p2);
    };
    auto on_2 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto line = (simplex[1].p - simplex[0].p);
        auto dir = line.normalized().cross(vec3<one>{0.0, 1.0, 0.0});
        if (dir.squared_norm() < 1e-6) dir = line.normalized().cross(vec3<one>{0.0, 0.0, 1.0});
        dir.normalize();
        auto p3 = detail::minkowski_support(a, b, dir);
        if (line.cross(p3.p - simplex[0].p).squared_norm() < 1e-6 * pow<4>(si::metre))
            p3 = detail::minkowski_support(a, b, -dir);
        simplex.push_back(p3);
    };
    auto on_3 = [](const a_type &a, const b_type &b, physkit::simplex &simplex)
    {
        auto ab = simplex[1].p - simplex[0].p;
        auto ac = simplex[2].p - simplex[0].p;
        auto dir = ab.cross(ac).normalized();
        auto p4 = detail::minkowski_support(a, b, dir);
        if (abs((p4.p - simplex[0].p).dot(dir)) < 1e-6 * si::metre)
            p4 = detail::minkowski_support(a, b, -dir);
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

        if (face.normal.squared_norm() < 1e-12)
            face.normal = vec3<one>::zero();
        else
            face.normal.normalize();

        if (opposite_index != null_index &&
            face.normal.dot(polytope[opposite_index].p - polytope[i].p) > 0.0 * si::metre)
        {
            std::swap(face.vertices[1], face.vertices[2]);
            face.normal = -face.normal;
        }

        face.distance = face.normal.dot(polytope[i].p);
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
    static std::optional<collision_info> solve(const detail::SupportShape auto &a,
                                               const detail::SupportShape auto &b, simplex &simplex)
    {
        if (simplex.size() < 4 && !pad_simplex(a, b, simplex))
            return std::nullopt; // Degenerate case, treat as no collision

        epa_solver solver{.polytope = {simplex.begin(), simplex.end()}};

        solver.build_initial_tetrahedron();

        constexpr int max_iterations = 64;
        constexpr auto tolerance = 1e-6 * si::metre;

        auto get_barycentric = [&](const face &f, const detail::support_pt &p)
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

            const auto &min_face = solver.faces[min_face_idx];

            auto p = detail::minkowski_support(a, b, min_face.normal);
            auto p_dist = min_face.normal.dot(p.p);
            if (p_dist - min_face.distance < tolerance) // convergence
                return get_barycentric(min_face, p);

            auto horizon = solver.find_silhouette(min_face_idx, p.p);
            if (horizon.empty()) break;

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

        // return best guess if not converged
        std::size_t min_face_idx = solver.pop_face();
        if (min_face_idx == null_index) return std::nullopt;
        return get_barycentric(solver.faces[min_face_idx], solver.polytope.back());
    }

    absl::InlinedVector<face, buffer_size> faces;
    absl::InlinedVector<detail::support_pt, buffer_size> polytope;
    absl::InlinedVector<std::size_t, buffer_size> face_heap;
};

/// @brief return data for obb obb collision
std::optional<collision_info> gjk_epa(detail::SupportShape auto const &a,
                                      detail::SupportShape auto const &b)
{
    auto simplex = gjk_collision(a, b);
    if (simplex) return epa_solver::solve(a, b, *simplex);
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
INSTANTIATE_GJK_EPA(physkit::instance, physkit::instance)
INSTANTIATE_GJK_EPA(aabb, mesh::instance)
INSTANTIATE_GJK_EPA(mesh::instance, aabb)
INSTANTIATE_GJK_EPA(obb, mesh::instance)
INSTANTIATE_GJK_EPA(mesh::instance, obb)

#undef INSTANTIATE_GJK_EPA

std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b)
{
    // could be made faster if unique edges were stored in mesh.
    // SAT must be performed on 2 convex meshes.

    // need to compare projected intersection across every axis of the follosing types
    // 1. The normal of every face from both meshes.
    // 2. The cross product of every edge from mesh A with every edge from mesh B

    // can optimize by removing parallel axes

    constexpr auto eps = 1e-12;

    auto a_tris = a.geometry().triangles();
    auto b_tris = b.geometry().triangles();
    auto sum_triangle_count = a_tris.size() + b_tris.size();

    // the variables related to edge count assume the following:
    //  Polyhedra are convex
    //  All faces are triangles
    //  Edges are manifold (each edge belongs to exactly 2 faces)
    //  all verticies of every triangle are ordered CCW such that the norm generated using vertex 0
    //      as the base points away from the center

    // The edge count for mesh A
    auto a_edge_count = (a_tris.size() * 3) / 2;
    // The edge count for mesh B
    auto b_edge_count = (b_tris.size() * 3) / 2;
    // The total number of unique edges in both meshes.
    auto sum_edge_count = ((sum_triangle_count * 3) / 2);
    auto sum_vertex_count = ((sum_triangle_count * 3) / 2);
    // The maximum possible number of separating axes.
    auto max_axes = (a_edge_count * b_edge_count) + sum_triangle_count;

    auto a_vertices = a.geometry().vertices(); // std::span<const vec3<si::metre>>
    auto b_vertices = b.geometry().vertices();

    // extra collision info
    auto info = collision_info();
    info.depth = quantity<si::metre>::max();

    // returns a pair of the min and max value of a a set of verticies projected along an axis.
    auto project_mesh = [](auto const &axis, auto const &vertices)
    {
        // the divide by |axis| can be omitted from the difference
        // the axis's units are si::metre^2
        auto minv = vertices[0];
        auto maxv = minv;
        auto minc = axis.dot(minv);
        auto maxc = minc;
        for (size_t i = 1; i < vertices.size(); ++i)
        {
            auto p = axis.dot(vertices[i]);
            if (p < minc)
            {
                minv = vertices[i];
                minc = p;
            }
            else if (p > maxc)
            {
                maxv = vertices[i];
                maxc = p;
            }
        }
        return std::tuple{minv, maxv, minc, maxc};
    };

    auto test_axis = [&](const vec3<one> &axis)
    {
        auto [a_minv, a_maxv, a_minc, a_maxc] = project_mesh(axis, a_vertices);
        auto [b_minv, b_maxv, b_minc, b_maxc] = project_mesh(axis, b_vertices);

        // checks if the axes have collision
        auto overlap_unnormal = (std::min(a_maxc, b_maxc) - std::max(a_minc, b_minc));

        if (overlap_unnormal <= 0 * si::metre) return false; // no collision

        if (overlap_unnormal < info.depth)
        {
            // new minimum overlap
            info.depth = overlap_unnormal; // this is not the actual depth until it is normalized at
                                           // the end. lazy normalization.
            info.normal = axis;            // this is not the actual normal yet either
            if (a_maxc > b_maxc)
            {
                info.world_a = a_minv;
                info.world_b = b_maxv;
            }
            else
            {
                info.world_a = a_maxv;
                info.world_b = b_minv;
            }
        };
        return true;
    };

    // if unique edges were stored in mesh this would be 4 times more efficient.
    for (const auto &a_tri : a_tris)
    {
        auto a_ver = a_tri.vertices(a);
        std::array<vec3<si::metre>, 3> a_edges = {(a_ver[1] - a_ver[0]), (a_ver[2] - a_ver[1]),
                                                  (a_ver[0] - a_ver[2])};

        // Face axis
        auto n = (a_edges[0]).cross(a_edges[1]) *
                 (1 / si::metre / si::metre); // si::metre^2 -> unitless direction
        if (!test_axis(n)) return std::nullopt;

        for (const auto &b_tri : b_tris)
        {
            auto b_ver = b_tri.vertices(b);
            std::array<vec3<si::metre>, 3> b_edges = {(b_ver[1] - b_ver[0]), (b_ver[2] - b_ver[1]),
                                                      (b_ver[0] - b_ver[2])};

            // Face axis
            if (a_tri == a_tris.front())
            { // once per tri
                n = (b_edges[0]).cross(b_edges[1]) *
                    (1 / si::metre / si::metre); // si::metre^2 -> unitless direction
                if (!test_axis(n)) return std::nullopt;
            }

            // edge cross axes
            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    n = (a_edges[i]).cross(b_edges[j]) *
                        (1 / si::metre / si::metre); // si::metre^2 -> unitless direction

                    if (n.squared_norm() < eps) continue; // near 0 axis.
                    if (!test_axis(n)) return std::nullopt;
                }
            }
        }
    }

    info.depth /= info.normal.norm();
    info.normal = info.normal.normalized();
    return info;
}
} // namespace physkit