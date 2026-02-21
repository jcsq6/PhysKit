#include "physkit/mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

namespace bvh
{

constexpr unsigned int sah_bins = 16;
constexpr unsigned int leaf_max_tris = 4;

auto triangle_aabb(std::span<const vec3<m>> verts, const mesh::triangle_t &tri)
{
    const auto &a = verts[tri[0]];
    const auto &b = verts[tri[1]];
    const auto &c = verts[tri[2]];
    return aabb{
        .min = {std::min({a.x(), b.x(), c.x()}), std::min({a.y(), b.y(), c.y()}),
                std::min({a.z(), b.z(), c.z()})},
        .max = {std::max({a.x(), b.x(), c.x()}), std::max({a.y(), b.y(), c.y()}),
                std::max({a.z(), b.z(), c.z()})},
    };
}

auto triangle_centroid(std::span<const vec3<m>> verts, const mesh::triangle_t &tri)
{
    return (verts[tri[0]] + verts[tri[1]] + verts[tri[2]]) / 3.0;
}

auto aabb_union(const aabb &a, const aabb &b)
{
    return aabb{
        .min = {std::min(a.min.x(), b.min.x()), std::min(a.min.y(), b.min.y()),
                std::min(a.min.z(), b.min.z())},
        .max = {std::max(a.max.x(), b.max.x()), std::max(a.max.y(), b.max.y()),
                std::max(a.max.z(), b.max.z())},
    };
}

struct builder
{
    constexpr static double cost_traversal = 1.0;
    constexpr static double cost_intersect = 1.0;

    builder(std::span<const vec3<m>> vertices, std::span<mesh::triangle_t> triangles,
            std::vector<detail::bvh_node> &nodes)
        : vertices(vertices), triangles(triangles), nodes(nodes), centroids(triangles.size()),
          tri_bounds(triangles.size()), indices(triangles.size())
    {
        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            centroids[i] = triangle_centroid(vertices, triangles[i]);
            tri_bounds[i] = triangle_aabb(vertices, triangles[i]);
        }
        std::ranges::iota(indices, std::uint32_t{0});

        nodes.clear();
        nodes.reserve(2 * triangles.size());

        auto root_bounds = tri_bounds[0];
        for (std::size_t i = 1; i < triangles.size(); ++i)
            root_bounds = aabb_union(root_bounds, tri_bounds[i]);

        build_recursive(0, triangles.size(), root_bounds);

        std::vector<mesh::triangle_t> reordered(triangles.size());
        for (std::size_t i = 0; i < indices.size(); ++i) reordered[i] = triangles[indices[i]];
        std::ranges::copy(reordered, triangles.begin());
    }

    // NOTE: can be parallelized if needed. Evaluate cost for large meshes.
    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    void build_recursive(std::size_t start, std::size_t end, const aabb &bounds)
    {
        auto node_idx = nodes.size();
        nodes.push_back({});

        auto count = end - start;

        // Make leaf if few enough triangles
        if (count <= leaf_max_tris)
        {
            nodes[node_idx] = {.bounds = bounds,
                               .start = static_cast<std::uint16_t>(start),
                               .count = static_cast<std::uint16_t>(count),
                               .axis = 0};
            return;
        }

        // Find best SAH split across all 3 axes
        double best_cost = std::numeric_limits<double>::max();
        unsigned int best_axis = -1;
        unsigned int best_split = -1;
        aabb best_left_bounds{};
        aabb best_right_bounds{};
        auto parent_area = bounds.surface_area();

        struct bin_t
        {
            aabb bounds;
            std::uint32_t count = 0;
            bool empty = true;
        };

        for (int axis = 0; axis < 3; ++axis)
        {
            // Find centroid bounds on this axis
            auto cmin = centroids[indices[start]][axis];
            auto cmax = cmin;
            for (auto i = start + 1; i < end; ++i)
            {
                cmin = std::min(cmin, centroids[indices[i]][axis]);
                cmax = std::max(cmax, centroids[indices[i]][axis]);
            }

            if (cmax - cmin < 1e-12 * m) continue; // degenerate axis

            auto extent = cmax - cmin;

            // Fill bins
            std::array<bin_t, sah_bins> bins{};
            for (auto i = start; i < end; ++i)
            {
                auto idx = indices[i];
                auto frac = (centroids[idx][axis] - cmin) / extent;
                auto b_idx = std::min(static_cast<unsigned int>(frac * sah_bins), sah_bins - 1);
                if (bins[b_idx].empty)
                {
                    bins[b_idx].bounds = tri_bounds[idx];
                    bins[b_idx].empty = false;
                }
                else
                    bins[b_idx].bounds = aabb_union(bins[b_idx].bounds, tri_bounds[idx]);
                bins[b_idx].count++;
            }

            // Sweep from left: compute prefix areas, counts, and bounds
            std::array<quantity<pow<2>(m)>, sah_bins - 1> left_area{};
            std::array<std::uint32_t, sah_bins - 1> left_count{};
            std::array<aabb, sah_bins - 1> left_bounds_arr{};

            aabb left_bounds{};
            bool left_started = false;
            std::uint32_t left_n = 0;
            for (unsigned int i = 0; i < sah_bins - 1; ++i)
            {
                if (!bins[i].empty)
                {
                    left_bounds =
                        left_started ? aabb_union(left_bounds, bins[i].bounds) : bins[i].bounds;
                    left_started = true;
                }
                left_n += bins[i].count;
                left_count[i] = left_n;
                left_area[i] = left_started ? left_bounds.surface_area() : 0.0 * m * m;
                left_bounds_arr[i] = left_bounds;
            }

            // Sweep from right
            aabb right_bounds{};
            bool right_started = false;
            std::uint32_t right_n = 0;
            for (unsigned int i = sah_bins - 1; i >= 1; --i)
            {
                if (!bins[i].empty)
                {
                    right_bounds =
                        right_started ? aabb_union(right_bounds, bins[i].bounds) : bins[i].bounds;
                    right_started = true;
                }
                right_n += bins[i].count;
                auto right_area = right_started ? right_bounds.surface_area() : 0.0 * m * m;
                auto split_idx = i - 1;
                auto cost =
                    cost_traversal +
                    cost_intersect *
                        ((left_area[split_idx] * left_count[split_idx] + right_area * right_n) /
                         parent_area);

                if (cost < best_cost)
                {
                    best_cost = static_cast<double>(cost);
                    best_axis = axis;
                    best_split = i;
                    best_left_bounds = left_bounds_arr[split_idx];
                    best_right_bounds = right_bounds;
                }
            }
        }

        auto leaf_cost = cost_intersect * static_cast<double>(count);
        if (best_axis == -1 || best_cost >= leaf_cost)
        {
            nodes[node_idx] = {.bounds = bounds,
                               .start = static_cast<std::uint16_t>(start),
                               .count = static_cast<std::uint16_t>(count),
                               .axis = 0};
            return;
        }

        // Partition the index array around the best split
        auto cmin = centroids[indices[start]][best_axis];
        auto cmax = cmin;
        for (auto i = start + 1; i < end; ++i)
        {
            cmin = std::min(cmin, centroids[indices[i]][best_axis]);
            cmax = std::max(cmax, centroids[indices[i]][best_axis]);
        }
        auto part_extent = cmax - cmin;

        auto mid = start;
        for (auto i = start; i < end; ++i)
        {
            auto frac = (centroids[indices[i]][best_axis] - cmin) / part_extent;
            auto b_idx = std::min(static_cast<unsigned int>(frac * sah_bins), sah_bins - 1);
            if (b_idx < best_split)
            {
                std::swap(indices[i], indices[mid]);
                ++mid;
            }
        }

        // Fallback to median if partition is degenerate
        if (mid == start || mid == end)
        {
            mid = (start + end) / 2;
            auto cmp = [this, best_axis](std::uint32_t a, std::uint32_t b)
            { return centroids[a][best_axis] < centroids[b][best_axis]; };

            std::ranges::nth_element(indices.begin() + static_cast<std::ptrdiff_t>(start),
                                     indices.begin() + static_cast<std::ptrdiff_t>(mid),
                                     indices.begin() + static_cast<std::ptrdiff_t>(end), cmp);

            best_left_bounds = tri_bounds[indices[start]];
            for (auto i = start + 1; i < mid; ++i)
                best_left_bounds = aabb_union(best_left_bounds, tri_bounds[indices[i]]);
            best_right_bounds = tri_bounds[indices[mid]];
            for (auto i = mid + 1; i < end; ++i)
                best_right_bounds = aabb_union(best_right_bounds, tri_bounds[indices[i]]);
        }

        nodes[node_idx].bounds = bounds;
        nodes[node_idx].count = 0; // internal node
        nodes[node_idx].axis = static_cast<std::uint16_t>(best_axis);

        // Left child is always node_idx + 1 (DFS order)
        build_recursive(start, mid, best_left_bounds);

        // Right child: record its index
        nodes[node_idx].start = static_cast<std::uint32_t>(nodes.size());
        build_recursive(mid, end, best_right_bounds);
    }

    std::span<const vec3<m>> vertices;
    std::span<mesh::triangle_t> triangles;
    std::vector<vec3<m>> centroids;
    std::vector<aabb> tri_bounds;
    std::vector<std::uint32_t> indices;
    std::vector<detail::bvh_node> &nodes; // NOLINT
};

// Slab-based ray-AABB intersection test.
// When a ray direction component is zero, inv_dir is +-inf and the slab products can be
// 0*inf = NaN. Replace NaN with the correct limit: if the origin is between the slabs
// (or on a boundary), that axis is unconstrained [-inf, +inf]; if outside, it's empty.
// TODO: test if mp-units is stopping us from using SIMD here.
auto ray_aabb_intersect(const aabb &box, const vec3<m> &origin, const vec3<one> &inv_dir,
                        quantity<m> best_t)
{
    auto slab = [](double lo, double hi)
    {
        if (std::isnan(lo)) lo = -std::numeric_limits<double>::infinity();
        if (std::isnan(hi)) hi = std::numeric_limits<double>::infinity();
        return std::pair{std::fmin(lo, hi), std::fmax(lo, hi)};
    };

    auto [tminx, tmaxx] = slab(((box.min.x() - origin.x()) * inv_dir.x()).numerical_value_in(m),
                               ((box.max.x() - origin.x()) * inv_dir.x()).numerical_value_in(m));
    auto [tminy, tmaxy] = slab(((box.min.y() - origin.y()) * inv_dir.y()).numerical_value_in(m),
                               ((box.max.y() - origin.y()) * inv_dir.y()).numerical_value_in(m));
    auto [tminz, tmaxz] = slab(((box.min.z() - origin.z()) * inv_dir.z()).numerical_value_in(m),
                               ((box.max.z() - origin.z()) * inv_dir.z()).numerical_value_in(m));

    auto tmin = std::max({tminx, tminy, tminz});
    auto tmax = std::min({tmaxx, tmaxy, tmaxz});

    return tmax >= 0.0 && tmin <= tmax && tmin <= best_t.numerical_value_in(m);
}

// Squared distance from a point to the nearest point on an AABB
auto squared_distance_to_aabb(const aabb &box, const vec3<m> &point)
{
    auto dx = std::max({box.min.x() - point.x(), 0.0 * m, point.x() - box.max.x()});
    auto dy = std::max({box.min.y() - point.y(), 0.0 * m, point.y() - box.max.y()});
    auto dz = std::max({box.min.z() - point.z(), 0.0 * m, point.z() - box.max.z()});
    return dx * dx + dy * dy + dz * dz;
}

} // namespace bvh

void mesh::build_bvh() { bvh::builder{M_vertices, M_triangles, M_bvh_nodes}; }

/*
----------------------------------------------
-------------------- MESH --------------------
----------------------------------------------
*/

std::shared_ptr<mesh> mesh::make(std::span<const vec3<m>> vertices,
                                 std::span<const triangle_t> triangles)
{
    assert(!vertices.empty());
    assert(!triangles.empty());

    return std::make_shared<mesh>(vertices, triangles, key{});
}

std::shared_ptr<mesh> mesh::box(const vec3<m> &half_extents)
{
    auto hx = half_extents.x();
    auto hy = half_extents.y();
    auto hz = half_extents.z();

    std::array vertices{
        vec3{-hx, -hy, -hz}, // 0
        vec3{+hx, -hy, -hz}, // 1
        vec3{+hx, +hy, -hz}, // 2
        vec3{-hx, +hy, -hz}, // 3
        vec3{-hx, -hy, +hz}, // 4
        vec3{+hx, -hy, +hz}, // 5
        vec3{+hx, +hy, +hz}, // 6
        vec3{-hx, +hy, +hz}, // 7
    };

    std::array<triangle_t, 12> triangles{{
        {4, 5, 6},
        {4, 6, 7}, // +z
        {0, 3, 2},
        {0, 2, 1}, // -z
        {1, 2, 6},
        {1, 6, 5}, // +x
        {0, 4, 7},
        {0, 7, 3}, // -x
        {3, 7, 6},
        {3, 6, 2}, // +y
        {0, 1, 5},
        {0, 5, 4}, // -y
    }};

    return make(vertices, triangles);
}

std::shared_ptr<mesh> mesh::sphere(quantity<m> radius, unsigned int stacks, unsigned int sectors)
{
    assert(stacks >= 2);
    assert(sectors >= 3);

    std::vector<vec3<m>> vertices;
    std::vector<triangle_t> triangles;

    vertices.reserve(2 + ((stacks - 1) * sectors));
    triangles.reserve(2ULL * sectors * (stacks - 1));

    // North pole
    vertices.emplace_back(0.0 * m, radius, 0.0 * m);

    // Intermediate rings
    for (unsigned int i = 1; i < stacks; ++i)
    {
        auto phi = std::numbers::pi * static_cast<double>(i) / static_cast<double>(stacks);
        for (unsigned int j = 0; j < sectors; ++j)
        {
            auto theta =
                2.0 * std::numbers::pi * static_cast<double>(j) / static_cast<double>(sectors);
            vertices.emplace_back(radius * std::sin(phi) * std::cos(theta), radius * std::cos(phi),
                                  radius * std::sin(phi) * std::sin(theta));
        }
    }

    // South pole
    vertices.emplace_back(0.0 * m, -radius, 0.0 * m);
    auto south_pole = static_cast<unsigned int>(vertices.size() - 1);

    // North cap
    for (unsigned int j = 0; j < sectors; ++j)
        triangles.push_back({0, 1 + ((j + 1) % sectors), 1 + j});

    // Body bands
    for (unsigned int i = 1; i < stacks - 1; ++i)
    {
        for (unsigned int j = 0; j < sectors; ++j)
        {
            auto top = 1 + ((i - 1) * sectors) + j;
            auto top_next = 1 + ((i - 1) * sectors) + ((j + 1) % sectors);
            auto bot = 1 + (i * sectors) + j;
            auto bot_next = 1 + (i * sectors) + ((j + 1) % sectors);
            triangles.push_back({top, top_next, bot_next});
            triangles.push_back({top, bot_next, bot});
        }
    }

    // South cap
    auto last_ring = 1 + ((stacks - 2) * sectors);
    for (unsigned int j = 0; j < sectors; ++j)
        triangles.push_back({south_pole, last_ring + j, last_ring + ((j + 1) % sectors)});

    return make(vertices, triangles);
}

std::shared_ptr<mesh> mesh::pyramid(quantity<m> base_half, quantity<m> height)
{
    std::array vertices{
        vec3{-base_half, 0.0 * m, -base_half}, // 0
        vec3{+base_half, 0.0 * m, -base_half}, // 1
        vec3{+base_half, 0.0 * m, +base_half}, // 2
        vec3{-base_half, 0.0 * m, +base_half}, // 3
        vec3{0.0 * m, height, 0.0 * m},        // 4 (apex)
    };

    std::array<triangle_t, 6> triangles{{
        {0, 1, 2},
        {0, 2, 3}, // base (-y)
        {1, 0, 4}, // front (-z)
        {2, 1, 4}, // right (+x)
        {3, 2, 4}, // back  (+z)
        {0, 3, 4}, // left  (-x)
    }};

    return make(vertices, triangles);
}

quantity<pow<3>(m)> mesh::volume() const
{
    // Signed volume via sum of signed tetrahedra.
    auto vol = 0.0 * m * m * m;
    for (const auto &tri : M_triangles)
    {
        const auto &v0 = M_vertices[tri[0]];
        const auto &v1 = M_vertices[tri[1]];
        const auto &v2 = M_vertices[tri[2]];
        vol += v0.dot(v1.cross(v2));
    }
    return vol / 6.0;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
vec3<m> mesh::mass_center() const
{
    // TODO: implement volume-weighted centroid via divergence theorem
    throw std::runtime_error("mesh::mass_center not yet implemented");
}

mat3<kg * pow<2>(m)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
mesh::inertia_tensor(quantity<kg / pow<3>(m)> /*density*/) const
{
    // TODO: implement inertia tensor via canonical tetrahedron integrals
    throw std::runtime_error("mesh::inertia_tensor not yet implemented");
}

// Moeller-Trumbore ray-triangle test for a single triangle
// TODO: benchmark to see if std::optional is causing overhead
static std::optional<mesh::ray_hit> moeller_trumbore(const vec3<m> &origin, const vec3<one> &dir,
                                                     const std::array<vec3<m>, 3> &vertices,
                                                     quantity<m> max_dist)
{
    constexpr auto eps = 1e-12;

    auto edge1 = vertices[1] - vertices[0];
    auto edge2 = vertices[2] - vertices[0];
    auto h = dir.cross(edge2);
    auto a = edge1.dot(h);

    if (a > -eps * m * m && a < eps * m * m) return std::nullopt;

    auto f = 1.0 / a;
    auto s = origin - vertices[0];
    auto u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return std::nullopt;

    auto q = s.cross(edge1);
    auto v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return std::nullopt;

    auto t = f * edge2.dot(q);
    if (t < 0.0 * m || t > max_dist) return std::nullopt;

    return mesh::ray_hit{
        .pos = origin + dir * t,
        .normal = edge1.cross(edge2).normalized(),
        .distance = t,
    };
}

static constexpr std::size_t bvh_stack_size = 128;
static constexpr auto safe_inv(const vec3<one> &vec)
{
    if constexpr (std::numeric_limits<double>::is_iec559)
    {
        return vec3{1.0 / static_cast<double>(vec.x()), 1.0 / static_cast<double>(vec.y()),
                    1.0 / static_cast<double>(vec.z())};
    }
    else
    {
        auto safe_component = [](auto v)
        {
            return v == 0.0 ? std::copysign(std::numeric_limits<double>::infinity(),
                                            static_cast<double>(v))
                            : 1.0 / v;
        };
        return vec3{safe_component(vec.x()), safe_component(vec.y()), safe_component(vec.z())};
    }
}

// BVH-accelerated ray intersection
std::optional<mesh::ray_hit> mesh::ray_intersect(const ray &r, quantity<m> max_distance) const
{
    assert(!M_bvh_nodes.empty());

    auto dir = r.direction.normalized();
    auto inv_dir = safe_inv(dir);
    auto best_t = max_distance;
    std::optional<ray_hit> best;

    std::array<std::uint32_t, bvh_stack_size> stack{};
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0)
    {
        auto node_idx = stack[--sp];
        const auto &node = M_bvh_nodes[node_idx];

        if (!bvh::ray_aabb_intersect(node.bounds, r.origin, inv_dir, best_t)) continue;

        if (node.count > 0) // leaf
        {
            for (auto i = node.start; i < node.start + node.count; ++i)
                if (auto hit =
                        moeller_trumbore(r.origin, dir, M_triangles[i].vertices(*this), best_t))
                {
                    best_t = hit->distance;
                    best = hit;
                }
        }
        else // internal
        {
            auto left = node_idx + 1;
            auto right = node.start;

            if (stack.size() < sp + 2)
                throw std::runtime_error("BVH stack overflow: increase bvh_stack_size");

            if (dir[node.axis] < 0.0)
            {
                stack[sp++] = left;
                stack[sp++] = right;
            }
            else
            {
                stack[sp++] = right;
                stack[sp++] = left;
            }
        }
    }

    return best;
}

// Closest point on a single triangle to a query point, using barycentric coordinates.
// TODO: benchmark against other methods (e.g. Voronoi region tests) to see if this is a bottleneck.
static vec3<m> closest_point_on_triangle(const vec3<m> &p, const vec3<m> &a, const vec3<m> &b,
                                         const vec3<m> &c)
{
    auto ab = b - a;
    auto ac = c - a;
    auto ap = p - a;

    auto d00 = ab.dot(ab);
    auto d01 = ab.dot(ac);
    auto d11 = ac.dot(ac);
    auto d20 = ap.dot(ab);
    auto d21 = ap.dot(ac);

    auto denom = d00 * d11 - d01 * d01;
    if (abs(denom) < 1e-12 * pow<4>(m)) return a; // degenerate triangle, return a vertex

    auto v = (d11 * d20 - d01 * d21) / denom;
    auto w = (d00 * d21 - d01 * d20) / denom;
    auto u = 1.0 - v - w;

    if (u >= 0.0 && v >= 0.0 && w >= 0.0) return u * a + v * b + w * c;

    // Point projects outside the triangle - project onto each edge and pick closest
    auto candidate = a;
    auto candidate_dist_sq = (candidate - p).squared_norm();
    auto check_edge = [&p, &candidate, &candidate_dist_sq](const vec3<m> &e0, const vec3<m> &e1)
    {
        auto edge = e1 - e0;
        auto t =
            std::clamp(static_cast<double>((p - e0).dot(edge) / edge.squared_norm()), 0.0, 1.0);
        auto attempt = e0 + t * edge;
        if (auto attempt_dist_sq = (attempt - p).squared_norm();
            attempt_dist_sq < candidate_dist_sq)
        {
            candidate = attempt;
            candidate_dist_sq = attempt_dist_sq;
        }
    };
    check_edge(a, b);
    check_edge(b, c);
    check_edge(c, a);
    return candidate;
}

// BVH-accelerated closest point on mesh surface
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
vec3<m> mesh::closest_point(const vec3<m> &p) const
{
    assert(!M_bvh_nodes.empty() && "empty mesh");

    auto best_dist_sq = std::numeric_limits<double>::max() * m * m;
    auto closest = M_vertices[0];

    std::array<std::uint32_t, bvh_stack_size> stack{};
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0)
    {
        auto node_idx = stack[--sp];
        const auto &node = M_bvh_nodes[node_idx];

        if (bvh::squared_distance_to_aabb(node.bounds, p) >= best_dist_sq) continue;

        if (node.count > 0) // leaf
        {
            for (auto i = node.start; i < node.start + node.count; ++i)
            {
                const auto &tri = M_triangles[i];
                auto candidate = closest_point_on_triangle(p, M_vertices[tri[0]],
                                                           M_vertices[tri[1]], M_vertices[tri[2]]);
                auto d_sq = (candidate - p).squared_norm();
                if (d_sq < best_dist_sq)
                {
                    best_dist_sq = d_sq;
                    closest = candidate;
                }
            }
        }
        else // internal
        {
            auto left = node_idx + 1;
            auto right = node.start;

            auto d_left = bvh::squared_distance_to_aabb(M_bvh_nodes[left].bounds, p);
            auto d_right = bvh::squared_distance_to_aabb(M_bvh_nodes[right].bounds, p);

            if (stack.size() < sp + 2)
                throw std::runtime_error("BVH stack overflow: increase bvh_stack_size");

            // Push far child first so near child is popped first
            if (d_left <= d_right)
            {
                if (d_right < best_dist_sq) stack[sp++] = right;
                if (d_left < best_dist_sq) stack[sp++] = left;
            }
            else
            {
                if (d_left < best_dist_sq) stack[sp++] = left;
                if (d_right < best_dist_sq) stack[sp++] = right;
            }
        }
    }

    return closest;
}

// Ray-triangle intersection for containment parity counting.
// Returns {t, on_edge} where on_edge is true if the hit is near a triangle edge/vertex.
struct parity_hit
{
    quantity<m> t;
    bool on_edge;
};

static std::optional<parity_hit> ray_triangle_parity(const vec3<m> &origin, const vec3<one> &dir,
                                                     const std::array<vec3<m>, 3> &vertices)
{
    constexpr auto eps = 1e-12;
    constexpr auto edge_eps = 1e-10; // threshold for "on an edge"

    auto edge1 = vertices[1] - vertices[0];
    auto edge2 = vertices[2] - vertices[0];
    auto h = dir.cross(edge2);
    auto a = edge1.dot(h);

    if (a > -eps * m * m && a < eps * m * m) return std::nullopt; // parallel

    auto f = 1.0 / a;
    auto s = origin - vertices[0];
    auto u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return std::nullopt;

    auto q = s.cross(edge1);
    auto v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return std::nullopt;

    auto t = (f * edge2.dot(q));
    if (t <= 0.0 * m) return std::nullopt;

    // Hit is on an edge if any barycentric coordinate is near 0, or u+v is near 1.
    bool on_edge = (u < edge_eps || v < edge_eps || (u + v) > 1.0 - edge_eps);
    return parity_hit{.t = t, .on_edge = on_edge};
}

// Cast a ray through the BVH and count crossings. Returns {crossings, hit_edge}.
// If hit_edge is true, at least one intersection landed on a shared edge/vertex and
// the parity result may be unreliable.
static std::pair<unsigned int, bool>
count_ray_crossings(const vec3<m> &point, const vec3<one> &dir, quantity<m> max_t,
                    const std::vector<mesh::triangle_t> &triangles,
                    const std::vector<vec3<m>> &vertices,
                    const std::vector<detail::bvh_node> &nodes)
{
    auto inv_dir = safe_inv(dir);

    unsigned int crossings = 0;
    bool hit_edge = false;

    std::array<std::uint32_t, bvh_stack_size> stack{};
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0)
    {
        auto node_idx = stack[--sp];
        const auto &node = nodes[node_idx];

        if (!bvh::ray_aabb_intersect(node.bounds, point, inv_dir, max_t)) continue;

        if (node.count > 0) // leaf
        {
            for (auto i = node.start; i < node.start + node.count; ++i)
            {
                const auto &tri = triangles[i];
                if (auto hit = ray_triangle_parity(point, dir, tri.vertices(vertices));
                    hit && hit->t <= max_t)
                {
                    ++crossings;
                    if (hit->on_edge) hit_edge = true;
                }
            }
        }
        else // internal
        {
            auto left = node_idx + 1;
            auto right = node.start;

            if (stack.size() < sp + 2)
                throw std::runtime_error("BVH stack overflow: increase bvh_stack_size");
            stack[sp++] = right;
            stack[sp++] = left;
        }
    }

    return {crossings, hit_edge};
}

bool mesh::contains(const vec3<m> &point) const
{
    assert(!M_bvh_nodes.empty());

    if (!M_bounds.contains(point)) return false; // early out

    auto max_t = M_bounds.size().norm() + 1.0 * m;

    // Cast rays and count crossings. If a hit lands exactly on a shared edge/vertex retry with a
    // different (random) direction.
    static const std::array directions{
        vec3<one>{1.0, 0.0, 0.0},
        vec3<one>{std::numbers::pi / 2, std::numbers::e * 3,
                  std::numbers::phi * std::numbers::sqrt2}
            .normalized(),
        vec3<one>{std::numbers::ln10 * std::numbers::pi, std::numbers::egamma / 2,
                  std::numbers::pi * std::numbers::sqrt3}
            .normalized(),
        vec3<one>{std::numbers::e, std::numbers::phi * 2, std::numbers::ln2}.normalized(),
    };

    for (const auto &dir : directions)
    {
        auto [crossings, hit_edge] =
            count_ray_crossings(point, dir, max_t, M_triangles, M_vertices, M_bvh_nodes);
        if (!hit_edge) return (crossings & 1u) != 0;
    }

    throw std::runtime_error(
        "mesh::contains failed to determine parity (what the hell are you doing, man?)");
}

vec3<m> mesh::support(const vec3<one> &direction) const
{
    assert(!M_vertices.empty());
    std::size_t best_idx = 0;
    auto best_dot = M_vertices[0].dot(direction);

    for (std::size_t i = 1; i < M_vertices.size(); ++i)
    {
        auto d = M_vertices[i].dot(direction);
        if (d > best_dot)
        {
            best_dot = d;
            best_idx = i;
        }
    }

    return M_vertices[best_idx];
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
bool mesh::is_convex() const
{
    // TODO: implement convexity test (check all edges, verify all vertices on same side of each
    // face)
    throw std::runtime_error("mesh::is_convex not yet implemented");
}

/*
------------------------------------------------------------
-------------------- MESH :: INSTANCE --------------------
------------------------------------------------------------
*/

aabb mesh::instance::bounds() const { return M_mesh->bounds() * M_orientation + M_position; }

std::optional<mesh::ray_hit> mesh::instance::ray_intersect(const mesh::ray &r,
                                                           quantity<m> max_distance) const
{
    // Transform ray into local space.
    auto inv_orient = M_orientation.conjugate();
    auto local_origin = inv_orient * (r.origin - M_position);
    auto local_dir = inv_orient * r.direction;
    mesh::ray local_ray{.origin = local_origin, .direction = local_dir};

    auto hit = M_mesh->ray_intersect(local_ray, max_distance);
    if (hit)
    {
        hit->pos = M_orientation * hit->pos + M_position;
        hit->normal = M_orientation * hit->normal;
    }
    return hit;
}

vec3<m> mesh::instance::closest_point(const vec3<m> &point) const
{
    auto inv_orient = M_orientation.conjugate();
    auto local_point = inv_orient * (point - M_position);
    auto local_closest = M_mesh->closest_point(local_point);
    return M_orientation * local_closest + M_position;
}

bool mesh::instance::contains(const vec3<m> &point) const
{
    auto inv_orient = M_orientation.conjugate();
    auto local_point = inv_orient * (point - M_position);
    return M_mesh->contains(local_point);
}

vec3<m> mesh::instance::support(const vec3<one> &direction) const
{
    auto inv_orient = M_orientation.conjugate();
    auto local_dir = inv_orient * direction;
    auto local_support = M_mesh->support(local_dir);
    return M_orientation * local_support + M_position;
}

mat3<kg * pow<2>(m)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
mesh::instance::inertia_tensor(quantity<kg / pow<3>(m)> density) const
{
    // TODO: rotate local inertia tensor into world frame and apply parallel axis theorem
    throw std::runtime_error("mesh::instance::inertia_tensor not yet implemented");
}

} // namespace physkit
