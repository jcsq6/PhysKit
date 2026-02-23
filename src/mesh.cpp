#include "physkit/mesh.h"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

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

// BVH-accelerated ray intersection
std::optional<ray::hit> mesh::ray_intersect(const ray &r, quantity<m> max_distance) const
{
    assert(!M_bvh.empty());

    auto best_t = max_distance;
    std::optional<ray::hit> best;

    M_bvh.traverse_nearest(
        [&](const aabb &bounds) -> std::optional<quantity<m>>
        {
            auto d = r.intersect_distance(bounds, max_distance);
            if (d.has_value() && d.value() <= best_t) return d;
            return std::nullopt;
        },
        [&](std::uint32_t start, std::uint32_t count)
        {
            auto end = start + count;
            for (std::uint32_t i = start; i < end; ++i)
            {
                if (auto hit = r.intersect(M_triangles[i].vertices(*this), best_t))
                {
                    best_t = hit->distance;
                    best = hit;
                }
            }
        });

    return best;
}

// BVH-accelerated closest point on mesh surface
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
vec3<m> mesh::closest_point(const vec3<m> &p) const
{
    assert(!M_bvh.empty() && "empty mesh");

    auto best_dist_sq = std::numeric_limits<double>::infinity() * pow<2>(m);
    auto closest = M_vertices[0];

    M_bvh.traverse_nearest(
        [&](const aabb &bounds) -> std::optional<decltype(best_dist_sq)>
        {
            auto d_sq = bounds.squared_distance_to(p);
            if (d_sq >= best_dist_sq) return std::nullopt;
            return d_sq;
        },
        [&](std::uint32_t start, std::uint32_t count)
        {
            auto end = start + count;
            for (std::uint32_t i = start; i < end; ++i)
            {
                const auto &tri = M_triangles[i];
                auto candidate = tri.closest_point(p, M_vertices);
                if (auto d_sq = (candidate - p).squared_norm(); d_sq < best_dist_sq)
                {
                    best_dist_sq = d_sq;
                    closest = candidate;
                }
            }
        });

    return closest;
}

// Ray-triangle intersection for containment parity counting.
// Returns {t, on_edge} where on_edge is true if the hit is near a triangle edge/vertex.
struct parity_hit
{
    quantity<m> t;
    bool on_edge;
};

static std::optional<parity_hit> ray_triangle_parity(const ray &r,
                                                     const std::array<vec3<m>, 3> &vertices)
{
    constexpr auto eps = 1e-12;
    constexpr auto edge_eps = 1e-10; // threshold for "on an edge"

    auto edge1 = vertices[1] - vertices[0];
    auto edge2 = vertices[2] - vertices[0];
    auto h = r.direction().cross(edge2);
    auto a = edge1.dot(h);

    if (a > -eps * m * m && a < eps * m * m) return std::nullopt; // parallel

    auto f = 1.0 / a;
    auto s = r.origin() - vertices[0];
    auto u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return std::nullopt;

    auto q = s.cross(edge1);
    auto v = f * r.direction().dot(q);
    if (v < 0.0 || u + v > 1.0) return std::nullopt;

    auto t = (f * edge2.dot(q));
    if (t <= 0.0 * m) return std::nullopt;

    // Hit is on an edge if any barycentric coordinate is near 0, or u+v is near 1.
    bool on_edge = (u < edge_eps || v < edge_eps || (u + v) > 1.0 - edge_eps);
    return parity_hit{.t = t, .on_edge = on_edge};
}

bool mesh::contains(const vec3<m> &point) const
{
    assert(!M_bvh.empty());

    if (!M_bounds.contains(point)) return false; // early out

    auto max_t = M_bounds.size().norm() + 1.0 * m;

    // Cast rays and count crossings. If a hit lands exactly on a shared edge/vertex retry with a
    // different (random) direction.
    static const std::array directions{
        vec3<one>{1.0, 0.0, 0.0},
        vec3<one>{std::numbers::pi / 2, std::numbers::e * 3,
                  std::numbers::phi * std::numbers::sqrt2},
        vec3<one>{std::numbers::ln10 * std::numbers::pi, std::numbers::egamma / 2,
                  std::numbers::pi * std::numbers::sqrt3},
        vec3<one>{std::numbers::e, std::numbers::phi * 2, std::numbers::ln2},
    };

    for (const auto &dir : directions)
    {
        unsigned int crossings = 0;
        bool hit_edge = false;

        auto ray = physkit::ray{point, dir};

        M_bvh.traverse([&](const aabb &bounds)
                       { return ray.intersect_distance(bounds, max_t).has_value(); },
                       [&](const aabb &bounds, std::uint32_t offset, std::uint32_t count)
                       {
                           for (std::uint32_t i = 0; i < count; ++i)
                           {
                               const auto &tri = M_triangles[offset + i];
                               if (auto hit = ray_triangle_parity(ray, tri.vertices(*this));
                                   hit && hit->t <= max_t)
                               {
                                   ++crossings;
                                   if (hit->on_edge) hit_edge = true;
                               }
                           }
                       });

        if (!hit_edge) return (crossings & 1u) != 0;
    }

    throw std::runtime_error(
        "mesh::contains failed to determine parity (what the hell are you doing, man?)");
}

std::vector<std::uint32_t> mesh::overlap_sphere(const bounding_sphere &sphere) const
{
    assert(!M_bvh.empty());

    auto r_sq = sphere.radius * sphere.radius;

    std::vector<std::uint32_t> overlapping;       // TODO: Optimize allocation
    overlapping.reserve(M_triangles.size() / 10); // heuristic

    M_bvh.traverse([&](const aabb &bounds) { return sphere.intersects(bounds); },
                   [&](const aabb & /*bound*/, std::uint32_t start, std::uint32_t count)
                   {
                       auto end = start + count;
                       for (std::uint32_t i = start; i < end; ++i)
                           if (auto closest = M_triangles[i].closest_point(sphere.center, *this);
                               (closest - sphere.center).squared_norm() < r_sq)
                               overlapping.push_back(i);
                   });
    return overlapping;
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

std::optional<ray::hit> mesh::instance::ray_intersect(const ray &r, quantity<m> max_distance) const
{
    // Transform ray into local space.
    auto inv_orient = M_orientation.conjugate();
    auto local_origin = inv_orient * (r.origin() - M_position);
    auto local_dir = inv_orient * r.direction();
    ray local_ray{local_origin, local_dir};

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

std::vector<std::uint32_t> mesh::instance::overlap_sphere(const bounding_sphere &sphere) const
{
    auto inv_orient = M_orientation.conjugate();
    bounding_sphere local_sphere{
        .center = inv_orient * (sphere.center - M_position),
        .radius = sphere.radius,
    };
    return M_mesh->overlap_sphere(local_sphere);
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
