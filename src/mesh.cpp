#include "physkit/mesh.h"

#include <cmath>

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

/*
----------------------------------------------
-------------------- MESH --------------------
----------------------------------------------
*/

std::shared_ptr<mesh> mesh::make(std::span<const vec3<si::metre>> vertices,
                                 std::span<const triangle_t> triangles)
{
    assert(!vertices.empty());
    assert(!triangles.empty());

    auto m = std::make_shared<mesh>();
    m->M_vertices.assign(vertices.begin(), vertices.end());
    m->M_triangles.assign(triangles.begin(), triangles.end());

    m->M_bounds = aabb::from_points(vertices);
    m->M_bsphere = bounding_sphere::from_points(vertices);

    return m;
}

std::shared_ptr<mesh> mesh::box(const vec3<si::metre> &half_extents)
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

std::shared_ptr<mesh> mesh::sphere(quantity<si::metre> radius, unsigned int stacks,
                                   unsigned int sectors)
{
    assert(stacks >= 2);
    assert(sectors >= 3);

    std::vector<vec3<si::metre>> vertices;
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

std::shared_ptr<mesh> mesh::pyramid(quantity<si::metre> base_half, quantity<si::metre> height)
{
    // clang-format off
    std::array vertices{
        vec3{-base_half, 0.0 * m, -base_half}, // 0
        vec3{+base_half, 0.0 * m, -base_half}, // 1
        vec3{+base_half, 0.0 * m, +base_half}, // 2
        vec3{-base_half, 0.0 * m, +base_half}, // 3
        vec3{    0.0 * m,  height,     0.0 * m}, // 4 (apex)
    };

    std::array<triangle_t, 6> triangles{{
        {0, 1, 2}, {0, 2, 3}, // base (-y)
        {1, 0, 4},            // front (-z)
        {2, 1, 4},            // right (+x)
        {3, 2, 4},            // back  (+z)
        {0, 3, 4},            // left  (-x)
    }};
    // clang-format on

    return make(vertices, triangles);
}

quantity<pow<3>(si::metre)> mesh::volume() const
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
vec3<si::metre> mesh::mass_center() const
{
    // TODO: implement volume-weighted centroid via divergence theorem
    assert(false && "mesh::mass_center not yet implemented");
    return vec3<si::metre>::zero();
}

mat3<si::kilogram * pow<2>(si::metre)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
mesh::inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> /*density*/) const
{
    // TODO: implement inertia tensor via canonical tetrahedron integrals
    assert(false && "mesh::inertia_tensor not yet implemented");
    return mat3<si::kilogram * pow<2>(si::metre)>::zero();
}

// Moeller–Trumbore
std::optional<mesh::ray_hit> mesh::ray_intersect(const ray &r,
                                                 quantity<si::metre> max_distance) const
{
    // TODO: accelerate with BVH
    std::optional<ray_hit> best;

    auto dir = r.direction.normalized(); // normalized for units

    constexpr auto eps = 1e-12;

    for (const auto &tri : M_triangles)
    {
        const auto &v0 = M_vertices[tri[0]];
        const auto &v1 = M_vertices[tri[1]];
        const auto &v2 = M_vertices[tri[2]];

        auto edge1 = v1 - v0;
        auto edge2 = v2 - v0;
        auto h = dir.cross(edge2);
        auto a = edge1.dot(h);

        if (a > -eps * m * m && a < eps * m * m) continue;

        auto f = 1.0 / a;
        auto s = r.origin - v0;
        auto u = f * s.dot(h);
        if (u < 0.0 || u > 1.0) continue;

        auto q = s.cross(edge1);
        auto v = f * dir.dot(q);
        if (v < 0.0 || u + v > 1.0) continue;

        auto t = f * edge2.dot(q);
        if (t < 0.0 * m || t > max_distance) continue;

        if (!best || t < best->distance)
        {
            auto normal = edge1.cross(edge2).normalized();
            best = ray_hit{
                .pos = r.origin + dir * t,
                .normal = normal,
                .distance = t,
            };
        }
    }

    return best;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
vec3<si::metre> mesh::closest_point(const vec3<si::metre> &p) const
{
    assert(!M_triangles.empty() && "empty mesh");
    vec3<si::metre> closest = M_vertices[0];
    for (const auto &tri : M_triangles)
    {
        //get points and vectors
        std::array<vec3<si::metre>, 3> vertices = {
            M_vertices.at(tri[0]),
            M_vertices.at(tri[1]),
            M_vertices.at(tri[2])
        };
        std::array<vec3<si::metre>, 3> sides = {
            M_vertices.at(tri[1]) - M_vertices.at(tri[0]),
            M_vertices.at(tri[2]) - M_vertices.at(tri[1]),
            M_vertices.at(tri[0]) - M_vertices.at(tri[2])
        };
        auto n = tri.normal(*this);

        //get planar projection of the point
        vec3<si::metre> projected = p - (((p-vertices[0]).dot(n))*n);

        //project planar point onto the lines of the triangle, and clamp them to the line segment
        //compare the distance to the point
        //was the point within the triangle?
        bool bound = true;
        for (int i = 0; i < 3; i++)
        {
            //projection
            vec3<si::metre> v = (projected-vertices.at(i));
            v = vertices.at(i) + ((v.dot(sides.at(i))/sides.at(i).dot(sides.at(i))) * sides.at(i));

            //clamp
            auto x = v.x();
            auto y = v.y();
            auto z = v.z();

            auto less = sides.at(i).x()+vertices.at(i).x() < vertices.at(i).x()? sides.at(i).x()+vertices.at(i).x() : vertices.at(i).x();
            auto more = sides.at(i).x()+vertices.at(i).x() > vertices.at(i).x()? sides.at(i).x()+vertices.at(i).x() : vertices.at(i).x();
            x = std::clamp(v.x(), less, more);
            if (x != v.x()) bound = false;

            less = sides.at(i).y()+vertices.at(i).y() < vertices.at(i).y()? sides.at(i).y()+vertices.at(i).y() : vertices.at(i).y();
            more = sides.at(i).y()+vertices.at(i).y() > vertices.at(i).y()? sides.at(i).y()+vertices.at(i).y() : vertices.at(i).y();
            y = std::clamp(v.y(), less, more);
            if (y != v.y()) bound = false;

            less = sides.at(i).z()+vertices.at(i).z() < vertices.at(i).z()? sides.at(i).z()+vertices.at(i).z() : vertices.at(i).z();
            more = sides.at(i).z()+vertices.at(i).z() > vertices.at(i).z()? sides.at(i).z()+vertices.at(i).z() : vertices.at(i).z();
            z = std::clamp(v.z(), less, more);
            if (z != v.z()) bound = false;

            //compare
            if ((vec3<m>{x,y,z} - p).norm() < (closest - p).norm())
                closest = vec3<m>{x,y,z};
        }
        //compare planar point
        if (((projected - p).norm() < (closest - p).norm()) && bound)
            closest = projected;
    }
    return closest;
}

bool mesh::contains(const vec3<si::metre> &point) const
{
    // Ray-casting parity test: cast a ray in +x and count crossings.
    auto dir = vec3{1.0, 0.0, 0.0};
    int crossings = 0;

    constexpr auto eps = 1e-12;

    for (const auto &tri : M_triangles)
    {
        const auto &v0 = M_vertices[tri[0]];
        const auto &v1 = M_vertices[tri[1]];
        const auto &v2 = M_vertices[tri[2]];

        auto edge1 = v1 - v0;
        auto edge2 = v2 - v0;
        auto h = dir.cross(edge2);
        auto a = edge1.dot(h);

        if (a > -eps * m * m && a < eps * m * m) continue;

        auto f = 1.0 / a;
        auto s = point - v0;
        auto u = f * s.dot(h);
        if (u < 0.0 || u > 1.0) continue;

        auto q = s.cross(edge1);
        auto v = f * dir.dot(q);
        if (v < 0.0 || u + v > 1.0) continue;

        auto t = f * edge2.dot(q);
        if (t > 0.0 * m) ++crossings;
    }

    return (crossings % 2) == 1;
}

vec3<si::metre> mesh::support(const vec3<one> &direction) const
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
    assert(false && "mesh::is_convex not yet implemented");
    return false;
}

/*
------------------------------------------------------------
-------------------- MESH :: INSTANCE --------------------
------------------------------------------------------------
*/

aabb mesh::instance::bounds() const { return M_mesh.bounds() * M_orientation + M_position; }

std::optional<mesh::ray_hit> mesh::instance::ray_intersect(const mesh::ray &r,
                                                           quantity<si::metre> max_distance) const
{
    // Transform ray into local space.
    auto inv_orient = M_orientation.conjugate();
    auto local_origin = inv_orient * (r.origin - M_position);
    auto local_dir = inv_orient * r.direction;
    mesh::ray local_ray{.origin = local_origin, .direction = local_dir};

    auto hit = M_mesh.ray_intersect(local_ray, max_distance);
    if (hit)
    {
        hit->pos = M_orientation * hit->pos + M_position;
        hit->normal = M_orientation * hit->normal;
    }
    return hit;
}

vec3<si::metre> mesh::instance::closest_point(const vec3<si::metre> &point) const
{
    auto inv_orient = M_orientation.conjugate();
    auto local_point = inv_orient * (point - M_position);
    auto local_closest = M_mesh.closest_point(local_point);
    return M_orientation * local_closest + M_position;
}

bool mesh::instance::contains(const vec3<si::metre> &point) const
{
    auto inv_orient = M_orientation.conjugate();
    auto local_point = inv_orient * (point - M_position);
    return M_mesh.contains(local_point);
}

vec3<si::metre> mesh::instance::support(const vec3<one> &direction) const
{
    auto inv_orient = M_orientation.conjugate();
    auto local_dir = inv_orient * direction;
    auto local_support = M_mesh.support(local_dir);
    return M_orientation * local_support + M_position;
}

mat3<si::kilogram * pow<2>(si::metre)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
mesh::instance::inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
{
    // TODO: rotate local inertia tensor into world frame and apply parallel axis theorem
    assert(false && "mesh::instance::inertia_tensor not yet implemented");
    return mat3<si::kilogram * pow<2>(si::metre)>::zero();
}

} // namespace physkit
