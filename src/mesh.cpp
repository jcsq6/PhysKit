#include "physkit/mesh.h"

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
std::optional<mesh::ray_hit> mesh::ray_intersect_local(const ray &r,
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
vec3<si::metre> mesh::closest_point_local(const vec3<si::metre> & /*point*/) const
{
    // TODO: implement closest-point-on-mesh (project onto each triangle, take closest)
    assert(false && "mesh::closest_point_local not yet implemented");
    return vec3<si::metre>::zero();
}

bool mesh::contains_local(const vec3<si::metre> &point) const
{
    // Ray-casting parity test: cast a ray in +x and count crossings.
    auto dir = vec3<one>{1.0 * one, 0.0 * one, 0.0 * one};
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

vec3<si::metre> mesh::support_local(const vec3<one> &direction) const
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
-------------------------------------------------------
-------------------- MESH INSTANCE --------------------
-------------------------------------------------------
*/

aabb mesh_instance::world_bounds() const
{
    return M_mesh->bounds().operator* <quantity<one>>(M_orientation) + M_position;
}

std::optional<mesh::ray_hit> mesh_instance::ray_intersect(const mesh::ray &r,
                                                          quantity<si::metre> max_distance) const
{
    // Transform ray into local space.
    auto inv_orient = M_orientation.transpose();
    auto local_origin = inv_orient * (r.origin - M_position);
    auto local_dir = inv_orient * r.direction;
    mesh::ray local_ray{.origin = local_origin, .direction = local_dir};

    auto hit = M_mesh->ray_intersect_local(local_ray, max_distance);
    if (hit)
    {
        hit->pos = M_orientation * hit->pos + M_position;
        hit->normal = M_orientation * hit->normal;
    }
    return hit;
}

vec3<si::metre> mesh_instance::closest_point(const vec3<si::metre> &point) const
{
    auto inv_orient = M_orientation.transpose();
    auto local_point = inv_orient * (point - M_position);
    auto local_closest = M_mesh->closest_point_local(local_point);
    return M_orientation * local_closest + M_position;
}

bool mesh_instance::contains(const vec3<si::metre> &point) const
{
    auto inv_orient = M_orientation.transpose();
    auto local_point = inv_orient * (point - M_position);
    return M_mesh->contains_local(local_point);
}

vec3<si::metre> mesh_instance::support(const vec3<one> &direction) const
{
    auto inv_orient = M_orientation.transpose();
    auto local_dir = inv_orient * direction;
    auto local_support = M_mesh->support_local(local_dir);
    return M_orientation * local_support + M_position;
}

mat3<si::kilogram * pow<2>(si::metre)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
mesh_instance::world_inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
{
    // TODO: rotate local inertia tensor into world frame and apply parallel axis theorem
    assert(false && "mesh_instance::world_inertia_tensor not yet implemented");
    return mat3<si::kilogram * pow<2>(si::metre)>::zero();
}

} // namespace physkit
