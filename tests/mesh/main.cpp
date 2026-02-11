// tests written by claude opus 4.6

#include "physkit/mesh.h"

#include <mp-units/systems/si/unit_symbols.h>

#include <cassert>
#include <cmath>
#include <print>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

constexpr double eps = 1e-9;

bool approx(double a, double b) { return std::abs(a - b) < eps; }
bool approx_q(quantity<si::metre> a, quantity<si::metre> b)
{
    return approx(a.numerical_value_in(si::metre), b.numerical_value_in(si::metre));
}
bool approx_q(quantity<pow<2>(si::metre)> a, quantity<pow<2>(si::metre)> b)
{
    return approx(a.numerical_value_in(pow<2>(si::metre)), b.numerical_value_in(pow<2>(si::metre)));
}
bool approx_q(quantity<pow<3>(si::metre)> a, quantity<pow<3>(si::metre)> b)
{
    return approx(a.numerical_value_in(pow<3>(si::metre)), b.numerical_value_in(pow<3>(si::metre)));
}

bool approx_vec(const vec3<si::metre> &a, const vec3<si::metre> &b)
{
    return approx_q(a.x(), b.x()) && approx_q(a.y(), b.y()) && approx_q(a.z(), b.z());
}

/// Unit cube: vertices at (0,0,0)..(1,1,1), 12 triangles with outward normals.
/// Winding order is counter-clockwise when viewed from outside.
struct cube_fixture
{
    std::array<vec3<m>, 8> vertices = {{
        vec3{0.0, 0.0, 0.0} * m, // 0
        vec3{1.0, 0.0, 0.0} * m, // 1
        vec3{1.0, 1.0, 0.0} * m, // 2
        vec3{0.0, 1.0, 0.0} * m, // 3
        vec3{0.0, 0.0, 1.0} * m, // 4
        vec3{1.0, 0.0, 1.0} * m, // 5
        vec3{1.0, 1.0, 1.0} * m, // 6
        vec3{0.0, 1.0, 1.0} * m, // 7
    }};

    // 12 triangles, 2 per face, outward-facing normals via CCW winding
    std::array<mesh::triangle_t, 12> triangles = {{
        // -Z face (z=0): normal (0,0,-1)
        {0, 2, 1},
        {0, 3, 2},
        // +Z face (z=1): normal (0,0,+1)
        {4, 5, 6},
        {4, 6, 7},
        // -Y face (y=0): normal (0,-1,0)
        {0, 1, 5},
        {0, 5, 4},
        // +Y face (y=1): normal (0,+1,0)
        {2, 3, 7},
        {2, 7, 6},
        // -X face (x=0): normal (-1,0,0)
        {0, 4, 7},
        {0, 7, 3},
        // +X face (x=1): normal (+1,0,0)
        {1, 2, 6},
        {1, 6, 5},
    }};

    std::shared_ptr<mesh> make_mesh() const { return mesh::make(vertices, triangles); }
};

// ---------------------------------------------------------------------------
// AABB tests
// ---------------------------------------------------------------------------

void test_aabb_from_points()
{
    std::println("  aabb::from_points");
    cube_fixture cube;
    auto box = aabb::from_points(cube.vertices);
    assert(approx_vec(box.min, vec3{0, 0, 0} * m));
    assert(approx_vec(box.max, vec3{1, 1, 1} * m));
}

void test_aabb_size_center_extent()
{
    std::println("  aabb::size/center/extent");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 4, 6} * m};
    auto s = box.size();
    assert(approx_q(s.x(), 2.0 * m));
    assert(approx_q(s.y(), 4.0 * m));
    assert(approx_q(s.z(), 6.0 * m));

    auto c = box.center();
    assert(approx_q(c.x(), 1.0 * m));
    assert(approx_q(c.y(), 2.0 * m));
    assert(approx_q(c.z(), 3.0 * m));

    auto e = box.extent();
    assert(approx_q(e.x(), 1.0 * m));
    assert(approx_q(e.y(), 2.0 * m));
    assert(approx_q(e.z(), 3.0 * m));
}

void test_aabb_volume()
{
    std::println("  aabb::volume");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 3, 4} * m};
    assert(approx_q(box.volume(), 24.0 * m * m * m));
}

void test_aabb_point()
{
    std::println("  aabb::point");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    // index 0 = (min.x, min.y, min.z)
    assert(approx_vec(box.point(0), box.min));
    // index 7 = (max.x, max.y, max.z)
    assert(approx_vec(box.point(7), box.max));
    // index 1 = (max.x, min.y, min.z)
    assert(approx_vec(box.point(1), vec3{1, 0, 0} * m));
    // index 5 = (max.x, min.y, max.z)
    assert(approx_vec(box.point(5), vec3{1, 0, 1} * m));
}

void test_aabb_contains()
{
    std::println("  aabb::contains");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    assert(box.contains(vec3{0.5, 0.5, 0.5} * m));
    assert(box.contains(vec3{0, 0, 0} * m)); // on boundary
    assert(box.contains(vec3{1, 1, 1} * m)); // on boundary
    assert(!box.contains(vec3{1.1, 0.5, 0.5} * m));
    assert(!box.contains(vec3{-0.1, 0.5, 0.5} * m));
}

void test_aabb_intersects()
{
    std::println("  aabb::intersects");
    aabb a{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    aabb b{.min = vec3{0.5, 0.5, 0.5} * m, .max = vec3{1.5, 1.5, 1.5} * m};
    aabb c{.min = vec3{2, 2, 2} * m, .max = vec3{3, 3, 3} * m};
    assert(a.intersects(b));
    assert(b.intersects(a));
    assert(!a.intersects(c));
    assert(!c.intersects(a));
    // Touching at a single corner counts as intersecting
    aabb d{.min = vec3{1, 1, 1} * m, .max = vec3{2, 2, 2} * m};
    assert(a.intersects(d));
}

void test_aabb_translate()
{
    std::println("  aabb::operator+/operator-");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    auto shifted = box + vec3{2.0, 3.0, 4.0} * m;
    assert(approx_vec(shifted.min, vec3{2.0, 3.0, 4.0} * m));
    assert(approx_vec(shifted.max, vec3{3.0, 4.0, 5.0} * m));

    auto back = shifted - vec3{2.0, 3.0, 4.0} * m;
    assert(approx_vec(back.min, box.min));
    assert(approx_vec(back.max, box.max));
}

void test_aabb_scale()
{
    std::println("  aabb::operator*/operator/");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 2, 2} * m};
    // Scaling by 2 doubles the size about center (1,1,1)
    auto scaled = box * 2.0;
    assert(approx_vec(scaled.min, vec3{-1, -1, -1} * m));
    assert(approx_vec(scaled.max, vec3{3, 3, 3} * m));

    // Scaling by 0.5 halves the size
    auto halved = box / 2.0;
    assert(approx_vec(halved.min, vec3{0.5, 0.5, 0.5} * m));
    assert(approx_vec(halved.max, vec3{1.5, 1.5, 1.5} * m));

    // Quantity scale
    auto qscaled = box * 2.0;
    assert(approx_vec(qscaled.min, vec3{-1.0, -1.0, -1.0} * m));
    assert(approx_vec(qscaled.max, vec3{3.0, 3.0, 3.0} * m));

    auto qdiv = box / 2.0;
    assert(approx_vec(qdiv.min, vec3{0.5, 0.5, 0.5} * m));
    assert(approx_vec(qdiv.max, vec3{1.5, 1.5, 1.5} * m));
}

void test_aabb_compound_assign()
{
    std::println("  aabb::operator+=/operator-=/operator*=/operator/=");
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 2, 2} * m};

    box += vec3{1.0, 1.0, 1.0} * m;
    assert(approx_vec(box.min, vec3{1.0, 1.0, 1.0} * m));
    assert(approx_vec(box.max, vec3{3.0, 3.0, 3.0} * m));

    box -= vec3{1.0, 1.0, 1.0} * m;
    assert(approx_vec(box.min, vec3{0.0, 0.0, 0.0} * m));
    assert(approx_vec(box.max, vec3{2.0, 2.0, 2.0} * m));

    box *= 2.0;
    assert(approx_vec(box.min, vec3{-1.0, -1.0, -1.0} * m));
    assert(approx_vec(box.max, vec3{3.0, 3.0, 3.0} * m));

    // Reset
    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box /= 2.0;
    assert(approx_vec(box.min, vec3{0.5, 0.5, 0.5} * m));
    assert(approx_vec(box.max, vec3{1.5, 1.5, 1.5} * m));

    // Quantity compound assign
    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box *= 2.0;
    assert(approx_vec(box.min, vec3{-1.0, -1.0, -1.0} * m));
    assert(approx_vec(box.max, vec3{3.0, 3.0, 3.0} * m));

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box /= 2.0;
    assert(approx_vec(box.min, vec3{0.5, 0.5, 0.5} * m));
    assert(approx_vec(box.max, vec3{1.5, 1.5, 1.5} * m));
}

void test_aabb_mat_transform()
{
    std::println("  aabb::operator*(mat3/quat)");
    aabb box{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};

    // Identity transform should leave bounds unchanged
    auto identity = mat3<one>::identity();
    auto result = box * identity;
    assert(approx_vec(result.min, box.min));
    assert(approx_vec(result.max, box.max));

    // 90-degree rotation around Z: (x,y,z) -> (-y,x,z)
    // cos(90)=0, sin(90)=1
    auto rot_z = mat3<one>{
        {0.0, -1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
    };
    auto rotated = box * rot_z;
    assert(approx_vec(rotated.min, vec3{-1.0, 0.0, 0.0} * m));
    assert(approx_vec(rotated.max, vec3{0.0, 1.0, 1.0} * m));

    auto rot_q = quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0, 0, 1});
    auto rotated_q = box * rot_q;
    assert(approx_vec(rotated_q.min, rotated.min));
    assert(approx_vec(rotated_q.max, rotated.max));
}

// ---------------------------------------------------------------------------
// Bounding sphere tests
// ---------------------------------------------------------------------------

void test_bsphere_from_points()
{
    std::println("  bounding_sphere::from_points");
    cube_fixture cube;
    auto sphere = bounding_sphere::from_points(cube.vertices);
    // Must contain all vertices
    for (const auto &v : cube.vertices)
    {
        auto dist = (v - sphere.center).norm();
        assert(dist <= sphere.radius + 1e-9 * m);
    }
}

void test_bsphere_from_aabb()
{
    std::println("  bounding_sphere::from_aabb");
    aabb box{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    auto sphere = bounding_sphere::from_aabb(box);
    assert(approx_vec(sphere.center, vec3{1.0, 1.0, 1.0} * m));
    // radius = half-diagonal = sqrt(3)
    assert(approx_q(sphere.radius, std::sqrt(3.0) * m));
}

void test_bsphere_merge()
{
    std::println("  bounding_sphere::merge");
    bounding_sphere a{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    bounding_sphere b{.center = vec3{3.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto merged = bounding_sphere::merge(a, b);
    // merged should enclose both: center at (1.5,0,0), radius = 2.5
    assert(approx_q(merged.center.x(), 1.5 * m));
    assert(approx_q(merged.center.y(), 0.0 * m));
    assert(approx_q(merged.center.z(), 0.0 * m));
    assert(approx_q(merged.radius, 2.5 * m));

    // Merging when one encloses the other
    bounding_sphere big{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 10.0 * m};
    bounding_sphere small{.center = vec3{1.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto m1 = bounding_sphere::merge(big, small);
    assert(approx_q(m1.radius, big.radius));
    auto m2 = bounding_sphere::merge(small, big);
    assert(approx_q(m2.radius, big.radius));
}

void test_bsphere_surface_area_volume()
{
    std::println("  bounding_sphere::surface_area/volume");
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 2.0 * m};
    auto sa = s.surface_area();
    auto expected_sa = 4.0 * std::numbers::pi * 4.0 * m * m; // 4 pi r^2
    assert(approx_q(sa, expected_sa));

    auto vol = s.volume();
    auto expected_vol = (4.0 / 3.0) * std::numbers::pi * 8.0 * m * m * m; // (4/3) pi r^3
    assert(approx_q(vol, expected_vol));
}

void test_bsphere_contains()
{
    std::println("  bounding_sphere::contains");
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    assert(s.contains(vec3{0.0, 0.0, 0.0} * m));
    assert(s.contains(vec3{1.0, 0.0, 0.0} * m)); // on surface
    assert(!s.contains(vec3{1.1, 0.0, 0.0} * m));
}

void test_bsphere_intersects_sphere()
{
    std::println("  bounding_sphere::intersects(sphere)");
    bounding_sphere a{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    bounding_sphere b{.center = vec3{1.5, 0.0, 0.0} * m, .radius = 1.0 * m};
    bounding_sphere c{.center = vec3{3.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    assert(a.intersects(b));
    assert(!a.intersects(c));
}

void test_bsphere_intersects_aabb()
{
    std::println("  bounding_sphere::intersects(aabb)");
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    aabb box{.min = vec3{0.5, 0.5, 0.5} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    assert(s.intersects(box));

    aabb far_box{.min = vec3{5.0, 5.0, 5.0} * m, .max = vec3{6.0, 6.0, 6.0} * m};
    assert(!s.intersects(far_box));
}

void test_bsphere_translate()
{
    std::println("  bounding_sphere::operator+/operator-");
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto moved = s + vec3{1.0, 2.0, 3.0} * m;
    assert(approx_vec(moved.center, vec3{1.0, 2.0, 3.0} * m));
    assert(approx_q(moved.radius, 1.0 * m));

    auto back = moved - vec3{1.0, 2.0, 3.0} * m;
    assert(approx_vec(back.center, vec3{0.0, 0.0, 0.0} * m));
}

void test_bsphere_scale()
{
    std::println("  bounding_sphere::operator*");
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto scaled = s * 3.0;
    assert(approx_q(scaled.radius, 3.0 * m));
    assert(approx_vec(scaled.center, s.center)); // center unchanged

    auto qscaled = s * 2.0;
    assert(approx_q(qscaled.radius, 2.0 * m));
}

void test_bsphere_compound_assign()
{
    std::println("  bounding_sphere::operator+=/operator-=/operator*=");
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    s += vec3{1.0, 0.0, 0.0} * m;
    assert(approx_vec(s.center, vec3{1.0, 0.0, 0.0} * m));

    s -= vec3{1.0, 0.0, 0.0} * m;
    assert(approx_vec(s.center, vec3{0.0, 0.0, 0.0} * m));

    s *= 5.0;
    assert(approx_q(s.radius, 5.0 * m));

    s = {.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    s *= 3.0;
    assert(approx_q(s.radius, 3.0 * m));
}

// ---------------------------------------------------------------------------
// Mesh tests
// ---------------------------------------------------------------------------

void test_mesh_make()
{
    std::println("  mesh::make");
    cube_fixture cube;
    auto msh = cube.make_mesh();
    assert(msh->vertices().size() == 8);
    assert(msh->triangles().size() == 12);
}

void test_mesh_bounds()
{
    std::println("  mesh::bounds");
    cube_fixture cube;
    auto msh = cube.make_mesh();
    assert(approx_vec(msh->bounds().min, vec3{0.0, 0.0, 0.0} * m));
    assert(approx_vec(msh->bounds().max, vec3{1.0, 1.0, 1.0} * m));
}

void test_mesh_bsphere()
{
    std::println("  mesh::bsphere");
    cube_fixture cube;
    auto msh = cube.make_mesh();
    // Bounding sphere must contain all vertices
    for (const auto &v : cube.vertices)
    {
        auto dist = (v - msh->bsphere().center).norm();
        assert(dist <= msh->bsphere().radius + 1e-9 * m);
    }
}

void test_mesh_volume()
{
    std::println("  mesh::volume");
    cube_fixture cube;
    auto msh = cube.make_mesh();
    assert(approx_q(msh->volume(), 1.0 * m * m * m));
}

void test_mesh_ray_intersect()
{
    std::println("  mesh::ray_intersect");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray from outside hitting the cube
    mesh::ray r{
        .origin = vec3{-1.0, 0.5, 0.5} * m,
        .direction = {1.0, 0.0, 0.0},
    };
    auto hit = msh->ray_intersect(r);
    assert(hit.has_value());
    assert(approx_q(hit->pos.x(), 0.0 * m));
    assert(approx_q(hit->pos.y(), 0.5 * m));
    assert(approx_q(hit->pos.z(), 0.5 * m));
    assert(approx_q(hit->distance, 1.0 * m));

    // Ray pointing away - no hit
    mesh::ray away{
        .origin = vec3{-1.0, 0.5, 0.5} * m,
        .direction = vec3{-1.0, 0.0, 0.0},
    };
    assert(!msh->ray_intersect(away).has_value());

    // Ray with max_distance that is too short
    auto short_hit = msh->ray_intersect(r, 0.5 * m);
    assert(!short_hit.has_value());

    // Ray from inside - should still hit
    mesh::ray inside{
        .origin = vec3{0.5, 0.5, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    auto inside_hit = msh->ray_intersect(inside);
    assert(inside_hit.has_value());
    assert(approx_q(inside_hit->distance, 0.5 * m));
}

void test_mesh_contains()
{
    std::println("  mesh::contains");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Use points where y != z to avoid hitting the shared diagonal edge of each face pair,
    // which causes the ray-parity test to double-count crossings.
    assert(msh->contains(vec3{0.3, 0.2, 0.4} * m));
    // Use points where y != z to avoid hitting the shared diagonal edge of each face
    assert(msh->contains(vec3{0.7, 0.2, 0.4} * m));
    assert(msh->contains(vec3{0.3, 0.7, 0.4} * m));

    // Exterior points
    assert(!msh->contains(vec3{-0.5, 0.2, 0.4} * m));
    assert(!msh->contains(vec3{1.5, 0.2, 0.4} * m));
    assert(!msh->contains(vec3{0.3, -0.5, 0.4} * m));
    assert(!msh->contains(vec3{0.3, 0.2, 1.5} * m));
}

void test_mesh_support()
{
    std::println("  mesh::support");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Support in +x direction -> should be one of the x=1 vertices
    auto sup_x = msh->support(vec3{1.0, 0.0, 0.0});
    assert(approx_q(sup_x.x(), 1.0 * m));

    // Support in -x direction -> should be one of the x=0 vertices
    auto sup_nx = msh->support(vec3{-1.0, 0.0, 0.0});
    assert(approx_q(sup_nx.x(), 0.0 * m));

    // Support in +y direction -> y=1
    auto sup_y = msh->support(vec3{0.0, 1.0, 0.0});
    assert(approx_q(sup_y.y(), 1.0 * m));

    // Support in diagonal direction -> should be corner (1,1,1)
    auto sup_diag = msh->support(vec3{1.0, 1.0, 1.0});
    assert(approx_vec(sup_diag, vec3{1.0, 1.0, 1.0} * m));
}

// ---------------------------------------------------------------------------
// Mesh instance tests
// ---------------------------------------------------------------------------

void test_mesh_instance_basic()
{
    std::println("  mesh::instance basic construction");
    cube_fixture cube;
    auto msh = cube.make_mesh();
    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);

    assert(&inst.geometry() == msh.get());
    assert(approx_vec(inst.position(), vec3{5.0, 0.0, 0.0} * m));
}

void test_mesh_instance_world_bounds()
{
    std::println("  mesh::instance world_bounds");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Translated by (5,0,0)
    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    auto wb = inst.bounds();
    assert(approx_vec(wb.min, vec3{5.0, 0.0, 0.0} * m));
    assert(approx_vec(wb.max, vec3{6.0, 1.0, 1.0} * m));
}

void test_mesh_instance_world_bsphere()
{
    std::println("  mesh::instance world_bsphere");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    auto ws = inst.bsphere();
    // Center should be translated
    auto local_center = msh->bsphere().center;
    assert(approx_vec(ws.center, local_center + vec3{5.0, 0.0, 0.0} * m));
    // Radius unchanged (rotation invariant)
    assert(approx_q(ws.radius, msh->bsphere().radius));
}

void test_mesh_instance_ray_intersect()
{
    std::println("  mesh::instance ray_intersect");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Place the cube at (5,0,0)
    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);

    mesh::ray r{
        .origin = vec3{3.0, 0.5, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    auto hit = inst.ray_intersect(r);
    assert(hit.has_value());
    assert(approx_q(hit->pos.x(), 5.0 * m));
    assert(approx_q(hit->distance, 2.0 * m));

    // Miss
    mesh::ray miss{
        .origin = vec3{3.0, 5.0, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    assert(!inst.ray_intersect(miss).has_value());
}

void test_mesh_instance_contains()
{
    std::println("  mesh::instance contains");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    assert(inst.contains(vec3{5.3, 0.2, 0.4} * m));
    assert(!inst.contains(vec3{0.3, 0.2, 0.4} * m)); // original location, not translated
}

void test_mesh_instance_support()
{
    std::println("  mesh::instance support");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    // Support in +x should be x = 5 + 1 = 6
    auto sup = inst.support(vec3{1.0, 0.0, 0.0});
    assert(approx_q(sup.x(), 6.0 * m));
}

void test_mesh_instance_rotated()
{
    std::println("  mesh::instance rotated ray_intersect");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // 90-degree rotation around Z: (x,y,z) -> (-y,x,z)
    auto rot_z =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});

    auto inst = msh->at(vec3{0.0, 0.0, 0.0} * m, rot_z);

    // After rotating the unit cube [0,1]^3 by 90 degrees around Z,
    // the AABB becomes [-1,0] x [0,1] x [0,1]
    auto wb = inst.bounds();
    assert(approx_q(wb.min.x(), -1.0 * m));
    assert(approx_q(wb.max.x(), 0.0 * m));

    // Ray in +x should hit at x = -1
    mesh::ray r{
        .origin = vec3{-3.0, 0.5, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    auto hit = inst.ray_intersect(r);
    assert(hit.has_value());
    assert(approx_q(hit->pos.x(), -1.0 * m));
}

void test_mesh_instance_different_positions()
{
    std::println("  mesh::instance different positions");
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Same mesh, different instances — position is copied so temporaries are fine
    auto inst1 = msh->at(vec3{10.0, 0.0, 0.0} * m);
    assert(inst1.contains(vec3{10.3, 0.2, 0.4} * m));

    auto inst2 = msh->at(vec3{0.0, 0.0, 0.0} * m);
    assert(inst2.contains(vec3{0.3, 0.2, 0.4} * m));
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
    std::println("=== AABB Tests ===");
    test_aabb_from_points();
    test_aabb_size_center_extent();
    test_aabb_volume();
    test_aabb_point();
    test_aabb_contains();
    test_aabb_intersects();
    test_aabb_translate();
    test_aabb_scale();
    test_aabb_compound_assign();
    test_aabb_mat_transform();

    std::println("\n=== Bounding Sphere Tests ===");
    test_bsphere_from_points();
    test_bsphere_from_aabb();
    test_bsphere_merge();
    test_bsphere_surface_area_volume();
    test_bsphere_contains();
    test_bsphere_intersects_sphere();
    test_bsphere_intersects_aabb();
    test_bsphere_translate();
    test_bsphere_scale();
    test_bsphere_compound_assign();

    std::println("\n=== Mesh Tests ===");
    test_mesh_make();
    test_mesh_bounds();
    test_mesh_bsphere();
    test_mesh_volume();
    test_mesh_ray_intersect();
    test_mesh_contains();
    test_mesh_support();

    std::println("\n=== Mesh Instance Tests ===");
    test_mesh_instance_basic();
    test_mesh_instance_world_bounds();
    test_mesh_instance_world_bsphere();
    test_mesh_instance_ray_intersect();
    test_mesh_instance_contains();
    test_mesh_instance_support();
    test_mesh_instance_rotated();
    test_mesh_instance_different_positions();

    std::println("\nAll mesh tests passed!");
    return 0;
}
