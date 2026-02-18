// tests written by claude opus 4.6

#include "test.h"

using namespace testing;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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
    cube_fixture cube;
    auto box = aabb::from_points(cube.vertices);
    CHECK_APPROX(box.min, vec3{0, 0, 0} * m);
    CHECK_APPROX(box.max, vec3{1, 1, 1} * m);
}

void test_aabb_size_center_extent()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 4, 6} * m};
    auto s = box.size();
    CHECK_APPROX(s.x(), 2.0 * m);
    CHECK_APPROX(s.y(), 4.0 * m);
    CHECK_APPROX(s.z(), 6.0 * m);

    auto c = box.center();
    CHECK_APPROX(c.x(), 1.0 * m);
    CHECK_APPROX(c.y(), 2.0 * m);
    CHECK_APPROX(c.z(), 3.0 * m);

    auto e = box.extent();
    CHECK_APPROX(e.x(), 1.0 * m);
    CHECK_APPROX(e.y(), 2.0 * m);
    CHECK_APPROX(e.z(), 3.0 * m);
}

void test_aabb_volume()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 3, 4} * m};
    CHECK_APPROX(box.volume(), 24.0 * m * m * m);
}

void test_aabb_point()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    CHECK_APPROX(box.point(0), box.min);
    CHECK_APPROX(box.point(7), box.max);
    CHECK_APPROX(box.point(1), vec3{1, 0, 0} * m);
    CHECK_APPROX(box.point(5), vec3{1, 0, 1} * m);
}

void test_aabb_contains()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    CHECK(box.contains(vec3{0.5, 0.5, 0.5} * m));
    CHECK(box.contains(vec3{0, 0, 0} * m));
    CHECK(box.contains(vec3{1, 1, 1} * m));
    CHECK(!box.contains(vec3{1.1, 0.5, 0.5} * m));
    CHECK(!box.contains(vec3{-0.1, 0.5, 0.5} * m));
}

void test_aabb_intersects()
{
    aabb a{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    aabb b{.min = vec3{0.5, 0.5, 0.5} * m, .max = vec3{1.5, 1.5, 1.5} * m};
    aabb c{.min = vec3{2, 2, 2} * m, .max = vec3{3, 3, 3} * m};
    CHECK(a.intersects(b));
    CHECK(b.intersects(a));
    CHECK(!a.intersects(c));
    CHECK(!c.intersects(a));
    // Touching at a single corner counts as intersecting
    aabb d{.min = vec3{1, 1, 1} * m, .max = vec3{2, 2, 2} * m};
    CHECK(a.intersects(d));
}

void test_aabb_translate()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 1} * m};
    auto shifted = box + vec3{2.0, 3.0, 4.0} * m;
    CHECK_APPROX(shifted.min, vec3{2.0, 3.0, 4.0} * m);
    CHECK_APPROX(shifted.max, vec3{3.0, 4.0, 5.0} * m);

    auto back = shifted - vec3{2.0, 3.0, 4.0} * m;
    CHECK_APPROX(back.min, box.min);
    CHECK_APPROX(back.max, box.max);
}

void test_aabb_scale()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 2, 2} * m};
    auto scaled = box * 2.0;
    CHECK_APPROX(scaled.min, vec3{-1, -1, -1} * m);
    CHECK_APPROX(scaled.max, vec3{3, 3, 3} * m);

    auto halved = box / 2.0;
    CHECK_APPROX(halved.min, vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(halved.max, vec3{1.5, 1.5, 1.5} * m);

    auto qscaled = box * 2.0;
    CHECK_APPROX(qscaled.min, vec3{-1.0, -1.0, -1.0} * m);
    CHECK_APPROX(qscaled.max, vec3{3.0, 3.0, 3.0} * m);

    auto qdiv = box / 2.0;
    CHECK_APPROX(qdiv.min, vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(qdiv.max, vec3{1.5, 1.5, 1.5} * m);
}

void test_aabb_compound_assign()
{
    aabb box{.min = vec3{0, 0, 0} * m, .max = vec3{2, 2, 2} * m};

    box += vec3{1.0, 1.0, 1.0} * m;
    CHECK_APPROX(box.min, vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(box.max, vec3{3.0, 3.0, 3.0} * m);

    box -= vec3{1.0, 1.0, 1.0} * m;
    CHECK_APPROX(box.min, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(box.max, vec3{2.0, 2.0, 2.0} * m);

    box *= 2.0;
    CHECK_APPROX(box.min, vec3{-1.0, -1.0, -1.0} * m);
    CHECK_APPROX(box.max, vec3{3.0, 3.0, 3.0} * m);

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box /= 2.0;
    CHECK_APPROX(box.min, vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(box.max, vec3{1.5, 1.5, 1.5} * m);

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box *= 2.0;
    CHECK_APPROX(box.min, vec3{-1.0, -1.0, -1.0} * m);
    CHECK_APPROX(box.max, vec3{3.0, 3.0, 3.0} * m);

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box /= 2.0;
    CHECK_APPROX(box.min, vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(box.max, vec3{1.5, 1.5, 1.5} * m);
}

void test_aabb_mat_transform()
{
    aabb box{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};

    auto identity = mat3<one>::identity();
    auto result = box * identity;
    CHECK_APPROX(result, box);

    // 90-degree rotation around Z: (x,y,z) -> (-y,x,z)
    auto rot_z = mat3<one>{
        {0.0, -1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
    };
    auto rotated = box * rot_z;
    CHECK_APPROX(rotated.min, vec3{-1.0, 0.0, 0.0} * m);
    CHECK_APPROX(rotated.max, vec3{0.0, 1.0, 1.0} * m);

    auto rot_q = quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0, 0, 1});
    auto rotated_q = box * rot_q;
    CHECK_APPROX(rotated_q, rotated);
}

// ---------------------------------------------------------------------------
// Bounding sphere tests
// ---------------------------------------------------------------------------

void test_bsphere_from_points()
{
    cube_fixture cube;
    auto sphere = bounding_sphere::from_points(cube.vertices);
    for (const auto &v : cube.vertices)
    {
        auto dist = (v - sphere.center).norm();
        CHECK(dist <= sphere.radius + 1e-9 * m);
    }
}

void test_bsphere_from_aabb()
{
    aabb box{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    auto sphere = bounding_sphere::from_aabb(box);
    CHECK_APPROX(sphere.center, vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(sphere.radius, std::sqrt(3.0) * m);
}

void test_bsphere_merge()
{
    bounding_sphere a{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    bounding_sphere b{.center = vec3{3.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto merged = bounding_sphere::merge(a, b);
    CHECK_APPROX(merged.center.x(), 1.5 * m);
    CHECK_APPROX(merged.center.y(), 0.0 * m);
    CHECK_APPROX(merged.center.z(), 0.0 * m);
    CHECK_APPROX(merged.radius, 2.5 * m);

    // Merging when one encloses the other
    bounding_sphere big{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 10.0 * m};
    bounding_sphere small{.center = vec3{1.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto m1 = bounding_sphere::merge(big, small);
    CHECK_APPROX(m1.radius, big.radius);
    auto m2 = bounding_sphere::merge(small, big);
    CHECK_APPROX(m2.radius, big.radius);
}

void test_bsphere_surface_area_volume()
{
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 2.0 * m};
    CHECK_APPROX(s.surface_area(), 4.0 * std::numbers::pi * 4.0 * m * m);
    CHECK_APPROX(s.volume(), (4.0 / 3.0) * std::numbers::pi * 8.0 * m * m * m);
}

void test_bsphere_contains()
{
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    CHECK(s.contains(vec3{0.0, 0.0, 0.0} * m));
    CHECK(s.contains(vec3{1.0, 0.0, 0.0} * m));
    CHECK(!s.contains(vec3{1.1, 0.0, 0.0} * m));
}

void test_bsphere_intersects_sphere()
{
    bounding_sphere a{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    bounding_sphere b{.center = vec3{1.5, 0.0, 0.0} * m, .radius = 1.0 * m};
    bounding_sphere c{.center = vec3{3.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    CHECK(a.intersects(b));
    CHECK(!a.intersects(c));
}

void test_bsphere_intersects_aabb()
{
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    aabb box{.min = vec3{0.5, 0.5, 0.5} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    CHECK(s.intersects(box));

    aabb far_box{.min = vec3{5.0, 5.0, 5.0} * m, .max = vec3{6.0, 6.0, 6.0} * m};
    CHECK(!s.intersects(far_box));
}

void test_bsphere_translate()
{
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto moved = s + vec3{1.0, 2.0, 3.0} * m;
    CHECK_APPROX(moved.center, vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(moved.radius, 1.0 * m);

    auto back = moved - vec3{1.0, 2.0, 3.0} * m;
    CHECK_APPROX(back.center, vec3{0.0, 0.0, 0.0} * m);
}

void test_bsphere_scale()
{
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    auto scaled = s * 3.0;
    CHECK_APPROX(scaled.radius, 3.0 * m);
    CHECK_APPROX(scaled.center, s.center);

    auto qscaled = s * 2.0;
    CHECK_APPROX(qscaled.radius, 2.0 * m);
}

void test_bsphere_compound_assign()
{
    bounding_sphere s{.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    s += vec3{1.0, 0.0, 0.0} * m;
    CHECK_APPROX(s.center, vec3{1.0, 0.0, 0.0} * m);

    s -= vec3{1.0, 0.0, 0.0} * m;
    CHECK_APPROX(s.center, vec3{0.0, 0.0, 0.0} * m);

    s *= 5.0;
    CHECK_APPROX(s.radius, 5.0 * m);

    s = {.center = vec3{0.0, 0.0, 0.0} * m, .radius = 1.0 * m};
    s *= 3.0;
    CHECK_APPROX(s.radius, 3.0 * m);
}

// ---------------------------------------------------------------------------
// Mesh tests
// ---------------------------------------------------------------------------

void test_mesh_make()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();
    CHECK(msh->vertices().size() == 8);
    CHECK(msh->triangles().size() == 12);
}

void test_mesh_bounds()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();
    CHECK_APPROX(msh->bounds().min, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(msh->bounds().max, vec3{1.0, 1.0, 1.0} * m);
}

void test_mesh_bsphere()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();
    for (const auto &v : cube.vertices)
    {
        auto dist = (v - msh->bsphere().center).norm();
        CHECK(dist <= msh->bsphere().radius + 1e-9 * m);
    }
}

void test_mesh_volume()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();
    CHECK_APPROX(msh->volume(), 1.0 * m * m * m);
}

void test_mesh_ray_intersect()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray from outside hitting the cube
    mesh::ray r{
        .origin = vec3{-1.0, 0.5, 0.5} * m,
        .direction = {1.0, 0.0, 0.0},
    };
    auto hit = msh->ray_intersect(r);
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 0.0 * m);
    CHECK_APPROX(hit->pos.y(), 0.5 * m);
    CHECK_APPROX(hit->pos.z(), 0.5 * m);
    CHECK_APPROX(hit->distance, 1.0 * m);

    // Ray pointing away - no hit
    mesh::ray away{
        .origin = vec3{-1.0, 0.5, 0.5} * m,
        .direction = vec3{-1.0, 0.0, 0.0},
    };
    CHECK(!msh->ray_intersect(away).has_value());

    // Ray with max_distance that is too short
    auto short_hit = msh->ray_intersect(r, 0.5 * m);
    CHECK(!short_hit.has_value());

    // Ray from inside - should still hit
    mesh::ray inside{
        .origin = vec3{0.5, 0.5, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    auto inside_hit = msh->ray_intersect(inside);
    CHECK(inside_hit.has_value());
    CHECK_APPROX(inside_hit->distance, 0.5 * m);
}

void test_mesh_contains()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Use points where y != z to avoid hitting the shared diagonal edge of each face pair,
    // which causes the ray-parity test to double-count crossings.
    CHECK(msh->contains(vec3{0.3, 0.2, 0.4} * m));
    CHECK(msh->contains(vec3{0.7, 0.2, 0.4} * m));
    CHECK(msh->contains(vec3{0.3, 0.7, 0.4} * m));

    // Exterior points
    CHECK(!msh->contains(vec3{-0.5, 0.2, 0.4} * m));
    CHECK(!msh->contains(vec3{1.5, 0.2, 0.4} * m));
    CHECK(!msh->contains(vec3{0.3, -0.5, 0.4} * m));
    CHECK(!msh->contains(vec3{0.3, 0.2, 1.5} * m));
}

void test_mesh_support()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    CHECK_APPROX(msh->support(vec3{1.0, 0.0, 0.0}).x(), 1.0 * m);
    CHECK_APPROX(msh->support(vec3{-1.0, 0.0, 0.0}).x(), 0.0 * m);
    CHECK_APPROX(msh->support(vec3{0.0, 1.0, 0.0}).y(), 1.0 * m);
    CHECK_APPROX(msh->support(vec3{1.0, 1.0, 1.0}), vec3{1.0, 1.0, 1.0} * m);
}

void test_mesh_closest_point()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    //on the corner, edge and face of the cube
    CHECK_APPROX(msh->closest_point(vec3{1.0*m, 1.0*m, 1.0*m}), vec3{1.0*m, 1.0*m, 1.0*m})
    CHECK_APPROX(msh->closest_point(vec3{1.0*m, 0.5*m, 1.0*m}), vec3{1.0*m, 0.5*m, 1.0*m})
    CHECK_APPROX(msh->closest_point(vec3{0.0*m, 0.5*m, 0.5*m}), vec3{0.0*m, 0.5*m, 0.5*m})

    //close to a corner, edge, and face of the cube
    CHECK_APPROX(msh->closest_point(vec3{1.5*m, 1.5*m, 1.5*m}), vec3{1.0*m, 1.0*m, 1.0*m});
    CHECK_APPROX(msh->closest_point(vec3{1.5*m, 1.5*m, 0.5*m}), vec3{1.0*m, 1.0*m, 0.5*m});
    CHECK_APPROX(msh->closest_point(vec3{0.5*m, 0.5*m, 1.5*m}), vec3{0.5*m, 0.5*m, 1.0*m});

    CHECK_APPROX(msh->closest_point(vec3{-0.5*m, -0.5*m, -0.5*m}), vec3{0.0*m, 0.0*m, 0.0*m});
    CHECK_APPROX(msh->closest_point(vec3{-0.5*m, -0.5*m, 0.5*m}), vec3{0.0*m, 0.0*m, 0.5*m});
    CHECK_APPROX(msh->closest_point(vec3{0.5*m, 0.5*m, -0.5*m}), vec3{0.5*m, 0.5*m, 0.0*m});

    //inside of the cube
    CHECK_APPROX(msh->closest_point(vec3{0.5*m, 0.5*m, 0.4*m}), vec3{0.5*m, 0.5*m, 0.0*m});

}
// ---------------------------------------------------------------------------
// Mesh instance tests
// ---------------------------------------------------------------------------

void test_mesh_instance_basic()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();
    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);

    CHECK(&inst.geometry() == msh.get());
    CHECK_APPROX(inst.position(), vec3{5.0, 0.0, 0.0} * m);
}

void test_mesh_instance_world_bounds()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    auto wb = inst.bounds();
    CHECK_APPROX(wb.min, vec3{5.0, 0.0, 0.0} * m);
    CHECK_APPROX(wb.max, vec3{6.0, 1.0, 1.0} * m);
}

void test_mesh_instance_world_bsphere()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    auto ws = inst.bsphere();
    auto local_center = msh->bsphere().center;
    CHECK_APPROX(ws.center, local_center + vec3{5.0, 0.0, 0.0} * m);
    CHECK_APPROX(ws.radius, msh->bsphere().radius);
}

void test_mesh_instance_ray_intersect()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);

    mesh::ray r{
        .origin = vec3{3.0, 0.5, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    auto hit = inst.ray_intersect(r);
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 5.0 * m);
    CHECK_APPROX(hit->distance, 2.0 * m);

    mesh::ray miss{
        .origin = vec3{3.0, 5.0, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    CHECK(!inst.ray_intersect(miss).has_value());
}

void test_mesh_instance_contains()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    CHECK(inst.contains(vec3{5.3, 0.2, 0.4} * m));
    CHECK(!inst.contains(vec3{0.3, 0.2, 0.4} * m));
}

void test_mesh_instance_support()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    CHECK_APPROX(inst.support(vec3{1.0, 0.0, 0.0}).x(), 6.0 * m);
}

void test_mesh_instance_rotated()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // 90-degree rotation around Z: (x,y,z) -> (-y,x,z)
    auto rot_z =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});

    auto inst = msh->at(vec3{0.0, 0.0, 0.0} * m, rot_z);

    // After rotating the unit cube [0,1]^3 by 90 degrees around Z,
    // the AABB becomes [-1,0] x [0,1] x [0,1]
    auto wb = inst.bounds();
    CHECK_APPROX(wb.min.x(), -1.0 * m);
    CHECK_APPROX(wb.max.x(), 0.0 * m);

    // Ray in +x should hit at x = -1
    mesh::ray r{
        .origin = vec3{-3.0, 0.5, 0.5} * m,
        .direction = vec3{1.0, 0.0, 0.0},
    };
    auto hit = inst.ray_intersect(r);
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), -1.0 * m);
}

void test_mesh_instance_different_positions()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst1 = msh->at(vec3{10.0, 0.0, 0.0} * m);
    CHECK(inst1.contains(vec3{10.3, 0.2, 0.4} * m));

    auto inst2 = msh->at(vec3{0.0, 0.0, 0.0} * m);
    CHECK(inst2.contains(vec3{0.3, 0.2, 0.4} * m));
}

int main()
{
    suite tests;

    tests.group("AABB Tests")
        .test("from_points", test_aabb_from_points)
        .test("size/center/extent", test_aabb_size_center_extent)
        .test("volume", test_aabb_volume)
        .test("point", test_aabb_point)
        .test("contains", test_aabb_contains)
        .test("intersects", test_aabb_intersects)
        .test("operator+/operator-", test_aabb_translate)
        .test("operator*/operator/", test_aabb_scale)
        .test("operator+=/operator-=/operator*=/operator/=", test_aabb_compound_assign)
        .test("operator*(mat3/quat)", test_aabb_mat_transform);

    tests.group("Bounding Sphere Tests")
        .test("from_points", test_bsphere_from_points)
        .test("from_aabb", test_bsphere_from_aabb)
        .test("merge", test_bsphere_merge)
        .test("surface_area/volume", test_bsphere_surface_area_volume)
        .test("contains", test_bsphere_contains)
        .test("intersects(sphere)", test_bsphere_intersects_sphere)
        .test("intersects(aabb)", test_bsphere_intersects_aabb)
        .test("operator+/operator-", test_bsphere_translate)
        .test("operator*", test_bsphere_scale)
        .test("operator+=/operator-=/operator*=", test_bsphere_compound_assign);

    tests.group("Mesh Tests")
        .test("make", test_mesh_make)
        .test("bounds", test_mesh_bounds)
        .test("bsphere", test_mesh_bsphere)
        .test("volume", test_mesh_volume)
        .test("ray_intersect", test_mesh_ray_intersect)
        .test("contains", test_mesh_contains)
        .test("support", test_mesh_support)
        .test("closest_point", test_mesh_closest_point);

    tests.group("Mesh Instance Tests")
        .test("basic construction", test_mesh_instance_basic)
        .test("world_bounds", test_mesh_instance_world_bounds)
        .test("world_bsphere", test_mesh_instance_world_bsphere)
        .test("ray_intersect", test_mesh_instance_ray_intersect)
        .test("contains", test_mesh_instance_contains)
        .test("support", test_mesh_instance_support)
        .test("rotated ray_intersect", test_mesh_instance_rotated)
        .test("different positions", test_mesh_instance_different_positions);

    return tests.run();
}
