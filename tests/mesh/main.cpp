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
    CHECK_APPROX(scaled.min, vec3{0, 0, 0} * m);
    CHECK_APPROX(scaled.max, vec3{4, 4, 4} * m);

    auto halved = box / 2.0;
    CHECK_APPROX(halved.min, vec3{0, 0, 0} * m);
    CHECK_APPROX(halved.max, vec3{1, 1, 1} * m);
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
    CHECK_APPROX(box.min, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(box.max, vec3{4.0, 4.0, 4.0} * m);

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box /= 2.0;
    CHECK_APPROX(box.min, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(box.max, vec3{1.0, 1.0, 1.0} * m);

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box *= 2.0;
    CHECK_APPROX(box.min, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(box.max, vec3{4.0, 4.0, 4.0} * m);

    box = {.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    box /= 2.0;
    CHECK_APPROX(box.min, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(box.max, vec3{1.0, 1.0, 1.0} * m);
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

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_mesh_contains()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // --- Interior points ---

    // Center
    CHECK(msh->contains(vec3{0.5, 0.5, 0.5} * m));

    // General interior points
    CHECK(msh->contains(vec3{0.3, 0.2, 0.4} * m));
    CHECK(msh->contains(vec3{0.7, 0.2, 0.4} * m));
    CHECK(msh->contains(vec3{0.3, 0.7, 0.4} * m));

    // Points along mesh face diagonals (y == z lands on shared triangle edges)
    CHECK(msh->contains(vec3{0.5, 0.3, 0.3} * m)); // y == z
    CHECK(msh->contains(vec3{0.5, 0.6, 0.6} * m)); // y == z
    CHECK(msh->contains(vec3{0.2, 0.4, 0.4} * m)); // y == z

    // Points along the space diagonal (x == y == z)
    CHECK(msh->contains(vec3{0.2, 0.2, 0.2} * m));
    CHECK(msh->contains(vec3{0.8, 0.8, 0.8} * m));

    // Near each face (just inside)
    constexpr auto eps = 0.01;
    CHECK(msh->contains(vec3{eps, 0.5, 0.5} * m));       // near -X face
    CHECK(msh->contains(vec3{1.0 - eps, 0.5, 0.5} * m)); // near +X face
    CHECK(msh->contains(vec3{0.5, eps, 0.5} * m));       // near -Y face
    CHECK(msh->contains(vec3{0.5, 1.0 - eps, 0.5} * m)); // near +Y face
    CHECK(msh->contains(vec3{0.5, 0.5, eps} * m));       // near -Z face
    CHECK(msh->contains(vec3{0.5, 0.5, 1.0 - eps} * m)); // near +Z face

    // Near corners (just inside)
    CHECK(msh->contains(vec3{eps, eps, eps} * m));
    CHECK(msh->contains(vec3{1.0 - eps, 1.0 - eps, 1.0 - eps} * m));
    CHECK(msh->contains(vec3{eps, 1.0 - eps, eps} * m));
    CHECK(msh->contains(vec3{1.0 - eps, eps, 1.0 - eps} * m));

    // --- Exterior points ---

    // One side of each face
    CHECK(!msh->contains(vec3{-0.5, 0.2, 0.4} * m)); // -X side
    CHECK(!msh->contains(vec3{1.5, 0.2, 0.4} * m));  // +X side
    CHECK(!msh->contains(vec3{0.3, -0.5, 0.4} * m)); // -Y side
    CHECK(!msh->contains(vec3{0.3, 1.5, 0.4} * m));  // +Y side
    CHECK(!msh->contains(vec3{0.3, 0.2, -0.5} * m)); // -Z side
    CHECK(!msh->contains(vec3{0.3, 0.2, 1.5} * m));  // +Z side

    // Just outside each face
    CHECK(!msh->contains(vec3{-eps, 0.5, 0.5} * m));
    CHECK(!msh->contains(vec3{1.0 + eps, 0.5, 0.5} * m));
    CHECK(!msh->contains(vec3{0.5, -eps, 0.5} * m));
    CHECK(!msh->contains(vec3{0.5, 1.0 + eps, 0.5} * m));
    CHECK(!msh->contains(vec3{0.5, 0.5, -eps} * m));
    CHECK(!msh->contains(vec3{0.5, 0.5, 1.0 + eps} * m));

    // Outside corner and edge regions
    CHECK(!msh->contains(vec3{-0.5, -0.5, -0.5} * m)); // corner region
    CHECK(!msh->contains(vec3{1.5, 1.5, 1.5} * m));    // corner region
    CHECK(!msh->contains(vec3{1.5, -0.5, 0.5} * m));   // edge region
    CHECK(!msh->contains(vec3{-0.5, 0.5, 1.5} * m));   // edge region

    // Far exterior
    CHECK(!msh->contains(vec3{-10.0, 0.5, 0.5} * m));
    CHECK(!msh->contains(vec3{10.0, 0.5, 0.5} * m));
    CHECK(!msh->contains(vec3{0.5, 0.5, -10.0} * m));
    CHECK(!msh->contains(vec3{0.5, 0.5, 10.0} * m));

    // --- Factory mesh: mesh::box (half-extents = 1 → range [-1, 1]^3) ---
    {
        auto box_msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);

        CHECK(box_msh->contains(vec3{0.0, 0.0, 0.0} * m)); // center
        CHECK(box_msh->contains(vec3{0.5, 0.5, 0.5} * m));
        CHECK(box_msh->contains(vec3{-0.5, -0.5, 0.0} * m));
        CHECK(box_msh->contains(vec3{0.0, 0.0, 0.9} * m));  // near +Z face
        CHECK(box_msh->contains(vec3{0.0, -0.9, 0.0} * m)); // near -Y face

        CHECK(!box_msh->contains(vec3{1.5, 0.0, 0.0} * m));    // outside +X
        CHECK(!box_msh->contains(vec3{0.0, -1.5, 0.0} * m));   // outside -Y
        CHECK(!box_msh->contains(vec3{-2.0, -2.0, -2.0} * m)); // far corner
    }

    // --- Factory mesh: mesh::sphere ---
    {
        auto sph_msh = mesh::sphere(1.0 * m, 32, 32);

        CHECK(sph_msh->contains(vec3{0.0, 0.0, 0.0} * m)); // center
        CHECK(sph_msh->contains(vec3{0.5, 0.0, 0.0} * m));
        CHECK(sph_msh->contains(vec3{0.0, 0.5, 0.5} * m));

        CHECK(!sph_msh->contains(vec3{2.0, 0.0, 0.0} * m));  // outside
        CHECK(!sph_msh->contains(vec3{0.8, 0.8, 0.8} * m));  // outside (diagonal)
        CHECK(!sph_msh->contains(vec3{-5.0, 0.0, 0.0} * m)); // far outside
    }

    // --- Translated instance ---
    {
        auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);

        CHECK(inst.contains(vec3{5.5, 0.5, 0.5} * m));       // center of translated cube
        CHECK(inst.contains(vec3{5.0 + eps, 0.5, 0.5} * m)); // just inside -X face
        CHECK(inst.contains(vec3{5.5, 0.3, 0.3} * m));       // along diagonal

        CHECK(!inst.contains(vec3{0.5, 0.5, 0.5} * m)); // original location
        CHECK(!inst.contains(vec3{4.5, 0.5, 0.5} * m)); // just outside -X face
        CHECK(!inst.contains(vec3{6.5, 0.5, 0.5} * m)); // just outside +X face
    }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_all_faces()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Hit each face from outside along the corresponding axis
    struct face_case
    {
        vec3<m> origin;
        vec3<one> dir;
        vec3<m> expected_pos;
        quantity<m> expected_dist;
    };

    std::array cases{
        // -X face
        face_case{.origin = vec3{-2.0, 0.5, 0.5} * m,
                  .dir = {1, 0, 0},
                  .expected_pos = vec3{0.0, 0.5, 0.5} * m,
                  .expected_dist = 2.0 * m},
        // +X face
        face_case{.origin = vec3{3.0, 0.5, 0.5} * m,
                  .dir = {-1, 0, 0},
                  .expected_pos = vec3{1.0, 0.5, 0.5} * m,
                  .expected_dist = 2.0 * m},
        // -Y face
        face_case{.origin = vec3{0.5, -2.0, 0.5} * m,
                  .dir = {0, 1, 0},
                  .expected_pos = vec3{0.5, 0.0, 0.5} * m,
                  .expected_dist = 2.0 * m},
        // +Y face
        face_case{.origin = vec3{0.5, 3.0, 0.5} * m,
                  .dir = {0, -1, 0},
                  .expected_pos = vec3{0.5, 1.0, 0.5} * m,
                  .expected_dist = 2.0 * m},
        // -Z face
        face_case{.origin = vec3{0.5, 0.5, -2.0} * m,
                  .dir = {0, 0, 1},
                  .expected_pos = vec3{0.5, 0.5, 0.0} * m,
                  .expected_dist = 2.0 * m},
        // +Z face
        face_case{.origin = vec3{0.5, 0.5, 3.0} * m,
                  .dir = {0, 0, -1},
                  .expected_pos = vec3{0.5, 0.5, 1.0} * m,
                  .expected_dist = 2.0 * m},
    };

    for (const auto &[origin, dir, expected_pos, expected_dist] : cases)
    {
        auto hit = msh->ray_intersect({.origin = origin, .direction = dir});
        CHECK(hit.has_value());
        CHECK_APPROX(hit->pos, expected_pos);
        CHECK_APPROX(hit->distance, expected_dist);
    }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_diagonal()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Diagonal ray aimed at the cube center from a corner direction
    auto origin = vec3{-1.0, -1.0, -1.0} * m;
    auto dir = vec3{1.0, 1.0, 1.0}.normalized();
    auto hit = msh->ray_intersect({.origin = origin, .direction = dir});
    CHECK(hit.has_value());
    // Should hit the corner region; distance from origin to (0,0,0) is sqrt(3)
    CHECK_APPROX(hit->distance, std::sqrt(3.0) * m);

    // Diagonal from opposite corner
    auto origin2 = vec3{2.0, 2.0, 2.0} * m;
    auto dir2 = vec3{-1.0, -1.0, -1.0}.normalized();
    auto hit2 = msh->ray_intersect({.origin = origin2, .direction = dir2});
    CHECK(hit2.has_value());
    CHECK_APPROX(hit2->distance, std::sqrt(3.0) * m);
}

void test_ray_intersect_miss_parallel()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray parallel to +X face, passing just outside
    CHECK(!msh->ray_intersect({.origin = vec3{0.5, 1.1, 0.5} * m, .direction = {1, 0, 0}})
               .has_value());
    CHECK(!msh->ray_intersect({.origin = vec3{0.5, -0.1, 0.5} * m, .direction = {1, 0, 0}})
               .has_value());

    // Ray parallel to Z axis, passing outside in X
    CHECK(!msh->ray_intersect({.origin = vec3{1.5, 0.5, -2.0} * m, .direction = {0, 0, 1}})
               .has_value());

    // Ray parallel to Y axis, passing outside in Z
    CHECK(!msh->ray_intersect({.origin = vec3{0.5, -2.0, 1.5} * m, .direction = {0, 1, 0}})
               .has_value());
}

void test_ray_intersect_miss_wrong_direction()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray pointing away from cube on each axis
    CHECK(!msh->ray_intersect({.origin = vec3{-1.0, 0.5, 0.5} * m, .direction = {-1, 0, 0}})
               .has_value());
    CHECK(!msh->ray_intersect({.origin = vec3{0.5, -1.0, 0.5} * m, .direction = {0, -1, 0}})
               .has_value());
    CHECK(!msh->ray_intersect({.origin = vec3{0.5, 0.5, -1.0} * m, .direction = {0, 0, -1}})
               .has_value());
    CHECK(!msh->ray_intersect({.origin = vec3{2.0, 0.5, 0.5} * m, .direction = {1, 0, 0}})
               .has_value());
}

void test_ray_intersect_from_inside()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // From center, should hit at distance 0.5 along each axis
    auto center = vec3{0.5, 0.5, 0.5} * m;
    for (const auto &dir : {vec3<one>{1, 0, 0}, vec3<one>{-1, 0, 0}, vec3<one>{0, 1, 0},
                            vec3<one>{0, -1, 0}, vec3<one>{0, 0, 1}, vec3<one>{0, 0, -1}})
    {
        auto hit = msh->ray_intersect({.origin = center, .direction = dir});
        CHECK(hit.has_value());
        CHECK_APPROX(hit->distance, 0.5 * m);
    }

    // From off-center interior point
    auto hit = msh->ray_intersect({.origin = vec3{0.2, 0.5, 0.5} * m, .direction = {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->distance, 0.2 * m);
    CHECK_APPROX(hit->pos.x(), 0.0 * m);

    auto hit2 = msh->ray_intersect({.origin = vec3{0.2, 0.5, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit2.has_value());
    CHECK_APPROX(hit2->distance, 0.8 * m);
    CHECK_APPROX(hit2->pos.x(), 1.0 * m);
}

void test_ray_intersect_max_distance()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    mesh::ray r{.origin = vec3{-5.0, 0.5, 0.5} * m, .direction = {1, 0, 0}};

    // Too short to reach
    CHECK(!msh->ray_intersect(r, 4.0 * m).has_value());

    // Exactly at the boundary
    CHECK(msh->ray_intersect(r, 5.0 * m).has_value());

    // Well past the cube
    CHECK(msh->ray_intersect(r, 100.0 * m).has_value());

    // From closer, restricting to first face only (should not reach back face)
    mesh::ray r2{.origin = vec3{-1.0, 0.5, 0.5} * m, .direction = {1, 0, 0}};
    auto hit = msh->ray_intersect(r2, 1.5 * m);
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 0.0 * m); // front face, not back face
}

void test_ray_intersect_hit_normal()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Hit -X face: normal should point in -X
    auto hit_nx = msh->ray_intersect({.origin = vec3{-2.0, 0.5, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit_nx.has_value());
    CHECK_APPROX(hit_nx->normal, vec3<one>{-1, 0, 0});

    // Hit +X face: normal should point in +X
    auto hit_px = msh->ray_intersect({.origin = vec3{3.0, 0.5, 0.5} * m, .direction = {-1, 0, 0}});
    CHECK(hit_px.has_value());
    CHECK_APPROX(hit_px->normal, vec3<one>{1, 0, 0});

    // Hit +Z face: normal should point in +Z
    auto hit_pz = msh->ray_intersect({.origin = vec3{0.5, 0.5, 3.0} * m, .direction = {0, 0, -1}});
    CHECK(hit_pz.has_value());
    CHECK_APPROX(hit_pz->normal, vec3<one>{0, 0, 1});

    // Hit -Y face
    auto hit_ny = msh->ray_intersect({.origin = vec3{0.5, -2.0, 0.5} * m, .direction = {0, 1, 0}});
    CHECK(hit_ny.has_value());
    CHECK_APPROX(hit_ny->normal, vec3<one>{0, -1, 0});
}

void test_ray_intersect_edge_hit()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray aimed exactly at an edge of the cube (where two faces meet)
    // Should still register a hit on one of the adjacent triangles
    auto hit = msh->ray_intersect({.origin = vec3{-1.0, 0.0, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 0.0 * m);
    CHECK_APPROX(hit->pos.y(), 0.0 * m);

    // Ray aimed at a vertex
    auto hit_v = msh->ray_intersect({.origin = vec3{-1.0, 0.0, 0.0} * m, .direction = {1, 0, 0}});
    CHECK(hit_v.has_value());
    CHECK_APPROX(hit_v->pos, vec3{0.0, 0.0, 0.0} * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_grazing()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Nearly parallel to +Z face: ray just barely enters the cube
    auto dir = vec3{1.0, 0.0, 0.001}.normalized();
    auto hit = msh->ray_intersect({.origin = vec3{-1.0, 0.5, 0.5} * m, .direction = dir});
    CHECK(hit.has_value());
    CHECK(hit->pos.x() >= -1e-6 * m); // should hit on or near the -X face

    // Nearly parallel but just misses (slightly outside)
    auto miss_dir = vec3{1.0, 0.0, 0.0};
    auto miss = msh->ray_intersect({.origin = vec3{-1.0, 0.5, 1.0001} * m, .direction = miss_dir});
    CHECK(!miss.has_value());
}

void test_ray_intersect_zero_distance()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray origin exactly on the -X face surface, pointing inward
    auto hit = msh->ray_intersect({.origin = vec3{0.0, 0.5, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit.has_value());
    // Should hit the far +X face at distance 1, or the near face at distance ~0
    CHECK(hit->distance <= 1.0 * m + 1e-9 * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_sphere()
{
    auto msh = mesh::sphere(1.0 * m, 32, 32);

    // Along +X axis from outside
    auto hit = msh->ray_intersect({.origin = vec3{5.0, 0.0, 0.0} * m, .direction = {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK(std::abs(hit->pos.x().numerical_value_in(si::metre) - 1.0) < 0.05);
    CHECK(std::abs(hit->pos.y().numerical_value_in(si::metre)) < 0.05);
    CHECK(std::abs(hit->pos.z().numerical_value_in(si::metre)) < 0.05);
    CHECK(std::abs(hit->distance.numerical_value_in(si::metre) - 4.0) < 0.05);

    // Normal at +X pole should point roughly in +X
    CHECK(static_cast<double>(hit->normal.x()) > 0.9);

    // Along -Y axis from below
    auto hit_y = msh->ray_intersect({.origin = vec3{0.0, -5.0, 0.0} * m, .direction = {0, 1, 0}});
    CHECK(hit_y.has_value());
    CHECK(std::abs(hit_y->pos.y().numerical_value_in(si::metre) - (-1.0)) < 0.05);

    // Miss: ray passing beside the sphere
    CHECK(!msh->ray_intersect({.origin = vec3{0.0, 2.0, 0.0} * m, .direction = {1, 0, 0}})
               .has_value());
}

void test_ray_intersect_pyramid()
{
    auto msh = mesh::pyramid(1.0 * m, 2.0 * m);
    // Base at y=0, apex at (0, 2, 0)

    // Ray from below hitting the base
    auto hit = msh->ray_intersect({.origin = vec3{0.0, -1.0, 0.0} * m, .direction = {0, 1, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.y(), 0.0 * m);

    // Ray from above hitting the apex
    auto hit_top = msh->ray_intersect({.origin = vec3{0.0, 5.0, 0.0} * m, .direction = {0, -1, 0}});
    CHECK(hit_top.has_value());
    CHECK_APPROX(hit_top->pos, vec3{0.0, 2.0, 0.0} * m);

    // Ray from the side hitting a lateral face
    auto hit_side =
        msh->ray_intersect({.origin = vec3{5.0, 1.0, 0.0} * m, .direction = {-1, 0, 0}});
    CHECK(hit_side.has_value());
    CHECK(hit_side->pos.x() > 0.0 * m);
    CHECK(hit_side->pos.x() < 1.0 * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_returns_nearest()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Ray passes through the entire cube: should return the FIRST (nearest) hit
    mesh::ray r{.origin = vec3{-2.0, 0.5, 0.5} * m, .direction = {1, 0, 0}};
    auto hit = msh->ray_intersect(r);
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 0.0 * m); // front face, not back face at x=1
    CHECK_APPROX(hit->distance, 2.0 * m);

    // Same from the other direction
    mesh::ray r2{.origin = vec3{3.0, 0.5, 0.5} * m, .direction = {-1, 0, 0}};
    auto hit2 = msh->ray_intersect(r2);
    CHECK(hit2.has_value());
    CHECK_APPROX(hit2->pos.x(), 1.0 * m); // front face from +X side
    CHECK_APPROX(hit2->distance, 2.0 * m);
}

void test_ray_intersect_unnormalized_direction()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Direction vector with length != 1; ray_intersect normalizes internally
    mesh::ray r{.origin = vec3{-2.0, 0.5, 0.5} * m, .direction = {10, 0, 0}};
    auto hit = msh->ray_intersect(r);
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, vec3{0.0, 0.5, 0.5} * m);
    // Distance should be the actual geometric distance, not scaled by direction length
    CHECK_APPROX(hit->distance, 2.0 * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_instance_translated()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    auto inst = msh->at(vec3{10.0, 0.0, 0.0} * m);

    // Hit from the left
    auto hit = inst.ray_intersect({.origin = vec3{5.0, 0.5, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 10.0 * m);
    CHECK_APPROX(hit->distance, 5.0 * m);

    // Miss at original location
    CHECK(!inst.ray_intersect({.origin = vec3{-2.0, 0.5, 0.5} * m, .direction = {1, 0, 0}}, 5.0 * m)
               .has_value());

    // From inside the translated cube
    auto hit_in = inst.ray_intersect({.origin = vec3{10.5, 0.5, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit_in.has_value());
    CHECK_APPROX(hit_in->pos.x(), 11.0 * m);
    CHECK_APPROX(hit_in->distance, 0.5 * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_ray_intersect_instance_rotated()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // 90-degree rotation around Z: (x,y,z) -> (-y,x,z)
    // Unit cube [0,1]^3 becomes [-1,0] x [0,1] x [0,1]
    auto rot_z =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    auto inst = msh->at(vec3{0.0, 0.0, 0.0} * m, rot_z);

    // Ray from -X side
    auto hit = inst.ray_intersect({.origin = vec3{-3.0, 0.5, 0.5} * m, .direction = {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), -1.0 * m);
    CHECK_APPROX(hit->distance, 2.0 * m);

    // Ray from +X side should hit x=0 face
    auto hit2 = inst.ray_intersect({.origin = vec3{3.0, 0.5, 0.5} * m, .direction = {-1, 0, 0}});
    CHECK(hit2.has_value());
    CHECK_APPROX(hit2->pos.x(), 0.0 * m);
    CHECK_APPROX(hit2->distance, 3.0 * m);
}

void test_ray_intersect_centered_box()
{
    auto msh = mesh::box(vec3{1.0, 2.0, 3.0} * m); // half-extents -> [-1,1] x [-2,2] x [-3,3]

    // Hit from +X
    auto hit = msh->ray_intersect({.origin = vec3{5.0, 0.0, 0.0} * m, .direction = {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 1.0 * m);
    CHECK_APPROX(hit->distance, 4.0 * m);

    // Hit from -Y (half-extent is 2 in Y)
    auto hit_y = msh->ray_intersect({.origin = vec3{0.0, -5.0, 0.0} * m, .direction = {0, 1, 0}});
    CHECK(hit_y.has_value());
    CHECK_APPROX(hit_y->pos.y(), -2.0 * m);
    CHECK_APPROX(hit_y->distance, 3.0 * m);

    // Hit from +Z (half-extent is 3 in Z)
    auto hit_z = msh->ray_intersect({.origin = vec3{0.0, 0.0, 10.0} * m, .direction = {0, 0, -1}});
    CHECK(hit_z.has_value());
    CHECK_APPROX(hit_z->pos.z(), 3.0 * m);
    CHECK_APPROX(hit_z->distance, 7.0 * m);
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

    // on the corner, edge and face of the cube
    CHECK_APPROX(msh->closest_point(vec3{1.0, 1.0, 1.0} * m), vec3{1.0, 1.0, 1.0} * m)
    CHECK_APPROX(msh->closest_point(vec3{1.0, 0.5, 1.0} * m), vec3{1.0, 0.5, 1.0} * m)
    CHECK_APPROX(msh->closest_point(vec3{0.0, 0.5, 0.5} * m), vec3{0.0, 0.5, 0.5} * m)

    // close to a corner, edge, and face of the cube
    CHECK_APPROX(msh->closest_point(vec3{1.5, 1.5, 1.5} * m), vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{1.5, 1.5, 0.5} * m), vec3{1.0, 1.0, 0.5} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 1.5} * m), vec3{0.5, 0.5, 1.0} * m);

    CHECK_APPROX(msh->closest_point(vec3{-0.5, -0.5, -0.5} * m), vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{-0.5, -0.5, 0.5} * m), vec3{0.0, 0.0, 0.5} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, -0.5} * m), vec3{0.5, 0.5, 0.0} * m);

    // inside of the cube
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 0.4} * m), vec3{0.5, 0.5, 0.0} * m);
}

// Exterior point projects onto each of the 6 cube faces
void test_closest_point_all_faces()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // +X face: point outside at x=2, should project to x=1
    CHECK_APPROX(msh->closest_point(vec3{2.0, 0.5, 0.5} * m), vec3{1.0, 0.5, 0.5} * m);
    // -X face: point outside at x=-1, should project to x=0
    CHECK_APPROX(msh->closest_point(vec3{-1.0, 0.5, 0.5} * m), vec3{0.0, 0.5, 0.5} * m);
    // +Y face: point outside at y=2, should project to y=1
    CHECK_APPROX(msh->closest_point(vec3{0.5, 2.0, 0.5} * m), vec3{0.5, 1.0, 0.5} * m);
    // -Y face: point outside at y=-1, should project to y=0
    CHECK_APPROX(msh->closest_point(vec3{0.5, -1.0, 0.5} * m), vec3{0.5, 0.0, 0.5} * m);
    // +Z face: point outside at z=2, should project to z=1
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 2.0} * m), vec3{0.5, 0.5, 1.0} * m);
    // -Z face: point outside at z=-1, should project to z=0
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, -1.0} * m), vec3{0.5, 0.5, 0.0} * m);
}

// Exterior points closest to each of the 12 edges of the unit cube
void test_closest_point_all_edges()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Edges parallel to X-axis (4 edges)
    // y=0,z=0 edge
    CHECK_APPROX(msh->closest_point(vec3{0.5, -1.0, -1.0} * m), vec3{0.5, 0.0, 0.0} * m);
    // y=1,z=0 edge
    CHECK_APPROX(msh->closest_point(vec3{0.5, 2.0, -1.0} * m), vec3{0.5, 1.0, 0.0} * m);
    // y=0,z=1 edge
    CHECK_APPROX(msh->closest_point(vec3{0.5, -1.0, 2.0} * m), vec3{0.5, 0.0, 1.0} * m);
    // y=1,z=1 edge
    CHECK_APPROX(msh->closest_point(vec3{0.5, 2.0, 2.0} * m), vec3{0.5, 1.0, 1.0} * m);

    // Edges parallel to Y-axis (4 edges)
    // x=0,z=0 edge
    CHECK_APPROX(msh->closest_point(vec3{-1.0, 0.5, -1.0} * m), vec3{0.0, 0.5, 0.0} * m);
    // x=1,z=0 edge
    CHECK_APPROX(msh->closest_point(vec3{2.0, 0.5, -1.0} * m), vec3{1.0, 0.5, 0.0} * m);
    // x=0,z=1 edge
    CHECK_APPROX(msh->closest_point(vec3{-1.0, 0.5, 2.0} * m), vec3{0.0, 0.5, 1.0} * m);
    // x=1,z=1 edge
    CHECK_APPROX(msh->closest_point(vec3{2.0, 0.5, 2.0} * m), vec3{1.0, 0.5, 1.0} * m);

    // Edges parallel to Z-axis (4 edges)
    // x=0,y=0 edge
    CHECK_APPROX(msh->closest_point(vec3{-1.0, -1.0, 0.5} * m), vec3{0.0, 0.0, 0.5} * m);
    // x=1,y=0 edge
    CHECK_APPROX(msh->closest_point(vec3{2.0, -1.0, 0.5} * m), vec3{1.0, 0.0, 0.5} * m);
    // x=0,y=1 edge
    CHECK_APPROX(msh->closest_point(vec3{-1.0, 2.0, 0.5} * m), vec3{0.0, 1.0, 0.5} * m);
    // x=1,y=1 edge
    CHECK_APPROX(msh->closest_point(vec3{2.0, 2.0, 0.5} * m), vec3{1.0, 1.0, 0.5} * m);
}

// Exterior points closest to each of the 8 corners of the unit cube
void test_closest_point_all_corners()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Each corner approached from a diagonal direction
    CHECK_APPROX(msh->closest_point(vec3{-1.0, -1.0, -1.0} * m), vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{2.0, -1.0, -1.0} * m), vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{2.0, 2.0, -1.0} * m), vec3{1.0, 1.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{-1.0, 2.0, -1.0} * m), vec3{0.0, 1.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{-1.0, -1.0, 2.0} * m), vec3{0.0, 0.0, 1.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{2.0, -1.0, 2.0} * m), vec3{1.0, 0.0, 1.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{2.0, 2.0, 2.0} * m), vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{-1.0, 2.0, 2.0} * m), vec3{0.0, 1.0, 1.0} * m);
}

// Interior points nearest to each face
void test_closest_point_interior_all_faces()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Closest to -X face (x=0)
    CHECK_APPROX(msh->closest_point(vec3{0.1, 0.5, 0.5} * m), vec3{0.0, 0.5, 0.5} * m);
    // Closest to +X face (x=1)
    CHECK_APPROX(msh->closest_point(vec3{0.9, 0.5, 0.5} * m), vec3{1.0, 0.5, 0.5} * m);
    // Closest to -Y face (y=0)
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.1, 0.5} * m), vec3{0.5, 0.0, 0.5} * m);
    // Closest to +Y face (y=1)
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.9, 0.5} * m), vec3{0.5, 1.0, 0.5} * m);
    // Closest to -Z face (z=0)
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 0.1} * m), vec3{0.5, 0.5, 0.0} * m);
    // Closest to +Z face (z=1)
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 0.9} * m), vec3{0.5, 0.5, 1.0} * m);
}

// Points very far from the mesh
void test_closest_point_far_away()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Very far along +X axis -> projects to +X face
    CHECK_APPROX(msh->closest_point(vec3{1000.0, 0.5, 0.5} * m), vec3{1.0, 0.5, 0.5} * m);
    // Very far diagonally -> projects to corner
    CHECK_APPROX(msh->closest_point(vec3{100.0, 100.0, 100.0} * m), vec3{1.0, 1.0, 1.0} * m);
    // Very far along -Y axis -> projects to -Y face
    CHECK_APPROX(msh->closest_point(vec3{0.3, -500.0, 0.7} * m), vec3{0.3, 0.0, 0.7} * m);
}

// Points very close to the surface (just barely inside/outside)
void test_closest_point_near_surface()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();
    constexpr auto delta = 0.001;

    // Just outside +Z face
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, (1.0 + delta)} * m), vec3{0.5, 0.5, 1.0} * m);
    // Just inside +Z face -> closest is +Z face
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, (1.0 - delta)} * m), vec3{0.5, 0.5, 1.0} * m);
    // Just outside -X face
    CHECK_APPROX(msh->closest_point(vec3{-delta, 0.5, 0.5} * m), vec3{0.0, 0.5, 0.5} * m);
    // Just inside -X face -> closest is -X face
    CHECK_APPROX(msh->closest_point(vec3{delta, 0.5, 0.5} * m), vec3{0.0, 0.5, 0.5} * m);
}

// Points exactly on surface features
void test_closest_point_on_surface()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Exact face center on each face
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 0.0} * m), vec3{0.5, 0.5, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.5, 1.0} * m), vec3{0.5, 0.5, 1.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.0, 0.5} * m), vec3{0.5, 0.0, 0.5} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.5, 1.0, 0.5} * m), vec3{0.5, 1.0, 0.5} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.0, 0.5, 0.5} * m), vec3{0.0, 0.5, 0.5} * m);
    CHECK_APPROX(msh->closest_point(vec3{1.0, 0.5, 0.5} * m), vec3{1.0, 0.5, 0.5} * m);

    // Edge midpoints
    CHECK_APPROX(msh->closest_point(vec3{0.5, 0.0, 0.0} * m), vec3{0.5, 0.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.0, 0.5, 0.0} * m), vec3{0.0, 0.5, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.0, 0.0, 0.5} * m), vec3{0.0, 0.0, 0.5} * m);

    // All 8 vertices
    for (const auto &v : cube.vertices) CHECK_APPROX(msh->closest_point(v), v);
}

// Non-axis-aligned exterior: approach from arbitrary diagonal
void test_closest_point_diagonal_approach()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Point at (2, 0.5, 0.5): directly outside +X face
    CHECK_APPROX(msh->closest_point(vec3{2.0, 0.5, 0.5} * m), vec3{1.0, 0.5, 0.5} * m);

    // Point at (1.5, 0.5, 1.5): outside the +X/+Z edge region
    // Closest is on the edge at x=1, z=1
    CHECK_APPROX(msh->closest_point(vec3{1.5, 0.5, 1.5} * m), vec3{1.0, 0.5, 1.0} * m);

    // Point at (1.3, -0.3, 0.5): outside in the +X/-Y edge region
    CHECK_APPROX(msh->closest_point(vec3{1.3, -0.3, 0.5} * m), vec3{1.0, 0.0, 0.5} * m);
}

// Verify distance is minimized: the returned closest point should be
// nearer than other surface sample points for various query points.
void test_closest_point_distance_property()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Various query points (interior, exterior face, exterior edge, exterior corner)
    std::array<vec3<si::metre>, 5> queries = {{
        vec3{0.3, 0.3, 0.2} * m,    // interior
        vec3{2.0, 0.5, 0.5} * m,    // outside face
        vec3{1.5, -0.5, 0.5} * m,   // outside edge
        vec3{-1.0, -1.0, -1.0} * m, // outside corner
        vec3{0.5, 0.5, 0.5} * m,    // center
    }};

    // Sample surface points to compare against
    std::array<vec3<si::metre>, 6> surface_samples = {{
        vec3{0.5, 0.5, 0.0} * m, // -Z face center
        vec3{0.5, 0.5, 1.0} * m, // +Z face center
        vec3{0.0, 0.5, 0.5} * m, // -X face center
        vec3{1.0, 0.5, 0.5} * m, // +X face center
        vec3{0.5, 0.0, 0.5} * m, // -Y face center
        vec3{0.5, 1.0, 0.5} * m, // +Y face center
    }};

    for (const auto &q : queries)
    {
        auto cp = msh->closest_point(q);
        auto best_dist_sq = (cp - q).squared_norm();
        for (const auto &sp : surface_samples)
        {
            auto sp_dist_sq = (sp - q).squared_norm();
            CHECK(best_dist_sq <= sp_dist_sq + 1e-9 * m * m);
        }
    }
}

// Test closest_point on a centered box (symmetric about origin)
void test_closest_point_centered_box()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m); // half-extents -> [-1,1]^3

    // Exterior: face projections
    CHECK_APPROX(msh->closest_point(vec3{3.0, 0.0, 0.0} * m), vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.0, -3.0, 0.0} * m), vec3{0.0, -1.0, 0.0} * m);

    // Exterior: corner
    CHECK_APPROX(msh->closest_point(vec3{2.0, 2.0, 2.0} * m), vec3{1.0, 1.0, 1.0} * m);

    // Interior: nearest face
    CHECK_APPROX(msh->closest_point(vec3{0.0, 0.0, 0.9} * m), vec3{0.0, 0.0, 1.0} * m);
    CHECK_APPROX(msh->closest_point(vec3{0.0, -0.9, 0.0} * m), vec3{0.0, -1.0, 0.0} * m);
}

// Test closest_point on a sphere mesh with known geometry
void test_closest_point_sphere_mesh()
{
    auto msh = mesh::sphere(2.0 * m, 32, 32);

    // Point far along +X axis: closest point should be near (2,0,0)
    auto cp = msh->closest_point(vec3{5.0, 0.0, 0.0} * m);
    CHECK(std::abs(cp.x().numerical_value_in(si::metre) - 2.0) < 0.1);
    CHECK(std::abs(cp.y().numerical_value_in(si::metre)) < 0.1);
    CHECK(std::abs(cp.z().numerical_value_in(si::metre)) < 0.1);

    // Point at origin (center of sphere): closest should be on surface
    auto cp_center = msh->closest_point(vec3{0.0, 0.0, 0.0} * m);
    auto dist = cp_center.norm();
    CHECK(std::abs(dist.numerical_value_in(si::metre) - 2.0) < 0.15);

    // Point far along -Y: closest near (0,-2,0)
    auto cp_ny = msh->closest_point(vec3{0.0, -10.0, 0.0} * m);
    CHECK(std::abs(cp_ny.y().numerical_value_in(si::metre) - (-2.0)) < 0.01);
}

// Test closest_point on a pyramid mesh
void test_closest_point_pyramid()
{
    auto msh = mesh::pyramid(1.0 * m, 2.0 * m); // base half=1, height=2
    // Base is at y=0, apex at y=2, base corners at (±1, 0, ±1)

    // Point below base center -> projects to base
    auto cp = msh->closest_point(vec3{0.0, -1.0, 0.0} * m);
    CHECK_APPROX(cp.y(), 0.0 * m);

    // Point above apex -> projects to apex (0, 2, 0)
    auto cp_top = msh->closest_point(vec3{0.0, 5.0, 0.0} * m);
    CHECK_APPROX(cp_top, vec3{0.0, 2.0, 0.0} * m);

    // Point at base corner far away projects to base corner
    CHECK_APPROX(msh->closest_point(vec3{3.0, -1.0, 3.0} * m), vec3{1.0, 0.0, 1.0} * m);
}

// Test closest_point via mesh::instance (transformed mesh)
void test_closest_point_instance()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // Translated instance at (5, 0, 0)
    auto inst = msh->at(vec3{5.0, 0.0, 0.0} * m);
    CHECK_APPROX(inst.closest_point(vec3{7.0, 0.5, 0.5} * m), vec3{6.0, 0.5, 0.5} * m);
    CHECK_APPROX(inst.closest_point(vec3{4.0, 0.5, 0.5} * m), vec3{5.0, 0.5, 0.5} * m);

    // Interior of translated instance
    CHECK_APPROX(inst.closest_point(vec3{5.5, 0.5, 0.1} * m), vec3{5.5, 0.5, 0.0} * m);
}

// Test closest_point via rotated mesh::instance
void test_closest_point_instance_rotated()
{
    cube_fixture cube;
    auto msh = cube.make_mesh();

    // 90-degree rotation around Z: (x,y,z) -> (-y,x,z)
    // Unit cube [0,1]^3 becomes [-1,0] x [0,1] x [0,1]
    auto rot_z =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    auto inst = msh->at(vec3{0.0, 0.0, 0.0} * m, rot_z);

    // Point outside at x=-2: should project to x=-1 face
    auto cp = inst.closest_point(vec3{-2.0, 0.5, 0.5} * m);
    CHECK_APPROX(cp.x(), -1.0 * m);
    CHECK_APPROX(cp.y(), 0.5 * m);
    CHECK_APPROX(cp.z(), 0.5 * m);

    // Point outside at y=2: should project to y=1 face
    auto cp2 = inst.closest_point(vec3{-0.5, 2.0, 0.5} * m);
    CHECK_APPROX(cp2.y(), 1.0 * m);
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

    tests.group("Ray Intersect Tests")
        .test("all 6 faces", test_ray_intersect_all_faces)
        .test("diagonal", test_ray_intersect_diagonal)
        .test("miss parallel", test_ray_intersect_miss_parallel)
        .test("miss wrong direction", test_ray_intersect_miss_wrong_direction)
        .test("from inside", test_ray_intersect_from_inside)
        .test("max_distance", test_ray_intersect_max_distance)
        .test("hit normal", test_ray_intersect_hit_normal)
        .test("edge hit", test_ray_intersect_edge_hit)
        .test("grazing", test_ray_intersect_grazing)
        .test("zero distance", test_ray_intersect_zero_distance)
        .test("sphere mesh", test_ray_intersect_sphere)
        .test("pyramid mesh", test_ray_intersect_pyramid)
        .test("returns nearest", test_ray_intersect_returns_nearest)
        .test("unnormalized direction", test_ray_intersect_unnormalized_direction)
        .test("translated instance", test_ray_intersect_instance_translated)
        .test("rotated instance", test_ray_intersect_instance_rotated)
        .test("centered box", test_ray_intersect_centered_box);

    tests.group("Closest Point Tests")
        .test("all faces exterior", test_closest_point_all_faces)
        .test("all 12 edges", test_closest_point_all_edges)
        .test("all 8 corners", test_closest_point_all_corners)
        .test("interior near each face", test_closest_point_interior_all_faces)
        .test("far-away points", test_closest_point_far_away)
        .test("near surface (epsilon)", test_closest_point_near_surface)
        .test("on surface features", test_closest_point_on_surface)
        .test("diagonal approach", test_closest_point_diagonal_approach)
        .test("distance minimality", test_closest_point_distance_property)
        .test("centered box mesh", test_closest_point_centered_box)
        .test("sphere mesh", test_closest_point_sphere_mesh)
        .test("pyramid mesh", test_closest_point_pyramid)
        .test("translated instance", test_closest_point_instance)
        .test("rotated instance", test_closest_point_instance_rotated);

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
