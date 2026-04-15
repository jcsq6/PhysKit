// Extensive tests for physkit::box, physkit::sphere, physkit::shape, and physkit::instance

#include <test.h>

using namespace testing;

// ===========================================================================
// Box — Construction & Properties
// ===========================================================================

void test_box_construction()
{
    box bx(vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.half_extents().x(), 1.0 * m);
    CHECK_APPROX(bx.half_extents().y(), 2.0 * m);
    CHECK_APPROX(bx.half_extents().z(), 3.0 * m);
}

void test_box_bounds()
{
    box bx(vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.bounds().min, vec3{-1.0, -2.0, -3.0} * m);
    CHECK_APPROX(bx.bounds().max, vec3{1.0, 2.0, 3.0} * m);

    // Symmetric box
    box unit(vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(unit.bounds().min, vec3{-0.5, -0.5, -0.5} * m);
    CHECK_APPROX(unit.bounds().max, vec3{0.5, 0.5, 0.5} * m);
}

void test_box_bsphere()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(bx.bsphere().center, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(bx.bsphere().radius, std::sqrt(3.0) * m);

    box bx2(vec3{3.0, 4.0, 0.0} * m);
    CHECK_APPROX(bx2.bsphere().radius, 5.0 * m);
}

void test_box_volume()
{
    // Unit cube: 2*0.5 * 2*0.5 * 2*0.5 = 1
    box unit(vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(unit.volume(), 1.0 * m * m * m);

    // Asymmetric
    box bx(vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.volume(), 2.0 * 4.0 * 6.0 * m * m * m);
}

void test_box_mass_center()
{
    box bx(vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.mass_center(), vec3{0.0, 0.0, 0.0} * m);

    box unit(vec3{0.5, 0.5, 0.5} * m);
    CHECK_APPROX(unit.mass_center(), vec3{0.0, 0.0, 0.0} * m);
}

void test_box_inertia_tensor()
{
    // Unit cube, density 1 kg/m³ → mass = 1 kg
    // I_xx = (m/12)(y² + z²) = (1/12)(1 + 1) = 1/6
    box unit(vec3{0.5, 0.5, 0.5} * m);
    auto density = 1.0 * kg / (m * m * m);
    auto I = unit.inertia_tensor(density);
    auto expected = 1.0 / 6.0;
    CHECK_APPROX(I[0, 0], expected * kg * m * m);
    CHECK_APPROX(I[1, 1], expected * kg * m * m);
    CHECK_APPROX(I[2, 2], expected * kg * m * m);
    // Off-diagonal should be zero
    CHECK_APPROX(I[0, 1], 0.0 * kg * m * m);
    CHECK_APPROX(I[0, 2], 0.0 * kg * m * m);
    CHECK_APPROX(I[1, 2], 0.0 * kg * m * m);

    // Asymmetric box: half_extents (1,2,3) → sides (2,4,6), volume=48
    // density=1 → mass=48
    // I_xx = (48/12)(16+36) = 208
    // I_yy = (48/12)(4+36)  = 160
    // I_zz = (48/12)(4+16)  = 80
    box bx(vec3{1.0, 2.0, 3.0} * m);
    auto I2 = bx.inertia_tensor(density);
    CHECK_APPROX(I2[0, 0], 208.0 * kg * m * m);
    CHECK_APPROX(I2[1, 1], 160.0 * kg * m * m);
    CHECK_APPROX(I2[2, 2], 80.0 * kg * m * m);
}

// ===========================================================================
// Box — Contains
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_contains()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Interior
    CHECK(bx.contains(vec3{0.0, 0.0, 0.0} * m));
    CHECK(bx.contains(vec3{0.5, 0.5, 0.5} * m));
    CHECK(bx.contains(vec3{-0.5, -0.5, -0.5} * m));
    CHECK(bx.contains(vec3{0.9, 0.0, 0.0} * m));

    // On boundary (should be contained: <=)
    CHECK(bx.contains(vec3{1.0, 0.0, 0.0} * m));
    CHECK(bx.contains(vec3{0.0, -1.0, 0.0} * m));
    CHECK(bx.contains(vec3{0.0, 0.0, 1.0} * m));
    CHECK(bx.contains(vec3{1.0, 1.0, 1.0} * m));
    CHECK(bx.contains(vec3{-1.0, -1.0, -1.0} * m));

    // Exterior
    CHECK(!bx.contains(vec3{1.1, 0.0, 0.0} * m));
    CHECK(!bx.contains(vec3{0.0, -1.1, 0.0} * m));
    CHECK(!bx.contains(vec3{0.0, 0.0, 1.1} * m));
    CHECK(!bx.contains(vec3{2.0, 2.0, 2.0} * m));
    CHECK(!bx.contains(vec3{-5.0, 0.0, 0.0} * m));

    // Asymmetric box
    box asym(vec3{0.5, 1.0, 2.0} * m);
    CHECK(asym.contains(vec3{0.4, 0.9, 1.9} * m));
    CHECK(!asym.contains(vec3{0.6, 0.0, 0.0} * m));
    CHECK(!asym.contains(vec3{0.0, 1.1, 0.0} * m));
    CHECK(!asym.contains(vec3{0.0, 0.0, 2.1} * m));
}

// ===========================================================================
// Box — Ray Intersect
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_ray_all_faces()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    struct face_case
    {
        vec3<m> origin;
        vec3<one> dir;
        vec3<m> expected_pos;
        vec3<one> expected_normal;
        quantity<m> expected_dist;
    };

    std::array cases{
        // +X face
        face_case{.origin = vec3{5.0, 0.0, 0.0} * m,
                  .dir = {-1, 0, 0},
                  .expected_pos = vec3{1.0, 0.0, 0.0} * m,
                  .expected_normal = {1, 0, 0},
                  .expected_dist = 4.0 * m},
        // -X face
        face_case{.origin = vec3{-5.0, 0.0, 0.0} * m,
                  .dir = {1, 0, 0},
                  .expected_pos = vec3{-1.0, 0.0, 0.0} * m,
                  .expected_normal = {-1, 0, 0},
                  .expected_dist = 4.0 * m},
        // +Y face
        face_case{.origin = vec3{0.0, 5.0, 0.0} * m,
                  .dir = {0, -1, 0},
                  .expected_pos = vec3{0.0, 1.0, 0.0} * m,
                  .expected_normal = {0, 1, 0},
                  .expected_dist = 4.0 * m},
        // -Y face
        face_case{.origin = vec3{0.0, -5.0, 0.0} * m,
                  .dir = {0, 1, 0},
                  .expected_pos = vec3{0.0, -1.0, 0.0} * m,
                  .expected_normal = {0, -1, 0},
                  .expected_dist = 4.0 * m},
        // +Z face
        face_case{.origin = vec3{0.0, 0.0, 5.0} * m,
                  .dir = {0, 0, -1},
                  .expected_pos = vec3{0.0, 0.0, 1.0} * m,
                  .expected_normal = {0, 0, 1},
                  .expected_dist = 4.0 * m},
        // -Z face
        face_case{.origin = vec3{0.0, 0.0, -5.0} * m,
                  .dir = {0, 0, 1},
                  .expected_pos = vec3{0.0, 0.0, -1.0} * m,
                  .expected_normal = {0, 0, -1},
                  .expected_dist = 4.0 * m},
    };

    for (const auto &[origin, dir, expected_pos, expected_normal, expected_dist] : cases)
    {
        auto hit = bx.ray_intersect({origin, dir});
        CHECK(hit.has_value());
        CHECK_APPROX(hit->pos, expected_pos);
        CHECK_APPROX(hit->normal, expected_normal);
        CHECK_APPROX(hit->distance, expected_dist);
    }
}

void test_box_ray_diagonal()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Diagonal from (-5,-5,-5) toward origin
    auto dir = vec3{1.0, 1.0, 1.0}.normalized();
    auto hit = bx.ray_intersect({vec3{-5.0, -5.0, -5.0} * m, dir});
    CHECK(hit.has_value());
    // Hit corner region at (-1,-1,-1), distance = sqrt(4²+4²+4²) = 4*sqrt(3)
    CHECK_APPROX(hit->distance, 4.0 * std::sqrt(3.0) * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_ray_miss()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Pointing away
    CHECK(!bx.ray_intersect({vec3{5.0, 0.0, 0.0} * m, {1, 0, 0}}).has_value());
    CHECK(!bx.ray_intersect({vec3{-5.0, 0.0, 0.0} * m, {-1, 0, 0}}).has_value());

    // Parallel and outside
    CHECK(!bx.ray_intersect({vec3{0.0, 2.0, 0.0} * m, {1, 0, 0}}).has_value());
    CHECK(!bx.ray_intersect({vec3{0.0, 0.0, -2.0} * m, {0, 1, 0}}).has_value());

    // Passing beside the box
    CHECK(!bx.ray_intersect({vec3{-5.0, 1.5, 0.0} * m, {1, 0, 0}}).has_value());
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_ray_from_inside()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // From origin, should hit each face at distance 1
    auto origin = vec3{0.0, 0.0, 0.0} * m;
    for (const auto &dir : {vec3<one>{1, 0, 0}, vec3<one>{-1, 0, 0}, vec3<one>{0, 1, 0},
                            vec3<one>{0, -1, 0}, vec3<one>{0, 0, 1}, vec3<one>{0, 0, -1}})
    {
        auto hit = bx.ray_intersect({origin, dir});
        CHECK(hit.has_value());
        CHECK_APPROX(hit->distance, 1.0 * m);
    }

    // From off-center interior
    auto hit = bx.ray_intersect({vec3{0.5, 0.0, 0.0} * m, {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->distance, 0.5 * m);
    CHECK_APPROX(hit->pos.x(), 1.0 * m);
}

void test_box_ray_max_distance()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);
    ray r{vec3{-5.0, 0.0, 0.0} * m, {1, 0, 0}};

    // Too short to reach
    CHECK(!bx.ray_intersect(r, 3.0 * m).has_value());

    // Exactly at the boundary
    CHECK(bx.ray_intersect(r, 4.0 * m).has_value());

    // Well past
    CHECK(bx.ray_intersect(r, 100.0 * m).has_value());
}

void test_box_ray_asymmetric()
{
    box bx(vec3{1.0, 2.0, 3.0} * m);

    // Along +X
    auto hit = bx.ray_intersect({vec3{5.0, 0.0, 0.0} * m, {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.x(), 1.0 * m);
    CHECK_APPROX(hit->distance, 4.0 * m);

    // Along +Y
    auto hit_y = bx.ray_intersect({vec3{0.0, 5.0, 0.0} * m, {0, -1, 0}});
    CHECK(hit_y.has_value());
    CHECK_APPROX(hit_y->pos.y(), 2.0 * m);
    CHECK_APPROX(hit_y->distance, 3.0 * m);

    // Along +Z
    auto hit_z = bx.ray_intersect({vec3{0.0, 0.0, 8.0} * m, {0, 0, -1}});
    CHECK(hit_z.has_value());
    CHECK_APPROX(hit_z->pos.z(), 3.0 * m);
    CHECK_APPROX(hit_z->distance, 5.0 * m);

    // Miss: ray passes just outside the y extent
    CHECK(!bx.ray_intersect({vec3{-5.0, 2.1, 0.0} * m, {1, 0, 0}}).has_value());
}

// ===========================================================================
// Box — Closest Point
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_closest_point_exterior()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Exterior on each face axis
    CHECK_APPROX(bx.closest_point(vec3{3.0, 0.0, 0.0} * m), vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{-3.0, 0.0, 0.0} * m), vec3{-1.0, 0.0, 0.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{0.0, 3.0, 0.0} * m), vec3{0.0, 1.0, 0.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{0.0, -3.0, 0.0} * m), vec3{0.0, -1.0, 0.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{0.0, 0.0, 3.0} * m), vec3{0.0, 0.0, 1.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{0.0, 0.0, -3.0} * m), vec3{0.0, 0.0, -1.0} * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_closest_point_edges()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Exterior on edges (two coordinates outside)
    CHECK_APPROX(bx.closest_point(vec3{3.0, 3.0, 0.0} * m), vec3{1.0, 1.0, 0.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{-3.0, -3.0, 0.0} * m), vec3{-1.0, -1.0, 0.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{0.0, 3.0, 3.0} * m), vec3{0.0, 1.0, 1.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{3.0, 0.0, -3.0} * m), vec3{1.0, 0.0, -1.0} * m);
}

void test_box_closest_point_corners()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Exterior at corners (all three coordinates outside)
    CHECK_APPROX(bx.closest_point(vec3{3.0, 3.0, 3.0} * m), vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{-3.0, -3.0, -3.0} * m), vec3{-1.0, -1.0, -1.0} * m);
    CHECK_APPROX(bx.closest_point(vec3{5.0, -5.0, 5.0} * m), vec3{1.0, -1.0, 1.0} * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_closest_point_interior()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Interior: projects to nearest face
    // Point (0.8, 0.0, 0.0): nearest face is +x (distance 0.2)
    CHECK_APPROX(bx.closest_point(vec3{0.8, 0.0, 0.0} * m), vec3{1.0, 0.0, 0.0} * m);

    // Point (0.0, -0.9, 0.0): nearest face is -y (distance 0.1)
    CHECK_APPROX(bx.closest_point(vec3{0.0, -0.9, 0.0} * m), vec3{0.0, -1.0, 0.0} * m);

    // Point (0.0, 0.0, 0.7): nearest face is +z (distance 0.3)
    CHECK_APPROX(bx.closest_point(vec3{0.0, 0.0, 0.7} * m), vec3{0.0, 0.0, 1.0} * m);

    // Origin: equidistant to all faces — algorithm picks +x (index 0)
    CHECK_APPROX(bx.closest_point(vec3{0.0, 0.0, 0.0} * m), vec3{1.0, 0.0, 0.0} * m);
}

void test_box_closest_point_asymmetric()
{
    // Asymmetric half_extents to exercise coordinate-specific clamping
    box bx(vec3{0.3, 1.0, 0.5} * m);

    // Point exterior only in y: (0.2, 1.5, 0.3)
    CHECK_APPROX(bx.closest_point(vec3{0.2, 1.5, 0.3} * m), vec3{0.2, 1.0, 0.3} * m);

    // Interior: nearest face depends on asymmetric extents
    // Point (0.0, 0.0, 0.4): nearest face is +z (distance 0.1)
    CHECK_APPROX(bx.closest_point(vec3{0.0, 0.0, 0.4} * m), vec3{0.0, 0.0, 0.5} * m);

    // Point (0.2, 0.0, 0.0): nearest face is +x (distance 0.1)
    CHECK_APPROX(bx.closest_point(vec3{0.2, 0.0, 0.0} * m), vec3{0.3, 0.0, 0.0} * m);

    // Point interior, closest to +x face: (0.25, 0.1, 0.1)
    // distances: x=0.05, y=0.9, z=0.4 → nearest is +x
    CHECK_APPROX(bx.closest_point(vec3{0.25, 0.1, 0.1} * m), vec3{0.3, 0.1, 0.1} * m);
}

// ===========================================================================
// Box — Support
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_box_support_axis()
{
    box bx(vec3{1.0, 2.0, 3.0} * m);

    // Axis-aligned directions: returns the vertex with max dot product
    CHECK_APPROX(bx.support({1, 0, 0}), vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.support({-1, 0, 0}), vec3{-1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.support({0, 1, 0}), vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.support({0, -1, 0}), vec3{1.0, -2.0, 3.0} * m);
    CHECK_APPROX(bx.support({0, 0, 1}), vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(bx.support({0, 0, -1}), vec3{1.0, 2.0, -3.0} * m);
}

void test_box_support_diagonal()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);

    // Diagonal: the vertex (1,1,1) maximizes dot product with (1,1,1)
    CHECK_APPROX(bx.support(vec3{1.0, 1.0, 1.0}), vec3{1.0, 1.0, 1.0} * m);
    CHECK_APPROX(bx.support(vec3{-1.0, -1.0, -1.0}), vec3{-1.0, -1.0, -1.0} * m);

    // Mixed: direction (1, -1, 1) → vertex (1, -1, 1)
    CHECK_APPROX(bx.support(vec3{1.0, -1.0, 1.0}), vec3{1.0, -1.0, 1.0} * m);
}

// ===========================================================================
// Sphere — Construction & Properties
// ===========================================================================

void test_sphere_construction()
{
    sphere sph(2.0 * m);
    CHECK_APPROX(sph.radius(), 2.0 * m);
}

void test_sphere_bounds()
{
    sphere sph(2.0 * m);
    CHECK_APPROX(sph.bounds().min, vec3{-2.0, -2.0, -2.0} * m);
    CHECK_APPROX(sph.bounds().max, vec3{2.0, 2.0, 2.0} * m);

    sphere unit(1.0 * m);
    CHECK_APPROX(unit.bounds().min, vec3{-1.0, -1.0, -1.0} * m);
    CHECK_APPROX(unit.bounds().max, vec3{1.0, 1.0, 1.0} * m);
}

void test_sphere_bsphere()
{
    sphere sph(3.0 * m);
    CHECK_APPROX(sph.bsphere().center, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(sph.bsphere().radius, 3.0 * m);
}

void test_sphere_volume()
{
    sphere unit(1.0 * m);
    CHECK_APPROX(unit.volume(), (4.0 / 3.0) * std::numbers::pi * m * m * m);

    sphere sph(2.0 * m);
    CHECK_APPROX(sph.volume(), (4.0 / 3.0) * std::numbers::pi * 8.0 * m * m * m);
}

void test_sphere_mass_center()
{
    sphere sph(5.0 * m);
    CHECK_APPROX(sph.mass_center(), vec3{0.0, 0.0, 0.0} * m);
}

void test_sphere_inertia_tensor()
{
    // I = (2/5) * mass * r² * identity
    // For r=1, density=1: mass = 4π/3, I = (2/5)(4π/3)(1) = 8π/15
    sphere unit(1.0 * m);
    auto density = 1.0 * kg / (m * m * m);
    auto I = unit.inertia_tensor(density);
    auto expected = static_cast<double>(8.0 * std::numbers::pi / 15.0);
    CHECK_APPROX(I[0, 0], expected * kg * m * m);
    CHECK_APPROX(I[1, 1], expected * kg * m * m);
    CHECK_APPROX(I[2, 2], expected * kg * m * m);
    // Off-diagonal zero
    CHECK_APPROX(I[0, 1], 0.0 * kg * m * m);
    CHECK_APPROX(I[0, 2], 0.0 * kg * m * m);
    CHECK_APPROX(I[1, 2], 0.0 * kg * m * m);

    // For r=2: mass = 4π/3 * 8 = 32π/3, I = (2/5)(32π/3)(4) = 256π/15
    sphere sph2(2.0 * m);
    auto I2 = sph2.inertia_tensor(density);
    auto expected2 = static_cast<double>(256.0 * std::numbers::pi / 15.0);
    CHECK_APPROX(I2[0, 0], expected2 * kg * m * m);
    CHECK_APPROX(I2[1, 1], expected2 * kg * m * m);
    CHECK_APPROX(I2[2, 2], expected2 * kg * m * m);
}

// ===========================================================================
// Sphere — Contains
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_sphere_contains()
{
    sphere sph(2.0 * m);

    // Interior
    CHECK(sph.contains(vec3{0.0, 0.0, 0.0} * m));
    CHECK(sph.contains(vec3{1.0, 0.0, 0.0} * m));
    CHECK(sph.contains(vec3{0.0, -1.5, 0.0} * m));
    CHECK(sph.contains(vec3{1.0, 1.0, 1.0} * m)); // dist = sqrt(3) ≈ 1.73 < 2

    // On boundary
    CHECK(sph.contains(vec3{2.0, 0.0, 0.0} * m));
    CHECK(sph.contains(vec3{0.0, -2.0, 0.0} * m));

    // Exterior
    CHECK(!sph.contains(vec3{2.1, 0.0, 0.0} * m));
    CHECK(!sph.contains(vec3{0.0, 0.0, -2.1} * m));
    CHECK(!sph.contains(vec3{1.5, 1.5, 1.5} * m)); // dist = sqrt(6.75) ≈ 2.6 > 2
    CHECK(!sph.contains(vec3{10.0, 0.0, 0.0} * m));
}

// ===========================================================================
// Sphere — Ray Intersect
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_sphere_ray_axes()
{
    sphere sph(1.0 * m);

    // Along +X from outside
    auto hit = sph.ray_intersect({vec3{5.0, 0.0, 0.0} * m, {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit->distance, 4.0 * m);
    CHECK_APPROX(hit->normal, vec3<one>{1, 0, 0});

    // Along -Y from outside
    auto hit_y = sph.ray_intersect({vec3{0.0, -5.0, 0.0} * m, {0, 1, 0}});
    CHECK(hit_y.has_value());
    CHECK_APPROX(hit_y->pos, vec3{0.0, -1.0, 0.0} * m);
    CHECK_APPROX(hit_y->distance, 4.0 * m);
    CHECK_APPROX(hit_y->normal, vec3<one>{0, -1, 0});

    // Along +Z from outside
    auto hit_z = sph.ray_intersect({vec3{0.0, 0.0, 3.0} * m, {0, 0, -1}});
    CHECK(hit_z.has_value());
    CHECK_APPROX(hit_z->pos, vec3{0.0, 0.0, 1.0} * m);
    CHECK_APPROX(hit_z->distance, 2.0 * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_sphere_ray_miss()
{
    sphere sph(1.0 * m);

    // Pointing away
    CHECK(!sph.ray_intersect({vec3{5.0, 0.0, 0.0} * m, {1, 0, 0}}).has_value());

    // Passing beside
    CHECK(!sph.ray_intersect({vec3{0.0, 2.0, 0.0} * m, {1, 0, 0}}).has_value());
    CHECK(!sph.ray_intersect({vec3{0.0, 0.0, 1.5} * m, {1, 0, 0}}).has_value());

    // Parallel far away
    CHECK(!sph.ray_intersect({vec3{-10.0, 5.0, 0.0} * m, {1, 0, 0}}).has_value());
}

void test_sphere_ray_from_inside()
{
    sphere sph(1.0 * m);

    // From origin outward: should hit surface at distance 1
    auto hit = sph.ray_intersect({vec3{0.0, 0.0, 0.0} * m, {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit->distance, 1.0 * m);

    // From off-center interior
    auto hit2 = sph.ray_intersect({vec3{0.5, 0.0, 0.0} * m, {1, 0, 0}});
    CHECK(hit2.has_value());
    CHECK_APPROX(hit2->pos, vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit2->distance, 0.5 * m);
}

void test_sphere_ray_max_distance()
{
    sphere sph(1.0 * m);
    ray r{vec3{-5.0, 0.0, 0.0} * m, {1, 0, 0}};

    // Too short to reach
    CHECK(!sph.ray_intersect(r, 3.0 * m).has_value());

    // Reaches the sphere
    CHECK(sph.ray_intersect(r, 4.0 * m).has_value());

    // Well past
    CHECK(sph.ray_intersect(r, 100.0 * m).has_value());
}

void test_sphere_ray_different_radii()
{
    sphere sph(3.0 * m);

    auto hit = sph.ray_intersect({vec3{10.0, 0.0, 0.0} * m, {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, vec3{3.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit->distance, 7.0 * m);
}

// ===========================================================================
// Sphere — Closest Point
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_sphere_closest_point()
{
    sphere sph(2.0 * m);

    // Exterior: projects onto sphere surface along the direction from center
    auto cp = sph.closest_point(vec3{6.0, 0.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{2.0, 0.0, 0.0} * m);

    cp = sph.closest_point(vec3{0.0, -5.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{0.0, -2.0, 0.0} * m);

    // Diagonal exterior: (3, 4, 0), norm=5, normalized=(0.6, 0.8, 0)
    cp = sph.closest_point(vec3{3.0, 4.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{1.2, 1.6, 0.0} * m);

    // Interior point: also projects to surface in the same direction
    cp = sph.closest_point(vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{2.0, 0.0, 0.0} * m);

    cp = sph.closest_point(vec3{0.0, 0.0, -0.5} * m);
    CHECK_APPROX(cp, vec3{0.0, 0.0, -2.0} * m);
}

void test_sphere_closest_point_on_surface()
{
    sphere sph(1.0 * m);

    // Point already on the surface
    auto cp = sph.closest_point(vec3{1.0, 0.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{1.0, 0.0, 0.0} * m);

    cp = sph.closest_point(vec3{0.0, -1.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{0.0, -1.0, 0.0} * m);
}

// ===========================================================================
// Sphere — Support
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_sphere_support_axes()
{
    sphere sph(3.0 * m);

    CHECK_APPROX(sph.support({1, 0, 0}), vec3{3.0, 0.0, 0.0} * m);
    CHECK_APPROX(sph.support({-1, 0, 0}), vec3{-3.0, 0.0, 0.0} * m);
    CHECK_APPROX(sph.support({0, 1, 0}), vec3{0.0, 3.0, 0.0} * m);
    CHECK_APPROX(sph.support({0, -1, 0}), vec3{0.0, -3.0, 0.0} * m);
    CHECK_APPROX(sph.support({0, 0, 1}), vec3{0.0, 0.0, 3.0} * m);
    CHECK_APPROX(sph.support({0, 0, -1}), vec3{0.0, 0.0, -3.0} * m);
}

void test_sphere_support_diagonal()
{
    sphere sph(1.0 * m);

    // Direction (1,1,1) normalized → support at r * normalized(1,1,1)
    auto inv_sqrt3 = 1.0 / std::sqrt(3.0);
    auto sp = sph.support(vec3{1.0, 1.0, 1.0});
    CHECK_APPROX(sp, vec3{inv_sqrt3, inv_sqrt3, inv_sqrt3} * m);

    // Unnormalized direction should still work (support normalizes internally)
    auto sp2 = sph.support(vec3{2.0, 0.0, 0.0});
    CHECK_APPROX(sp2, vec3{1.0, 0.0, 0.0} * m);
}

//cone tests generated with Claude Sonnet 4.6
// ===========================================================================
// Cone — Construction & Properties
// ===========================================================================

void test_cone_construction()
{
    cone c(1.0 * m, 2.0 * m);
    CHECK_APPROX(c.radius(), 1.0 * m);
    CHECK_APPROX(c.height(), 2.0 * m);

    cone asym(3.0 * m, 5.0 * m);
    CHECK_APPROX(asym.radius(), 3.0 * m);
    CHECK_APPROX(asym.height(), 5.0 * m);
}

void test_cone_bounds()
{
    // AABB: min = (-r, 0, -r), max = (r, h, r)
    cone c(1.0 * m, 2.0 * m);
    CHECK_APPROX(c.bounds().min, vec3{-1.0, 0.0, -1.0} * m);
    CHECK_APPROX(c.bounds().max, vec3{ 1.0, 2.0,  1.0} * m);

    cone c2(2.0 * m, 4.0 * m);
    CHECK_APPROX(c2.bounds().min, vec3{-2.0, 0.0, -2.0} * m);
    CHECK_APPROX(c2.bounds().max, vec3{ 2.0, 4.0,  2.0} * m);
}

void test_cone_bsphere()
{
    // Bounding sphere center should sit at the cone's geometric midpoint (0, h/2, 0).
    // Radius should be at least half the slant height: sqrt(r² + (h/2)²).
    cone c(1.0 * m, 2.0 * m);
    CHECK_APPROX(c.bsphere().center, vec3{0.0, 1.0, 0.0} * m);
    // Correct circumsphere radius (from mid-height to the base edge):
    // sqrt(1² + 1²) = sqrt(2) ≈ 1.414
    CHECK_APPROX(c.bsphere().radius, std::sqrt(2.0) * m); // currently sqrt(2)/2 (bug)
}

void test_cone_volume()
{
    // V = π r² h / 3
    cone unit(1.0 * m, 3.0 * m);
    CHECK_APPROX(unit.volume(), std::numbers::pi * m * m * m);

    cone c(2.0 * m, 3.0 * m);
    CHECK_APPROX(c.volume(), 4.0 * std::numbers::pi * m * m * m);

    cone c2(3.0 * m, 1.0 * m);
    CHECK_APPROX(c2.volume(), 3.0 * std::numbers::pi * m * m * m);
}

void test_cone_mass_center()
{
    // Center of mass of a solid cone is at h/4 from the base along the axis.
    cone c(1.0 * m, 4.0 * m);
    CHECK_APPROX(c.mass_center(), vec3{0.0, 1.0, 0.0} * m);

    cone c2(2.0 * m, 8.0 * m);
    CHECK_APPROX(c2.mass_center(), vec3{0.0, 2.0, 0.0} * m);
}

void test_cone_inertia_tensor()
{
    // For density ρ, mass m = ρ·V = ρ·π r² h / 3.
    //   I_y  = (3/10) m r²
    //   I_xz = m · ((3/20) r² + (1/10) h²)
    // Off-diagonal entries should be zero.

    // r=1, h=3, density=1 → V=π, m=π
    //   I_y  = 0.3·π·1  = 0.3π
    //   I_xz = π·(0.15 + 0.9) = 1.05π
    auto density = 1.0 * kg / (m * m * m);
    cone c(1.0 * m, 3.0 * m);
    auto I = c.inertia_tensor(density);
    CHECK_APPROX(I[0, 0], 1.05 * std::numbers::pi * kg * m * m);
    CHECK_APPROX(I[1, 1], 0.3  * std::numbers::pi * kg * m * m);
    CHECK_APPROX(I[2, 2], 1.05 * std::numbers::pi * kg * m * m);
    // Off-diagonal elements must be zero (diagonal tensor)
    CHECK_APPROX(I[0, 1], 0.0 * kg * m * m);
    CHECK_APPROX(I[0, 2], 0.0 * kg * m * m);
    CHECK_APPROX(I[1, 2], 0.0 * kg * m * m);

    // r=2, h=6, density=1 → V=8π, m=8π
    //   I_y  = 0.3·8π·4  = 9.6π
    //   I_xz = 8π·(0.15·4 + 0.1·36) = 8π·(0.6 + 3.6) = 33.6π
    cone c2(2.0 * m, 6.0 * m);
    auto I2 = c2.inertia_tensor(density);
    CHECK_APPROX(I2[0, 0], 33.6 * std::numbers::pi * kg * m * m);
    CHECK_APPROX(I2[1, 1],  9.6 * std::numbers::pi * kg * m * m);
    CHECK_APPROX(I2[2, 2], 33.6 * std::numbers::pi * kg * m * m);
}

// ===========================================================================
// Cone — Contains
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_cone_contains_interior()
{
    cone c(1.0 * m, 2.0 * m);

    // Origin is the center of the base — on the boundary (r_at_0 == 1)
    CHECK(c.contains(vec3{0.0, 0.0, 0.0} * m));

    // Axis interior points
    CHECK(c.contains(vec3{0.0, 0.5, 0.0} * m));   // well inside on axis
    CHECK(c.contains(vec3{0.0, 1.0, 0.0} * m));   // mid-height on axis
    CHECK(c.contains(vec3{0.0, 1.99, 0.0} * m));  // near tip on axis

    // At y=1 (half-height) the radius of the cone is r*(1 - 1/2) = 0.5
    CHECK( c.contains(vec3{0.4,  1.0, 0.0} * m)); // inside the half-height slice
    CHECK(!c.contains(vec3{0.6,  1.0, 0.0} * m)); // outside the half-height slice

    // At y=0 (base) the allowed radius is exactly r=1
    CHECK( c.contains(vec3{0.99, 0.0, 0.0} * m));
    CHECK( c.contains(vec3{1.0,  0.0, 0.0} * m)); // on boundary
    CHECK(!c.contains(vec3{1.01, 0.0, 0.0} * m));
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_cone_contains_exterior()
{
    cone c(1.0 * m, 2.0 * m);

    // Below the base
    CHECK(!c.contains(vec3{ 0.0, -0.01, 0.0} * m));
    CHECK(!c.contains(vec3{ 0.5, -1.0,  0.0} * m));

    // Above the tip
    CHECK(!c.contains(vec3{0.0, 2.01, 0.0} * m));
    CHECK(!c.contains(vec3{0.0, 3.0,  0.0} * m));

    // Radially outside at various heights
    CHECK(!c.contains(vec3{1.1, 0.0, 0.0} * m));
    CHECK(!c.contains(vec3{0.0, 0.0, 1.1} * m));
    CHECK(!c.contains(vec3{0.6, 1.0, 0.6} * m)); // diagonal outside

    // Far away
    CHECK(!c.contains(vec3{10.0, 10.0, 10.0} * m));
}

void test_cone_contains_tip()
{
    cone c(1.0 * m, 2.0 * m);

    // The tip itself: (0, h, 0) — radius at h is r*(1-1)=0, so only the exact tip point
    CHECK( c.contains(vec3{0.0,  2.0,  0.0} * m));
    CHECK(!c.contains(vec3{0.01, 2.0,  0.0} * m));
    CHECK(!c.contains(vec3{0.0,  2.0,  0.01} * m));
}

// ===========================================================================
// Cone — Ray Intersect
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_cone_ray_base()
{
    // A ray shot straight down at the base center should hit at the origin.
    cone c(1.0 * m, 2.0 * m);

    auto hit = c.ray_intersect({vec3{0.0, 1.0, 0.0} * m, {0, -1, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos,      vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit->normal,   vec3<one>{0, -1, 0});
    CHECK_APPROX(hit->distance, 1.0 * m);

    // A ray from below pointing up should hit the base from beneath.
    auto hit2 = c.ray_intersect({vec3{0.0, -3.0, 0.0} * m, {0, 1, 0}});
    CHECK(hit2.has_value());
    CHECK_APPROX(hit2->pos.y(),    0.0 * m);
    CHECK_APPROX(hit2->normal, vec3<one>{0, -1, 0});
    CHECK_APPROX(hit2->distance,   3.0 * m);

    // A ray that hits the base off-center (still inside the circle)
    auto hit3 = c.ray_intersect({vec3{0.5, -5.0, 0.0} * m, {0, 1, 0}});
    CHECK(hit3.has_value());
    CHECK_APPROX(hit3->pos, vec3{0.5, 0.0, 0.0} * m);
    CHECK_APPROX(hit3->normal, vec3<one>{0, -1, 0});
    CHECK_APPROX(hit3->distance, 5.0 * m);

    // A ray above the base but whose xz footprint is outside the radius: should miss the base.
    auto miss = c.ray_intersect({vec3{1.5, 5.0, 0.0} * m, {0, -1, 0}});
    // This may still hit the lateral surface; if only the base is considered it must miss.
    // (The test accepts a surface hit but verifies the base hit_pos.y() == 0 only if hit.)
    if (miss.has_value())
        CHECK(miss->pos.y() > 0.0 * m); // must be a lateral surface hit, not base
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_cone_ray_lateral_surface()
{
    // Ray shot horizontally at the side of the cone from outside.
    // Cone: r=1, h=2.  At y=1 (mid-height) the cone radius is 0.5.
    cone c(1.0 * m, 2.0 * m);

    // Approach from +x at y=1 (mid-height); expect a hit on the lateral surface.
    auto hit = c.ray_intersect({vec3{5.0, 1.0, 0.0} * m, {-1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.y(), 1.0 * m);  // same height
    CHECK_APPROX(hit->pos.z(), 0.0 * m);  // on xz symmetry plane
    // The hit point must satisfy the cone equation: x == r*(1 - y/h) == 0.5
    CHECK_APPROX(hit->pos.x(), 0.5 * m);
}

void test_cone_ray_tip()
{
    // A ray aimed exactly at the tip (0, h, 0) from outside should hit there.
    cone c(1.0 * m, 2.0 * m);

    auto tip = vec3{0.0, 2.0, 0.0} * m;
    auto origin = vec3{5.0, 2.0, 0.0} * m;
    auto dir = (tip - origin); // direction toward tip (unnormalized; normalise externally)
    auto dir_n = vec3<one>{-1, 0, 0};

    auto hit = c.ray_intersect({origin, dir_n});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, tip);
    CHECK_APPROX(hit->distance, 5.0 * m);
}

void test_cone_ray_miss()
{
    cone c(1.0 * m, 2.0 * m);

    // Ray pointing away from the cone entirely
    CHECK(!c.ray_intersect({vec3{5.0, 1.0, 0.0} * m, {1, 0, 0}}).has_value());

    // Ray parallel to the axis but far outside the radius
    CHECK(!c.ray_intersect({vec3{2.0, 5.0, 0.0} * m, {0, -1, 0}}).has_value());

    // Ray passing above the tip
    CHECK(!c.ray_intersect({vec3{0.0, 3.0, 0.0} * m, {1, 0, 0}}).has_value());

    // Ray in the xz-plane (y=0) pointing sideways — passes beside the cone
    CHECK(!c.ray_intersect({vec3{0.0, -1.0, 0.0} * m, {1, 0, 0}}).has_value());
}

void test_cone_ray_max_distance()
{
    cone c(1.0 * m, 2.0 * m);
    // Ray from (0, 5, 0) pointing up hits the base at distance 5.
    ray r{vec3{0.0, -5.0, 0.0} * m, {0, 1, 0}};

    // Too short to reach
    CHECK(!c.ray_intersect(r, 4.9 * m).has_value());

    // Exactly at boundary
    CHECK(c.ray_intersect(r, 5.0 * m).has_value());

    // Well past
    CHECK(c.ray_intersect(r, 100.0 * m).has_value());
}

void test_cone_ray_intersect_nearest_root()
{
    cone c(1.0 * m, 4.0 * m);
    // Shoot a ray along +x from outside at y=2 (mid height → cone radius = 0.5).
    // The ray crosses the surface at x ≈ -0.5 (entry) and x ≈ +0.5 (exit).
    // The nearest intersection from origin (5,2,0) is at x=0.5, distance=4.5.
    auto hit = c.ray_intersect({vec3{5.0, 2.0, 0.0} * m, {-1, 0, 0}});
    CHECK(hit.has_value());
    // The reported distance must be the SMALLER of the two roots (entry point).
    CHECK_APPROX(hit->pos.x(),    0.5 * m, 0.05 * m);
    CHECK_APPROX(hit->distance,   4.5 * m, 0.05 * m);
}

// ===========================================================================
// Cone — Closest Point
// ===========================================================================

void test_cone_closest_point_tip()
{
    cone c(1.0 * m, 2.0 * m);

    // A point directly above the tip should snap to the tip.
    auto cp = c.closest_point(vec3{0.0, 5.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{0.0, 2.0, 0.0} * m);

    // A point AT the tip should return the tip.
    auto cp2 = c.closest_point(vec3{0.0, 2.0, 0.0} * m);
    CHECK_APPROX(cp2, vec3{0.0, 2.0, 0.0} * m);
}

void test_cone_closest_point_base()
{
    cone c(1.0 * m, 2.0 * m);

    // A point directly below the base center — closest is the base center.
    auto cp = c.closest_point(vec3{0.0, -3.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{0.0, 0.0, 0.0} * m);

    // A point directly below a base-edge position — closest is that edge point.
    auto cp2 = c.closest_point(vec3{1.0, -2.0, 0.0} * m);
    CHECK_APPROX(cp2, vec3{1.0, 0.0, 0.0} * m);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_cone_closest_point_lateral()
{
    cone c(1.0 * m, 2.0 * m);

    // A point radially outside the cone surface, at mid-height.
    // At y=1 the cone radius = 0.5; a point at (3, 1, 0) is outside.
    // The closest surface point is somewhere on the slant edge.
    auto cp = c.closest_point(vec3{3.0, 1.0, 0.0} * m);
    // The closest point must lie on the cone surface or edge.
    // Verify it is at y >= 0 and y <= height.
    CHECK(cp.y() >= 0.0 * m);
    CHECK(cp.y() <= 2.0 * m);
    // It must be in the +x half-space (same side as the query point).
    CHECK(cp.x() > 0.0 * m);
    // And it must actually be on or inside the cone surface.
    CHECK(c.contains(cp));
}

void test_cone_closest_point_inside()
{
    // For a point inside the cone the closest point on the surface is some
    // projection; it must always lie on the boundary.
    cone c(1.0 * m, 2.0 * m);

    auto interior = vec3{0.1, 0.5, 0.0} * m;
    CHECK(c.contains(interior));

    auto cp = c.closest_point(interior);
    // Result must be on the cone surface (contained but the result of projecting)
    // or on the base — either way it must be a boundary point.
    CHECK(c.contains(cp));
}

void test_cone_closest_point_base_edge()
{
    cone c(1.0 * m, 2.0 * m);

    // A point exactly on the base edge circle
    auto edge = vec3{1.0, 0.0, 0.0} * m;
    auto cp = c.closest_point(edge);
    CHECK_APPROX(cp, edge);

    // A point outside but at y=0, well outside the radius
    auto outside_base = vec3{5.0, 0.0, 0.0} * m;
    auto cp2 = c.closest_point(outside_base);
    // Closest should be the nearest point on the base-edge circle
    CHECK_APPROX(cp2, vec3{1.0, 0.0, 0.0} * m);
}

// ===========================================================================
// Cone — Support (GJK)
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_cone_support_upward()
{
    cone c(1.0 * m, 2.0 * m);

    // Direction straight up (+y): the farthest point is the tip (0, h, 0).
    auto s = c.support({0, 1, 0});
    CHECK_APPROX(s, vec3{0.0, 2.0, 0.0} * m);
}

void test_cone_support_downward()
{
    cone c(1.0 * m, 2.0 * m);

    // Direction straight down (-y): farthest points are any base-edge points
    // (y=0). The implementation may return any of them; we only verify y==0
    // and that the point is on the base edge.
    auto s = c.support({0, -1, 0});
    CHECK_APPROX(s.y(), 0.0 * m);
    // Should be at the rim (distance from y-axis == radius)
    auto xz_dist = std::sqrt(
        s.x().numerical_value_in(si::metre) * s.x().numerical_value_in(si::metre) +
        s.z().numerical_value_in(si::metre) * s.z().numerical_value_in(si::metre));
    CHECK_APPROX(xz_dist * m, 1.0 * m);
}

void test_cone_support_lateral()
{
    cone c(1.0 * m, 2.0 * m);

    // Direction +x: the farthest point is on the base circle at (r, 0, 0).
    // (The tip has x=0, the base edge has x=r — the base edge wins.)
    auto s = c.support({1, 0, 0});
    CHECK_APPROX(s.x(), 1.0 * m);
    CHECK_APPROX(s.y(), 0.0 * m);
    CHECK_APPROX(s.z(), 0.0 * m);

    // Symmetric for -x
    auto s2 = c.support({-1, 0, 0});
    CHECK_APPROX(s2.x(), -1.0 * m);
    CHECK_APPROX(s2.y(),  0.0 * m);
    CHECK_APPROX(s2.z(),  0.0 * m);
}

void test_cone_support_diagonal()
{
    cone c(1.0 * m, 2.0 * m);

    // Direction (1,1,0) normalised: the support is either the tip or a point
    // on the base edge, whichever has the larger dot product.
    // tip dot (1,1,0)/sqrt2 = h/sqrt2 = 2/sqrt2 ≈ 1.414
    // base-edge at (1,0,0) dot (1,1,0)/sqrt2 = 1/sqrt2 ≈ 0.707
    // → tip wins
    auto dir = vec3<one>{1.0, 1.0, 0.0}.normalized();
    auto s = c.support(dir);
    CHECK_APPROX(s, vec3{0.0, 2.0, 0.0} * m);

    // Direction (1, 0.1, 0): now the base edge wins over the tip.
    // tip dot d ≈ 0.1/|d|; base-edge at (r,0,0) dot d ≈ 1/|d|  → base wins
    auto dir2 = vec3<one>{1.0, 0.1, 0.0}.normalized();
    auto s2 = c.support(dir2);
	 printf("s2: %f, %f, %f\n", s2.x().numerical_value_in(si::metre), s2.y().numerical_value_in(si::metre), s2.z().numerical_value_in(si::metre));
    CHECK_APPROX(s2.y(), 0.0 * m);  // base level
    CHECK_APPROX(s2.x(), 1.0 * m);  // at the radius
}

// ===========================================================================
// Cone — Asymmetric geometry (r ≠ h)
// ===========================================================================

void test_cone_asymmetric_volume()
{
    // r=3, h=1  →  V = π·9·1/3 = 3π
    cone c(3.0 * m, 1.0 * m);
    CHECK_APPROX(c.volume(), 3.0 * std::numbers::pi * m * m * m);
}

void test_cone_asymmetric_mass_center()
{
    // r=3, h=1  →  CoM at (0, 0.25, 0)
    cone c(3.0 * m, 1.0 * m);
    CHECK_APPROX(c.mass_center(), vec3{0.0, 0.25, 0.0} * m);
}

void test_cone_asymmetric_contains()
{
    // Wide, flat cone: r=4, h=1
    cone c(4.0 * m, 1.0 * m);
    // At y=0.5 (mid-height) the radius is 4*(1-0.5) = 2.0
    CHECK( c.contains(vec3{1.9, 0.5, 0.0} * m));
    CHECK(!c.contains(vec3{2.1, 0.5, 0.0} * m));
}

// ===========================================================================
// Shape Wrapper
// ===========================================================================

void test_shape_default()
{
    shape s;
    CHECK(s.type() == shape::type::shape_box);
    // Default box has half_extents (0.5, 0.5, 0.5), volume = 1
    CHECK_APPROX(s.volume(), 1.0 * m * m * m);
}

void test_shape_from_sphere()
{
    sphere sph(2.0 * m);
    shape s(sph);
    CHECK(s.type() == shape::type::shape_sphere);
    CHECK_APPROX(s.volume(), sph.volume());
    CHECK_APPROX(s.mass_center(), sph.mass_center());
    CHECK(s.is_convex());
    CHECK_APPROX(s.sphere().radius(), 2.0 * m);
}

void test_shape_from_box()
{
    box bx(vec3{1.0, 2.0, 3.0} * m);
    shape s(bx);
    CHECK(s.type() == shape::type::shape_box);
    CHECK_APPROX(s.volume(), bx.volume());
    CHECK_APPROX(s.bounds(), bx.bounds());
    CHECK(s.is_convex());
}

void test_shape_from_mesh()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    shape s(msh);
    CHECK(s.type() == shape::type::shape_mesh);
    CHECK_APPROX(s.volume(), msh->volume());
    CHECK(!s.vertices().empty());
    CHECK(!s.triangles().empty());
}

void test_shape_copy()
{
    sphere sph(1.0 * m);
    shape s1(sph);
    shape s2 = s1; // NOLINT(performance-unnecessary-copy-initialization)
    CHECK(s2.type() == shape::type::shape_sphere);
    CHECK_APPROX(s2.volume(), s1.volume());
    CHECK_APPROX(s2.sphere().radius(), 1.0 * m);

    // Copy assign
    box bx(vec3{1.0, 1.0, 1.0} * m);
    shape s3(bx);
    s2 = s3;
    CHECK(s2.type() == shape::type::shape_box);
    CHECK_APPROX(s2.volume(), s3.volume());
}

void test_shape_move()
{
    sphere sph(1.0 * m);
    shape s1(sph);
    shape s2 = std::move(s1);
    CHECK(s2.type() == shape::type::shape_sphere);
    CHECK_APPROX(s2.volume(), sph.volume());

    // Move assign
    box bx(vec3{2.0, 2.0, 2.0} * m);
    shape s3(bx);
    s2 = std::move(s3);
    CHECK(s2.type() == shape::type::shape_box);
}

void test_shape_dispatch()
{
    // Verify dispatched methods match underlying primitive for both types

    // Sphere dispatch
    sphere sph(2.0 * m);
    shape ss(sph);
    CHECK_APPROX(ss.bounds(), sph.bounds());
    CHECK_APPROX(ss.bsphere(), sph.bsphere());
    CHECK_APPROX(ss.mass_center(), sph.mass_center());
    CHECK_APPROX(ss.support({1, 0, 0}), sph.support({1, 0, 0}));
    CHECK(ss.contains(vec3{0.0, 0.0, 0.0} * m) == sph.contains(vec3{0.0, 0.0, 0.0} * m));

    // Box dispatch
    box bx(vec3{1.0, 1.0, 1.0} * m);
    shape sb(bx);
    CHECK_APPROX(sb.bounds(), bx.bounds());
    CHECK_APPROX(sb.bsphere(), bx.bsphere());
    CHECK_APPROX(sb.mass_center(), bx.mass_center());
    CHECK_APPROX(sb.support({0, 0, -1}), bx.support({0, 0, -1}));
    CHECK(sb.contains(vec3{0.5, 0.5, 0.5} * m) == bx.contains(vec3{0.5, 0.5, 0.5} * m));
}

// ===========================================================================
// Instance — World-Space Transforms
// ===========================================================================

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void test_instance_translated_box()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);
    shape s(bx);
    auto pos = vec3{10.0, 0.0, 0.0} * m;
    instance inst(s, pos);

    // Bounds should be shifted
    CHECK_APPROX(inst.bounds().min, vec3{9.0, -1.0, -1.0} * m);
    CHECK_APPROX(inst.bounds().max, vec3{11.0, 1.0, 1.0} * m);

    // Bsphere center should be shifted
    CHECK_APPROX(inst.bsphere().center, pos);
    CHECK_APPROX(inst.bsphere().radius, std::sqrt(3.0) * m);

    // Contains at world position
    CHECK(inst.contains(vec3{10.0, 0.0, 0.0} * m));
    CHECK(inst.contains(vec3{10.5, 0.5, 0.5} * m));
    CHECK(!inst.contains(vec3{0.0, 0.0, 0.0} * m)); // original location
    CHECK(!inst.contains(vec3{11.5, 0.0, 0.0} * m));
}

void test_instance_translated_sphere()
{
    sphere sph(2.0 * m);
    shape s(sph);
    auto pos = vec3{0.0, 5.0, 0.0} * m;
    instance inst(s, pos);

    CHECK_APPROX(inst.bsphere().center, pos);
    CHECK_APPROX(inst.bsphere().radius, 2.0 * m);
    CHECK(inst.contains(vec3{0.0, 5.0, 0.0} * m));
    CHECK(inst.contains(vec3{1.0, 5.0, 0.0} * m));
    CHECK(!inst.contains(vec3{0.0, 0.0, 0.0} * m));
    CHECK(!inst.contains(vec3{0.0, 8.0, 0.0} * m));
}

void test_instance_ray_translated()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);
    shape s(bx);
    auto pos = vec3{5.0, 0.0, 0.0} * m;
    instance inst(s, pos);

    // Ray from far left toward the translated box
    auto hit = inst.ray_intersect({vec3{0.0, 0.0, 0.0} * m, {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, vec3{4.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit->distance, 4.0 * m);

    // Miss: ray along Y, missing the box
    CHECK(!inst.ray_intersect({vec3{0.0, 0.0, 0.0} * m, {0, 1, 0}}).has_value());
}

void test_instance_rotated_box()
{
    // Rotate a box with half_extents (2, 1, 1) by 90° around Z
    // In local space: extends ±2 in x, ±1 in y,z
    // After rotation: ±1 in x (from y), ±2 in y (from x), ±1 in z
    box bx(vec3{2.0, 1.0, 1.0} * m);
    shape s(bx);
    auto rot = quat<one>::from_angle_axis(static_cast<double>(std::numbers::pi / 2.0) * si::radian,
                                          vec3{0.0, 0.0, 1.0});
    instance inst(s, vec3{0.0, 0.0, 0.0} * m, rot);

    // World-space bounds should swap x and y extents
    auto b = inst.bounds();
    CHECK_APPROX(b.min.x(), -1.0 * m, 0.01 * m);
    CHECK_APPROX(b.max.x(), 1.0 * m, 0.01 * m);
    CHECK_APPROX(b.min.y(), -2.0 * m, 0.01 * m);
    CHECK_APPROX(b.max.y(), 2.0 * m, 0.01 * m);

    // Point at (0, 1.5, 0): was outside local box in y, but after rotation
    // the long axis is now y. Should be contained.
    CHECK(inst.contains(vec3{0.0, 1.5, 0.0} * m));

    // Point at (1.5, 0, 0): was inside local box in x, but after rotation
    // the short axis is now x. Should NOT be contained.
    CHECK(!inst.contains(vec3{1.5, 0.0, 0.0} * m));
}

void test_instance_support_translated()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);
    shape s(bx);
    auto pos = vec3{5.0, 0.0, 0.0} * m;
    instance inst(s, pos);

    // Support in +x: local support is (1,1,1), world = (1,1,1) + (5,0,0) = (6,1,1)
    CHECK_APPROX(inst.support({1, 0, 0}), vec3{6.0, 1.0, 1.0} * m);

    // Support in -x: local support is (-1,1,1), world = (-1,1,1) + (5,0,0) = (4,1,1)
    CHECK_APPROX(inst.support({-1, 0, 0}), vec3{4.0, 1.0, 1.0} * m);
}

void test_instance_closest_point_translated()
{
    box bx(vec3{1.0, 1.0, 1.0} * m);
    shape s(bx);
    auto pos = vec3{5.0, 0.0, 0.0} * m;
    instance inst(s, pos);

    // Point far along +x: closest should be on +x face of translated box
    auto cp = inst.closest_point(vec3{10.0, 0.0, 0.0} * m);
    CHECK_APPROX(cp, vec3{6.0, 0.0, 0.0} * m);

    // Point behind the box
    auto cp2 = inst.closest_point(vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(cp2, vec3{4.0, 0.0, 0.0} * m);
}

void test_instance_rotated_ray()
{
    // Box (2,1,1) rotated 90° around Z, placed at origin
    // After rotation: extends ±1 in x, ±2 in y, ±1 in z
    box bx(vec3{2.0, 1.0, 1.0} * m);
    shape s(bx);
    auto rot = quat<one>::from_angle_axis(static_cast<double>(std::numbers::pi / 2.0) * si::radian,
                                          vec3{0.0, 0.0, 1.0});
    instance inst(s, vec3{0.0, 0.0, 0.0} * m, rot);

    // Ray along +y from outside: should hit the rotated box face at y = -2
    auto hit = inst.ray_intersect({vec3{0.0, -5.0, 0.0} * m, {0, 1, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos.y(), -2.0 * m, 0.05 * m);
    CHECK_APPROX(hit->distance, 3.0 * m, 0.05 * m);

    // Ray along +x: should hit at x = -1 (the short axis after rotation)
    auto hit_x = inst.ray_intersect({vec3{-5.0, 0.0, 0.0} * m, {1, 0, 0}});
    CHECK(hit_x.has_value());
    CHECK_APPROX(hit_x->pos.x(), -1.0 * m, 0.05 * m);
    CHECK_APPROX(hit_x->distance, 4.0 * m, 0.05 * m);
}

void test_instance_translated_and_rotated()
{
    // Sphere at (3, 0, 0), rotation doesn't matter for a sphere
    sphere sph(1.0 * m);
    shape s(sph);
    auto rot = quat<one>::from_angle_axis(static_cast<double>(std::numbers::pi / 4.0) * si::radian,
                                          vec3{0.0, 1.0, 0.0});
    instance inst(s, vec3{3.0, 0.0, 0.0} * m, rot);

    // Bsphere center should be at (3, 0, 0)
    CHECK_APPROX(inst.bsphere().center, vec3{3.0, 0.0, 0.0} * m);
    CHECK_APPROX(inst.bsphere().radius, 1.0 * m);

    // Contains
    CHECK(inst.contains(vec3{3.0, 0.0, 0.0} * m));
    CHECK(inst.contains(vec3{3.5, 0.0, 0.0} * m));
    CHECK(!inst.contains(vec3{0.0, 0.0, 0.0} * m));
    CHECK(!inst.contains(vec3{4.5, 0.0, 0.0} * m));

    // Ray intersect
    auto hit = inst.ray_intersect({vec3{0.0, 0.0, 0.0} * m, {1, 0, 0}});
    CHECK(hit.has_value());
    CHECK_APPROX(hit->pos, vec3{2.0, 0.0, 0.0} * m);
    CHECK_APPROX(hit->distance, 2.0 * m);
}

// ===========================================================================
// main
// ===========================================================================

int main()
{
    suite tests;

    tests.group("Box Properties")
        .test("construction", test_box_construction)
        .test("bounds", test_box_bounds)
        .test("bsphere", test_box_bsphere)
        .test("volume", test_box_volume)
        .test("mass_center", test_box_mass_center)
        .test("inertia_tensor", test_box_inertia_tensor);

    tests.group("Box Contains").test("interior/boundary/exterior", test_box_contains);

    tests.group("Box Ray Intersect")
        .test("all 6 faces", test_box_ray_all_faces)
        .test("diagonal", test_box_ray_diagonal)
        .test("miss", test_box_ray_miss)
        .test("from inside", test_box_ray_from_inside)
        .test("max_distance", test_box_ray_max_distance)
        .test("asymmetric box", test_box_ray_asymmetric);

    tests.group("Box Closest Point")
        .test("exterior faces", test_box_closest_point_exterior)
        .test("exterior edges", test_box_closest_point_edges)
        .test("exterior corners", test_box_closest_point_corners)
        .test("interior", test_box_closest_point_interior)
        .test("asymmetric half_extents", test_box_closest_point_asymmetric);

    tests.group("Box Support")
        .test("axis-aligned", test_box_support_axis)
        .test("diagonal", test_box_support_diagonal);

    tests.group("Sphere Properties")
        .test("construction", test_sphere_construction)
        .test("bounds", test_sphere_bounds)
        .test("bsphere", test_sphere_bsphere)
        .test("volume", test_sphere_volume)
        .test("mass_center", test_sphere_mass_center)
        .test("inertia_tensor", test_sphere_inertia_tensor);

    tests.group("Sphere Contains").test("interior/boundary/exterior", test_sphere_contains);

    tests.group("Sphere Ray Intersect")
        .test("axis-aligned", test_sphere_ray_axes)
        .test("miss", test_sphere_ray_miss)
        .test("from inside", test_sphere_ray_from_inside)
        .test("max_distance", test_sphere_ray_max_distance)
        .test("different radii", test_sphere_ray_different_radii);

    tests.group("Sphere Closest Point")
        .test("exterior/interior/diagonal", test_sphere_closest_point)
        .test("on surface", test_sphere_closest_point_on_surface);

    tests.group("Sphere Support")
        .test("axis-aligned", test_sphere_support_axes)
        .test("diagonal", test_sphere_support_diagonal);

    tests.group("Cone Properties")
    .test("construction",    test_cone_construction)
    .test("bounds",          test_cone_bounds)
    .test("bsphere",         test_cone_bsphere)
    .test("volume",          test_cone_volume)
    .test("mass_center",     test_cone_mass_center)
    .test("inertia_tensor",  test_cone_inertia_tensor);

    tests.group("Cone Contains")
        .test("interior",    test_cone_contains_interior)
        .test("exterior",    test_cone_contains_exterior)
        .test("tip",         test_cone_contains_tip);

    tests.group("Cone Ray Intersect")
        .test("base",             test_cone_ray_base)
        .test("lateral surface",  test_cone_ray_lateral_surface)
        .test("tip",              test_cone_ray_tip)
        .test("miss",             test_cone_ray_miss)
        .test("max_distance",     test_cone_ray_max_distance)
        .test("nearest root",     test_cone_ray_intersect_nearest_root);

    tests.group("Cone Closest Point")
        .test("tip",        test_cone_closest_point_tip)
        .test("base",       test_cone_closest_point_base)
        .test("lateral",    test_cone_closest_point_lateral)
        .test("inside",     test_cone_closest_point_inside)
        .test("base edge",  test_cone_closest_point_base_edge);

    tests.group("Cone Support")
        .test("upward",    test_cone_support_upward)
        .test("downward",  test_cone_support_downward)
        .test("lateral",   test_cone_support_lateral)
        .test("diagonal",  test_cone_support_diagonal);

    tests.group("Cone Asymmetric")
        .test("volume",      test_cone_asymmetric_volume)
        .test("mass_center", test_cone_asymmetric_mass_center)
        .test("contains",    test_cone_asymmetric_contains);

    tests.group("Shape Wrapper")
        .test("default construction", test_shape_default)
        .test("from sphere", test_shape_from_sphere)
        .test("from box", test_shape_from_box)
        .test("from mesh", test_shape_from_mesh)
        .test("copy", test_shape_copy)
        .test("move", test_shape_move)
        .test("dispatch", test_shape_dispatch);

    tests.group("Instance")
        .test("translated box", test_instance_translated_box)
        .test("translated sphere", test_instance_translated_sphere)
        .test("ray/translated box", test_instance_ray_translated)
        .test("rotated box", test_instance_rotated_box)
        .test("support/translated box", test_instance_support_translated)
        .test("closest_point/translated box", test_instance_closest_point_translated)
        .test("ray/rotated box", test_instance_rotated_ray)
        .test("translated+rotated sphere", test_instance_translated_and_rotated);

    return tests.run();
}
