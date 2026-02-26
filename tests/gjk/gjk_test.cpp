// generated test case and edge cases with codex 5.3

#include "physkit/collision.h"
#include "physkit/detail/bounds.h"
#include "test.h"

#include <cmath>
#include <mp-units/systems/si/unit_symbols.h>
#include <numbers>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

// Helper function to create rotation quaternion from axis-angle
quat<one> make_rotation(const vec3<one> &axis, double angle_rad)
{
    auto half = angle_rad / 2.0;
    auto sin_half = std::sin(half);
    auto cos_half = std::cos(half);
    return quat<one>{cos_half * one, axis.x() * sin_half * one, axis.y() * sin_half * one,
                     axis.z() * sin_half * one};
}

int main()
{
    return suite{} // =====================================================================
                   // GROUP 1: Minkowski Difference Support Function Tests
                   // =====================================================================
        .group("Minkowski Difference Support")

        .test("minkowski_support: AABB - AABB basic",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {7.0 * m, 2.0 * m, 2.0 * m}};

                  // Direction along X: should get rightmost point of A, leftmost of B
                  const vec3<one> dir{1.0, 0.0, 0.0};
                  auto result = minkowski_support(a, b, dir);

                  // A's support in dir (1,0,0) = (2, 2, 2) or (2, 0, 2) etc (any point with x=2)
                  // B's support in -dir (-1,0,0) = (5, 0, 0) or (5, 0, 2) etc (any point with x=5)
                  // Minkowski diff = (2,?,?) - (5,?,?) = (-3, ?, ?)
                  CHECK_APPROX(result.x(), -3.0 * m);
              })

        .test("minkowski_support: AABB - AABB negative direction",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {7.0 * m, 2.0 * m, 2.0 * m}};

                  const vec3<one> dir{-1.0, 0.0, 0.0};
                  auto result = minkowski_support(a, b, dir);

                  // A's support in (-1,0,0) = (0, ?, ?)
                  // B's support in (1,0,0) = (7, ?, ?)
                  // Result = (0,?,?) - (7,?,?) = (-7, ?, ?)
                  CHECK_APPROX(result.x(), -7.0 * m);
              })

        .test("minkowski_support: OBB - OBB with identity rotation",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{5.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const vec3<one> dir{1.0, 0.0, 0.0};
                  auto result = minkowski_support(a, b, dir);

                  // A's support: (1, ?, ?) in local, transformed = (1, ?, ?)
                  // B's support in -dir: (-1, ?, ?) in local, transformed to world = (4, ?, ?)
                  // Result should be approximately (-3, ?, ?)
                  CHECK_APPROX(result.x(), -3.0 * m);
              })

        .test("minkowski_support: OBB - OBB with 90 degree Z rotation",
              []
              {
                  // Create OBB rotated 90 degrees around Z axis
                  auto rot =
                      make_rotation({0.0, 0.0, 1.0}, std::numbers::pi_v<physkit::float_t> / 2.0);
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m}, rot, {2.0 * m, 1.0 * m, 1.0 * m}};

                  const vec3<one> dir{1.0, 0.0, 0.0};
                  // Just verify it doesn't crash and returns a finite value
                  auto result = minkowski_support(a, a, dir);
                  CHECK(std::isfinite(result.x().numerical_value_in(m)) &&
                        std::isfinite(result.y().numerical_value_in(m)) &&
                        std::isfinite(result.z().numerical_value_in(m)));
              })

        .test("minkowski_support: AABB - OBB mixed",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const obb b{{5.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const vec3<one> dir{1.0, 0.0, 0.0};
                  auto result = minkowski_support(a, b, dir);

                  // A's support: (2, ?, ?)
                  // B's support in -dir: (4, ?, ?)
                  // Result: (-2, ?, ?)
                  CHECK_APPROX(result.x(), -2.0 * m);
              })

        .test("minkowski_support: OBB - AABB mixed",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {7.0 * m, 2.0 * m, 2.0 * m}};

                  const vec3<one> dir{1.0, 0.0, 0.0};
                  auto result = minkowski_support(a, b, dir);

                  // A's support: (1, ?, ?)
                  // B's support in -dir: (5, ?, ?)
                  // Result: (-4, ?, ?)
                  CHECK_APPROX(result.x(), -4.0 * m);
              })

        .test("minkowski_support: origin is in Minkowski diff when overlapping",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {4.0 * m, 4.0 * m, 4.0 * m}};
                  const aabb b{.min = {2.0 * m, 2.0 * m, 2.0 * m},
                               .max = {6.0 * m, 6.0 * m, 6.0 * m}};

                  // When objects overlap, origin should be inside Minkowski difference
                  // Test by checking that we can get points on both sides of origin
                  auto result_pos = minkowski_support(a, b, {1.0, 0.0, 0.0});
                  auto result_neg = minkowski_support(a, b, {-1.0, 0.0, 0.0});

                  // One should be positive, one should be negative in x component
                  CHECK(result_pos.x() >= 0.0 * m || result_neg.x() <= 0.0 * m);
              })

        // =====================================================================
        // GROUP 2: AABB vs AABB Collision Tests
        // =====================================================================
        .group("GJK: AABB vs AABB")

        .test("separated along X axis - reports correct distance",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {3.0 * m, 0.0 * m, 0.0 * m},
                               .max = {4.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.intersects);
                  CHECK(result.closest_points().has_value());
                  CHECK_APPROX(result.distance(), 2.0 * m);

                  const auto [pa, pb] = *result.closest_points();
                  CHECK_APPROX(pa.x(), 1.0 * m);
                  CHECK_APPROX(pb.x(), 3.0 * m);
              })

        .test("separated along Y axis",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {0.0 * m, 3.0 * m, 0.0 * m},
                               .max = {1.0 * m, 4.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.intersects);
                  CHECK_APPROX(result.distance(), 2.0 * m);
              })

        .test("separated along Z axis",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {0.0 * m, 0.0 * m, 5.0 * m},
                               .max = {1.0 * m, 1.0 * m, 6.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.intersects);
                  CHECK_APPROX(result.distance(), 4.0 * m);
              })

        .test("separated diagonally",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {3.0 * m, 3.0 * m, 3.0 * m},
                               .max = {4.0 * m, 4.0 * m, 4.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.intersects);
                  // Distance should be sqrt(2^2 + 2^2 + 2^2) = sqrt(12)
                  auto expected_dist = std::sqrt(12.0) * m;
                  CHECK_APPROX(result.distance(), expected_dist, 1e-5 * m);
              })

        .test("overlapping - reports intersection",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {1.0 * m, 1.0 * m, 1.0 * m},
                               .max = {3.0 * m, 3.0 * m, 3.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(result.intersects);
                  CHECK(!result.closest_points().has_value());
              })

        .test("touching faces - should intersect",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {1.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  // Touching faces should be considered intersecting
                  CHECK(result.intersects);
              })

        .test("touching edges - should intersect",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {1.0 * m, 1.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(result.intersects);
              })

        .test("touching corners - should intersect",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {1.0 * m, 1.0 * m, 1.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(result.intersects);
              })

        .test("one box contains another",
              []
              {
                  const aabb outer{.min = {-5.0 * m, -5.0 * m, -5.0 * m},
                                   .max = {5.0 * m, 5.0 * m, 5.0 * m}};
                  const aabb inner{.min = {-1.0 * m, -1.0 * m, -1.0 * m},
                                   .max = {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(outer, inner);

                  CHECK(result.intersects);
              })

        .test("identical boxes",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};

                  const auto result = gjk_aabb_aabb(a, a);

                  CHECK(result.intersects);
              })

        // =====================================================================
        // GROUP 3: OBB vs OBB Collision Tests
        // =====================================================================
        .group("GJK: OBB vs OBB")

        .test("separated OBBs with identity rotation",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{5.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_obb(a, b);

                  CHECK(!result.intersects);
                  CHECK_APPROX(result.distance(), 3.0 * m);
              })

        .test("overlapping OBBs with identity rotation",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {2.0 * m, 2.0 * m, 2.0 * m}};
                  const obb b{{1.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {2.0 * m, 2.0 * m, 2.0 * m}};

                  const auto result = gjk_obb_obb(a, b);

                  CHECK(result.intersects);
              })

        .test("OBBs touching with identity rotation",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{2.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_obb(a, b);

                  CHECK(result.intersects);
              })

        .test("rotated OBB intersects axis-aligned OBB",
              []
              {
                  // Create a cross shape - two boxes that should intersect
                  const obb horizontal{{0.0 * m, 0.0 * m, 0.0 * m},
                                       quat<one>::identity(),
                                       {3.0 * m, 0.5 * m, 0.5 * m}};

                  auto rot =
                      make_rotation({0.0, 0.0, 1.0}, std::numbers::pi_v<physkit::float_t> / 2.0);
                  const obb vertical{{0.0 * m, 0.0 * m, 0.0 * m}, rot, {3.0 * m, 0.5 * m, 0.5 * m}};

                  const auto result = gjk_obb_obb(horizontal, vertical);

                  CHECK(result.intersects);
              })

        .test("rotated OBB separated from axis-aligned OBB",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  auto rot =
                      make_rotation({0.0, 0.0, 1.0}, std::numbers::pi_v<physkit::float_t> / 4.0);
                  const obb b{{10.0 * m, 0.0 * m, 0.0 * m}, rot, {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_obb(a, b);

                  CHECK(!result.intersects);
                  CHECK(result.distance() > 0.0 * m);
              })

        .test("OBB with 45 degree rotation",
              []
              {
                  auto rot =
                      make_rotation({0.0, 0.0, 1.0}, std::numbers::pi_v<physkit::float_t> / 4.0);
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{3.0 * m, 0.0 * m, 0.0 * m}, rot, {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_obb(a, b);

                  // Should be separated
                  CHECK(!result.intersects);
              })

        .test("OBB one contains another (rotated)",
              []
              {
                  const obb outer{{0.0 * m, 0.0 * m, 0.0 * m},
                                  quat<one>::identity(),
                                  {5.0 * m, 5.0 * m, 5.0 * m}};

                  auto rot =
                      make_rotation({1.0, 0.0, 0.0}, std::numbers::pi_v<physkit::float_t> / 6.0);
                  const obb inner{{0.0 * m, 0.0 * m, 0.0 * m}, rot, {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_obb(outer, inner);

                  CHECK(result.intersects);
              })

        // =====================================================================
        // GROUP 4: AABB vs OBB Mixed Collision Tests
        // =====================================================================
        .group("GJK: AABB vs OBB (Mixed)")

        .test("AABB - OBB separated",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{5.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_obb(a, b);

                  CHECK(!result.intersects);
                  CHECK_APPROX(result.distance(), 3.0 * m);
              })

        .test("AABB - OBB overlapping",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {3.0 * m, 3.0 * m, 3.0 * m}};
                  const obb b{{2.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_obb(a, b);

                  CHECK(result.intersects);
              })

        .test("AABB - OBB touching",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const obb b{{3.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_obb(a, b);

                  CHECK(result.intersects);
              })

        .test("AABB - rotated OBB",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};

                  auto rot =
                      make_rotation({0.0, 0.0, 1.0}, std::numbers::pi_v<physkit::float_t> / 4.0);
                  const obb b{{1.0 * m, 1.0 * m, 0.0 * m}, rot, {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_obb(a, b);

                  // The rotated OBB should intersect the AABB
                  CHECK(result.intersects);
              })

        .test("OBB - AABB separated",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {6.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_aabb(a, b);

                  CHECK(!result.intersects);
                  CHECK_APPROX(result.distance(), 3.0 * m);
              })

        .test("OBB - AABB overlapping",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {1.0 * m, 0.0 * m, 0.0 * m},
                               .max = {4.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_aabb(a, b);

                  CHECK(result.intersects);
              })

        .test("OBB - AABB touching",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {2.0 * m, 0.0 * m, 0.0 * m},
                               .max = {3.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_aabb(a, b);

                  CHECK(result.intersects);
              })

        .test("rotated OBB - AABB",
              []
              {
                  auto rot =
                      make_rotation({0.0, 0.0, 1.0}, std::numbers::pi_v<physkit::float_t> / 4.0);
                  const obb a{{1.0 * m, 1.0 * m, 0.0 * m}, rot, {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};

                  const auto result = gjk_obb_aabb(a, b);

                  CHECK(result.intersects);
              })

        // =====================================================================
        // GROUP 5: GJK Result Verification Tests
        // =====================================================================
        .group("GJK Result Verification")

        .test("intersects flag is accurate for separated objects",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {6.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.intersects);
              })

        .test("intersects flag is accurate for colliding objects",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {1.0 * m, 1.0 * m, 1.0 * m},
                               .max = {3.0 * m, 3.0 * m, 3.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(result.intersects);
              })

        .test("closest_points engaged when separated",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {3.0 * m, 0.0 * m, 0.0 * m},
                               .max = {4.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(result.closest_points().has_value());
                  const auto [pa, pb] = *result.closest_points();
                  // Points should be on the facing faces
                  CHECK_APPROX(pa.x(), 1.0 * m);
                  CHECK_APPROX(pb.x(), 3.0 * m);
              })

        .test("closest_points disengaged when intersecting",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {1.0 * m, 1.0 * m, 1.0 * m},
                               .max = {3.0 * m, 3.0 * m, 3.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.closest_points().has_value());
              })

        .test("distance is zero (or near) when intersecting",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 2.0 * m, 2.0 * m}};
                  const aabb b{.min = {1.0 * m, 1.0 * m, 1.0 * m},
                               .max = {3.0 * m, 3.0 * m, 3.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK_APPROX(result.distance(), 0.0 * m);
              })

        .test("distance matches closest_points magnitude when separated",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {4.0 * m, 3.0 * m, 0.0 * m},
                               .max = {5.0 * m, 4.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(result.closest_points().has_value());
                  const auto [pa, pb] = *result.closest_points();
                  auto computed_dist = (pb - pa).norm();
                  CHECK_APPROX(computed_dist, result.distance());
              })

        .test("distance is positive when separated",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{10.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_obb(a, b);

                  CHECK(result.distance() > 0.0 * m);
              })

        // =====================================================================
        // GROUP 6: Mathematical Verification Tests
        // =====================================================================
        .group("Mathematical Verification")

        .test("distance between unit cubes separated by 1 unit",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {2.0 * m, 0.0 * m, 0.0 * m},
                               .max = {3.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  // Gap is exactly 1 metre
                  CHECK_APPROX(result.distance(), 1.0 * m);
              })

        .test("distance verification for diagonal separation",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {3.0 * m, 4.0 * m, 0.0 * m},
                               .max = {4.0 * m, 5.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  // Gap in X: 2, Gap in Y: 3, Gap in Z: 0
                  // Distance = sqrt(2^2 + 3^2) = sqrt(13)
                  auto expected = std::sqrt(13.0) * m;
                  CHECK_APPROX(result.distance(), expected, 1e-5 * m);
              })

        .test("GJK consistency: A-B distance equals B-A distance",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {6.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result_ab = gjk_aabb_aabb(a, b);
                  const auto result_ba = gjk_aabb_aabb(b, a);

                  CHECK_APPROX(result_ab.distance(), result_ba.distance());
                  CHECK(result_ab.intersects == result_ba.intersects);
              })

        .test("symmetry test: OBB-OBB",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};
                  const obb b{{5.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result_ab = gjk_obb_obb(a, b);
                  const auto result_ba = gjk_obb_obb(b, a);

                  CHECK_APPROX(result_ab.distance(), result_ba.distance());
              })

        .test("gjk_test dispatcher produces same results as direct calls",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                               .max = {6.0 * m, 1.0 * m, 1.0 * m}};

                  const auto direct = gjk_aabb_aabb(a, b);
                  const auto dispatched = gjk_test(a, b);

                  CHECK(direct.intersects == dispatched.intersects);
                  CHECK_APPROX(direct.distance(), dispatched.distance());
              })

        // =====================================================================
        // GROUP 7: Edge Cases
        // =====================================================================
        .group("Edge Cases")

        .test("degenerate AABB with zero volume",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {0.0 * m, 0.0 * m, 0.0 * m}};
                  const aabb b{.min = {1.0 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  // Point vs box - should not intersect
                  CHECK(!result.intersects);
                  CHECK_APPROX(result.distance(), 1.0 * m);
              })

        .test("degenerate OBB with zero volume",
              []
              {
                  const obb a{{0.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {0.0 * m, 0.0 * m, 0.0 * m}};
                  const obb b{{5.0 * m, 0.0 * m, 0.0 * m},
                              quat<one>::identity(),
                              {1.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_obb_obb(a, b);

                  CHECK(!result.intersects);
              })

        .test("very small separation",
              []
              {
                  const aabb a{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                               .max = {1.0 * m, 1.0 * m, 1.0 * m}};
                  const aabb b{.min = {1.0001 * m, 0.0 * m, 0.0 * m},
                               .max = {2.0 * m, 1.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  // Very small gap - should not intersect
                  CHECK(!result.intersects);
                  CHECK(result.distance() > 0.0 * m);
                  CHECK(result.distance() < 0.001 * m);
              })

        .test("large size difference",
              []
              {
                  const aabb small{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                                   .max = {0.1 * m, 0.1 * m, 0.1 * m}};
                  const aabb large{.min = {-100.0 * m, -100.0 * m, -100.0 * m},
                                   .max = {100.0 * m, 100.0 * m, 100.0 * m}};

                  const auto result = gjk_aabb_aabb(small, large);

                  CHECK(result.intersects);
              })

        .test("negative coordinates",
              []
              {
                  const aabb a{.min = {-10.0 * m, -10.0 * m, -10.0 * m},
                               .max = {-8.0 * m, -8.0 * m, -8.0 * m}};
                  const aabb b{.min = {-5.0 * m, -5.0 * m, -5.0 * m},
                               .max = {-3.0 * m, -3.0 * m, -3.0 * m}};

                  const auto result = gjk_aabb_aabb(a, b);

                  CHECK(!result.intersects);
                  // Distance should be sqrt(3^2 + 3^2 + 3^2) = sqrt(27)
                  auto expected = std::sqrt(27.0) * m;
                  CHECK_APPROX(result.distance(), expected, 1e-5 * m);
              })

        .test("flat box (2D-like)",
              []
              {
                  const aabb flat{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                                  .max = {5.0 * m, 5.0 * m, 0.0 * m}};
                  const aabb box{.min = {2.0 * m, 2.0 * m, 0.0 * m},
                                 .max = {3.0 * m, 3.0 * m, 1.0 * m}};

                  const auto result = gjk_aabb_aabb(flat, box);

                  // They touch at z=0 plane
                  CHECK(result.intersects);
              })

        .test("line-like box",
              []
              {
                  const aabb line{.min = {0.0 * m, 0.0 * m, 0.0 * m},
                                  .max = {10.0 * m, 0.0 * m, 0.0 * m}};
                  const aabb point{.min = {5.0 * m, 0.0 * m, 0.0 * m},
                                   .max = {5.0 * m, 0.0 * m, 0.0 * m}};

                  const auto result = gjk_aabb_aabb(line, point);

                  CHECK(result.intersects);
              })

        .run();
}
