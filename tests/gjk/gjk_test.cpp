#include "physkit/collision.h"
#include "physkit/detail/bounds.h"
#include "test.h"

#include <numbers>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

namespace
{

// ============================================================
// AABB vs AABB
// ============================================================

void aabb_aabb_overlap_on_x()
{
    // A extends [-1,1], B extends [0.5, 2.5] - 0.5m overlap on x
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void aabb_aabb_no_collision_x()
{
    // A extends to x=1, B starts at x=2 - 1m gap
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{2.0, -1.0, -1.0} * m, .max = vec3{4.0, 1.0, 1.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void aabb_aabb_no_collision_y()
{
    // Aligned on XZ, separated on Y
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-1.0, 2.0, -1.0} * m, .max = vec3{1.0, 4.0, 1.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void aabb_aabb_no_collision_z()
{
    // Aligned on XY, separated on Z
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-1.0, -1.0, 2.0} * m, .max = vec3{1.0, 1.0, 4.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void aabb_aabb_containment()
{
    // Inner box fully inside outer box
    aabb outer{.min = vec3{-3.0, -3.0, -3.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    aabb inner{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(outer, inner).has_value());
    CHECK(gjk_epa(inner, outer).has_value());
}

void aabb_aabb_large_separation()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{100.0, -1.0, -1.0} * m, .max = vec3{102.0, 1.0, 1.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void aabb_aabb_same_box()
{
    // Minkowski difference of a shape with itself contains the origin
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, a).has_value());
}

void aabb_aabb_partial_overlap_all_axes()
{
    // Corner overlap: A=[0,2]^3, B=[1,3]^3 - overlap region [1,2]^3
    aabb a{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    aabb b{.min = vec3{1.0, 1.0, 1.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void aabb_aabb_separated_diagonal()
{
    // Boxes that don't overlap on any diagonal
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{5.0, 5.0, 5.0} * m, .max = vec3{7.0, 7.0, 7.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

// ============================================================
// OBB vs OBB
// ============================================================

void obb_obb_axis_aligned_overlap()
{
    // Same as AABB overlap but using OBBs with identity orientation
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{1.5, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void obb_obb_axis_aligned_separated()
{
    // 1m gap between faces
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{3.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void obb_obb_same_center_different_orientation()
{
    // Identical cubes at same center with different orientations - always collide
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{0.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void obb_obb_rotated_45_overlap()
{
    // OBB B rotated 45° around Z, center at (1,0,0).
    // After rotation, support in -x is at x = 1 - sqrt(2) ≈ -0.414 < 1 → overlap with A
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{1.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void obb_obb_rotated_45_separated()
{
    // B rotated 45° around Z at (3,0,0).
    // Support of B in -x direction is at x = 3 - sqrt(2) ≈ 1.586 > 1 (A's max x) → gap = 0.586m
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{3.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void obb_obb_cross_config_overlap()
{
    // Two elongated thin boxes forming a cross at origin - classic GJK test
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{2.0, 0.2, 0.2} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{0.0, 0.0, 0.0} * m, rot, vec3{2.0, 0.2, 0.2} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void obb_obb_cross_config_separated()
{
    // Cross configuration but B is elevated so the beams don't touch
    // A: center (0,0,0), identity, half-extents (2, 0.2, 0.2) → y range [-0.2, 0.2]
    // B: center (0, 3, 0), rot90Z, half-extents (2, 0.2, 0.2)
    //   → after rotation: extends ±2 in Y, ±0.2 in X → y range [0.8, 5.2]
    //   Gap on y = 0.8 - 0.2 = 0.6m
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{2.0, 0.2, 0.2} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{0.0, 3.0, 0.0} * m, rot, vec3{2.0, 0.2, 0.2} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

void obb_obb_rotated_90_x_overlap()
{
    // B rotated 90° around X axis, centers close together
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3<one>{1.0, 0.0, 0.0});
    obb b{vec3{0.5, 0.5, 0.5} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void obb_obb_non_uniform_extents_overlap()
{
    // Flat slab vs tall narrow box that intersect
    obb slab{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{3.0, 0.3, 3.0} * m};
    obb pillar{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{0.3, 3.0, 0.3} * m};
    CHECK(gjk_epa(slab, pillar).has_value());
}

void obb_obb_non_uniform_extents_separated()
{
    // Slab and pillar with no overlap: pillar offset so it's beside the slab
    obb slab{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{3.0, 0.3, 3.0} * m};
    obb pillar{vec3{5.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{0.3, 3.0, 0.3} * m};
    CHECK(!gjk_epa(slab, pillar).has_value());
}

void obb_obb_3d_diagonal_overlap()
{
    // Both unit cubes, centers 1.5m apart diagonally → 0.5m overlap on every axis
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{1.5, 1.5, 1.5} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value());
}

void obb_obb_3d_diagonal_separated()
{
    // Centers 3m apart diagonally → 1m gap on every axis
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{3.0, 3.0, 3.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    CHECK(!gjk_epa(a, b).has_value());
}

// ============================================================
// OBB vs AABB  /  AABB vs OBB
// ============================================================

void obb_aabb_axis_aligned_overlap()
{
    obb o{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    aabb a{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    CHECK(gjk_epa(o, a).has_value());
    CHECK(gjk_epa(a, o).has_value());
}

void obb_aabb_axis_aligned_separated()
{
    obb o{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    aabb a{.min = vec3{2.5, -1.0, -1.0} * m, .max = vec3{4.5, 1.0, 1.0} * m};
    CHECK(!gjk_epa(o, a).has_value());
    CHECK(!gjk_epa(a, o).has_value());
}

void obb_aabb_rotated_overlap()
{
    // Rotated OBB that still overlaps with an AABB
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb o{vec3{1.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(o, a).has_value());
    CHECK(gjk_epa(a, o).has_value());
}

void obb_aabb_rotated_separated()
{
    // Rotated OBB with a clear gap from the AABB
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb o{vec3{5.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    CHECK(!gjk_epa(o, a).has_value());
    CHECK(!gjk_epa(a, o).has_value());
}

void obb_aabb_containment()
{
    // Large AABB containing an OBB
    obb o{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{0.5, 0.5, 0.5} * m};
    aabb a{.min = vec3{-2.0, -2.0, -2.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    CHECK(gjk_epa(o, a).has_value());
    CHECK(gjk_epa(a, o).has_value());
}

// ============================================================
// Symmetry: gjk_epa(A,B) must agree with gjk_epa(B,A)
// ============================================================

void symmetry_aabb_aabb_colliding()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value() == gjk_epa(b, a).has_value());
}

void symmetry_aabb_aabb_separated()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{3.0, -1.0, -1.0} * m, .max = vec3{5.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value() == gjk_epa(b, a).has_value());
}

void symmetry_obb_obb_colliding()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot = quat<one>::from_angle_axis((std::numbers::pi / 6.0) * si::radian,
                                          vec3<one>{1.0, 1.0, 0.0}.normalized());
    obb b{vec3{1.0, 1.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value() == gjk_epa(b, a).has_value());
}

void symmetry_obb_obb_separated()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{4.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    CHECK(gjk_epa(a, b).has_value() == gjk_epa(b, a).has_value());
}

// ============================================================
// collision_info fields (EPA is currently a stub returning zeros)
// ============================================================

void collision_info_stub_fields()
{
    // EPA stub explicitly sets all fields to zero - verify those defaults hold
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.0, -1.0, -1.0} * m, .max = vec3{2.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.0 * m);
    CHECK_APPROX(result->local_a, vec3<si::metre>::zero());
    CHECK_APPROX(result->local_b, vec3<si::metre>::zero());
    CHECK_APPROX(result->normal, vec3<one>::zero());
}

void collision_info_nullopt_when_separated()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{3.0, -1.0, -1.0} * m, .max = vec3{5.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(!result.has_value());
}

} // namespace

int main()
{
    suite s;

    s.group("AABB vs AABB")
        .test("overlap on X", aabb_aabb_overlap_on_x)
        .test("no collision X", aabb_aabb_no_collision_x)
        .test("no collision Y", aabb_aabb_no_collision_y)
        .test("no collision Z", aabb_aabb_no_collision_z)
        .test("containment", aabb_aabb_containment)
        .test("large separation", aabb_aabb_large_separation)
        .test("same box", aabb_aabb_same_box)
        .test("partial overlap all axes", aabb_aabb_partial_overlap_all_axes)
        .test("separated diagonal", aabb_aabb_separated_diagonal);

    s.group("OBB vs OBB")
        .test("axis-aligned overlap", obb_obb_axis_aligned_overlap)
        .test("axis-aligned separated", obb_obb_axis_aligned_separated)
        .test("same center different orientation", obb_obb_same_center_different_orientation)
        .test("rotated 45deg overlap", obb_obb_rotated_45_overlap)
        .test("rotated 45deg separated", obb_obb_rotated_45_separated)
        .test("cross config overlap", obb_obb_cross_config_overlap)
        .test("cross config separated", obb_obb_cross_config_separated)
        .test("rotated 90deg X overlap", obb_obb_rotated_90_x_overlap)
        .test("non-uniform extents overlap", obb_obb_non_uniform_extents_overlap)
        .test("non-uniform extents separated", obb_obb_non_uniform_extents_separated)
        .test("3D diagonal overlap", obb_obb_3d_diagonal_overlap)
        .test("3D diagonal separated", obb_obb_3d_diagonal_separated);

    s.group("OBB vs AABB")
        .test("axis-aligned overlap", obb_aabb_axis_aligned_overlap)
        .test("axis-aligned separated", obb_aabb_axis_aligned_separated)
        .test("rotated overlap", obb_aabb_rotated_overlap)
        .test("rotated separated", obb_aabb_rotated_separated)
        .test("containment", obb_aabb_containment);

    s.group("Symmetry")
        .test("aabb-aabb colliding", symmetry_aabb_aabb_colliding)
        .test("aabb-aabb separated", symmetry_aabb_aabb_separated)
        .test("obb-obb colliding", symmetry_obb_obb_colliding)
        .test("obb-obb separated", symmetry_obb_obb_separated);

    s.group("collision_info")
        .test("stub fields are zero", collision_info_stub_fields)
        .test("nullopt when separated", collision_info_nullopt_when_separated);

    return s.run();
}
