#include "physkit/collision.h"
#include "physkit/detail/bounds.h"
#include "test.h"

#include <cmath>
#include <numbers>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

namespace
{

// Tolerance for EPA convergence (matches EPA internal tolerance of 1e-6)
constexpr auto depth_tol = 1e-4 * m;
// Looser tolerance for discretized meshes (spheres, etc.)
constexpr auto mesh_tol = 0.15 * m;
// Tolerance for OBB symmetry checks (iterative convergence can differ by order)
constexpr auto sym_tol = 0.05 * m;

// Helper: verify that a collision normal is unit length
void check_unit_normal(const vec3<one> &normal)
{
    auto norm_val = normal.norm().numerical_value_in(one);
    CHECK_APPROX(norm_val, 1.0, 1e-6);
}

// Helper: verify the MTV separates two shapes.
// Normal points from B toward A, so moving A by +normal * depth pushes A
// away from B, separating the shapes.
void check_mtv_separates(const aabb &a, const aabb &b, const collision_info &info)
{
    constexpr auto nudge = 1e-3 * m;
    auto offset = info.normal * (info.depth + nudge);
    aabb a_moved{.min = a.min + offset, .max = a.max + offset};
    CHECK(!gjk_epa(a_moved, b).has_value());
}

void check_mtv_separates(const obb &a, const obb &b, const collision_info &info)
{
    constexpr auto nudge = 1e-3 * m;
    auto offset = info.normal * (info.depth + nudge);
    obb a_moved{a.center + offset, a.orientation, a.half_extents};
    CHECK(!gjk_epa(a_moved, b).has_value());
}

// ============================================================
// EPA Depth Accuracy: AABB vs AABB
//
// The EPA depth is the distance from the origin to the closest face of the
// Minkowski difference D = A - B. For axis-aligned boxes this equals
//   min over all 6 faces of { a.max_i - b.min_i, b.max_i - a.min_i }
// ============================================================

void epa_aabb_depth_overlap_x()
{
    // A: [-1,1]^3, B: [0.5, 2.5] x [-1,1]^2
    // D faces: +x: 1-0.5=0.5, -x: 1+1=2, y/z: 2 each. Min = 0.5m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_overlap_y()
{
    // D faces: y: 0.5m each (min), x/z: 2m. Min = 0.5m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-1.0, 0.5, -1.0} * m, .max = vec3{1.0, 2.5, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_overlap_z()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-1.0, -1.0, 0.5} * m, .max = vec3{1.0, 1.0, 2.5} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_small_overlap()
{
    // +x face: 1-0.9=0.1. Min = 0.1m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.9, -1.0, -1.0} * m, .max = vec3{2.9, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.1 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_large_overlap()
{
    // A: [-1,1]^3, B: [-0.5,1.5]x[-1,1]^2
    // +x: 1-(-0.5)=1.5, -x: 1.5-(-1)=2.5, y/z: 2. Min = 1.5m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-0.5, -1.0, -1.0} * m, .max = vec3{1.5, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 1.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_containment()
{
    // Outer: [-2,2]^3, Inner: [-0.5,0.5]^3
    // D = outer - inner. D_i: [outer.min_i - inner.max_i, outer.max_i - inner.min_i]
    // D = [-2.5, 2.5]^3. Distance from origin to each face = 2.5m
    aabb outer{.min = vec3{-2.0, -2.0, -2.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    aabb inner{.min = vec3{-0.5, -0.5, -0.5} * m, .max = vec3{0.5, 0.5, 0.5} * m};
    auto result = gjk_epa(outer, inner);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 2.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_identical()
{
    // D = A - A = [-2,2]^3. Distance from origin = 2.0m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, a);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 2.0 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_asymmetric()
{
    // A: [-1,1]^3, B: [-0.5,0.5]x[-1,1]^2
    // D: +x: 1-(-0.5)=1.5, -x: 0.5-(-1)=1.5, y/z: 2. Min = 1.5m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-0.5, -1.0, -1.0} * m, .max = vec3{0.5, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 1.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_aabb_depth_corner_overlap()
{
    // A: [0,2]^3, B: [1,3]^3
    // D: +i: 2-1=1, -i: 3-0=3. Min = 1.0m
    aabb a{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    aabb b{.min = vec3{1.0, 1.0, 1.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 1.0 * m, depth_tol);
    check_unit_normal(result->normal);
}

// ============================================================
// EPA Normal Direction: AABB vs AABB
// ============================================================

void epa_aabb_normal_direction_x()
{
    // Min depth is on X (0.5m vs 2.0m on Y,Z) → normal along ±X
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(std::abs(result->normal.x().numerical_value_in(one)) > 0.9);
    CHECK(std::abs(result->normal.y().numerical_value_in(one)) < 0.1);
    CHECK(std::abs(result->normal.z().numerical_value_in(one)) < 0.1);
}

void epa_aabb_normal_direction_y()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-1.0, 0.5, -1.0} * m, .max = vec3{1.0, 2.5, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(std::abs(result->normal.x().numerical_value_in(one)) < 0.1);
    CHECK(std::abs(result->normal.y().numerical_value_in(one)) > 0.9);
    CHECK(std::abs(result->normal.z().numerical_value_in(one)) < 0.1);
}

void epa_aabb_normal_direction_z()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{-1.0, -1.0, 0.5} * m, .max = vec3{1.0, 1.0, 2.5} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(std::abs(result->normal.x().numerical_value_in(one)) < 0.1);
    CHECK(std::abs(result->normal.y().numerical_value_in(one)) < 0.1);
    CHECK(std::abs(result->normal.z().numerical_value_in(one)) > 0.9);
}

void epa_aabb_normal_is_unit_length()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, 0.3, -0.2} * m, .max = vec3{2.5, 2.3, 1.8} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_unit_normal(result->normal);
}

// ============================================================
// EPA MTV Validity: Applying the MTV separates shapes
// ============================================================

void epa_mtv_separates_aabb_x()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_mtv_separates(a, b, *result);
}

void epa_mtv_separates_aabb_diagonal()
{
    aabb a{.min = vec3{0.0, 0.0, 0.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    aabb b{.min = vec3{1.0, 1.0, 1.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_mtv_separates(a, b, *result);
}

void epa_mtv_separates_aabb_containment()
{
    aabb outer{.min = vec3{-3.0, -3.0, -3.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    aabb inner{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(outer, inner);
    CHECK(result.has_value());
    check_mtv_separates(outer, inner, *result);
}

void epa_mtv_separates_obb_axis_aligned()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{1.5, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_mtv_separates(a, b, *result);
}

void epa_mtv_separates_obb_rotated()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{1.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_mtv_separates(a, b, *result);
}

void epa_mtv_separates_obb_cross()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{2.0, 0.2, 0.2} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{0.0, 0.0, 0.0} * m, rot, vec3{2.0, 0.2, 0.2} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_mtv_separates(a, b, *result);
}

void epa_mtv_separates_obb_3d_rotation()
{
    auto rot = quat<one>::from_angle_axis((std::numbers::pi / 6.0) * si::radian,
                                          vec3<one>{1.0, 1.0, 1.0}.normalized());
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{1.0, 0.5, 0.3} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    check_mtv_separates(a, b, *result);
}

// ============================================================
// EPA Symmetry: gjk_epa(A,B) vs gjk_epa(B,A)
// ============================================================

void epa_symmetry_depth_aabb()
{
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto r1 = gjk_epa(a, b);
    auto r2 = gjk_epa(b, a);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    CHECK_APPROX(r1->depth, r2->depth, depth_tol);
}

void epa_symmetry_normal_aabb()
{
    // Normals should be approximately opposite
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto r1 = gjk_epa(a, b);
    auto r2 = gjk_epa(b, a);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    auto sum = r1->normal + r2->normal;
    CHECK(sum.norm().numerical_value_in(one) < 0.1);
}

void epa_symmetry_depth_obb()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{1.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto r1 = gjk_epa(a, b);
    auto r2 = gjk_epa(b, a);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    // EPA is iterative; rotated configs may converge slightly differently
    CHECK_APPROX(r1->depth, r2->depth, sym_tol);
}

void epa_symmetry_normal_obb()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{1.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto r1 = gjk_epa(a, b);
    auto r2 = gjk_epa(b, a);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    // For non-symmetric shapes, normals may not be exactly opposite but should be close
    auto sum = r1->normal + r2->normal;
    CHECK(sum.norm().numerical_value_in(one) < 0.3);
}

void epa_symmetry_depth_containment()
{
    aabb outer{.min = vec3{-3.0, -3.0, -3.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    aabb inner{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    auto r1 = gjk_epa(outer, inner);
    auto r2 = gjk_epa(inner, outer);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    CHECK_APPROX(r1->depth, r2->depth, depth_tol);
}

// ============================================================
// EPA with OBBs: depth & normal
// ============================================================

void epa_obb_axis_aligned_depth()
{
    // Two unit cubes 1.5m apart on X. Same as AABB: depth = 0.5m
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{1.5, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    CHECK(std::abs(result->normal.x().numerical_value_in(one)) > 0.9);
    check_unit_normal(result->normal);
}

void epa_obb_identical_at_origin()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, a);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 2.0 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_obb_cross_config_depth()
{
    // Beam A: [-2,2]x[-0.2,0.2]x[-0.2,0.2]
    // Beam B (rotated 90 Z): [-0.2,0.2]x[-2,2]x[-0.2,0.2]
    // D on z: [-0.4, 0.4]. Min face distance = 0.4m
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{2.0, 0.2, 0.2} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{0.0, 0.0, 0.0} * m, rot, vec3{2.0, 0.2, 0.2} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.4 * m, 1e-2 * m);
    check_unit_normal(result->normal);
}

void epa_obb_rotated_45_depth()
{
    // Verify depth > 0 and MTV separates. Exact depth hard to compute analytically.
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    obb b{vec3{1.0, 0.0, 0.0} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
    check_mtv_separates(a, b, *result);
}

void epa_obb_non_uniform_slab_pillar()
{
    // Slab: [-3,3]x[-0.3,0.3]x[-3,3], Pillar: [-0.3,0.3]x[-3,3]x[-0.3,0.3]
    // D = [-3.3,3.3]^3 (axis-aligned identity OBBs). Min face distance = 3.3m
    obb slab{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{3.0, 0.3, 3.0} * m};
    obb pillar{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{0.3, 3.0, 0.3} * m};
    auto result = gjk_epa(slab, pillar);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 3.3 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_obb_rotated_90_x()
{
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3<one>{1.0, 0.0, 0.0});
    obb b{vec3{0.5, 0.5, 0.5} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
    check_mtv_separates(a, b, *result);
}

// ============================================================
// EPA with OBB vs AABB: cross-type depth & MTV
// ============================================================

void epa_obb_aabb_axis_aligned_depth()
{
    obb o{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    aabb a{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto r1 = gjk_epa(o, a);
    auto r2 = gjk_epa(a, o);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    CHECK_APPROX(r1->depth, 0.5 * m, depth_tol);
    CHECK_APPROX(r2->depth, 0.5 * m, depth_tol);
}

void epa_obb_aabb_containment_depth()
{
    // OBB: [-0.5,0.5]^3, AABB: [-2,2]^3
    // D = OBB - AABB: [-0.5-2, 0.5+2]^3 = [-2.5, 2.5]^3. Depth = 2.5m
    obb o{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{0.5, 0.5, 0.5} * m};
    aabb a{.min = vec3{-2.0, -2.0, -2.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    auto result = gjk_epa(o, a);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 2.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

// ============================================================
// EPA with mesh::instance
// ============================================================

void epa_mesh_box_box_depth()
{
    // Two box meshes (half-extents 1m), 1.5m apart on X → depth 0.5m
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.5, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_mesh_box_box_containment_depth()
{
    // outer half=3, inner half=0.5, both at origin
    // D = [-3.5, 3.5]^3. Depth = 3.5m
    auto outer = mesh::box(vec3{3.0, 3.0, 3.0} * m);
    auto inner = mesh::box(vec3{0.5, 0.5, 0.5} * m);
    auto a = outer->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = inner->at(vec3{0.0, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 3.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_mesh_box_box_identical_depth()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto result = gjk_epa(a, a);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 2.0 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_mesh_box_box_normal_direction()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.5, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(std::abs(result->normal.x().numerical_value_in(one)) > 0.9);
}

void epa_mesh_box_box_symmetry()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.5, 0.0, 0.0} * m);
    auto r1 = gjk_epa(a, b);
    auto r2 = gjk_epa(b, a);
    CHECK(r1.has_value());
    CHECK(r2.has_value());
    CHECK_APPROX(r1->depth, r2->depth, depth_tol);
}

void epa_mesh_box_box_rotated_mtv()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.0, 0.0, 0.0} * m, rot);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
    // Verify MTV separates: normal points B→A, so move A by +normal * depth
    constexpr auto nudge = 1e-3 * m;
    auto offset = result->normal * (result->depth + nudge);
    auto a_moved = msh->at(offset);
    CHECK(!gjk_epa(a_moved, b).has_value());
}

void epa_mesh_sphere_sphere_depth()
{
    // Two unit spheres 1.5m apart → expected depth ~0.5m
    auto sph = mesh::sphere(1.0 * m);
    auto a = sph->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{1.5, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, mesh_tol);
    check_unit_normal(result->normal);
}

void epa_mesh_sphere_sphere_normal()
{
    auto sph = mesh::sphere(1.0 * m);
    auto a = sph->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{1.5, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(std::abs(result->normal.x().numerical_value_in(one)) > 0.8);
}

void epa_mesh_sphere_sphere_diagonal()
{
    // Two unit spheres along (1,1,0), distance sqrt(2)
    // depth ≈ 2 - sqrt(2) ≈ 0.586m
    auto sph = mesh::sphere(1.0 * m);
    auto a = sph->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{1.0, 1.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    auto expected_depth = (2.0 - std::numbers::sqrt2) * m;
    CHECK_APPROX(result->depth, expected_depth, mesh_tol);
    check_unit_normal(result->normal);
}

void epa_mesh_box_sphere_depth()
{
    // Unit box at origin, unit sphere at (1.5, 0, 0)
    auto box_msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto sph_msh = mesh::sphere(1.0 * m);
    auto a = box_msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph_msh->at(vec3{1.5, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, mesh_tol);
    check_unit_normal(result->normal);
}

// ============================================================
// EPA with mesh::pyramid
// ============================================================

void epa_pyramid_depth_overlap()
{
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 1.5, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
}

void epa_pyramid_mtv_separates()
{
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 1.5, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    constexpr auto nudge = 1e-3 * m;
    auto offset = result->normal * (result->depth + nudge);
    auto a_moved = pyr->at(offset);
    CHECK(!gjk_epa(a_moved, b).has_value());
}

void epa_pyramid_box_mtv()
{
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto box_msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = box_msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
    constexpr auto nudge = 1e-3 * m;
    auto offset = result->normal * (result->depth + nudge);
    auto a_moved = pyr->at(offset);
    CHECK(!gjk_epa(a_moved, b).has_value());
}

void epa_pyramid_flipped_depth()
{
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto rot = quat<one>::from_angle_axis(std::numbers::pi * si::radian, vec3<one>{0.0, 0.0, 1.0});
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 3.0, 0.0} * m, rot);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
}

// ============================================================
// EPA Depth Positive Invariant
// ============================================================

void epa_depth_always_positive()
{
    auto box_msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);

    struct config
    {
        vec3<si::metre> pos_a, pos_b;
        quat<one> rot_b;
    };

    config configs[] = {
        {vec3{0, 0, 0} * m, vec3{1.5, 0, 0} * m, quat<one>::identity()},
        {vec3{0, 0, 0} * m, vec3{0, 1.5, 0} * m, quat<one>::identity()},
        {vec3{0, 0, 0} * m, vec3{0, 0, 1.5} * m, quat<one>::identity()},
        {vec3{0, 0, 0} * m, vec3{0.5, 0.5, 0.5} * m, quat<one>::identity()},
        {vec3{0, 0, 0} * m, vec3{0, 0, 0} * m, quat<one>::identity()},
        {vec3{0, 0, 0} * m, vec3{1.0, 0, 0} * m,
         quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian,
                                    vec3<one>{0.0, 0.0, 1.0})},
        {vec3{0, 0, 0} * m, vec3{0.3, 0.3, 0.3} * m,
         quat<one>::from_angle_axis((std::numbers::pi / 6.0) * si::radian,
                                    vec3<one>{1.0, 1.0, 1.0}.normalized())},
    };

    for (const auto &[pos_a, pos_b, rot_b] : configs)
    {
        auto a = box_msh->at(pos_a);
        auto b = box_msh->at(pos_b, rot_b);
        auto result = gjk_epa(a, b);
        if (result.has_value())
        {
            CHECK(result->depth > 0.0 * m);
            check_unit_normal(result->normal);
        }
    }
}

// ============================================================
// EPA Edge Cases
// ============================================================

void epa_near_touching_aabb()
{
    // A: [-1,1]^3, B: [0.99, 2.99]x[-1,1]^2
    // +x face: 1-0.99=0.01. Min = 0.01m
    aabb a{.min = vec3{-1.0, -1.0, -1.0} * m, .max = vec3{1.0, 1.0, 1.0} * m};
    aabb b{.min = vec3{0.99, -1.0, -1.0} * m, .max = vec3{2.99, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.01 * m, 1e-2 * m);
    check_unit_normal(result->normal);
}

void epa_very_deep_containment()
{
    // Outer: [-100,100]^3, Inner: [-0.1,0.1]^3
    // D = [-100.1, 100.1]^3. Depth = 100.1m
    aabb outer{.min = vec3{-100.0, -100.0, -100.0} * m, .max = vec3{100.0, 100.0, 100.0} * m};
    aabb inner{.min = vec3{-0.1, -0.1, -0.1} * m, .max = vec3{0.1, 0.1, 0.1} * m};
    auto result = gjk_epa(outer, inner);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 100.1 * m, 0.5 * m);
    check_unit_normal(result->normal);
}

void epa_off_center_containment()
{
    // Outer: [-3,3]^3. Inner: [1,2]^3
    // D: +i: 3-1=2, -i: 2+3=5. Min = 2.0m
    aabb outer{.min = vec3{-3.0, -3.0, -3.0} * m, .max = vec3{3.0, 3.0, 3.0} * m};
    aabb inner{.min = vec3{1.0, 1.0, 1.0} * m, .max = vec3{2.0, 2.0, 2.0} * m};
    auto result = gjk_epa(outer, inner);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 2.0 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_flat_slab_overlap()
{
    // Two flat slabs at 0.15m apart on Y
    // D on Y: (0.1+0.1) - 0.15 = 0.05m face distance. Min = 0.05m
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{5.0, 0.1, 5.0} * m};
    obb b{vec3{0.0, 0.15, 0.0} * m, quat<one>::identity(), vec3{5.0, 0.1, 5.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.05 * m, depth_tol);
    CHECK(std::abs(result->normal.y().numerical_value_in(one)) > 0.9);
    check_unit_normal(result->normal);
}

void epa_multiple_rotation_axes()
{
    auto rot = quat<one>::from_angle_axis((std::numbers::pi / 3.0) * si::radian,
                                          vec3<one>{1.0, 2.0, 3.0}.normalized());
    obb a{vec3{0.0, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    obb b{vec3{0.5, 0.5, 0.5} * m, rot, vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK(result->depth > 0.0 * m);
    check_unit_normal(result->normal);
    check_mtv_separates(a, b, *result);
}

void epa_mixed_mesh_aabb_mtv()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto inst = msh->at(vec3{0.0, 0.0, 0.0} * m);
    aabb box{.min = vec3{0.5, -1.0, -1.0} * m, .max = vec3{2.5, 1.0, 1.0} * m};
    auto result = gjk_epa(inst, box);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

void epa_mixed_mesh_obb_mtv()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto inst = msh->at(vec3{0.0, 0.0, 0.0} * m);
    obb o{vec3{1.5, 0.0, 0.0} * m, quat<one>::identity(), vec3{1.0, 1.0, 1.0} * m};
    auto result = gjk_epa(inst, o);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.5 * m, depth_tol);
    check_unit_normal(result->normal);
}

} // namespace

int main()
{
    suite s;

    s.group("EPA Depth Accuracy (AABB)")
        .test("overlap X", epa_aabb_depth_overlap_x)
        .test("overlap Y", epa_aabb_depth_overlap_y)
        .test("overlap Z", epa_aabb_depth_overlap_z)
        .test("small overlap", epa_aabb_depth_small_overlap)
        .test("large overlap", epa_aabb_depth_large_overlap)
        .test("containment", epa_aabb_depth_containment)
        .test("identical at same position", epa_aabb_depth_identical)
        .test("asymmetric boxes", epa_aabb_depth_asymmetric)
        .test("corner overlap", epa_aabb_depth_corner_overlap);

    s.group("EPA Normal Direction (AABB)")
        .test("normal along X", epa_aabb_normal_direction_x)
        .test("normal along Y", epa_aabb_normal_direction_y)
        .test("normal along Z", epa_aabb_normal_direction_z)
        .test("normal is unit length", epa_aabb_normal_is_unit_length);

    s.group("EPA MTV Validity")
        .test("MTV separates AABB X", epa_mtv_separates_aabb_x)
        .test("MTV separates AABB diagonal", epa_mtv_separates_aabb_diagonal)
        .test("MTV separates AABB containment", epa_mtv_separates_aabb_containment)
        .test("MTV separates OBB axis-aligned", epa_mtv_separates_obb_axis_aligned)
        .test("MTV separates OBB rotated 45", epa_mtv_separates_obb_rotated)
        .test("MTV separates OBB cross", epa_mtv_separates_obb_cross)
        .test("MTV separates OBB 3D rotation", epa_mtv_separates_obb_3d_rotation);

    s.group("EPA Symmetry")
        .test("depth symmetric AABB", epa_symmetry_depth_aabb)
        .test("normal flips AABB", epa_symmetry_normal_aabb)
        .test("depth symmetric OBB", epa_symmetry_depth_obb)
        .test("normal flips OBB", epa_symmetry_normal_obb)
        .test("depth symmetric containment", epa_symmetry_depth_containment);

    s.group("EPA with OBBs")
        .test("axis-aligned depth", epa_obb_axis_aligned_depth)
        .test("identical at origin", epa_obb_identical_at_origin)
        .test("cross config depth", epa_obb_cross_config_depth)
        .test("rotated 45 depth + MTV", epa_obb_rotated_45_depth)
        .test("non-uniform slab/pillar", epa_obb_non_uniform_slab_pillar)
        .test("rotated 90 X", epa_obb_rotated_90_x);

    s.group("EPA OBB vs AABB")
        .test("axis-aligned depth", epa_obb_aabb_axis_aligned_depth)
        .test("containment depth", epa_obb_aabb_containment_depth);

    s.group("EPA mesh::instance")
        .test("box-box depth", epa_mesh_box_box_depth)
        .test("box-box containment depth", epa_mesh_box_box_containment_depth)
        .test("box-box identical depth", epa_mesh_box_box_identical_depth)
        .test("box-box normal direction", epa_mesh_box_box_normal_direction)
        .test("box-box symmetry", epa_mesh_box_box_symmetry)
        .test("box-box rotated MTV", epa_mesh_box_box_rotated_mtv)
        .test("sphere-sphere depth", epa_mesh_sphere_sphere_depth)
        .test("sphere-sphere normal", epa_mesh_sphere_sphere_normal)
        .test("sphere-sphere diagonal", epa_mesh_sphere_sphere_diagonal)
        .test("box-sphere depth", epa_mesh_box_sphere_depth);

    s.group("EPA mesh::pyramid")
        .test("overlap depth", epa_pyramid_depth_overlap)
        .test("MTV separates", epa_pyramid_mtv_separates)
        .test("pyramid-box MTV", epa_pyramid_box_mtv)
        .test("flipped depth", epa_pyramid_flipped_depth);

    s.group("EPA Invariants").test("depth always positive", epa_depth_always_positive);

    s.group("EPA Edge Cases")
        .test("near-touching AABB", epa_near_touching_aabb)
        .test("very deep containment", epa_very_deep_containment)
        .test("off-center containment", epa_off_center_containment)
        .test("flat slab overlap", epa_flat_slab_overlap)
        .test("multiple rotation axes", epa_multiple_rotation_axes)
        .test("mesh vs AABB", epa_mixed_mesh_aabb_mtv)
        .test("mesh vs OBB", epa_mixed_mesh_obb_mtv);

    return s.run();
}
