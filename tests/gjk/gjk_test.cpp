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

// ============================================================
// mesh::instance vs mesh::instance
// ============================================================

void mesh_instance_box_box_overlap()
{
    // Two unit boxes (half-extents 1m), centers 1.5m apart on X → 0.5m overlap
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.5, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
}

void mesh_instance_box_box_separated()
{
    // Two unit boxes, centers 3m apart on X → 1m gap
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{3.0, 0.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
}

void mesh_instance_box_box_containment()
{
    // Small box fully inside larger box
    auto outer = mesh::box(vec3{3.0, 3.0, 3.0} * m);
    auto inner = mesh::box(vec3{0.5, 0.5, 0.5} * m);
    auto a = outer->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = inner->at(vec3{0.0, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
    CHECK(gjk_epa(b, a).has_value());
}

void mesh_instance_box_box_same_instance()
{
    // Same instance at same position: Minkowski difference contains origin
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, a).has_value());
}

void mesh_instance_box_box_rotated_overlap()
{
    // B rotated 45° around Z at (1,0,0): support in -x ≈ 1 - sqrt(2) ≈ -0.414 < 1 → overlap
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.0, 0.0, 0.0} * m, rot);
    CHECK(gjk_epa(a, b).has_value());
}

void mesh_instance_box_box_rotated_separated()
{
    // B rotated 45° around Z at (3,0,0): support in -x ≈ 3 - sqrt(2) ≈ 1.586 > 1 → gap
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 4.0) * si::radian, vec3<one>{0.0, 0.0, 1.0});
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{3.0, 0.0, 0.0} * m, rot);
    CHECK(!gjk_epa(a, b).has_value());
}

void mesh_instance_sphere_sphere_overlap()
{
    // Two unit spheres, centers 1.5m apart → 0.5m overlap (well within discretization error)
    auto sph = mesh::sphere(1.0 * m);
    auto a = sph->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{1.5, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
}

void mesh_instance_sphere_sphere_separated()
{
    // Two unit spheres, centers 3m apart → 1m gap
    auto sph = mesh::sphere(1.0 * m);
    auto a = sph->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{3.0, 0.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
}

void mesh_instance_box_sphere_overlap()
{
    // Unit box at origin, unit sphere center at (1.5,0,0): sphere reaches x=0.5 → 0.5m overlap
    auto box_msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto sph_msh = mesh::sphere(1.0 * m);
    auto a = box_msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph_msh->at(vec3{1.5, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
    CHECK(gjk_epa(b, a).has_value());
}

void mesh_instance_box_sphere_separated()
{
    // Unit box at origin, unit sphere center at (3,0,0): sphere reaches x=2 → 1m gap
    auto box_msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto sph_msh = mesh::sphere(1.0 * m);
    auto a = box_msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph_msh->at(vec3{3.0, 0.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
    CHECK(!gjk_epa(b, a).has_value());
}

void mesh_instance_symmetry_colliding()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.5, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value() == gjk_epa(b, a).has_value());
}

void mesh_instance_symmetry_separated()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{4.0, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value() == gjk_epa(b, a).has_value());
}

void mesh_instance_collision_info_stub_fields()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{1.0, 0.0, 0.0} * m);
    auto result = gjk_epa(a, b);
    CHECK(result.has_value());
    CHECK_APPROX(result->depth, 0.0 * m);
    CHECK_APPROX(result->local_a, vec3<si::metre>::zero());
    CHECK_APPROX(result->local_b, vec3<si::metre>::zero());
    CHECK_APPROX(result->normal, vec3<one>::zero());
}

void mesh_instance_collision_info_nullopt_when_separated()
{
    auto msh = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = msh->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = msh->at(vec3{4.0, 0.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
}

// ============================================================
// mesh::pyramid tests
// pyramid(base_half, height): base corners at (±base_half, 0, ±base_half), apex at (0, height, 0)
// ============================================================

void pyramid_pyramid_same_pos()
{
    // Minkowski difference of a shape with itself contains the origin
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, a).has_value());
}

void pyramid_pyramid_overlap_y()
{
    // A: base y=0, apex y=2. B shifted 1.5m up: base y=1.5, apex y=3.5 → 0.5m overlap in y
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 1.5, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
}

void pyramid_pyramid_separated_y()
{
    // A: base y=0, apex y=2. B at y=4: base y=4, apex y=6 → 2m gap
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 4.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
}

void pyramid_pyramid_separated_x()
{
    // A: base x∈[-1,1]. B at x=4: base x∈[3,5] → 2m gap in x
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{4.0, 0.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
}

void pyramid_pyramid_flipped_overlap()
{
    // B rotated 180° around Z: apex maps to y=-2 in local space.
    // Placed at (0,3,0): world apex at y=1, world base at y=3.
    // A has apex at y=2 → overlap in y∈[1,2]
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto rot = quat<one>::from_angle_axis(std::numbers::pi * si::radian, vec3<one>{0.0, 0.0, 1.0});
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 3.0, 0.0} * m, rot);
    CHECK(gjk_epa(a, b).has_value());
}

void pyramid_pyramid_flipped_separated()
{
    // Same flip but B at (0,6,0): world apex at y=4, A apex at y=2 → 2m gap
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto rot = quat<one>::from_angle_axis(std::numbers::pi * si::radian, vec3<one>{0.0, 0.0, 1.0});
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = pyr->at(vec3{0.0, 6.0, 0.0} * m, rot);
    CHECK(!gjk_epa(a, b).has_value());
}

void pyramid_box_overlap()
{
    // Pyramid y∈[0,2], unit box centered at origin y∈[-1,1] → shared region y∈[0,1]
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto box = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = box->at(vec3{0.0, 0.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
    CHECK(gjk_epa(b, a).has_value());
}

void pyramid_box_separated()
{
    // Pyramid base at y=0, box centered at y=-3 (y∈[-4,-2]) → 2m gap below the base
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto box = mesh::box(vec3{1.0, 1.0, 1.0} * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = box->at(vec3{0.0, -3.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
    CHECK(!gjk_epa(b, a).has_value());
}

void pyramid_sphere_overlap()
{
    // Pyramid y∈[0,2], unit sphere centered at (0,1,0) y∈[0,2] → overlap
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto sph = mesh::sphere(1.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{0.0, 1.0, 0.0} * m);
    CHECK(gjk_epa(a, b).has_value());
    CHECK(gjk_epa(b, a).has_value());
}

void pyramid_sphere_separated()
{
    // Pyramid apex at y=2, unit sphere centered at y=5 (y∈[4,6]) → 2m gap
    auto pyr = mesh::pyramid(1.0 * m, 2.0 * m);
    auto sph = mesh::sphere(1.0 * m);
    auto a = pyr->at(vec3{0.0, 0.0, 0.0} * m);
    auto b = sph->at(vec3{0.0, 5.0, 0.0} * m);
    CHECK(!gjk_epa(a, b).has_value());
    CHECK(!gjk_epa(b, a).has_value());
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

    s.group("mesh::instance")
        .test("box-box overlap", mesh_instance_box_box_overlap)
        .test("box-box separated", mesh_instance_box_box_separated)
        .test("box-box containment", mesh_instance_box_box_containment)
        .test("box-box same instance", mesh_instance_box_box_same_instance)
        .test("box-box rotated overlap", mesh_instance_box_box_rotated_overlap)
        .test("box-box rotated separated", mesh_instance_box_box_rotated_separated)
        .test("sphere-sphere overlap", mesh_instance_sphere_sphere_overlap)
        .test("sphere-sphere separated", mesh_instance_sphere_sphere_separated)
        .test("box-sphere overlap", mesh_instance_box_sphere_overlap)
        .test("box-sphere separated", mesh_instance_box_sphere_separated)
        .test("symmetry colliding", mesh_instance_symmetry_colliding)
        .test("symmetry separated", mesh_instance_symmetry_separated)
        .test("stub fields are zero", mesh_instance_collision_info_stub_fields)
        .test("nullopt when separated", mesh_instance_collision_info_nullopt_when_separated);

    s.group("mesh::pyramid")
        .test("same pos", pyramid_pyramid_same_pos)
        .test("pyramid-pyramid overlap Y", pyramid_pyramid_overlap_y)
        .test("pyramid-pyramid separated Y", pyramid_pyramid_separated_y)
        .test("pyramid-pyramid separated X", pyramid_pyramid_separated_x)
        .test("pyramid-pyramid flipped overlap", pyramid_pyramid_flipped_overlap)
        .test("pyramid-pyramid flipped separated", pyramid_pyramid_flipped_separated)
        .test("pyramid-box overlap", pyramid_box_overlap)
        .test("pyramid-box separated", pyramid_box_separated)
        .test("pyramid-sphere overlap", pyramid_sphere_overlap)
        .test("pyramid-sphere separated", pyramid_sphere_separated);

    return s.run();
}
