#pragma once

#include "detail/types.h"
#include "mesh.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <array>

namespace physkit
{
class collision_info // TODO: time of impact for continuous collision detection
{
public:
    static constexpr std::size_t max_contact_points = 8;

    collision_info(const vec3<one> &normal, std::span<const vec3<si::metre>> contacts,
                   quantity<si::metre> depth)
        : M_normal{normal}, M_depth{depth}, M_contacts{contacts.size()}
    {
        assert(contacts.size() <= max_contact_points);
        std::ranges::copy(contacts, M_contact_buffer.begin());
    }

    [[nodiscard]] const auto &normal() const { return M_normal; }
    [[nodiscard]] auto depth() const { return M_depth; }
    [[nodiscard]] auto contacts() const { return std::span{M_contact_buffer.data(), M_contacts}; }
    [[nodiscard]] auto mtv() const { return M_normal * M_depth; }

    void contacts(std::span<const vec3<si::metre>> contacts)
    {
        assert(contacts.size() <= max_contact_points);
        M_contacts = contacts.size();
        std::ranges::copy(contacts, M_contact_buffer.begin());
    }
    void normal(const vec3<one> &normal) { M_normal = normal; }
    void depth(quantity<si::metre> depth) { M_depth = depth; }

private:
    vec3<one> M_normal{0, 0, 0};
    std::array<vec3<si::metre>, 8> M_contact_buffer{};
    std::size_t M_contacts{};
    quantity<si::metre> M_depth{};
};

using collision_result = std::optional<collision_info>;

collision_result gjk_epa(const mesh::instance &a, const mesh::instance &b);
collision_result sat(const mesh::instance &a, const mesh::instance &b);

// Implement GJK implementation into here from previous headers

/// @brief result of GJK intersection test
struct gjk_result
{
    bool intersects = false;
    std::optional<std::pair<vec3<si::metre>, vec3<si::metre>>>
    closeest_points();              // define cloeset point on each shape if not intersecting
    quantity<si::metre> distance(); // minimum distance between shapes
};

/// @brief add in GJK algorithm tests for obb and abb type intersections
/// obb - obb | aabb - aabb | aabb - obb |
[[nodiscard]] gjk_result gjk_obb_obb(const obb &a, const obb &b);

[[nodiscard]] gjk_result gjk_obb_aabb(const obb &obb_obj, const aabb &aabb_obj);

[[nodiscard]] gjk_result gjk_aabb_obb(const aabb &aabb_obj, const obb &obb_obj);

/// @brief GJK algorithm implementation for AABB-AABB intersection/distance
[[nodiscard]] gjk_result gjk_aabb_aabb(const aabb &a, const aabb &b);

/// TODO -> implement generic test for any obb abbb combination: future goal idk if we need
/// immediately.
template <typename ShapeA, typename ShapeB>
[[nodiscard]] gjk_result gjk_test(const ShapeA &shape_a, const ShapeB &shape_b);

/// @brief Support function for obb & aabb - finds furthest point in given direction
[[nodiscard]] vec3<si::metre> support_obb(const obb &obj, const vec3<one> &direction);

[[nodiscard]] vec3<si::metre> support_aabb(const aabb &obj, const vec3<one> &direction);

/// @brief Compute Minkowski difference of shapes A and B
template <typename ShapeA, typename ShapeB>
[[nodiscard]] vec3<si::metre> minkowski_difference_support(const ShapeA &shape_a,
                                                           const ShapeB &shape_b,
                                                           const vec3<one> &direction);

} // namespace physkit
