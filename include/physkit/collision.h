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
} // namespace physkit