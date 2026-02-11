#pragma once
#include "detail/types.h"
#include "mesh.h" // for aabb

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <numbers>
#include <span>
#include <vector>

namespace physkit
{
/// @brief An oriented bounding box defined by center, orientation, and half-extents. -> really just
/// from the real time collision book and AABB
class obb
{
public:
    vec3<si::metre> center;
    quat<one> orientation;        // rotation quaternion representing the orientation
    vec3<si::metre> half_extents; // half-size along each local axis

    /// @brief Default constructor creates a zero-sized box at origin with no rotation
    constexpr obb()
        : center(vec3<si::metre>::zero()), orientation(quat<one>::identity()),
          half_extents(vec3<si::metre>::zero())
    {
    }

    /// @brief Constructor with center, orientation, and half-extents
    constexpr obb(const vec3<si::metre> &c, const quat<one> &o, const vec3<si::metre> &he)
        : center(c), orientation(o), half_extents(he)
    {
    }

    /// @brief Create an OBB from an AABB by applying a transformation
    static obb from_aabb(const aabb &box, const vec3<si::metre> &position = vec3<si::metre>::zero(),
                         const quat<one> &orientation = quat<one>::identity())
    {
        auto box_center = box.center();
        auto transformed_center = orientation * box_center + position;

        return obb{transformed_center, orientation, box.extent()};
    }

    /// @brief this is unresolved. I can't figure out the proper size which accounts
    /// for current orientation.

    /// this is not working.
    [[nodiscard]] constexpr auto size() const {}
    /// @brief Get the center of the OBB
    [[nodiscard]] constexpr auto center_pos() const { return center; }
    // [[nodiscard]] constexpr auto center() const { return (min + max) / 2.0; }

    /// @brief Get the half-extents of the OBB
    [[nodiscard]] constexpr auto extents() const { return half_extents; }

    /// @brief Calculate the volume of the OBB
    [[nodiscard]] constexpr auto volume() const
    {
        auto s = half_extents * 2.0;
        return s.x() * s.y() * s.z();
    }

    /// @brief Get a corner point of the OBB by index (0-7)
    [[nodiscard]] vec3<si::metre> point(unsigned int index) const
    {
        assert(index < 8);

        // Generate local corner in the OBB's coordinate system
        vec3<si::metre> local_corner{((index & 1) != 0u) ? half_extents.x() : -half_extents.x(),
                                     ((index & 2) != 0u) ? half_extents.y() : -half_extents.y(),
                                     ((index & 4) != 0u) ? half_extents.z() : -half_extents.z()};

        // Transform to world coordinates
        auto world_corner = orientation * local_corner;
        return center + world_corner;
    }

    /// @brief Get all 8 corner points of the OBB
    [[nodiscard]] std::array<vec3<si::metre>, 8> corners() const
    {
        std::array<vec3<si::metre>, 8> pts;
        for (unsigned int i = 0; i < 8; ++i) { pts[i] = point(i); }
        return pts;
    }

    /// @brief Get the 3 axis vectors of the OBB in world space
    [[nodiscard]] std::array<vec3<one>, 3> axes() const
    {
        // The local axes are (1,0,0), (0,1,0), (0,0,1)
        // Rotate them by the orientation to get world axes
        vec3<one> local_x{1.0, 0.0, 0.0};
        vec3<one> local_y{0.0, 1.0, 0.0};
        vec3<one> local_z{0.0, 0.0, 1.0};

        return {orientation * local_x, orientation * local_y, orientation * local_z};
    }

    /// TODO: implement translations

    /// @brief Compound assignment operators - modified from aabb
    constexpr auto &operator+=(const vec3<si::metre> &offset)
    {
        center += offset;
        return *this;
    }

    constexpr auto &operator-=(const vec3<si::metre> &offset)
    {
        center -= offset;
        return *this;
    }

    constexpr auto &operator*=(float_t scale)
    {
        half_extents *= scale;
        return *this;
    }

    constexpr auto &operator*=(QuantityOf<dimensionless> auto scale)
    {
        half_extents *= scale;
        return *this;
    }

    /// @brief Check if a point is contained within the OBB
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        // Transform the point to the OBB's local coordinate system
        auto local_point = orientation.conjugate() * (point - center);

        // Check if the local point is within the half-extents
        return (local_point.x() >= -half_extents.x() && local_point.x() <= half_extents.x()) &&
               (local_point.y() >= -half_extents.y() && local_point.y() <= half_extents.y()) &&
               (local_point.z() >= -half_extents.z() && local_point.z() <= half_extents.z());
    }
};

} // namespace physkit