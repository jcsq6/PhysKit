#pragma once
#include "detail/lin_alg.h"
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

    /// @brief create OBB object from point cloud via PCA
    /// @note this requires at least 4 non-coplanar points for any meaningful results
    [[nodiscard]] static obb from_points(std::span<const vec3<si::metre>> points)
    {
        assert(points.size() >= 4 && "Requires 4 points for OBB fitting");

        // compute centroid - defined as the center of our object
        vec3<si::metre> centroid = vec3<si::metre>::zero();
        for (const auto &pt : points) { centroid = centroid + pt; }
        centroid = centroid / static_cast<float_t>(points.size());

        // compute covariance matrix for a 3x3
        mat3<one> cov = mat3<one>::zero();
        for (const auto &pt : points)
        {
            vec3<one> d = (pt - centroid) / si::metre; // noramlizes into dimensionless
            cov += d * d.transpose();
        }
        cov = cov / static_cast<float_t>(points.size());

        // extract eigencvectors {principal axes} needs eigen solver
        // project points onto axes to find min/max extents
        // build obb from these resultants.

        return obb{centroid, quat<one>::identity(), vec3<si::metre>::zero()};
    }

    /// @brief Get the center of the OBB
    [[nodiscard]] constexpr auto center_pos() const { return center; }

    /// @brief return the center of the obb
    /// TODO: fix center functionfor obb
    ///[[nodiscard]] constexpr auto center() const { return (min + max) / 2.0; }

    /// @brief Get the world-space size of the obb - does not account for orientation
    [[nodiscard]] constexpr auto size() const { return half_extents * 2.0; }

    /// @brief get the half extents in local space
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

    /// @brief create an OBB that encloses another OBB with a transformation applied
    [[nodiscard]] static constexpr obb transform(const obb &box, const quat<one> &rotation,
                                                 const vec3<si::metre> &translation)
    {
        auto new_orientation = rotation * box.orientation;
        auto new_center = rotation * box.center + translation;
        return obb{new_center, new_orientation, box.half_extents};
    }

    /// @brief calculate the surface area of the obb
    [[nodiscard]] constexpr auto surface_area() const
    {
        auto s = half_extents * 2.0;
        return 2.0 * ((s.x() * s.y()) + (s.x() * s.z()) + (s.z() * s.y()));
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

    /// @brief check if point is contained in local space of the obb
    [[nodiscard]] constexpr bool contains_local(const vec3<si::metre> &local_point) const
    {
        return (local_point.x() >= -half_extents.x() && local_point.x() <= half_extents.x()) &&
               (local_point.y() >= -half_extents.y() && local_point.y() <= half_extents.y()) &&
               (local_point.z() >= -half_extents.z() && local_point.z() <= half_extents.z());
    }

    /// @ brief calculate the closeset point within the obb (local space specific)
};

} // namespace physkit
