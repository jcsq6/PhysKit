#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "../detail/macros.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <cassert>
#include <memory>
#include <span>
#include <vector>
#endif

#include "../algebra/types.h"
#include "bounds.h"
#include "bvh.h"
#include "mesh.h"

PHYSKIT_EXPORT
namespace physkit
{

class instance;
class box
{
public:
    box(const box &) = default;
    box &operator=(const box &) = default;
    box(box &&) = default;
    box &operator=(box &&) = default;
    ~box() = default;

    box(const vec3<si::metre> &half_extents) : M_half_extents(half_extents)
    {
        M_aabb = aabb::from_points({half_extents, -half_extents});

        M_bsphere = bounding_sphere({0.0 * si::metre, 0.0 * si::metre, 0.0 * si::metre},
                                    half_extents.norm());
    }

    [[nodiscard]] const vec3<si::metre> &half_extents() const { return M_half_extents; }
    [[nodiscard]] const aabb &bounds() const { return M_aabb; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    { return 8 * M_half_extents.x() * M_half_extents.y() * M_half_extents.z(); }

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        using namespace mp_units::si::unit_symbols;
        return {0.0 * m, 0.0 * m, 0.0 * m};
    }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        using namespace mp_units::si::unit_symbols;
        auto mass = density * volume();
        auto x = M_half_extents.x() * 2;
        auto y = M_half_extents.y() * 2;
        auto z = M_half_extents.z() * 2;
        auto i1 = ((mass / 12) * (mp_units::pow<2>(y) + mp_units::pow<2>(z)))
                      .numerical_value_in(kg * m * m);
        auto i2 = ((mass / 12) * (mp_units::pow<2>(x) + mp_units::pow<2>(z)))
                      .numerical_value_in(kg * m * m);
        auto i3 = ((mass / 12) * (mp_units::pow<2>(x) + mp_units::pow<2>(y)))
                      .numerical_value_in(kg * m * m);

        return vec3{i1, i2, i3}.as_diagonal() * kg * m * m;
    }

    /// @brief Ray intersection in local (model) space. O(log N) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the box surface in local space. O(N) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space. O(N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        return (mp_units::abs(point.x()) <= M_half_extents.x() &&
                mp_units::abs(point.y()) <= M_half_extents.y() &&
                mp_units::abs(point.z()) <= M_half_extents.z());
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        return {direction.x() >= 0.0 ? M_half_extents.x() : -M_half_extents.x(),
                direction.y() >= 0.0 ? M_half_extents.y() : -M_half_extents.y(),
                direction.z() >= 0.0 ? M_half_extents.z() : -M_half_extents.z()};
    }

    [[nodiscard]] instance at(const vec3<si::metre> &pos,
                              const quat<one> &orientation = quat<one>::identity()) const;

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

    bool operator==(const box &other) const { return M_half_extents == other.M_half_extents; }

private:
    vec3<si::metre> M_half_extents;
    aabb M_aabb;
    bounding_sphere M_bsphere;
};

class sphere
{
public:
    /// @brief A view of a sphere placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.

    sphere(const sphere &) = default;
    sphere &operator=(const sphere &) = default;
    sphere(sphere &&) = default;
    sphere &operator=(sphere &&) = default;
    ~sphere() = default;

    sphere(const quantity<si::metre> radius) : M_radius{radius}
    {
        M_aabb = aabb::from_points({vec3<si::metre>{M_radius, M_radius, M_radius},
                                    vec3<si::metre>{-M_radius, -M_radius, -M_radius}});

        M_bsphere = bounding_sphere(vec3<si::metre>::zero(), M_radius);
    }

    [[nodiscard]] const quantity<si::metre> &radius() const { return M_radius; }
    [[nodiscard]] const aabb &bounds() const { return M_aabb; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    { return (4.0 / 3.0) * std::numbers::pi * mp_units::pow<3>(M_radius); }
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        using namespace mp_units::si::unit_symbols;
        return vec3<si::metre>::zero();
    }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        using namespace mp_units::si::unit_symbols;
        auto val = ((2.0 / 5.0) * density * volume() * (mp_units::pow<2>(M_radius)));

        return vec3{val, val, val}.as_diagonal();
    }

    /// @brief Ray intersection in local (model) space. O(log N) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        using namespace mp_units::si::unit_symbols;
        std::optional<ray::hit> best;

        // should be able to manually fill in the ray hit.
        // point of impact
        // normal of surface point
        // distance along the ray
        auto origin = r.origin();
        auto direction = r.direction();

        auto a = direction.dot(direction);
        auto b = 2 * (origin.dot(direction));
        auto c = origin.dot(origin) - mp_units::pow<2>(M_radius);

        auto det = mp_units::pow<2>(b) - 4 * a * c;
        if (det < 0 * m * m) return std::nullopt;

        auto sqrt_det = mp_units::sqrt(det);
        auto dist = (-b - sqrt_det) / (2 * a);
        if (dist < 0.0 * m) dist = (-b + sqrt_det) / (2 * a);
        if (dist < 0.0 * m || dist > max_distance) return std::nullopt;

        auto pos = origin + dist * direction;
        auto normal = pos.normalized();

        return ray::hit{.pos = pos, .normal = normal, .distance = dist};
    }
    /// @brief Closest point on the sphere surface in local space. O(1) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    { return point.normalized() * M_radius; }
    /// @brief Point containment test in local space. O(1) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        // return (point - M_position).norm() <= M_radius;
        return point.norm() <= M_radius;
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    { return direction.normalized() * M_radius; }

    [[nodiscard]] instance at(const vec3<si::metre> &pos,
                              const quat<one> &orientation = quat<one>::identity()) const;

    /// TODO: Add in support to return obb objects -> much more tedious, more research later.

    bool operator==(const sphere &other) const { return M_radius == other.M_radius; }

private:
    aabb M_aabb;
    bounding_sphere M_bsphere;
    quantity<si::metre> M_radius;
};

class cylinder // center at origin, height of height, radius of radius
{
public:
    /// @brief A view of a cylinder placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.

    cylinder(const cylinder &) = default;
    cylinder &operator=(const cylinder &) = default;
    cylinder(cylinder &&) = default;
    cylinder &operator=(cylinder &&) = default;
    ~cylinder() = default;

    cylinder(const quantity<si::metre> radius, const quantity<si::metre> height)
        : M_radius{radius}, M_height{height}
    {
        M_aabb = aabb::from_points({vec3<si::metre>{M_radius, M_height / 2.0f, M_radius},
                                    vec3<si::metre>{-M_radius, -M_height / 2.0f, -M_radius}});

        M_bsphere = bounding_sphere(vec3<si::metre>{0 * si::metre, 0 * si::metre, 0 * si::metre},
                                    sqrt(pow<2>(M_radius) + pow<2>(M_height / 2)));
    }

    [[nodiscard]] const quantity<si::metre> &radius() const { return M_radius; }
    [[nodiscard]] const quantity<si::metre> &height() const { return M_height; }
    [[nodiscard]] const aabb &bounds() const { return M_aabb; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    { return (M_height * pow<2>(M_radius)) * std::numbers::pi; } // pi*r^2*h
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] vec3<si::metre> mass_center() const { return vec3<si::metre>::zero(); }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        // rotation about y is m*r^2/2
        // rotation about other axis is m*(3.0*r^2 + h^2)/12
        using namespace mp_units::si::unit_symbols;
        auto iy = 0.5 * density * volume() * pow<2>(M_radius);
        auto ixz = density * volume() * (3.0 * pow<2>(M_radius) + pow<2>(M_height)) / 12.0;

        return vec3{ixz, iy, ixz}.as_diagonal();
    }

    /// @brief Ray intersection in local (model) space. O(1) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the cylinder surface in local space. O(1) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space. O(1) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        auto [x, y, z] = point;
        return (abs(y) <= M_height * 0.5) && (x * x + z * z <= M_radius * M_radius);
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &dir) const
    {

        const auto half_h = M_height / 2;

        // top or bottom cap
        const auto y = (dir.y() >= 0.0) ? half_h : -half_h; // m

        // project to xz
        const auto dxz = sqrt(dir.x() * dir.x() + dir.z() * dir.z()); // one

        if (dxz == 0.0) // direction is vertical
        {
            return vec3{0.0 * si::metre, y, 0.0 * si::metre};
        }

        // support point on rim.
        const auto sx = M_radius * dir.x() / dxz; // m
        const auto sz = M_radius * dir.z() / dxz; // m

        return vec3{sx, y, sz};
    }

    [[nodiscard]] instance at(const vec3<si::metre> &pos,
                              const quat<one> &orientation = quat<one>::identity()) const;

    /// TODO: Add in support to return obb objects -> much more tedious, more research later.

    bool operator==(const cylinder &other) const
    { return M_radius == other.M_radius && M_height == other.M_height; }

private:
    aabb M_aabb;
    bounding_sphere M_bsphere;
    quantity<si::metre> M_radius;
    quantity<si::metre> M_height;
};

class cone // base at origin, height of height, radius of radius
{
public:
    /// @brief A view of a cone placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.

    cone(const cone &) = default;
    cone &operator=(const cone &) = default;
    cone(cone &&) = default;
    cone &operator=(cone &&) = default;
    ~cone() = default;

    cone(const quantity<si::metre> radius, const quantity<si::metre> height)
        : M_radius{radius}, M_height{height}
    {
        M_aabb = aabb::from_points({vec3<si::metre>{M_radius, M_height, M_radius},
                                    vec3<si::metre>{-M_radius, 0.0 * si::metre, -M_radius}});

        M_bsphere = bounding_sphere(vec3<si::metre>{0 * si::metre, M_height / 2, 0 * si::metre},
                                    sqrt(pow<2>(M_radius) + pow<2>(M_height / 2)));
    }

    [[nodiscard]] const quantity<si::metre> &radius() const { return M_radius; }
    [[nodiscard]] const quantity<si::metre> &height() const { return M_height; }
    [[nodiscard]] const aabb &bounds() const { return M_aabb; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    { return M_height * pow<2>(M_radius) * std::numbers::pi / 3; }
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] vec3<si::metre> mass_center() const
    { return vec3<si::metre>{0 * si::metre, M_height / 4, 0 * si::metre}; }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        // rotation about y is 0.3*m*r^2
        // rotation about other axis is m*((3/20)*r^2 + 0.1*h^2)
        using namespace mp_units::si::unit_symbols;
        auto iy = 0.3 * density * volume() * pow<2>(M_radius);
        auto ixz = density * volume() * ((3.0 / 20.0) * pow<2>(M_radius) + 0.1 * pow<2>(M_height));

        return vec3{ixz, iy, ixz}.as_diagonal();
    }

    /// @brief Ray intersection in local (model) space. O(1) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the cone surface in local space. O(1) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space. O(1) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        using namespace mp_units;
        // calculate point distance from y axis
        // calculate radius at that point
        if (point.y() < 0.0 * si::metre || point.y() > M_height) return false;
        return pow<2>(point.x()) + pow<2>(point.z()) <=
               pow<2>(M_radius * (1.0 - point.y() / M_height));
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &dir) const
    {
        using namespace mp_units::si::unit_symbols;

        auto dxz = sqrt(dir.x() * dir.x() + dir.z() * dir.z());

        // tip*d = h*dy, base-rim*d = r*dxz; pick whichever is larger
        if (M_height * dir.y() >= M_radius * dxz) return vec3{0.0 * m, M_height, 0.0 * m};

        if (dxz > 0.0) return vec3{M_radius * dir.x() / dxz, 0.0 * m, M_radius * dir.z() / dxz};

        // Pure downward direction with zero horizontal component
        return vec3{M_radius, 0.0 * m, 0.0 * m};
    }

    [[nodiscard]] instance at(const vec3<si::metre> &pos,
                              const quat<one> &orientation = quat<one>::identity()) const;

    /// TODO: Add in support to return obb objects -> much more tedious, more research later.

    bool operator==(const cone &other) const
    { return M_radius == other.M_radius && M_height == other.M_height; }

private:
    aabb M_aabb;
    bounding_sphere M_bsphere;
    quantity<si::metre> M_radius;
    quantity<si::metre> M_height;
};

// pyramid points positioned at +-base_half, 0, +-base_half
class pyramid
{
public:
    pyramid(const pyramid &) = default;
    pyramid &operator=(const pyramid &) = default;
    pyramid(pyramid &&) = default;
    pyramid &operator=(pyramid &&) = default;
    ~pyramid() = default;

    pyramid(const quantity<si::metre> &base_half, const quantity<si::metre> &height)
        : M_base_half{base_half}, M_height{height}
    {
        M_aabb = aabb::from_points(
            {vec3{base_half, height, base_half}, vec3{-base_half, 0.0 * si::metre, -base_half}});

        M_bsphere = bounding_sphere(vec3<si::metre>{0 * si::metre, 0.5 * height, 0 * si::metre},
                                    sqrt(2 * pow<2>(base_half) + pow<2>(height / 2)));
    }

    [[nodiscard]] const quantity<si::metre> &base_half() const { return M_base_half; }
    [[nodiscard]] const quantity<si::metre> &height() const { return M_height; }
    [[nodiscard]] const aabb &bounds() const { return M_aabb; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    { return (1.0 / 3.0) * pow<2>(M_base_half * 2.0) * M_height; }

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        using namespace mp_units::si::unit_symbols;
        return {0.0 * m, 0.25 * M_height, 0.0 * m};
    }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        // Solid square pyramid (base side a = 2b, height h) about the base centre.
        //   I_y  = (1/10) M a^2  =  (2/5) M b^2  =  (8/15) \rho b^4 h
        //   I_xz = (1/10) M (a^2 + 4h^2/3)\times... derived directly as
        //          \rho (8 b^4 h + 4 b^2 h^3) / 30
        using namespace mp_units::si::unit_symbols;
        auto ixz =
            density *
            (8.0 * pow<4>(M_base_half) * M_height + 4.0 * pow<2>(M_base_half) * pow<3>(M_height)) /
            30.0;
        auto iy = (8.0 / 15.0) * density * pow<4>(M_base_half) * M_height;

        return vec3{ixz, iy, ixz}.as_diagonal();
    }

    /// @brief Ray intersection in local (model) space.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the box surface in local space.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        if (point.y() > M_height || point.y() < 0.0 * si::metre) return false;
        auto lim = M_base_half * (1 - point.y() / M_height);
        return (point.x() <= lim && point.x() >= -lim && point.z() <= lim && point.z() >= -lim);
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        using namespace mp_units::si::unit_symbols;
        quantity<m> x = (vec3<m>{M_base_half, 0 * m, 0 * m}).dot(direction) < 0.0 * m ? -M_base_half
                                                                                      : M_base_half;
        quantity<m> z = (vec3<m>{0 * m, 0 * m, M_base_half}).dot(direction) < 0.0 * m ? -M_base_half
                                                                                      : M_base_half;
        vec3<m> a = vec3<m>{0 * m, M_height, 0 * m};
        vec3<m> b = vec3<m>{x, 0 * m, z};
        vec3<m> ret = (a.dot(direction) > b.dot(direction)) ? a : b;
        return ret;
    }

    [[nodiscard]] instance at(const vec3<si::metre> &pos,
                              const quat<one> &orientation = quat<one>::identity()) const;

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

    bool operator==(const pyramid &other) const
    { return M_base_half == other.M_base_half && M_height == other.M_height; }

private:
    quantity<si::metre> M_base_half;
    quantity<si::metre> M_height;
    aabb M_aabb;
    bounding_sphere M_bsphere;
};

class shape
{
public:
    enum class type : std::uint8_t
    {
        sphere = 0,
        box,
        cylinder,
        cone,
        pyramid,
        // convex_hull,
        mesh
    };

    shape(const shape &other) : M_type(other.M_type) { copy_from(other); }

    shape &operator=(const shape &other)
    {
        if (this != &other)
        {
            destroy_active();
            M_type = other.M_type;
            copy_from(other);
        }
        return *this;
    }

    shape(shape &&other) noexcept : M_type(other.M_type) { move_from(std::move(other)); }

    shape &operator=(shape &&other) noexcept
    {
        if (this != &other)
        {
            destroy_active();
            M_type = other.M_type;
            move_from(std::move(other));
        }
        return *this;
    }
    ~shape() { destroy_active(); }

    shape() : M_type(type::box) { construct_box(vec3{0.5f, 0.5f, 0.5f} * si::metre); }

    shape(std::shared_ptr<const physkit::mesh> m) : M_type(type::mesh)
    { construct_mesh(std::move(m)); }

    shape(const physkit::sphere &s) : M_type(type::sphere) { construct_sphere(s); }

    shape(const physkit::box &b) : M_type(type::box) { construct_box(b); }

    shape(const physkit::cylinder &cy) : M_type(type::cylinder) { construct_cylinder(cy); }

    shape(const physkit::cone &c) : M_type(type::cone) { construct_cone(c); }

    shape(const physkit::pyramid &p) : M_type(type::pyramid) { construct_pyramid(p); }

    [[nodiscard]] const std::shared_ptr<const physkit::mesh> &mesh() const
    {
        assert(M_type == type::mesh);
        return M_storage.msh;
    }
    [[nodiscard]] const physkit::sphere &sphere() const
    {
        assert(M_type == type::sphere);
        return M_storage.sph;
    }
    [[nodiscard]] const physkit::box &box() const
    {
        assert(M_type == type::box);
        return M_storage.bx;
    }
    [[nodiscard]] const physkit::cylinder &cylinder() const
    {
        assert(M_type == type::cylinder);
        return M_storage.cyl;
    }
    [[nodiscard]] const physkit::cone &cone() const
    {
        assert(M_type == type::cone);
        return M_storage.cn;
    }
    [[nodiscard]] const physkit::pyramid &pyramid() const
    {
        assert(M_type == type::pyramid);
        return M_storage.pyr;
    }

    shape &operator=(std::shared_ptr<const physkit::mesh> m)
    {
        destroy_active();
        M_type = type::mesh;
        construct_mesh(std::move(m));
        return *this;
    }

private:
    /// @brief Dispatch a callable over the active primitive, dereferencing the mesh pointer
    /// so the visitor always receives a reference to the underlying shape.
    template <typename F> decltype(auto) visit(F &&f) const
    {
        switch (M_type)
        {
        case type::sphere:
            return std::forward<F>(f)(M_storage.sph);
        case type::box:
            return std::forward<F>(f)(M_storage.bx);
        case type::mesh:
            return std::forward<F>(f)(*M_storage.msh);
        case type::cylinder:
            return std::forward<F>(f)(M_storage.cyl);
        case type::cone:
            return std::forward<F>(f)(M_storage.cn);
        case type::pyramid:
            return std::forward<F>(f)(M_storage.pyr);
        default:
            std::unreachable();
        }
    }

public:
    [[nodiscard]] const aabb &bounds() const
    {
        return visit([](const auto &s) -> const aabb & { return s.bounds(); });
    }

    [[nodiscard]] const bounding_sphere &bsphere() const
    {
        return visit([](const auto &s) -> const bounding_sphere & { return s.bsphere(); });
    }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    {
        return visit([](const auto &s) { return s.volume(); });
    }

    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        return visit([](const auto &s) { return s.mass_center(); });
    }

    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        return visit([&](const auto &s) { return s.inertia_tensor(density); });
    }

    /// @brief Ray intersection
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        return visit([&](const auto &s) { return s.ray_intersect(r, max_distance); });
    }

    /// @brief Closest point on the surface
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        return visit([&](const auto &s) { return s.closest_point(point); });
    }

    /// @brief Point containment test in local space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        return visit([&](const auto &s) { return s.contains(point); });
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given sphere.
    [[nodiscard]] std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        assert(M_type == type::mesh);
        return M_storage.msh->overlap_sphere(sphere);
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        return visit([&](const auto &s) { return s.support(direction); });
    }

    [[nodiscard]] bool is_convex() const
    {
        switch (M_type)
        {
        default:
        case type::mesh:
            return M_storage.msh->is_convex();
        case type::sphere:
        case type::box:
        case type::cylinder:
        case type::cone:
        case type::pyramid:
            return true;
        }
    }

    [[nodiscard]] type type() const { return M_type; };

    // mesh only methods for compatibility
    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const
    {
        assert(M_type == type::mesh);
        return M_storage.msh->vertices();
    }
    [[nodiscard]] std::span<const triangle_t> triangles() const
    {
        assert(M_type == type::mesh);
        return M_storage.msh->triangles();
    }
    [[nodiscard]] const vec3<si::metre> &vertex(unsigned int index) const
    {
        assert(M_type == type::mesh);
        const auto &msh = M_storage.msh;
        assert(index < msh->vertices().size());
        return msh->vertices()[index];
    }

    [[nodiscard]] instance at(const vec3<si::metre> &pos,
                              const quat<one> &orientation = quat<one>::identity()) const;

    bool operator==(const shape &other) const
    {
        if (M_type != other.M_type) return false;
        switch (M_type)
        {
        case type::mesh:
            return M_storage.msh == other.M_storage.msh;
        case type::sphere:
            return M_storage.sph == other.M_storage.sph;
        case type::box:
            return M_storage.bx == other.M_storage.bx;
        case type::cylinder:
            return M_storage.cyl == other.M_storage.cyl;
        case type::cone:
            return M_storage.cn == other.M_storage.cn;
        case type::pyramid:
            return M_storage.pyr == other.M_storage.pyr;
        default:
            std::unreachable();
        }
    }

private:
    union storage_t
    {
        std::shared_ptr<const physkit::mesh> msh;
        physkit::sphere sph;
        physkit::box bx;
        physkit::cylinder cyl;
        physkit::cone cn;
        physkit::pyramid pyr;
        storage_t() {}
        storage_t(const storage_t &other) = delete;
        storage_t &operator=(const storage_t &other) = delete;
        storage_t(storage_t &&other) = delete;
        storage_t &operator=(storage_t &&other) = delete;
        ~storage_t() {}
    } M_storage;

    enum type M_type;

    void destroy_active()
    {
        switch (M_type)
        {
        case type::mesh:
            M_storage.msh.~shared_ptr();
            break;
        case type::sphere:
            M_storage.sph.~sphere();
            break;
        case type::box:
            M_storage.bx.~box();
            break;
        case type::cylinder:
            M_storage.cyl.~cylinder();
            break;
        case type::cone:
            M_storage.cn.~cone();
            break;
        case type::pyramid:
            M_storage.pyr.~pyramid();
            break;
        }
    }

    template <typename... Args> void construct_mesh(Args &&...args)
    { std::construct_at(&M_storage.msh, std::forward<Args>(args)...); }

    template <typename... Args> void construct_sphere(Args &&...args)
    { std::construct_at(&M_storage.sph, std::forward<Args>(args)...); }

    template <typename... Args> void construct_box(Args &&...args)
    { std::construct_at(&M_storage.bx, std::forward<Args>(args)...); }

    template <typename... Args> void construct_cylinder(Args &&...args)
    { std::construct_at(&M_storage.cyl, std::forward<Args>(args)...); }

    template <typename... Args> void construct_cone(Args &&...args)
    { std::construct_at(&M_storage.cn, std::forward<Args>(args)...); }

    template <typename... Args> void construct_pyramid(Args &&...args)
    { std::construct_at(&M_storage.pyr, std::forward<Args>(args)...); }

    void copy_from(const shape &other)
    {
        switch (M_type)
        {
        case type::mesh:
            construct_mesh(other.M_storage.msh);
            break;
        case type::sphere:
            construct_sphere(other.M_storage.sph);
            break;
        case type::box:
            construct_box(other.M_storage.bx);
            break;
        case type::cylinder:
            construct_cylinder(other.M_storage.cyl);
            break;
        case type::cone:
            construct_cone(other.M_storage.cn);
            break;
        case type::pyramid:
            construct_pyramid(other.M_storage.pyr);
            break;
        default:
            std::unreachable();
        }
    }

    // NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
    void move_from(shape &&other)
    {
        switch (M_type)
        {
        case type::mesh:
            construct_mesh(std::move(other.M_storage.msh));
            break;
        case type::sphere:
            construct_sphere(std::move(other.M_storage.sph));
            break;
        case type::box:
            construct_box(std::move(other.M_storage.bx));
            break;
        case type::cylinder:
            construct_cylinder(std::move(other.M_storage.cyl));
            break;
        case type::cone:
            construct_cone(std::move(other.M_storage.cn));
            break;
        case type::pyramid:
            construct_pyramid(std::move(other.M_storage.pyr));
            break;
        default:
            std::unreachable();
        }
    }
};

/// @brief A view of a collision shape placed in world space
/// Provides world-space collision queries by transforming into local space and back.
/// Instances are meant for temporary use in queries and should not be stored long-term by
/// users. Watch out for dangling references.
class instance
{
public:
    instance(shape shp, const vec3<si::metre> &position,
             const quat<one> &orientation = quat<one>::identity())
        : M_shape{std::move(shp)}, M_position{position}, M_orientation{orientation}
    {
    }

    [[nodiscard]] const shape &geometry() const { return M_shape; }
    [[nodiscard]] const vec3<si::metre> &position() const { return M_position; }
    [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }

    [[nodiscard]] vec3<si::metre> vertex(unsigned int index) const
    {
        assert(M_shape.type() == shape::type::mesh);
        assert(index < M_shape.vertices().size());
        return M_orientation * M_shape.vertices()[index] + M_position;
    }

    /// @brief Compute the world-space AABB by rotating the local AABB and translating.
    [[nodiscard]] aabb bounds() const { return M_shape.bounds() * M_orientation + M_position; }

    /// @brief Compute the world-space bounding sphere. Rotation-invariant - only the
    /// center is translated.
    [[nodiscard]] bounding_sphere bsphere() const
    {
        auto local = M_shape.bsphere();
        return {.center = M_orientation * local.center + M_position, .radius = local.radius};
    }

    /// @brief Compute the world-space ray intersection by transforming the ray into local space
    /// and back.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        // Transform ray into local space.
        auto inv_orient = M_orientation.conjugate();
        auto local_origin = inv_orient * (r.origin() - M_position);
        auto local_dir = inv_orient * r.direction();
        ray local_ray{local_origin, local_dir};

        auto hit = M_shape.ray_intersect(local_ray, max_distance);
        if (hit)
        {
            hit->pos = M_orientation * hit->pos + M_position;
            hit->normal = M_orientation * hit->normal;
        }
        return hit;
    }

    /// @brief Compute the closest point on the mesh surface to the given world-space point.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        auto local_point = M_orientation.conjugate() * (point - M_position);
        return M_orientation * M_shape.closest_point(local_point) + M_position;
    }

    /// @brief Point containment test in world space.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        auto local_point = M_orientation.conjugate() * (point - M_position);
        return M_shape.contains(local_point);
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given world-space sphere.
    [[nodiscard]] std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        bounding_sphere local_sphere{
            .center = M_orientation.conjugate() * (sphere.center - M_position),
            .radius = sphere.radius,
        };
        return M_shape.overlap_sphere(local_sphere);
    }

    /// @brief GJK support function in world space. Rotates the direction into
    /// local frame, queries the mesh, and transforms the result back.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        auto local_dir = M_orientation.conjugate() * direction;
        return M_orientation * M_shape.support(local_dir) + M_position;
    }

    /// @brief Rotate the local-frame inertia tensor into the world frame: I_world = R I_local R^T.
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        auto R = M_orientation.to_rotation_matrix(); // NOLINT
        return R * M_shape.inertia_tensor(density) * R.transpose();
    }

private:
    shape M_shape;
    vec3<si::metre> M_position;
    quat<one> M_orientation;
};

/// @brief Create an instance view of this mesh at the given position and orientation.
inline instance mesh::at(const vec3<si::metre> &position, const quat<one> &orientation) const
{ return {ptr(), position, orientation}; }

inline instance box::at(const vec3<si::metre> &position, const quat<one> &orientation) const
{ return {*this, position, orientation}; }
inline instance sphere::at(const vec3<si::metre> &position, const quat<one> &orientation) const
{ return {*this, position, orientation}; }
inline instance cylinder::at(const vec3<si::metre> &position, const quat<one> &orientation) const
{ return {*this, position, orientation}; }
inline instance cone::at(const vec3<si::metre> &position, const quat<one> &orientation) const
{ return {*this, position, orientation}; }
inline instance pyramid::at(const vec3<si::metre> &position, const quat<one> &orientation) const
{ return {*this, position, orientation}; }

inline instance shape::at(const vec3<si::metre> &pos, const quat<one> &orientation) const
{ return {*this, pos, orientation}; }

} // namespace physkit
