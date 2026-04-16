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
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

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

    /// TODO: Add in support to return obb objects -> much more tedious, more research later.

private:
    aabb M_aabb;
    bounding_sphere M_bsphere;
    quantity<si::metre> M_radius;
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
    { return (M_height * pow<2>(M_radius)) * std::numbers::pi / 3; }
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
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        using namespace mp_units::si::unit_symbols;
        std::optional<ray::hit> best;

        auto o = r.origin();
        auto dir = r.direction();

        // Base plane intersection (y = 0)
        if (dir.y() != 0)
            if (auto dist = -o.y() / dir.y(); dist >= 0 * m && dist <= max_distance)
                if (auto point = o + dist * dir;
                    pow<2>(point.x()) + pow<2>(point.z()) <= pow<2>(M_radius))
                    best = ray::hit{.pos = point, .normal = vec3<one>{0, -1, 0}, .distance = dist};

        // Lateral surface intersection: $x^2 + z^2 = (r/h)^2(h - y)^2$
        auto rad = M_radius;
        auto h = M_height;

        auto a = dir.x() * dir.x() + dir.z() * dir.z() - dir.y() * dir.y() * rad * rad / (h * h);
        auto b = 2 * o.x() * dir.x() + 2 * o.z() * dir.z() -
                 2 * rad * rad * o.y() * dir.y() / (h * h) + 2 * dir.y() * rad * rad / h;
        auto c = o.x() * o.x() + o.z() * o.z() - o.y() * o.y() * rad * rad / (h * h) +
                 2 * o.y() * rad * rad / h - rad * rad;

        auto det = b * b - 4 * a * c;
        if (det >= 0 * m * m)
        {
            auto sqrt_det = sqrt(det);
            auto t1 = (-b - sqrt_det) / (2 * a);
            auto t2 = (-b + sqrt_det) / (2 * a);
            if (t1 > t2) std::swap(t1, t2);

            for (auto t : {t1, t2})
            {
                auto dist = t;
                if (dist < 0 * m || dist > max_distance) continue;
                if (best.has_value() && best->distance <= dist) break;

                auto p = o + dist * dir;
                // Reject hits outside the finite cone
                if (p.y() < 0 * m || p.y() > M_height) continue;

                // Outward normal: $\nabla(x^2+z^2-(r/h)^2(h-y)^2) \propto (x, (r^2/h^2)(h-y), z)$

                vec3<one> norm;
                if (p.x() == 0.0 * m && p.z() == 0.0 * m)
                    norm = vec3<one>{0.0, 1.0, 0.0};
                else
                    norm = vec3{p.x(), rad * rad * (h - p.y()) / (h * h), p.z()}.normalized();

                best = ray::hit{.pos = p, .normal = norm, .distance = dist};
                break;
            }
        }

        return best;
    }
    /// @brief Closest point on the cone surface in local space. O(1) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        using namespace mp_units::si::unit_symbols;

        // Reduce to 2D: (d, py) where d = distance from the y-axis
        auto d = sqrt(point.x() * point.x() + point.z() * point.z());

        // Direction from axis in xz plane (for converting back to 3D)
        auto dir_x = d > 0 * m ? point.x() / d : 1.0 * one;
        auto dir_z = d > 0 * m ? point.z() / d : 0.0 * one;

        // Candidate 1: closest point on the base disk ($y=0, 0 \leq \text{dist} \leq r$)
        auto base_d = d < M_radius ? d : M_radius;
        auto base_dist2 = (d - base_d) * (d - base_d) + point.y() * point.y();

        // Candidate 2: closest point on slant from ($r$,0) to (0,$h$)
        // $P(t) = ((1-t)r, th)$ for $t \in [0,1]$
        // $u = (d-r, py), v = (-r, h), t = u \cdot v / v \cdot v$
        auto t = (-M_radius * (d - M_radius) + M_height * point.y()) /
                 (M_radius * M_radius + M_height * M_height);
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        auto slant_d = (1.0 - t) * M_radius;
        auto slant_y = t * M_height;
        auto slant_dist2 =
            (d - slant_d) * (d - slant_d) + (point.y() - slant_y) * (point.y() - slant_y);

        quantity<si::metre> res_d; // NOLINT
        quantity<si::metre> res_y; // NOLINT
        if (base_dist2 <= slant_dist2)
        {
            res_d = base_d;
            res_y = 0.0 * si::metre;
        }
        else
        {
            res_d = slant_d;
            res_y = slant_y;
        }

        return vec3{res_d * dir_x, res_y, res_d * dir_z};
    }
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

    /// TODO: Add in support to return obb objects -> much more tedious, more research later.

private:
    aabb M_aabb;
    bounding_sphere M_bsphere;
    quantity<si::metre> M_radius;
    quantity<si::metre> M_height;
};

class shape
{
public:
    enum class type : std::uint8_t
    {
        shape_sphere = 0,
        shape_box,
        // shape_cylinder,
        shape_cone,
        // shape_convex_hull,
        shape_mesh
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

    shape() : M_type(type::shape_box) { construct_box(vec3{0.5f, 0.5f, 0.5f} * si::metre); }

    shape(std::shared_ptr<const physkit::mesh> m) : M_type(type::shape_mesh)
    { construct_mesh(std::move(m)); }

    shape(const physkit::sphere &s) : M_type(type::shape_sphere) { construct_sphere(s); }

    shape(const physkit::box &b) : M_type(type::shape_box) { construct_box(b); }

    shape(const physkit::cone &c) : M_type(type::shape_cone) { construct_cone(c); }

    [[nodiscard]] const std::shared_ptr<const physkit::mesh> &mesh() const
    {
        assert(M_type == type::shape_mesh);
        return M_storage.msh;
    }
    [[nodiscard]] const physkit::sphere &sphere() const
    {
        assert(M_type == type::shape_sphere);
        return M_storage.sph;
    }
    [[nodiscard]] const physkit::box &box() const
    {
        assert(M_type == type::shape_box);
        return M_storage.bx;
    }
    [[nodiscard]] const physkit::cone &cone() const
    {
        assert(M_type == type::shape_cone);
        return M_storage.cn;
    }

    shape &operator=(std::shared_ptr<const physkit::mesh> m)
    {
        destroy_active();
        M_type = type::shape_mesh;
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
        case type::shape_sphere:
            return std::forward<F>(f)(M_storage.sph);
        case type::shape_box:
            return std::forward<F>(f)(M_storage.bx);
        case type::shape_mesh:
            return std::forward<F>(f)(*M_storage.msh);
        case type::shape_cone:
            return std::forward<F>(f)(M_storage.cn);
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
        assert(M_type == type::shape_mesh);
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
        case type::shape_mesh:
            return M_storage.msh->is_convex();
        case type::shape_sphere:
        case type::shape_box:
        case type::shape_cone:
            return true;
        }
    }

    [[nodiscard]] type type() const { return M_type; };

    // mesh only methods for compatibility
    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const
    {
        assert(M_type == type::shape_mesh);
        return M_storage.msh->vertices();
    }
    [[nodiscard]] std::span<const triangle_t> triangles() const
    {
        assert(M_type == type::shape_mesh);
        return M_storage.msh->triangles();
    }
    [[nodiscard]] const vec3<si::metre> &vertex(unsigned int index) const
    {
        assert(M_type == type::shape_mesh);
        const auto &msh = M_storage.msh;
        assert(index < msh->vertices().size());
        return msh->vertices()[index];
    }

private:
    union storage_t
    {
        std::shared_ptr<const physkit::mesh> msh;
        physkit::sphere sph;
        physkit::box bx;
        physkit::cone cn;
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
        case type::shape_mesh:
            M_storage.msh.~shared_ptr();
            break;
        case type::shape_sphere:
            M_storage.sph.~sphere();
            break;
        case type::shape_box:
            M_storage.bx.~box();
            break;
        case type::shape_cone:
            M_storage.cn.~cone();
            break;
        }
    }

    template <typename... Args> void construct_mesh(Args &&...args)
    { std::construct_at(&M_storage.msh, std::forward<Args>(args)...); }

    template <typename... Args> void construct_sphere(Args &&...args)
    { std::construct_at(&M_storage.sph, std::forward<Args>(args)...); }

    template <typename... Args> void construct_box(Args &&...args)
    { std::construct_at(&M_storage.bx, std::forward<Args>(args)...); }

    template <typename... Args> void construct_cone(Args &&...args)
    { std::construct_at(&M_storage.cn, std::forward<Args>(args)...); }

    void copy_from(const shape &other)
    {
        switch (M_type)
        {
        case type::shape_mesh:
            construct_mesh(other.M_storage.msh);
            break;
        case type::shape_sphere:
            construct_sphere(other.M_storage.sph);
            break;
        case type::shape_box:
            construct_box(other.M_storage.bx);
            break;
        case type::shape_cone:
            construct_cone(other.M_storage.cn);
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
        case type::shape_mesh:
            construct_mesh(std::move(other.M_storage.msh));
            break;
        case type::shape_sphere:
            construct_sphere(std::move(other.M_storage.sph));
            break;
        case type::shape_box:
            construct_box(std::move(other.M_storage.bx));
            break;
        case type::shape_cone:
            construct_cone(std::move(other.M_storage.cn));
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
    instance(const shape &shp, const vec3<si::metre> &position,
             const quat<one> &orientation = quat<one>::identity());

    [[nodiscard]] const shape &geometry() const { return *M_shape; }
    [[nodiscard]] const vec3<si::metre> &position() const { return M_position; }
    [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }

    [[nodiscard]] vec3<si::metre> vertex(unsigned int index) const
    {
        assert(M_shape->type() == shape::type::shape_mesh);
        assert(index < M_shape->vertices().size());
        return M_orientation * M_shape->vertices()[index] + M_position;
    }

    /// @brief Compute the world-space AABB by rotating the local AABB and translating.
    [[nodiscard]] aabb bounds() const
    {
        auto ret = M_shape->bounds() * M_orientation + M_position;
        return M_shape->bounds() * M_orientation + M_position;
    }

    /// @brief Compute the world-space bounding sphere. Rotation-invariant — only the
    /// center is translated.
    [[nodiscard]] bounding_sphere bsphere() const
    {
        auto local = M_shape->bsphere();
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

        auto hit = M_shape->ray_intersect(local_ray, max_distance);
        if (hit)
        {
            hit->pos = M_orientation * hit->pos + M_position;
            hit->normal = M_orientation * hit->normal;
        }
        return hit;
    }

    /// @brief Compute the closest point on the mesh surface to the given world-space point.
    /// O(log N) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_point = inv_orient * (point - M_position);
        auto local_closest = M_shape->closest_point(local_point);
        auto ret = M_orientation * local_closest + M_position;
        return M_orientation * local_closest + M_position;
    }

    /// @brief Point containment test in world space. O(log N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_point = inv_orient * (point - M_position);
        auto ret = M_shape->contains(local_point);
        return M_shape->contains(local_point);
    }

    /// @brief Gathers indices of triangles whose vertices overlap the given world-space sphere.
    [[nodiscard]] std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const
    {
        auto inv_orient = M_orientation.conjugate();
        bounding_sphere local_sphere{
            .center = inv_orient * (sphere.center - M_position),
            .radius = sphere.radius,
        };
        auto ret = M_shape->overlap_sphere(local_sphere);
        return M_shape->overlap_sphere(local_sphere);
    }

    /// @brief GJK support function in world space. Rotates the direction into
    /// local frame, queries the mesh, and transforms the result back.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        auto inv_orient = M_orientation.conjugate();
        auto local_dir = inv_orient * direction;
        auto local_support = M_shape->support(local_dir);
        auto ret = M_orientation * local_support + M_position;
        return M_orientation * local_support + M_position;
    }

    /// @brief Compute the inertia tensor rotated into the world frame and shifted to the
    /// instance's position via the parallel axis theorem.
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        // TODO is this correct???
        return M_shape->inertia_tensor(density) * M_orientation.to_rotation_matrix();
    }

private:
    const shape *M_shape; // NOLINT
    vec3<si::metre> M_position;
    quat<one> M_orientation;
};

inline instance::instance(const shape &shp, const vec3<si::metre> &position,
                          const quat<one> &orientation)
    : M_shape{&shp}, M_position{position}, M_orientation{orientation}
{
}

} // namespace physkit
