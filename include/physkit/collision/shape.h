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
    { return 2 * M_half_extents.x() * 2 * M_half_extents.y() * 2 * M_half_extents.z(); }

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

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

private:
    aabb M_aabb;
    bounding_sphere M_bsphere;
    quantity<si::metre> M_radius;
};

class cone //base at origin, height of height, radius of radius
{
public:
    /// @brief A view of a cone placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.

    cone(const cone &) = default;
    cone &operator=(const cone &) = default;
    cone(cone &&) = default;
    cone &operator=(cone &&) = default;
    ~cone() = default;

    cone(const quantity<si::metre> radius, const quantity<si::metre> height) : M_radius{radius}, M_height{radius}
    {
        M_aabb = aabb::from_points({vec3<si::metre>{M_radius, M_height, M_radius},
                                    vec3<si::metre>{-M_radius, 0.0*si::metre, -M_radius}});

        M_bsphere = bounding_sphere(vec3<si::metre>{0*si::metre, M_height/2, 0*si::metre}, sqrt(M_height*M_radius)/2);
    }

    [[nodiscard]] const quantity<si::metre> &radius() const { return M_radius; }
    [[nodiscard]] const quantity<si::metre> &height() const { return M_height; }
    [[nodiscard]] const aabb &bounds() const { return M_aabb; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const
    { return (M_height*pow<2>(M_radius)) * std::numbers::pi/3; }
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] vec3<si::metre> mass_center() const
    {
        return vec3<si::metre>{0*si::metre, M_height/4, 0*si::metre};
    }
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const
    {
        //rotation about y is 0.3*m*r^2
        //rotation about other axis is m*((3/20)*r^2 + 0.1*h^2)
        using namespace mp_units::si::unit_symbols;
        auto iy = 0.3*density*volume()*pow<2>(M_radius);
        auto ixz = density*volume()*((3.0/20.0)*pow<2>(M_radius) + 0.1*pow<2>(M_height));

        return vec3{ixz, iy, ixz}.as_diagonal();
    }

    /// @brief Ray intersection in local (model) space. O(log N) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const
    {
        //TODO
        using namespace mp_units::si::unit_symbols;
        std::optional<ray::hit> best;

        auto origin = r.origin();
        auto direction = r.direction();
        best = std::nullopt;

        //intersection with base
        auto dist = -origin.y() / direction.y();
        if (dist >= 0*si::metre)
        {
            auto point = origin + dist * direction;
            if (pow<2>(point.x()) + pow<2>(point.z()) <= M_radius*M_radius)
            {
                best = ray::hit{
                    .pos = point,
                    .normal = vec3<one>{0, -1, 0},
                    .distance = dist
                };
            }
        }

        //intersection with cone surface
        double x = direction.x().numerical_value_in(one);
        double y = direction.y().numerical_value_in(one);
        double z = direction.z().numerical_value_in(one);
        double x0 = origin.x().numerical_value_in(si::metre);
        double y0 = origin.y().numerical_value_in(si::metre);
        double z0 = origin.z().numerical_value_in(si::metre);
        double rad = M_radius.numerical_value_in(si::metre);
        double h = M_height.numerical_value_in(si::metre);

        double a = x*x + z*z - y*y*rad*rad/(h*h);
        double b = 2*x0*x+ 2*z0*z - 2*rad*rad*y0*y/(h*h) + 2*y*rad*rad/h;
        double c = x0*x0 + z0*z0 - y0*y0*rad*rad/(h*h) + 2*y0*rad*rad/h - rad*rad;

        double det = b*b -4*a*c;
        if (det >= 0)
        {
            auto dist1 = (-b+det)/(2*a) * si::metre;
            auto dist2 = (-b-det)/(2*a) * si::metre;
            if (dist < dist1)
                dist = dist1;
            if (dist < dist2)
                dist = dist2;

            if (best == std::nullopt || best.value().distance < dist)
            {
                auto p = origin + dist*direction;
                auto circle_point = vec3<si::metre>{p.x(), 0.0*si::metre, p.z()};

                //TODO i hope this is right
                vec3<one> norm = (vec3{M_height*circle_point.x(), M_radius*M_radius, M_height*circle_point.z()}).normalized();
                if (circle_point.x() == 0*si::metre && circle_point.z() == 0*si::metre)
                    norm = vec3<one>{0.0, 1.0, 0.0};

                best = ray::hit
                {
                    .pos = p,
                    .normal = norm,
                    .distance = dist
                };
            }
        }
        if (best.value().distance > max_distance)
            return std::nullopt;
        return best;
    }
    /// @brief Closest point on the sphere surface in local space. O(1) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const
    {
        auto x = point.x().numerical_value_in(si::metre);
        auto y = point.y().numerical_value_in(si::metre);
        auto z = point.z().numerical_value_in(si::metre);
        auto p = vec3<one>{x,y,z};

        auto r = M_radius.numerical_value_in(si::metre);
        auto h = M_height.numerical_value_in(si::metre);

        auto circle_point = vec3<one> {x,0.0,z};
        if (x*z >= r*r)
            circle_point = r * (vec3<one>{x,0.0,z}).normalized();

        auto edge_point = r * circle_point.normalized();
        if (edge_point.norm() == 0.0)
            edge_point = vec3<one>{r, 0.0, 0.0};

        auto tip_point = vec3<one>{0.0, h, 0.0};
        auto line = tip_point - edge_point;
        auto surface_point = edge_point - (tip_point-edge_point)*line.dot(p - edge_point);

        auto best = (p - tip_point).norm();
        int which_best = 0;

        if ((p-circle_point).norm() < best)
        {
            best = (p-circle_point).norm();
            which_best = 1;
        }
        if ((p-edge_point).norm() < best)
        {
            best = (p-edge_point).norm();
            which_best = 2;
        }
        if ((p-surface_point).norm() < best)
        {
            best = (p-surface_point).norm();
            which_best = 3;
        }

        switch (which_best)
        {
        default:
        case 0: return tip_point*si::metre;
        case 1: return circle_point*si::metre;
        case 2: return edge_point*si::metre;
        case 3: return surface_point*si::metre;
        }
    }
    /// @brief Point containment test in local space. O(1) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const
    {
        using namespace mp_units;
        //calculate point distance from y axis
        //calculate radius at that point
        if (point.y() < 0.0*si::metre) return false;
        return pow<2>(point.x())+pow<2>(point.z()) <= pow<2>(M_radius * (1.0 - point.y()/M_height));
    }

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const
    {
        //TODO
        //places to check: edge of base, tip, point on cone surface along direction
        auto x = direction.x();
        auto y = direction.y();
        auto z = direction.z();

        auto r = M_radius.numerical_value_in(si::metre);
        auto h = M_height.numerical_value_in(si::metre);

        vec3<one> tip_point = {0.0, h, 0.0};
        vec3<one> circle_point;
        if (pow<2>(direction.x())+pow<2>(direction.z()) > r*r)
            circle_point = r * (vec3<one>{x, 0.0, z}).normalized();
        else
            circle_point = r * vec3<one>{x,0.0,z};

        vec3<one> edge_point = r * circle_point.normalized();
        if (edge_point.norm() == 0.0)
            edge_point = vec3<one>{r, 0.0, 0.0};

        vec3<one> line = tip_point - edge_point;
        vec3<one> surface_point = edge_point - (line.dot(direction - edge_point)*
            (tip_point - edge_point));

        auto best = direction.dot(tip_point);
        int which_best = 0;

        //point along circle
        if (direction.dot(edge_point) > best)
        {
            best = direction.dot(edge_point);
            which_best = 1;
        }
        if (direction.dot(circle_point) > best)
        {
            best = direction.dot(circle_point);
            which_best = 2;
        }
        if (direction.dot(surface_point) > best)
        {
            best = direction.dot(surface_point);
            which_best = 3;
        }
        switch (which_best)
        {
        default:
        case 0: return tip_point*si::metre;
        case 1: return edge_point*si::metre;
        case 2: return circle_point*si::metre;
        case 3: return surface_point*si::metre;
        }
    }

    /// @brief - Add in support to return obb objects -> much more tedious, more research later.

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
