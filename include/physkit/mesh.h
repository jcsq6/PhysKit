#pragma once
#include "detail/bounds.h"
#include "detail/bvh.h"
#include "detail/types.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <cassert>
#include <memory>
#include <span>
#include <vector>

namespace physkit
{

struct triangle_t : std::array<unsigned int, 3>
{
    [[nodiscard]] std::array<vec3<si::metre>, 3>
    vertices(std::span<const vec3<si::metre>> verts) const
    { return {verts[(*this)[0]], verts[(*this)[1]], verts[(*this)[2]]}; }

    [[nodiscard]] std::array<vec3<si::metre>, 3> vertices(const auto &mesh) const
        requires(requires { mesh.vertex(0U); })
    { return {mesh.vertex((*this)[0]), mesh.vertex((*this)[1]), mesh.vertex((*this)[2])}; }

    [[nodiscard]] auto normal(const auto &ctx) const
    {
        auto [a, b, c] = vertices(ctx);
        return (b - a).cross(c - a).normalized();
    }

    [[nodiscard]] auto center(const auto &ctx) const
    {
        auto [a, b, c] = vertices(ctx);
        return (a + b + c) / 3.0;
    }

    [[nodiscard]] auto to_aabb(const auto &ctx) const
    {
        auto [a, b, c] = vertices(ctx);
        return aabb{
            .min = {std::min({a.x(), b.x(), c.x()}), std::min({a.y(), b.y(), c.y()}),
                    std::min({a.z(), b.z(), c.z()})},
            .max = {std::max({a.x(), b.x(), c.x()}), std::max({a.y(), b.y(), c.y()}),
                    std::max({a.z(), b.z(), c.z()})},
        };
    }

    // Closest point on a single triangle to a query point, using barycentric coordinates.
    // TODO: benchmark against other methods (e.g. Voronoi region tests) to see if this is a
    // bottleneck.
    [[nodiscard]] auto closest_point(const vec3<si::metre> &p, const auto &ctx) const
    {
        using namespace mp_units::si::unit_symbols;

        auto [a, b, c] = this->vertices(ctx);

        auto ab = b - a;
        auto ac = c - a;
        auto ap = p - a;

        auto d00 = ab.dot(ab);
        auto d01 = ab.dot(ac);
        auto d11 = ac.dot(ac);
        auto d20 = ap.dot(ab);
        auto d21 = ap.dot(ac);

        auto denom = d00 * d11 - d01 * d01;
        if (abs(denom) < 1e-12 * pow<4>(m)) return a; // degenerate triangle, return a vertex

        auto v = (d11 * d20 - d01 * d21) / denom;
        auto w = (d00 * d21 - d01 * d20) / denom;
        auto u = 1.0 - v - w;

        if (u >= 0.0 && v >= 0.0 && w >= 0.0) return u * a + v * b + w * c;

        // Point projects outside the triangle - project onto each edge and pick closest
        auto candidate = a;
        auto candidate_dist_sq = (candidate - p).squared_norm();
        auto check_edge = [&p, &candidate, &candidate_dist_sq](const vec3<m> &e0, const vec3<m> &e1)
        {
            auto edge = e1 - e0;
            auto t =
                std::clamp(static_cast<double>((p - e0).dot(edge) / edge.squared_norm()), 0.0, 1.0);
            auto attempt = e0 + t * edge;
            if (auto attempt_dist_sq = (attempt - p).squared_norm();
                attempt_dist_sq < candidate_dist_sq)
            {
                candidate = attempt;
                candidate_dist_sq = attempt_dist_sq;
            }
        };
        check_edge(a, b);
        check_edge(b, c);
        check_edge(c, a);
        return candidate;
    }
};

class mesh : public std::enable_shared_from_this<mesh>
{
    struct key
    {
    };

public:
    /// @brief A view of a mesh placed in world space.
    /// Provides world-space collision queries by transforming into local space and back.
    /// Instances are meant for temporary use in queries and should not be stored long-term by
    /// users. Watch out for dangling references.
    class instance
    {
    public:
        instance(const mesh &msh, const vec3<si::metre> &position,
                 const quat<one> &orientation = quat<one>::identity());

        [[nodiscard]] const mesh &geometry() const { return *M_mesh; }
        [[nodiscard]] const vec3<si::metre> &position() const { return M_position; }
        [[nodiscard]] const quat<one> &orientation() const { return M_orientation; }

        [[nodiscard]] vec3<si::metre> vertex(unsigned int index) const
        {
            assert(index < M_mesh->vertices().size());
            return M_orientation * M_mesh->vertices()[index] + M_position;
        }

        /// @brief Compute the world-space AABB by rotating the local AABB and translating.
        [[nodiscard]] aabb bounds() const;

        /// @brief Compute the world-space bounding sphere. Rotation-invariant — only the
        /// center is translated.
        [[nodiscard]] bounding_sphere bsphere() const
        {
            auto local = M_mesh->bsphere();
            return {.center = M_orientation * local.center + M_position, .radius = local.radius};
        }

        /// @brief Compute the world-space ray intersection by transforming the ray into local space
        /// and back.
        [[nodiscard]] std::optional<ray::hit>
        ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                        std::numeric_limits<quantity<si::metre>>::infinity()) const;

        /// @brief Compute the closest point on the mesh surface to the given world-space point.
        /// O(log N) time.
        [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;

        /// @brief Point containment test in world space. O(log N) time.
        [[nodiscard]] bool contains(const vec3<si::metre> &point) const;

        /// @brief Gathers indices of triangles whose vertices overlap the given world-space sphere.
        [[nodiscard]] std::vector<std::uint32_t>
        overlap_sphere(const bounding_sphere &sphere) const;

        /// @brief GJK support function in world space. Rotates the direction into
        /// local frame, queries the mesh, and transforms the result back.
        [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;

        /// @brief Compute the inertia tensor rotated into the world frame and shifted to the
        /// instance's position via the parallel axis theorem.
        [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
        inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const;

    private:
        const mesh *M_mesh; // NOLINT
        vec3<si::metre> M_position;
        quat<one> M_orientation;
    };

    mesh(key /*unused*/) {}
    mesh(std::span<const vec3<si::metre>> vertices, std::span<const triangle_t> triangles,
         key /*unused*/)
        : M_vertices(std::from_range_t{}, vertices), M_triangles(std::from_range_t{}, triangles),
          M_bounds(aabb::from_points(vertices)), M_bsphere(bounding_sphere::from_points(vertices))
    {
        thread_local std::vector<aabb> bounds;
        thread_local std::vector<vec3<si::metre>> centroids;

        bounds.clear();
        centroids.clear();
        bounds.reserve(M_triangles.size());
        centroids.reserve(M_triangles.size());

        for (const auto &tri : M_triangles)
        {
            bounds.push_back(tri.to_aabb(*this));
            centroids.push_back(tri.center(*this));
        }

        auto indices = M_bvh.build(bounds, centroids);
        std::vector<triangle_t> sorted_tris(M_triangles.size());
        for (std::size_t i = 0; i < indices.size(); ++i) sorted_tris[i] = M_triangles[indices[i]];
        M_triangles = std::move(sorted_tris);
    }

    mesh(const mesh &) = default;
    mesh &operator=(const mesh &) = default;
    mesh(mesh &&) = default;
    mesh &operator=(mesh &&) = default;
    ~mesh() = default;

    static std::shared_ptr<mesh> make(std::span<const vec3<si::metre>> vertices,
                                      std::span<const triangle_t> triangles);

    /// @brief Create a box mesh centered at the origin with the given half-extents.
    static std::shared_ptr<mesh> box(const vec3<si::metre> &half_extents);

    /// @brief Create a UV-sphere mesh centered at the origin.
    /// @param radius Sphere radius.
    /// @param stacks Number of horizontal divisions (latitude, >= 2).
    /// @param sectors Number of vertical divisions (longitude, >= 3).
    static std::shared_ptr<mesh> sphere(quantity<si::metre> radius, unsigned int stacks = 16,
                                        unsigned int sectors = 32);

    /// @brief Create a square-base pyramid with base centered at the origin and apex at
    /// (0, height, 0).
    /// @param base_half Half the side length of the square base.
    /// @param height Height from base to apex.
    static std::shared_ptr<mesh> pyramid(quantity<si::metre> base_half, quantity<si::metre> height);

    auto ptr(this auto &&self) { return std::forward<decltype(self)>(self).shared_from_this(); }

    [[nodiscard]] std::span<const vec3<si::metre>> vertices() const { return M_vertices; }
    [[nodiscard]] std::span<const triangle_t> triangles() const { return M_triangles; }
    [[nodiscard]] const vec3<si::metre> &vertex(unsigned int index) const
    {
        assert(index < M_vertices.size());
        return M_vertices[index];
    }

    [[nodiscard]] const aabb &bounds() const { return M_bounds; }
    [[nodiscard]] const bounding_sphere &bsphere() const { return M_bsphere; }

    [[nodiscard]] quantity<pow<3>(si::metre)> volume() const;
    [[nodiscard]] vec3<si::metre> mass_center() const;
    [[nodiscard]] mat3<si::kilogram * pow<2>(si::metre)>
    inertia_tensor(quantity<si::kilogram / pow<3>(si::metre)> density) const;

    /// @brief Ray intersection in local (model) space. O(log N) time.
    [[nodiscard]] std::optional<ray::hit>
    ray_intersect(const ray &r, quantity<si::metre> max_distance =
                                    std::numeric_limits<quantity<si::metre>>::infinity()) const;
    /// @brief Closest point on the mesh surface in local space. O(log N) time.
    [[nodiscard]] vec3<si::metre> closest_point(const vec3<si::metre> &point) const;
    /// @brief Point containment test in local space. O(log N) time.
    [[nodiscard]] bool contains(const vec3<si::metre> &point) const;

    /// @brief Gathers indices of triangles whose vertices overlap the given sphere.
    std::vector<std::uint32_t> overlap_sphere(const bounding_sphere &sphere) const;

    /// @brief GJK support function in local space.
    [[nodiscard]] vec3<si::metre> support(const vec3<one> &direction) const;
    [[nodiscard]] bool is_convex() const;

    /// @brief Create an instance view of this mesh at the given position and orientation.
    [[nodiscard]] instance at(const vec3<si::metre> &position,
                              const quat<one> &orientation = quat<one>::identity()) const
    { return {*this, position, orientation}; }

private:
    aabb M_bounds;
    bounding_sphere M_bsphere;
    std::vector<vec3<si::metre>> M_vertices;
    std::vector<triangle_t> M_triangles;
    detail::static_bvh M_bvh;
};

inline mesh::instance::instance(const mesh &msh, const vec3<si::metre> &position,
                                const quat<one> &orientation)
    : M_mesh{&msh}, M_position{position}, M_orientation{orientation}
{
}

} // namespace physkit
