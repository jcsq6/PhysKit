#pragma once
#include "bounds.h"
#include <cstdint>
#include <vector>

namespace physkit
{
namespace detail
{
constexpr auto safe_inv(auto x)
{
    if constexpr (std::numeric_limits<double>::is_iec559)
        return 1.0 / static_cast<double>(x);
    else
        return x == 0.0
                   ? std::copysign(std::numeric_limits<double>::infinity(), static_cast<double>(x))
                   : 1.0 / static_cast<double>(x);
}
} // namespace detail

class ray
{
public:
    struct hit
    {
        vec3<si::metre> pos;
        vec3<one> normal;
        quantity<si::metre> distance;
    };

    ray(const vec3<si::metre> &origin, const vec3<one> &direction)
        : M_origin(origin), M_direction(direction.normalized())
    {
    }

    [[nodiscard]] const vec3<si::metre> &origin() const { return M_origin; }
    [[nodiscard]] const vec3<one> &direction() const { return M_direction; }

    // Slab-based ray-AABB intersection test.
    // When a ray direction component is zero, inv_dir is +-inf and the slab products can be
    // 0*inf = NaN. Replace NaN with the correct limit: if the origin is between the slabs
    // (or on a boundary), that axis is unconstrained [-inf, +inf]; if outside, it's empty.
    // TODO: test if mp-units is stopping us from using SIMD here.
    [[nodiscard]] std::optional<quantity<si::metre>>
    intersect_distance(const aabb &box, quantity<si::metre> max_distance) const
    {
        using namespace mp_units::si::unit_symbols;

        auto slab = [](double lo, double hi)
        {
            if (std::isnan(lo)) lo = -std::numeric_limits<double>::infinity();
            if (std::isnan(hi)) hi = std::numeric_limits<double>::infinity();
            return std::pair{std::fmin(lo, hi), std::fmax(lo, hi)};
        };

        auto [tminx, tmaxx] =
            slab(((box.min.x() - M_origin.x()) * detail::safe_inv(M_direction.x()))
                     .numerical_value_in(m),
                 ((box.max.x() - M_origin.x()) * detail::safe_inv(M_direction.x()))
                     .numerical_value_in(m));
        auto [tminy, tmaxy] =
            slab(((box.min.y() - M_origin.y()) * detail::safe_inv(M_direction.y()))
                     .numerical_value_in(m),
                 ((box.max.y() - M_origin.y()) * detail::safe_inv(M_direction.y()))
                     .numerical_value_in(m));
        auto [tminz, tmaxz] =
            slab(((box.min.z() - M_origin.z()) * detail::safe_inv(M_direction.z()))
                     .numerical_value_in(m),
                 ((box.max.z() - M_origin.z()) * detail::safe_inv(M_direction.z()))
                     .numerical_value_in(m));

        auto tmin = std::max({tminx, tminy, tminz});
        auto tmax = std::min({tmaxx, tmaxy, tmaxz});

        if (tmax >= 0.0 && tmin <= tmax && tmin <= max_distance.numerical_value_in(m))
            // If the ray origin is inside the box, tmin will be negative. Clamp to 0.
            return std::max(0.0, tmin) * m;

        return std::nullopt;
    }

    // Moeller-Trumbore ray-triangle test for a single triangle
    // TODO: benchmark to see if std::optional is causing overhead
    [[nodiscard]] std::optional<hit> intersect(std::span<const vec3<si::metre>, 3> tri_vertices,
                                               quantity<si::metre> max_distance) const
    {
        using namespace mp_units::si::unit_symbols;

        constexpr auto eps = 1e-12;

        auto edge1 = tri_vertices[1] - tri_vertices[0];
        auto edge2 = tri_vertices[2] - tri_vertices[0];
        auto h = M_direction.cross(edge2);
        auto a = edge1.dot(h);

        if (a > -eps * m * m && a < eps * m * m) return std::nullopt;

        auto f = 1.0 / a;
        auto s = M_origin - tri_vertices[0];
        auto u = f * s.dot(h);
        if (u < 0.0 || u > 1.0) return std::nullopt;

        auto q = s.cross(edge1);
        auto v = f * M_direction.dot(q);
        if (v < 0.0 || u + v > 1.0) return std::nullopt;

        auto t = f * edge2.dot(q);
        if (t < 0.0 * m || t > max_distance) return std::nullopt;

        return ray::hit{
            .pos = M_origin + M_direction * t,
            .normal = edge1.cross(edge2).normalized(),
            .distance = t,
        };
    }

private:
    vec3<si::metre> M_origin;
    vec3<one> M_direction;
};

namespace detail
{

template <typename T> constexpr bool is_optional_v = false;

template <typename T> constexpr bool is_optional_v<std::optional<T>> = true;

class static_bvh
{
public:
    static constexpr std::size_t stack_size = 128;

    struct node
    {
        aabb bounds;
        std::uint32_t start{}; // leaf: first tri index; internal: right child index
        std::uint16_t count{}; // >0 = leaf (tri count); 0 = internal node
        std::uint16_t axis{};  // split axis (0/1/2)

        [[nodiscard]] bool is_leaf() const { return count > 0; }
        [[nodiscard]] auto primitive_count() const { return count; }
        [[nodiscard]] auto primitive_offset() const { return start; }

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] auto left_child(std::uint32_t current) const { return current + 1; }
        [[nodiscard]] auto right_child() const { return start; }
    };

    static_bvh() = default;

    [[nodiscard]] auto empty() const { return M_nodes.empty(); }

    [[nodiscard]] auto &nodes() const { return M_nodes; }

    std::vector<std::size_t> build(std::span<const aabb> bounds,
                                   std::span<const vec3<si::metre>> centroids);

    void traverse(
        std::predicate<const aabb &> auto &&visit_node,
        std::regular_invocable<const aabb &, std::uint32_t, std::uint32_t> auto &&visit_leaf) const
    {
        if (empty()) return;

        std::array<std::uint32_t, stack_size> stack{};
        int sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            auto node_idx = stack[--sp];
            const auto &node = M_nodes[node_idx];

            if (!visit_node(node.bounds)) continue;

            if (!node.is_leaf())
            {
                assert(sp + 1 < stack_size &&
                       "BVH stack overflow: increase detail::static_bvh::stack_size");

                stack[sp++] = node.right_child();
                stack[sp++] = node.left_child(node_idx);
            }
            else
                visit_leaf(node.bounds, node.primitive_offset(), node.primitive_count());
        }
    }

    void
    traverse_nearest(std::regular_invocable<const aabb &> auto &&calc_distance,
                     std::regular_invocable<std::uint32_t, std::uint32_t> auto &&visit_leaf) const
        requires(is_optional_v<std::invoke_result_t<decltype(calc_distance), const aabb &>>)
    {
        if (empty()) return;

        std::array<std::uint32_t, stack_size> stack{};
        int sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            auto node_idx = stack[--sp];
            const auto &node = M_nodes[node_idx];

            auto d_node = calc_distance(node.bounds);

            if (!d_node.has_value()) continue;

            if (!node.is_leaf())
            {
                assert(sp + 1 < stack_size &&
                       "BVH stack overflow: increase detail::static_bvh::stack_size");

                // Push children in order of proximity
                auto left_idx = node.left_child(node_idx);
                auto right_idx = node.right_child();

                auto d_left = calc_distance(M_nodes[left_idx].bounds);
                auto d_right = calc_distance(M_nodes[right_idx].bounds);

                bool push_left = d_left.has_value();
                bool push_right = d_right.has_value();

                if (push_left && push_right)
                {
                    if (d_left.value() <= d_right.value())
                    {
                        stack[sp++] = right_idx;
                        stack[sp++] = left_idx;
                    }
                    else
                    {
                        stack[sp++] = left_idx;
                        stack[sp++] = right_idx;
                    }
                }
                else if (push_left)
                    stack[sp++] = left_idx;
                else if (push_right)
                    stack[sp++] = right_idx;
            }
            else
                visit_leaf(node.primitive_offset(), node.primitive_count());
        }
    }

private:
    std::vector<node> M_nodes;

    friend struct builder;
};
} // namespace detail
} // namespace physkit