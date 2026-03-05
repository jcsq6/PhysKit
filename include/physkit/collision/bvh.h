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
        std::uint32_t start{}; // leaf: first node index; internal: right child index
        std::uint16_t count{}; // >0 = leaf (node count); 0 = internal node
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

class dynamic_bvh
{
public:
    static constexpr std::size_t stack_size = 64;

    struct node_handle
    {
        using value_type = std::uint32_t;
        value_type value;

        constexpr operator value_type() const { return value; }
    };

    struct object_handle
    {
        using value_type = std::uint32_t;

        value_type value;

        constexpr operator value_type() const { return value; }
    };

    dynamic_bvh(std::size_t initial_capacity = 1024) { M_nodes.reserve(initial_capacity); }

    auto add(object_handle id, const aabb &bounds)
    {
        auto leaf_idx = allocate_node();
        M_nodes[leaf_idx].data = id;
        M_nodes[leaf_idx].bounds = bounds;
        insert_leaf(leaf_idx);
        refit_and_rotate(M_nodes[leaf_idx].parent);
        return leaf_idx;
    }

    bool update_leaf(node_handle leaf_idx, const aabb &true_bounds,
                     const vec3<si::metre> &displacement);

    void remove_leaf(node_handle leaf_idx)
    {
        extract_leaf(leaf_idx);
        free_node(leaf_idx);
    }

    void query_aabb(const aabb &box, std::predicate<object_handle> auto &&callback) const
    {
        if (M_root == node::null) return;

        std::array<node_handle, stack_size> stack{};
        int sp = 0;
        stack[sp++] = M_root;

        while (sp > 0)
        {
            auto node_idx = stack[--sp];
            const auto &node = M_nodes[node_idx];

            if (!node.bounds.intersects(box)) continue;

            if (node.is_leaf())
            {
                if (!callback(node.data)) return;
                continue;
            }

            assert(sp + 1 < stack_size &&
                   "BVH stack overflow: adjust detail::dynamic_bvh::stack_size");

            auto [first, second] = M_nodes[node.children.left].bounds.surface_area() >
                                           M_nodes[node.children.right].bounds.surface_area()
                                       ? std::pair{node.children.left, node.children.right}
                                       : std::pair{node.children.right, node.children.left};
            stack[sp++] = first;
            stack[sp++] = second;
        }
    }

    void raycast(const ray &r, quantity<si::metre> max_distance,
                 std::regular_invocable<object_handle, quantity<si::metre>,
                                        quantity<si::metre>> auto &&callback) const
    {
        if (M_root == node::null) return;

        std::array<node_handle, stack_size> stack{};
        int sp = 0;
        stack[sp++] = M_root;

        while (sp > 0)
        {
            auto node_idx = stack[--sp];
            const auto &node = M_nodes[node_idx];

            auto d_node = r.intersect_distance(node.bounds, max_distance);
            if (!d_node.has_value()) continue;

            if (node.is_leaf())
            {
                // callback should evaluate the exact shape intersection and return the new max
                // distance. returning 0.0 * m ends the raycast
                max_distance = callback(node.data, *d_node, max_distance);
                if (max_distance <= 0.0 * si::metre) return;
                continue;
            }

            assert(sp + 1 < stack_size &&
                   "BVH stack overflow: adjust detail::dynamic_bvh::stack_size");

            auto [left, right] = node.children;
            auto d_left = r.intersect_distance(M_nodes[left].bounds, max_distance);
            auto d_right = r.intersect_distance(M_nodes[right].bounds, max_distance);

            if (d_left && d_right)
            {
                // push further node first so that closer one is processed first
                if (*d_left > *d_right)
                {
                    stack[sp++] = left;
                    stack[sp++] = right;
                }
                else
                {
                    stack[sp++] = right;
                    stack[sp++] = left;
                }
            }
            else if (d_left)
                stack[sp++] = left;
            else if (d_right)
                stack[sp++] = right;
        }
    }

    [[nodiscard]] auto &bounds(node_handle leaf_idx) const
    {
        assert(leaf_idx < M_nodes.size() && M_nodes[leaf_idx].is_leaf());
        return M_nodes[leaf_idx].bounds;
    }

    [[nodiscard]] auto data(node_handle leaf_idx) const
    {
        assert(leaf_idx < M_nodes.size() && M_nodes[leaf_idx].is_leaf());
        return M_nodes[leaf_idx].data;
    }

private:
    struct node
    {
        static constexpr node_handle null{std::numeric_limits<node_handle::value_type>::max()};
        static constexpr auto free_node = std::numeric_limits<std::uint32_t>::max();
        struct child_locs
        {
            node_handle left;
            node_handle right;
        };
        aabb bounds;
        node_handle parent = null;

        union
        {
            // State 1: internal node
            child_locs children;

            // State 2: leaf node
            object_handle data{
                std::numeric_limits<object_handle::value_type>::
                    max()}; // Points to the rigid body or scene entity this leaf represents.

            // State 3: free node
            node_handle next_free; // Index of the next free node in the free list.
        };

        std::uint32_t height = free_node;

        void set_free(node_handle next_free)
        {
            height = free_node;
            this->next_free = next_free;
        }

        [[nodiscard]] bool is_leaf() const { return height == 0; }
        [[nodiscard]] bool is_free() const { return height == free_node; }
    };

    std::vector<node> M_nodes;
    node_handle M_root = node::null;
    node_handle M_free_head = node::null;

    node_handle allocate_node()
    {
        if (M_free_head != node::null)
        {
            auto allocated = M_free_head;
            M_free_head = M_nodes[allocated].next_free;

            M_nodes[allocated].parent = node::null;
            M_nodes[allocated].height = 0; // Mark as leaf by default
            return allocated;
        }

        M_nodes.emplace_back().height = 0;
        return static_cast<node_handle>(M_nodes.size() - 1);
    }

    void free_node(node_handle node)
    {
        assert(node < M_nodes.size());

        M_nodes[node].set_free(M_free_head);
        M_free_head = node;
    }

    node_handle insert_leaf(node_handle leaf_idx);
    void extract_leaf(node_handle leaf_idx);
    void refit_and_rotate(node_handle node_idx);
    node_handle balance(node_handle node_idx);
};
} // namespace detail
} // namespace physkit