#include "physkit/detail/bvh.h"

namespace physkit::detail
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

constexpr unsigned int sah_bins = 16;
constexpr unsigned int leaf_max_nodes = 4;

// ----------------------------
// -------- Static BVH --------
// ----------------------------

struct builder
{
    constexpr static double cost_traversal = 1.0;
    constexpr static double cost_intersect = 1.0;

    builder(std::span<const aabb> bounds, std::span<const vec3<m>> centroids,
            std::vector<static_bvh::node> &nodes, std::vector<std::size_t> &indices)
        : bounds(bounds), centroids(centroids), nodes(nodes), indices(indices)
    {
        std::ranges::iota(indices, std::size_t{0});

        nodes.clear();
        nodes.reserve(2 * bounds.size());

        auto root_bounds = bounds[0];
        for (std::size_t i = 1; i < bounds.size(); ++i)
            root_bounds = aabb_union(root_bounds, bounds[i]);

        build_recursive(0, bounds.size(), root_bounds);
    }

    // NOTE: can be parallelized if needed. Evaluate cost for large meshes.
    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    void build_recursive(std::size_t start, std::size_t end, const aabb &cur_bound)
    {
        auto node_idx = nodes.size();
        nodes.push_back({});

        auto count = end - start;

        // Make leaf if few enough nodes
        if (count <= leaf_max_nodes)
        {
            nodes[node_idx] = {.bounds = cur_bound,
                               .start = static_cast<std::uint32_t>(start),
                               .count = static_cast<std::uint16_t>(count),
                               .axis = 0};
            return;
        }

        // Find best SAH split across all 3 axes
        double best_cost = std::numeric_limits<double>::max();
        unsigned int best_axis = -1;
        unsigned int best_split = -1;
        aabb best_left_bounds{};
        aabb best_right_bounds{};
        auto parent_area = cur_bound.surface_area();

        struct bin_t
        {
            aabb bounds;
            std::uint32_t count = 0;
            bool empty = true;
        };

        for (int axis = 0; axis < 3; ++axis)
        {
            // Find centroid bounds on this axis
            auto cmin = centroids[indices[start]][axis];
            auto cmax = cmin;
            for (auto i = start + 1; i < end; ++i)
            {
                cmin = std::min(cmin, centroids[indices[i]][axis]);
                cmax = std::max(cmax, centroids[indices[i]][axis]);
            }

            if (cmax - cmin < 1e-12 * m) continue; // degenerate axis

            auto extent = cmax - cmin;

            // Fill bins
            std::array<bin_t, sah_bins> bins{};
            for (auto i = start; i < end; ++i)
            {
                auto idx = indices[i];
                auto frac = (centroids[idx][axis] - cmin) / extent;
                auto b_idx = std::min(static_cast<unsigned int>(frac * sah_bins), sah_bins - 1);
                if (bins[b_idx].empty)
                {
                    bins[b_idx].bounds = bounds[idx];
                    bins[b_idx].empty = false;
                }
                else
                    bins[b_idx].bounds = aabb_union(bins[b_idx].bounds, bounds[idx]);
                bins[b_idx].count++;
            }

            // Sweep from left: compute prefix areas, counts, and bounds
            std::array<quantity<pow<2>(m)>, sah_bins - 1> left_area{};
            std::array<std::uint32_t, sah_bins - 1> left_count{};
            std::array<aabb, sah_bins - 1> left_bounds_arr{};

            aabb left_bounds{};
            bool left_started = false;
            std::uint32_t left_n = 0;
            for (unsigned int i = 0; i < sah_bins - 1; ++i)
            {
                if (!bins[i].empty)
                {
                    left_bounds =
                        left_started ? aabb_union(left_bounds, bins[i].bounds) : bins[i].bounds;
                    left_started = true;
                }
                left_n += bins[i].count;
                left_count[i] = left_n;
                left_area[i] = left_started ? left_bounds.surface_area() : 0.0 * m * m;
                left_bounds_arr[i] = left_bounds;
            }

            // Sweep from right
            aabb right_bounds{};
            bool right_started = false;
            std::uint32_t right_n = 0;
            for (unsigned int i = sah_bins - 1; i >= 1; --i)
            {
                if (!bins[i].empty)
                {
                    right_bounds =
                        right_started ? aabb_union(right_bounds, bins[i].bounds) : bins[i].bounds;
                    right_started = true;
                }
                right_n += bins[i].count;
                auto right_area = right_started ? right_bounds.surface_area() : 0.0 * m * m;
                auto split_idx = i - 1;
                auto cost =
                    cost_traversal +
                    cost_intersect *
                        ((left_area[split_idx] * left_count[split_idx] + right_area * right_n) /
                         parent_area);

                if (cost < best_cost)
                {
                    best_cost = static_cast<double>(cost);
                    best_axis = axis;
                    best_split = i;
                    best_left_bounds = left_bounds_arr[split_idx];
                    best_right_bounds = right_bounds;
                }
            }
        }

        auto leaf_cost = cost_intersect * static_cast<double>(count);
        if (best_axis == -1 || best_cost >= leaf_cost)
        {
            nodes[node_idx] = {.bounds = cur_bound,
                               .start = static_cast<std::uint32_t>(start),
                               .count = static_cast<std::uint16_t>(count),
                               .axis = 0};
            return;
        }

        // Partition the index array around the best split
        auto cmin = centroids[indices[start]][best_axis];
        auto cmax = cmin;
        for (auto i = start + 1; i < end; ++i)
        {
            cmin = std::min(cmin, centroids[indices[i]][best_axis]);
            cmax = std::max(cmax, centroids[indices[i]][best_axis]);
        }
        auto part_extent = cmax - cmin;

        auto mid = start;
        for (auto i = start; i < end; ++i)
        {
            auto frac = (centroids[indices[i]][best_axis] - cmin) / part_extent;
            auto b_idx = std::min(static_cast<unsigned int>(frac * sah_bins), sah_bins - 1);
            if (b_idx < best_split)
            {
                std::swap(indices[i], indices[mid]);
                ++mid;
            }
        }

        // Fallback to median if partition is degenerate
        if (mid == start || mid == end)
        {
            mid = (start + end) / 2;
            auto cmp = [this, best_axis](std::uint32_t a, std::uint32_t b)
            { return centroids[a][best_axis] < centroids[b][best_axis]; };

            std::ranges::nth_element(indices.begin() + static_cast<std::ptrdiff_t>(start),
                                     indices.begin() + static_cast<std::ptrdiff_t>(mid),
                                     indices.begin() + static_cast<std::ptrdiff_t>(end), cmp);

            best_left_bounds = bounds[indices[start]];
            for (auto i = start + 1; i < mid; ++i)
                best_left_bounds = aabb_union(best_left_bounds, bounds[indices[i]]);
            best_right_bounds = bounds[indices[mid]];
            for (auto i = mid + 1; i < end; ++i)
                best_right_bounds = aabb_union(best_right_bounds, bounds[indices[i]]);
        }

        nodes[node_idx].bounds = cur_bound;
        nodes[node_idx].count = 0; // internal node
        nodes[node_idx].axis = static_cast<std::uint16_t>(best_axis);

        // Left child is always node_idx + 1 (DFS order)
        build_recursive(start, mid, best_left_bounds);

        // Right child: record its index
        nodes[node_idx].start = static_cast<std::uint32_t>(nodes.size());
        build_recursive(mid, end, best_right_bounds);
    }

    std::span<const aabb> bounds;
    std::span<const vec3<m>> centroids;
    std::vector<static_bvh::node> &nodes; // NOLINT
    std::vector<std::size_t> &indices;    // NOLINT
};

std::vector<std::size_t> static_bvh::build(std::span<const aabb> bounds,
                                           std::span<const vec3<m>> centroids)
{
    std::vector<std::size_t> indices(bounds.size());
    builder b(bounds, centroids, M_nodes, indices);
    return indices;
}

// -----------------------------
// -------- Dynamic BVH --------
// -----------------------------

dynamic_bvh::node_handle dynamic_bvh::insert_leaf(dynamic_bvh::node_handle leaf_idx)
{
    constexpr static auto cost_traversal = 2.0 / pow<2>(m);
    constexpr static auto cost_make = 2.0 / pow<2>(m);

    if (M_root == node::null)
    {
        M_root = leaf_idx;
        M_nodes[leaf_idx].parent = node::null;
        return leaf_idx;
    }

    // Find the best sibling for the new leaf
    aabb leaf_bounds = M_nodes[leaf_idx].bounds;
    auto cur_idx = M_root;
    while (!M_nodes[cur_idx].is_leaf())
    {
        auto [left, right] = M_nodes[cur_idx].children;

        auto area_cur = M_nodes[cur_idx].bounds.surface_area();
        auto new_bounds = aabb_union(M_nodes[cur_idx].bounds, leaf_bounds);
        auto area_new = new_bounds.surface_area();

        auto cost_new_sibling = cost_make * area_new;
        auto inherited_cost = cost_traversal * (area_new - area_cur);

        quantity<one> cost_left; // NOLINT
        if (M_nodes[left].is_leaf())
            cost_left =
                cost_traversal * aabb_union(leaf_bounds, M_nodes[left].bounds).surface_area() +
                inherited_cost;
        else
            cost_left =
                cost_traversal * (aabb_union(leaf_bounds, M_nodes[left].bounds).surface_area() -
                                  M_nodes[left].bounds.surface_area()) +
                inherited_cost;

        quantity<one> cost_right; // NOLINT
        if (M_nodes[right].is_leaf())
            cost_right =
                cost_traversal * aabb_union(leaf_bounds, M_nodes[right].bounds).surface_area() +
                inherited_cost;
        else
            cost_right =
                cost_traversal * (aabb_union(leaf_bounds, M_nodes[right].bounds).surface_area() -
                                  M_nodes[right].bounds.surface_area()) +
                inherited_cost;

        if (cost_new_sibling < cost_left && cost_new_sibling < cost_right) break;

        cur_idx = (cost_left < cost_right) ? left : right;
    }

    auto sibling_idx = cur_idx;
    auto old_parent_idx = M_nodes[sibling_idx].parent;
    auto new_parent_idx = allocate_node();
    auto &sibling = M_nodes[sibling_idx];

    auto &new_parent = M_nodes[new_parent_idx];
    new_parent.bounds = aabb_union(leaf_bounds, sibling.bounds);
    new_parent.parent = old_parent_idx;
    new_parent.children.left = sibling_idx;
    new_parent.children.right = leaf_idx;
    new_parent.height = sibling.height + 1;

    sibling.parent = new_parent_idx;
    M_nodes[leaf_idx].parent = new_parent_idx;

    if (old_parent_idx != node::null)
    {
        if (M_nodes[old_parent_idx].children.left == sibling_idx)
            M_nodes[old_parent_idx].children.left = new_parent_idx;
        else
            M_nodes[old_parent_idx].children.right = new_parent_idx;
    }
    else
        M_root = new_parent_idx;

    return leaf_idx;
}

void dynamic_bvh::extract_leaf(dynamic_bvh::node_handle leaf_idx)
{
    assert(leaf_idx < M_nodes.size() && M_nodes[leaf_idx].is_leaf());

    if (leaf_idx == M_root)
    {
        M_root = node::null;
        return;
    }

    auto &leaf = M_nodes[leaf_idx];
    auto parent_idx = leaf.parent;
    auto &parent = M_nodes[parent_idx];
    auto grandparent_idx = parent.parent;
    auto sibling_idx =
        (parent.children.left == leaf_idx) ? parent.children.right : parent.children.left;

    if (grandparent_idx != node::null)
    {
        if (M_nodes[grandparent_idx].children.left == parent_idx)
            M_nodes[grandparent_idx].children.left = sibling_idx;
        else
            M_nodes[grandparent_idx].children.right = sibling_idx;
        M_nodes[sibling_idx].parent = grandparent_idx;
        free_node(parent_idx);
        refit_and_rotate(grandparent_idx);
    }
    else
    {
        M_root = sibling_idx;
        M_nodes[sibling_idx].parent = node::null;
        free_node(parent_idx);
    }

    leaf.parent = node::null;
}

void dynamic_bvh::refit_and_rotate(dynamic_bvh::node_handle node_idx)
{
    while (node_idx != node::null)
    {
        node_idx = balance(node_idx);

        auto &n = M_nodes[node_idx];

        auto &left = M_nodes[n.children.left];
        auto &right = M_nodes[n.children.right];
        n.height = 1 + std::max(left.height, right.height);
        n.bounds = aabb_union(left.bounds, right.bounds);

        node_idx = n.parent;
    }
}

dynamic_bvh::node_handle dynamic_bvh::balance(dynamic_bvh::node_handle node_idx)
{
    auto &i_a = node_idx;

    auto &a = M_nodes[i_a];
    if (a.is_leaf() || a.height < 2) return i_a;

    auto [i_b, i_c] = a.children;
    auto &b = M_nodes[i_b];
    auto &c = M_nodes[i_c];

    auto balance = static_cast<int>(c.height) - static_cast<int>(b.height);
    // is C higher by at least 2
    if (balance > 1)
    {
        auto [i_f, i_g] = c.children;
        auto &f = M_nodes[i_f];
        auto &g = M_nodes[i_g];

        // swap A and C, C becomes new root
        c.children.left = i_a;
        c.parent = a.parent;
        a.parent = i_c;

        // make A's old parent point to C
        if (c.parent != node::null)
        {
            if (M_nodes[c.parent].children.left == i_a)
                M_nodes[c.parent].children.left = i_c;
            else
                M_nodes[c.parent].children.right = i_c;
        }
        else
            M_root = i_c;

        // Child of C (either F or G) with larger height becomes child of C, shorter one becomes
        // child of A
        if (f.height > g.height)
        {
            c.children.right = i_f;
            a.children.right = i_g;
            g.parent = i_a;
        }
        else
        {
            c.children.right = i_g;
            a.children.right = i_f;
            f.parent = i_a;
        }

        a.bounds = aabb_union(b.bounds, M_nodes[a.children.right].bounds);
        a.height = 1 + std::max(M_nodes[a.children.left].height, M_nodes[a.children.right].height);

        return i_c;
    }
    if (balance < -1)
    {
        auto [i_d, i_e] = b.children;
        auto &d = M_nodes[i_d];
        auto &e = M_nodes[i_e];

        // swap A and B, B becomes new root
        b.children.left = i_a;
        b.parent = a.parent;
        a.parent = i_b;

        // make A's old parent point to B
        if (b.parent != node::null)
        {
            if (M_nodes[b.parent].children.left == i_a)
                M_nodes[b.parent].children.left = i_b;
            else
                M_nodes[b.parent].children.right = i_b;
        }
        else
            M_root = i_b;

        // Child of B (either D or E) with larger height becomes child of B, shorter one becomes
        // child of A
        if (d.height > e.height)
        {
            b.children.right = i_d;
            a.children.left = i_e;
            e.parent = i_a;
        }
        else
        {
            b.children.right = i_e;
            a.children.left = i_d;
            d.parent = i_a;
        }

        a.bounds = aabb_union(c.bounds, M_nodes[a.children.left].bounds);
        a.height = 1 + std::max(M_nodes[a.children.left].height, M_nodes[a.children.right].height);

        return i_b;
    }

    return i_a;
}

bool dynamic_bvh::update_leaf(dynamic_bvh::node_handle leaf_idx, const aabb &true_bounds,
                              const vec3<si::metre> &displacement)
{
    assert(!M_nodes[leaf_idx].is_free() && "BVH ERROR: Attempted to update a freed leaf!");
    assert(M_nodes[leaf_idx].is_leaf() && "BVH ERROR: Attempted to update an internal node!");

    auto &fat_bounds = M_nodes[leaf_idx].bounds;

    if (fat_bounds.contains(true_bounds)) return false;

    extract_leaf(leaf_idx);

    auto margin = .1 * m; // TODO: tune this
    auto margin_vec = vec3{margin, margin, margin};

    aabb new_bounds{.min = true_bounds.min - margin_vec, .max = true_bounds.max + margin_vec};

    // predict future movement
    if (displacement.x() < 0.0 * m)
        new_bounds.min.x(new_bounds.min.x() + displacement.x());
    else
        new_bounds.max.x(new_bounds.max.x() + displacement.x());

    if (displacement.y() < 0.0 * m)
        new_bounds.min.y(new_bounds.min.y() + displacement.y());
    else
        new_bounds.max.y(new_bounds.max.y() + displacement.y());

    if (displacement.z() < 0.0 * m)
        new_bounds.min.z(new_bounds.min.z() + displacement.z());
    else
        new_bounds.max.z(new_bounds.max.z() + displacement.z());

    M_nodes[leaf_idx].bounds = new_bounds;

    insert_leaf(leaf_idx);
    refit_and_rotate(M_nodes[leaf_idx].parent);

    return true;
}

} // namespace physkit::detail