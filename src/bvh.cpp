#include "physkit/detail/bvh.h"

namespace physkit::detail
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

constexpr unsigned int sah_bins = 16;
constexpr unsigned int leaf_max_tris = 4;

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

        // Make leaf if few enough triangles
        if (count <= leaf_max_tris)
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

} // namespace physkit::detail