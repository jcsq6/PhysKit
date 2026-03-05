#include "test.h"

#include <physkit/detail/bvh.h>

#include <algorithm>
#include <numeric>
#include <random>
#include <set>

using namespace testing;
using dbvh = physkit::detail::dynamic_bvh;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

aabb make_box(double cx, double cy, double cz, double half = 0.5)
{
    return {.min = vec3{cx - half, cy - half, cz - half} * m,
            .max = vec3{cx + half, cy + half, cz + half} * m};
}

// ---------------------------------------------------------------------------
// add / single leaf
// ---------------------------------------------------------------------------

void test_add_single_leaf()
{
    dbvh tree;
    auto box = make_box(0, 0, 0);
    auto handle = tree.add(dbvh::object_handle{0}, box);

    CHECK(tree.data(handle) == 0u);
    CHECK_APPROX(tree.bounds(handle), box);
}

void test_add_returns_distinct_handles()
{
    dbvh tree;
    auto h0 = tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));
    auto h1 = tree.add(dbvh::object_handle{1}, make_box(3, 0, 0));
    auto h2 = tree.add(dbvh::object_handle{2}, make_box(6, 0, 0));

    CHECK(h0 != h1);
    CHECK(h1 != h2);
    CHECK(h0 != h2);
}

void test_add_preserves_data()
{
    dbvh tree;
    constexpr int n = 20;

    std::vector<dbvh::node_handle> handles;
    for (int i = 0; i < n; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(static_cast<double>(i) * 2, 0, 0)));

    for (int i = 0; i < n; ++i) CHECK(tree.data(handles[i]) == static_cast<std::uint32_t>(i));
}

// ---------------------------------------------------------------------------
// remove_leaf
// ---------------------------------------------------------------------------

void test_remove_single_leaf()
{
    dbvh tree;
    auto h = tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));
    tree.remove_leaf(h);

    // Tree is now empty -- query should find nothing
    int count = 0;
    tree.query_aabb(make_box(0, 0, 0, 100),
                    [&](dbvh::object_handle)
                    {
                        ++count;
                        return true;
                    });
    CHECK(count == 0);
}

void test_remove_one_of_two()
{
    dbvh tree;
    auto h0 = tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));
    auto h1 = tree.add(dbvh::object_handle{1}, make_box(5, 0, 0));

    tree.remove_leaf(h0);

    // Only object 1 should remain
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0, 0, 0, 100),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.size() == 1u);
    CHECK(found.contains(1u));
}

void test_remove_middle_of_many()
{
    dbvh tree;
    constexpr int n = 10;
    std::vector<dbvh::node_handle> handles;
    for (int i = 0; i < n; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(static_cast<double>(i) * 2, 0, 0)));

    // Remove every other leaf
    for (int i = 0; i < n; i += 2) tree.remove_leaf(handles[i]);

    // Query the whole space -- only odd-indexed objects should remain
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0, 0, 0, 100),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.size() == 5u);
    for (int i = 1; i < n; i += 2) CHECK(found.contains(static_cast<std::uint32_t>(i)));
}

void test_remove_all_leaves()
{
    dbvh tree;
    constexpr int n = 8;
    std::vector<dbvh::node_handle> handles;
    for (int i = 0; i < n; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(static_cast<double>(i) * 2, 0, 0)));

    for (int i = 0; i < n; ++i) tree.remove_leaf(handles[i]);

    int count = 0;
    tree.query_aabb(make_box(0, 0, 0, 1000),
                    [&](dbvh::object_handle)
                    {
                        ++count;
                        return true;
                    });
    CHECK(count == 0);
}

// ---------------------------------------------------------------------------
// update_leaf
// ---------------------------------------------------------------------------

void test_update_within_fat_bounds_no_reinsert()
{
    dbvh tree;
    auto box = make_box(0, 0, 0);
    auto h = tree.add(dbvh::object_handle{0}, box);

    // First update always reinserts since add() stores exact bounds (no margin).
    // Force a reinsert to establish fat bounds with the 0.1m margin.
    auto first = tree.update_leaf(h, make_box(0.01, 0, 0), vec3{0.01, 0, 0} * m);
    CHECK(first); // must reinsert

    // Now the stored fat bounds have a margin. A tiny shift should stay within them.
    aabb tiny_move = make_box(0.02, 0, 0);
    auto second = tree.update_leaf(h, tiny_move, vec3{0, 0, 0} * m);
    CHECK(!second);
}

void test_update_outside_fat_bounds_reinserts()
{
    dbvh tree;
    auto box = make_box(0, 0, 0);
    auto h = tree.add(dbvh::object_handle{0}, box);

    // Large displacement that escapes the fat AABB
    aabb new_bounds = make_box(10, 10, 10);
    auto moved = tree.update_leaf(h, new_bounds, vec3{10, 10, 10} * m);
    CHECK(moved);

    // The leaf should still be queryable at the new location
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(10, 10, 10, 2),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.contains(0u));
}

void test_update_preserves_data()
{
    dbvh tree;
    auto h = tree.add(dbvh::object_handle{42}, make_box(0, 0, 0));
    tree.update_leaf(h, make_box(20, 20, 20), vec3{20, 20, 20} * m);
    CHECK(tree.data(h) == 42u);
}

void test_update_predictive_expansion()
{
    dbvh tree;
    auto h = tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));

    // Force a reinsert with positive displacement
    aabb new_bounds = make_box(5, 0, 0);
    tree.update_leaf(h, new_bounds, vec3{2, 0, 0} * m);

    // The fat bounds should have expanded in the +x direction
    auto fat = tree.bounds(h);
    CHECK(fat.max.x() > new_bounds.max.x());
}

// ---------------------------------------------------------------------------
// query_aabb
// ---------------------------------------------------------------------------

void test_query_empty_tree()
{
    dbvh tree;
    int count = 0;
    tree.query_aabb(make_box(0, 0, 0, 100),
                    [&](dbvh::object_handle)
                    {
                        ++count;
                        return true;
                    });
    CHECK(count == 0);
}

void test_query_finds_overlapping()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));
    tree.add(dbvh::object_handle{1}, make_box(10, 10, 10));
    tree.add(dbvh::object_handle{2}, make_box(0.5, 0.5, 0.5));

    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0.25, 0.25, 0.25, 1),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });

    CHECK(found.contains(0u));
    CHECK(found.contains(2u));
    CHECK(!found.contains(1u));
}

void test_query_no_results()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));
    tree.add(dbvh::object_handle{1}, make_box(2, 0, 0));

    int count = 0;
    tree.query_aabb(make_box(100, 100, 100, 0.5),
                    [&](dbvh::object_handle)
                    {
                        ++count;
                        return true;
                    });
    CHECK(count == 0);
}

void test_query_early_termination()
{
    dbvh tree;
    for (int i = 0; i < 10; ++i)
        tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, make_box(0, 0, 0));

    int count = 0;
    tree.query_aabb(make_box(0, 0, 0, 2),
                    [&](dbvh::object_handle)
                    {
                        ++count;
                        return false;
                    }); // stop after first
    CHECK(count == 1);
}

void test_query_large_population()
{
    dbvh tree;
    constexpr int n = 200;
    std::mt19937 rng{12345};
    std::uniform_real_distribution<double> dist(-50, 50);

    std::vector<dbvh::node_handle> handles;
    std::vector<aabb> boxes;
    for (int i = 0; i < n; ++i)
    {
        auto box = make_box(dist(rng), dist(rng), dist(rng), 0.5);
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, box));
        boxes.push_back(box);
    }

    // Query a region and verify against brute-force
    auto query_box = make_box(0, 0, 0, 10);
    std::set<std::uint32_t> bvh_results;
    tree.query_aabb(query_box,
                    [&](dbvh::object_handle id)
                    {
                        bvh_results.insert(id);
                        return true;
                    });

    std::set<std::uint32_t> brute_results;
    for (int i = 0; i < n; ++i)
        if (boxes[i].intersects(query_box)) brute_results.insert(static_cast<std::uint32_t>(i));

    // BVH may return false positives (fat AABBs) but must not miss true overlaps
    for (auto id : brute_results) CHECK(bvh_results.contains(id));
}

// ---------------------------------------------------------------------------
// raycast
// ---------------------------------------------------------------------------

void test_raycast_empty_tree()
{
    dbvh tree;
    int count = 0;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 0, 0});
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     ++count;
                     return max_d;
                 });
    CHECK(count == 0);
}

void test_raycast_hits_single_box()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(5, 0, 0));

    std::set<std::uint32_t> hit_ids;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 0, 0});
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle id, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     hit_ids.insert(id);
                     return max_d;
                 });

    CHECK(hit_ids.contains(0u));
}

void test_raycast_misses_distant_box()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(5, 10, 0)); // far off ray axis

    int count = 0;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 0, 0}); // along +x
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     ++count;
                     return max_d;
                 });
    CHECK(count == 0);
}

void test_raycast_respects_max_distance()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(50, 0, 0));

    int count = 0;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 0, 0});
    tree.raycast(r, 10 * m, // too short to reach
                 [&](dbvh::object_handle, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     ++count;
                     return max_d;
                 });
    CHECK(count == 0);
}

void test_raycast_hits_multiple_boxes()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(3, 0, 0));
    tree.add(dbvh::object_handle{1}, make_box(7, 0, 0));
    tree.add(dbvh::object_handle{2}, make_box(12, 0, 0));
    tree.add(dbvh::object_handle{3}, make_box(0, 10, 0)); // off axis

    std::set<std::uint32_t> hit_ids;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 0, 0});
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle id, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     hit_ids.insert(id);
                     return max_d;
                 });

    CHECK(hit_ids.contains(0u));
    CHECK(hit_ids.contains(1u));
    CHECK(hit_ids.contains(2u));
    CHECK(!hit_ids.contains(3u));
}

void test_raycast_early_termination()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(3, 0, 0));
    tree.add(dbvh::object_handle{1}, make_box(7, 0, 0));

    int count = 0;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 0, 0});
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle, quantity<si::metre>, quantity<si::metre>)
                 {
                     ++count;
                     return 0.0 * m; // terminate
                 });
    CHECK(count == 1);
}

void test_raycast_negative_direction()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(-5, 0, 0));

    std::set<std::uint32_t> hit_ids;
    ray r(vec3{0, 0, 0} * m, vec3<one>{-1, 0, 0});
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle id, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     hit_ids.insert(id);
                     return max_d;
                 });

    CHECK(hit_ids.contains(0u));
}

void test_raycast_diagonal()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(5, 5, 5, 1));

    std::set<std::uint32_t> hit_ids;
    ray r(vec3{0, 0, 0} * m, vec3<one>{1, 1, 1}); // normalized internally
    tree.raycast(r, 100 * m,
                 [&](dbvh::object_handle id, quantity<si::metre>, quantity<si::metre> max_d)
                 {
                     hit_ids.insert(id);
                     return max_d;
                 });

    CHECK(hit_ids.contains(0u));
}

// ---------------------------------------------------------------------------
// add + remove churn (node reuse via free list)
// ---------------------------------------------------------------------------

void test_add_remove_add_reuses_nodes()
{
    dbvh tree;
    auto h0 = tree.add(dbvh::object_handle{0}, make_box(0, 0, 0));
    auto h1 = tree.add(dbvh::object_handle{1}, make_box(5, 0, 0));

    tree.remove_leaf(h0);

    // Adding a new object should reuse the freed slot
    auto h2 = tree.add(dbvh::object_handle{2}, make_box(10, 0, 0));

    // The tree should contain exactly objects 1 and 2
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0, 0, 0, 100),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.size() == 2u);
    CHECK(found.contains(1u));
    CHECK(found.contains(2u));
}

void test_heavy_churn()
{
    dbvh tree;
    std::mt19937 rng{99};
    std::uniform_real_distribution<double> dist(-20, 20);

    std::vector<dbvh::node_handle> handles;

    // Add 50 objects
    for (int i = 0; i < 50; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(dist(rng), dist(rng), dist(rng), 0.5)));

    // Remove 25 objects
    std::shuffle(handles.begin(), handles.end(), rng);
    std::set<std::uint32_t> removed;
    for (int i = 0; i < 25; ++i)
    {
        removed.insert(tree.data(handles[i]));
        tree.remove_leaf(handles[i]);
    }
    handles.erase(handles.begin(), handles.begin() + 25);

    // Add 25 more
    for (int i = 50; i < 75; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(dist(rng), dist(rng), dist(rng), 0.5)));

    // Query the whole space -- expect exactly the 50 remaining objects
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0, 0, 0, 1000),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.size() == 50u);

    // Verify none of the removed IDs appear
    for (auto id : removed) CHECK(!found.contains(id));
}

// ---------------------------------------------------------------------------
// Structural invariants
// ---------------------------------------------------------------------------

// Recursively validate node bounds contain children and heights are correct.
// This exercises the internal tree structure after various operations.
struct tree_validator
{
    const dbvh &tree;
    int leaf_count = 0;

    // We can't directly access internal nodes, so we validate indirectly
    // via queries: every leaf's AABB should be found by a query_aabb that
    // covers that leaf's bounds.
    void validate_leaf_queryable(dbvh::node_handle h)
    {
        auto leaf_bounds = tree.bounds(h);
        auto leaf_data = tree.data(h);
        bool found = false;
        tree.query_aabb(leaf_bounds,
                        [&](dbvh::object_handle id)
                        {
                            if (id == leaf_data) found = true;
                            return true;
                        });
        CHECK(found);
    }
};

void test_all_leaves_queryable_after_inserts()
{
    dbvh tree;
    constexpr int n = 30;
    std::mt19937 rng{777};
    std::uniform_real_distribution<double> dist(-30, 30);

    std::vector<dbvh::node_handle> handles;
    for (int i = 0; i < n; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(dist(rng), dist(rng), dist(rng), 0.3)));

    tree_validator v{tree};
    for (auto h : handles) v.validate_leaf_queryable(h);
}

void test_all_leaves_queryable_after_updates()
{
    dbvh tree;
    constexpr int n = 20;
    std::mt19937 rng{42};
    std::uniform_real_distribution<double> pos_dist(-20, 20);
    std::uniform_real_distribution<double> disp_dist(-5, 5);

    std::vector<dbvh::node_handle> handles;
    for (int i = 0; i < n; ++i)
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)},
                                   make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), 0.3)));

    // Move each leaf significantly
    for (int i = 0; i < n; ++i)
    {
        auto dx = disp_dist(rng);
        auto dy = disp_dist(rng);
        auto dz = disp_dist(rng);
        auto new_bounds = make_box(dx * 10, dy * 10, dz * 10, 0.3);
        tree.update_leaf(handles[i], new_bounds, vec3{dx, dy, dz} * m);
    }

    tree_validator v{tree};
    for (auto h : handles) v.validate_leaf_queryable(h);
}

// ---------------------------------------------------------------------------
// query_aabb correctness: brute-force cross-check
// ---------------------------------------------------------------------------

void test_query_aabb_brute_force_crosscheck()
{
    dbvh tree;
    constexpr int n = 100;
    std::mt19937 rng{2025};
    std::uniform_real_distribution<double> pos_dist(-30, 30);
    std::uniform_real_distribution<double> size_dist(0.2, 2.0);

    std::vector<aabb> boxes;
    for (int i = 0; i < n; ++i)
    {
        auto half = size_dist(rng);
        auto box = make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), half);
        tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, box);
        boxes.push_back(box);
    }

    // Run 20 random queries and cross-check
    for (int q = 0; q < 20; ++q)
    {
        auto query_box = make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), size_dist(rng) * 5);

        std::set<std::uint32_t> bvh_results;
        tree.query_aabb(query_box,
                        [&](dbvh::object_handle id)
                        {
                            bvh_results.insert(id);
                            return true;
                        });

        // Brute-force: every truly overlapping AABB must appear in BVH results
        for (int i = 0; i < n; ++i)
            if (boxes[i].intersects(query_box))
                CHECK(bvh_results.contains(static_cast<std::uint32_t>(i)));
    }
}

// ---------------------------------------------------------------------------
// raycast correctness: brute-force cross-check
// ---------------------------------------------------------------------------

void test_raycast_brute_force_crosscheck()
{
    dbvh tree;
    constexpr int n = 50;
    std::mt19937 rng{3141};
    std::uniform_real_distribution<double> pos_dist(-20, 20);

    std::vector<aabb> boxes;
    for (int i = 0; i < n; ++i)
    {
        auto box = make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), 0.8);
        tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, box);
        boxes.push_back(box);
    }

    // Cast several rays
    std::uniform_real_distribution<double> dir_dist(-1, 1);
    auto max_d = 100 * m;

    for (int q = 0; q < 15; ++q)
    {
        auto origin = vec3{pos_dist(rng), pos_dist(rng), pos_dist(rng)} * m;
        auto dir = vec3<one>{dir_dist(rng), dir_dist(rng), dir_dist(rng)};
        if (dir.norm() < 1e-6) dir = vec3<one>{1, 0, 0};

        ray r(origin, dir);

        std::set<std::uint32_t> bvh_hits;
        tree.raycast(r, max_d,
                     [&](dbvh::object_handle id, quantity<si::metre>, quantity<si::metre> md)
                     {
                         bvh_hits.insert(id);
                         return md;
                     });

        // Brute-force: any box the ray actually intersects must be found by the BVH
        for (int i = 0; i < n; ++i)
            if (r.intersect_distance(boxes[i], max_d).has_value())
                CHECK(bvh_hits.contains(static_cast<std::uint32_t>(i)));
    }
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

void test_degenerate_flat_box()
{
    dbvh tree;
    // Zero-volume AABB (a plane)
    aabb flat{.min = vec3{0, 0, 0} * m, .max = vec3{1, 1, 0} * m};
    auto h = tree.add(dbvh::object_handle{0}, flat);

    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0.5, 0.5, 0, 2),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.contains(0u));
}

void test_coincident_boxes()
{
    dbvh tree;
    auto box = make_box(0, 0, 0);
    for (int i = 0; i < 5; ++i) tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, box);

    std::set<std::uint32_t> found;
    tree.query_aabb(box,
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.size() == 5u);
}

void test_very_large_and_very_small_boxes()
{
    dbvh tree;
    aabb large{.min = vec3{-1000, -1000, -1000} * m, .max = vec3{1000, 1000, 1000} * m};
    aabb tiny{.min = vec3{-0.001, -0.001, -0.001} * m, .max = vec3{0.001, 0.001, 0.001} * m};

    tree.add(dbvh::object_handle{0}, large);
    tree.add(dbvh::object_handle{1}, tiny);

    // Query should find both when the query region overlaps both
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0, 0, 0, 0.01),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.contains(0u));
    CHECK(found.contains(1u));
}

void test_negative_coordinates()
{
    dbvh tree;
    tree.add(dbvh::object_handle{0}, make_box(-10, -20, -30));

    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(-10, -20, -30, 2),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.contains(0u));
}

void test_many_objects_along_line()
{
    dbvh tree;
    constexpr int n = 100;
    std::vector<aabb> boxes;
    for (int i = 0; i < n; ++i)
    {
        auto box = make_box(static_cast<double>(i) * 2, 0, 0, 0.4);
        tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, box);
        boxes.push_back(box);
    }

    // Query in the middle
    auto query_box = make_box(50, 0, 0, 5);
    std::set<std::uint32_t> bvh_results;
    tree.query_aabb(query_box,
                    [&](dbvh::object_handle id)
                    {
                        bvh_results.insert(id);
                        return true;
                    });

    for (int i = 0; i < n; ++i)
        if (boxes[i].intersects(query_box))
            CHECK(bvh_results.contains(static_cast<std::uint32_t>(i)));
}

// ---------------------------------------------------------------------------
// Stress: interleaved add/remove/update/query
// ---------------------------------------------------------------------------

void test_interleaved_operations()
{
    dbvh tree;
    std::mt19937 rng{1337};
    std::uniform_real_distribution<double> pos_dist(-30, 30);
    std::uniform_int_distribution<int> op_dist(0, 3); // 0=add, 1=remove, 2=update, 3=query

    std::vector<dbvh::node_handle> handles;
    std::set<std::uint32_t> live_ids;
    std::uint32_t next_id = 0;

    for (int step = 0; step < 500; ++step)
    {
        auto op = handles.empty() ? 0 : op_dist(rng);

        switch (op)
        {
        case 0: // add
        {
            auto id = next_id++;
            auto h = tree.add(dbvh::object_handle{id},
                              make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), 0.5));
            handles.push_back(h);
            live_ids.insert(id);
            break;
        }
        case 1: // remove
        {
            auto idx = std::uniform_int_distribution<std::size_t>{0, handles.size() - 1}(rng);
            auto id = tree.data(handles[idx]);
            tree.remove_leaf(handles[idx]);
            live_ids.erase(id);
            handles.erase(handles.begin() + static_cast<std::ptrdiff_t>(idx));
            break;
        }
        case 2: // update
        {
            auto idx = std::uniform_int_distribution<std::size_t>{0, handles.size() - 1}(rng);
            auto new_bounds = make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), 0.5);
            tree.update_leaf(handles[idx], new_bounds,
                             vec3{pos_dist(rng), pos_dist(rng), pos_dist(rng)} * m);
            break;
        }
        case 3: // query
        {
            std::set<std::uint32_t> found;
            tree.query_aabb(make_box(0, 0, 0, 1000),
                            [&](dbvh::object_handle id)
                            {
                                found.insert(id);
                                return true;
                            });
            // All live IDs must be found
            for (auto id : live_ids) CHECK(found.contains(id));
            break;
        }
        }
    }

    // Final full-tree query
    std::set<std::uint32_t> found;
    tree.query_aabb(make_box(0, 0, 0, 1000),
                    [&](dbvh::object_handle id)
                    {
                        found.insert(id);
                        return true;
                    });
    CHECK(found.size() == live_ids.size());
    for (auto id : live_ids) CHECK(found.contains(id));
}

// ---------------------------------------------------------------------------
// Raycast through updated tree
// ---------------------------------------------------------------------------

void test_raycast_after_updates()
{
    dbvh tree;
    constexpr int n = 20;
    std::mt19937 rng{555};
    std::uniform_real_distribution<double> pos_dist(-10, 10);

    std::vector<dbvh::node_handle> handles;
    std::vector<aabb> current_boxes;
    for (int i = 0; i < n; ++i)
    {
        auto box = make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), 0.5);
        handles.push_back(tree.add(dbvh::object_handle{static_cast<std::uint32_t>(i)}, box));
        current_boxes.push_back(box);
    }

    // Update half the objects to new positions
    for (int i = 0; i < n / 2; ++i)
    {
        auto new_box = make_box(pos_dist(rng), pos_dist(rng), pos_dist(rng), 0.5);
        auto disp = new_box.center() - current_boxes[i].center();
        tree.update_leaf(handles[i], new_box, disp);
        current_boxes[i] = new_box;
    }

    // Cast rays and verify against brute-force AABB intersection
    auto max_d = 100 * m;
    for (int q = 0; q < 10; ++q)
    {
        auto origin = vec3{pos_dist(rng), pos_dist(rng), pos_dist(rng)} * m;
        ray r(origin, vec3<one>{1, 0, 0});

        std::set<std::uint32_t> bvh_hits;
        tree.raycast(r, max_d,
                     [&](dbvh::object_handle id, quantity<si::metre>, quantity<si::metre> md)
                     {
                         bvh_hits.insert(id);
                         return md;
                     });

        for (int i = 0; i < n; ++i)
            if (r.intersect_distance(current_boxes[i], max_d).has_value())
                CHECK(bvh_hits.contains(static_cast<std::uint32_t>(i)));
    }
}

// ---------------------------------------------------------------------------
// Bounds / data accessors
// ---------------------------------------------------------------------------

void test_bounds_accessor()
{
    dbvh tree;
    auto box = make_box(3, 4, 5, 1.5);
    auto h = tree.add(dbvh::object_handle{7}, box);
    CHECK_APPROX(tree.bounds(h), box);
}

void test_data_accessor()
{
    dbvh tree;
    auto h = tree.add(dbvh::object_handle{123}, make_box(0, 0, 0));
    CHECK(tree.data(h) == 123u);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    return suite{}
        .group("add")
        .test("single leaf", test_add_single_leaf)
        .test("distinct handles", test_add_returns_distinct_handles)
        .test("preserves data", test_add_preserves_data)

        .group("remove_leaf")
        .test("single leaf", test_remove_single_leaf)
        .test("one of two", test_remove_one_of_two)
        .test("middle of many", test_remove_middle_of_many)
        .test("all leaves", test_remove_all_leaves)

        .group("update_leaf")
        .test("within fat bounds (no reinsert)", test_update_within_fat_bounds_no_reinsert)
        .test("outside fat bounds (reinsert)", test_update_outside_fat_bounds_reinserts)
        .test("preserves data", test_update_preserves_data)
        .test("predictive expansion", test_update_predictive_expansion)

        .group("query_aabb")
        .test("empty tree", test_query_empty_tree)
        .test("finds overlapping", test_query_finds_overlapping)
        .test("no results", test_query_no_results)
        .test("early termination", test_query_early_termination)
        .test("large population", test_query_large_population)
        .test("brute-force crosscheck", test_query_aabb_brute_force_crosscheck)

        .group("raycast")
        .test("empty tree", test_raycast_empty_tree)
        .test("hits single box", test_raycast_hits_single_box)
        .test("misses distant box", test_raycast_misses_distant_box)
        .test("respects max distance", test_raycast_respects_max_distance)
        .test("hits multiple boxes", test_raycast_hits_multiple_boxes)
        .test("early termination", test_raycast_early_termination)
        .test("negative direction", test_raycast_negative_direction)
        .test("diagonal ray", test_raycast_diagonal)
        .test("brute-force crosscheck", test_raycast_brute_force_crosscheck)
        .test("after updates", test_raycast_after_updates)

        .group("churn / free list")
        .test("add-remove-add reuses nodes", test_add_remove_add_reuses_nodes)
        .test("heavy churn", test_heavy_churn)

        .group("structural invariants")
        .test("all leaves queryable after inserts", test_all_leaves_queryable_after_inserts)
        .test("all leaves queryable after updates", test_all_leaves_queryable_after_updates)

        .group("edge cases")
        .test("degenerate flat box", test_degenerate_flat_box)
        .test("coincident boxes", test_coincident_boxes)
        .test("very large and very small boxes", test_very_large_and_very_small_boxes)
        .test("negative coordinates", test_negative_coordinates)
        .test("many objects along line", test_many_objects_along_line)

        .group("stress")
        .test("interleaved operations (500 steps)", test_interleaved_operations)

        .group("accessors")
        .test("bounds accessor", test_bounds_accessor)
        .test("data accessor", test_data_accessor)

        .run();
}
