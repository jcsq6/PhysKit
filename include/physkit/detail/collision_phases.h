#include "bvh.h"
#include "physkit/collision.h"
#include "physkit/object.h"

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>

#include <array>

namespace physkit::detail
{
class pair_manager
{
public:
    pair_manager(std::size_t initial_capacity = 8192) { M_active.reserve(initial_capacity); }

    bool try_add_pair(dynamic_bvh::object_handle a, dynamic_bvh::object_handle b)
    {
        return M_active.insert(make_pair_key(a, b)).second;
    }
    void remove_pair(auto it) { M_active.erase(it); }

    void remove_for_object(dynamic_bvh::object_handle id,
                           std::regular_invocable<std::uint64_t> auto &&on_pair_removed)
    {
        for (auto it = M_active.begin(); it != M_active.end();)
        {
            auto [a, b] = extract_ids(*it);
            if (a == id || b == id)
            {
                on_pair_removed(*it);
                M_active.erase(it++);
            }
            else
                ++it;
        }
    }

    [[nodiscard]] auto &active_pairs() const { return M_active; }

    static std::pair<dynamic_bvh::object_handle, dynamic_bvh::object_handle>
    extract_ids(std::uint64_t key)
    {
        return {{static_cast<std::uint32_t>(key >> 32)},
                {static_cast<std::uint32_t>(key & 0xFFFFFFFF)}};
    }

    static constexpr std::uint64_t make_pair_key(dynamic_bvh::object_handle a,
                                                 dynamic_bvh::object_handle b)
    {
        auto min_id = std::min(a, b);
        auto max_id = std::max(a, b);
        return (static_cast<std::uint64_t>(min_id) << 32) | max_id;
    }

private:
    absl::flat_hash_set<std::uint64_t> M_active;
};

class manifold // TODO: time of impact for continuous collision detection
{
public:
    struct contact_info
    {
        collision_info info;

        quantity<si::newton * si::second> normal_impulse{};
        quantity<si::newton * si::second> tangent_impulse{};
    };

    static constexpr std::size_t max_contact_points = 4;

    manifold() = default;
    template <std::ranges::range R>
        requires(std::same_as<std::ranges::range_value_t<R>, contact_info>)
    manifold(const R &contacts) : M_contacts{std::ranges::size(contacts)}
    {
        assert(std::ranges::size(contacts) <= max_contact_points);
        std::ranges::copy(contacts, M_contact_buffer.begin());
    }

    [[nodiscard]] auto contacts(this auto &&self)
    {
        return std::span{self.M_contact_buffer.data(), self.M_contacts};
    }

    template <std::ranges::range R>
        requires(std::same_as<std::ranges::range_value_t<R>, contact_info>)
    void contacts(const R &contacts)
    {
        assert(std::ranges::size(contacts) <= max_contact_points);
        M_contacts = contacts.size();
        std::ranges::copy(contacts, M_contact_buffer.begin());
    }

    void add_contact(const contact_info &info)
    {
        if (M_contacts < max_contact_points)
            M_contact_buffer[M_contacts++] = info;
        else
            add_reduce(info);
    }

    [[nodiscard]] bool empty() const { return M_contacts == 0; }
    [[nodiscard]] operator bool() const { return !empty(); }

private:
    std::array<contact_info, max_contact_points> M_contact_buffer{};
    std::size_t M_contacts{};

    void add_reduce(const contact_info &new_pt)
    {
        static_assert(max_contact_points == 4, "add_reduce logic assumes max_contact_points is 4");

        std::array<contact_info, 5> pool;
        std::ranges::copy(contacts(), pool.begin());
        pool[5] = new_pt;

        std::array<std::size_t, 4> best = {0, 1, 2, 3};

        // find deepest point
        for (std::size_t i = 1; i < pool.size(); ++i)
            if (pool[i].info.depth > pool[best[0]].info.depth) best[0] = i;

        // point furthest from best[0]
        auto max_dist2 = -1.0 * si::metre * si::metre;
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            if (i == best[0]) continue;
            if (auto dist2 = (pool[i].info.local_a - pool[best[0]].info.local_a).squared_norm();
                dist2 > max_dist2)
            {
                max_dist2 = dist2;
                best[1] = i;
            }
        }

        // Point maximizing area with best[0] and best[1]
        auto max_area2 = -1.0 * pow<4>(si::metre);
        auto edge0 = pool[best[1]].info.local_a - pool[best[0]].info.local_a;
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            if (i == best[0] || i == best[1]) continue;
            auto edge1 = pool[i].info.local_a - pool[best[0]].info.local_a;
            auto area2 = edge0.cross(edge1).squared_norm();
            if (area2 > max_area2)
            {
                max_area2 = area2;
                best[2] = i;
            }
        }

        // point furthest from best[2] (heuristic)
        max_dist2 = -1.0 * si::metre * si::metre;
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            if (i == best[0] || i == best[1] || i == best[2]) continue;

            auto dist2 = (pool[i].info.local_a - pool[best[2]].info.local_a).squared_norm();
            if (dist2 > max_dist2)
            {
                max_dist2 = dist2;
                best[3] = i;
            }
        }

        // copy back to manifold
        M_contacts = 0;
        for (auto i : best) M_contact_buffer[M_contacts++] = pool[i];
    }
};

class narrow_phase
{
public:
    struct manifold_info
    {
        dynamic_bvh::object_handle a{};
        dynamic_bvh::object_handle b{};

        manifold man;
    };

    static constexpr auto distance2_eps = (.005 * .005 * pow<2>(si::metre));
    static constexpr auto contact_breaking_threshold = .02 * si::metre;

    narrow_phase(std::size_t initial_capacity = 1024) : M_contact_map(initial_capacity)
    {
        M_manifolds.reserve(initial_capacity);
    }

    void on_pair_added(std::uint64_t key)
    {
        auto [a, b] = pair_manager::extract_ids(key);
        M_contact_map[key] = M_manifolds.size();
        M_manifolds.push_back({a, b});
    }

    void on_pair_removed(std::uint64_t key)
    {
        auto it = M_contact_map.find(key);
        if (it == M_contact_map.end()) return;

        auto idx = it->second;
        auto last_idx = M_manifolds.size() - 1;
        if (idx != last_idx)
        {
            M_manifolds[idx] = M_manifolds.back();
            std::uint64_t last_key =
                pair_manager::make_pair_key(M_manifolds[idx].a, M_manifolds[idx].b);
            M_contact_map[last_key] = idx;
        }

        M_manifolds.pop_back();
        M_contact_map.erase(it);
    }

    void calculate(std::regular_invocable<dynamic_bvh::object_handle> auto &&get_object)
        requires(std::same_as<
                 std::invoke_result_t<decltype(get_object), dynamic_bvh::object_handle>, object &>)
    {
        // TODO: parallelize
        for (auto &man : M_manifolds)
        {
            auto &obj_a = get_object(man.a);
            auto &obj_b = get_object(man.b);

            auto col_ret = gjk_epa(
                obj_a.instance(),
                obj_b.instance()); // TODO: replace with dispatcher with updated shape options
            if (!col_ret)
            {
                man.man = manifold{};
                continue;
            }

            auto new_contact = manifold::contact_info{.info = {*col_ret}};

            manifold new_man;

            for (auto &old_contact : man.man.contacts())
            {
                // warm start new point
                if ((new_contact.info.local_a - old_contact.info.local_a).squared_norm() <
                        distance2_eps &&
                    (new_contact.info.local_b - old_contact.info.local_b).squared_norm() <
                        distance2_eps)
                {
                    // transfer cached impulses to new point
                    new_contact.normal_impulse = old_contact.normal_impulse;
                    new_contact.tangent_impulse = old_contact.tangent_impulse;
                    continue;
                }

                auto world_old_a = obj_a.project_local(old_contact.info.local_a);
                auto world_old_b = obj_b.project_local(old_contact.info.local_b);

                // Note that world_old_a - world_old_b = old_contact.info.normal *
                // old_contact.info.depth (if normal points from b to a)
                auto relative = world_old_b - world_old_a; // If normal points from b to a
                auto depth = relative.dot(new_contact.info.normal);

                // how far did the points drift tangentially along contact plane
                auto projected_a = world_old_a - new_contact.info.normal * depth;
                auto drift2 = (projected_a - world_old_b).squared_norm();

                if (abs(depth) < contact_breaking_threshold && drift2 < distance2_eps)
                {
                    old_contact.info.depth = depth;
                    old_contact.info.normal = new_contact.info.normal;

                    new_man.add_contact(old_contact);
                }
            }

            new_man.add_contact(new_contact);
            man.man = new_man;
        }
    }

    [[nodiscard]] std::span<const manifold_info> manifolds() const { return M_manifolds; }

private:
    std::vector<manifold_info> M_manifolds;
    absl::flat_hash_map<std::uint64_t, std::size_t> M_contact_map;
};

class broad_phase
{
public:
    broad_phase(std::size_t static_capacity = 1024, std::size_t dynamic_capacity = 1024,
                std::size_t pair_capacity = 8192)
        : M_static_tree(static_capacity), M_dynamic_tree(dynamic_capacity),
          M_pair_manager(pair_capacity)
    {
        M_moved.reserve((static_capacity + dynamic_capacity) / 3);
    }

    auto add(dynamic_bvh::object_handle object_id, const aabb &bounds, bool is_static)
    {
        if (is_static) return M_static_tree.add(object_id, bounds);
        return M_dynamic_tree.add(object_id, bounds);
    }

    // TODO: Mark object as dead, remove from tree at end of frame after narrow phase, update trees
    // in separate thread
    void remove(dynamic_bvh::node_handle id, bool is_static, narrow_phase &narrow)
    {
        dynamic_bvh::object_handle obj_id; // NOLINT
        if (is_static)
        {
            obj_id = M_static_tree.data(id);
            M_static_tree.remove_leaf(id);
        }
        else
        {
            obj_id = M_dynamic_tree.data(id);
            M_dynamic_tree.remove_leaf(id);

            auto removed = std::ranges::remove(M_moved, id);
            M_moved.erase(removed.begin(), removed.end());
        }

        M_pair_manager.remove_for_object(obj_id, [&narrow](std::uint64_t key)
                                         { narrow.on_pair_removed(key); });
    }

    void update_node(dynamic_bvh::node_handle id, const aabb &bounds,
                     const vec3<si::metre> &displacement)
    {
        if (M_dynamic_tree.update_leaf(id, bounds, displacement)) M_moved.push_back(id);
    }

    void calculate_pairs(narrow_phase &narrow,
                         std::regular_invocable<dynamic_bvh::object_handle> auto &&get_node_handle)
        requires(std::same_as<
                 std::invoke_result_t<decltype(get_node_handle), dynamic_bvh::object_handle>,
                 std::pair<dynamic_bvh::node_handle, bool>>)
    {
        for (auto it = M_pair_manager.active_pairs().begin();
             it != M_pair_manager.active_pairs().end(); ++it)
        {
            auto key = *it;
            auto [a, b] = pair_manager::extract_ids(key);
            auto [node_a, is_static_a] = get_node_handle(a);
            auto [node_b, is_static_b] = get_node_handle(b);

            auto bounds_a =
                is_static_a ? M_static_tree.bounds(node_a) : M_dynamic_tree.bounds(node_a);
            auto bounds_b =
                is_static_b ? M_static_tree.bounds(node_b) : M_dynamic_tree.bounds(node_b);

            if (bounds_a.intersects(bounds_b))
            {
                narrow.on_pair_removed(key);
                M_pair_manager.remove_pair(it++);
            }
            else
                ++it;
        }

        for (auto id : M_moved)
        {
            const auto &moving_bounds = M_dynamic_tree.bounds(id);
            const auto entity_a = M_dynamic_tree.data(id);

            // check if potential duplicate checks is a bottleneck
            M_dynamic_tree.query_aabb(
                moving_bounds,
                [&](dynamic_bvh::object_handle entity_b)
                {
                    if (entity_a != entity_b)
                    {
                        if (M_pair_manager.try_add_pair(entity_a, entity_b))
                            narrow.on_pair_added(pair_manager::make_pair_key(entity_a, entity_b));
                    }
                    return true;
                });
            M_static_tree.query_aabb(
                moving_bounds,
                [&](dynamic_bvh::object_handle entity_b)
                {
                    if (entity_a != entity_b)
                    {
                        if (M_pair_manager.try_add_pair(entity_a, entity_b))
                            narrow.on_pair_added(pair_manager::make_pair_key(entity_a, entity_b));
                    }
                    return true;
                });
        }

        M_moved.clear();
    }

private:
    dynamic_bvh M_static_tree;
    dynamic_bvh M_dynamic_tree;

    std::vector<dynamic_bvh::node_handle> M_moved; // TODO: handle duplicates

    pair_manager M_pair_manager;
};
} // namespace physkit::detail