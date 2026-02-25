#pragma once
#include "detail/collision_phases.h"
#include "integrator.h"
#include "object.h"

#include <expected>
#include <stack>

namespace physkit
{

class world_desc
{
public:
    enum integ_t : std::uint8_t
    {
        forward_euler,
        semi_implicit_euler,
        rk4,
    };

    static world_desc make() { return {}; }

    [[nodiscard]] auto gravity() const { return M_gravity; }
    auto &&with_gravity(this auto &&self, const vec3<si::metre / si::second / si::second> &gravity)
    {
        self.M_gravity = gravity;
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] auto integrator() const { return M_integrator_type; }
    auto &&with_integrator(this auto &&self, integ_t type)
    {
        self.M_integrator_type = type;
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] auto solver_iterations() const { return M_solver_iterations; }
    auto &&with_solver_iterations(this auto &&self, std::size_t iterations)
    {
        self.M_solver_iterations = iterations;
        return std::forward<decltype(self)>(self);
    }

private:
    vec3<si::metre / si::second / si::second> M_gravity = {
        0.0 * si::metre / si::second / si::second,
        -9.81 * si::metre / si::second / si::second,
        0.0 * si::metre / si::second / si::second,
    };
    integ_t M_integrator_type = semi_implicit_euler;
    std::size_t M_solver_iterations = 10;
};

class world
{
public:
    enum class err_t : std::uint8_t
    {
        stale_handle
    };

    class handle
    {
        struct key
        {
        };

    public:
        static constexpr std::uint32_t noslot = std::numeric_limits<std::uint32_t>::max();

        handle(key /*unused*/, std::uint32_t idx, std::uint16_t gen) noexcept
            : M_idx(idx), M_gen(gen)
        {
        }

        [[nodiscard]] auto index() const { return M_idx; }
        [[nodiscard]] auto generation() const { return M_gen; }

    private:
        std::uint32_t M_idx;
        std::uint16_t M_gen;

        friend world;
    };

    explicit world(const world_desc &desc)
        : M_gravity(desc.gravity()), M_iters(desc.solver_iterations())
    {
        switch (desc.integrator())
        {
        case world_desc::forward_euler:
            M_int = std::make_unique<forward_euler>();
            break;
        case world_desc::semi_implicit_euler:
            M_int = std::make_unique<semi_implicit_euler>();
            break;
        case world_desc::rk4:
            M_int = std::make_unique<rk4>();
            break;
        }
    }

    void step(quantity<si::second> dt);

    auto create_rigid(const object_desc &desc) { return M_rigid.add(object(desc), M_broad); }
    auto remove_rigid(handle h) { return M_rigid.remove(h, M_broad, M_narrow); }
    auto get_rigid(this auto &&self, handle h) { return self.M_rigid.get(h); }

private:
    struct arena
    {
        struct slot
        {
            object obj;
            detail::dynamic_bvh::node_handle broad_handle;
            std::uint16_t gen;
            bool available;
        };

        std::vector<slot> slots;
        std::stack<std::uint32_t, std::vector<std::uint32_t>> free;

        handle add(object &&obj, detail::broad_phase &broad)
        {
            if (free.empty())
            {
                auto bh = broad.add(static_cast<detail::dynamic_bvh::object_handle>(slots.size()),
                                    obj.instance().bounds(), obj.is_static());
                slots.push_back(
                    {.obj = std::move(obj), .broad_handle = bh, .gen = 0, .available = false});
                return handle{handle::key{}, static_cast<std::uint32_t>(slots.size() - 1), 0};
            }

            auto idx = free.top();
            free.pop();
            slots[idx].obj = std::move(obj);
            slots[idx].broad_handle =
                broad.add(static_cast<detail::dynamic_bvh::object_handle>(idx),
                          slots[idx].obj.instance().bounds(), slots[idx].obj.is_static());
            slots[idx].available = false;
            return handle{handle::key{}, idx, slots[idx].gen};
        }

        std::expected<object, err_t> remove(handle h, detail::broad_phase &broad,
                                            detail::narrow_phase &narrow)
        {
            assert(h.index() < slots.size());
            auto &slot = slots[h.index()];
            if (slot.gen != h.generation() || slot.available)
                return std::unexpected(err_t::stale_handle);
            broad.remove(slot.broad_handle, slot.obj.is_static(), narrow);
            slot.available = true;
            slot.gen++;
            free.push(h.index());
            return std::move(slot.obj);
        }

        auto get(this auto &&self, handle h) -> std::expected<
            std::conditional_t<std::is_const_v<std::remove_reference_t<decltype(self)>>,
                               const object *, object *>,
            err_t>
        {
            assert(h.index() < self.slots.size());
            auto &slot = self.slots[h.index()];
            if (slot.gen != h.generation() || slot.available)
                return std::unexpected(err_t::stale_handle);
            return &slot.obj;
        }

        auto &get_slot(this auto &&self, std::uint32_t idx)
        {
            assert(idx < self.slots.size());
            return self.slots[idx];
        }
    };

    std::unique_ptr<integrator> M_int;
    arena M_rigid;
    detail::broad_phase M_broad;
    detail::narrow_phase M_narrow;
    vec3<si::metre / si::second / si::second> M_gravity;
    std::size_t M_iters;
};

} // namespace physkit