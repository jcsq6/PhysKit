#pragma once
#include "integrator.h"
#include "object.h"

#include <expected>
#include <stack>

namespace physkit
{

class world_desc
{
public:
    enum class integ_t : std::uint8_t
    {
        forward_euler,
        semi_implicit_euler,
        velocity_verlet,
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
    integ_t M_integrator_type = integ_t::semi_implicit_euler;
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
        static constexpr std::size_t noslot = std::numeric_limits<std::size_t>::max();

        handle(key /*unused*/, std::size_t idx, std::size_t gen) noexcept : M_idx(idx), M_gen(gen)
        {
        }

        [[nodiscard]] auto index() const { return M_idx; }
        [[nodiscard]] auto generation() const { return M_gen; }

    private:
        std::size_t M_idx;
        std::size_t M_gen;

        friend world;
    };

    world(const world_desc &desc) : M_gravity(desc.gravity()), M_iters(desc.solver_iterations())
    {
        switch (desc.integrator())
        {
        case world_desc::integ_t::forward_euler:
            M_int = std::make_unique<forward_euler>();
            break;
        case world_desc::integ_t::semi_implicit_euler:
            M_int = std::make_unique<semi_implicit_euler>();
            break;
        case world_desc::integ_t::velocity_verlet:
            M_int = std::make_unique<velocity_verlet>();
            break;
        case world_desc::integ_t::rk4:
            M_int = std::make_unique<rk4>();
            break;
        }
    }

    void step(quantity<si::second> dt);

    auto create_rigid(const object_desc &desc) { return M_rigid.add(object(desc)); }
    auto remove_rigid(handle h) { return M_rigid.remove(h); }
    auto get_rigid(handle h)
    {
        return M_rigid.get(h).transform([](auto p) { return std::ref(*p); });
    }

private:
    template <typename T> struct slot
    {
        std::size_t gen;
        T value;
        bool available;
    };
    template <typename T> struct arena
    {
        std::vector<slot<T>> slots;
        std::stack<std::size_t, std::vector<std::size_t>> free;

        handle add(T &&value)
        {
            if (free.empty())
            {
                slots.push_back({.gen = 0, .value = std::move(value), .available = false});
                return handle{handle::key{}, slots.size() - 1, 0};
            }

            auto idx = free.top();
            free.pop();
            slots[idx].value = std::move(value);
            slots[idx].available = false;
            return handle{handle::key{}, idx, slots[idx].gen};
        }

        std::expected<T, err_t> remove(handle h)
        {
            assert(h.index() < slots.size());
            auto &slot = slots[h.index()];
            if (slot.gen != h.generation() || slot.available)
                return std::unexpected(err_t::stale_handle);
            slot.available = true;
            slot.gen++;
            free.push(h.index());
            return std::move(slot.value);
        }

        std::expected<T *, err_t> get(handle h)
        {
            assert(h.index() < slots.size());
            auto &slot = slots[h.index()];
            if (slot.gen != h.generation() || slot.available)
                return std::unexpected(err_t::stale_handle);
            return &slot.value;
        }
    };

    std::unique_ptr<integrator> M_int;
    arena<object> M_rigid;
    vec3<si::metre / si::second / si::second> M_gravity;
    std::size_t M_iters;
};

} // namespace physkit