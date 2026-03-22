#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "macros.h"
#include "util.h"
#include <cassert>
#include <cstdint>
#include <optional>
#include <stack>
#include <utility>
#include <vector>
#endif

PHYSKIT_EXPORT
namespace physkit::detail
{
template <typename T> struct arena
{
    struct slot
    {
        T value;
        std::uint16_t gen;
        bool available;
    };

    class handle
    {
    public:
        using id_type = std::uint64_t;

        static constexpr std::uint32_t null = std::numeric_limits<std::uint32_t>::max();

        constexpr handle(std::uint32_t idx, std::uint32_t gen, passkey<arena> /*unnamed*/) noexcept
            : M_idx(idx), M_gen(gen)
        {
        }

        static constexpr handle from_id(id_type id) noexcept { return std::bit_cast<handle>(id); }

        [[nodiscard]] constexpr id_type id() const { return std::bit_cast<id_type>(*this); }

        [[nodiscard]] constexpr bool is_null_handle() const { return M_idx == null; }

        [[nodiscard]] constexpr auto index() const { return M_idx; }
        [[nodiscard]] constexpr auto generation() const { return M_gen; }

    private:
        std::uint32_t M_idx;
        std::uint32_t M_gen;
    };

    std::vector<slot> slots;
    std::stack<std::uint32_t, std::vector<std::uint32_t>> free;

    arena(std::size_t initial_capacity = 100) { slots.reserve(initial_capacity); }

    handle add(T value)
    {
        if (free.empty())
        {
            slots.push_back(
                {.value = std::forward<decltype(value)>(value), .gen = 0, .available = false});
            return handle{static_cast<std::uint32_t>(slots.size() - 1), 0, {}};
        }

        auto idx = free.top();
        free.pop();
        slots[idx].value = std::forward<decltype(value)>(value);
        slots[idx].available = false;
        return handle{idx, slots[idx].gen, {}};
    }

    auto next_handle() const
    {
        if (free.empty()) return handle{static_cast<std::uint32_t>(slots.size()), 0, {}};
        auto idx = free.top();
        return handle{idx, slots[idx].gen, {}};
    }

    std::optional<T> remove(handle h)
    {
        assert(h.index() < slots.size());
        auto &slot = slots[h.index()];
        if (slot.gen != h.generation() || slot.available) return std::nullopt;
        slot.available = true;
        slot.gen++;
        free.push(h.index());
        return std::move(slot.value);
    }

    auto get(this auto &&self, handle h)
    {
        assert(h.index() < self.slots.size());
        auto &slot = self.slots[h.index()];
        if (slot.gen != h.generation() || slot.available) return decltype(&slot.value)(nullptr);
        return &slot.value;
    }

    auto &get_slot(this auto &&self, std::uint32_t idx)
    {
        assert(idx < self.slots.size());
        return self.slots[idx];
    }
};
} // namespace physkit::detail