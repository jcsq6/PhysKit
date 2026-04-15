#pragma once
#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "macros.h"
#include <ranges>
#include <vector>
#endif

PHYSKIT_EXPORT
namespace physkit::detail
{
// TODO: make thread safe for future multithreading support
template <typename T> struct swappable_queue
{
    class scoped_range
    {
    public:
        scoped_range(swappable_queue &queue) : M_queue(queue)
        {
            queue.swap_and_clear();
            // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
            M_current = M_queue.M_swap.begin();
        }
        scoped_range(const scoped_range &) = delete;
        scoped_range(scoped_range &&) = delete;
        scoped_range &operator=(const scoped_range &) = delete;
        scoped_range &operator=(scoped_range &&) = delete;

        ~scoped_range()
        {
            M_queue.M_main.append_range(std::ranges::subrange(M_current, M_queue.M_swap.end()) |
                                        std::views::as_rvalue);
            M_queue.M_swap.clear();
        }

        struct sentinel
        {
            std::vector<T>::iterator end;
        };

        class iterator
        {
            std::vector<T>::iterator *M_current;

        public:
            using iterator_category = std::input_iterator_tag;
            using value_type = T;
            using difference_type = std::ptrdiff_t;

            iterator(std::vector<T>::iterator *current) : M_current(current) {}
            ~iterator() = default;

            iterator(iterator &&other) noexcept : M_current(std::exchange(other.M_current, nullptr))
            {
            }
            iterator &operator=(iterator &&other) noexcept
            {
                M_current = std::exchange(other.M_current, nullptr);
                return *this;
            }
            iterator(const iterator &) = delete;
            iterator &operator=(const iterator &) = delete;

            decltype(auto) operator*() const { return **M_current; }
            auto operator->() const { return &(*M_current); }

            iterator &operator++()
            {
                ++*M_current;
                return *this;
            }
            void operator++(int) { ++(*this); }

            bool operator==(const sentinel &s) const { return *M_current == s.end; }
        };

        iterator begin() { return {&M_current}; }
        sentinel end() { return {M_queue.M_swap.end()}; }

    private:
        swappable_queue &M_queue;
        std::vector<T>::iterator M_current;
    };

    void push(T item) { M_main.emplace_back(std::move(item)); }

    bool drain(std::regular_invocable<T &> auto &&on_each)
    {
        bool drained = !M_main.empty();

        while (!M_main.empty())
            for (auto &item : iter()) on_each(item);

        return drained;
    }

    [[nodiscard]] auto iter() { return scoped_range(*this); }

    [[nodiscard]] bool empty() const { return M_main.empty(); }

private:
    std::vector<T> M_main;
    std::vector<T> M_swap;

    void swap_and_clear()
    {
        M_swap.clear();
        M_swap.swap(M_main);
    }
};
} // namespace physkit::detail