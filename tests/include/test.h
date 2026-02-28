#pragma once
#include <physkit/physkit.h>

#include <cmath>
#include <functional>
#include <iostream> // IWYU pragma: keep
#include <print>
#include <source_location> // IWYU pragma: keep
#include <string_view>
#include <vector>

namespace testing
{

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;

// ---------------------------------------------------------------------------
// Approximate comparison
// ---------------------------------------------------------------------------

constexpr double eps = 1e-9;

template <std::floating_point T> bool approx(T a, T b)
{
    return std::abs(a - b) < static_cast<T>(eps);
}

template <std::floating_point T> bool approx(T a, T b, T tol) { return std::abs(a - b) < tol; }

template <Quantity Q> bool approx(Q a, Q b)
{
    return approx(a.numerical_value_in(Q::reference), b.numerical_value_in(Q::reference));
}

template <Quantity Q> bool approx(Q a, Q b, Q tol)
{
    return std::abs((a - b).numerical_value_in(Q::reference)) <
           tol.numerical_value_in(Q::reference);
}

template <Quantity Q, int Rows, int Cols>
bool approx(const unit_mat<Q, Rows, Cols> &a, const unit_mat<Q, Rows, Cols> &b)
{
    return std::abs((a - b).norm().numerical_value_in(Q::reference)) < eps;
}

template <Quantity Q> bool approx(const unit_quat<Q> &a, const unit_quat<Q> &b)
{
    return std::abs(a.angular_distance(b)) < eps;
}

inline bool approx(const aabb &a, const aabb &b)
{
    return approx(a.min, b.min) && approx(a.max, b.max);
}

inline bool approx(const bounding_sphere &a, const bounding_sphere &b)
{
    return approx(a.center, b.center) && approx(a.radius, b.radius);
}

inline bool approx(const ray::hit &a, const ray::hit &b)
{
    return approx(a.pos, b.pos) && approx(a.normal, b.normal) && approx(a.distance, b.distance);
}

namespace detail
{
template <typename A, typename B> bool approx_check(const A &a, const B &b) { return approx(a, b); }
template <typename A, typename B, typename C>
bool approx_check(const A &a, const B &b, const C &tol)
{
    return approx(a, b, tol);
}
} // namespace detail

// NOLINTBEGIN(cppcoreguidelines-macro-usage)

class check_failure : public std::runtime_error
{
public:
    explicit check_failure(const std::string &message) : std::runtime_error(message) {}
};

#define CHECK(...)                                                                                 \
    {                                                                                              \
        if (!(__VA_ARGS__))                                                                        \
        {                                                                                          \
            throw check_failure(std::format("{} ({}:{})", #__VA_ARGS__,                            \
                                            std::source_location::current().file_name(),           \
                                            std::source_location::current().line()));              \
        }                                                                                          \
    }

#define CHECK_APPROX(...)                                                                          \
    {                                                                                              \
        if (!testing::detail::approx_check(__VA_ARGS__))                                           \
        {                                                                                          \
            throw check_failure(std::format("approx({}) ({}:{})", #__VA_ARGS__,                    \
                                            std::source_location::current().file_name(),           \
                                            std::source_location::current().line()));              \
        }                                                                                          \
    }

#define FAIL(msg)                                                                                  \
    {                                                                                              \
        throw check_failure(std::format("{} ({}:{})", (msg),                                       \
                                        std::source_location::current().file_name(),               \
                                        std::source_location::current().line()));                  \
    }

// NOLINTEND(cppcoreguidelines-macro-usage)

class suite
{
public:
    suite &group(std::string_view name)
    {
        M_sections.push_back({.name = name});
        return *this;
    }

    suite &test(std::string_view name, std::function<void()> fn)
    {
        if (M_sections.empty()) group("Tests");
        M_sections.back().tests.push_back({.name = name, .fn = std::move(fn)});
        return *this;
    }

    [[nodiscard]] int run() const
    {
        int total = 0;
        int num_passed = 0;
        for (const auto &sec : M_sections)
        {
            std::println("=== {} ===", sec.name);
            for (const auto &[name, fn] : sec.tests)
            {
                try
                {
                    fn();
                    std::println("  PASS: {}", name);
                    ++num_passed;
                }
                catch (const check_failure &e)
                {
                    std::println(std::cerr, "  FAIL: {} - {}", name, e.what());
                }
                catch (const std::exception &e)
                {
                    std::println(std::cerr, "  ERROR: {} - {}", name, e.what());
                }
                catch (...)
                {
                    std::println(std::cerr, "  ERROR: {} - unknown exception", name);
                }
                ++total;
            }
            std::println("");
        }
        if (num_passed == total)
        {
            std::println("All {} tests passed!", total);
            return 0;
        }

        if (num_passed == 0)
            std::println("All {} tests failed!", total);
        else
            std::println("{} out of {} tests passed!", num_passed, total);
        return 1;
    }

private:
    struct test_case
    {
        std::string_view name;
        std::function<void()> fn;
    };
    struct section
    {
        std::string_view name;
        std::vector<test_case> tests;
    };

    std::vector<section> M_sections;
};

} // namespace testing
