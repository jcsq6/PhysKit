#pragma once
#include <Eigen/Dense>
#include <Eigen/src/Core/IO.h>
#include <Eigen/src/Core/Matrix.h>
#include <format>
#include <ranges>
#include <type_traits>

#include "lin_alg.h"

template <class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct std::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
{
    std::formatter<Scalar> underlying;

    constexpr auto parse(std::format_parse_context &pc) { return underlying.parse(pc); }

    template <class FormatContext>
    constexpr auto format(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &m,
                          FormatContext &ctx) const
    {
        ctx.advance_to(std::ranges::copy("[", ctx.out()).out);

        auto on_row = [this, &ctx](std::ranges::range auto &&vec, bool terminate = false)
        {
            bool use_separator = false;
            for (auto &&e : vec)
            {
                if (use_separator)
                    ctx.advance_to(std::ranges::copy(", ", ctx.out()).out);
                else
                    use_separator = true;
                ctx.advance_to(underlying.format(e, ctx));
            }

            if (terminate) ctx.advance_to(std::ranges::copy("]", ctx.out()).out);
        };

        auto on_mat = [&on_row, &ctx](auto &&m)
        {
            auto rows = m.rowwise();
            bool use_separator = false;
            for (auto row : rows)
            {
                if (use_separator)
                    ctx.advance_to(std::ranges::copy(",\n ", ctx.out()).out);
                else
                    use_separator = true;
                on_row(row, false);
            }

            ctx.advance_to(std::ranges::copy("]", ctx.out()).out);
        };

        if constexpr (m.IsVectorAtCompileTime == 1)
        {
            if ((m.rows() == 1) || (m.cols() == 1))
                on_row(m, true);
            else
                on_mat(m);
        }
        else
            on_mat(m);
        return ctx.out();
    }
};

template <mp_units::Quantity Q, int Rows, int Cols>
class std::formatter<physkit::unit_mat<Q, Rows, Cols>>
    : public std::formatter<typename physkit::unit_mat<Q, Rows, Cols>::eigen_type>
{
public:
    template <typename FormatContext>
    auto format(const physkit::unit_mat<Q, Rows, Cols> &matrix, FormatContext &ctx) const
    {
        auto it = std::formatter<typename physkit::unit_mat<Q, Rows, Cols>::eigen_type>::format(
            matrix.base(), ctx);
        return std::format_to(it, " {}", Q::unit);
    }
};

template <mp_units::Quantity Q>
class std::formatter<physkit::detail::quantity_ref<Q>> : public std::formatter<std::remove_cv_t<Q>>
{
public:
    template <typename FormatContext>
    auto format(const physkit::detail::quantity_ref<Q> &ref, FormatContext &ctx) const
    {
        return std::formatter<std::remove_cv_t<Q>>::format(static_cast<Q>(ref), ctx);
    }
};