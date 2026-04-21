#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#else
#include <Eigen/Dense>
#endif

#include "lin_alg.h"

template <class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct std::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
{
    constexpr auto parse(std::format_parse_context &pc)
    {
        const auto *it = pc.begin();
        if ((it != pc.end()) && (*it != '}')) throw std::format_error("invalid format");
        return it;
    }

    template <class FormatContext>
    constexpr auto format(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &m,
                          FormatContext &ctx) const
    {
        if ((m.rows() == 1) || (m.cols() == 1))
        {
            if (m.rows() == 1)
            {
                auto row_range = std::views::iota(Eigen::Index{0}, m.cols()) |
                                 std::views::transform([&m](Eigen::Index col) -> const Scalar &
                                                       { return m(0, col); });
                return std::format_to(ctx.out(), "{}", row_range);
            }

            auto col_range = std::views::iota(Eigen::Index{0}, m.rows()) |
                             std::views::transform([&m](Eigen::Index row) -> const Scalar &
                                                   { return m(row, 0); });
            return std::format_to(ctx.out(), "{}", col_range);
        }

        auto mat_range =
            std::views::iota(Eigen::Index{0}, m.rows()) |
            std::views::transform(
                [&m](Eigen::Index row)
                {
                    return std::views::iota(Eigen::Index{0}, m.cols()) |
                           std::views::transform([&m, row](Eigen::Index col) -> const Scalar &
                                                 { return m(row, col); });
                });

        return std::format_to(ctx.out(), "{}", mat_range);
    }
};

template <mp_units::Quantity Q, int Rows, int Cols>
struct std::formatter<physkit::unit_mat<Q, Rows, Cols>>
    : public std::formatter<typename physkit::unit_mat<Q, Rows, Cols>::eigen_type>
{
    template <typename FormatContext>
    auto format(const physkit::unit_mat<Q, Rows, Cols> &matrix, FormatContext &ctx) const
    {
        auto it = std::formatter<typename physkit::unit_mat<Q, Rows, Cols>::eigen_type>::format(
            matrix.base(), ctx);
        return std::format_to(it, " {}", Q::unit);
    }
};

template <mp_units::Quantity Q> struct std::formatter<physkit::unit_quat<Q>>
{
    std::formatter<typename Q::rep> underlying;
    constexpr auto parse(std::format_parse_context &pc) { return underlying.parse(pc); }

    template <typename FormatContext>
    auto format(const physkit::unit_quat<Q> &q, FormatContext &ctx) const
    {
        ctx.advance_to(std::ranges::copy("(", ctx.out()).out);
        ctx.advance_to(underlying.format(q.base().w(), ctx));
        ctx.advance_to(std::ranges::copy(", ", ctx.out()).out);
        ctx.advance_to(underlying.format(q.base().x(), ctx));
        ctx.advance_to(std::ranges::copy(", ", ctx.out()).out);
        ctx.advance_to(underlying.format(q.base().y(), ctx));
        ctx.advance_to(std::ranges::copy(", ", ctx.out()).out);
        ctx.advance_to(underlying.format(q.base().z(), ctx));
        ctx.advance_to(std::ranges::copy(")", ctx.out()).out);
        return std::format_to(ctx.out(), " {}", Q::unit);
    }
};
