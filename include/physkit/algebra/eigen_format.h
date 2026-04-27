#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#else
#include <Eigen/Dense>
#endif

#include "lin_alg.h"

namespace physkit::detail
{
template <typename T>
using row_wise_range = std::ranges::subrange<decltype(std::declval<T>().rowwise().begin()),
                                             decltype(std::declval<T>().rowwise().end())>;
} // namespace physkit::detail

template <class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    requires(Rows != 1 && Cols != 1)
struct std::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
    : public std::formatter<physkit::detail::row_wise_range<
          const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>>
{
    template <typename FormatContext>
    auto format(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &matrix,
                FormatContext &ctx) const
    {
        auto rowwise = matrix.rowwise();
        return std::formatter<physkit::detail::row_wise_range<
            const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>>::
            format(std::ranges::subrange{rowwise.begin(), rowwise.end()}, ctx);
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
