#pragma once
#include <Eigen/Dense>
#include <format>
#include <sstream>
#include <type_traits>

#include "lin_alg.h"

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
class std::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
    : public std::formatter<std::string>
{
public:
    template <typename FormatContext>
    auto format(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &matrix,
                FormatContext &ctx) const
    {
        std::ostringstream oss;
        if constexpr (Cols == 1)
            oss << matrix.format(
                Eigen::IOFormat(Eigen::FullPrecision, 0, "", ",", "", "", "[", "]"));
        else
            oss << matrix.format(
                Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]", "[", "]"));
        return std::formatter<std::string>::format(oss.str(), ctx);
    }
};

template <mp_units::Quantity Q, int Rows, int Cols>
class std::formatter<physkit::unit_mat<Q, Rows, Cols>> : public std::formatter<std::string>
{
public:
    template <typename FormatContext>
    auto format(const physkit::unit_mat<Q, Rows, Cols> &matrix, FormatContext &ctx) const
    {
        std::ostringstream oss;
        if constexpr (Cols == 1)
        {
            oss << matrix.base().format(
                Eigen::IOFormat(Eigen::FullPrecision, 0, "", ",", "", "", "[", "]"));
            std::print(oss, "{}", Q::unit);
        }
        else
        {
            oss << matrix.base().format(
                Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]", "[", "]"));
            std::print(oss, " {}", Q::unit);
        }
        return std::formatter<std::string>::format(oss.str(), ctx);
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