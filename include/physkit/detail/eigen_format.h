#pragma once
#include <Eigen/Dense>
#include <format>
#include <sstream>

#include "lin_alg.h"

// fix annoying deprecation from mp-units version 2.4.0. It should be fixed in 2.5.0, when that's
// released
MP_UNITS_DIAGNOSTIC_IGNORE_DEPRECATED
#include <mp-units/format.h> // needed for std::println with mp-units
MP_UNITS_DIAGNOSTIC_POP

template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
class std::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> : public std::formatter<std::string>
{
public:
    template <typename FormatContext>
    auto format(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& matrix, FormatContext& ctx) const
    {
        std::ostringstream oss;
        if constexpr (Cols == 1)
            oss << matrix.format(Eigen::IOFormat(Eigen::FullPrecision, 0, "", ",", "", "", "[", "]"));
        else
            oss << matrix.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]", "[", "]"));
        return std::formatter<std::string>::format(oss.str(), ctx);
    }
};

template <mp_units::Quantity Q, int Rows, int Cols>
class std::formatter<physkit::unit_mat<Q, Rows, Cols>> : public std::formatter<std::string>
{
public:
    template <typename FormatContext>
    auto format(const physkit::unit_mat<Q, Rows, Cols>& matrix, FormatContext& ctx) const
    {
        std::ostringstream oss;
        if constexpr (Cols == 1)
        {
            oss << matrix.base().format(Eigen::IOFormat(Eigen::FullPrecision, 0, "", ",", "", "", "[", "]"));
            std::print(oss, "{}", Q::unit);
        }
        else
        {
            oss << matrix.base().format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]", "[", "]"));
            std::print(oss, " {}", Q::unit);
        }
        return std::formatter<std::string>::format(oss.str(), ctx);
    }
};