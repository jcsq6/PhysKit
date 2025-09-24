#pragma once
#include <Eigen/Dense>
#include <format>
#include <sstream>

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