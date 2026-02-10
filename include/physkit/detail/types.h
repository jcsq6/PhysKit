#pragma once
#include "lin_alg.h"

namespace physkit
{
using namespace mp_units;

using float_t = double;

template <auto unit, typename Rep = float_t> using uvec3 = unit_mat<quantity<unit, Rep>, 3, 1>;
template <auto unit, typename Rep = float_t> using umat3 = unit_mat<quantity<unit, Rep>, 3, 3>;
template <auto unit, typename Rep = float_t> using umat4 = unit_mat<quantity<unit, Rep>, 4, 4>;
template <int Size, auto unit, typename Rep = float_t>
using uvec = unit_mat<quantity<unit, Rep>, Size, 1>;

template <auto unit, typename Rep = float_t> using uquat = unit_quat<quantity<unit, Rep>>;

template <typename Rep = float_t>
using vec3 =
    Eigen::Matrix<Rep, 3, 1, Eigen::ColMajor, 3, 1>; // explicitly type all template args for CTAD
template <typename Rep = float_t> using mat3 = Eigen::Matrix<Rep, 3, 3, Eigen::ColMajor, 3, 3>;
template <typename Rep = float_t> using mat4 = Eigen::Matrix<Rep, 4, 4, Eigen::ColMajor, 4, 4>;
template <int Size, typename Rep = float_t>
using vec = Eigen::Matrix<Rep, Size, 1, Eigen::ColMajor, Size, 1>;

template <typename Rep = float_t> using quat = Eigen::Quaternion<Rep>;

} // namespace physkit