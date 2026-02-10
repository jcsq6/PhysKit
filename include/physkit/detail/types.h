#pragma once
#include "lin_alg.h"

namespace physkit
{
using namespace mp_units;

using float_t = double;

template <auto unit, typename Rep = float_t> using vec3 = unit_mat<quantity<unit, Rep>, 3, 1>;
template <auto unit, typename Rep = float_t> using mat3 = unit_mat<quantity<unit, Rep>, 3, 3>;
template <auto unit, typename Rep = float_t> using mat4 = unit_mat<quantity<unit, Rep>, 4, 4>;
template <int Size, auto unit, typename Rep = float_t>
using vec = unit_mat<quantity<unit, Rep>, Size, 1>;

template <auto unit, typename Rep = float_t> using quat = unit_quat<quantity<unit, Rep>>;

} // namespace physkit