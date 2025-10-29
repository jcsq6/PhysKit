#pragma once
#include "lin_alg.h"

namespace physkit
{
using namespace mp_units;

template <auto unit, typename Rep = double> using vec3 = unit_mat<quantity<unit, Rep>, 3, 1>;
template <auto unit, typename Rep = double> using mat3 = unit_mat<quantity<unit, Rep>, 3, 3>;
template <auto unit, typename Rep = double> using mat4 = unit_mat<quantity<unit, Rep>, 4, 4>;
template <int Size, auto unit, typename Rep = double> using vec = unit_mat<quantity<unit, Rep>, Size, 1>;

} // namespace physkit