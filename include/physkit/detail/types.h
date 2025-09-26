#pragma once
#include "lin_alg.h"

namespace physkit
{
using namespace mp_units;

template <auto unit> using vec3 = unit_mat<quantity<unit>, 3, 1>;
template <auto unit> using mat3 = unit_mat<quantity<unit>, 3, 3>;
template <auto unit> using mat4 = unit_mat<quantity<unit>, 4, 4>;

} // namespace physkit