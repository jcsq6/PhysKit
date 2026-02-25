#pragma once
#include "lin_alg.h"

namespace physkit
{
using namespace mp_units;

using float_t = double;

template <auto unit = one, typename Rep = float_t> using vec3 = unit_mat<quantity<unit, Rep>, 3, 1>;
template <auto unit = one, typename Rep = float_t> using mat3 = unit_mat<quantity<unit, Rep>, 3, 3>;
template <auto unit = one, typename Rep = float_t> using mat4 = unit_mat<quantity<unit, Rep>, 4, 4>;
template <int Size, auto unit = one, typename Rep = float_t>
using vec = unit_mat<quantity<unit, Rep>, Size, 1>;

template <auto unit = one, typename Rep = float_t> using quat = unit_quat<quantity<unit, Rep>>;

template <auto unit = one> using fvec3 = vec3<unit, float>;
template <auto unit = one> using fmat3 = mat3<unit, float>;
template <auto unit = one> using fmat4 = mat4<unit, float>;
template <int Size, auto unit = one> using fvec = vec<Size, unit, float>;

template <auto unit = one> using fquat = quat<unit, float>;

template <typename Rep = float_t> using area_t = quantity<si::metre * si::metre, Rep>;
template <typename Rep = float_t> using volume_t = quantity<si::metre * si::metre * si::metre, Rep>;

} // namespace physkit