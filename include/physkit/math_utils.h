#pragma once

#include "detail/types.h"
#include <cmath>
#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{
using namespace mp_units;

/// @brief Compute the dot product of two vectors with units: U · U → U²
template <auto unit, typename Rep>
auto dot_product(const vec3<unit, Rep> &a, const vec3<unit, Rep> &b)
{
    return a.dot(b); // Returns quantity<decltype(unit * unit), Rep>
}

/// @brief Compute the squared length of a vector: |v|² → U²
template <auto unit, typename Rep> auto squared_length(const vec3<unit, Rep> &v)
{
    return v.squared_norm(); // Returns quantity<decltype(unit * unit), Rep>
}

/// @brief Compute the length of a vector: |v| → U
template <auto unit, typename Rep> quantity<unit, Rep> length(const vec3<unit, Rep> &v)
{
    return v.norm(); // Already returns quantity<unit, Rep> — no reconstruction needed
}

/// @brief Normalize a vector (returns dimensionless unit vector)
template <auto unit, typename Rep> vec3<one, Rep> normalize(const vec3<unit, Rep> &v)
{
    auto len = v.norm();
    if (len > quantity<unit, Rep>{1e-9 * unit})
    {
        return v.normalized(); // Returns vec3<one, Rep>
    }
    return vec3<one, Rep>{0 * one, 0 * one, 0 * one};
}

/// @brief Compute the cross product: U × U → U² (vector)
template <auto unit, typename Rep>
auto cross_product(const vec3<unit, Rep> &a, const vec3<unit, Rep> &b)
{
    return a.cross(
        b); // Returns vec3<result_unit, Rep> where result_unit is derived from unit * unit
}

/// @brief Triple scalar product: a · (b × c) → U³
template <auto unit, typename Rep>
auto triple_scalar_product(const vec3<unit, Rep> &a, const vec3<unit, Rep> &b,
                           const vec3<unit, Rep> &c)
{
    return dot_product(a, cross_product(b, c));
}

/// @brief Distance between two points → U
template <auto unit, typename Rep>
quantity<unit, Rep> distance(const vec3<unit, Rep> &a, const vec3<unit, Rep> &b)
{
    return length(a - b);
}

/// @brief Squared distance → U²
template <auto unit, typename Rep>
auto squared_distance(const vec3<unit, Rep> &a, const vec3<unit, Rep> &b)
{
    return squared_length(a - b);
}

/// @brief Approximate equality check with tolerance
template <auto unit, typename Rep>
bool approx_equal(const vec3<unit, Rep> &a, const vec3<unit, Rep> &b,
                  quantity<unit, Rep> epsilon = quantity<unit, Rep>{1e-6 * unit})
{
    return squared_distance(a, b) < epsilon * epsilon;
}

/// @brief Reflection across a normal (normal must be dimensionless unit vector)
template <auto unit, typename Rep>
vec3<unit, Rep> reflect(const vec3<unit, Rep> &incident, const vec3<one, Rep> &normal)
{
    // dot = (U) · (dimensionless) → U
    auto dot_val = dot_product(incident, vec3<unit, Rep>{normal.x(), normal.y(), normal.z()});
    // 2 * dot * normal → U (normal is dimensionless)
    return incident - vec3<unit, Rep>{2 * dot_val * normal.x(), 2 * dot_val * normal.y(),
                                      2 * dot_val * normal.z()};
}

/// @brief Clamp value between bounds (generic)
template <typename T> T clamp(T value, T min_val, T max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

/// @brief Absolute value per component (preserves units)
template <auto unit, typename Rep> vec3<unit, Rep> abs_vector(const vec3<unit, Rep> &v)
{
    using std::abs;
    return vec3<unit, Rep>{abs(v.x()), abs(v.y()), abs(v.z())};
}

} // namespace physkit