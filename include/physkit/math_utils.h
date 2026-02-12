#pragma once

#include "detail/types.h"
#include <cmath>
#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{
/// @brief Compute the dot product of two vectors with units (wrapper for existing method)
template <typename T> auto dot_product(const vec3<T> &a, const vec3<T> &b) -> decltype(a.dot(b))
{
    return a.dot(b);
}

/// @brief Compute the squared length of a vector with units (wrapper for existing method)
template <typename T> auto squared_length(const vec3<T> &v) -> decltype(v.squared_norm())
{
    return v.squared_norm();
}

/// @brief Compute the length of a vector with units (wrapper for existing method)
template <typename T> auto length(const vec3<T> &v) -> T
{
    return T{v.norm().numerical_value_in(T{})};
}

/// @brief Normalize a vector (returns unit vector) (wrapper for existing method)
template <typename T> vec3<one> normalize(const vec3<T> &v)
{
    auto norm = v.norm().numerical_value_in(T{});
    if (norm > 1e-9) { return v.normalized(); }
    return vec3<one>{0, 0, 0};
}

/// @brief Compute the cross product of two vectors (wrapper for existing method)
template <typename T> vec3<T> cross_product(const vec3<T> &a, const vec3<T> &b)
{
    return a.cross(b);
}

/// @brief Compute the triple scalar product: a · (b × c)
template <typename T>
auto triple_scalar_product(const vec3<T> &a, const vec3<T> &b, const vec3<T> &c) -> T
{
    return dot_product(a, cross_product(b, c));
}

/// @brief Compute the distance between two points
template <typename T> T distance(const vec3<T> &a, const vec3<T> &b) { return length(a - b); }

/// @brief Compute the squared distance between two points
template <typename T>
auto squared_distance(const vec3<T> &a, const vec3<T> &b) -> decltype(squared_length(a - b))
{
    return squared_length(a - b);
}

/// @brief Check if two vectors are approximately equal
template <typename T> bool approx_equal(const vec3<T> &a, const vec3<T> &b, T epsilon = T{1e-6})
{
    return squared_distance(a, b) < epsilon * epsilon;
}

/// @brief Compute the reflection of a vector across a normal
template <typename T> vec3<T> reflect(const vec3<T> &incident, const vec3<one> &normal)
{
    auto dot = incident.dot(vec3<T>{normal.x(), normal.y(), normal.z()});
    return incident - vec3<T>{2 * dot.numerical_value_in(T{}) * normal.x(),
                              2 * dot.numerical_value_in(T{}) * normal.y(),
                              2 * dot.numerical_value_in(T{}) * normal.z()};
}

/// @brief Clamp a value between two bounds
template <typename T> T clamp(T value, T min_val, T max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

// Helper function to get absolute value of a vector
template <typename T> vec3<one> abs_vector(const vec3<T> &v)
{
    return vec3<one>{std::abs(v.x().numerical_value_in(T{})),
                     std::abs(v.y().numerical_value_in(T{})),
                     std::abs(v.z().numerical_value_in(T{}))};
}
} // namespace physkit