#include "physkit/gjk.h"
#include "physkit/mesh.h"

#include <array>
#include <mp-units/systems/si/unit_symbols.h>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;

#include <algorithm>
#include <cmath>
#include <limits>

namespace physkit
{
///@brief support function for GJK to take two collides and a direction - returns minkowski
/// difference
template <typename ShapeA, typename ShapeB>
vec3<si::metre> minkowski_difference_support(const ShapeA &shape_a, const ShapeB &shape_b,
                                             const vec3<one> &direction)
{
    // support A - B in direction d
    // Note: This assumes support_function is defined elsewhere for each shape type
    // For now, we'll leave this as a placeholder that needs implementation
    return vec3<si::metre>{0 * si::metre, 0 * si::metre, 0 * si::metre}; // Placeholder
}

///@brief construct simplex class
class Simplex
{
public:
    Simplex() : default {}
    void add_point(const vec3<si::metre> &point)
    {
        if (M_count < 4)
        {
            M_points[M_count] = point;
            M_count++;
        }
    }

    [[nodiscard]] size_t size() const { return M_count; }

    vec3<si::metre> &operator[](size_t i) { return M_points[i]; }
    const vec3<si::metre> &operator[](size_t i) const
    {
        return M_points[i];
    } // Fixed: was returning pointer, should return reference

    // remove points at index i and shift all points after
    void remove_point(size_t i)
    {
        for (size_t j = i; j < M_count - 1; ++j)
        { // Fixed: extra space in expression
            M_points[j] = M_points[j + 1];
        }
        M_count--;
    }

    // find the last point in the simplex
    const vec3<si::metre> &last() const
    {
        return M_points[M_count - 1];
    } // Fixed: return const reference
private:
    std::array<vec3<si::metre>, 4> M_points;
    [[nodiscard]] size_t M_count{};

    // Create support functions before creating the iteration loop

    vec
};

} // namespace physkit