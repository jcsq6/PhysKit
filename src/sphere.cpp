#include "physkit/sphere.h"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <numbers>

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

const aabb &sphere::bounds() const { return M_aabb; }

const bounding_sphere &sphere::bsphere() const { return M_bsphere; }

quantity<pow<3>(m)> sphere::volume() const
{
    return (4.0f / 3.0f) * std::numbers::pi * mp_units::pow<3>(M_radius);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
vec3<m> sphere::mass_center() const { return {0.0f * m, 0.0f * m, 0.0f * m}; }

mat3<kg * pow<2>(m)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
sphere::inertia_tensor(quantity<kg / pow<3>(m)> density) const
{
    auto val = ((2.0f / 5.0f) * density * this->volume() * (mp_units::pow<2>(M_radius)))
                   .numerical_value_in(kg * m * m);

    const auto ret = mat3{{val, 0.0f, 0.0f, 0.0f, val, 0.0f, 0.0f, 0.0f, val}} * kg * m * m;

    return ret;
    // return physkit::mat3<kg * pow<2>(m)>
    //{val, 0*kg*m*m, 0*kg*m*m,
    // 0*kg*m*m, val, 0*kg*m*m,
    // 0*kg*m*m, 0*kg*m*m, val};
}

// ray intersection
std::optional<ray::hit> sphere::ray_intersect(const ray &r, quantity<m> max_distance) const
{

    auto best_t = max_distance;
    std::optional<ray::hit> best;

    // should be able to manualy fill in the ray hit.
    // point of impact
    // normal of surface point
    // distance along the ray
    auto origin = r.origin();
    auto direction = r.direction();

    /*
    auto a = direction.dot(direction);
    auto b = 2*(origin - M_position).dot(direction);
    auto c = (origin - M_position).dot(origin-M_position) - mp_units::pow<2>(M_radius);

    auto det = mp_units::pow<2>(b) - 4*a*c;
    if (det < 0*m*m)
    {
        return std::nullopt;
    }
    auto dist = (b - mp_units::sqrt(det)) / (2*a);
    auto pos = origin + dist*direction;

    auto normal = (pos - M_position).normalized();
    */
    auto a = direction.dot(direction);
    auto b = 2 * (origin.dot(direction));
    auto c = origin.dot(origin) - mp_units::pow<2>(M_radius);

    auto det = mp_units::pow<2>(b) - 4 * a * c;
    if (det < 0 * m * m) return std::nullopt;

    auto dist = (b - mp_units::sqrt(det)) / (2 * a);
    auto pos = origin + dist * direction;
    auto normal = pos.normalized();

    return ray::hit{.pos = pos, .normal = normal, .distance = dist};
}

vec3<m> sphere::closest_point(const vec3<m> &p) const
{
    // return (p - M_position).normalized()*M_radius;
    return p.normalized() * M_radius;
}

bool sphere::contains(const vec3<m> &point) const
{
    // return (point - M_position).norm() <= M_radius;
    return point.norm() <= M_radius;
}

// point on sphere that gives greatest dot product with direction????
vec3<m> sphere::support(const vec3<one> &direction) const
{
    return direction.normalized() * M_radius;
}

} // namespace physkit