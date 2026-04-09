#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/collision/shape.h"
#include <cmath>
#include <stdexcept>
#endif

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

const aabb &sphere::bounds() const { return M_aabb; }

const bounding_sphere &sphere::bsphere() const { return M_bsphere; }

quantity<pow<3>(m)> sphere::volume() const
{ return (4.0f / 3.0f) * std::numbers::pi * mp_units::pow<3>(M_radius); }

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
{ return direction.normalized() * M_radius; }

const aabb &box::bounds() const { return M_aabb; }

const bounding_sphere &box::bsphere() const { return M_bsphere; }

quantity<pow<3>(m)> box::volume() const
{ return 2 * M_half_extents.x() * 2 * M_half_extents.y() * 2 * M_half_extents.z(); }

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
vec3<m> box::mass_center() const { return {0.0f * m, 0.0f * m, 0.0f * m}; }

mat3<kg * pow<2>(m)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
box::inertia_tensor(quantity<kg / pow<3>(m)> density) const
{
    auto mass = density * volume();
    auto x = M_half_extents.x() * 2;
    auto y = M_half_extents.y() * 2;
    auto z = M_half_extents.z() * 2;
    auto i1 =
        ((mass / 12) * (mp_units::pow<2>(y) + mp_units::pow<2>(z))).numerical_value_in(kg * m * m);
    auto i2 =
        ((mass / 12) * (mp_units::pow<2>(x) + mp_units::pow<2>(z))).numerical_value_in(kg * m * m);
    auto i3 =
        ((mass / 12) * (mp_units::pow<2>(x) + mp_units::pow<2>(y))).numerical_value_in(kg * m * m);

    return mat3{{i1, 0.0f, 0.0f, 0.0f, i2, 0.0f, 0.0f, 0.0f, i3}} * kg * m * m;
}

// ray intersection
std::optional<ray::hit> box::ray_intersect(const ray &r, quantity<m> max_distance) const
{
    ray::hit best;
    auto origin = r.origin();
    auto direction = r.direction();
    bool hit = false;
    best.distance = max_distance;
    for (int i = 0; i < 6; i++)
    {
        auto p = i % 2 == 0 ? M_half_extents[i / 2] : -M_half_extents[i / 2];
        vec3<m> point = vec3{0.0f * m, 0.0f * m, 0.0f * m};
        point[i / 2] = p;
        vec3<one> n = point.normalized();

        // parallel case
        if (direction.dot(n) == 0.0f)
        {
            // contained on the plane or no hit.
            // we will detect the ray hit by checking the other planes.
            continue;
        }

        // other cases
        quantity<m> dist = ((point - origin).dot(n)) / (direction.dot(n));
        if (dist < best.distance && dist >= 0.0f * m)
        {
            vec3<m> pos = origin + dist * direction;
            if (mp_units::abs(pos.x()) <= M_half_extents.x() &&
                mp_units::abs(pos.y()) <= M_half_extents.y() &&
                mp_units::abs(pos.z()) <= M_half_extents.z())
            {
                best.distance = dist;
                best.pos = origin + dist * direction;
                best.normal = n;
                hit = true;
            }
        }
    }

    if (hit) return best;
    return std::nullopt;
}

vec3<m> box::closest_point(const vec3<m> &p) const
{
    auto x = p.x();
    auto y = p.y();
    auto z = p.z();

    // bind the point to the space inside the cube
    if (mp_units::abs(x) > M_half_extents.x())
        x = (x < 0.0f * m) ? -M_half_extents.x() : M_half_extents.x();
    if (mp_units::abs(x) > M_half_extents.y())
        y = (y < 0.0f * m) ? -M_half_extents.y() : M_half_extents.y();
    if (mp_units::abs(z) > M_half_extents.z())
        z = (z < 0.0f * m) ? -M_half_extents.z() : M_half_extents.z();

    // bind the point to the edge of the cube
    quantity<m> dist = mp_units::abs(x) - M_half_extents.x();
    int j = 0;
    for (int i = 1; i < 3; i++)
    {
        vec3<m> v = {x, y, z};
        quantity<m> d = mp_units::abs(v[i]) - M_half_extents[i];
        if (dist < d)
        {
            dist = d;
            j = i;
        }
    }
    switch (j)
    {
    default:
    case 0:
        x = (x < 0.0f * m) ? -M_half_extents.x() : M_half_extents.x();
        break;
    case 1:
        y = (y < 0.0f * m) ? -M_half_extents.y() : M_half_extents.y();
        break;
    case 2:
        z = (z < 0.0f * m) ? -M_half_extents.z() : M_half_extents.z();
        break;
    }
    vec3<m> ret = vec3(x, y, z);
    return ret;
}

bool box::contains(const vec3<m> &point) const
{
    return (mp_units::abs(point.x()) <= M_half_extents.x() &&
            mp_units::abs(point.y()) <= M_half_extents.y() &&
            mp_units::abs(point.z()) <= M_half_extents.z());
}

// point on sphere that gives greatest dot product with direction????
vec3<m> box::support(const vec3<one> &direction) const
{
    int best_i = 0;
    quantity<m> best = 0.0f * m;
    for (int i = 0; i < 8; i++)
    {
        auto x = (i < 4) ? -M_half_extents.x() : M_half_extents.x();
        auto y = ((i / 2) % 2 == 0) ? -M_half_extents.y() : M_half_extents.y();
        auto z = (i % 2 == 0) ? -M_half_extents.z() : M_half_extents.z();

        if (best < vec3(x, y, z).dot(direction))
        {
            best_i = i;
            best = vec3(x, y, z).dot(direction);
        }
    }
    auto x = (best_i < 4) ? -M_half_extents.x() : M_half_extents.x();
    auto y = ((best_i / 2) % 2 == 0) ? -M_half_extents.y() : M_half_extents.y();
    auto z = (best_i % 2 == 0) ? -M_half_extents.z() : M_half_extents.z();

    return vec3(x, y, z);
}

} // namespace physkit