#include "physkit/box.h"

#include <cmath>
#include <stdexcept>

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

const aabb &box::bounds() const
{
    return M_aabb;
}

const bounding_sphere &box::bsphere() const
{
    return M_bsphere;
}


quantity<pow<3>(m)> box::volume() const
{
    return 2*M_half_extents.x()*2*M_half_extents.y()*2*M_half_extents.z();
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
vec3<m> box::mass_center() const
{
    return {0.0f*m, 0.0f*m, 0.0f*m};
}

mat3<kg * pow<2>(m)>
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
box::inertia_tensor(quantity<kg / pow<3>(m)> density) const
{
    auto mass = density*volume();
    auto x = M_half_extents.x()*2;
    auto y = M_half_extents.y()*2;
    auto z = M_half_extents.z()*2;
    auto i1 = ((mass/12) * (mp_units::pow<2>(y) + mp_units::pow<2>(z))).numerical_value_in(kg*m*m);
    auto i2 = ((mass/12) * (mp_units::pow<2>(x) + mp_units::pow<2>(z))).numerical_value_in(kg*m*m);
    auto i3 = ((mass/12) * (mp_units::pow<2>(x) + mp_units::pow<2>(y))).numerical_value_in(kg*m*m);

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
        auto p = i%2 == 0 ? M_half_extents[i/2] : -M_half_extents[i/2];
        vec3<m> point = vec3{0.0f*m, 0.0f*m, 0.0f*m};
        point[i/2] = p;
        vec3<one> n = point.normalized();

        //parallel case
        if (direction.dot(n) == 0.0f)
        {
            //contained on the plane or no hit.
            //we will detect the ray hit by checking the other planes.
            continue;
        }

        //other cases
        quantity<m> dist = ((point - origin).dot(n))/(direction.dot(n));
        if (dist < best.distance && dist >= 0.0f*m)
        {
            vec3<m> pos = origin+dist*direction;
            if (mp_units::abs(pos.x()) <= M_half_extents.x() &&
                mp_units::abs(pos.y()) <= M_half_extents.y() &&
                mp_units::abs(pos.z()) <= M_half_extents.z())
            {
                best.distance = dist;
                best.pos = origin+dist*direction;
                best.normal = n;
                hit = true;
            }
        }
    }

    if (hit)
        return best;
    return std::nullopt;
}

vec3<m> box::closest_point(const vec3<m> &p) const
{
    vec3<m> ret = p;

    //bind the point to the space inside the cube
    if (mp_units::abs(ret.x()) > M_half_extents.x())
        ret.x() = (ret.x() < 0.0f*m)? -M_half_extents.x() : M_half_extents.x();
    if (mp_units::abs(ret.y()) > M_half_extents.y())
        ret.y() = (ret.y() < 0.0f*m)? -M_half_extents.y() : M_half_extents.y();
    if (mp_units::abs(ret.z()) > M_half_extents.z())
        ret.z() = (ret.z() < 0.0f*m)? -M_half_extents.z() : M_half_extents.z();

    //bind the point to the edge of the cube
    quantity<m> dist = mp_units::abs(ret.x()) - M_half_extents.x();
    int j = 0;
    for (int i = 1; i < 3; i++)
    {
        quantity<m> d = mp_units::abs(ret[i]) - M_half_extents[i];
        if (dist < d)
        {
            dist = d;
            j = i;
        }
    }
    ret[j] = (ret[j] < 0.0f*m)? -M_half_extents[j] : M_half_extents[j];
    return ret;
}

bool box::contains(const vec3<m> &point) const
{
    return (mp_units::abs(point.x()) <= M_half_extents.x() &&
            mp_units::abs(point.y()) <= M_half_extents.y() &&
            mp_units::abs(point.z()) <= M_half_extents.z());
}

//point on sphere that gives greatest dot product with direction????
vec3<m> box::support(const vec3<one> &direction) const
{
    vec3<m> ret = direction.normalized()*M_half_extents.norm();
    if (mp_units::abs(ret.x()) > M_half_extents.x())
        ret.x() = (ret.x() < 0.0f*m)? -M_half_extents.x() : M_half_extents.x();
    if (mp_units::abs(ret.y()) > M_half_extents.y())
        ret.y() = (ret.y() < 0.0f*m)? -M_half_extents.y() : M_half_extents.y();
    if (mp_units::abs(ret.z()) > M_half_extents.z())
        ret.z() = (ret.z() < 0.0f*m)? -M_half_extents.z() : M_half_extents.z();

    return ret;
}

}
