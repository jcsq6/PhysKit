#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/collision/shape.h"
#endif

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

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

// point on box that gives greatest dot product with direction????
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

    return {x, y, z};
}

} // namespace physkit