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
        vec3<m> point = vec3{0.0, 0.0, 0.0} * m;
        point.set(i / 2, p);
        vec3<one> n = point.normalized();

        // parallel case
        if (direction.dot(n) == 0.0)
        {
            // contained on the plane or no hit.
            // we will detect the ray hit by checking the other planes.
            continue;
        }

        // other cases
        quantity<m> dist = ((point - origin).dot(n)) / (direction.dot(n));
        if (dist <= best.distance && dist >= 0.0 * m)
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

    // clamp the point to the box interior
    if (mp_units::abs(x) > M_half_extents.x())
        x = (x < 0.0 * m) ? -M_half_extents.x() : M_half_extents.x();
    if (mp_units::abs(y) > M_half_extents.y())
        y = (y < 0.0 * m) ? -M_half_extents.y() : M_half_extents.y();
    if (mp_units::abs(z) > M_half_extents.z())
        z = (z < 0.0 * m) ? -M_half_extents.z() : M_half_extents.z();

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
        x = (x < 0.0 * m) ? -M_half_extents.x() : M_half_extents.x();
        break;
    case 1:
        y = (y < 0.0 * m) ? -M_half_extents.y() : M_half_extents.y();
        break;
    case 2:
        z = (z < 0.0 * m) ? -M_half_extents.z() : M_half_extents.z();
        break;
    }
    vec3<m> ret = vec3(x, y, z);
    return ret;
}

[[nodiscard]] std::optional<ray::hit> pyramid::ray_intersect(const ray &r,
                                                             quantity<si::metre> max_distance) const
{
    using namespace mp_units::si::unit_symbols;
    std::optional<ray::hit> ret = std::nullopt;
    quantity<m> best = std::numeric_limits<quantity<si::metre>>::infinity();
    constexpr auto epsilon = 1e-10 * m;

    // base intersect
    if (r.direction().y() != 0)
    {
        quantity<m> dist = -r.origin().y() / r.direction().y();
        vec3<m> point = r.origin() + r.direction() * dist;
        if (dist >= 0.0 * si::metre && dist <= max_distance + epsilon &&
            abs(point.x()) <= M_base_half && abs(point.z()) <= M_base_half)
        {
            best = dist;
            ret = ray::hit{.pos = point, .normal = vec3<one>{0.0, -1.0, 0.0}, .distance = dist};
        }
    }

    // triangle intersections
    for (int i = 0; i < 4; i++)
    {
        quantity<one> y = M_base_half.numerical_value_in(m);
        quantity<one> x =
            i % 2 == 0 ? M_height.numerical_value_in(m) : -M_height.numerical_value_in(m);
        quantity<one> z =
            i % 2 == 0 ? M_height.numerical_value_in(m) : -M_height.numerical_value_in(m);
        if ((i / 2) % 2 == 0)
            z = 0.0;
        else
            x = 0.0;
        auto normal = (vec3<one>{x, y, z}).normalized();
        auto plane_point = (vec3<one>{x, 0, z}) * si::metre * M_base_half / M_height;
        if (r.direction().dot(normal) == 0.0) continue;

        quantity<m> dist = -(r.origin() - plane_point).dot(normal) / (r.direction().dot(normal));
        vec3<m> point = r.origin() + r.direction() * dist;

        if (dist >= 0.0 * si::metre && dist <= max_distance + epsilon && best > dist &&
            point.y() >= 0.0 * si::metre && point.y() <= M_height &&
            abs(point.x()) <= M_base_half * (1 - point.y() / M_height) &&
            abs(point.z()) <= M_base_half * (1 - point.y() / M_height))
        {
            best = dist;
            ret = ray::hit{.pos = point, .normal = normal, .distance = dist};
        }
    }
    return ret;
}

[[nodiscard]] vec3<si::metre> pyramid::closest_point(const vec3<si::metre> &point) const
{
    // tip point
    auto retx = 0.0 * si::metre;
    auto rety = M_height;
    auto retz = 0.0 * si::metre;
    auto best = (vec3<si::metre>{retx, rety, retz} - point).norm();

    // base point
    auto x = point.x();
    auto y = point.y();
    auto z = point.z();
    if (abs(x) > M_base_half) x = x < 0.0 * si::metre ? -M_base_half : M_base_half;
    if (abs(z) > M_base_half) z = z < 0.0 * si::metre ? -M_base_half : M_base_half;

    if (best > (vec3<si::metre>{x, 0.0 * si::metre, z} - point).norm())
    {
        best = (vec3{x, 0.0 * si::metre, z} - point).norm();
        retx = x;
        rety = 0.0 * si::metre;
        retz = z;
    }

    // edges
    for (int i = 0; i < 4; i++)
    {
        vec3<one> p =
            vec3{point.x().numerical_value_in(si::metre), point.y().numerical_value_in(si::metre),
                 point.z().numerical_value_in(si::metre)};
        y = 0.0 * si::metre;
        x = i % 2 == 0 ? M_base_half : -M_base_half;
        z = (i / 2) % 2 == 0 ? M_base_half : -M_base_half;
        vec3<one> temp_p = vec3{x.numerical_value_in(si::metre), y.numerical_value_in(si::metre),
                                z.numerical_value_in(si::metre)};
        vec3<one> temp_v =
            (vec3<one>{0.0, M_height.numerical_value_in(si::metre), 0.0} - temp_p).normalized();
        vec3<si::metre> temp = (temp_p + ((temp_v).dot(p - temp_p)) * temp_v) * si::metre;
        if (temp.y() >= 0.0 * si::metre && temp.y() <= M_height && best > (temp - point).norm())
        {
            best = (temp - point).norm();
            retx = temp.x();
            rety = temp.y();
            retz = temp.z();
        }
        if (best > (vec3{x, y, z} - point).norm())
        {
            best = (vec3{x, y, z} - point).norm();
            retx = x;
            rety = y;
            retz = z;
        }
    }

    // sides
    for (int i = 0; i < 4; i++)
    {
        y = M_base_half;
        x = i % 2 == 0 ? M_height : -M_height;
        z = i % 2 == 0 ? M_height : -M_height;
        if ((i / 2) % 2 == 0)
            z = 0.0 * si::metre;
        else
            x = 0.0 * si::metre;
        auto normal = (vec3<si::metre>{x, y, z}).normalized();
        auto plane_point =
            point - (normal * normal.dot(point - vec3<si::metre>{x, 0.0 * si::metre, z}));
        if (point.y() < 0.0 * si::metre || point.y() > M_height) continue;
        if (abs(plane_point.x()) > M_base_half * (1.0 - plane_point.y() / M_height)) continue;
        if (abs(plane_point.z()) > M_base_half * (1.0 - plane_point.y() / M_height)) continue;

        if (best > (plane_point - point).norm())
        {
            best = (plane_point - point).norm();
            retx = plane_point.x();
            rety = plane_point.y();
            retz = plane_point.z();
        }
    }
    return vec3<si::metre>{retx, rety, retz};
}

[[nodiscard]] vec3<si::metre> cone::closest_point(const vec3<si::metre> &point) const
{
    using namespace mp_units::si::unit_symbols;

    // Reduce to 2D: (d, py) where d = distance from the y-axis
    auto d = sqrt(point.x() * point.x() + point.z() * point.z());

    // Direction from axis in xz plane (for converting back to 3D)
    auto dir_x = d > 0 * m ? point.x() / d : 1.0 * one;
    auto dir_z = d > 0 * m ? point.z() / d : 0.0 * one;

    // Candidate 1: closest point on the base disk ($y=0, 0 \leq \text{dist} \leq r$)
    auto base_d = d < M_radius ? d : M_radius;
    auto base_dist2 = (d - base_d) * (d - base_d) + point.y() * point.y();

    // Candidate 2: closest point on slant from ($r$,0) to (0,$h$)
    // $P(t) = ((1-t)r, th)$ for $t \in [0,1]$
    // $u = (d-r, py), v = (-r, h), t = u \cdot v / v \cdot v$
    auto t = (-M_radius * (d - M_radius) + M_height * point.y()) /
             (M_radius * M_radius + M_height * M_height);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    auto slant_d = (1.0 - t) * M_radius;
    auto slant_y = t * M_height;
    auto slant_dist2 =
        (d - slant_d) * (d - slant_d) + (point.y() - slant_y) * (point.y() - slant_y);

    quantity<si::metre> res_d; // NOLINT
    quantity<si::metre> res_y; // NOLINT
    if (base_dist2 <= slant_dist2)
    {
        res_d = base_d;
        res_y = 0.0 * si::metre;
    }
    else
    {
        res_d = slant_d;
        res_y = slant_y;
    }

    return vec3{res_d * dir_x, res_y, res_d * dir_z};
}

[[nodiscard]] std::optional<ray::hit> cone::ray_intersect(const ray &r,
                                                          quantity<si::metre> max_distance) const
{
    using namespace mp_units::si::unit_symbols;
    std::optional<ray::hit> best;

    auto o = r.origin();
    auto dir = r.direction();

    // Base plane intersection (y = 0)
    if (dir.y() != 0)
        if (auto dist = -o.y() / dir.y(); dist >= 0 * m && dist <= max_distance)
            if (auto point = o + dist * dir;
                pow<2>(point.x()) + pow<2>(point.z()) <= pow<2>(M_radius))
                best = ray::hit{.pos = point, .normal = vec3<one>{0, -1, 0}, .distance = dist};

    // Lateral surface intersection: $x^2 + z^2 = (r/h)^2(h - y)^2$
    auto rad = M_radius;
    auto h = M_height;

    auto a = dir.x() * dir.x() + dir.z() * dir.z() - dir.y() * dir.y() * rad * rad / (h * h);
    auto b = 2 * o.x() * dir.x() + 2 * o.z() * dir.z() - 2 * rad * rad * o.y() * dir.y() / (h * h) +
             2 * dir.y() * rad * rad / h;
    auto c = o.x() * o.x() + o.z() * o.z() - o.y() * o.y() * rad * rad / (h * h) +
             2 * o.y() * rad * rad / h - rad * rad;

    auto det = b * b - 4 * a * c;
    if (det >= 0 * m * m)
    {
        auto sqrt_det = sqrt(det);
        auto t1 = (-b - sqrt_det) / (2 * a);
        auto t2 = (-b + sqrt_det) / (2 * a);
        if (t1 > t2) std::swap(t1, t2);

        for (auto t : {t1, t2})
        {
            auto dist = t;
            if (dist < 0 * m || dist > max_distance) continue;
            if (best.has_value() && best->distance <= dist) break;

            auto p = o + dist * dir;
            // Reject hits outside the finite cone
            if (p.y() < 0 * m || p.y() > M_height) continue;

            // Outward normal: $\nabla(x^2+z^2-(r/h)^2(h-y)^2) \propto (x, (r^2/h^2)(h-y), z)$

            vec3<one> norm;
            if (p.x() == 0.0 * m && p.z() == 0.0 * m)
                norm = vec3<one>{0.0, 1.0, 0.0};
            else
                norm = vec3{p.x(), rad * rad * (h - p.y()) / (h * h), p.z()}.normalized();

            best = ray::hit{.pos = p, .normal = norm, .distance = dist};
            break;
        }
    }

    return best;
}

[[nodiscard]] vec3<si::metre> cylinder::closest_point(const vec3<si::metre> &point) const
{
    auto [x, y, z] = point;
    const auto d2 = x * x + z * z;
    const auto d = sqrt(d2);
    // const auto r2 = M_radius * M_radius;
    const auto half_height =
        M_height * 0.5; // no fp error because this just decrements the exponent.
    y = std::clamp(y, -half_height, half_height);
    if (d2 > M_radius * M_radius)
    {
        // project to side
        auto s =
            M_radius / d; // d can't be 0 here even if the cylinder was 0 radius for some reason.
        x *= s;
        z *= s;
    }
    else if (abs(y) < half_height)
    {
        // this is only if strictly inside caps
        auto side = M_radius - d;              // should be positive.
        auto caps = abs(half_height - abs(y)); // y <
        if (side >= caps)
        { // prioritize caps y axis for faser code.
            y = (y > 0.0 * si::metre) ? (half_height) : (-half_height);
        }
        else
        {
            if (d > 0.0 * si::metre)
            {
                // project to side
                auto s = M_radius / d;
                x *= s;
                z *= s;
            }
            else
            { // on y-axis but need project to side. chose +x axis for default.
                x = M_radius;
                z = 0.0 * si::metre;
            }
        }
    }

    return vec3{x, y, z};
}

[[nodiscard]] std::optional<ray::hit>
cylinder::ray_intersect(const ray &r, quantity<si::metre> max_distance) const
{
    std::optional<ray::hit> best;

    const auto &o = r.origin();
    const auto &d = r.direction();
    const auto &[ox, oy, oz] = o;
    const auto &[dx, dy, dz] = d;
    // const auto [x1,y1,z1] = r.origin() + r.direction() * max_distance;
    const auto half_h = M_height * 0.5;

    auto a = dx * dx + dz * dz;                       // m2
    auto b = 2 * (ox * dx + oz * dz);                 // m2
    auto c = ox * ox + oz * oz - M_radius * M_radius; // m2

    if (a != 0.0 * one)
    {
        auto det = b * b - 4 * a * c; // m4

        if (det >= 0.0 * si::metre * si::metre)
        {
            auto sqrt_det = sqrt(det); // m2

            auto t1 = (-b - sqrt_det) / (2 * a); // m
            auto t2 = (-b + sqrt_det) / (2 * a); // m

            if (t1 > t2) std::swap(t1, t2);

            for (auto t : {t1, t2}) // t -> m
            {
                if (t < (0.0 * si::metre) || t > max_distance) continue;

                if (best && best->distance <= t) break;

                auto p = o + t * d; // point of possible intersection. m

                if (p.y() < -half_h || p.y() > half_h) // out of bounds.
                    continue;

                vec3<one> norm;

                if (p.x() == (0.0 * si::metre) && p.z() == (0.0 * si::metre))
                {
                    norm = vec3<one>{0, 1, 0};
                }
                else
                {
                    norm = vec3{p.x(), 0.0 * si::metre, p.z()}.normalized();
                }

                best = ray::hit{.pos = p, .normal = norm, .distance = t};

                break;
            }
        }
    }

    // caps
    if (dy != 0.0 * one)
    {
        // bottom cap y = -h/2
        {
            auto t = (-half_h - oy) / dy;

            if (t >= (0.0 * si::metre) && t <= max_distance)
            {
                auto p = o + t * d;

                if (p.x() * p.x() + p.z() * p.z() <= M_radius * M_radius)
                {
                    if (!best || t < best->distance)
                    {
                        best = ray::hit{.pos = p, .normal = vec3<one>{0, -1, 0}, .distance = t};
                    }
                }
            }
        }

        // top cap y = +h/2
        {
            auto t = (half_h - oy) / dy;

            if (t >= (0.0 * si::metre) && t <= max_distance)
            {
                auto p = o + t * d;

                if (p.x() * p.x() + p.z() * p.z() <= M_radius * M_radius)
                {
                    if (!best || t < best->distance)
                    {
                        best = ray::hit{.pos = p, .normal = vec3<one>{0, 1, 0}, .distance = t};
                    }
                }
            }
        }
    }

    return best;
}

} // namespace physkit