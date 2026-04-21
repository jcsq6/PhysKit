#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/collision/shape.h"
#endif

namespace physkit
{
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

// slab method
std::optional<ray::hit> box::ray_intersect(const ray &r, quantity<m> max_distance) const
{
    const auto &o = r.origin();
    const auto &d = r.direction();

    auto tmin = -std::numeric_limits<quantity<m>>::infinity();
    auto tmax = std::numeric_limits<quantity<m>>::infinity();
    int axis_min = 0;
    int axis_max = 0;
    double sign_min = 0.0;
    double sign_max = 0.0;

    for (int i = 0; i < 3; ++i)
    {
        auto half = M_half_extents[i];
        auto oi = o[i];
        auto di = d[i];

        if (di == 0.0)
        {
            if (mp_units::abs(oi) > half) return std::nullopt;
            continue;
        }

        auto inv = 1.0 / di;
        auto t1 = (-half - oi) * inv;
        auto t2 = (half - oi) * inv;
        double s1 = -1.0;
        double s2 = 1.0;
        if (t1 > t2)
        {
            std::swap(t1, t2);
            std::swap(s1, s2);
        }
        if (t1 > tmin)
        {
            tmin = t1;
            axis_min = i;
            sign_min = s1;
        }
        if (t2 < tmax)
        {
            tmax = t2;
            axis_max = i;
            sign_max = s2;
        }
        if (tmin > tmax) return std::nullopt;
    }

    // ray starts inside
    int axis = axis_min;
    double sign = sign_min;
    auto dist = tmin;
    if (dist < 0.0 * m)
    {
        axis = axis_max;
        sign = sign_max;
        dist = tmax;
    }
    if (dist < 0.0 * m || dist > max_distance) return std::nullopt;

    vec3<one> normal{0.0, 0.0, 0.0};
    normal.set(axis, sign);
    return ray::hit{.pos = o + dist * d, .normal = normal, .distance = dist};
}

vec3<m> box::closest_point(const vec3<m> &p) const
{
    vec3<m> clamped{std::clamp(p.x(), -M_half_extents.x(), M_half_extents.x()),
                    std::clamp(p.y(), -M_half_extents.y(), M_half_extents.y()),
                    std::clamp(p.z(), -M_half_extents.z(), M_half_extents.z())};

    if (clamped != p) return clamped;

    int axis = 0;
    auto min_slack = M_half_extents[0] - mp_units::abs(p[0]);
    for (int i = 1; i < 3; ++i)
    {
        auto slack = M_half_extents[i] - mp_units::abs(p[i]);
        if (slack < min_slack)
        {
            min_slack = slack;
            axis = i;
        }
    }
    clamped.set(axis, p[axis] < 0.0 * m ? -M_half_extents[axis] : M_half_extents[axis]);
    return clamped;
}

[[nodiscard]] std::optional<ray::hit> pyramid::ray_intersect(const ray &r,
                                                             quantity<si::metre> max_distance) const
{
    std::optional<ray::hit> best;
    auto update = [&](quantity<m> dist, const vec3<m> &point, const vec3<one> &normal)
    {
        if (dist < 0.0 * m || dist > max_distance) return;
        if (best && best->distance <= dist) return;
        best = ray::hit{.pos = point, .normal = normal, .distance = dist};
    };

    const auto &o = r.origin();
    const auto &d = r.direction();

    // base face
    if (d.y() != 0.0)
    {
        auto dist = -o.y() / d.y();
        auto point = o + dist * d;
        if (abs(point.x()) <= M_base_half && abs(point.z()) <= M_base_half)
            update(dist, point, vec3<one>{0.0, -1.0, 0.0});
    }

    // slant faces
    static const auto face_axes = std::array{
        vec3<one>{1.0, 0.0, 0.0},
        vec3<one>{-1.0, 0.0, 0.0},
        vec3<one>{0.0, 0.0, 1.0},
        vec3<one>{0.0, 0.0, -1.0},
    };
    const auto hval = M_height;
    const auto bval = M_base_half;
    const auto rhs = bval * hval;

    for (const auto &axis : face_axes)
    {
        vec3<m> n_unn{axis.x() * hval, bval, axis.z() * hval};
        auto denom = d.dot(n_unn);
        if (denom == 0.0 * m) continue;

        auto num = rhs - o.dot(n_unn);
        auto dist = num / denom;
        auto point = o + dist * d;
        if (point.y() < 0.0 * m || point.y() > hval) continue;
        auto slice = bval * (1.0 - point.y() / hval);
        if (abs(point.x()) > slice + 1e-12 * m) continue;
        if (abs(point.z()) > slice + 1e-12 * m) continue;

        update(dist, point, n_unn.normalized());
    }

    return best;
}

[[nodiscard]] vec3<si::metre> pyramid::closest_point(const vec3<si::metre> &point) const
{
    const auto b = M_base_half;
    const auto h = M_height;

    static const std::array<vec3<si::metre>, 5> verts{
        vec3{b, 0.0 * m, b},  vec3{-b, 0.0 * m, b},      vec3{-b, 0.0 * m, -b},
        vec3{b, 0.0 * m, -b}, vec3{0.0 * m, h, 0.0 * m},
    };

    static constexpr std::array<triangle_t, 6> tris{
        triangle_t{0, 4, 1}, // +z slant
        triangle_t{1, 4, 2}, // -x slant
        triangle_t{2, 4, 3}, // -z slant
        triangle_t{3, 4, 0}, // +x slant
        triangle_t{0, 1, 2}, // base half #1
        triangle_t{0, 2, 3}, // base half #2
    };

    auto best = tris[0].closest_point(point, std::span{verts});
    auto best_dist2 = (best - point).squared_norm();
    for (std::size_t i = 1; i < tris.size(); ++i)
    {
        auto candidate = tris[i].closest_point(point, std::span{verts});
        auto dist2 = (candidate - point).squared_norm();
        if (dist2 < best_dist2)
        {
            best = candidate;
            best_dist2 = dist2;
        }
    }
    return best;
}

[[nodiscard]] vec3<si::metre> cone::closest_point(const vec3<si::metre> &point) const
{
    using namespace mp_units::si::unit_symbols;

    // Reduce to 2D: (d, py) where d = distance from the y-axis
    auto d = sqrt(point.x() * point.x() + point.z() * point.z());

    // Direction from axis in xz plane (for converting back to 3D)
    auto dir_x = d > 0 * m ? point.x() / d : 1.0;
    auto dir_z = d > 0 * m ? point.z() / d : 0.0;

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

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
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
        if (side < caps)
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
        else // prioritize caps y axis for speed.
            y = (y > 0.0 * si::metre) ? (half_height) : (-half_height);
    }

    return vec3{x, y, z};
}

[[nodiscard]] std::optional<ray::hit>
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
cylinder::ray_intersect(const ray &r, quantity<si::metre> max_distance) const
{
    std::optional<ray::hit> best;

    const auto &o = r.origin();
    const auto &d = r.direction();
    const auto half_h = M_height * 0.5;

    auto a = d.x() * d.x() + d.z() * d.z();                       // m2
    auto b = 2 * (o.x() * d.x() + o.z() * d.z());                 // m2
    auto c = o.x() * o.x() + o.z() * o.z() - M_radius * M_radius; // m2

    if (a != 0.0)
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
                    norm = vec3<one>{0, 1, 0};
                else
                    norm = vec3{p.x(), 0.0 * si::metre, p.z()}.normalized();

                best = ray::hit{.pos = p, .normal = norm, .distance = t};

                break;
            }
        }
    }

    // caps
    if (d.y() != 0.0)
    {
        // bottom cap y = -h/2
        if (auto t = (-half_h - o.y()) / d.y(); t >= (0.0 * si::metre) && t <= max_distance)
            if (auto p = o + t * d; p.x() * p.x() + p.z() * p.z() <= M_radius * M_radius)
                if (!best || t < best->distance)
                    best = ray::hit{.pos = p, .normal = vec3<one>{0, -1, 0}, .distance = t};

        // top cap y = +h/2
        if (auto t = (half_h - o.y()) / d.y(); t >= (0.0 * si::metre) && t <= max_distance)
            if (auto p = o + t * d; p.x() * p.x() + p.z() * p.z() <= M_radius * M_radius)
                if (!best || t < best->distance)
                    best = ray::hit{.pos = p, .normal = vec3<one>{0, 1, 0}, .distance = t};
    }

    return best;
}

} // namespace physkit