#pragma once
#include "object.h"
#include "physkit/detail/arena.h"
#include "physkit/detail/collision_phases.h"
#include "physkit/integrator.h"

// TODO: Extended Position Based Dynamics (XPBD) constraints.
// https://www.emergentmind.com/topics/extended-position-based-dynamics-xpbd
// TODO: Block Gauss-Seidel solver for better convergence

namespace physkit
{

enum class constraint_type : std::uint8_t
{
    distance = 0,
    ball_socket,
    hinge,
    slider,
    weld,
    contact,
    DetailCount,
};

namespace impulse
{

struct jacobian_row
{
    vec3<one> J_v;
    vec3<si::metre> J_w_a;
    vec3<si::metre> J_w_b;
    quantity<si::kilogram> M_eff{};
    quantity<si::metre / si::second> bias{};

    // For warm starting
    quantity<si::newton * si::second> accumulated_impulse{};
    quantity<si::newton * si::second> impulse_min =
        -std::numeric_limits<quantity<si::newton * si::second>>::max();
    quantity<si::newton * si::second> impulse_max =
        std::numeric_limits<quantity<si::newton * si::second>>::max();

    object *a{};
    object *b{};

    void apply()
    {
        // NOLINTNEXTLINE(readability-identifier-naming)
        auto Jv = J_v.dot(a->vel()) + (J_w_a.dot(a->ang_vel()) / si::radian) - J_v.dot(b->vel()) +
                  (J_w_b.dot(b->ang_vel()) / si::radian);
        auto lambda = -(Jv + bias) * M_eff;
        auto old_impulse = accumulated_impulse;
        accumulated_impulse = std::clamp(old_impulse + lambda, impulse_min, impulse_max);

        auto applied_lambda = (accumulated_impulse - old_impulse);
        a->apply_impulse(J_v * applied_lambda);
        a->apply_angular_impulse(J_w_a * applied_lambda);
        b->apply_impulse(-J_v * applied_lambda);
        b->apply_angular_impulse(J_w_b * applied_lambda);
    }

    void warm_start() const
    {
        if (accumulated_impulse == 0 * si::newton * si::second) return;

        a->apply_impulse(J_v * accumulated_impulse);
        a->apply_angular_impulse(J_w_a * accumulated_impulse);
        b->apply_impulse(-J_v * accumulated_impulse);
        b->apply_angular_impulse(J_w_b * accumulated_impulse);
    }
};

class constraint_base;

namespace detail
{

template <typename T> class passkey
{
    friend T;
    passkey() = default;
};

template <typename T>
concept constraint_impl = requires(const std::remove_cvref_t<T> &c, quantity<si::second> dt,
                                   std::vector<jacobian_row> &rows, passkey<constraint_base> pk) {
    { std::remove_cvref_t<T>::jacobian_dimension } -> std::convertible_to<std::size_t>;
    { c.build_jacobian_impl(dt, rows, pk) };
};

// "Building an Orthonormal Basis, Revisited" (2017).
static std::pair<vec3<one>, vec3<one>> build_orthonormal_basis(const vec3<one> &n)
{
    double sign = std::copysign(1.0f, static_cast<double>(n.z()));

    const auto a = -1.0f / (sign + n.z());
    const auto b = n.x() * n.y() * a;

    return {vec3(1.0f + sign * n.x() * n.x() * a, sign * b, -sign * n.x()),
            vec3(b, sign + n.y() * n.y() * a, -n.y())};
}
} // namespace detail

// \frac{dC}{dt} = Jv = 0, where v=[v_a, \omega_a, v_b, \omega_b]^T
class constraint_base
{
public:
    constraint_base(object &a, object &b) : M_a(&a), M_b(&b) {}

    template <detail::constraint_impl Derived>
        requires(std::derived_from<std::remove_cvref_t<Derived>, constraint_base>)
    void build_jacobian(this Derived &&self, quantity<si::second> dt,
                        std::vector<jacobian_row> &rows)
    {
        std::forward<Derived>(self).build_jacobian_impl(dt, rows, {});
    }

    [[nodiscard]] auto &obj_a() const { return *M_a; }
    [[nodiscard]] auto &obj_b() const { return *M_b; }

private:
    object *M_a{};
    object *M_b{};
};

// for a,b as anchor points, where a=x_a+r_a and b=x_b+r_b (where x is center of mass and r is local
// offset), and distance d: C=||a-b||-d=0 For u=\frac{a-b}{||a-b||} as the normalized direction from
// b to a, we have Jacobian J=[u^T, (r_a \times u)^T, -u^T, -(r_b \times u)^T]
class distance_constraint : public constraint_base
{
public:
    static constexpr auto static_type = constraint_type::distance;
    static constexpr auto bias_factor = .1;

    static constexpr auto jacobian_dimension = 1;

    distance_constraint(object &a, object &b, const vec3<si::metre> &local_anchor_a,
                        const vec3<si::metre> &local_anchor_b, quantity<si::metre> distance)
        : constraint_base(a, b), M_local_a(local_anchor_a), M_local_b(local_anchor_b),
          M_distance(distance)
    {
    }

    auto build_jacobian_impl(quantity<si::second> dt, std::vector<jacobian_row> &rows,
                             detail::passkey<constraint_base> /*unused*/) const
    {
        // NOLINTBEGIN(readability-identifier-naming)
        static constexpr auto dist_epsilon = 1e-6 * si::metre;
        static constexpr auto mass_epsilon = 1e-6 / si::kilogram;

        auto &a = obj_a();
        auto &b = obj_b();

        // world-space anchor points
        auto r_a = a.orientation() * M_local_a;
        auto r_b = b.orientation() * M_local_b;
        auto p_a = a.pos() + r_a;
        auto p_b = b.pos() + r_b;

        auto delta = p_a - p_b;
        auto cur_dist = delta.norm();
        if (cur_dist < dist_epsilon) return;

        auto u = delta / cur_dist;

        // J = [u^T, (r_a \times u)^T, -u^T, -(r_b \times u)^T]
        jacobian_row row{
            .J_v = u,
            .J_w_a = r_a.cross(u),
            .J_w_b = -r_b.cross(u),
            .a = &a,
            .b = &b,
        };

        // I^-1 * (r \times u)
        auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
        auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

        row.M_eff = 1.0 / (a.inv_mass() + b.inv_mass() + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));

        // Baumgarte stabilization
        row.bias = (bias_factor / dt) * (cur_dist - M_distance);

        rows.push_back(row);

        // NOLINTEND(readability-identifier-naming)
    }

    vec3<si::metre> M_local_a;
    vec3<si::metre> M_local_b;
    quantity<si::metre> M_distance;
};

// For a, b as anchor points, r_a is the skew-symettric matrix of vector r:
// C=a-b=0
// J=[I, -r_a, -I, r_b]
class ball_socket_constraint : public constraint_base
{
public:
    static constexpr auto static_type = constraint_type::ball_socket;
    static constexpr auto bias_factor = .1;
    static constexpr auto jacobian_dimension = 3;

    ball_socket_constraint(object &a, object &b, const vec3<si::metre> &anchor)
        : constraint_base(a, b)
    {
        M_local_a = a.project_to_local(anchor);
        M_local_b = b.project_to_local(anchor);
    }

    void build_jacobian_impl(quantity<si::second> dt, std::vector<jacobian_row> &rows,
                             detail::passkey<constraint_base> /*unused*/) const
    {
        auto &a = obj_a();
        auto &b = obj_b();

        // world-space anchor points
        auto r_a = a.orientation() * M_local_a;
        auto r_b = b.orientation() * M_local_b;

        auto p_a = a.pos() + r_a;
        auto p_b = b.pos() + r_b;

        auto delta = p_a - p_b;

        // J = [I, -r_a, -I, r_b]
        for (const auto &n : {vec3{1, 0, 0}, vec3{0, 1, 0}, vec3{0, 0, 1}})
        {
            jacobian_row row{
                .J_v = n,
                .J_w_a = r_a.cross(n),
                .J_w_b = -r_b.cross(n),
                .a = &a,
                .b = &b,
            };

            // I^-1 * (r \times u)
            auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
            auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

            row.M_eff =
                1.0 / (a.inv_mass() + b.inv_mass() + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));

            // Baumgarte stabilization
            row.bias = (bias_factor / dt) * delta.dot(n);

            rows.push_back(row);
        }
    }

private:
    vec3<si::metre> M_local_a;
    vec3<si::metre> M_local_b;
};

class hinge_constraint : public constraint_base
{
public:
    static constexpr auto static_type = constraint_type::hinge;
    static constexpr auto bias_factor = .2;
    static constexpr auto jacobian_dimension = 5;

    hinge_constraint(object &a, object &b, const vec3<si::metre> &local_anchor_a,
                     const vec3<si::metre> &local_anchor_b, const vec3<one> &local_axis_a,
                     const vec3<one> &local_axis_b)
        : constraint_base(a, b), M_local_anchor_a(local_anchor_a), M_local_anchor_b(local_anchor_b),
          M_local_axis_a(local_axis_a.normalized()), M_local_axis_b(local_axis_b.normalized())
    {
        std::tie(M_local_v_a, M_local_w_a) = detail::build_orthonormal_basis(M_local_axis_a);
    }

    void build_jacobian_impl(quantity<si::second> dt, std::vector<jacobian_row> &rows,
                             detail::passkey<constraint_base> /*unused*/) const
    {
        auto &a = obj_a();
        auto &b = obj_b();

        // world-space anchor points
        auto r_a = a.orientation() * M_local_anchor_a;
        auto r_b = b.orientation() * M_local_anchor_b;
        auto p_a = a.pos() + r_a;
        auto p_b = b.pos() + r_b;
        auto delta = p_a - p_b;

        auto inv_mass = a.inv_mass() + b.inv_mass();
        const auto &inv_inertia_a = a.world_inv_inertia_tensor();
        const auto &inv_inertia_b = b.world_inv_inertia_tensor();

        // Positional rows: keep anchors coincident.
        for (const auto &n : {vec3{1, 0, 0}, vec3{0, 1, 0}, vec3{0, 0, 1}})
        {
            jacobian_row row{
                .J_v = n,
                .J_w_a = r_a.cross(n),
                .J_w_b = -r_b.cross(n),
                .a = &a,
                .b = &b,
            };

            auto k_a = inv_inertia_a * row.J_w_a;
            auto k_b = inv_inertia_b * row.J_w_b;

            row.M_eff = 1.0 / (inv_mass + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            row.bias = (bias_factor / dt) * delta.dot(n);
            rows.push_back(row);
        }

        auto u_a = a.orientation() * M_local_axis_a;
        auto u_b = b.orientation() * M_local_axis_b;
        auto angular_error = u_a.cross(u_b);

        // Rotational rows: remove 2 angular DOF orthogonal to hinge axis.
        auto v_a = a.orientation() * M_local_v_a;
        auto w_a = a.orientation() * M_local_w_a;

        static constexpr auto angular_scale = 1.0 * si::metre;
        for (const auto &n : {v_a, w_a})
        {
            auto jt = n * angular_scale;
            jacobian_row row{
                .J_v = vec3<one>::zero(),
                .J_w_a = jt,
                .J_w_b = -jt,
                .a = &a,
                .b = &b,
            };

            auto k_a = inv_inertia_a * row.J_w_a;
            auto k_b = inv_inertia_b * row.J_w_b;

            row.M_eff = 1.0 / (row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            row.bias = (bias_factor / dt) * (angular_error.dot(n) * angular_scale);
            rows.push_back(row);
        }
    }

private:
    vec3<si::metre> M_local_anchor_a;
    vec3<si::metre> M_local_anchor_b;
    vec3<one> M_local_axis_a;
    vec3<one> M_local_axis_b;
    vec3<one> M_local_v_a;
    vec3<one> M_local_w_a;
};

class slider_constraint : public constraint_base
{
public:
    static constexpr auto static_type = constraint_type::slider;
    static constexpr auto bias_factor = .1;
    static constexpr auto jacobian_dimension = 5;

    slider_constraint(object &a, object &b, const vec3<si::metre> &world_anchor,
                      const vec3<one> &world_axis)
        : constraint_base(a, b), M_local_a(a.project_to_local(world_anchor)),
          M_local_b(b.project_to_local(world_anchor))
    {
        auto axis_n = world_axis.normalized();
        M_local_axis_a = a.orientation().conjugate() * axis_n;
        M_local_axis_b = b.orientation().conjugate() * axis_n;

        std::tie(M_local_t1_a, M_local_t2_a) = detail::build_orthonormal_basis(M_local_axis_a);
        std::tie(M_local_t1_b, M_local_t2_b) = detail::build_orthonormal_basis(M_local_axis_b);
    }

    void build_jacobian_impl(quantity<si::second> dt, std::vector<jacobian_row> &rows,
                             detail::passkey<constraint_base> /*unused*/) const
    {
        auto &a = obj_a();
        auto &b = obj_b();

        // world-space anchor points
        auto r_a = a.orientation() * M_local_a;
        auto r_b = b.orientation() * M_local_b;
        auto delta = (a.pos() + r_a) - (b.pos() + r_b);

        auto axis_a = a.orientation() * M_local_axis_a;
        auto axis_b = b.orientation() * M_local_axis_b;

        auto t1_a = a.orientation() * M_local_t1_a;
        auto t2_a = a.orientation() * M_local_t2_a;
        auto t1_b = b.orientation() * M_local_t1_b;
        auto t2_b = b.orientation() * M_local_t2_b;

        // linear constraint: restrict movement perpendiclar to slider axis
        for (const auto &n : {t1_a, t2_a})
        {
            jacobian_row row{
                .J_v = n,
                .J_w_a = (r_a - delta).cross(n),
                .J_w_b = -r_b.cross(n),
                .a = &a,
                .b = &b,
            };

            auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
            auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

            row.M_eff =
                1.0 / (a.inv_mass() + b.inv_mass() + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            row.bias = (bias_factor / dt) * delta.dot(n);
            rows.push_back(row);
        }

        constexpr auto angular_scale = 1.0 * si::metre;

        // angular constraint: keep orientations aligned
        auto angular_error = .5 * (axis_a.cross(axis_b) + t1_a.cross(t1_b) + t2_a.cross(t2_b));

        for (const auto &n : {axis_a, t1_a, t2_a})
        {
            auto jt = n * angular_scale;
            jacobian_row row{
                .J_v = vec3<one>::zero(),
                .J_w_a = jt,
                .J_w_b = -jt,
                .a = &a,
                .b = &b,
            };

            auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
            auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

            row.M_eff = 1.0 / (row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            row.bias = (bias_factor / dt) * (angular_error.dot(n) * angular_scale);
            rows.push_back(row);
        }
    }

private:
    vec3<si::metre> M_local_a;
    vec3<si::metre> M_local_b;
    vec3<one> M_local_axis_a;
    vec3<one> M_local_axis_b;
    vec3<one> M_local_t1_a;
    vec3<one> M_local_t2_a;
    vec3<one> M_local_t1_b;
    vec3<one> M_local_t2_b;
};

// TODO: compliant weld constraint
class weld_constraint : public constraint_base
{
public:
    static constexpr auto static_type = constraint_type::weld;
    static constexpr auto bias_factor = .1;
    static constexpr auto jacobian_dimension = 6;

    weld_constraint(object &a, object &b, const vec3<si::metre> &world_anchor)
        : constraint_base(a, b), M_local_anchor_a(a.project_to_local(world_anchor)),
          M_local_anchor_b(b.project_to_local(world_anchor)),
          M_local_orientation_rel(a.orientation().conjugate() * b.orientation())
    {
    }

    void build_jacobian_impl(quantity<si::second> dt, std::vector<jacobian_row> &rows,
                             detail::passkey<constraint_base> /*unused*/) const
    {
        auto &a = obj_a();
        auto &b = obj_b();

        // linear constraints
        auto r_a = a.orientation() * M_local_anchor_a;
        auto r_b = b.orientation() * M_local_anchor_b;
        auto delta = (a.pos() + r_a) - (b.pos() + r_b);

        static auto axes = std::array{vec3{1, 0, 0}, vec3{0, 1, 0}, vec3{0, 0, 1}};

        // Positional rows: keep anchors coincident.
        for (const auto &n : axes)
        {
            jacobian_row row{
                .J_v = n,
                .J_w_a = r_a.cross(n),
                .J_w_b = -r_b.cross(n),
                .a = &a,
                .b = &b,
            };

            // I^-1 * (r \times u)
            auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
            auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

            row.M_eff =
                1.0 / (a.inv_mass() + b.inv_mass() + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            row.bias = (bias_factor / dt) * delta.dot(n);
            rows.push_back(row);
        }

        // angular constraints: keep minimal initial relative orientation
        auto cur_rel_orientation = a.orientation().conjugate() * b.orientation();
        auto orientation_error = M_local_orientation_rel * cur_rel_orientation.conjugate();
        if (orientation_error.w() < 0)
            orientation_error =
                quat{-orientation_error.w(), -orientation_error.x(), -orientation_error.y(),
                     -orientation_error.z()}; // Ensure shortest path

        auto angular_error_world = a.orientation() * (orientation_error.vec() * 2); // TODO: need 2?
        constexpr auto angular_scale = 1.0 * si::metre;
        for (const auto &n : axes)
        {
            auto jt = n * angular_scale;
            jacobian_row row{
                .J_v = vec3<one>::zero(),
                .J_w_a = jt,
                .J_w_b = -jt,
                .a = &a,
                .b = &b,
            };

            auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
            auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

            row.M_eff = 1.0 / (row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            row.bias = (bias_factor / dt) * angular_error_world.dot(n) * angular_scale;
            rows.push_back(row);
        }
    }

private:
    vec3<si::metre> M_local_anchor_a;
    vec3<si::metre> M_local_anchor_b;
    quat<one> M_local_orientation_rel;
};

struct contact_jacobian
{
    jacobian_row normal;
    jacobian_row tangent1;
    jacobian_row tangent2;

    vec3<one> old_tangent1;
    vec3<one> old_tangent2;
};

inline std::optional<contact_jacobian>
build_contact_jacobian(object &a, object &b, const vec3<si::metre> &local_contact_a,
                       const vec3<si::metre> &local_contact_b,
                       const vec3<one> &world_contact_normal, quantity<si::second> dt)
{
    constexpr auto bias_factor = .2;
    constexpr auto linear_slop = 0.005 * si::metre;

    auto local_normal_a = a.orientation().conjugate() * world_contact_normal.normalized();
    auto local_normal_b = b.orientation().conjugate() * world_contact_normal.normalized();

    contact_jacobian res;
    // world-space contact points
    auto r_a = a.orientation() * local_contact_a;
    auto r_b = b.orientation() * local_contact_b;
    auto n = b.orientation() * local_normal_b;
    auto penetration = ((b.pos() + r_b - (a.pos() + r_a))).dot(n);
    if (penetration <= 0.0 * si::metre)
        return std::nullopt; // Ensure consistency with collision detection (positive penetration
                             // means penetration).

    // Normal constraint: prevent penetration
    {
        jacobian_row row{
            .J_v = n,
            .J_w_a = r_a.cross(n),
            .J_w_b = -r_b.cross(n),
            .impulse_min = 0.0 * si::newton * si::second, // contacts can only push, not pull
            .a = &a,
            .b = &b,
        };

        auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
        auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

        row.M_eff = 1.0 / (a.inv_mass() + b.inv_mass() + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));

        auto v_ca = a.vel() + a.ang_vel().cross(r_a);
        auto v_cb = b.vel() + b.ang_vel().cross(r_b);

        auto v_rel_n = (v_ca - v_cb).dot(n);
        double restitution_coeff = std::max(a.restitution(), b.restitution());

        // prevent jitter when objects come to rest
        constexpr auto restitution_threshold = 1.0 * si::metre / si::second;
        auto restitution_bias = 0.0 * si::metre / si::second;

        if (v_rel_n < -restitution_threshold) restitution_bias = restitution_coeff * v_rel_n;

        auto baumgarte_bias =
            (-bias_factor / dt) * std::max(0.0 * si::metre, penetration - linear_slop);

        row.bias = std::min(baumgarte_bias, restitution_bias);
        res.normal = row;
    }

    // friction constraints
    {
        auto build_tangent = [&](const vec3<one> &tangent)
        {
            jacobian_row row{
                .J_v = tangent,
                .J_w_a = r_a.cross(tangent),
                .J_w_b = -r_b.cross(tangent),
                .bias = 0 * si::metre / si::second, // no bias for friction
                .a = &a,
                .b = &b,
            };

            auto k_a = a.world_inv_inertia_tensor() * row.J_w_a;
            auto k_b = b.world_inv_inertia_tensor() * row.J_w_b;

            row.M_eff =
                1.0 / (a.inv_mass() + b.inv_mass() + row.J_w_a.dot(k_a) + row.J_w_b.dot(k_b));
            return row;
        };
        auto [t1, t2] = detail::build_orthonormal_basis(n);
        res.tangent1 = build_tangent(t1);
        res.tangent2 = build_tangent(t2);
    }

    return res;
}

template <constraint_type Type>
using constraint_class_type = std::conditional_t<
    Type == constraint_type::distance, distance_constraint,
    std::conditional_t<Type == constraint_type::ball_socket, ball_socket_constraint,
                       std::conditional_t<Type == constraint_type::slider, slider_constraint,
                                          std::conditional_t<Type == constraint_type::weld,
                                                             weld_constraint, hinge_constraint>>>>;

namespace detail
{
template <std::size_t... Is>
inline auto make_tuple_helper(std::size_t initial_capacity, std::index_sequence<Is...> /*unused*/)
{
    return std::tuple{
        physkit::detail::arena<impulse::constraint_class_type<static_cast<constraint_type>(Is)>>(
            initial_capacity)...};
}

constexpr auto constraint_count = static_cast<std::size_t>(constraint_type::DetailCount);
using constraint_arenas =
    decltype(make_tuple_helper({}, std::make_index_sequence<constraint_count>{}));
} // namespace detail

template <std::derived_from<integrator> Integrator> class constraint_solver;

template <> class constraint_solver<semi_implicit_euler>
{
public:
    constraint_solver(std::size_t iterations, std::size_t initial_capacity = 100)
        : M_iterations(iterations),
          M_constraints(detail::make_tuple_helper(
              initial_capacity, std::make_index_sequence<detail::constraint_count>{}))
    {
    }

    template <constraint_type Type>
    auto add_constraint(const impulse::constraint_class_type<Type> &constraint)
    {
        return std::get<physkit::detail::arena<impulse::constraint_class_type<Type>>>(M_constraints)
            .add(std::forward<decltype(constraint)>(constraint));
    }

    template <constraint_type Type>
    auto remove_constraint(
        typename physkit::detail::arena<impulse::constraint_class_type<Type>>::handle h)
    {
        return std::get<physkit::detail::arena<impulse::constraint_class_type<Type>>>(M_constraints)
            .remove(h);
    }

    void setup_contacts(
        quantity<si::second> dt, physkit::detail::narrow_phase &narrow,
        std::regular_invocable<physkit::detail::dynamic_bvh::object_handle> auto &&get_object)
        requires(std::same_as<std::invoke_result_t<decltype(get_object),
                                                   physkit::detail::dynamic_bvh::object_handle>,
                              object &>)
    {
        M_contact_rows.clear();
        for (auto &man_info : narrow.manifolds())
        {
            if (!man_info.man) continue;

            auto &a = get_object(man_info.a);
            auto &b = get_object(man_info.b);

            for (auto &contact : man_info.man.contacts())
            {
                auto jac_opt = build_contact_jacobian(
                    a, b, contact.info.local_a, contact.info.local_b, contact.info.normal, dt);
                if (!jac_opt) continue;

                contact_solver_point p{.jac = *jac_opt,
                                       .friction_coeff = std::sqrt(a.friction() * b.friction()),
                                       .cache = &contact};
                p.jac.normal.accumulated_impulse = contact.normal_impulse;
                vec3 old_friction_impulse = p.jac.old_tangent1 * contact.tangent_impulses[0] +
                                            p.jac.old_tangent2 * contact.tangent_impulses[1];
                p.jac.tangent1.accumulated_impulse = old_friction_impulse.dot(p.jac.tangent1.J_v);
                p.jac.tangent2.accumulated_impulse = old_friction_impulse.dot(p.jac.tangent2.J_v);
                p.jac.old_tangent1 = p.jac.tangent1.J_v;
                p.jac.old_tangent2 = p.jac.tangent2.J_v;
                M_contact_rows.push_back(p);
            }
        }
    }

    // Call after setup_contacts
    void solve_constraints(quantity<si::second> dt)
    {
        M_jacobian_rows.clear();
        std::apply(
            [&](auto &&...vecs)
            {
                (std::ranges::for_each(vecs.slots,
                                       [&](auto &s)
                                       {
                                           if (!s.available)
                                               s.value.build_jacobian(dt, M_jacobian_rows);
                                       }),
                 ...);
            },
            M_constraints);

        // warm start
        for (auto &row : M_jacobian_rows) row.warm_start();
        for (auto &pt : M_contact_rows)
        {
            pt.jac.normal.warm_start();
            pt.jac.tangent1.warm_start();
            pt.jac.tangent2.warm_start();
        }

        for (std::size_t i = 0; i < M_iterations; ++i)
        {
            for (auto &row : M_jacobian_rows) row.apply();
            for (auto &pt : M_contact_rows)
            {
                pt.jac.normal.apply();
                auto max_friction = pt.friction_coeff * pt.jac.normal.accumulated_impulse;

                // TODO: implement friction cone (instead of box) projection for better realism
                pt.jac.tangent1.impulse_min = pt.jac.tangent2.impulse_min = -max_friction;
                pt.jac.tangent1.impulse_max = pt.jac.tangent2.impulse_max = max_friction;
                pt.jac.tangent1.apply();
                pt.jac.tangent2.apply();
            }
        }

        for (const auto &pt : M_contact_rows)
        {
            pt.cache->normal_impulse = pt.jac.normal.accumulated_impulse;
            pt.cache->tangent_impulses = {pt.jac.tangent1.accumulated_impulse,
                                          pt.jac.tangent2.accumulated_impulse};
        }
    }

private:
    struct contact_solver_point
    {
        impulse::contact_jacobian jac;
        double friction_coeff{};

        physkit::detail::manifold::contact_info *cache{};
    };

    detail::constraint_arenas M_constraints;
    std::vector<contact_solver_point> M_contact_rows;

    std::vector<impulse::jacobian_row> M_jacobian_rows;
    std::size_t M_iterations;
};
} // namespace impulse
} // namespace physkit
