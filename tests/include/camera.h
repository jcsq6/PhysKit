#pragma once
#include <physkit/physkit.h>

#include <Magnum/Animation/Track.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Math/DualQuaternion.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/AbstractTranslationRotation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

#include <ranges>
#include <span>

#include "Corrade/Containers/GrowableArray.h"
#include "convert.h"

namespace graphics
{
using namespace Magnum;

/// @brief Keyframe for camera animation tracks.
///
/// A keyframe represents a single point in a camera animation, specifying position and/or
/// orientation. Keyframes can be chained together using the builder pattern to create smooth
/// camera movements.
///
/// @par Example: Creating a simple camera path
/// @code{.cpp}
/// cam.set_move_track(camera_track({
///     kf::make_pos({0.0f * m, 0.0f * m, 0.0f * m})
///         .look_at({10.0f * m, 0.0f * m, 0.0f * m})
///         .transition(2.0f * s),
///     kf::make_pos({5.0f * m, 2.0f * m, 0.0f * m})
///         .transition(1.5f * s)
/// }));
/// @endcode
struct kf
{
    /// @brief Look-at orientation: camera points toward a target position.
    struct look_at_t
    {
        physkit::vec3<physkit::si::metre, float> target;
    };
    /// @brief Explicit quaternion orientation.
    struct orient_t
    {
        Magnum::Quaternion rot;
    };
    /// @brief Direction orientation: camera faces a normalized direction vector.
    struct facing_t
    {
        physkit::vec3<physkit::one, float> dir;
    };

    /// @brief Create a keyframe with a facing direction.
    /// @param d The direction the camera should face (will be normalized).
    /// @return A keyframe configured to face the specified direction.
    static kf make_dir(const physkit::vec3<physkit::one, float> &d) { return kf{facing_t{d}}; }

    /// @brief Create a keyframe that looks at a specific target position.
    /// @param target The world-space position to look at.
    /// @return A keyframe configured to look at the target.
    static kf make_look_at(const physkit::vec3<physkit::si::metre, float> &target)
    {
        return kf{look_at_t{target}};
    }

    /// @brief Create a keyframe with an explicit quaternion orientation.
    /// @param rot The quaternion representing the camera's orientation.
    /// @return A keyframe with the specified orientation.
    static kf make_orient(const Magnum::Quaternion &rot) { return kf{orient_t{rot}}; }

    /// @brief Create a keyframe with a position.
    /// @param p The world-space position for the camera.
    /// @return A keyframe at the specified position.
    static kf make_pos(const physkit::vec3<physkit::si::metre, float> &p) { return kf{p}; }

    /// @brief Set the camera direction for this keyframe (builder pattern).
    /// @param d The direction vector (will be normalized).
    /// @return Reference to this keyframe for method chaining.
    auto &&dir(this auto &&self, const physkit::vec3<physkit::one, float> &d)
    {
        self.M_orient = facing_t{d};
        return std::forward<decltype(self)>(self);
    }

    /// @brief Set the camera position for this keyframe (builder pattern).
    /// @param p The world-space position.
    /// @return Reference to this keyframe for method chaining.
    auto &&pos(this auto &&self, const physkit::vec3<physkit::si::metre, float> &p)
    {
        self.M_point = p;
        return std::forward<decltype(self)>(self);
    }

    /// @brief Set an explicit quaternion orientation for this keyframe (builder pattern).
    /// @param rot The quaternion orientation.
    /// @return Reference to this keyframe for method chaining.
    auto &&orient(this auto &&self, const Magnum::Quaternion &rot)
    {
        self.M_orient = orient_t{rot};
        return std::forward<decltype(self)>(self);
    }

    /// @brief Set the transition duration from this keyframe to the next (builder pattern).
    /// @param duration The time it takes to interpolate to the next keyframe.
    /// @return Reference to this keyframe for method chaining.
    /// @note The transition time is added to the absolute time of this keyframe to determine
    ///       when the next keyframe is reached.
    auto &&transition(this auto &&self, physkit::quantity<physkit::si::second, float> duration)
    {
        self.M_dt = duration;
        return std::forward<decltype(self)>(self);
    }

    /// @brief Set the camera to look at a target position (builder pattern).
    /// @param target The world-space position to look at.
    /// @return Reference to this keyframe for method chaining.
    auto &&look_at(this auto &&self, const physkit::vec3<physkit::si::metre, float> &target)
    {
        self.M_orient = look_at_t{target};
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] auto &orient() const { return M_orient; }

    [[nodiscard]] auto &point() const { return M_point; }

    [[nodiscard]] auto &transition() const { return M_dt; }

private:
    std::optional<physkit::vec3<physkit::si::metre, float>> M_point;
    std::optional<std::variant<facing_t, orient_t, look_at_t>> M_orient;
    physkit::quantity<physkit::si::second, float> M_dt{0.0f * physkit::si::second};

    kf(physkit::vec3<physkit::si::metre, float> point) : M_point{point} {}
    kf(look_at_t dir) : M_orient{dir} {}
    kf(orient_t orient) : M_orient{orient} {}
    kf(facing_t facing) : M_orient{facing} {}
};

/// @brief Interpolated camera animation track.
///
/// A camera_track manages smooth camera movement through a series of keyframes. It interpolates
/// both position and orientation over time using cubic Hermite splines (or simpler interpolation).
/// If no orientation is given, the user will be free to look around normally.
///
/// @par Interpolation Modes
/// - `constant`: Step function (no interpolation)
/// - `linear`: Linear interpolation between keyframes
/// - `spline`: Smooth cubic Hermite spline interpolation (default, recommended)
///
/// @par Usage Example
/// @code{.cpp}
/// using namespace mp_units::si::unit_symbols;
///
/// // Pass keyframes directly via initializer list
/// cam.set_move_track(camera_track({
///     kf::make_pos({0.0f * m, 1.0f * m, -5.0f * m})
///         .look_at({0.0f * m, 0.0f * m, 0.0f * m})
///         .transition(2.0f * s),
///     kf::make_pos({5.0f * m, 2.0f * m, -3.0f * m})
///         .transition(1.5f * s),
///     kf::make_pos({0.0f * m, 1.0f * m, 5.0f * m})
///         .dir({0.0f, 0.0f, 1.0f})
///         .transition(2.0f * s)
///         })
///     .with_interp(camera_track::spline)
///     .with_extrap(camera_track::release));
///
/// // Alternatively, for manual sampling:
/// camera_track track({...keyframes...});
/// auto [pos, rot] = track.at(current_time);
/// @endcode
///
/// @note At least two keyframes with position data are required.
/// @note Keyframes without explicit orientation will derive orientation from movement direction.
class camera_track
{
public:
    /// @brief Interpolation method for the camera track.
    enum interpolation_t : std::uint8_t
    {
        constant, ///< No interpolation (step function)
        linear,   ///< Linear interpolation
        spline    ///< Cubic Hermite spline interpolation (smooth)
    };

    /// @brief Extrapolation behavior for the camera track.
    enum extrapolation_t : std::uint8_t
    {
        release, ///< Camera is released from camera track
        loop,    ///< Camera returns to start and loops through track
        reverse  ///< Camera moves back and forth through the track
    };

    /// @brief Default constructor creates an empty track.
    camera_track() = default;

    /// @brief Construct a camera track from a span of keyframes.
    /// @param pts Span of keyframes defining the camera path.
    /// @param interp Interpolation method to use (default: spline).
    /// @throws std::runtime_error if fewer than two keyframes with position data are provided.
    explicit camera_track(const std::span<const kf> pts) { deduce(pts); }

    auto &&with_interp(this auto &&self, interpolation_t interp)
    {
        self.M_interpolation = interp;
        self.mark_dirty();
        return std::forward<decltype(self)>(self);
    }

    auto &&with_extrap(this auto &&self, extrapolation_t extrap)
    {
        self.M_extrapolation = extrap;
        self.mark_dirty();
        return std::forward<decltype(self)>(self);
    }

    camera_track(camera_track &&) = default;
    camera_track &operator=(camera_track &&) = default;
    camera_track(const camera_track &) = delete;
    camera_track &operator=(const camera_track &) = delete;
    ~camera_track() = default;

    /// @brief Camera pose (position and orientation).
    struct pose
    {
        Vector3 pos;            ///< Camera position in world space
        Magnum::Quaternion rot; ///< Camera orientation as a quaternion
    };

    /// @brief Sample the camera track at a specific time.
    /// @param time The time at which to sample the track (clamped to [0, duration()]).
    /// @return The interpolated camera pose at the specified time.
    [[nodiscard]] pose at(physkit::quantity<physkit::si::second, float> time)
    {
        clean();

        return {.pos = M_pos_track.at(time.numerical_value_in(physkit::si::second)),
                .rot = M_orient_track.at(time.numerical_value_in(physkit::si::second))};
    }

    /// @brief Check if the camera should be released at a certain time.
    /// @param time The time at which to check.
    /// @return Boolean representing if the camera is released.
    [[nodiscard]] bool released(const physkit::quantity<physkit::si::second, float> time)
    {
        return (time > M_duration && M_extrapolation == release);
    }

    /// @brief Get the total duration of the camera track.
    /// @return The sum of all transition durations in the track.
    [[nodiscard]] auto duration() const { return M_duration; }

    /// @brief Get the freelook flag of the camera track.
    /// @return The freelook flag of the camera track.
    [[nodiscard]] bool freelook() const { return M_freelook; }

private:
    using second_t = physkit::quantity<physkit::si::second, float>;
    std::vector<std::pair<second_t, kf>> M_kfs;
    second_t M_duration = -1.f * physkit::si::second;
    interpolation_t M_interpolation{interpolation_t::spline};
    extrapolation_t M_extrapolation{extrapolation_t::release};
    Animation::Track<float, Math::CubicHermite3D<float>, Math::Vector3<float>> M_pos_track;
    Animation::Track<float, Math::CubicHermiteQuaternion<float>, Quaternion> M_orient_track;

    bool M_dirty{true};
    bool M_freelook{false};

    void deduce(const std::span<const kf> pts)
    {
        M_kfs.clear();
        M_kfs.reserve(pts.size());
        M_duration = std::ranges::fold_left(pts, 0.0f * physkit::si::second,
                                            [this](second_t acc, const kf &p)
                                            {
                                                M_kfs.emplace_back(acc, p);
                                                return acc + p.transition();
                                            });
        if (std::ranges::count_if(pts, [](const kf &p) { return p.point().has_value(); }) < 2)
            throw std::runtime_error(
                "camera_track requires at least two keyframes with position data");

        mark_dirty();
    }

    void compile_pos()
    {
        if (M_kfs.size() < 2) return;

        std::size_t n = M_kfs.size();

        Corrade::Containers::Array<float> t;
        Corrade::Containers::Array<Vector3> p;
        arrayReserve(t, n);
        arrayReserve(p, n);

        for (auto &[time, keyframe] :
             M_kfs | std::views::filter([](auto &kv) { return kv.second.point().has_value(); }))
        {
            arrayAppend(t, time.numerical_value_in(physkit::si::second));
            arrayAppend(p, to_magnum_vector<float>(keyframe.point().value()));
        }

        n = p.size();

        auto safe_div = [](auto &&a, float b) { return b == 0.0f ? decltype(a){} : a / b; };

        Corrade::Containers::Array<Vector3> v;
        arrayReserve(v, n);
        arrayAppend(v, safe_div(p[1] - p[0], t[1] - t[0]));
        for (std::size_t i = 1; i + 1 < n; ++i)
            arrayAppend(v, safe_div(p[i + 1] - p[i - 1], t[i + 1] - t[i - 1]));
        arrayAppend(v, safe_div(p[n - 1] - p[n - 2], t[n - 1] - t[n - 2]));

        Corrade::Containers::Array<std::pair<float, Math::CubicHermite3D<float>>> pos_format;
        Containers::arrayReserve(pos_format, n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const float dt_in = (i > 0) ? (t[i] - t[i - 1]) : (t[1] - t[0]);
            const float dt_out = (i + 1 < n) ? (t[i + 1] - t[i]) : (t[n - 1] - t[n - 2]);

            const Vector3 in_tangent = v[i] * dt_in;
            const Vector3 out_tangent = v[i] * dt_out;

            arrayAppend(pos_format,
                        {t[i], Math::CubicHermite3D<float>{in_tangent, p[i], out_tangent}});
        }

        switch (M_interpolation)
        {
        case interpolation_t::constant:
            M_pos_track = Animation::Track{std::move(pos_format), Math::select,
                                           Animation::Extrapolation::Constant};
            break;
        case interpolation_t::linear:
            M_pos_track = Animation::Track{std::move(pos_format), Math::lerp,
                                           Animation::Extrapolation::Constant};
            break;
        case interpolation_t::spline:
            M_pos_track = Animation::Track{std::move(pos_format), Math::splerp,
                                           Animation::Extrapolation::Constant};
            break;
        }
    }

    void compile_orient()
    {
        if (M_kfs.size() < 2) return;

        const float t_end = M_duration.numerical_value_in(physkit::si::second);

        Containers::Array<float> t;
        Containers::Array<Quaternion> q;
        arrayReserve(t, M_kfs.size());
        arrayReserve(q, M_kfs.size());
        for (auto &[time, keyframe] :
             M_kfs | std::views::filter([](auto &kv) { return kv.second.orient().has_value(); }))
        {
            const float t_sec = time.numerical_value_in(physkit::si::second);
            arrayAppend(t, t_sec);
            arrayAppend(q, resolve_orient_at_time(keyframe, t_sec, M_pos_track));
        }

        if (q.isEmpty())
        {
            M_freelook = true;
            return;
        }

        auto prepend = [&](float time, const Quaternion &orient)
        {
            arrayInsert(t, 0, auto(time));
            arrayInsert(q, 0, auto(orient));
        };

        auto append = [&](float time, const Quaternion &orient)
        {
            arrayAppend(t, auto(time));
            arrayAppend(q, auto(orient));
        };

        if (t.front() > 0.0f) prepend(0.0f, q.front());
        if (t.back() < t_end) append(t_end, q.back());
        if (q.size() == 1) append(t.front() == t_end ? 0.0f : t_end, q.front());

        const auto n = q.size();
        // enforce sign continuity
        for (std::size_t i = 1; i < n; ++i)
            if (dot(q[i - 1], q[i]) < 0.0f) q[i] = -q[i];

        auto safe_div = [](auto &&a, float b) { return b == 0.0f ? decltype(a){} : a / b; };

        Containers::Array<Quaternion> v;
        arrayReserve(v, n);

        arrayAppend(v, safe_div(q[1] - q[0], t[1] - t[0]));
        for (std::size_t i = 1; i + 1 < n; ++i)
            arrayAppend(v, safe_div(q[i + 1] - q[i - 1], t[i + 1] - t[i - 1]));
        arrayAppend(v, safe_div(q[n - 1] - q[n - 2], t[n - 1] - t[n - 2]));

        Containers::Array<std::pair<float, Math::CubicHermiteQuaternion<float>>> orient_format;
        Containers::arrayReserve(orient_format, n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const float dt_in = (i > 0) ? (t[i] - t[i - 1]) : (t[1] - t[0]);
            const float dt_out = (i + 1 < n) ? (t[i + 1] - t[i]) : (t[n - 1] - t[n - 2]);

            const Quaternion in_tangent = v[i] * dt_in;
            const Quaternion out_tangent = v[i] * dt_out;

            arrayAppend(orient_format,
                        {t[i], Math::CubicHermiteQuaternion<float>{in_tangent, q[i], out_tangent}});
        }

        switch (M_interpolation)
        {
        case interpolation_t::constant:
            M_orient_track = Animation::Track{std::move(orient_format), Math::select,
                                              Animation::Extrapolation::Constant};
            break;
        case interpolation_t::linear:
            M_orient_track = Animation::Track{std::move(orient_format), Math::lerp,
                                              Animation::Extrapolation::Constant};
            break;
        case interpolation_t::spline:
            M_orient_track = Animation::Track{std::move(orient_format), Math::splerp,
                                              Animation::Extrapolation::Constant};
            break;
        }
    }

    void mark_dirty() { M_dirty = true; }

    void clean()
    {
        if (M_dirty)
        {
            compile_pos();
            compile_orient();
            M_dirty = false;
        }
    }

    static Quaternion look_rotation(Vector3 forward, Vector3 up = Vector3::yAxis())
    {
        forward = forward.normalized();
        up = up.normalized();

        auto right = cross(forward, up);
        const float right_len2 = dot(right, right);
        if (right_len2 < 1e-6f)
        {
            up = (abs(forward.y()) < .99f) ? Vector3::yAxis() : Vector3::xAxis();
            right = cross(forward, up);
        }

        right = right.normalized();
        auto corrected_up = cross(right, forward);

        const Matrix3 rot{right, corrected_up, -forward};
        return Quaternion::fromMatrix(rot).normalized();
    }

    static Quaternion resolve_orient_at_time(
        const kf &keyframe, float t_seconds,
        const Animation::Track<float, Math::CubicHermite3D<float>, Vector3> &pos_track)
    {
        if (!keyframe.orient().has_value()) return {};

        const auto &ov = *keyframe.orient();
        return std::visit(
            [&](auto &&o) -> Quaternion
            {
                using T = std::decay_t<decltype(o)>;
                if constexpr (std::same_as<T, kf::orient_t>)
                    return o.rot.normalized();
                else if constexpr (std::same_as<T, kf::facing_t>)
                    return look_rotation(to_magnum_vector<float>(o.dir));
                else if constexpr (std::same_as<T, kf::look_at_t>)
                    return look_rotation(to_magnum_vector<float>(o.target) -
                                         pos_track.at(t_seconds));
                else
                    static_assert(sizeof(T) == 0, "non-exhaustive visitor!");
            },
            ov);
    }
};

class camera
{
public:
    constexpr static float near = 0.01f;
    constexpr static float far = 1000.0f;

    constexpr static auto up = Vector3::yAxis();

    template <class Transform>
    explicit camera(SceneGraph::Scene<Transform> &scene,
                    physkit::quantity<physkit::si::degree, float> fov, const Vector3 &position,
                    const Vector3 &dir, const Vector2i &window_size, const Vector2i &viewport_size,
                    physkit::quantity<physkit::si::metre / physkit::si::second, float> speed =
                        4.0f * physkit::si::metre / physkit::si::second)
        : M_fov{fov}, M_window_size{window_size}, M_pos{position}, M_speed{speed}
    {
        auto *obj = new SceneGraph::Object<Transform>{&scene};
        M_obj = obj;
        M_cam = new SceneGraph::Camera3D{*obj};
        M_cam->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(
                Matrix4::perspectiveProjection(Rad(M_fov.numerical_value_in(physkit::si::radian)),
                                               Vector2(viewport_size).aspectRatio(), near, far))
            .setViewport(viewport_size);

        set_view(dir);
    }

    void set_view(const Vector3 &dir)
    {
        if (Math::isInf(dir).any() || dir.isZero()) return;

        Vector3 f = dir.normalized();
        Vector3 r = Math::cross(f, up);
        if (r.isZero())
        {
            const Vector3 alt_up = Math::abs(Math::dot(f, Vector3::zAxis())) > 0.99f
                                       ? Vector3::xAxis()
                                       : Vector3::zAxis();
            r = Math::cross(f, alt_up);
            if (r.isZero()) return;
        }

        r = r.normalized();
        const Vector3 u = Math::cross(r, f);
        M_rot = Quaternion::fromMatrix({Matrix3{r, u, -f}}).normalized();
        mark_dirty();
    }

    [[nodiscard]] DualQuaternion transformation() const noexcept
    {
        return DualQuaternion::from(M_rot, M_pos);
    }

    void resize(const Vector2i &window_size, const Vector2i &viewport_size)
    {
        M_window_size = window_size;
        M_cam
            ->setProjectionMatrix(
                Matrix4::perspectiveProjection(Rad(M_fov.numerical_value_in(physkit::si::radian)),
                                               Vector2(viewport_size).aspectRatio(), near, far))
            .setViewport(viewport_size);
    }

    void look_at(const physkit::vec3<physkit::si::metre, float> &pos)
    {
        const Vector3 target = to_magnum_vector<float>(pos);
        const Vector3 dir = target - M_pos;
        set_view(dir);
    }

    void move(const physkit::vec3<physkit::si::metre, float> &delta)
    {
        M_pos += to_magnum_vector<float>(delta);
        mark_dirty();
    }

    void move(float forward, float right, float up,
              physkit::quantity<physkit::si::second, float> dt)
    {
        Vector3 delta{0.0f};
        if (forward != 0) delta += forward_axis() * forward;
        if (right != 0) delta += right_axis() * right;
        if (up != 0) delta += camera::up * up;
        if (delta == Vector3{0.0f}) return;
        move((to_physkit_vector<physkit::one, float>(delta.normalized())) * (M_speed * dt));
    }

    void rotate(physkit::quantity<physkit::si::radian, float> d_yaw,
                physkit::quantity<physkit::si::radian, float> d_pitch)
    {
        float yaw = d_yaw.numerical_value_in(physkit::si::radian);
        float pitch = d_pitch.numerical_value_in(physkit::si::radian);
        if (yaw == 0.0f && pitch == 0.0f) return;

        M_rot = Quaternion::rotation(Rad(yaw), up) * M_rot;

        constexpr auto max_pitch = std::numbers::pi_v<float> / 2 * .98f;
        const Vector3 fwd = forward_axis();
        const auto cur = (float) Math::asin(Math::clamp(fwd.y(), -1.0f, 1.0f));
        const auto tgt = Math::clamp(cur + pitch, -max_pitch, max_pitch);

        const Vector3 right = M_rot.transformVector(Vector3::xAxis()).normalized();
        M_rot = (Quaternion::rotation(Rad(tgt - cur), right) * M_rot).normalized();

        mark_dirty();
    }

    [[nodiscard]] physkit::quantity<physkit::si::metre / physkit::si::second, float> speed() const
    {
        return M_speed;
    }
    void speed(physkit::quantity<physkit::si::metre / physkit::si::second, float> s)
    {
        M_speed = s;
    }

    void pointer_move(Platform::Application::PointerMoveEvent &event, bool drag)
    {
        constexpr float sx = 0.002f; // yaw per pixel
        constexpr float sy = 0.002f; // pitch per pixel

        const Vector2 d = event.relativePosition();
        float dx = d.x();
        float dy = d.y();

        if (!std::isfinite(dx) || !std::isfinite(dy)) return;
        constexpr float max_delta = 200.0f;
        dx = Math::clamp(dx, -max_delta, max_delta);
        dy = Math::clamp(dy, -max_delta, max_delta);
        if (drag)
            rotate((dx * sx) * physkit::si::radian, (dy * sy) * physkit::si::radian);
        else
            rotate((-dx * sx) * physkit::si::radian, (-dy * sy) * physkit::si::radian);
    }

    /// @brief Set an animated movement track for the camera.
    /// @param track The camera_track to follow. Once set, the camera will automatically
    ///              interpolate through the track's keyframes during draw() calls.
    /// @note This overrides manual camera controls. To restore manual control, set an empty track.
    /// @note The track takes ownership via move semantics.
    /// @see camera_track for details on creating animation tracks.
    void set_move_track(camera_track track)
    {
        M_track = std::move(track);
        mark_dirty();
    }

    SceneGraph::Camera3D &cam() { return *M_cam; }

    [[nodiscard]] Matrix4 view_matrix() const { return transformation().toMatrix(); }
    [[nodiscard]] Matrix4 inverse_view_matrix() const
    {
        if (M_dirty)
        {
            M_inverse_view = transformation().inverted();
            M_dirty = false;
        }
        return M_inverse_view.toMatrix();
    }

    bool update(physkit::quantity<physkit::si::second> t)
    {
        auto update = [&]
        {
            M_obj->resetTransformation().rotate(M_rot).translate(M_pos);
            M_inverse_view = transformation().inverted();
            M_dirty = false;
        };

        if ((M_track.duration() > 0.0f * physkit::si::second) &&
             !M_track.released(t))
        {
            auto [pos, rot] = M_track.at(t);
            M_pos = pos;
            if (!M_track.freelook()) M_rot = rot;
            return update(), true;
        }

        if (M_dirty) return update(), true;
        return false;
    }

    void draw(SceneGraph::DrawableGroup3D &drawables, physkit::quantity<physkit::si::second> t)
    {
        update(t);
        M_cam->draw(drawables);
    }

    auto projection_matrix() const { return M_cam->projectionMatrix(); }

private:
    SceneGraph::Camera3D *M_cam{};
    SceneGraph::AbstractTranslationRotation3D *M_obj{};

    physkit::quantity<physkit::si::degree, float> M_fov{};
    physkit::quantity<physkit::si::metre / physkit::si::second, float> M_speed{};

    camera_track M_track;

    Quaternion M_rot{Magnum::Math::IdentityInit};
    Vector3 M_pos{0.0f};

    mutable DualQuaternion M_inverse_view;
    mutable bool M_dirty{true};

    Vector2i M_window_size{1, 1};

    [[nodiscard]] Vector3 right_axis() const noexcept
    {
        return M_rot.transformVector(Vector3::xAxis());
    }
    [[nodiscard]] Vector3 up_axis() const noexcept
    {
        return M_rot.transformVector(Vector3::yAxis());
    }
    [[nodiscard]] Vector3 forward_axis() const noexcept
    {
        return M_rot.transformVector(-Vector3::zAxis());
    }

    void mark_dirty()
    {
        M_dirty = true;
        M_inverse_view = transformation().inverted();
    }
};

} // namespace graphics
