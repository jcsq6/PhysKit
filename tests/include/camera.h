#pragma once
#include <physkit/physkit.h>

#include <Magnum/Magnum.h>
#include <Magnum/Math/DualQuaternion.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/AbstractTranslationRotation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/Animation/Track.h>

#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

#include "convert.h"

namespace graphics
{
using namespace Magnum;

class track
{
public:
    std::initializer_list<physkit::vec3<physkit::si::metre, float>> points;
    std::initializer_list<physkit::quantity<physkit::si::second, float>> times;
    std::initializer_list<physkit::unit_mat<physkit::quantity<physkit::si::degree, float>, 2, 1>> angles;
    enum interpolation{constant, linear, spline} interpolation;

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
        Vector3 f = dir.normalized();
        const Vector3 r = Math::cross(f, up).normalized();
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
        const Vector3 f = (target - M_pos).normalized();
        set_view(f);
    }

    void move(const physkit::vec3<physkit::si::metre, float> &delta)
    {
        M_pos += to_magnum_vector<float>(delta);
        mark_dirty();
    }

    void move(float forward, float right, float up, physkit::quantity<physkit::si::second, float> dt)
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

    void set_move_track(track *t)
    {
        //put the set of points and times into a format usable by magnum
        //Corrade::Containers::Array<std::pair<float,Math::Vector3<float>>> points{t->points.size()};
        Containers::Array<float> times{t->points.size()};
        Containers::Array<Math::Vector3<float>> points{t->points.size()};
        Containers::Array<Math::Vector3<float>> tangents{t->points.size()};

        Corrade::Containers::Array<std::pair<float, Math::CubicHermite3D<float>>> format{t->points.size()};

        //extract data from physkit types
        auto p_it = points.begin();
        auto t_it = times.begin();
        auto tp_it = t->points.begin();
        auto tt_it = t->times.begin();
        while (tp_it != t->points.end() && tt_it != t->times.end())
        {
            *t_it = tt_it->numerical_value_in(physkit::si::second);
            *p_it = detail::expand<Magnum::Math::Vector3<float>>([&](auto i)
                                  { return static_cast<float>((*tp_it)[i].numerical_value_in(physkit::si::metre)); },
                                  std::make_index_sequence<3>{});


            ++t_it;
            ++p_it;
            ++tp_it;
            ++tt_it;
        }

        //calculate cubic hermite spline tangents
        for (int i = 0; i < points.size(); i++)
        {
            Math::Vector3<float> a{0,0,0};
            Math::Vector3<float> b{0,0,0};

            //get tangent between this point and neighboring points for easy spline
            if (i != points.size() - 1)
                a += (points.data()[i+1] - points.data()[i]) / (times.data()[i+1] - times.data()[i]);
            else if (i != 0)
                b += (points.data()[i] - points.data()[i-1]) / (times.data()[i] - times.data()[i-1]);

            //average tangents
            if (i == points.size() - 1 || i == 0)
                tangents.data()[i] = a + b;
            else
                tangents.data()[i] = (a + b) / 2;
        }

        //construct cubic hermites
        for (int i = 0; i < format.size(); i++)
        {
            printf("%f,%f\n", tangents.data()[i].x(), points.data()[i].x());
            format.data()[i].second.inTangent() = tangents.data()[i];
            format.data()[i].second.point() = points.data()[i];
            format.data()[i].second.outTangent() = tangents.data()[i];
            format.data()[i].first = times.data()[i];
        }

        //choose interpolation function
        //Math::constant constant
        //Math::lerp linear
        //Math::splerp spline
        //Math::slerp spherical

        //create animation track
        switch (t->interpolation)
        {
        case track::interpolation::constant:
            M_move_track = Animation::Track((std::move(format)), Math::select,
                Animation::Extrapolation::Constant);
            break;
        case track::interpolation::spline:
            M_move_track = Animation::Track((std::move(format)), Math::splerp,
                Animation::Extrapolation::Constant);
            break;
        default:
            M_move_track = Animation::Track((std::move(format)), Math::lerp,
                Animation::Extrapolation::Constant);
            break;
        }
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
        if (M_move_track.size() > 0)
        {
            M_pos = M_move_track.at((float) t.numerical_value_in(physkit::si::second));
            //printf("%f\n", M_pos.x()); //nan...
            M_dirty = true;
        }

        if (M_dirty)
        {
            M_obj->resetTransformation().rotate(M_rot).translate(M_pos);
            M_dirty = false;
            return true;
        }
        return false;
    }

    void draw(SceneGraph::DrawableGroup3D &drawables, physkit::quantity<physkit::si::second> t)
    {
        //printf("%2.2f\n", t.numerical_value_in(physkit::si::second));
        update(t);
        M_cam->draw(drawables);
    }

    auto projection_matrix() const { return M_cam->projectionMatrix(); }

private:
    SceneGraph::Camera3D *M_cam{};
    SceneGraph::AbstractTranslationRotation3D *M_obj{};

    physkit::quantity<physkit::si::degree, float> M_fov{};
    physkit::quantity<physkit::si::metre / physkit::si::second, float> M_speed{};

    Animation::Track <float, Math::CubicHermite<Math::Vector3<float>>, Math::Vector3<float>> M_move_track;
    Animation::Track <float, Math::CubicHermite<Math::Vector2<float>>> M_rotate_track;

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
