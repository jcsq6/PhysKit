#pragma once
#include <physkit/physkit.h>

#include <Magnum/Magnum.h>
#include <Magnum/Math/DualQuaternion.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/AbstractTranslationRotation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

#include "Magnum/Math/Functions.h"
#include "convert.h"
#include "physkit/detail/types.h"

namespace graphics
{
using namespace Magnum;

class camera
{
public:
    constexpr static float near = 0.01f;
    constexpr static float far = 1000.0f;

    template <class Transform>
    explicit camera(SceneGraph::Scene<Transform> &scene,
                    physkit::quantity<physkit::si::degree, float> fov, const Vector3 &position,
                    const Vector3 &dir, const Vector3 &up, const Vector2i &window_size,
                    const Vector2i &viewport_size)
        : M_fov{fov}, M_window_size{window_size}, M_pos{position}
    {
        auto *obj = new SceneGraph::Object<Transform>{&scene};
        M_obj = obj;
        M_cam = new SceneGraph::Camera3D{*obj};
        M_cam->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(
                Matrix4::perspectiveProjection(Rad(M_fov.numerical_value_in(physkit::si::radian)),
                                               Vector2(viewport_size).aspectRatio(), near, far))
            .setViewport(viewport_size);

        set_view(dir, up);
    }

    void set_view(const Vector3 &dir, const Vector3 &up)
    {
        Vector3 f = dir.normalized();
        Vector3 wu = up.normalized();
        if (Math::abs(Math::dot(f, wu)) > 0.999f) wu = {0.0f, 1.0f, 0.0f}; // NOLINT
        const Vector3 r = Math::cross(f, wu).normalized();
        const Vector3 u = Math::cross(r, f);
        M_rot = Quaternion::fromMatrix({Matrix3{r, u, -f}});
        M_rot = M_rot.normalized();
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
        set_view(f, up_axis());
    }

    void move(const physkit::vec3<physkit::si::metre, float> &delta)
    {
        M_pos += to_magnum_vector<float>(delta);
        mark_dirty();
    }

    void rotate(physkit::quantity<physkit::si::radian, float> d_yaw,
                physkit::quantity<physkit::si::radian, float> d_pitch)
    {
        const float yaw = d_yaw.numerical_value_in(physkit::si::radian);
        const float pitch = d_pitch.numerical_value_in(physkit::si::radian);
        if (yaw == 0.0f && pitch == 0.0f) return;

        const Quaternion q_yaw = Quaternion::rotation(Rad(yaw), up_axis());
        const Quaternion q_pitch = Quaternion::rotation(Rad(pitch), right_axis());
        M_rot = (q_pitch * q_yaw) * M_rot;
        M_rot = M_rot.normalized();
        mark_dirty();
    }

    void move_forward(physkit::quantity<physkit::si::second, float> dt)
    {
        move(to_physkit_vector<physkit::si::metre, float>(forward_axis()) * M_speed * dt);
    }

    void move_right(physkit::quantity<physkit::si::second, float> dt)
    {
        move(to_physkit_vector<physkit::si::metre, float>(right_axis()) * M_speed * dt);
    }

    void move_up(physkit::quantity<physkit::si::second, float> dt)
    {
        move(to_physkit_vector<physkit::si::metre, float>(up_axis()) * M_speed * dt);
    }

    [[nodiscard]] physkit::quantity<physkit::si::metre / physkit::si::second, float> speed() const
    {
        return M_speed;
    }
    void speed(physkit::quantity<physkit::si::metre / physkit::si::second, float> s)
    {
        M_speed = s;
    }

    void key_press(Platform::Application::KeyEvent &event,
                   physkit::quantity<physkit::si::second, float> dt)
    {
        using Key = Platform::Application::Key;
        if (event.key() == Key::Down) move_forward(-dt);
        if (event.key() == Key::Up) move_forward(dt);
        if (event.key() == Key::Left) move_right(-dt);
        if (event.key() == Key::Right) move_right(dt);
        if (event.key() == Key::Space) move_up(dt);
        if (event.key() == Key::LeftShift) move_up(-dt);
    }

    void pointer_move(Platform::Application::PointerMoveEvent &event)
    {
        constexpr float sx = 0.002f; // yaw per pixel
        constexpr float sy = 0.002f; // pitch per pixel
        const Vector2 dp = event.relativePosition();
        if (dp == Vector2{}) return;
        rotate((dp.x() * sx) * physkit::si::radian, (-dp.y() * sy) * physkit::si::radian);
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

    bool update()
    {
        if (M_dirty)
        {
            M_obj->resetTransformation().rotate(M_rot).translate(M_pos);
            M_dirty = false;
            return true;
        }
        return false;
    }

    void draw(SceneGraph::DrawableGroup3D &drawables)
    {
        update();
        M_cam->draw(drawables);
    }

private:
    SceneGraph::Camera3D *M_cam{};
    SceneGraph::AbstractTranslationRotation3D *M_obj{};

    physkit::quantity<physkit::si::degree, float> M_fov{};
    physkit::quantity<physkit::si::metre / physkit::si::second, float> M_speed{
        1.0f * physkit::si::metre / physkit::si::second};

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
