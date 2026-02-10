#pragma once
#include "camera.h"
#include "convert.h"

#include <memory>
#include <thread>
#include <unordered_map>
#include <utility>

#include <physkit/physkit.h>

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/StringView.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/Primitives/Cone.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <Magnum/Timeline.h>
#include <Magnum/Trade/MeshData.h>

namespace graphics
{

using namespace Magnum;
using gfx_obj_base = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;

class gfx_obj : public gfx_obj_base
{
public:
    using SceneGraph::Object<SceneGraph::MatrixTransformation3D>::Object;

    void rotate(physkit::quantity<physkit::si::radian, float> angle,
                const physkit::vec3<physkit::one> &axis)
    {
        Quaternion rot = Quaternion::rotation(Rad(angle.numerical_value_in(physkit::si::radian)),
                                              to_magnum_vector<float>(axis.normalized()));
        Object::rotate(rot);
    }
};

// class caching_object : public gfx_obj, SceneGraph::AbstractFeature3D
// {
// public:
//     explicit caching_object(gfx_obj* parent): gfx_obj{parent},
//     SceneGraph::AbstractFeature3D{*this}
//     {
//         setCachedTransformations(SceneGraph::CachedTransformation::Absolute);
//     }

// protected:
//     void clean(const Matrix4& transform) override
//     {

//     }

// private:
//     Vector3 M_absolute_position;
// };

class physics_obj : public gfx_obj
{
public:
    explicit physics_obj(std::derived_from<gfx_obj_base> auto &parent, physkit::world &world,
                         physkit::world::handle handle)
        : gfx_obj(&parent), M_world{&world}, M_handle{handle}
    {
    }

    [[nodiscard]] auto handle() const { return M_handle; }

    physkit::object &obj() { return **M_world->get_rigid(M_handle); }
    [[nodiscard]] const physkit::object &obj() const
    {
        return **std::as_const(*M_world).get_rigid(M_handle);
    }

    void sync()
    {
        if (auto res = M_world->get_rigid(M_handle))
            resetTransformation()
                .rotate(to_magnum_quaternion<float>(res.value()->orientation()))
                .translate(to_magnum_vector<float>(res.value()->particle().pos()));
    }

private:
    physkit::world *M_world;
    physkit::world::handle M_handle;
};

class instanced_drawable;

class instanced_drawables : public SceneGraph::Drawable3D
{
public:
    explicit instanced_drawables(std::derived_from<gfx_obj_base> auto &parent,
                                 SceneGraph::DrawableGroup3D *group, std::shared_ptr<GL::Mesh> mesh,
                                 Shaders::PhongGL &shader)
        : SceneGraph::Drawable3D{parent, group}, M_mesh{std::move(mesh)}, M_shader{&shader}
    {
        M_mesh->addVertexBufferInstanced(M_buffer, 1, 0, Shaders::PhongGL::TransformationMatrix{},
                                         Shaders::PhongGL::NormalMatrix{},
                                         Shaders::PhongGL::Color3{});
    }

    struct instance_data
    {
        Matrix4 transformation;
        Matrix3x3 normal_matrix;
        Color3 color;
    };
    Containers::Array<instance_data> &instances() { return M_instances; }

    void add_object(instanced_drawable &drawable)
    {
        Containers::arrayAppend(M_instances, instance_data{});
        Containers::arrayAppend(M_objects, &drawable);
        M_added = true;
    }

private:
    Containers::Array<instance_data> M_instances;
    Containers::Array<instanced_drawable *> M_objects;
    std::shared_ptr<GL::Mesh> M_mesh;
    GL::Buffer M_buffer;
    Shaders::PhongGL *M_shader;
    bool M_added{false};

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;
};

class instanced_drawable : public SceneGraph::Drawable3D, public gfx_obj
{
public:
    explicit instanced_drawable(std::derived_from<gfx_obj_base> auto &parent,
                                instanced_drawables &group)
        : gfx_obj{&parent}, SceneGraph::Drawable3D{parent}
    {
        group.add_object(*this);
    }

    instanced_drawable(const instanced_drawable &) = delete;
    instanced_drawable &operator=(const instanced_drawable &) = delete;
    instanced_drawable(instanced_drawable &&) = delete;
    instanced_drawable &operator=(instanced_drawable &&) = delete;

    [[nodiscard]] virtual Color3 color() const { return Color3{1.0f}; }
    [[nodiscard]] virtual const Matrix3x3 *normal_matrix() const { return nullptr; }

    friend instanced_drawables;

    virtual ~instanced_drawable() = default;

private:
    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override {}
};

inline void instanced_drawables::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera)
{
    if (M_instances.isEmpty()) return;

    for (auto &&[instance, obj] : std::views::zip(M_instances, M_objects))
    {
        obj->draw(transformation, camera);

        instance.transformation = transformation * (obj->absoluteTransformation());
        instance.normal_matrix = instance.transformation.normalMatrix();
        instance.color = obj->color();
        obj->setClean();
    }

    // TODO: optimize: only update changed instances
    M_buffer.setData(M_instances, GL::BufferUsage::DynamicDraw);
    M_added = false;

    if (M_mesh->instanceCount() != (int) M_instances.size())
        M_mesh->setInstanceCount((int) M_instances.size());

    M_shader->draw(*M_mesh);
}

class colored_drawable : public instanced_drawable
{
public:
    explicit colored_drawable(gfx_obj &object, const Color3 &color, instanced_drawables &group)
        : instanced_drawable(object, group), M_color{color}
    {
    }

    [[nodiscard]] Color3 color() const override { return M_color; }
    void color(const Color3 &color)
    {
        M_color = color;
        setDirty();
    }

private:
    Color3 M_color;
};

class graphics_app : public Magnum::Platform::Application
{
public:
    struct key_state
    {
    public:
        [[nodiscard]] bool is_initial_press() const { return M_pressed && !M_repeated; }
        [[nodiscard]] bool is_pressed() const { return M_pressed; }
        [[nodiscard]] bool is_repeated() const { return M_repeated; }

    private:
        bool M_pressed{false};
        bool M_repeated{false};

        void press()
        {
            if (M_pressed) M_repeated = true;
            M_pressed = true;
        }

        void release()
        {
            M_pressed = false;
            M_repeated = false;
        }

        friend graphics_app;
    };
    graphics_app(const graphics_app &) = delete;
    graphics_app &operator=(const graphics_app &) = delete;
    graphics_app(graphics_app &&) = delete;
    graphics_app &operator=(graphics_app &&) = delete;

    graphics_app(const Arguments &arguments, const Vector2i &window_size,
                 physkit::quantity<physkit::si::degree> fov,
                 std::string_view title = "PhysKit Graphics Demo",
                 const Vector3 &initial_cam_pos = {0.0f, 0.0f, -5.0f},
                 const Vector3 &initial_cam_dir = {0.0f, 0.0f, 1.0f}, bool drag = false,
                 bool vsync = true)
        : Magnum::Platform::Application{arguments, Configuration{}
                                                       .setTitle(Containers::StringView{
                                                           title.data(), title.size()})
                                                       .setSize(window_size)},
          M_cam(M_scene, fov, initial_cam_pos, initial_cam_dir, window_size, window_size),
          M_drag(drag)
    {
        using namespace Math::Literals::ColorLiterals;

        M_shader = Shaders::PhongGL{Shaders::PhongGL::Configuration{}.setFlags(
            Shaders::PhongGL::Flag::VertexColor | Shaders::PhongGL::Flag::InstancedTransformation)};

        M_shader
            .setAmbientColor(0x222222_rgbf) // a touch brighter so objects are visible
            .setSpecularColor(0x330000_rgbf)
            .setLightPositions({{0.f, 5.f, 0.f, 0.f}});

        GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
        GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
        GL::Renderer::setClearColor(0x202020_rgbf);
        M_mouse.reserve(8);
        M_keys.reserve(128);

        if (!M_drag) setCursor(Cursor::HiddenLocked);

        M_timeline.start();
        if (vsync) set_vsync();
    }

    /// @brief Register and create, or fetch a previously registered shared mesh for this
    /// physkit::mesh instance. This mesh will be used across all methods with this physkit::mesh.
    /// @param phys_mesh The physkit::mesh to get the shared GL::Mesh for.
    /// @return A shared pointer to the GL::Mesh corresponding to the given physkit::mesh.
    std::shared_ptr<GL::Mesh> get_mesh(const physkit::mesh &phys_mesh)
    {
        if (auto it = M_phys_mesh_map.find(&phys_mesh); it != M_phys_mesh_map.end())
            return it->second;
        return M_phys_mesh_map
            .emplace(&phys_mesh, std::make_shared<GL::Mesh>(to_magnum_mesh(phys_mesh)))
            .first->second;
    }

    /// @brief Add a physics object to the scene with the specified color. This method will create
    /// or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param world The physics world that owns the object.
    /// @param handle Handle to the rigid body in the world.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(physkit::world &world, physkit::world::handle handle, Color3 color)
    {
        auto *phys_obj = new physics_obj{M_scene, world, handle};
        auto &obj = phys_obj->obj();
        std::shared_ptr<GL::Mesh> mesh;
        if (auto it = M_phys_mesh_map.find(&obj.mesh()); it == M_phys_mesh_map.end())
            mesh = M_phys_mesh_map
                       .emplace(&obj.mesh(), std::make_shared<GL::Mesh>(to_magnum_mesh(obj.mesh())))
                       .first->second;
        else
            mesh = it->second;

        internal_add_obj(phys_obj, std::move(mesh), color);

        M_physics_objs.push_back(phys_obj);
        return phys_obj;
    }

    /// @brief Add a physics object to the scene with the specified color. This method will create
    /// or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param world The physics world that owns the object.
    /// @param handle Handle to the rigid body in the world.
    /// @param mesh The shared GL::Mesh to use for rendering.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(physkit::world &world, physkit::world::handle handle,
                    std::shared_ptr<GL::Mesh> mesh, Color3 color)
    {

        return world.get_rigid(handle)
            .transform(
                [&](auto obj)
                {
                    if (auto it = M_phys_mesh_map.find(&obj->mesh()); it == M_phys_mesh_map.end())
                        M_phys_mesh_map.emplace(&obj->mesh(), mesh);
                    else if (it->second != mesh)
                        throw std::runtime_error(
                            "graphics_app::add_object: provided mesh does not match "
                            "physkit::object mesh");
                    auto *phys_obj = new physics_obj{M_scene, world, handle};

                    internal_add_obj(phys_obj, std::move(mesh), color);

                    M_physics_objs.push_back(phys_obj);
                    return phys_obj;
                })
            .value_or(nullptr);
    }

    /// @brief Add a graphics object to the scene with the specified color and shared mesh.
    /// @param mesh The shared GL::Mesh to use for rendering the object.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created gfx_obj instance.
    auto add_object(std::shared_ptr<GL::Mesh> mesh, Color3 color)
    {
        auto *obj = new gfx_obj{&M_scene};
        internal_add_obj(obj, std::move(mesh), color);
        return obj;
    }

    virtual ~graphics_app() = default;

protected:
    // called every frame to update the scene, before drawing
    virtual void update(physkit::quantity<physkit::si::second> dt) = 0;
    virtual void key_press(KeyEvent &event, bool pressed) {}
    virtual void pointer_move(PointerMoveEvent &event) {}
    virtual void pointer_press(PointerEvent &event, bool pressed) {}

    auto &cam() { return M_cam; }
    auto &keys() const { return M_keys; }
    auto &mouse() const { return M_mouse; } // map to pointer states
    auto &mouse_pos() const { return M_mouse_pos; }

    auto drag() const { return M_drag; }
    void drag(bool d)
    {
        if (M_drag == d) return;
        if (d)
        {
            setCursor(Cursor::Arrow);
            warpCursor(windowSize() / 2);
        }
        else
            setCursor(Cursor::HiddenLocked);
        M_drag = d;
    }

    /// @brief Enable or disable vertical synchronization (vsync).
    /// @param enabled If true, vsync is enabled; if false, vsync is disabled.
    void set_vsync(bool enabled = true)
    {
        if (enabled)
            setSwapInterval(1);
        else
            setSwapInterval(0);
    }

    /// @brief Set a frame rate limit for the application.
    /// @param fps The desired frame rate limit in hertz. If set to 0, no limit is applied.
    void frame_limit(physkit::quantity<physkit::si::hertz> fps)
    {
        if (fps == 0 * physkit::si::hertz)
            M_frame_limit = 0 * physkit::si::second;
        else
            M_frame_limit = 1 / fps;
    }

    physkit::quantity<physkit::si::hertz> frame_limit() const
    {
        return M_frame_limit != 0 * physkit::si::second ? 1 / M_frame_limit
                                                        : 0 * physkit::si::hertz;
    }

    physkit::quantity<physkit::si::second> dt() const
    {
        return M_timeline.previousFrameDuration() * physkit::si::second;
    }

    physkit::quantity<physkit::si::second> current_time() const
    {
        return M_timeline.previousFrameTime() * physkit::si::second;
    }

private:
    void internal_add_obj(gfx_obj *obj, std::shared_ptr<GL::Mesh> mesh, Color3 color)
    {
        instanced_drawables *instances{};
        if (auto it = M_mesh_drawables.find(mesh); it == M_mesh_drawables.end())
        {
            auto copy = mesh;
            instances = M_mesh_drawables[std::move(copy)] =
                new instanced_drawables{M_scene, &M_shaded, std::move(mesh), M_shader};
        }
        else
            instances = it->second;
        new colored_drawable{*obj, color, *instances};
    }

    void drawEvent() override
    {
        GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

        float forward{};
        float right{};
        float up{};
        if (M_keys[Key::S].is_pressed()) forward -= 1.0f;
        if (M_keys[Key::W].is_pressed()) forward += 1.0f;
        if (M_keys[Key::A].is_pressed()) right -= 1.0f;
        if (M_keys[Key::D].is_pressed()) right += 1.0f;
        if (M_keys[Key::Space].is_pressed()) up += 1.0f;
        if (M_keys[Key::LeftShift].is_pressed()) up -= 1.0f;

        M_cam.move(forward, right, up, dt());

        update(dt());

        for (auto *obj : M_physics_objs) obj->sync();

        M_shader.setProjectionMatrix(M_cam.projection_matrix());
        M_cam.draw(M_shaded, current_time());

        swapBuffers();
        redraw();
        while (M_timeline.currentFrameDuration() * physkit::si::second < M_frame_limit)
            std::this_thread::yield();
        M_timeline.nextFrame();
    }

    void keyPressEvent(KeyEvent &event) final
    {
        M_keys[event.key()].press();
        key_press(event, true);
        if (event.key() == Key::Esc) drag(!M_drag);
    }

    void keyReleaseEvent(KeyEvent &event) final
    {
        M_keys[event.key()].release();
        key_press(event, false);
    }

    void pointerPressEvent(PointerEvent &event) final
    {
        M_mouse[event.pointer()].press();
        pointer_press(event, true);
    }

    void pointerReleaseEvent(PointerEvent &event) final
    {
        M_mouse[event.pointer()].release();
        pointer_press(event, false);
    }

    void pointerMoveEvent(PointerMoveEvent &event) final
    {
        if (!M_drag ||
            M_mouse[Pointer::MouseLeft].is_pressed()) // may fire one frame late, not a big deal
            M_cam.pointer_move(event, M_drag);
        M_mouse_pos = event.position();
        pointer_move(event);
    }

    SceneGraph::Scene<SceneGraph::MatrixTransformation3D> M_scene;
    Shaders::PhongGL M_shader{NoCreate};
    SceneGraph::DrawableGroup3D M_shaded;
    camera M_cam;
    std::unordered_map<const physkit::mesh *, std::shared_ptr<GL::Mesh>> M_phys_mesh_map;
    std::unordered_map<std::shared_ptr<GL::Mesh>, instanced_drawables *> M_mesh_drawables;
    std::vector<physics_obj *> M_physics_objs;
    std::unordered_map<Key, key_state> M_keys;
    std::unordered_map<Pointer, key_state> M_mouse;
    Vector2 M_mouse_pos;
    Timeline M_timeline;
    physkit::quantity<physkit::si::second> M_frame_limit{};
    bool M_debug = false;
    bool M_drag = false;
};

namespace mesh_objs
{
// radius 1
constexpr auto cube()
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cubeSolid()));
}

// radius 1
constexpr auto sphere(unsigned int subdivisions = 3)
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::icosphereSolid(subdivisions)));
}

constexpr auto cone(unsigned int rings, unsigned int segments, float half_length)
{
    return std::make_shared<GL::Mesh>(
        MeshTools::compile(Primitives::coneSolid(rings, segments, half_length)));
}

constexpr auto cylinder(unsigned int rings, unsigned int segments, float half_length,
                        bool include_caps = true)
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cylinderSolid(
        rings, segments, half_length,
        include_caps ? Primitives::CylinderFlag::CapEnds : Primitives::CylinderFlag{})));
}

constexpr auto plane()
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::planeSolid()));
}
}; // namespace mesh_objs
} // namespace graphics
