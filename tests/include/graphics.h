#pragma once
#include "camera.h"
#include "convert.h"

#include <cstddef>
#include <memory>
#include <physkit/physkit.h>

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/StringView.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Renderer.h>
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
#include <Magnum/Trade/MeshData.h>
#include <unordered_map>
#include <utility>

namespace physkit
{
class object // TODO: implement actual physkit::object
{
public:
    object() = default;
    object(object &&) noexcept = default;
    object &operator=(object &&) noexcept = default;
    object(const object &) = default;
    object &operator=(const object &) = default;

    explicit object(const physkit::particle &p, std::shared_ptr<physkit::mesh> m)
        : M_particle{p}, M_mesh{std::move(m)}
    {
    }

    [[nodiscard]] const physkit::particle &particle() const { return M_particle; }

    void mesh(std::shared_ptr<physkit::mesh> m) { M_mesh = std::move(m); }
    [[nodiscard]] const physkit::mesh &mesh() const { return *M_mesh; }

    virtual ~object() = default;

private:
    physkit::particle M_particle;
    std::shared_ptr<const physkit::mesh> M_mesh;
};
} // namespace physkit

namespace graphics
{

using namespace Magnum;
using gfx_obj = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;

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

class physics_obj : public SceneGraph::Drawable3D, public gfx_obj
{
public:
    explicit physics_obj(gfx_obj &parent, std::unique_ptr<physkit::object> obj = nullptr)
        : SceneGraph::Drawable3D{parent}, gfx_obj(&parent), M_phys{std::move(obj)}
    {
    }

    void obj(std::unique_ptr<physkit::object> o) { M_phys = std::move(o); }

    decltype(auto) obj(this auto &&self) { return std::forward_like<decltype(self)>(*self.M_phys); }

private:
    void draw(const Matrix4 &transformation, SceneGraph::Camera3D & /*camera*/) override
    {
        translate(to_magnum_vector<float>(M_phys->particle().pos));
    }
    std::unique_ptr<physkit::object> M_phys; // TODO: use shared_ptr? plain ptr?
};

class instanced_drawable;

class instanced_drawables : public SceneGraph::Drawable3D
{
public:
    explicit instanced_drawables(gfx_obj &parent, SceneGraph::DrawableGroup3D *group,
                                 std::shared_ptr<GL::Mesh> mesh, GL::AbstractShaderProgram &shader)
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
    GL::AbstractShaderProgram *M_shader;
    bool M_added{false};

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D & /*camera*/) override;
};

class instanced_drawable : public SceneGraph::Drawable3D, public gfx_obj
{
public:
    explicit instanced_drawable(gfx_obj &parent, instanced_drawables &group)
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
                 const Vector3 &initial_cam_dir = {0.0f, 0.0f, 1.0f}, bool drag = false)
        : Magnum::Platform::Application{arguments, Configuration{}.setTitle(Containers::StringView{
                                                       title.data(), title.size()})},
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

        if (!M_drag)
            setCursor(Cursor::HiddenLocked);
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
    /// @param obj The physics object to add to the scene.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(std::unique_ptr<physkit::object> obj, Color3 color)
    {
        auto *phys_obj = new physics_obj{M_scene, std::move(obj)};
        std::shared_ptr<GL::Mesh> mesh;
        if (auto it = M_phys_mesh_map.find(&phys_obj->obj().mesh()); it == M_phys_mesh_map.end())
            mesh = M_phys_mesh_map
                       .emplace(&phys_obj->obj().mesh(),
                                std::make_shared<GL::Mesh>(to_magnum_mesh(phys_obj->obj().mesh())))
                       .first->second;
        else
            mesh = it->second;

        internal_add_obj(phys_obj, std::move(mesh), color);

        return phys_obj;
    }

    /// @brief Add a physics object to the scene with the specified color. This method will create
    /// or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param obj The physics object to add to the scene.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(std::unique_ptr<physkit::object> obj, std::shared_ptr<GL::Mesh> mesh,
                    Color3 color)
    {
        if (auto it = M_phys_mesh_map.find(&obj->mesh()); it == M_phys_mesh_map.end())
            M_phys_mesh_map.emplace(&obj->mesh(), mesh);
        else if (it->second != mesh)
            throw std::runtime_error("graphics_app::add_object: provided mesh does not match "
                                     "physkit::object mesh"); // caller loses ownership of obj?
                                                              // TODO: fix.

        auto *phys_obj = new physics_obj{M_scene, std::move(obj)};

        internal_add_obj(phys_obj, std::move(mesh), color);

        return phys_obj;
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

        if (M_keys[Key::S].is_pressed()) M_cam.move_forward(-M_dt);
        if (M_keys[Key::W].is_pressed()) M_cam.move_forward(M_dt);
        if (M_keys[Key::A].is_pressed()) M_cam.move_right(-M_dt);
        if (M_keys[Key::D].is_pressed()) M_cam.move_right(M_dt);
        if (M_keys[Key::Space].is_pressed()) M_cam.move_up(M_dt);
        if (M_keys[Key::LeftShift].is_pressed()) M_cam.move_up(-M_dt);

        update(M_dt);

        M_shader.setProjectionMatrix(M_cam.projection_matrix());
        M_cam.draw(M_shaded);

        swapBuffers();
        redraw();
    }

    void keyPressEvent(KeyEvent &event) final
    {
        M_keys[event.key()].press();
        key_press(event, true);
        if (event.key() == Key::Esc)
           drag(!M_drag);
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
    physkit::quantity<physkit::si::second> M_dt{0.1 * physkit::si::second}; // NOLINT
    std::unordered_map<const physkit::mesh *, std::shared_ptr<GL::Mesh>> M_phys_mesh_map;
    std::unordered_map<std::shared_ptr<GL::Mesh>, instanced_drawables *> M_mesh_drawables;
    std::unordered_map<Key, key_state> M_keys;
    std::unordered_map<Pointer, key_state> M_mouse;
    Vector2 M_mouse_pos;
    bool M_debug = false;
    bool M_drag = false;
};

namespace mesh_objs
{
constexpr auto cube()
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cubeSolid()));
}

constexpr auto sphere(unsigned int subdivisions = 3)
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::icosphereSolid(subdivisions)));
}

constexpr auto cone(unsigned int rings, unsigned int segments, float half_length)
{
    return std::make_shared<GL::Mesh>(
        MeshTools::compile(Primitives::coneSolid(rings, segments, half_length)));
}

constexpr auto cylinder(unsigned int rings, unsigned int segments, float half_length)
{
    return std::make_shared<GL::Mesh>(
        MeshTools::compile(Primitives::cylinderSolid(rings, segments, half_length)));
}

constexpr auto plane()
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::planeSolid()));
}
}; // namespace mesh_objs
} // namespace graphics