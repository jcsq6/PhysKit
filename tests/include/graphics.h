#pragma once

#include "camera.h"
#include "convert.h"

#include <cstddef>
#include <memory>
#include <physkit/physkit.h>

#include <Corrade/Containers/StringView.h>
#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/Primitives/Cone.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Plane.h>
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

    explicit object(const physkit::particle &p) : M_particle{p} {}

    [[nodiscard]] const physkit::particle &particle() const { return M_particle; }

    void mesh(const physkit::mesh &m) { M_mesh = &m; }
    [[nodiscard]] const physkit::mesh &mesh() const { return *M_mesh; }

    virtual ~object() = default;

private:
    physkit::particle M_particle;
    const physkit::mesh *M_mesh{};
};
} // namespace physkit

namespace graphics
{

using namespace Magnum;
using gfx_obj = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;

class physics_obj : public gfx_obj
{
public:
    explicit physics_obj(gfx_obj *parent, std::unique_ptr<physkit::object> obj = nullptr)
        : gfx_obj{parent}, M_phys{std::move(obj)}
    {
    }

    void obj(std::unique_ptr<physkit::object> o) { M_phys = std::move(o); }

    decltype(auto) obj(this auto &&self) { return std::forward_like<decltype(self)>(*self.M_phys); }

private:
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
        M_mesh->addVertexBufferInstanced(M_buffer, 1, 0, Shaders::PhongGL::Position{},
                                         Shaders::PhongGL::Normal{}, Shaders::PhongGL::Color3{});
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

class instanced_drawable : public gfx_obj
{
public:
    explicit instanced_drawable(gfx_obj &parent, instanced_drawables &group) : gfx_obj{&parent}
    {
        group.add_object(*this);
    }

    [[nodiscard]] virtual Color3 color() const { return Color3{1.0f}; }
    [[nodiscard]] virtual const Matrix3x3 *normal_matrix() const { return nullptr; }

    friend instanced_drawables;
};

inline void instanced_drawables::draw(const Matrix4 &transformation,
                                      SceneGraph::Camera3D & /*camera*/)
{
    if (M_instances.isEmpty()) return;

    for (auto &&[instance, obj] : std::views::zip(M_instances, M_objects))
    {
        if (obj->isDirty())
        {
            instance.transformation = transformation * (obj->transformation());
            if (obj->normal_matrix() != nullptr) instance.normal_matrix = *obj->normal_matrix();
            instance.color = obj->color();
            obj->setClean();

            if (!M_added) M_buffer.setSubData(&instance - M_instances.data(), {instance});
        }
    }

    if (M_added)
    {
        M_buffer.setData(M_instances, GL::BufferUsage::DynamicDraw);
        M_added = false;
    }

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
    static constexpr auto m = mp_units::si::unit_symbols::m;

public:
    graphics_app(const graphics_app &) = delete;
    graphics_app &operator=(const graphics_app &) = delete;
    graphics_app(graphics_app &&) = delete;
    graphics_app &operator=(graphics_app &&) = delete;

    graphics_app(const Arguments &arguments, const Vector2i &window_size,
                 physkit::quantity<physkit::si::degree> fov,
                 std::string_view title = "PhysKit Graphics Demo")
        : Magnum::Platform::Application{arguments, Configuration{}.setTitle(Containers::StringView{
                                                       title.data(), title.size()})},
          M_cam(M_scene, fov, {0.0f, 0.0f, 0.0f}, Magnum::Vector3::yAxis(),
                Magnum::Vector3::xAxis(), window_size, window_size)
    {
        using namespace Math::Literals::ColorLiterals;

        M_shader = Shaders::PhongGL{
            Shaders::PhongGL::Configuration{}.setFlags(Shaders::PhongGL::Flag::VertexColor)};

        M_shader.setAmbientColor(0x111111_rgbf).setSpecularColor(0x330000_rgbf);
    }

    /// @brief Register and create, or fetch a previously registered shared mesh for this physkit::mesh instance. This mesh will be used across all methods with this physkit::mesh.
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

    /// @brief Add a physics object to the scene with the specified color. This method will create or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param obj The physics object to add to the scene.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(std::unique_ptr<physkit::object> obj, Color3 color)
    {
        auto *phys_obj = new physics_obj{&M_scene, std::move(obj)};
        std::shared_ptr<GL::Mesh> mesh;
        if (auto it = M_phys_mesh_map.find(&phys_obj->obj().mesh()); it == M_phys_mesh_map.end())
            mesh =
                M_phys_mesh_map
                    .emplace(&phys_obj->obj().mesh(),
                             std::make_shared<GL::Mesh>(to_magnum_mesh(phys_obj->obj().mesh())))
                    .first->second;
        else
            mesh = it->second;

        internal_add_obj(phys_obj, std::move(mesh), color);

        return phys_obj;
    }

        /// @brief Add a physics object to the scene with the specified color. This method will create or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param obj The physics object to add to the scene.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(std::unique_ptr<physkit::object> obj, std::shared_ptr<GL::Mesh> mesh, Color3 color)
    {
        if (auto it = M_phys_mesh_map.find(&obj->mesh()); it == M_phys_mesh_map.end())
        {
            M_phys_mesh_map.emplace(&obj->mesh(), mesh);
        }
        else
        {
            if (it->second != mesh)
                throw std::runtime_error("graphics_app::add_object: provided mesh does not match physkit::object mesh");
        }

        auto *phys_obj = new physics_obj{&M_scene, std::move(obj)};

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
    virtual void key_press(KeyEvent &event) {}
    virtual void pointer_move(PointerMoveEvent &event) {}

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

        update(M_dt);

        M_cam.draw(M_shaded);

        swapBuffers();
    }
    void keyPressEvent(KeyEvent &event) override
    {
        M_cam.key_press(event, M_dt);
        key_press(event);
    }

    void pointerMoveEvent(PointerMoveEvent &event) override
    {
        M_cam.pointer_move(event);
        pointer_move(event);
    }

    SceneGraph::Scene<SceneGraph::MatrixTransformation3D> M_scene;
    Shaders::PhongGL M_shader;
    SceneGraph::DrawableGroup3D M_shaded;
    camera M_cam;
    physkit::quantity<physkit::si::second> M_dt{0.1 * physkit::si::second}; // NOLINT
    std::unordered_map<const physkit::mesh *, std::shared_ptr<GL::Mesh>> M_phys_mesh_map;
    std::unordered_map<std::shared_ptr<GL::Mesh>, instanced_drawables *> M_mesh_drawables;
    bool M_debug = false;
};

namespace mesh_objs {
    constexpr auto cube()
    {
        return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cubeSolid()));
    }

    constexpr auto sphere(unsigned int subdivisions = 3)
    {
        return std::make_shared<GL::Mesh>(
            MeshTools::compile(Primitives::icosphereSolid(subdivisions)));
    }

    constexpr auto cone(unsigned int rings, unsigned int segments, float half_length)
    {
        return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::coneSolid(rings, segments, half_length)));
    }

    constexpr auto cylinder(unsigned int rings, unsigned int segments, float half_length)
    {
        return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cylinderSolid(rings, segments, half_length)));
    }

    constexpr auto plane()
    {
        return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::planeSolid()));
    }
};
} // namespace graphics