#pragma once
#include "camera.h"
#include "convert.h"

#include <memory>
#include <print>
#include <thread>
#include <unordered_map>
#include <utility>

#include <physkit/physkit.h>

#include "dep/argparse.hpp"
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
#include <glaze/glaze.hpp>

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
        Quaternion rot =
            Quaternion::rotation(Rad(angle.numerical_value_in(physkit::si::radian)),
                                 to_magnum_vector<physkit::one, float>(axis.normalized()));
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
    explicit physics_obj(std::derived_from<gfx_obj_base> auto &parent, physkit::world_base &world,
                         physkit::world_base::handle handle)
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
                .rotate(to_magnum_quaternion<physkit::one, float>(res.value()->orientation()))
                .translate(to_magnum_vector<physkit::si::metre, float>(res.value()->pos()));
    }

private:
    physkit::world_base *M_world;
    physkit::world_base::handle M_handle;
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
} // namespace graphics

template <> struct glz::meta<physkit::body_type>
{
    using enum physkit::body_type;
    static constexpr auto value = enumerate(physkit::body_type::stat, physkit::body_type::dynam);
};

// template <> struct glz::meta<physkit::world_desc::integ_t>
// {
//     using enum physkit::world_desc::integ_t;
//     static constexpr auto value =
//         enumerate(physkit::world_desc::forward_euler, physkit::world_desc::semi_implicit_euler,
//                   physkit::world_desc::rk4);
// };

namespace graphics
{

class g_config
{
public:
    static constexpr auto default_fov = 45 * physkit::si::degree;
    static inline const auto default_cam_pos = physkit::fvec3{0, 0, 0} * physkit::si::metre;
    static inline const auto default_cam_dir = physkit::fvec3{0, 0, 1} * physkit::one;
    static constexpr std::string_view default_title = "PhysKit Graphics Demo";
    static constexpr auto default_window_size = Vector2i{1280, 720};
    static constexpr bool default_drag = false;
    static constexpr bool default_vsync = true;
    static constexpr auto default_time_step = 1.0f / 60.0f * physkit::si::second;

    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    auto &&read_file(this auto &&self, std::string_view path)
    {
        using namespace physkit::si::unit_symbols;
        struct box_type // NOLINT(cppcoreguidelines-pro-type-member-init)
        {
            std::string name; // must be "box"
            std::array<float, 3> half_extents;
        };

        struct pyramid_type // NOLINT(cppcoreguidelines-pro-type-member-init)
        {
            std::string name; // must be "pyramid"
            float height;
            float base_size;
        };

        struct sphere_type // NOLINT(cppcoreguidelines-pro-type-member-init)
        {
            std::string name; // must be "sphere"
            float radius;
            unsigned int stacks = 16;
            unsigned int sectors = 32;
        };

        struct obj // NOLINT(cppcoreguidelines-pro-type-member-init)
        {
            std::array<double, 3> pos;
            std::array<double, 3> vel = {0.0, 0.0, 0.0};
            std::array<double, 4> orientation = {1.0, 0.0, 0.0, 0.0}; // [w, x, y, z]
            std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0}; // rad/s
            std::array<double, 3> inertia_tensor = {1.0, 1.0, 1.0};   // diagonal [Ixx, Iyy, Izz]
            physkit::body_type type = physkit::body_type::dynam;
            std::variant<box_type, pyramid_type, sphere_type> mesh =
                box_type{"box", {0.5f, 0.5f, 0.5f}};
            double mass;
            std::array<float, 4> color = {1.0, 1.0, 1.0, 1.0};
        };

        struct world_config
        {
            std::array<double, 3> gravity = {0.0, -9.81, 0.0};
            // physkit::world_desc::integ_t integrator = physkit::world_desc::semi_implicit_euler;
            std::size_t solver_iterations = 10;
            std::vector<obj> objects{};
            std::optional<std::array<double, 3>> look_at;
            std::optional<double> fov;
            std::optional<std::array<double, 3>> cam_pos;
            std::optional<std::array<double, 3>> cam_dir;
            std::optional<std::string> title;
            std::optional<std::array<int, 2>> window_size;
            std::optional<bool> drag;
            std::optional<bool> vsync;
        };

        world_config config;
        std::string buffer;
        if (auto ez = glz::read_file_json(config, path, buffer))
        {
            std::println(std::cerr, "Error reading JSON config:\n{}",
                         glz::format_error(ez, buffer));
            std::cerr.flush();
            return std::forward<decltype(self)>(self);
        }

        self.M_gravity =
            physkit::vec3{config.gravity[0], config.gravity[1], config.gravity[2]} * m / s / s;
        // self.M_integrator = config.integrator;
        self.M_solver_iterations = config.solver_iterations;

        std::map<std::string, std::shared_ptr<physkit::mesh>> mesh_map;
        self.M_objects.reserve(config.objects.size());
        for (const obj &obj : config.objects)
        {
            auto mesh_name = *glz::write_json(obj.mesh);
            auto it = mesh_map.find(mesh_name);
            if (it == mesh_map.end())
            {
                obj.mesh.visit(
                    [&](auto &&mesh_desc)
                    {
                        using T = std::decay_t<decltype(mesh_desc)>;
                        if constexpr (std::same_as<T, box_type>)
                            mesh_map[mesh_name] = physkit::mesh::box(
                                physkit::vec3{mesh_desc.half_extents[0], mesh_desc.half_extents[1],
                                              mesh_desc.half_extents[2]} *
                                m);
                        else if constexpr (std::same_as<T, pyramid_type>)
                            mesh_map[mesh_name] = physkit::mesh::pyramid(mesh_desc.height * m,
                                                                         mesh_desc.base_size * m);
                        else if constexpr (std::same_as<T, sphere_type>)
                            mesh_map[mesh_name] = physkit::mesh::sphere(
                                mesh_desc.radius * m, mesh_desc.stacks, mesh_desc.sectors);
                    });
            }

            auto inertia = physkit::mat3<kg * m * m>::zero();
            inertia.set(0, 0, obj.inertia_tensor[0] * kg * m * m);
            inertia.set(1, 1, obj.inertia_tensor[1] * kg * m * m);
            inertia.set(2, 2, obj.inertia_tensor[2] * kg * m * m);

            self.M_objects.emplace_back(
                physkit::object_desc{obj.type}
                    .with_pos(physkit::vec3{obj.pos[0], obj.pos[1], obj.pos[2]} * m)
                    .with_vel(physkit::vec3{obj.vel[0], obj.vel[1], obj.vel[2]} * m / s)
                    .with_orientation(
                        physkit::quat<physkit::one>{obj.orientation[0], obj.orientation[1],
                                                    obj.orientation[2], obj.orientation[3]})
                    .with_ang_vel(physkit::vec3{obj.angular_velocity[0], obj.angular_velocity[1],
                                                obj.angular_velocity[2]} *
                                  rad / s)
                    .with_inertia_tensor(inertia)
                    .with_mass(obj.mass * kg)
                    .with_mesh(mesh_map.at(mesh_name)),
                Color3{obj.color[0], obj.color[1], obj.color[2]});
        }

        std::println("Read config from: {}", path);

        if (config.look_at)
        {
            self.M_look_at =
                physkit::vec3{(*config.look_at)[0], (*config.look_at)[1], (*config.look_at)[2]} * m;
            std::println("  Look at: {}", *self.M_look_at);
        }
        if (config.fov)
        {
            self.M_fov = *config.fov * deg;
            std::println("  FOV: {}", *self.M_fov);
        }
        if (config.cam_pos)
        {
            self.M_cam_pos =
                physkit::vec3{(*config.cam_pos)[0], (*config.cam_pos)[1], (*config.cam_pos)[2]} * m;
            std::println("  Camera position: {}", *self.M_cam_pos);
        }
        if (config.cam_dir)
        {
            self.M_cam_dir =
                physkit::vec3{(*config.cam_dir)[0], (*config.cam_dir)[1], (*config.cam_dir)[2]};
            std::println("  Camera direction: {}", *self.M_cam_dir);
        }
        if (config.title)
        {
            self.M_title = config.title;
            std::println("  Title: {}", *self.M_title);
        }
        if (config.window_size)
        {
            self.M_window_size = Vector2i{(*config.window_size)[0], (*config.window_size)[1]};
            std::println("  Window size: {}x{}", self.M_window_size->x(), self.M_window_size->y());
        }
        if (config.drag)
        {
            self.M_drag = config.drag;
            std::println("  Drag: {}", *self.M_drag ? "true" : "false");
        }
        if (config.vsync)
        {
            self.M_vsync = config.vsync;
            std::println("  VSync: {}", *self.M_vsync);
        }

        std::println("  Gravity: {}", *self.M_gravity);
        // std::println("  Integrator: {}",
        //              glz::reflect<physkit::world_desc::integ_t>::keys[static_cast<int>( // NOLINT
        //                  config.integrator)]);
        std::println("  Solver iterations: {}", config.solver_iterations);
        std::println("  Objects: {}", config.objects.size());
        for (std::size_t i = 0; i < self.M_objects.size(); ++i)
        {
            const auto &[o, color] = self.M_objects[i];
            std::println("    Object [{}]:", i);
            std::println("      Position: {}", o.pos());
            std::println("      Velocity: {}", o.vel());
            std::println("      Orientation: {}", o.orientation());
            std::println("      Angular velocity: {}", o.ang_vel());
            std::println("      Inertia tensor (diag): [{}, {}, {}]", o.inertia_tensor()[0, 0],
                         o.inertia_tensor()[1, 1], o.inertia_tensor()[2, 2]);
            std::println("      Mass: {}", o.mass());
            std::println(
                "      Type: {}",
                glz::reflect<physkit::body_type>::keys[static_cast<int>(o.type())]); // NOLINT
            std::println("      Color: ({}, {}, {})", color[0], color[1], color[2]);
            config.objects[i].mesh.visit(
                [](auto &&mesh_desc)
                {
                    using T = std::decay_t<decltype(mesh_desc)>;
                    if constexpr (std::same_as<T, box_type>)
                        std::println("      Mesh: box (half_extents: {})",
                                     physkit::vec3{mesh_desc.half_extents[0],
                                                   mesh_desc.half_extents[1],
                                                   mesh_desc.half_extents[2]} *
                                         m);
                    else if constexpr (std::same_as<T, pyramid_type>)
                        std::println("      Mesh: pyramid (height: {}, base_size: {})",
                                     mesh_desc.height * m, mesh_desc.base_size * m);
                    else if constexpr (std::same_as<T, sphere_type>)
                        std::println("      Mesh: sphere (radius: {}, stacks: {}, sectors: {})",
                                     mesh_desc.radius * m, mesh_desc.stacks, mesh_desc.sectors);
                });
        }
        std::cout.flush();

        return std::forward<decltype(self)>(self);
    }

    auto &&look_at(this auto &&self, const physkit::fvec3<physkit::si::metre> &target)
    {
        self.M_look_at = target;
        return std::forward<decltype(self)>(self);
    }

    auto &&look_at_or(this auto &&self, const physkit::fvec3<physkit::si::metre> &target)
    {
        if (!self.M_look_at) self.M_look_at = target;
        return std::forward<decltype(self)>(self);
    }

    auto &&fov(this auto &&self, physkit::quantity<physkit::si::degree> fov)
    {
        self.M_fov = fov;
        return std::forward<decltype(self)>(self);
    }
    auto &&fov_or(this auto &&self, physkit::quantity<physkit::si::degree> fov)
    {
        if (!self.M_fov) self.M_fov = fov;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_pos(this auto &&self, const physkit::fvec3<physkit::si::metre> &pos)
    {
        self.M_cam_pos = pos;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_pos_or(this auto &&self, const physkit::fvec3<physkit::si::metre> &pos)
    {
        if (!self.M_cam_pos) self.M_cam_pos = pos;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_dir(this auto &&self, const physkit::fvec3<physkit::one> &dir)
    {
        self.M_cam_dir = dir;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_dir_or(this auto &&self, const physkit::fvec3<physkit::one> &dir)
    {
        if (!self.M_cam_dir) self.M_cam_dir = dir;
        return std::forward<decltype(self)>(self);
    }
    auto &&title(this auto &&self, std::string_view title)
    {
        self.M_title = title;
        return std::forward<decltype(self)>(self);
    }
    auto &&title_or(this auto &&self, std::string_view title)
    {
        if (!self.M_title) self.M_title = title;
        return std::forward<decltype(self)>(self);
    }
    auto &&window_size(this auto &&self, const Vector2i &size)
    {
        self.M_window_size = size;
        return std::forward<decltype(self)>(self);
    }
    auto &&window_size_or(this auto &&self, const Vector2i &size)
    {
        if (!self.M_window_size) self.M_window_size = size;
        return std::forward<decltype(self)>(self);
    }
    auto &&drag(this auto &&self, bool drag)
    {
        self.M_drag = drag;
        return std::forward<decltype(self)>(self);
    }
    auto &&drag_or(this auto &&self, bool drag)
    {
        if (!self.M_drag) self.M_drag = drag;
        return std::forward<decltype(self)>(self);
    }
    auto &&vsync(this auto &&self, bool vsync)
    {
        self.M_vsync = vsync;
        return std::forward<decltype(self)>(self);
    }
    auto &&vsync_or(this auto &&self, bool vsync)
    {
        if (!self.M_vsync) self.M_vsync = vsync;
        return std::forward<decltype(self)>(self);
    }

    auto &&gravity(this auto &&self, const physkit::vec3<physkit::si::metre / physkit::si::second /
                                                         physkit::si::second> &gravity)
    {
        self.M_gravity = gravity;
        return std::forward<decltype(self)>(self);
    }
    auto &&gravity_or(this auto &&self,
                      const physkit::vec3<physkit::si::metre / physkit::si::second /
                                          physkit::si::second> &gravity)
    {
        if (!self.M_gravity) self.M_gravity = gravity;
        return std::forward<decltype(self)>(self);
    }
    // auto &&integrator(this auto &&self, physkit::world_desc::integ_t type)
    // {
    //     self.M_integrator = type;
    //     return std::forward<decltype(self)>(self);
    // }
    // auto &&integrator_or(this auto &&self, physkit::world_desc::integ_t type)
    // {
    //     if (!self.M_integrator) self.M_integrator = type;
    //     return std::forward<decltype(self)>(self);
    // }
    auto &&solver_iterations(this auto &&self, std::size_t iterations)
    {
        self.M_solver_iterations = iterations;
        return std::forward<decltype(self)>(self);
    }
    auto &&solver_iterations_or(this auto &&self, std::size_t iterations)
    {
        if (!self.M_solver_iterations) self.M_solver_iterations = iterations;
        return std::forward<decltype(self)>(self);
    }
    auto &&time_step(this auto &&self, physkit::quantity<physkit::si::second> time_step)
    {
        self.M_time_step = time_step;
        return std::forward<decltype(self)>(self);
    }
    auto &&time_step_or(this auto &&self, physkit::quantity<physkit::si::second> time_step)
    {
        if (!self.M_time_step) self.M_time_step = time_step;
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] auto window_size() const { return M_window_size.value_or(default_window_size); }
    [[nodiscard]] auto fov() const { return M_fov.value_or(default_fov); }
    [[nodiscard]] auto cam_pos() const { return M_cam_pos.value_or(default_cam_pos); }
    [[nodiscard]] auto cam_dir() const
    {
        if (M_look_at && M_cam_pos) return (*M_look_at - *M_cam_pos) / physkit::si::metre;
        return M_cam_dir.value_or(default_cam_dir);
    }
    [[nodiscard]] auto look_at() const { return M_look_at; }
    [[nodiscard]] auto title() const
    {
        return M_title.transform([](const std::string &s) { return std::string_view(s); })
            .value_or(default_title);
    }
    [[nodiscard]] auto drag() const { return M_drag.value_or(default_drag); }
    [[nodiscard]] auto vsync() const { return M_vsync.value_or(default_vsync); }
    [[nodiscard]] auto time_step() const { return M_time_step.value_or(default_time_step); }
    [[nodiscard]] auto &objects() const { return M_objects; }
    [[nodiscard]] auto world_desc() const
    {
        auto desc = physkit::world_desc::make();
        if (M_gravity) desc.with_gravity(*M_gravity);
        // if (M_integrator) desc.with_integrator(*M_integrator);
        if (M_solver_iterations) desc.with_solver_iterations(*M_solver_iterations);
        return desc;
    }
    [[nodiscard]] auto &args() const { return M_args; }

    [[nodiscard]] auto testing() const { return M_testing; }

    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    g_config(Magnum::Platform::Application::Arguments args, bool read_config = true) : M_args(args)
    {
        using namespace physkit::si::unit_symbols;
        using namespace physkit;
        argparse::ArgumentParser parser("PhysKit Configurable Demo", std::string(version_string()));
        parser.add_argument("--fov", "-f")
            .default_value(static_cast<double>(default_fov.numerical_value_in(deg)))
            .help("Camera field of view in degrees")
            .scan<'g', double>();
        parser.add_argument("--cam-pos", "-p")
            .default_value(std::format("{},{},{}", default_cam_pos.x().numerical_value_in(m),
                                       default_cam_pos.y().numerical_value_in(m),
                                       default_cam_pos.z().numerical_value_in(m)))
            .help("Comma-separated x,y,z components of the camera position in metres");
        parser.add_argument("--cam-dir", "-d")
            .default_value(std::format("{},{},{}", default_cam_dir.x().numerical_value_in(one),
                                       default_cam_dir.y().numerical_value_in(one),
                                       default_cam_dir.z().numerical_value_in(one)))
            .help("Comma-separated x,y,z components of the camera direction vector");
        parser.add_argument("--look-at", "-l")
            .help("Comma-separated x,y,z components of a point for the camera to look at");
        parser.add_argument("--title", "-t")
            .default_value(std::string(default_title))
            .help("Window title");
        parser.add_argument("--window-size", "-w")
            .default_value(std::format("{},{}", default_window_size.x(), default_window_size.y()))
            .help("Comma-separated width and height of the window, e.g. 1280,720");
        parser.add_argument("--drag")
            .default_value(default_drag)
            .implicit_value(true)
            .help("Enable or disable mouse drag to rotate camera");
        parser.add_argument("--vsync", "-v")
            .default_value(default_vsync)
            .implicit_value(true)
            .help("Enable or disable VSync");
        parser.add_argument("--time-step", "-s")
            .default_value(static_cast<double>(default_time_step.numerical_value_in(s)))
            .help("Time step in seconds")
            .scan<'g', double>();
        parser.add_argument("--testing").flag().help("Enable testing mode");
        if (read_config)
            parser.add_argument("--config", "-c")
                .help("Path to JSON config file for the world (CLI arguments will override JSON "
                      "config)");

        try
        {
            parser.parse_args(args.argc, args.argv);
        }
        catch (const std::exception &e)
        {
            std::println(std::cerr, "{}", e.what());
            std::cerr << parser;
            std::exit(1);
        }

        if (read_config)
            if (auto file = parser.present("--config")) read_file(*file);

        auto read_vec = []<int size, typename Rep = double>(
                            const std::string &s, std::integral_constant<int, size> size_constant,
                            Rep rep = {}) -> physkit::vec<size, one, Rep>
        {
            physkit::vec<size, one, Rep> result{};
            std::istringstream ss(s);
            std::string item;
            int i = 0;
            while (std::getline(ss, item, ',') && i < size)
            {
                try
                {
                    Rep value;
                    std::istringstream(item) >> value;
                    result.set(i++, value);
                }
                catch (const std::exception &e)
                {
                    std::println(std::cerr, "Error parsing vector from string \"{}\": {}", s,
                                 e.what());
                    std::exit(1);
                }
            }
            if (i != size)
            {
                std::println(
                    std::cerr,
                    "Error parsing vector from string \"{}\": expected {} components but got {}", s,
                    size, i);
                std::exit(1);
            }
            return result;
        };

        try
        {
            auto fov_val = parser.get<double>("--fov") * deg;
            auto cam_pos_val =
                read_vec(parser.get<std::string>("--cam-pos"), std::integral_constant<int, 3>{}) *
                m;
            auto cam_dir_val =
                read_vec(parser.get<std::string>("--cam-dir"), std::integral_constant<int, 3>{});
            auto title_val = parser.get<std::string>("--title");
            auto window_size_val = to_magnum_vector<one, int>(read_vec(
                parser.get<std::string>("--window-size"), std::integral_constant<int, 2>{}, int{}));
            auto drag_val = parser.get<bool>("--drag");
            auto vsync_val = parser.get<bool>("--vsync");
            auto time_step_val = parser.get<double>("--time-step") * s;
            M_testing = parser.get<bool>("--testing");

            if (parser.is_used("--fov")) fov(fov_val);
            if (parser.is_used("--cam-pos")) cam_pos(cam_pos_val);
            if (parser.is_used("--cam-dir")) cam_dir(cam_dir_val);
            if (parser.is_used("--title")) title(title_val);
            if (parser.is_used("--window-size")) window_size(window_size_val);
            if (parser.is_used("--drag")) drag(drag_val);
            if (parser.is_used("--vsync")) vsync(vsync_val);
            if (parser.is_used("--time-step")) time_step(time_step_val);

            if (auto look_at_str = parser.present("--look-at"))
                look_at(read_vec(*look_at_str, std::integral_constant<int, 3>{}) * m);
        }
        catch (const std::exception &e)
        {
            std::println(std::cerr, "Error parsing CLI arguments: {}", e.what());
            std::exit(1);
        }
    }

private:
    std::vector<std::pair<physkit::object_desc, Color3>> M_objects;
    Magnum::Platform::Application::Arguments M_args;
    std::optional<Vector2i> M_window_size;
    std::optional<physkit::quantity<physkit::si::degree, float>> M_fov;
    std::optional<std::string> M_title;
    std::optional<physkit::fvec3<physkit::si::metre>> M_cam_pos;
    std::optional<physkit::fvec3<physkit::one>> M_cam_dir;
    std::optional<physkit::fvec3<physkit::si::metre>> M_look_at;
    std::optional<bool> M_drag = default_drag;
    std::optional<bool> M_vsync = default_vsync;
    std::optional<physkit::vec3<physkit::si::metre / physkit::si::second / physkit::si::second>>
        M_gravity;
    // std::optional<physkit::world_desc::integ_t> M_integrator;
    std::optional<std::size_t> M_solver_iterations;
    std::optional<physkit::quantity<physkit::si::second>> M_time_step;
    bool M_testing{false};
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
    auto add_object(physkit::world_base::handle handle, Color3 color)
    {
        auto *phys_obj = new physics_obj{M_scene, *M_world, handle};
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
    auto add_object(physkit::world_base::handle handle, std::shared_ptr<GL::Mesh> mesh,
                    Color3 color)
    {

        return M_world->get_rigid(handle)
            .transform(
                [&](auto obj)
                {
                    if (auto it = M_phys_mesh_map.find(&obj->mesh()); it == M_phys_mesh_map.end())
                        M_phys_mesh_map.emplace(&obj->mesh(), mesh);
                    else if (it->second != mesh)
                        throw std::runtime_error(
                            "graphics_app::add_object: provided mesh does not match "
                            "physkit::object mesh");
                    auto *phys_obj = new physics_obj{M_scene, *M_world, handle};

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

    auto &world() { return *M_world; }

    auto &physics_objects() const { return M_physics_objs; }

    graphics_app(const g_config &config)
        : Magnum::Platform::Application{config.args(),
                                        Configuration{}
                                            .setTitle(Containers::StringView{config.title().data(),
                                                                             config.title().size()})
                                            .setSize(config.window_size())},
          M_cam(M_scene, config.fov(), config.cam_pos(), config.cam_dir(), config.window_size(),
                config.window_size()),
          M_drag(config.drag()), M_testing(config.testing())
    {
        using namespace Math::Literals::ColorLiterals;

        M_world =
            std::make_unique<physkit::world<physkit::semi_implicit_euler>>(config.world_desc());

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
        if (config.vsync()) set_vsync();

        for (const auto &[obj_desc, color] : config.objects())
            add_object(M_world->create_rigid(obj_desc), color);

        M_stepper = physkit::stepper(*M_world, config.time_step());
    }

    virtual ~graphics_app()
    {
        if (M_testing) prompt_results();
    }

protected:
    // called every frame to update the scene, before drawing. stepper::update() is called after
    // this.
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
        M_stepper.update(dt());

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

    static void prompt_results()
    {
#if defined(_WIN32)
        int result = MessageBoxA(nullptr, "Success?", "Test Result", MB_YESNO | MB_ICONQUESTION);
#elif defined(__APPLE__)
        int result =
            std::system("osascript -e 'display dialog \"Success?\" buttons {\"No\", \"Yes\"} "
                        "default button \"Yes\" with title \"Test Result\"' >/dev/null 2>&1");
#else
        int result =
            std::system(R"(zenity --question --text="Success?" --title="Test Result" 2>/dev/null)");
#endif
        std::exit(result == 0 ? 0 : 1);
    }

    SceneGraph::Scene<SceneGraph::MatrixTransformation3D> M_scene;
    Shaders::PhongGL M_shader{NoCreate};
    SceneGraph::DrawableGroup3D M_shaded;
    std::unique_ptr<physkit::world_base> M_world;
    physkit::stepper M_stepper;
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
    bool M_testing = false;
};

namespace mesh_objs
{
// radius 1
inline auto cube()
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cubeSolid()));
}

// radius 1
inline auto sphere(unsigned int subdivisions = 3)
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::icosphereSolid(subdivisions)));
}

inline auto cone(unsigned int rings, unsigned int segments, float half_length)
{
    return std::make_shared<GL::Mesh>(
        MeshTools::compile(Primitives::coneSolid(rings, segments, half_length)));
}

inline auto cylinder(unsigned int rings, unsigned int segments, float half_length,
                     bool include_caps = true)
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cylinderSolid(
        rings, segments, half_length,
        include_caps ? Primitives::CylinderFlag::CapEnds : Primitives::CylinderFlag{})));
}

inline auto plane()
{
    return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::planeSolid()));
}
}; // namespace mesh_objs
} // namespace graphics
