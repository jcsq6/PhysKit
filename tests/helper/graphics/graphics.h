#pragma once

#include "detail/macro.h"

#ifndef GRAPHICS_IN_MODULE_IMPL

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <coroutine> // IWYU pragma: keep
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>
#endif

#include <GLFW/glfw3.h>
#ifndef PHYSKIT_MODULES
#include <mp-units/framework.h>
#include <physkit/physkit.h>
#endif

#include "camera.h"
#include "convert.h"

#endif

GRAPHICS_EXPORT namespace graphics
{
    using namespace Magnum;
    using gfx_obj_base = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;

    class gfx_obj : public gfx_obj_base
    {
    public:
        using SceneGraph::Object<SceneGraph::MatrixTransformation3D>::Object;

        void rotate(physkit::quantity<mp_units::si::radian, float> angle,
                    const physkit::vec3<mp_units::one> &axis)
        {
            Quaternion rot =
                Quaternion::rotation(Rad(angle.numerical_value_in(mp_units::si::radian)),
                                     to_magnum_vector<mp_units::one, float>(axis.normalized()));
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

    class graphics_app;

    class physics_obj : public gfx_obj
    {
    public:
        explicit physics_obj(std::derived_from<gfx_obj_base> auto &parent, graphics_app &app,
                             physkit::world_base::handle handle)
            : gfx_obj(&parent), M_app{&app}, M_handle{handle}
        {
        }

        physics_obj(const physics_obj &) = delete;
        physics_obj &operator=(const physics_obj &) = delete;
        physics_obj(physics_obj &&) = delete;
        physics_obj &operator=(physics_obj &&) = delete;

        ~physics_obj();

        [[nodiscard]] auto handle() const { return M_handle; }

        physkit::object &obj();
        [[nodiscard]] const physkit::object &obj() const;

        void sync();

    private:
        graphics_app *M_app;
        physkit::world_base::handle M_handle;
    };

    class instanced_drawable;

    class instanced_drawables
    {
    public:
        struct instance_data
        {
            Matrix4 transformation;
            Matrix3x3 normal_matrix;
            Color4 color;
        };

        class pass : public SceneGraph::Drawable3D
        {
        public:
            enum class kind
            {
                opaque,
                transparent
            };
            pass(gfx_obj_base &parent, SceneGraph::DrawableGroup3D *group,
                 instanced_drawables &owner, kind k)
                : SceneGraph::Drawable3D{parent, group}, M_owner{&owner}, M_kind{k}
            {
            }

        private:
            instanced_drawables *M_owner;
            kind M_kind;

            void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override
            {
                if (M_kind == kind::opaque)
                    M_owner->draw_opaque(transformation, camera);
                else
                    M_owner->draw_transparent(transformation, camera);
            }
        };

        template <typename Parent>
            requires std::derived_from<Parent, gfx_obj_base>
        explicit instanced_drawables(Parent &parent, SceneGraph::DrawableGroup3D *opaque_group,
                                     SceneGraph::DrawableGroup3D *transparent_group,
                                     std::shared_ptr<GL::Mesh> mesh, Shaders::PhongGL &shader)
            : M_mesh{std::move(mesh)}, M_shader{&shader}
        {
            // Two sub-drawables, one per render pass, sharing this instance list.
            M_opaque_pass = new pass{parent, opaque_group, *this, pass::kind::opaque};
            M_transparent_pass =
                new pass{parent, transparent_group, *this, pass::kind::transparent};

            M_mesh->addVertexBufferInstanced(
                M_buffer, 1, 0, Shaders::PhongGL::TransformationMatrix{},
                Shaders::PhongGL::NormalMatrix{}, Shaders::PhongGL::Color4{});
        }

        Containers::Array<instance_data> &instances() { return M_instances; }

        void add_object(instanced_drawable &drawable)
        {
            Containers::arrayAppend(M_instances, instance_data{});
            Containers::arrayAppend(M_objects, &drawable);
        }

        void remove_object(instanced_drawable &drawable)
        {
            auto *it = std::ranges::find(M_objects, &drawable);
            if (it != M_objects.end())
            {
                auto index = std::distance(M_objects.begin(), it);
                Containers::arrayRemove(M_instances, index);
                Containers::arrayRemove(M_objects, index);
            }
        }

    private:
        Containers::Array<instance_data> M_instances;
        Containers::Array<instanced_drawable *> M_objects;
        Containers::Array<std::size_t> M_opaque_indices;
        Containers::Array<std::size_t> M_transparent_indices;
        Containers::Array<instance_data> M_upload_scratch;
        std::shared_ptr<GL::Mesh> M_mesh;
        GL::Buffer M_buffer;
        Shaders::PhongGL *M_shader;
        pass *M_opaque_pass{};
        pass *M_transparent_pass{};

        void draw_opaque(const Matrix4 &transformation, SceneGraph::Camera3D &camera);
        void draw_transparent(const Matrix4 &transformation, SceneGraph::Camera3D &camera);
    };

    class instanced_drawable : public SceneGraph::Drawable3D, public gfx_obj
    {
    public:
        explicit instanced_drawable(std::derived_from<gfx_obj_base> auto &parent,
                                    instanced_drawables &group)
            : gfx_obj{&parent}, SceneGraph::Drawable3D{parent}, M_group{&group}
        { group.add_object(*this); }

        instanced_drawable(const instanced_drawable &) = delete;
        instanced_drawable &operator=(const instanced_drawable &) = delete;
        instanced_drawable(instanced_drawable &&) = delete;
        instanced_drawable &operator=(instanced_drawable &&) = delete;

        [[nodiscard]] virtual Color4 color() const { return Color4{1.0f, 1.0f, 1.0f, 1.0f}; }
        [[nodiscard]] virtual const Matrix3x3 *normal_matrix() const { return nullptr; }

        friend instanced_drawables;

        virtual ~instanced_drawable()
        {
            if (M_group != nullptr)
            {
                M_group->remove_object(*this);
                M_group = nullptr;
            }
        }

    private:
        void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override {}

        instanced_drawables *M_group;
    };

    inline void instanced_drawables::draw_opaque(const Matrix4 &transformation,
                                                 SceneGraph::Camera3D &camera)
    {
        Containers::arrayResize(M_opaque_indices, 0);
        Containers::arrayResize(M_transparent_indices, 0);
        if (M_instances.isEmpty()) return;

        for (std::size_t i = 0; i < M_instances.size(); ++i)
        {
            auto *obj = M_objects[i];
            auto &instance = M_instances[i];
            obj->draw(transformation, camera);
            instance.transformation = transformation * (obj->absoluteTransformation());
            instance.normal_matrix = instance.transformation.normalMatrix();
            instance.color = obj->color();
            obj->setClean();
            if (instance.color.a() < 1.0f)
                Containers::arrayAppend(M_transparent_indices, i);
            else
                Containers::arrayAppend(M_opaque_indices, i);
        }

        if (M_opaque_indices.isEmpty()) return;

        Containers::arrayResize(M_upload_scratch, M_opaque_indices.size());
        for (std::size_t k = 0; k < M_opaque_indices.size(); ++k)
            M_upload_scratch[k] = M_instances[M_opaque_indices[k]];

        M_buffer.setData(M_upload_scratch, GL::BufferUsage::DynamicDraw);
        M_mesh->setInstanceCount(static_cast<int>(M_upload_scratch.size()));
        M_shader->draw(*M_mesh);
    }

    inline void instanced_drawables::draw_transparent(const Matrix4 & /*transformation*/,
                                                      SceneGraph::Camera3D & /*camera*/)
    {
        if (M_transparent_indices.isEmpty()) return;

        std::ranges::sort(M_transparent_indices,
                          [this](std::size_t a, std::size_t b)
                          {
                              return M_instances[a].transformation.translation().z() <
                                     M_instances[b].transformation.translation().z();
                          });

        GL::Renderer::setDepthMask(false);
        Containers::arrayResize(M_upload_scratch, 1);
        M_mesh->setInstanceCount(1);

        for (const auto idx : M_transparent_indices)
        {
            M_upload_scratch[0] = M_instances[idx];
            M_buffer.setData(M_upload_scratch, GL::BufferUsage::DynamicDraw);

            GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Front);
            M_shader->draw(*M_mesh);

            GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Back);
            M_shader->draw(*M_mesh);
        }

        GL::Renderer::setDepthMask(true);
        GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Back);
    }

    class colored_drawable : public instanced_drawable
    {
    public:
        explicit colored_drawable(gfx_obj &object, const Color4 &color, instanced_drawables &group)
            : instanced_drawable(object, group), M_color{color}
        {
        }

        [[nodiscard]] Color4 color() const override { return M_color; }
        void color(const Color4 &color)
        {
            M_color = color;
            setDirty();
        }

    private:
        Color4 M_color;
    };
} // namespace graphics

GRAPHICS_EXPORT
namespace graphics
{
class g_config
{
public:
    static constexpr auto default_fov = 45 * mp_units::si::degree;
    static inline const auto default_cam_pos = physkit::fvec3{0, 0, 0} * mp_units::si::metre;
    static inline const auto default_cam_dir = physkit::fvec3{0, 0, 1} * mp_units::one;
    static constexpr std::string_view default_title = "PhysKit Graphics Demo";
    static constexpr auto default_window_size = Vector2i{1280, 720};
    static constexpr bool default_drag = false;
    static constexpr bool default_vsync = true;
    static constexpr auto default_time_step = 1.0f / 60.0f * mp_units::si::second;
    static constexpr auto default_record_duration = 10.0f * mp_units::si::second;
    static constexpr int default_record_fps = 60;

    template <typename Self> Self &&read_file(this Self &&self, std::string_view path);

    auto &&look_at(this auto &&self, const physkit::fvec3<mp_units::si::metre> &target)
    {
        self.M_look_at = target;
        return std::forward<decltype(self)>(self);
    }

    auto &&look_at_or(this auto &&self, const physkit::fvec3<mp_units::si::metre> &target)
    {
        if (!self.M_look_at) self.M_look_at = target;
        return std::forward<decltype(self)>(self);
    }

    auto &&fov(this auto &&self, physkit::quantity<mp_units::si::degree> fov)
    {
        self.M_fov = fov;
        return std::forward<decltype(self)>(self);
    }
    auto &&fov_or(this auto &&self, physkit::quantity<mp_units::si::degree> fov)
    {
        if (!self.M_fov) self.M_fov = fov;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_pos(this auto &&self, const physkit::fvec3<mp_units::si::metre> &pos)
    {
        self.M_cam_pos = pos;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_pos_or(this auto &&self, const physkit::fvec3<mp_units::si::metre> &pos)
    {
        if (!self.M_cam_pos) self.M_cam_pos = pos;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_dir(this auto &&self, const physkit::fvec3<mp_units::one> &dir)
    {
        self.M_cam_dir = dir;
        return std::forward<decltype(self)>(self);
    }
    auto &&cam_dir_or(this auto &&self, const physkit::fvec3<mp_units::one> &dir)
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

    auto &&gravity(this auto &&self,
                   const physkit::vec3<mp_units::si::metre / mp_units::si::second /
                                       mp_units::si::second> &gravity)
    {
        self.M_gravity = gravity;
        return std::forward<decltype(self)>(self);
    }
    auto &&gravity_or(this auto &&self,
                      const physkit::vec3<mp_units::si::metre / mp_units::si::second /
                                          mp_units::si::second> &gravity)
    {
        if (!self.M_gravity) self.M_gravity = gravity;
        return std::forward<decltype(self)>(self);
    }
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
    auto &&time_step(this auto &&self, physkit::quantity<mp_units::si::second> time_step)
    {
        self.M_time_step = time_step;
        return std::forward<decltype(self)>(self);
    }
    auto &&time_step_or(this auto &&self, physkit::quantity<mp_units::si::second> time_step)
    {
        if (!self.M_time_step) self.M_time_step = time_step;
        return std::forward<decltype(self)>(self);
    }
    auto &&record_output(this auto &&self, std::string_view path)
    {
        self.M_record_output = std::string(path);
        return std::forward<decltype(self)>(self);
    }
    auto &&record_duration(this auto &&self, physkit::quantity<mp_units::si::second> duration)
    {
        self.M_record_duration = duration;
        return std::forward<decltype(self)>(self);
    }
    auto &&record_fps(this auto &&self, int fps)
    {
        self.M_record_fps = fps;
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] auto window_size() const { return M_window_size.value_or(default_window_size); }
    [[nodiscard]] auto fov() const { return M_fov.value_or(default_fov); }
    [[nodiscard]] auto cam_pos() const { return M_cam_pos.value_or(default_cam_pos); }
    [[nodiscard]] auto cam_dir() const
    {
        if (M_look_at && M_cam_pos) return (*M_look_at - *M_cam_pos) / mp_units::si::metre;
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
    [[nodiscard]] auto &record_output() const { return M_record_output; }
    [[nodiscard]] auto record_duration() const
    { return M_record_duration.value_or(default_record_duration); }
    [[nodiscard]] auto record_fps() const { return M_record_fps.value_or(default_record_fps); }
    [[nodiscard]] bool recording() const { return M_record_output.has_value(); }
    [[nodiscard]] auto &objects() const { return M_objects; }
    [[nodiscard]] auto world_desc() const
    {
        auto desc = physkit::world_desc::make();
        if (M_gravity) desc.with_gravity(*M_gravity);
        if (M_solver_iterations) desc.with_solver_iterations(*M_solver_iterations);
        return desc;
    }
    [[nodiscard]] auto &args() const { return M_args; }

    [[nodiscard]] auto testing() const { return M_testing; }

    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    g_config(Magnum::Platform::Application::Arguments args, bool read_config = true);

private:
    std::vector<std::pair<physkit::object_desc, Color4>> M_objects;
    Magnum::Platform::Application::Arguments M_args;
    std::optional<Vector2i> M_window_size;
    std::optional<physkit::quantity<mp_units::si::degree, float>> M_fov;
    std::optional<std::string> M_title;
    std::optional<physkit::fvec3<mp_units::si::metre>> M_cam_pos;
    std::optional<physkit::fvec3<mp_units::one>> M_cam_dir;
    std::optional<physkit::fvec3<mp_units::si::metre>> M_look_at;
    std::optional<bool> M_drag = default_drag;
    std::optional<bool> M_vsync = default_vsync;
    std::optional<physkit::vec3<mp_units::si::metre / mp_units::si::second / mp_units::si::second>>
        M_gravity;
    std::optional<std::size_t> M_solver_iterations;
    std::optional<physkit::quantity<mp_units::si::second>> M_time_step;
    std::optional<std::string> M_record_output;
    std::optional<physkit::quantity<mp_units::si::second>> M_record_duration;
    std::optional<int> M_record_fps;
    bool M_testing{false};
};

template <typename... Args> class event_dispatcher
{
public:
    using callback_type = std::function<bool(Args...)>;
    using callback_id = std::size_t;
    static constexpr callback_id null_id = 0;

    callback_id add(callback_type cb)
    {
        const auto id = ++M_next_id;
        M_callbacks.push_back({id, std::move(cb), /*removed=*/false});
        return id;
    }

    bool remove(callback_id id)
    {
        auto it = std::ranges::find(M_callbacks, id, &entry::id);
        if (it == M_callbacks.end() || it->removed) return false;
        it->removed = true;
        if (!M_firing) std::erase_if(M_callbacks, [](const entry &e) { return e.removed; });
        return true;
    }

    void clear()
    {
        if (M_firing)
            for (auto &e : M_callbacks) e.removed = true;
        else
            M_callbacks.clear();
    }

    [[nodiscard]] bool empty() const { return M_callbacks.empty(); }

    void fire(Args... args)
    {
        M_firing = true;
        const auto count = M_callbacks.size();
        for (std::size_t i = 0; i < count; ++i)
        {
            if (!M_callbacks[i].removed)
            {
                if (M_callbacks[i].cb(args...)) M_callbacks[i].removed = true;
            }
        }
        M_firing = false;
        std::erase_if(M_callbacks, [](const entry &e) { return e.removed; });
    }

private:
    struct entry
    {
        callback_id id;
        callback_type cb;
        bool removed;
    };
    std::vector<entry> M_callbacks;
    callback_id M_next_id{null_id};
    bool M_firing{false};
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

        void update()
        {
            if (M_pressed) M_repeated = true;
        }

        friend graphics_app;
    };
    graphics_app(const graphics_app &) = delete;
    graphics_app &operator=(const graphics_app &) = delete;
    graphics_app(graphics_app &&) = delete;
    graphics_app &operator=(graphics_app &&) = delete;

    /// @brief Register and create, or fetch a previously registered shared mesh for this
    /// physkit::mesh instance. This mesh will be used across all methods with this
    /// physkit::mesh.
    /// @param phys_mesh The physkit::mesh to get the shared GL::Mesh for.
    /// @return A shared pointer to the GL::Mesh corresponding to the given physkit::mesh.
    std::shared_ptr<GL::Mesh> get_mesh(const physkit::shape &phys_shape)
    {
        auto shared = &phys_shape;
        if (auto it = M_phys_shape_map.find(shared); it != M_phys_shape_map.end()) return it->second;
        auto mesh = std::make_shared<GL::Mesh>(to_magnum_mesh(phys_shape));
        M_phys_shape_map[std::move(shared)] = mesh;
        return mesh;
    }

    /// @brief Add a physics object to the scene with the specified color. This method will
    /// create or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param world The physics world that owns the object.
    /// @param handle Handle to the rigid body in the world.
    /// @param mesh The shared GL::Mesh to use for rendering.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(physkit::world_base::handle handle, std::shared_ptr<GL::Mesh> mesh,
                    Color4 color)
    {

        return M_world->get_rigid(handle)
            .transform(
                [&](auto obj)
                {
                    if (auto it = M_phys_shape_map.find(obj->shape_ptr());
                        it == M_phys_shape_map.end())
                        M_phys_shape_map[obj->shape_ptr()] = mesh;
                    else if (it->second != mesh)
                        throw std::runtime_error(
                            "graphics_app::add_object: provided mesh does not match "
                            "physkit::object mesh");
                    auto *phys_obj = new physics_obj{M_scene, *this, handle};

                    internal_add_obj(phys_obj, std::move(mesh), color);

                    M_physics_objs.push_back(phys_obj);
                    return phys_obj;
                })
            .value_or(nullptr);
    }

    /// @brief Add a physics object to the scene with the specified color. This method will
    /// create or fetch a shared GL::Mesh for the object's mesh, as if by calling get_mesh().
    /// @param world The physics world that owns the object.
    /// @param handle Handle to the rigid body in the world.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created physics_obj instance.
    auto add_object(physkit::world_base::handle handle, Color4 color)
    {
        auto *phys_obj = new physics_obj{M_scene, *this, handle};
        auto &obj = phys_obj->obj();

        internal_add_obj(phys_obj, get_mesh(obj.shape()), color);

        M_physics_objs.push_back(phys_obj);
        return phys_obj;
    }

    void remove_object(physics_obj *obj, physkit::detail::passkey<physics_obj> /*key*/)
    {
        if (!M_world) return;

        auto _ = M_world->remove_rigid(obj->handle());
        auto it = std::ranges::find(M_physics_objs, obj);
        if (it != M_physics_objs.end())
        {
            std::ranges::iter_swap(it, M_physics_objs.end() - 1);
            M_physics_objs.pop_back();
        }
    }

    /// @brief Add a graphics object to the scene with the specified color and shared mesh.
    /// @param mesh The shared GL::Mesh to use for rendering the object.
    /// @param color The color to use for rendering the object.
    /// @return A pointer to the created gfx_obj instance.
    auto add_object(std::shared_ptr<GL::Mesh> mesh, Color4 color)
    {
        auto *obj = new gfx_obj{&M_scene};
        internal_add_obj(obj, std::move(mesh), color);
        return obj;
    }

    auto &world() { return *M_world; }
    auto &world() const { return std::as_const(*M_world); }

    auto &physics_objects() const { return M_physics_objs; }

    // ---- Input event callbacks ----
    // Register a callback for an input event. Each callback returns true to detach
    // itself after this invocation (one-shot), or false to remain registered. add()
    // returns a non-zero id usable with the matching remove_*() method to detach early.
    // Multiple callbacks may be registered for the same event; they fire in registration
    // order after any subclass override of key_press / pointer_press / pointer_move runs.

    using key_callback = event_dispatcher<KeyEvent &>::callback_type;
    using pointer_callback = event_dispatcher<PointerEvent &>::callback_type;
    using pointer_move_callback = event_dispatcher<PointerMoveEvent &>::callback_type;
    using callback_id = event_dispatcher<>::callback_id;

    callback_id on_key_press(key_callback cb) { return M_on_key_press.add(std::move(cb)); }
    bool remove_key_press(callback_id id) { return M_on_key_press.remove(id); }

    callback_id on_key_release(key_callback cb) { return M_on_key_release.add(std::move(cb)); }
    bool remove_key_release(callback_id id) { return M_on_key_release.remove(id); }

    /// Pointer-press ("click") callback. Fires when any pointer button is pressed.
    callback_id on_click(pointer_callback cb) { return M_on_pointer_press.add(std::move(cb)); }
    bool remove_click(callback_id id) { return M_on_pointer_press.remove(id); }

    callback_id on_release(pointer_callback cb) { return M_on_pointer_release.add(std::move(cb)); }
    bool remove_release(callback_id id) { return M_on_pointer_release.remove(id); }

    callback_id on_move(pointer_move_callback cb) { return M_on_pointer_move.add(std::move(cb)); }
    bool remove_move(callback_id id) { return M_on_pointer_move.remove(id); }

    auto wait_until_mouse_press(Pointer button)
    {
        return physkit::wait_for_event{
            .setup_fn =
                [this, button](auto resume)
            {
                return on_click(
                    [resume = std::move(resume), button](PointerEvent &event)
                    {
                        if (event.pointer() == button)
                        {
                            resume();
                            return true;
                        }
                        return false;
                    });
            },
            .destroy_fn = [this](auto id) { remove_click(id); }};
    }

    auto wait_until_mouse_release(Pointer button)
    {
        return physkit::wait_for_event{
            .setup_fn =
                [this, button](auto resume)
            {
                return on_release(
                    [resume = std::move(resume), button](PointerEvent &event)
                    {
                        if (event.pointer() == button)
                        {
                            resume();
                            return true;
                        }
                        return false;
                    });
            },
            .destroy_fn = [this](auto id) { remove_release(id); }};
    }

    auto wait_until_key_press(Key key)
    {
        return physkit::wait_for_event{.setup_fn =
                                           [this, key](auto resume)
                                       {
                                           return on_key_press(
                                               [resume = std::move(resume), key](KeyEvent &event)
                                               {
                                                   if (event.key() == key)
                                                   {
                                                       resume();
                                                       return true;
                                                   }
                                                   return false;
                                               });
                                       },
                                       .destroy_fn = [this](auto id) { remove_key_press(id); }};
    }

    auto wait_until_key_release(Key key)
    {
        return physkit::wait_for_event{.setup_fn =
                                           [this, key](auto resume)
                                       {
                                           return on_key_release(
                                               [resume = std::move(resume), key](KeyEvent &event)
                                               {
                                                   if (event.key() == key)
                                                   {
                                                       resume();
                                                       return true;
                                                   }
                                                   return false;
                                               });
                                       },
                                       .destroy_fn = [this](auto id) { remove_key_release(id); }};
    }

    physkit::task<physkit::quantity<mp_units::si::second>> next_render_frame()
    {
        mp_units::quantity<mp_units::si::second> time{};
        co_await physkit::wait_for_event{
            .setup_fn =
                [this, &time](auto resume)
            {
                return M_on_next_frame.add(
                    [resume = std::move(resume), &time](mp_units::quantity<mp_units::si::second> dt)
                    {
                        time = dt;
                        resume();
                        return true;
                    });
            },
            .destroy_fn = [this](auto id) { M_on_next_frame.remove(id); }};
        co_return time;
    }

    // physkit::task<physics_obj *> add_rigid(physkit::object_desc obj_desc, Color3 color)
    // { return add_rigid(std::move(obj_desc), Color4{color, 1.0f}); }

    physkit::task<physics_obj *> add_rigid(physkit::object_desc obj_desc, Color4 color)
    {
        auto bh = *co_await physkit::create_rigid(obj_desc);
        co_return add_object(bh, color);
    }

    graphics_app(const g_config &config)
        : Magnum::Platform::Application{
              config.args(),
              [&config]
              {
                  Configuration configuration;
                  configuration.setTitle(
                      Containers::StringView{config.title().data(), config.title().size()});
                  if (config.recording())
                      configuration.setSize(config.window_size(), Vector2{1.0f});
                  else
                      configuration.setSize(config.window_size());
                  configuration.setWindowFlags(
                      config.recording()
                          ? Platform::Application::Configuration::WindowFlag::Hidden
                          : Platform::Application::Configuration::WindowFlag::Focused);
                  return configuration;
              }()},
          M_cam(M_scene, config.fov(), config.cam_pos(), config.cam_dir(), config.window_size(),
                config.window_size()),
          M_drag(true), M_grab_focus(!config.drag()), M_testing(config.testing())
    {
        using namespace Math::Literals::ColorLiterals;

        M_world = std::make_unique<physkit::world>(config.world_desc());

        M_shader = Shaders::PhongGL{Shaders::PhongGL::Configuration{}.setFlags(
            Shaders::PhongGL::Flag::VertexColor | Shaders::PhongGL::Flag::InstancedTransformation)};

        // Ambient/specular alpha must be 0 — PhongGL's fragment shader adds specularColor.a
        // unconditionally on every lit fragment (see Phong.frag, the `fragmentColor += vec4(...,
        // finalSpecularColor.a)` line), which otherwise forces lit pixels of transparent
        // objects to alpha=1 in the shape of the specular lobe.
        M_shader
            .setAmbientColor(Color4{0x222222_rgbf, 0.0f}) // a touch brighter so objects are visible
            .setSpecularColor(Color4{0x330000_rgbf, 0.0f})
            .setLightPositions({{0.f, 5.f, 0.f, 0.f}});

        GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
        GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
        GL::Renderer::enable(GL::Renderer::Feature::Blending);
        GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                       GL::Renderer::BlendFunction::OneMinusSourceAlpha);
        GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add);
        GL::Renderer::setClearColor(0x202020_rgbf);
        M_mouse.reserve(8);
        M_keys.reserve(128);

        M_timeline.start();
        if (config.recording())
            start_recording(config);
        else if (config.vsync())
            set_vsync();

        for (const auto &[obj_desc, color] : config.objects())
            add_object(M_world->create_rigid(obj_desc), color);

        M_stepper = physkit::stepper(*M_world, config.time_step());
    }

    virtual ~graphics_app()
    {
        finish_recording();
        if (M_testing) prompt_results();
    }

protected:
    // called every frame to update the scene, before drawing. stepper::update() is called after
    // this.
    virtual void update(physkit::quantity<mp_units::si::second> dt) = 0;
    virtual void key_press(KeyEvent &event, bool pressed) {}
    virtual void pointer_move(PointerMoveEvent &event) {}
    virtual void pointer_press(PointerEvent &event, bool pressed) {}

    auto &cam() { return M_cam; }
    auto &keys() const { return M_keys; }
    auto &mouse() const { return M_mouse; } // map to pointer states
    auto &mouse_pos() const { return M_mouse_pos; }

    bool drag() const { return M_drag; }
    void drag(bool d)
    {
        if (M_drag == d) return;
        if (d)
        {
            setCursor(Cursor::Arrow);
            warpCursor(windowSize() / 2);
            M_drag = true;
        }
        else if (glfwGetWindowAttrib(window(), GLFW_HOVERED) == GLFW_TRUE)
        {
            M_drag = false;
            setCursor(Cursor::HiddenLocked);
        }
        else
            M_grab_focus = true;
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
    void frame_limit(physkit::quantity<mp_units::si::hertz> fps)
    {
        if (fps == 0 * mp_units::si::hertz)
            M_frame_limit = 0 * mp_units::si::second;
        else
            M_frame_limit = 1 / fps;
    }

    physkit::quantity<mp_units::si::hertz> frame_limit() const
    {
        return M_frame_limit != 0 * mp_units::si::second ? 1 / M_frame_limit
                                                         : 0 * mp_units::si::hertz;
    }

    physkit::quantity<mp_units::si::second> dt() const
    { return M_timeline.previousFrameDuration() * mp_units::si::second; }

    physkit::quantity<mp_units::si::second> current_time() const
    { return M_timeline.previousFrameTime() * mp_units::si::second; }

    key_state get_mouse_button(Pointer button) const
    {
        auto it = M_mouse.find(button);
        return it != M_mouse.end() ? it->second : key_state{};
    }

    key_state get_key(Key key) const
    {
        auto it = M_keys.find(key);
        return it != M_keys.end() ? it->second : key_state{};
    }

private:
    static std::string shell_quote(std::string_view value)
    {
        std::string out;
        out.reserve(value.size() + 2);
        out.push_back('\'');
        for (char c : value)
        {
            if (c == '\'')
                out += "'\\''";
            else
                out.push_back(c);
        }
        out.push_back('\'');
        return out;
    }

    void start_recording(const g_config &config)
    {
        M_recording = true;
        M_record_output = *config.record_output();
        M_record_fps = config.record_fps();
        M_record_size = config.window_size();
        M_record_dt = (1.0f / static_cast<float>(M_record_fps)) * mp_units::si::second;
        M_record_time = 0.0f * mp_units::si::second;
        M_record_frame = 0;

        const auto duration = config.record_duration();
        M_record_frame_count = std::max<std::size_t>(
            1,
            static_cast<std::size_t>(std::ceil(duration.numerical_value_in(mp_units::si::second) *
                                               static_cast<double>(M_record_fps))));

        set_vsync(false);
        M_cam.cam().setViewport(M_record_size);

        const auto command = std::format(
            "ffmpeg -y -loglevel error -f rawvideo -pixel_format rgba -video_size {}x{} "
            "-framerate {} -i - -vf vflip -an -c:v libx264 -pix_fmt yuv420p {}",
            M_record_size.x(), M_record_size.y(), M_record_fps, shell_quote(M_record_output));

        M_record_pipe = popen(command.c_str(), "w");
        if (M_record_pipe == nullptr)
            throw std::runtime_error(
                std::format("failed to start ffmpeg for recording: {}", M_record_output));
    }

    void finish_recording()
    {
        if (M_record_pipe == nullptr) return;
        pclose(M_record_pipe);
        M_record_pipe = nullptr;
    }

    bool capture_record_frame()
    {
        if (M_record_pipe == nullptr) return true;

        Image2D image = GL::defaultFramebuffer.read({{}, M_record_size}, {PixelFormat::RGBA8Unorm});
        const auto data = image.data();
        const auto written = std::fwrite(data.data(), 1, data.size(), M_record_pipe);
        if (written != data.size())
            throw std::runtime_error(
                std::format("failed while writing recorded frame to {}", M_record_output));

        ++M_record_frame;
        M_record_time += M_record_dt;
        return M_record_frame >= M_record_frame_count;
    }

    void internal_add_obj(gfx_obj *obj, std::shared_ptr<GL::Mesh> mesh, Color4 color)
    {
        instanced_drawables *instances{};
        if (auto it = M_mesh_drawables.find(mesh); it == M_mesh_drawables.end())
        {
            auto copy = mesh;
            instances = M_mesh_drawables[std::move(copy)] = new instanced_drawables{
                M_scene, &M_shaded, &M_transparent, std::move(mesh), M_shader};
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

        const auto frame_dt = M_recording ? M_record_dt : dt();
        const auto frame_time = M_recording ? M_record_time : current_time();

        M_cam.move(forward, right, up, frame_dt);

        M_on_next_frame.fire(frame_dt);

        update(frame_dt);
        M_stepper.update(frame_dt);

        for (auto *obj : M_physics_objs) obj->sync();

        M_shader.setProjectionMatrix(M_cam.projection_matrix());
        M_cam.draw(M_shaded, frame_time);
        M_cam.draw(M_transparent, frame_time);

        if (M_recording && capture_record_frame())
        {
            finish_recording();
            exit(0);
            return;
        }

        swapBuffers();
        redraw();

        for (auto &[_, st] : M_keys) st.update();
        for (auto &[_, st] : M_mouse) st.update();

        if (!M_recording)
            while (M_timeline.currentFrameDuration() * mp_units::si::second < M_frame_limit)
                std::this_thread::yield();
        M_timeline.nextFrame();
    }

    void keyPressEvent(KeyEvent &event) final
    {
        M_keys[event.key()].press();
        key_press(event, true);
        M_on_key_press.fire(event);
        if (event.key() == Key::Esc) drag(!M_drag);
    }

    void keyReleaseEvent(KeyEvent &event) final
    {
        M_keys[event.key()].release();
        key_press(event, false);
        M_on_key_release.fire(event);
    }

    void pointerPressEvent(PointerEvent &event) final
    {
        if (M_drag && M_grab_focus)
        {
            M_grab_focus = false;
            drag(false);
        }
        else
        {
            M_mouse[event.pointer()].press();
            pointer_press(event, true);
            M_on_pointer_press.fire(event);
        }
    }

    void pointerReleaseEvent(PointerEvent &event) final
    {
        M_mouse[event.pointer()].release();
        pointer_press(event, false);
        M_on_pointer_release.fire(event);
    }

    void pointerMoveEvent(PointerMoveEvent &event) final
    {
        if (!M_drag ||
            M_mouse[Pointer::MouseLeft].is_pressed()) // may fire one frame late, not a big deal
            M_cam.pointer_move(event, M_drag);
        M_mouse_pos = event.position();
        pointer_move(event);
        M_on_pointer_move.fire(event);
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
    SceneGraph::DrawableGroup3D M_shaded;      // opaque pass
    SceneGraph::DrawableGroup3D M_transparent; // α<1 pass, drawn after M_shaded
    std::unique_ptr<physkit::world_base> M_world;
    physkit::stepper M_stepper;
    camera M_cam;
    std::unordered_map<const physkit::shape *, std::shared_ptr<GL::Mesh>>
        M_phys_shape_map;
    std::unordered_map<std::shared_ptr<GL::Mesh>, instanced_drawables *> M_mesh_drawables;
    std::vector<physics_obj *> M_physics_objs;
    std::unordered_map<Key, key_state> M_keys;
    std::unordered_map<Pointer, key_state> M_mouse;
    Vector2 M_mouse_pos;
    event_dispatcher<KeyEvent &> M_on_key_press;
    event_dispatcher<KeyEvent &> M_on_key_release;
    event_dispatcher<PointerEvent &> M_on_pointer_press;
    event_dispatcher<PointerEvent &> M_on_pointer_release;
    event_dispatcher<PointerMoveEvent &> M_on_pointer_move;
    event_dispatcher<physkit::quantity<mp_units::si::second>> M_on_next_frame;
    Timeline M_timeline;
    physkit::quantity<mp_units::si::second> M_frame_limit{};
    std::FILE *M_record_pipe{};
    std::string M_record_output;
    Vector2i M_record_size{1, 1};
    physkit::quantity<mp_units::si::second> M_record_dt = 0.0f * mp_units::si::second;
    physkit::quantity<mp_units::si::second> M_record_time = 0.0f * mp_units::si::second;
    std::size_t M_record_frame{};
    std::size_t M_record_frame_count{};
    int M_record_fps{};
    bool M_recording = false;
    bool M_debug = false;
    bool M_drag = false;
    bool M_grab_focus = false;
    bool M_testing = false;
};

inline physkit::object &physics_obj::obj() { return **M_app->world().get_rigid(M_handle); }
inline const physkit::object &physics_obj::obj() const
{ return **std::as_const(*M_app).world().get_rigid(M_handle); }

inline void physics_obj::sync()
{
    if (auto res = M_app->world().get_rigid(M_handle))
        resetTransformation()
            .rotate(to_magnum_quaternion<mp_units::one, float>(res.value()->orientation()))
            .translate(to_magnum_vector<mp_units::si::metre, float>(res.value()->pos()));
}

inline physics_obj::~physics_obj() { M_app->remove_object(this, {}); }

namespace mesh_objs
{
// radius 1
inline auto cube()
{ return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cubeSolid())); }

// radius 1
inline auto sphere(unsigned int subdivisions = 3)
{ return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::icosphereSolid(subdivisions))); }

    inline auto cylinder(unsigned int rings, unsigned int segments, float half_length,
                         bool include_caps = true)
    {
        return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::cylinderSolid(
            rings, segments, half_length,
            include_caps ? Primitives::CylinderFlag::CapEnds : Primitives::CylinderFlag{})));
    }

    inline auto cone(unsigned int rings, unsigned int segments, float half_length)
    {
        return std::make_shared<GL::Mesh>(
            MeshTools::compile(Primitives::coneSolid(rings, segments, half_length)));
    }

    inline auto plane()
    { return std::make_shared<GL::Mesh>(MeshTools::compile(Primitives::planeSolid())); }
    }; // namespace mesh_objs
} // namespace graphics
