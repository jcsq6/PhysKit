#ifndef GRAPHICS_IN_MODULE_IMPL
#include "graphics/graphics.h"

#include "dep/argparse.hpp"
#include <glaze/glaze.hpp>
#endif

template <> struct glz::meta<physkit::body_type>
{
    using enum physkit::body_type;
    static constexpr auto value = enumerate(physkit::body_type::stat, physkit::body_type::dynam);
};

namespace graphics
{
template g_config &g_config::read_file<g_config &>(g_config &, std::string_view);
template g_config &&g_config::read_file<g_config &&>(g_config &&, std::string_view);

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
template <typename Self> Self &&g_config::read_file(this Self &&self, std::string_view path)
{
    using namespace mp_units::si::unit_symbols;
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
    };

    struct cylinder_type // NOLINT(cppcoreguidelines-pro-type-member-init)
    {
        std::string name; // must be "cylinder"
        float radius;
        float height;
    };

    struct cone_type // NOLINT(cppcoreguidelines-pro-type-member-init)
    {
        std::string name; // must be "cone"
        float radius;
        float height;
    };

    struct obj // NOLINT(cppcoreguidelines-pro-type-member-init)
    {
        std::array<double, 3> pos;
        std::array<double, 3> vel = {0.0, 0.0, 0.0};
        std::array<double, 4> orientation = {1.0, 0.0, 0.0, 0.0}; // [w, x, y, z]
        std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0}; // rad/s
        std::array<double, 3> inertia_tensor = {1.0, 1.0, 1.0};   // diagonal [Ixx, Iyy, Izz]
        physkit::body_type type = physkit::body_type::dynam;
        std::variant<box_type, pyramid_type, sphere_type, cone_type> shape =
            box_type{"box", {0.5f, 0.5f, 0.5f}};
        double mass;
        double restitution = 0.5;
        double friction = 0.5;
        std::array<float, 4> color = {1.0, 1.0, 1.0, 1.0};
    };

    struct world_config
    {
        std::array<double, 3> gravity = {0.0, -9.81, 0.0};
        // physkit::world_desc::integ_t integrator =
        // physkit::world_desc::semi_implicit_euler;
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
        std::println(std::cerr, "Error reading JSON config:\n{}", glz::format_error(ez, buffer));
        std::cerr.flush();
        return std::forward<decltype(self)>(self);
    }

    self.M_gravity =
        physkit::vec3{config.gravity[0], config.gravity[1], config.gravity[2]} * m / s / s;
    // self.M_integrator = config.integrator;
    self.M_solver_iterations = config.solver_iterations;

    std::map<std::string, physkit::shape> shape_map;
    self.M_objects.reserve(config.objects.size());
    for (const obj &obj : config.objects)
    {
        auto shape_name = *glz::write_json(obj.shape);
        auto it = shape_map.find(shape_name);
        if (it == shape_map.end())
        {
            obj.shape.visit(
                [&](auto &&shape_desc)
                {
                    using T = std::decay_t<decltype(shape_desc)>;
                    if constexpr (std::same_as<T, box_type>)
                        shape_map[shape_name] = physkit::box{
                            physkit::vec3{shape_desc.half_extents[0], shape_desc.half_extents[1],
                                          shape_desc.half_extents[2]} *
                            m};
                    else if constexpr (std::same_as<T, pyramid_type>)
                        shape_map[shape_name] =
                            physkit::pyramid(shape_desc.base_size * m, shape_desc.height * m);
                            //physkit::mesh::pyramid(shape_desc.height * m, shape_desc.base_size * m);
                    else if constexpr (std::same_as<T, sphere_type>)
                        shape_map[shape_name] = physkit::sphere{shape_desc.radius * m};
                    else if constexpr (std::same_as<T, cone_type>)
                        shape_map[shape_name] =
                            physkit::cone{shape_desc.radius * m, shape_desc.height * m};
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
                .with_orientation(physkit::quat<mp_units::one>{
                    obj.orientation[0], obj.orientation[1], obj.orientation[2], obj.orientation[3]})
                .with_ang_vel(physkit::vec3{obj.angular_velocity[0], obj.angular_velocity[1],
                                            obj.angular_velocity[2]} *
                              rad / s)
                .with_inertia_tensor(inertia)
                .with_mass(obj.mass * kg)
                .with_shape(shape_map.at(shape_name))
                .with_restitution(obj.restitution)
                .with_friction(obj.friction),
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
    //              glz::reflect<physkit::world_desc::integ_t>::keys[static_cast<int>( //
    //              NOLINT
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
        std::println("      Type: {}",
                     glz::reflect<physkit::body_type>::keys[static_cast<int>(o.type())]); // NOLINT
        std::println("      Color: ({}, {}, {})", color[0], color[1], color[2]);
        config.objects[i].shape.visit(
            [](auto &&shape_desc)
            {
                using T = std::decay_t<decltype(shape_desc)>;
                if constexpr (std::same_as<T, box_type>)
                    std::println("      Shape: box (half_extents: {})",
                                 physkit::vec3{shape_desc.half_extents[0],
                                               shape_desc.half_extents[1],
                                               shape_desc.half_extents[2]} *
                                     m);
                else if constexpr (std::same_as<T, pyramid_type>)
                    std::println("      Shape: pyramid (height: {}, base_size: {})",
                                 shape_desc.height * m, shape_desc.base_size * m);
                else if constexpr (std::same_as<T, sphere_type>)
                    std::println("      Mesh: sphere (radius: {})", // TODO
                                 shape_desc.radius * m);
                else if constexpr (std::same_as<T, cone_type>)
                    std::println("      Shape: cone (radius: {}, height: {})",
                                 shape_desc.radius * m, shape_desc.height * m);
            });
    }
    std::cout.flush();

    return std::forward<decltype(self)>(self);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
g_config::g_config(Magnum::Platform::Application::Arguments args, bool read_config) : M_args(args)
{
    using namespace mp_units::si::unit_symbols;
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
                std::println(std::cerr, "Error parsing vector from string \"{}\": {}", s, e.what());
                std::exit(1);
            }
        }
        if (i != size)
        {
            std::println(std::cerr,
                         "Error parsing vector from string \"{}\": expected {} components "
                         "but got {}",
                         s, size, i);
            std::exit(1);
        }
        return result;
    };

    try
    {
        auto fov_val = parser.get<double>("--fov") * deg;
        auto cam_pos_val =
            read_vec(parser.get<std::string>("--cam-pos"), std::integral_constant<int, 3>{}) * m;
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
} // namespace graphics