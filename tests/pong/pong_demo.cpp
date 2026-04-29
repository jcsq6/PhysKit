#ifdef PHYSKIT_GRAPHICS_MODULES
import graphics;
#else
#inlcude <graphics/graphics.h>
#endif

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#else
#include <physkit/physkit.h>
#endif

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace graphics;

class pong_game : public graphics_app {
    static inline const auto gravity = vec3{0.0, 0.0, 0.0} * m / s / s;
    static constexpr auto ball_radius = 0.04 * m;
    static constexpr auto paddle_width = 0.015 * m;
    static constexpr auto paddle_height = 0.25 * m;
    static constexpr auto paddle_depth = 0.08 * m;
    static constexpr auto play_width = 1.8 * m;
    static constexpr auto play_height = 1.2 * m;

    // Scoring boundires:
    static constexpr auto left_score_x = -play_width / 2 - 0.1 * m;
    static constexpr auto right_score_x = play_width / 2 + 0.1 * m;

    public:
        explicit pong_game(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                            .title("Physkit - Pong Game Demo")
                            .window_size({1280, 720})
                            .cam_pos(fvec3{0.0f, 0.0f, -2.5f} * si::metre)
                            .look_at(fvec3{0.0f, 0.0f, 0.0f} * si::metre)
                            .drag(false)
                            .gravity(gravity)
                            .time_step(1.0 / 240.0 * si::second)
                            .solver_iterations(30)}
        
        {
            cam().speed(2.0f * si::metre / si::second);
            world().add_task(scene());
        }

        void update(mp_units::qunatity<mp_units::si::second> dt) override {
            static auto last_score_update = 0.0 * s;
            last_score_update += dt;

            if (last_score_update > 0.1 * s) {
                last_score_update = 0.0 * s;

                std::string title = "Physkit Pong - Player: " + std::to_string(player_score) + " AI: " + std::to_string(ai_score);
                set_window_title(title.c_str());
            }

        }

    private:
        int player_score = 0;
        int ai_score = 0;
        world_base::handle ball_handle;
        world_base::handle left_paddle_handle;
        world_base::handle right_paddle_handle;

        task<> make_wall(vec3<si::metre> pos, vec3<si::metre> half_extents, Color3 color, float restitution = 1.0f) {
            co_await add_rigid(object_desc::stat()
                                    .with_shape(box(half_extents))
                                    .with_pos(pos)
                                    .with_restitution(restitution)
                                    .with_friction(0.0), color);
        }

        task<> scene {
            // Top Wall
            co_await make_wall(
                vec3{0.0 * m, play_height / 2 + 0.05 * m, 0.0 * m},
                vec3{play_width / 2, 0.03 * m, 0.2 * m},
                Color3{0.0f, 0.3f, 0.3f}
            );

            // Bottom Wall
            co_await make_wall(
                vec3{0.0 * m, -play_height / 2 - 0.05 * m, 0.0 * m},
                vec3{play_width / 2, 0.03 * m, 0.2 * m},
                Color3{0.0f, 0.3f, 0.3f}
            );

            // Back Wall
            co_await make_wall(
                vec3{0.0 * m, 0.0 m, -0.3 * m},
                vec3{play_width / 2, play_height / 2, 0.02 * m},
                Color3{0.2f, 0.2f, 0.25f},
                0.5f
            );

            // Front Wall
            co_await make_wall(
                vec3{0.0 * m, 0.0 m, 0.3 * m},
                vec3{play_width / 2, play_height / 2, 0.01 * m},
                Color3{0.15f, 0.15f, 0.2f},
                0.5f
            );

            // Center Line
            auto line_segment = box(vec3{0.01 * m, 0.05 * m, 0.01 * m});
            for (int i = -5; i <= 5; ++i) {
                co_await add_rigid(object_desc::stat()
                                        .with_shape(line_segment)
                                        .with_pos(vec3{0.0 * m, i * 0.12 * m, 0.0 * m})
                                        .with_restitution(1.0), Color3{0.5f, 0.5f, 0.5f});
            }

            // Left Paddle
            auto left_paddle_obj = co_await add_rigid(
                object_desc::dynam()
                    .with_shape(vec3{paddle_width, paddle_height, paddle_depth})
                    .with_pos(vec3{-play_width / 2 + 0.08 * m, 0.0 * m, 0.0 * m})
                    .with_mass(2.0 * kg)
                    .with_restitution(1.2)
                    .with_friction(0.0), Color3{0.2f, 0.6f, 0.9f}
            );
            left_paddle_handle = (*left_paddle_obj)->handle();

            (**left_paddle_obj)

        }



}