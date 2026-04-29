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





}