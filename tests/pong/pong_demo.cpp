#ifdef PHYSKIT_GRAPHICS_MODULES
import graphics;
#else
#include <graphics/graphics.h>
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

            // Left Paddle - Player
            auto left_paddle_obj = co_await add_rigid(
                object_desc::dynam()
                    .with_shape(vec3{paddle_width, paddle_height, paddle_depth})
                    .with_pos(vec3{-play_width / 2 + 0.08 * m, 0.0 * m, 0.0 * m})
                    .with_mass(2.0 * kg)
                    .with_restitution(1.2)
                    .with_friction(0.0), Color3{0.2f, 0.6f, 0.9f}
            );
            left_paddle_handle = (*left_paddle_obj)->handle();

            (**left_paddle_obj)->is_kinematic(true);

            // Right Paddle - AI
            auto right_paddle_obj = co_await add_rigid(
                object_desc::dynam()
                    .with_shape(vec3{paddle_width, paddle_height, paddle_depth})
                    .with_pos(vec3{play_width / 2 - 0.08 * m, 0.0 * m, 0.0 * m})
                    .with_mass(2.0 * kg)
                    .with_restitution(1.2)
                    .with_friction(0.0), Color3{0.2f, 0.6f, 0.9f}
            );
            right_paddle_handle = (*right_paddle_obj)->handle();

            (**right_paddle_obj)->is_kinematic(true);

            auto ball_obj = co_await add_rigid(
                object_desc::dynam()
                    .with_shape(sphere(ball_radius))
                    .with_pos(vec3{0.0 * m, 0.0 * m, 0.0 * m})
                    .with_mass(0.1 * kg)
                    .with_restitution(1.0)
                    .with_friction(0.0), Color3{1.0f, 1.0f, 0.2f}
            );
            ball_handle = (*ball_obj)->handle();

            co_await add_task<policy::no_wait>(game_loop());
        }

        task<> game_loop()
        {
            auto ball = co_await get_rigid(ball_handle);
            if (!ball) {
                co_return;
            }

            float angle = (rand() % 60 - 30) * 3.14159f / 180.0f;
            float speed = 3.5f * m / s;
            float dir_x = (rand() % 2 == 0) ? 1.0f : -1.0f;

            (**ball)->vel() = vec3{dir_x * speed * cos(angle), speed * sin(angle), 0.0f} * m / s;

            co_await add_task<policy::no_wait>(
                [this](this auto self) -> task<>
                {
                    while (true) {
                        co_await next_frame{};
                        auto ball_opt = co_await get_rigid(ball_handle);
                        if (!ball_opt) {
                            break;
                        }

                        auto &ball = **ball_opt;

                        if (ball.vel().norm() < 0.5 * m / s && (abs(ball.pos().x()) < play_width / 3)) {
                            auto vel = ball.vel();
                            vel.x() = (vel.x() >= 0.0f ? 1.0f : -1.0f) * 1.0f * m / s;
                            vel.y() = (vel.y() >= 0.0f ? 0.5f : -0.5f) * m / s;
                            ball.vel() = vel;
                        }

                        auto vel = ball.vel();
                        if (abs(vel.y()) > 6.0f * m / s) {
                            vel.y() = (vel.y() > 0 ? 1.0f : -1.0f) * 5.0f * m / s;
                            ball.vel() = vel;
                        }

                    }


                }());

            while (true) {
                auto dt = co_await next_render_frame();
            
                auto ball_opt = co_await get_rigid(ball_handle);
                auto left_opt = co_await get_rigid(left_paddle_handle);
                auto right_opt = co_await get_rigid(right_paddle_handle);

                if (!ball_opt || !left_opt || !right_opt) {
                    break;
                }

                auto &ball = **ball_opt;
                auto &left_paddle = **left_opt;
                auto &right_paddle = **right_opt;

                float player_speed = 6.0f * m / s;
                vec3<si::metre / si::second> paddle_vel{0.0 * m / s, 0.0 * m / s, 0.0 * m / s};

                if (is_key_pressed(Key::W) || is_key_pressed(Key::Up)) {
                    paddle_vel.y() = player_speed;
                } else if (is_key_pressed(Key::S) || is_key_pressed(Key::Down)) {
                    paddle_vel.y() = -player_speed;
                }

                auto new_y = left_paddle.pos().y() + paddle_vel.y() * dt;
                new_y = std::clamp(new_y, -player_height / 2 + paddle_height / 2, play_height / 2 - paddle_height / 2);
                left_paddle.pos() = vec3{left_paddle.pos().x(), new_y, left_paddle.pos().z()};

                float ai_speed = 5.0f * m / s;
                vec3<si::metre, si::second> ai_vel{0.0 * m / s, 0.0 * m / s, 0.0 * m / s};

                float error_margin = 0.03f * m;
                if (ball.pos().y() > right_paddle.pos().y() + error_margin) {
                    ai_vel.y() = ai_speed;
                } else if (ball.pos().y() > right_paddle.pos().y() - error_margin) {
                    ai_vel.y() = -ai_speed;
                }

                auto ai_new_y = right_paddle.pos().y() + ai_vel.y() * dt;
                ai_new_y = std::clamp(ai_new_y, -player_height / 2 + paddle_height / 2, play_height / 2 - paddle_height / 2);
                right_paddle.pos() = vec3{right_paddle.pos().x(), ai_new_y, right_paddle.pos().z()};

                if (ball.pos().x() < left_score_x) {
                    ai_score++
                    co_await reset_ball(true);
                } else if (ball.pos().x() < right_score_x) {
                    player_score++
                    co_await reset_ball(false);
                }

            }
        }

        task<> reset_ball(bool serve_to_player) {
            auto ball_opt = co_await get_rigid(ball_handle);
            if (!ball_opt) {
                co_return;
            }

            auto &ball = **ball_opt;

            ball.pos() = vec3{0.0 * m, 0.0 * m, 0.0 * m};

            co_await wait_for(1.5 * s); 

            float angle = ((rand() % 50) - 25) * 3.14159f / 180.0f;
            float speed = 3.0f * m / s;

            float dir_x = serve_to_player ? -1.0f : 1.0f

            ball.vel() = vec3{dir_x * speed * cos(angle), speed * sin(angle), 0.0f} * m / s;
            ball.ang_vel() = vec3<one / si::second>::zero();
        }

        void set_window_title(const std::string& title)
        {
            #ifdef __linux__
            std::string cmd = "echo -ne '\\033]0;" + title + "\\007'";
            system(cmd.c_str());
            #endif
        }

};

MAGNUM_APPLICATION_MAIN(pong_game)