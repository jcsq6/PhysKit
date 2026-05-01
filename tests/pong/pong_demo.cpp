#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <coroutine> // IWYU pragma: keep
#include <format>
#include <numbers>
#include <optional>
#include <random>
#endif

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#else
#include <physkit/physkit.h>
#endif

#ifdef PHYSKIT_GRAPHICS_MODULES
import graphics;
#else
#include <graphics/graphics.h>
#endif

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace graphics;

class pong_game : public graphics_app
{
    static inline const auto gravity = vec3{0.0, 0.0, 0.0} * m / s / s;
    static constexpr auto ball_radius = 0.04 * m;
    static constexpr auto paddle_width = 0.015 * m;
    static constexpr auto paddle_height = 0.25 * m;
    static constexpr auto paddle_depth = 0.08 * m;
    static constexpr auto play_width = 1.8 * m;
    static constexpr auto play_height = 1.2 * m;

    // Scoring boundaries:
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
        cam().speed(2.0f * m / s);
        world().add_task(scene());
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override
    {
        static auto last_score_update = 0.0 * s;
        last_score_update += dt;

        if (last_score_update > 0.1 * s)
        {
            last_score_update = 0.0 * s;
            setWindowTitle(std::format("Physkit Pong - Player: {} AI: {}", player_score, ai_score));
        }
    }

private:
    int player_score = 0;
    int ai_score = 0;
    std::mt19937 random_engine{std::random_device{}()};
    std::uniform_real_distribution<float> initial_angle_distribution{-30.0f, 30.0f};
    std::uniform_real_distribution<float> reset_angle_distribution{-25.0f, 25.0f};
    std::bernoulli_distribution direction_distribution{0.5};

    task<> make_wall(vec3<si::metre> pos, vec3<si::metre> half_extents, Color3 color,
                     float restitution = 1.0f)
    {
        co_await add_rigid(object_desc::stat()
                               .with_shape(box(half_extents))
                               .with_pos(pos)
                               .with_restitution(restitution)
                               .with_friction(0.0),
                           color);
    }

    task<> scene()
    {
        // Top Wall
        co_await make_wall(vec3{0.0 * m, play_height / 2 + 0.05 * m, 0.0 * m},
                           vec3{play_width / 2, 0.03 * m, 0.2 * m}, Color3{0.0f, 0.3f, 0.3f});

        // Bottom Wall
        co_await make_wall(vec3{0.0 * m, -play_height / 2 - 0.05 * m, 0.0 * m},
                           vec3{play_width / 2, 0.03 * m, 0.2 * m}, Color3{0.0f, 0.3f, 0.3f});

        // Back Wall
        co_await make_wall(vec3{0.0 * m, 0.0 * m, -0.3 * m},
                           vec3{play_width / 2, play_height / 2, 0.02 * m},
                           Color3{0.2f, 0.2f, 0.25f}, 0.5f);

        // Front Wall
        co_await make_wall(vec3{0.0 * m, 0.0 * m, 0.3 * m},
                           vec3{play_width / 2, play_height / 2, 0.01 * m},
                           Color3{0.15f, 0.15f, 0.2f}, 0.5f);

        // Center Line
        auto line_segment = box(vec3{0.01 * m, 0.05 * m, 0.01 * m});
        for (int i = -5; i <= 5; ++i)
        {
            co_await add_rigid(object_desc::stat()
                                   .with_shape(line_segment)
                                   .with_pos(vec3{0.0 * m, i * 0.12 * m, 0.0 * m})
                                   .with_restitution(1.0),
                               Color3{0.5f, 0.5f, 0.5f});
        }

        // Left Paddle - Player
        auto left_paddle_obj = co_await add_rigid(
            object_desc::dynam()
                .with_shape(box(vec3{paddle_width / 2, paddle_height / 2, paddle_depth / 2}))
                .with_pos(vec3{-play_width / 2 + 0.08 * m, 0.0 * m, 0.0 * m})
                .with_mass(2.0 * kg)
                .with_restitution(1.2)
                .with_friction(0.0),
            Color3{0.2f, 0.6f, 0.9f});

        // (*left_paddle_obj)->is_kinematic(true);

        // Right Paddle - AI
        auto right_paddle_obj = co_await add_rigid(
            object_desc::dynam()
                .with_shape(box(vec3{paddle_width / 2, paddle_height / 2, paddle_depth / 2}))
                .with_pos(vec3{play_width / 2 - 0.08 * m, 0.0 * m, 0.0 * m})
                .with_mass(2.0 * kg)
                .with_restitution(1.2)
                .with_friction(0.0),
            Color3{0.2f, 0.6f, 0.9f});
        // (*right_paddle_obj)->is_kinematic(true);

        auto ball_obj = co_await add_rigid(object_desc::dynam()
                                               .with_shape(sphere(ball_radius))
                                               .with_pos(vec3{0.0 * m, 0.0 * m, 0.0 * m})
                                               .with_mass(0.1 * kg)
                                               .with_restitution(1.0)
                                               .with_friction(0.0),
                                           Color3{1.0f, 1.0f, 0.2f});

        co_await add_task<policy::no_wait>(game_loop(
            (*ball_obj)->handle(), (*left_paddle_obj)->handle(), (*right_paddle_obj)->handle()));
    }

    task<> game_loop(physkit::world_base::handle ball_handle,
                     physkit::world_base::handle left_paddle_handle,
                     physkit::world_base::handle right_paddle_handle)
    {
        auto ball = co_await get_rigid(ball_handle);
        if (!ball) { co_return; }

        auto angle = initial_angle_distribution(random_engine) * std::numbers::pi_v<float> / 180.0f;
        auto speed = 3.5f * m / s;
        auto dir_x = direction_distribution(random_engine) ? 1.0f : -1.0f;

        (*ball)->vel() = vec3{dir_x * cos(angle), sin(angle), 0.0f} * speed;

        co_await add_task<policy::no_wait>(
            [&, this](this auto self) -> task<>
            {
                while (true)
                {
                    co_await next_frame{};
                    auto ball_opt = co_await get_rigid(ball_handle);
                    if (!ball_opt) { break; }

                    auto &ball = **ball_opt;

                    if (ball.vel().norm() < 0.5 * m / s && (abs(ball.pos().x()) < play_width / 3))
                    {
                        auto vel = ball.vel();
                        vel.x() = (vel.x() >= 0.0f * m / s ? 1.0f : -1.0f) * 1.0f * m / s;
                        vel.y() = (vel.y() >= 0.0f * m / s ? 0.5f : -0.5f) * m / s;
                        ball.vel() = vel;
                    }

                    auto vel = ball.vel();
                    if (abs(vel.y()) > 6.0f * m / s)
                    {
                        vel.y() = (vel.y() > 0.0f * m / s ? 1.0f : -1.0f) * 5.0f * m / s;
                        ball.vel() = vel;
                    }
                }
            }());

        while (true)
        {
            auto dt = *co_await next_render_frame();

            auto ball_opt = co_await get_rigid(ball_handle);
            auto left_opt = co_await get_rigid(left_paddle_handle);
            auto right_opt = co_await get_rigid(right_paddle_handle);

            if (!ball_opt || !left_opt || !right_opt) { break; }

            auto &ball = **ball_opt;
            auto &left_paddle = **left_opt;
            auto &right_paddle = **right_opt;

            auto player_speed = 6.0f * m / s;
            auto paddle_vel = vec3{0.0, 0.0, 0.0} * m / s;

            if (get_key(Key::W).is_pressed() || get_key(Key::Up).is_pressed())
            {
                paddle_vel.y(player_speed);
            }
            else if (get_key(Key::S).is_pressed() || get_key(Key::Down).is_pressed())
            {
                paddle_vel.y(-player_speed);
            }

            auto new_y = left_paddle.pos().y() + paddle_vel.y() * dt;
            new_y = std::clamp(new_y, -play_height / 2 + paddle_height / 2,
                               play_height / 2 - paddle_height / 2);
            left_paddle.pos() = vec3{left_paddle.pos().x(), new_y, left_paddle.pos().z()};

            auto ai_speed = 5.0f * m / s;
            auto ai_vel = vec3{0.0, 0.0, 0.0} * m / s;

            auto error_margin = 0.03f * m;
            if (ball.pos().y() > right_paddle.pos().y() + error_margin) { ai_vel.y() = ai_speed; }
            else if (ball.pos().y() < right_paddle.pos().y() - error_margin)
            {
                ai_vel.y() = -ai_speed;
            }

            auto ai_new_y = right_paddle.pos().y() + ai_vel.y() * dt;
            ai_new_y = std::clamp(ai_new_y, -play_height / 2 + paddle_height / 2,
                                  play_height / 2 - paddle_height / 2);
            right_paddle.pos() = vec3{right_paddle.pos().x(), ai_new_y, right_paddle.pos().z()};

            if (ball.pos().x() < left_score_x)
            {
                ai_score++;
                co_await reset_ball(true, ball_handle);
            }
            else if (ball.pos().x() > right_score_x)
            {
                player_score++;
                co_await reset_ball(false, ball_handle);
            }
        }
    }

    task<> reset_ball(bool serve_to_player, physkit::world_base::handle ball_handle)
    {
        auto ball_opt = co_await get_rigid(ball_handle);
        if (!ball_opt) { co_return; }

        auto &ball = **ball_opt;

        ball.pos() = vec3{0.0 * m, 0.0 * m, 0.0 * m};

        co_await wait_for(1.5 * s);

        float angle = reset_angle_distribution(random_engine) * std::numbers::pi_v<float> / 180.0f;
        auto speed = 3.0f * m / s;

        float dir_x = serve_to_player ? -1.0f : 1.0f;

        ball.vel() = vec3{dir_x * cos(angle), sin(angle), 0.0f} * speed;
        ball.ang_vel() = vec3<one / si::second>::zero();
    }
};

MAGNUM_APPLICATION_MAIN(pong_game)
