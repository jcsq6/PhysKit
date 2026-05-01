#ifndef GRAPHICS_IN_MODULE_IMPL

#include "graphics/debug_overlay.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <ranges>
#include <string_view>
#include <utility>

#include <GLFW/glfw3.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Text/Alignment.h>
#endif

namespace debug
{
namespace
{
using Magnum::Color4;
using Magnum::Matrix3;
using Magnum::Vector2;
using Magnum::Text::Alignment;

constexpr auto margin_x = 24.0f;
constexpr auto margin_y = 24.0f;
constexpr auto section_gap = 14.0f;
constexpr auto title_gap = 6.0f;

const auto header_color = Color4{1.0f, 0.42f, 0.42f, 1.0f};
const auto title_color = Color4{1.0f, 0.78f, 0.30f, 1.0f};
const auto value_color = Color4{0.94f, 0.94f, 0.94f, 1.0f};

constexpr std::string_view glyphs = "abcdefghijklmnopqrstuvwxyz"
                                    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                    "0123456789"
                                    " .,:;!?+-*/=<>()[]{}#_%|";

using speed_quantity = physkit::quantity<physkit::si::metre / physkit::si::second>;

template <auto Unit, typename Rep> double quantity_value(const physkit::quantity<Unit, Rep> &value)
{ return static_cast<double>(value.numerical_value_in(Unit)); }

Magnum::Vector2 projected_window_size(const Matrix3 &projection, Vector2 fallback)
{
    const auto x_scale = projection[0].x();
    const auto y_scale = projection[1].y();
    if (x_scale == 0.0f || y_scale == 0.0f) return fallback;
    return {std::abs(2.0f / x_scale), std::abs(2.0f / y_scale)};
}
} // namespace

overlay::overlay(physkit::world_base &world, const graphics::camera &camera)
    : M_world(world), M_camera(camera)
{ initialize_font(); }

void overlay::initialize_font()
{
    Corrade::Utility::Resource rs("physkit-data");
    const auto candidate = rs.getRaw("SourceSansPro-Regular.ttf");

    auto font = M_font_manager.loadAndInstantiate("StbTrueTypeFont");
    if (!font || !font->openData(candidate, font_raster_size))
        throw std::runtime_error("Failed to open font file");

    auto cache = std::make_unique<Magnum::Text::GlyphCacheGL>(Magnum::PixelFormat::R8Unorm,
                                                              Magnum::Vector2i{2048, 1024});
    if (!font->fillGlyphCache(*cache, {glyphs.data(), glyphs.size()}))
        throw std::runtime_error("Failed to fill glyph cache");

    auto shaper = font->createShaper();
    if (!shaper) throw std::runtime_error("Failed to create font shaper");

    M_font = std::move(font);
    M_shaper = std::move(shaper);
    M_cache = std::move(cache);
    M_text_renderer = std::make_unique<Magnum::Text::RendererGL>(*M_cache);
    M_text_renderer->reserve(256, 1);
}

bool overlay::text_ready() const { return M_font && M_shaper && M_cache && M_text_renderer; }

bool overlay::has_visible_content() const
{ return M_visible || (M_controls_visible && !M_controls.empty()); }

void overlay::update(physkit::quantity<physkit::si::second> dt,
                     physkit::quantity<physkit::si::second> total_time)
{
    update_performance(dt);
    M_total_time = total_time;

    // Drop entries whose objects no longer exist, then refresh remaining fields
    // in place so insertion order (and therefore on-screen order) stays stable
    // across frames.
    std::erase_if(M_object_info, [this](const ObjectDebugInfo &info)
                  { return !M_world.get_rigid(info.handle).has_value(); });

    for (auto &info : M_object_info)
    {
        const auto &object = **M_world.get_rigid(info.handle);
        auto velocity = object.vel();
        info.position = object.pos();
        info.velocity = velocity;
        info.speed = velocity.norm();
        info.mass = object.mass();
        info.is_static = object.is_static();
        info.type = object.is_static() ? "static" : "dynamic";
    }

    for (const auto &[handle, object_ptr] : M_world.rigids_range())
    {
        const bool tracked = std::ranges::any_of(M_object_info, [&](const ObjectDebugInfo &info)
                                                 { return info.handle == handle; });
        if (tracked) continue;

        const auto &object = *object_ptr;
        auto velocity = object.vel();
        M_object_info.push_back({
            .position = object.pos(),
            .velocity = velocity,
            .speed = velocity.norm(),
            .mass = object.mass(),
            .is_static = object.is_static(),
            .type = object.is_static() ? "static" : "dynamic",
            .handle = handle,
        });
    }

    clamp_object_scroll();
}

void overlay::draw(Magnum::Shaders::VectorGL2D &shader, const Matrix3 &projection)
{
    if (!has_visible_content()) return;

    M_window_size = projected_window_size(projection, M_window_size);

    if (!M_visible && M_controls_visible && !M_controls.empty())
    {
        const auto block_height =
            line_height * static_cast<float>(M_controls.size() + 1) + title_gap;
        auto y_controls = std::max(margin_y, M_window_size.y() - margin_y - block_height);
        draw_section(shader, M_controls_title, margin_x, y_controls, M_controls,
                     Alignment::TopLeft);
    }

    if (!M_visible) return;

    draw_text(shader, "PhysKit Debug Overlay", M_window_size.x() * 0.5f, margin_y, header_color,
              Alignment::TopCenter);
    draw_text(shader, "F1 for help", M_window_size.x() * 0.5f, margin_y + line_height, value_color,
              Alignment::TopCenter);

    auto y_left = margin_y + line_height * 2.6f;
    auto y_right = margin_y + line_height * 2.6f;
    const auto right_x = M_window_size.x() - margin_x;

    if (M_show_physics)
    {
        const auto &world = std::as_const(M_world);
        std::vector<std::string> lines{
            std::format("elapsed   {::N[.2f]}", M_total_time),
            std::format("objects   {}", world.object_count()),
            std::format("gravity   {::.2f}", world.gravity()),
        };

        if (M_show_collision) lines.push_back(std::format("contacts  {}", M_total_collisions));

        draw_section(shader, "Physics", margin_x, y_left, lines, Alignment::TopLeft);
    }

    if (M_show_objects && !M_object_info.empty())
    {
        if (M_show_detail_objects)
        {
            // Each object entry uses 4 lines; reserve space for the title, the
            // "showing" header, the scroll hint, and the trailing section gap so
            // we can size the visible window to the actual viewport.
            constexpr auto rows_per_entry = 4.0f;
            const auto reserved =
                (line_height + title_gap) + line_height + line_height + section_gap;
            const auto available = M_window_size.y() - margin_y - y_left - reserved;
            const auto fit_rows =
                available > 0.0f
                    ? static_cast<std::size_t>(available / (line_height * rows_per_entry))
                    : std::size_t{0};
            const auto rows = std::min<std::size_t>(fit_rows, max_detail_rows);

            const auto max_start =
                M_object_info.size() > rows ? M_object_info.size() - rows : std::size_t{0};
            const auto start = std::min(M_object_scroll_offset, max_start);
            const auto end = std::min(start + rows, M_object_info.size());

            std::vector<std::string> lines{
                end > start
                    ? std::format("showing {}-{} of {}", start + 1, end, M_object_info.size())
                    : std::format("showing 0 of {}", M_object_info.size()),
            };

            for (std::size_t i = start; i < end; ++i)
            {
                const auto &object = M_object_info[i];
                lines.push_back(std::format("[{}:{}] {}  m={::N[.2f]}", object.handle.index(),
                                            object.handle.generation(), object.type, object.mass));
                lines.push_back(std::format("    pos {::.2f}", object.position));
                lines.push_back(std::format("    vel {::.2f}", object.velocity));
                lines.push_back(std::format("    spd {::N[.2f]}", object.speed));
            }

            if (end < M_object_info.size() || start > 0) lines.emplace_back("([ ] to scroll)");

            draw_section(shader, "Objects", margin_x, y_left, lines, Alignment::TopLeft);
        }
        else
        {
            std::size_t dynamic_count = 0;
            std::size_t static_count = 0;
            auto total_mass = 0.0 * physkit::si::kilogram;
            auto total_speed = 0.0 * physkit::si::metre / physkit::si::second;
            auto max_speed = 0.0 * physkit::si::metre / physkit::si::second;
            auto min_speed = std::numeric_limits<speed_quantity>::max();
            const auto speed_threshold = 0.01 * physkit::si::metre / physkit::si::second;

            for (const auto &object : M_object_info)
            {
                max_speed = std::max(max_speed, object.speed);
                if (object.speed > speed_threshold) min_speed = std::min(min_speed, object.speed);

                if (object.is_static)
                {
                    ++static_count;
                    continue;
                }

                ++dynamic_count;
                total_mass += object.mass;
                total_speed += object.speed;
            }

            const auto average_speed = dynamic_count == 0
                                           ? 0.0 * physkit::si::metre / physkit::si::second
                                           : total_speed / static_cast<double>(dynamic_count);
            if (min_speed == std::numeric_limits<speed_quantity>::max())
                min_speed = 0.0 * physkit::si::metre / physkit::si::second;

            draw_section(shader, "Object Summary", margin_x, y_left,
                         {
                             std::format("total     {}", M_object_info.size()),
                             std::format("dynamic   {}  m={::N[.2f]}", dynamic_count, total_mass),
                             std::format("static    {}", static_count),
                             std::format("avg spd   {::N[.2f]}", average_speed),
                             std::format("max spd   {::N[.2f]}", max_speed),
                             std::format("min spd   {::N[.2f]}", min_speed),
                         },
                         Alignment::TopLeft);
        }
    }

    if (M_show_performance)
    {
        draw_section(shader, "Performance", right_x, y_right,
                     {
                         std::format("fps        {::N[.1f]}", M_fps),
                         std::format("frame avg  {::N[.2f]}", M_avg_frame_time),
                         std::format("frame min  {::N[.2f]}", M_min_frame_time),
                         std::format("frame max  {::N[.2f]}", M_max_frame_time),
                         std::format("samples    {}/{}", M_frame_history.size(), max_frame_samples),
                     },
                     Alignment::TopRight);
    }

    if (M_show_camera)
    {
        draw_section(shader, "Camera", right_x, y_right,
                     {
                         std::format("position  {::.2f}", M_camera.pos()),
                         std::format("forward   {::.2f}", M_camera.forward()),
                         std::format("speed     {::N[.2f]}", M_camera.speed()),
                     },
                     Alignment::TopRight);
    }

    if (M_show_help)
    {
        const std::vector<std::string> lines{
            "F1   help",           "F2   physics",        "F3   camera",
            "F4   performance",    "F5   contact count",  "F6   object list",
            "F7   object detail",  "F9   toggle overlay", "Esc  hide overlay",
            "[ ]  scroll objects",
        };
        const auto block_height = line_height * (static_cast<float>(lines.size()) + 1.5f);
        const auto cx = M_window_size.x() * 0.5f;
        auto cy = (M_window_size.y() - block_height) * 0.5f;
        draw_section(shader, "Help", cx, cy, lines, Alignment::TopCenter);
    }
}

void overlay::draw_text(Magnum::Shaders::VectorGL2D &shader, const std::string &text, float x,
                        float y, const Color4 &color, Magnum::Text::Alignment alignment)
{
    if (!text_ready() || text.empty()) return;

    const Vector2 cursor{(-M_window_size.x() * 0.5f) + x, (M_window_size.y() * 0.5f) - y};

    M_text_renderer->clear().setCursor(cursor).setAlignment(alignment).render(*M_shaper, font_size,
                                                                              text);

    shader.setTransformationProjectionMatrix(Matrix3::projection(M_window_size))
        .setColor(color)
        .bindVectorTexture(M_cache->texture())
        .draw(M_text_renderer->mesh());
}

void overlay::draw_section(Magnum::Shaders::VectorGL2D &shader, const std::string &title, float x,
                           float &y, const std::vector<std::string> &lines,
                           Magnum::Text::Alignment alignment)
{
    draw_text(shader, title, x, y, title_color, alignment);
    y += line_height + title_gap;

    for (const auto &line : lines)
    {
        draw_text(shader, line, x, y, value_color, alignment);
        y += line_height;
    }

    y += section_gap;
}

void overlay::handle_key(int key, int scan_code, int action, int mods)
{
    if (action != GLFW_PRESS) return;
    if (key != GLFW_KEY_F9 && !M_visible) return;
    switch (key)
    {
    case GLFW_KEY_F1:
        M_show_help = !M_show_help;
        break;
    case GLFW_KEY_F2:
        M_show_physics = !M_show_physics;
        break;
    case GLFW_KEY_F3:
        M_show_camera = !M_show_camera;
        break;
    case GLFW_KEY_F4:
        M_show_performance = !M_show_performance;
        break;
    case GLFW_KEY_F5:
        M_show_collision = !M_show_collision;
        break;
    case GLFW_KEY_F6:
        M_show_objects = !M_show_objects;
        break;
    case GLFW_KEY_F7:
        M_show_detail_objects = !M_show_detail_objects;
        break;
    case GLFW_KEY_F9:
        toggle();
        break;
    case GLFW_KEY_RIGHT_BRACKET:
        if (M_show_detail_objects)
        {
            M_object_scroll_offset += 5;
            clamp_object_scroll();
        }
        break;
    case GLFW_KEY_LEFT_BRACKET:
        if (M_show_detail_objects)
            M_object_scroll_offset = M_object_scroll_offset > 5 ? M_object_scroll_offset - 5 : 0;
        break;
    default:
        break;
    }
}

void overlay::update_performance(physkit::quantity<physkit::si::second> dt)
{
    using namespace mp_units::si::unit_symbols;
    const physkit::quantity<physkit::si::milli<s>> dt_ms = dt;
    const auto now = std::chrono::steady_clock::now();

    M_frame_history.push_back({.dt_seconds = dt, .time = now});
    while (M_frame_history.size() > max_frame_samples) M_frame_history.pop_front();

    std::size_t frames_last_second = 0;
    for (auto &it : std::views::reverse(M_frame_history))
    {
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - it.time);
        if (age.count() >= 1000) break;
        ++frames_last_second;
    }

    M_fps = static_cast<double>(frames_last_second) / 1.0 * physkit::si::hertz;
    if (M_frame_history.empty())
    {
        M_avg_frame_time = dt_ms;
        M_min_frame_time = dt_ms;
        M_max_frame_time = dt_ms;
        return;
    }

    const auto samples = std::min(M_frame_history.size(), max_frame_samples);
    auto sum = 0.0 * ms;
    M_min_frame_time = std::numeric_limits<physkit::quantity<physkit::si::milli<s>>>::max();
    M_max_frame_time = 0.0 * ms;

    auto it = M_frame_history.rbegin();
    for (std::size_t i = 0; i < samples; ++i, ++it)
    {
        const physkit::quantity<physkit::si::milli<s>> sample_ms = it->dt_seconds;
        sum += sample_ms;
        M_min_frame_time = std::min(M_min_frame_time, sample_ms);
        M_max_frame_time = std::max(M_max_frame_time, sample_ms);
    }

    M_avg_frame_time = sum / static_cast<double>(samples);
}

void overlay::clamp_object_scroll()
{
    // The visible row count is computed at draw time (depends on window size),
    // so cap the offset at the last index here and let draw() further clamp the
    // displayed start so a full page is shown when room allows.
    if (M_object_info.empty())
        M_object_scroll_offset = 0;
    else
        M_object_scroll_offset = std::min(M_object_scroll_offset, M_object_info.size() - 1);
}

} // namespace debug
