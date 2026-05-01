#pragma once

#include "detail/macro.h"

#ifndef GRAPHICS_IN_MODULE_IMPL
#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Corrade/Containers/Pointer.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Shaders/VectorGL.h>
#include <Magnum/Text/AbstractFont.h>
#include <Magnum/Text/AbstractShaper.h>
#include <Magnum/Text/Alignment.h>
#include <Magnum/Text/GlyphCacheGL.h>
#include <Magnum/Text/RendererGL.h>

#include <physkit/physkit.h>

#include "camera.h"
#endif

GRAPHICS_EXPORT
namespace debug
{

class overlay
{
public:
    struct ObjectDebugInfo
    {
        physkit::vec3<physkit::si::metre> position;
        physkit::vec3<physkit::si::metre / physkit::si::second> velocity;
        physkit::quantity<physkit::si::metre / physkit::si::second> speed{};
        physkit::quantity<physkit::si::kilogram> mass{};
        bool is_static{};
        std::string type;
        physkit::object_handle handle =
            physkit::object_handle::from_id(physkit::object_handle::null);
    };

    overlay(physkit::world_base &world, const graphics::camera &camera);

    void update(physkit::quantity<physkit::si::second> dt,
                physkit::quantity<physkit::si::second> total_time);
    void draw(Magnum::Shaders::VectorGL2D &shader, const Magnum::Matrix3 &projection);
    void handle_key(int key, int scan_code, int action, int mods);

    void controls(std::string_view title, std::initializer_list<std::string_view> lines)
    {
        controls(title,
                 lines | std::views::transform([](std::string_view s) { return std::string(s); }));
    }
    template <std::ranges::range R> void controls(std::string_view title, R &&lines)
    {
        M_controls.assign_range(std::forward<R>(lines));
        M_controls_title = title;
        M_controls_visible = !M_controls.empty();
    }
    void controls_visible(bool visible) { M_controls_visible = visible; }
    [[nodiscard]] bool has_visible_content() const;

    void toggle() { M_visible = !M_visible; }
    void visible(bool visible) { M_visible = visible; }
    [[nodiscard]] bool is_visible() const { return M_visible; }

private:
    struct FrameData
    {
        physkit::quantity<physkit::si::second> dt_seconds{};
        std::chrono::steady_clock::time_point time;
    };

    static constexpr std::size_t max_frame_samples = 120;
    static constexpr std::size_t max_detail_rows = 15;
    static constexpr float font_size = 14.0f;
    // Rasterize glyphs at a higher resolution than they are displayed so the cache stays crisp
    // on HiDPI/Retina framebuffers, where one logical pixel covers multiple physical pixels.
    static constexpr float font_raster_size = font_size * 3.0f;
    static constexpr float line_height = 20.0f;

    physkit::world_base &M_world;
    const graphics::camera &M_camera;

    Corrade::PluginManager::Manager<Magnum::Text::AbstractFont> M_font_manager;
    Corrade::Containers::Pointer<Magnum::Text::AbstractFont> M_font;
    Corrade::Containers::Pointer<Magnum::Text::AbstractShaper> M_shaper;
    std::unique_ptr<Magnum::Text::GlyphCacheGL> M_cache;
    std::unique_ptr<Magnum::Text::RendererGL> M_text_renderer;
    Magnum::Vector2 M_window_size{800.0f, 600.0f};

    physkit::quantity<physkit::si::second> M_total_time{0.0 * physkit::si::second};

    bool M_visible = true;
    bool M_show_help = false;
    bool M_show_physics = true;
    bool M_show_camera = true;
    bool M_show_performance = true;
    bool M_show_collision = true;
    bool M_show_objects = true;
    bool M_show_detail_objects = true;
    bool M_controls_visible = false;

    std::string M_controls_title = "Controls";
    std::vector<std::string> M_controls;

    std::deque<FrameData> M_frame_history;
    physkit::quantity<physkit::si::hertz> M_fps = 0.0 * physkit::si::hertz;
    physkit::quantity<physkit::si::milli<physkit::si::second>> M_avg_frame_time{};
    physkit::quantity<physkit::si::milli<physkit::si::second>> M_min_frame_time{};
    physkit::quantity<physkit::si::milli<physkit::si::second>> M_max_frame_time{};

    std::vector<ObjectDebugInfo> M_object_info;
    std::size_t M_total_collisions = 0;
    std::size_t M_object_scroll_offset = 0;

    void initialize_font();
    [[nodiscard]] bool text_ready() const;

    void draw_text(Magnum::Shaders::VectorGL2D &shader, const std::string &text, float x, float y,
                   const Magnum::Color4 &color = Magnum::Color4{1.0f},
                   Magnum::Text::Alignment alignment = Magnum::Text::Alignment::TopLeft);
    void draw_section(Magnum::Shaders::VectorGL2D &shader, const std::string &title, float x,
                      float &y, const std::vector<std::string> &lines,
                      Magnum::Text::Alignment alignment = Magnum::Text::Alignment::TopLeft);

    void update_performance(physkit::quantity<physkit::si::second> dt);
    void clamp_object_scroll();
};

} // namespace debug
