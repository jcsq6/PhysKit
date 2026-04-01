#pragma once
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <format>
#include <cstdint>

#include <Magnum/Text/AbstractFont.h>
#include <Magnum/Text/Renderer.h>
#include <Magnum/Text/DistanceFieldGlyphCache.h>
#include <Magnum/Shaders/VectorGL.h>
#include <Magnum/GL/Texture.h>
#include <Corrade/Containers/StringView.h>

#include "physkit/world.h"
#include "camera.h"

namespace debug {

    class Overlay {
    public:
        struct ObjectDebugInfo {
            physkit::vec3<physkit::si::metre, float> position;
            physkit::vec3<physkit::si::metre / physkit::si::second, float> velocity;
            float speed;
            float mass;
            bool is_static;
            std::string type;
            size_t handle_index;
        };

    Overlay(physkit::world& world, const graphics::camera& camera);

    void update(float dt, physkit::quantity<physkit::si::second> total_time);
    void draw(Magnum::Shaders::VectorGL2D& shader, const Magnum::Matrix3& projection);
    
    void toggle() { M_visible = !M_visible; }
    [[nodiscard]] bool is_visable() const { return M_visible; }

    void handle_key(int key, int scan_code, int action, int mods);

    private:
    // Rendering:
    Magnum::GL::Buffer M_buffer;
    Magnum::GL::Mesh M_mesh;
    std::unique_ptr<Magnum::Text::AbstractFont> M_font;
    std::unique_ptr<Magnum::Text::DistanceFieldGlyphCache> M_cache;
    Magnum::Shaders::VectorGL2D* M_shader = nullptr;
    Magnum::Vector2 M_window_size {800.0f, 600.0f}; // Default resolution for now, but can change later.

    physkit::world& M_world;
    const graphics::camera& M_camera;

    physkit::quantity<physkit::si::second> M_total_time{0.0f * physkit::si::second};

    // Debug State:
    bool M_visible = true;
    bool M_show_help = false;
    bool M_show_physics = true;
    bool M_show_camera = true;
    bool M_show_performance = true;
    bool M_show_collision = true;
    bool M_show_objects = true;
    bool M_show_detail_objects = true;

    struct FrameData {
        float dt;
        std::chrono::steady_clock::time_point time;
    };

    std::deque<FrameData> M_frame_history;
    static constexpr size_t max_frames = 120; // Subject to change

    float M_fps = 0.0f;
    float M_avg_frame_time = 0.0f;
    float M_min_frame_time = 1000.0f;
    float M_max_frame_time = 0.0f;

    std::vector<ObjectDebugInfo> M_object_info;
    size_t M_total_collisions = 0;

    void draw_text(Magnum::Shaders::VectorGL2D& shader,
                   const std::string& text,
                   float x, float y,
                   const Magnum::Color4& color = Magnum::Color4::WHITE());

    void draw_panel(Magnum::Shaders::VectorGL2D& shader,
                    const std::string& title,
                    float x, float y,
                    const std::vector<std::string>& lines);
    
    void update_performance(float dt);
    std::string format_vector(const physkit::vec3<physkit::si::metre / physkit::si::second>& v);
    std::string format_quat(const physkit::quat<physkit::one, float>& q);

    };

}