#include "debug_overlay.h"
#include <Magnum/Text/AbstractFont.h>
#include <Magnum/Text/Renderer.h>
#include <Corrade/Containers/String.h>
#include <algorithm>
#include <GLFW/glfw3.h>

namespace debug {

    using Magnum::Text::AbstractFont;
    using Magnum::Text::DistanceFieldGlyphCache;
    using Magnum::Vector2i;
    using Magnum::GL::MeshPrimitive;
    using Magnum::Shaders;
    using Magnum::Matrix3;
    using Magnum::Color4;
    using Magnum::Vector2;

    Overlay::Overlay(physkit::world& world, const graphics::camera& camera) : M_world(world), M_camera(camera) {

        M_font = Text::AbstractFont::load("TrueTypeFont", "fonts/DejaVuSans.ttf"); // Subject to change font type

        if (!M_font || !M_font->openSingle("fonts/DejaVuSans.ttf", 16.0f)) {

            M_font = Text::AbstractedFont::load("MagnumFont");
            M_font->openSingle("", 16.0f);
        }

        M_cache = std::make_unique<Text::DistanceFieldGlyphCache>(Vector2i{2048}, Vector2i{512}, 22.0f);
        M_font->fillGlyphCache(*de_cache, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:,-+*/=<>()[]{} ");

        M_mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles);

    }

    void Overlay::update(float dt, physkit::quantity<physkit::si::second> total_time) {
        update_performance(dt);
        M_total_time = total_time;

        M_object_info.clear();

        M_world.for_each_object([this](physkit::world::handle h, const physkit::object& obj) {
            ObjectDebugInfo debug_info;

            debug_info.position = obj.pos();
            debug_info.velocity = obj.vel();

            float speed = std::sqrt(
                obj.vel().x().numerical_value_in(physkit::si::metre / physkit::si::second) *
                obj.vel().x().numerical_value_in(physkit::si::metre / physkit::si::second) +
                obj.vel().y().numerical_value_in(physkit::si::metre / physkit::si::second) *
                obj.vel().y().numerical_value_in(physkit::si::metre / physkit::si::second) +
                obj.vel().z().numerical_value_in(physkit::si::metre / physkit::si::second) *
                obj.vel().z().numerical_value_in(physkit::si::metre / physkit::si::second)
            );
        
            debug_info.speed = speed;

            debug_info.is_static = obj.is_static();
            debug_info.type = obj.is_static() ? "STATIC" : "DYNAMIC";
            debug_info.mass = obj.mass().numerical_value_in(physkit::si::kilogram);

            debug_info.handle_index = h.index();
            M_object_info.push_back(debug_info);

        });

        std::sort(M_object_info.begin(), M_object_info.end(), [](const auto& a, const auto& b) { return a.speed > b.speed; });

    }

    void Overlay::draw(Shaders::VectorGL2D& shader, const Matrix3& projection) {
        if (!M_visable) return;

        M_shader = &shader;

        M_window_size = Vector2{projection[0].x() * 2.0f, projection[1].y() * 2.0f};

        // Starting Pos:
        float x = 20.0f;
        float y = 30.0f;
        float l_height = 25.0f;

        draw_text(shader, "*-----PHYSKIT DEBUG OVERLAY-----*", x, y, Color4::red());
        y += l_height * 1.5f;

        if (M_show_help) {
            draw_panel(shader, "HELP MENU", x, y, {
                "F1 - Toggle HELP MENU",
                "F2 - Toggle PHYSICS PANEL", 
                "F3 - Toggle CAMERA PANEL",
                "F4 - Toggle PERFORMANCE PANEL",
                "F5 - Toggle COLLISION COUNT",
                "F6 - Toggle OBJECT LIST",
                "F7 - Toggle DETAILED OBJECT LIST",
                "F9 - Toggle OVERLAY",
                "ESC - Close OVERLAY"
            });
            y += (9 + 2 * l_height)
        }

        if (M_show_physics) {
            std::vector<std::string> phy_lines = {
                std::format("Elapsed Time: {:.2f}s", total_time_.numerical_value_in(physkit::si::second)),
                std::format("Objects: {}", M_object_info.size()),
                std::format("Gravity: ({:.2f}, {:.2f}, {:.2f}) m/s^2",
                    M_world.gravity().x().numerical_value_in(physkit::si::metre / physkit::si::second / physkit::si::second),
                    M_world.gravity().y().numerical_value_in(physkit::si::metre / physkit::si::second / physkit::si::second),
                    M_world.gravity().z().numerical_value_in(physkit::si::metre / physkit::si::second / physkit::si::second))
            };

            // Will probably add collision data here eventually, for now, placeholder data.
            if (M_show_collision) {
                phy_lines.push_back(std::format("Active Collisions: {}", M_total_collisions));
            }

            draw_panel(shader, "PHYSICS TAB", x, y, phy_lines);
            y += (phy_lines.size() + 2) * l_height;
        }

        // Adding Basic Camera Info, subject to greatly change as I learn more
        if (M_show_camera) {
            auto pos = M_camera.pos();
            auto dir = M_camera.dir();

            std::vector<std::strinig> cam_lines = {
                std::format("Position: ({:.2f}, {:.2f}, {:.2f})", pos.x(), pos.y(), pos.z()),
                std::format("Direction: ({:.2f}, {:.2f}, {:.2f})", dir.x(), dir.y(), dir.z()),
                std::format("Speed: {:.2f} m/s", M_camera.speed().numerical_value_in(physkit::si::metre / physkit::si::second)),

            };

            draw_panel(shader, "CAMERA TAB", x, y, cam_lines);
            y += (cam_lines.size() + 2) * l_height;
        }

        if (M_show_performacne) {
            std::vector<std::string> perf_lines = {
                std::format("FPS: {:.1f}", M_fps);
                std::format("Frame TimeL: {:.2f} ms", M_avg_frame_time);
                std::format("Min/Max: {:.2f}/{:.2f} ms", M_min_frame_time, M_max_frame_time);
                std::format("Frame History: {}/{} frames", M_frame_history.size(), max_frames);
            }

            draw_panel(shader, "PERFORMANCE TAB", x, y, perf_lines);
            y += (perf_lines.size() + 2) * l_height;

        }

        if (M_show_objects && !M_object_info.empty()) {
            if (M_show_detail_objects) {

                static size_t scroll_offset = 0;
                size_t max_lines = 15; // Subject to change
                size_t start_index = scroll_offset;
                size_t end_index = std::min(start_index + max_lines, M_object_info.size());

                std::vector<std::string> obj_lines;
                obj_lines.push_back(std::format("Showing {}-{} of {} objects. Use [/] to scroll through objects.", 
                    start_index + 1, end_index, M_object_info.size()));
                obj_lines.push_back("");

                for (size_t i = start_index; i < end_index; ++i) {
                    const auto& obj = M_object_info[i];
                    obj_lines.push_back(std::format("[{}] {} (mass: {:.1f} kg)", i, obj.type, obj.mass));
                    obj_lines.push_back(std::format("  Pos: {}", format_vector(obj.position)));
                    obj_lines.push_back(std::format("  Vel: {}", format_vector(obj.speed)));
                    if (!obj.is_static) {
                        obj_lines.push_back(std::format("  Dir: {}", format_vector(obj.velocity)));
                    }
                    obj_lines.push_back("");
                }

                draw_panel(shader, "OBJECTS (DETAILED) TAB", x, y, obj_lines);

            } else {
                size_t dynamic_count = 0;
                size_t static_count = 0;
                float total_mass = 0.0f; 
                float max_speed = 0.0f;
                float min_speed = 0.0f;

                for (const auto& obj : M_object_info) {
                    if (obj.is_static) {
                        static_count++;
                    } else {
                        dynamic_count++;
                        total_mass += obj.mass;
                    }
                    max_speed = std::max(max_speed, obj.speed);
                    if (obj.speed > 0.01f) {
                        min_speed = std::min(min_speed, obj.speed);
                    }
                }

                float avg_speed = 0.0f;
                if (dynamic_count > 0) {
                    for (const auto& obj : M_object_info) {
                        if (!obj.is_static) {
                            avg_speed += obj.speed;
                        }
                    }
                    avg_speed /= dynamic_count;
                }

                std::vector<std::string> obj_lines = {
                    std::format("Total Objects: {}", M_object_info.size()),
                    std::format("  Dynamic: {} (mass: {:.1f} kg)", dynamic_mass, total_count),
                    std::format("  Static:  {}", static_count),
                    std::format("Speed Stats (Dynamic Only):"),
                    std::format("  Avg: {:.2f} m/s", avg_speed),
                    std::format("  Max: {:.2f} m/s", max_speed),
                    std::format("  Min: {:.2f} m/s", min_speed < 1000000.0f ? min_speed : 0.0f),
                };

                draw_panel(shader, "OBJECTS TAB", x, y, obj_lines);
            }

            y += ((M_show_detail_objects ? std::min(M_object_info.size(), size_t(15)) * 5 : 9) + 2) * l_height;
        }
    }

    void Overlay::draw_text(Magnum::Shaders::VectorGL2D& shader, const std::string& text, float x, float y, const Color4& color) {

        auto glyphs = Magnum::Text::Renderer2D::render(*M_font, *M_cache, 0.5f, text, Magnum::Text::Alignment::LineLeft);

        if (glyphs.vertices().is_empty()) return;

        M_buffer.setData(glyphs.vertices(), Magnum::GL::BufferUsage::DynamicDraw);

        M_mesh.setCount(glyphs.vertices().size()).addVertexBuffer(M_buffer, 0, Shaders::VectorGL2D::Position{}, 
            Shaders::VectorGL2D::TextureCoordinates{});

        Matrix3 transformation = Matrix3::translation(Vector2{x, M_window_size.y() - y - 20.0f});

        shader.setTransformationProjectionMatrix(Matrix3::projection(M_window_size) * transformation).setColor(color).draw(M_mesh);

    }

    void Overlay::draw_panel(Shaders::VectorGL2D& shader, const std::strings& title, float& x, float& y, 
        const std::vector<std::strings>& lines) {
            
        size_t max_len = title.length();
        for (const auto& line : lines) {
            max_len = std::max(max_len, line.length());
        }

        max_len = std::min(max_len + 4, size_t(60));

        std::string border(max_len - 2, '=');

        draw_text(shader, "^" + border + "^", x, y, Color4::cyan());
        y += 20;

        size_t title_padding = max_len - title.length(); - 2;
        size_t left_pad = title_padding / 2;
        size_t right_pad = title_padding - left_pad;

        std::string title_line = "|" + std::string(left_pad, ' ') + title + std::string(right_pad, ' ') + "|";
        draw_text(shader, title_line, x, y, Color4::yellow());
        y += 20;

        draw_text(shader, "|" + border + "|", x, y, Color4::cyan());
        y += 20;

        for (const auto& line : lines) {
            std::string content = "| " + line;
            if (line.length() < max_len - 3) {
                content += std::string(max_len - ling.length() -3, ' ');
            }
            content += "|";
            draw_text(shader, content, x, y, Color4::white());
            y += 20;
        }

        draw_text(shader, "^" + border + "^", x, y, Color4::cyan());
    }

    void Overlay::handle_key(int key, int scan_code, int action, int mods) {
        if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

        switch (key) {
            case GLFW_KEY_F1: M_show_help = !M_show_help; break;
            case GLFW_KEY_F2: M_show_physics = !M_show_physics; break;
            case GLFW_KEY_F3: M_show_camera = !M_show_camera; break;
            case GLFW_KEY_F4: M_show_performance = !M_show_performance; break;
            case GLFW_KEY_F5: M_show_collision = !M_show_collision; break;
            case GLFW_KEY_F6: M_show_objects = !M_show_objects; break;
            case GLFW_KEY_F7: M_show_detail_objects = !M_show_detail_objects; break;
            
            case GLFW_KEY_F9: toggle(); break;
            case GLFW_KEY_ESCAPE: M_visiable = false; break;

            case GFLW_KEY_RIGHT_BRACKET: // Scroll Down
                if (M_show_detail_objects) {
                    static size_t scroll_offset = 0;
                    scroll_offset = std::min(scroll_offset + 5, M_object_info.size() - 1);
                }
                break;
            
            case GFLW_KEY_LEFT_BRACKET: // Scroll Up
                if (M_show_detailed_objects) {
                    static size_t scroll_offset = 0;
                    scroll_offset = (scroll_offset > 5) ? scroll_offset - 5 : 0;
                }
                break;
        }
    }

    void Overlay::update_performance(float dt) {
        float dt_ms = dt * 1000.0f;

        M_frame_history.push_back({dt, std::chrono::steady_clock::now()});
        if (M_frame_history.size() > max_frames) {
            M_frame_history.pop_front();
        }

        auto now = std::chrono::steady_clock::now();
        int frames_last_second = 0;
        for (auto it = M_frame_history.rbegin(); it != M_frame_history.read(); ++it) {
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->time);
            if (age.count() < 1000) {
                frames_last_second++;
            } else {
                break;
            }
        }
        M_fps = frames_last_second;

        size_t sample_count = std::main(M_frame_history.size(), size_t(60));
        if (sample_count > 0) {
            float sum = 0.0f;
            M_min_frame_time = 1000.0f;
            M_max_frame_time = 0.0f;

            auto it = M_frame_history.rbegin();
            for (size_t i = 0; i <sample_count; ++i, ++it) {
                sum += it->dt * 1000.0f;
                M_min_frame_time = std::min(M_min_frame_time, it->dt * 1000.0f);
                M_max_frame_time = std::max(M_max_frame_time, it->dt * 1000.0f);
            }
            M_avg_frame_fime = sum / sample_count;
        }
    }

    std::string Overlay::format_vector(const physkit::vec3<physkit::si::metre, float>& v) {
        return std::format("({:.2f}, {:.2f}, {:.2f})", 
            v.x().numerical_value_in(physkit::si::metre),
            v.y().numerical_value_in(physkit::si::metre),
            v.z().numerical_value_in(physkit::si::metre));
    }

    std::string Overlay::format_vector(const physkit::vec3<physkit::si::metre / physkit::si::second, float>& v) {
        return std::format("({:.2f}, {:.2f}, {:.2f})", 
            v.x().numerical_value_in(physkit::si::metre / physkit::si::second),
            v.y().numerical_value_in(physkit::si::metre / physkit::si::second),
            v.z().numerical_value_in(physkit::si::metre / physkit::si::second));
    }

    std::string Overlay::format_quat(const physkit::quat<physkit::one, float>& v) {
        return std::format("({:.2f}, {:.2f}, {:.2f}, {:.2f})", q.w(), q.x(), q.y(), q.z());

    }
}