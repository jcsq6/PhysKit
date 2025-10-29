#include <graphics.h>
#include <print>

using namespace Magnum;

class triangle_app : public Platform::Application // NOLINT
{
public:
    virtual ~triangle_app() = default;
    explicit triangle_app(const Arguments &arguments)
        : Platform::Application{arguments, Configuration{}.setTitle("Magnum Triangle Example")}
    {
        using namespace Math::Literals;
        using namespace ColorLiterals;

        struct TriangleVertex
        {
            Vector2 position;
            Color3 color;
        };
        const std::array<TriangleVertex, 3> vertices = {{
            {.position = {-0.5f, -0.5f}, .color = 0xff0000_rgbf}, /* Left vertex, red color */
            {.position = {0.5f, -0.5f}, .color = 0x00ff00_rgbf},  /* Right vertex, green color */
            {.position = {0.0f, 0.5f}, .color = 0x0000ff_rgbf}    /* Top vertex, blue color */
        }};

        M_mesh.setCount(vertices.size())
            .addVertexBuffer(GL::Buffer{Containers::arrayView(vertices.data(), vertices.size())}, 0,
                             Shaders::VertexColorGL2D::Position{},
                             Shaders::VertexColorGL2D::Color3{});
    }


private:
    void drawEvent() override
    {
        GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);

        M_shader.draw(M_mesh);

        swapBuffers();
    }

    GL::Mesh M_mesh;
    Shaders::VertexColorGL2D M_shader;
};

int main(int argc, char **argv)
{
    std::print("PhysKit graphics demo: {}\n", physkit::version_string());
    triangle_app app{{argc, argv}};
    return app.exec();
}