#pragma once
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Vector.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Shaders/PhongGL.h>
#include <cstddef>
#include <mp-units/framework.h>
#include <physkit/physkit.h>
#include <utility>
#include <vector>

namespace graphics
{
namespace detail
{
template <typename Ret, std::size_t... Indices>
constexpr auto expand(auto &&nth, std::index_sequence<Indices...> /*helper*/)
{
    return Ret{nth(Indices)...};
}
} // namespace detail

template <mp_units::Reference auto ref, typename T, int Size, mp_units::Quantity Q>
constexpr Magnum::Math::Vector<Size, T> to_magnum_vector(const physkit::unit_mat<Q, Size, 1> &v)
{
    return detail::expand<Magnum::Math::Vector<Size, T>>(
        [&](auto i) { return static_cast<T>(v[i].numerical_value_in(ref)); },
        std::make_index_sequence<Size>{});
}

template <mp_units::Reference auto ref, typename T, typename U, std::size_t Size>
constexpr physkit::uvec<Size, ref, T> to_physkit_vector(const Magnum::Math::Vector<Size, U> &v)
{
    return detail::expand<physkit::uvec<Size, ref, T>>(
        [&](auto i) { return physkit::value_cast<ref, T>(v[i] * ref); },
        std::make_index_sequence<Size>{});
}

template <mp_units::Reference auto ref, typename T, mp_units::Quantity Q>
constexpr Magnum::Math::Quaternion<T> to_magnum_quaternion(const physkit::unit_quat<Q> &q)
{
    return Magnum::Math::Quaternion<T>(to_magnum_vector<ref, T>(q.vec()),
                                       q.w().numerical_value_in(ref));
}

template <mp_units::Reference auto ref, typename T, typename U>
constexpr physkit::uquat<ref, T> to_physkit_quaternion(const Magnum::Math::Quaternion<U> &q)
{
    return physkit::quat<T>(q.w(), q.x(), q.y(), q.z()) * ref;
}

inline Magnum::GL::Mesh to_magnum_mesh(const physkit::mesh &phys_mesh)
{
    using namespace Magnum;

    auto verts = phys_mesh.vertices();
    auto tris = phys_mesh.triangles();

    struct vertex_data
    {
        Math::Vector3<Float> position;
        Math::Vector3<Float> normal;
    };

    std::vector<vertex_data> vertex_buf(verts.size());

    for (std::size_t i = 0; i < verts.size(); ++i)
    {
        vertex_buf[i].position = to_magnum_vector<physkit::si::metre, Float>(verts[i]);
        vertex_buf[i].normal = {};
    }

    std::vector<UnsignedInt> indices;
    indices.reserve(tris.size() * 3);

    for (const auto &tri : tris)
    {
        auto normal = to_magnum_vector<physkit::one, float>(tri.normal(phys_mesh));
        vertex_buf[tri[0]].normal += normal;
        vertex_buf[tri[1]].normal += normal;
        vertex_buf[tri[2]].normal += normal;

        indices.append_range(tri);
    }

    for (auto &v : vertex_buf) v.normal = v.normal.normalized();

    GL::Buffer vertex_buffer;
    vertex_buffer.setData(vertex_buf);

    GL::Buffer index_buffer;
    index_buffer.setData(indices);

    GL::Mesh mesh;
    mesh.setPrimitive(GL::MeshPrimitive::Triangles)
        .setCount(static_cast<Int>(indices.size()))
        .addVertexBuffer(std::move(vertex_buffer), 0, Shaders::PhongGL::Position{},
                         Shaders::PhongGL::Normal{})
        .setIndexBuffer(std::move(index_buffer), 0, GL::MeshIndexType::UnsignedInt);

    return mesh;
}
} // namespace graphics