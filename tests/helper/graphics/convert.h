#pragma once

#include "detail/macro.h"

#ifndef GRAPHICS_IN_MODULE_IMPL

#ifndef PHYSKIT_IMPORT_STD
#include <cstddef>
#endif

#ifndef PHYSKIT_MODULES
#include <mp-units/framework.h>
#include <physkit/physkit.h>
#endif

#include "detail/magnum_headers.h"

#endif

GRAPHICS_EXPORT
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
constexpr physkit::vec<Size, ref, T> to_physkit_vector(const Magnum::Math::Vector<Size, U> &v)
{
    return detail::expand<physkit::vec<Size, ref, T>>(
        [&](auto i) { return mp_units::value_cast<ref, T>(v[i] * ref); },
        std::make_index_sequence<Size>{});
}

template <mp_units::Reference auto ref, typename T, mp_units::Quantity Q>
constexpr Magnum::Math::Quaternion<T> to_magnum_quaternion(const physkit::unit_quat<Q> &q)
{
    return Magnum::Math::Quaternion<T>(to_magnum_vector<ref, T>(q.vec()),
                                       q.w().numerical_value_in(ref));
}

template <mp_units::Reference auto ref, typename T, typename U>
constexpr physkit::quat<ref, T> to_physkit_quaternion(const Magnum::Math::Quaternion<U> &q)
{
    return physkit::quat<ref, T>(q.scalar(), q.vector().x(), q.vector().y(), q.vector().z());
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
        vertex_buf[i].position = to_magnum_vector<mp_units::si::metre, Float>(verts[i]);
        vertex_buf[i].normal = {};
    }

    std::vector<UnsignedInt> indices;
    indices.reserve(tris.size() * 3);

    for (const auto &tri : tris)
    {
        auto normal = to_magnum_vector<mp_units::one, float>(tri.normal(phys_mesh));
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

inline Magnum::GL::Mesh to_magnum_mesh(const physkit::box &phys_box)
{
    using namespace Magnum;
    Vector3 half_extents = to_magnum_vector<physkit::si::metre, float>(phys_box.half_extents());
    auto data = MeshTools::copy(Primitives::cubeSolid());
    for (Vector3 &i : data.mutableAttribute<Vector3>(Trade::MeshAttribute::Position))
        i = Matrix4::scaling(half_extents).transformPoint(i);

    return MeshTools::compile(data);
}

inline Magnum::GL::Mesh to_magnum_mesh(const physkit::sphere &phys_sphere,
                                       unsigned int subdivisions = 3)
{
    using namespace Magnum;
    float radius = phys_sphere.radius().numerical_value_in(physkit::si::metre);
    auto data = Primitives::icosphereSolid(subdivisions);
    if (radius != 1.0)
    {
        for (Vector3 &i : data.mutableAttribute<Vector3>(Trade::MeshAttribute::Position))
            i = Matrix4::scaling({radius, radius, radius}).transformPoint(i);
    }

    return MeshTools::compile(data);
}

inline Magnum::GL::Mesh to_magnum_mesh(const physkit::cone &phys_cone, unsigned int rings = 3,
                                       unsigned int segments = 24)
{
	//THERE seems to be no bottom to the cone...
    //rings, segments, half length
    using namespace Magnum;
    using namespace Math::Literals;
    float radius = phys_cone.radius().numerical_value_in(physkit::si::metre);
    float height = phys_cone.height().numerical_value_in(physkit::si::metre);
    auto data = Primitives::coneSolid(rings, segments, 0.5f*height/radius, Primitives::ConeFlag::CapEnd);

    for (Vector3 &i : data.mutableAttribute<Vector3>(Trade::MeshAttribute::Position))
        i = Matrix4::scaling({radius, radius, radius}).transformPoint(i);
    for (Vector3 &i : data.mutableAttribute<Vector3>(Trade::MeshAttribute::Position))
        i = Matrix4::translation({0,height*0.5f,0}).transformPoint(i);


    return MeshTools::compile(data);
}
} // namespace graphics
