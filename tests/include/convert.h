#pragma once
#include <Magnum/Math/Vector.h>
#include <Magnum/GL/Mesh.h>
#include <mp-units/framework.h>
#include <physkit/physkit.h>
#include <utility>

namespace graphics
{
    template <typename T, int Size, mp_units::Quantity Q>
    constexpr Magnum::Math::Vector<Size, T> to_magnum_vector(const physkit::unit_mat<Q, Size, 1> &v)
    {
        auto [...indices] = std::make_index_sequence<Size>{};
        return Magnum::Math::Vector<Size, T>{static_cast<T>(v[indices].value().numerical_value_in(Q::reference))...};
    }

    template <auto ref, typename T, typename U, std::size_t Size>
    constexpr physkit::vec<Size, ref, T> to_physkit_vector(const Magnum::Math::Vector<Size, U> &v)
    {
        auto [...indices] = std::make_index_sequence<Size>{};
        return physkit::vec<Size, ref, T>(physkit::value_cast<ref, T>(v[indices] * ref)...);
    }

    Magnum::GL::Mesh to_magnum_mesh(const physkit::mesh &magnum_mesh) // TODO
    {
        Magnum::GL::Mesh mesh;
        return mesh;
    }
}