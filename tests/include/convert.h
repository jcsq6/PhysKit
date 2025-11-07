#pragma once
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Vector.h>
#include <cstddef>
#include <mp-units/framework.h>
#include <physkit/physkit.h>
#include <utility>

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
template <typename T, int Size, mp_units::Quantity Q>
constexpr Magnum::Math::Vector<Size, T> to_magnum_vector(const physkit::unit_mat<Q, Size, 1> &v)
{
    return detail::expand<Magnum::Math::Vector<Size, T>>([&](auto i)
                          { return static_cast<T>(v[i].numerical_value_in(Q::reference)); },
                          std::make_index_sequence<Size>{});
}

template <auto ref, typename T, typename U, std::size_t Size>
constexpr physkit::vec<Size, ref, T> to_physkit_vector(const Magnum::Math::Vector<Size, U> &v)
{
    return detail::expand<physkit::vec<Size, ref, T>>([&](auto i) { return physkit::value_cast<ref, T>(v[i] * ref); },
                          std::make_index_sequence<Size>{});
}

Magnum::GL::Mesh to_magnum_mesh(const physkit::mesh &magnum_mesh) // TODO
{
    Magnum::GL::Mesh mesh;
    return mesh;
}
} // namespace graphics