#pragma once
#include <Eigen/Dense>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/NumTraits.h>
#include <Eigen/src/Core/util/Meta.h>

#include <mp-units/framework.h>
#include <mp-units/systems/international.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/unit_symbols.h>
#include <mp-units/systems/si/units.h>

#include <type_traits>
#include <utility>

#include <initializer_list>

#include <Eigen/Core>

namespace physkit
{

using namespace mp_units;

namespace detail
{

template <Quantity Q>
    requires(!std::is_const_v<Q>)
class quantity_ref // NOLINT
{
public:
    using value_type = Q;
    using rep_type = typename value_type::rep;
    static constexpr auto ref = Q::reference;

    constexpr explicit quantity_ref(rep_type &value) noexcept : data(&value) {}
    quantity_ref(const quantity_ref &) = delete;

    constexpr quantity_ref &operator=(const quantity_ref &other) noexcept // NOLINT
    {
        return *this = static_cast<Q>(other);
    }

    constexpr operator Q() const noexcept { return *data * ref; }
    constexpr Q value() const noexcept { return *this; }

    constexpr quantity_ref &operator=(Q other) noexcept
    {
        *data = other.numerical_value_in(ref);
        return *this;
    }

    constexpr quantity_ref &operator+=(Q other) noexcept
    {
        *data += other.numerical_value_in(ref);
        return *this;
    }

    constexpr quantity_ref &operator-=(Q other) noexcept
    {
        *data -= other.numerical_value_in(ref);
        return *this;
    }

    constexpr quantity_ref &operator*=(QuantityOf<dimensionless> auto scalar) noexcept
    {
        *data *= scalar.numerical_value_in(dimensionless);
        return *this;
    }

    constexpr quantity_ref &operator/=(QuantityOf<dimensionless> auto scalar) noexcept
    {
        *data /= scalar.numerical_value_in(dimensionless);
        return *this;
    }

    constexpr quantity_ref &operator++() noexcept
    {
        ++(*data);
        return *this;
    }

    constexpr quantity_ref &operator--() noexcept
    {
        --(*data);
        return *this;
    }

    constexpr quantity_ref operator++(int) noexcept
    {
        quantity_ref temp = *this;
        ++(*data);
        return temp;
    }

    constexpr quantity_ref operator--(int) noexcept
    {
        quantity_ref temp = *this;
        --(*data);
        return temp;
    }

    constexpr auto operator<=>(Q other) const noexcept { return static_cast<Q>(*this) <=> other; }

    quantity_ref *operator&() = delete;
    const quantity_ref *operator&() const = delete;

private:
    rep_type *data;
};

template <typename T>
    requires(requires {
        typename std::remove_cvref_t<T>::value_type;
        std::remove_cvref_t<T>::ref;
    })
auto to_reference(auto &&value)
{
    if constexpr (std::is_const_v<std::remove_reference_t<T>>)
        return value * std::remove_cvref_t<T>::ref;
    else
        return quantity_ref<typename std::remove_cvref_t<T>::value_type>{value};
}

template <Quantity Q> constexpr auto operator*(quantity_ref<Q> a, auto scalar)
{
    return static_cast<Q>(a) * scalar;
}
template <Quantity Q> constexpr auto operator/(quantity_ref<Q> a, auto scalar)
{
    return static_cast<Q>(a) / scalar;
}
template <Quantity Q> constexpr auto operator+(quantity_ref<Q> a, Q other)
{
    return static_cast<Q>(a) + other;
}
template <Quantity Q> constexpr auto operator-(quantity_ref<Q> a, Q other)
{
    return static_cast<Q>(a) - other;
}

template <Quantity Q> constexpr auto operator*(auto scalar, quantity_ref<Q> b)
{
    return scalar * static_cast<Q>(b);
}
template <Quantity Q> constexpr auto operator/(auto scalar, quantity_ref<Q> b)
{
    return scalar / static_cast<Q>(b);
}
template <Quantity Q> constexpr auto operator+(auto scalar, quantity_ref<Q> b)
{
    return scalar + static_cast<Q>(b);
}
template <Quantity Q> constexpr auto operator-(auto scalar, quantity_ref<Q> b)
{
    return scalar - static_cast<Q>(b);
}
} // namespace detail

template <Quantity Q, int _rows, int _cols> class unit_mat
{
public:
    using value_type = Q;
    using rep_type = typename value_type::rep;
    using eigen_type = Eigen::Matrix<rep_type, _rows, _cols>;

    using reference = detail::quantity_ref<Q>;
    using const_reference = Q;

    static constexpr auto ref = Q::reference;

    constexpr unit_mat() = default;
    constexpr unit_mat(const unit_mat &) = default;
    constexpr unit_mat &operator=(const unit_mat &) = default;
    constexpr unit_mat(unit_mat &&) = default;
    constexpr unit_mat &operator=(unit_mat &&) = default;
    constexpr ~unit_mat() = default;

    constexpr unit_mat(const eigen_type &data) : data(data) {}
    constexpr unit_mat(eigen_type &&data) : data(std::move(data)) {}

    constexpr unit_mat(Eigen::Index dim) : data(dim) {}
    constexpr unit_mat(Eigen::Index rows, Eigen::Index cols) : data(rows, cols) {}

    constexpr unit_mat(const Q &scalar) : data(scalar.numerical_value_in(ref)) {}
    constexpr unit_mat(const Q *data_ptr) : data(static_cast<rep_type *>(data_ptr)) {}

    constexpr unit_mat(const Q &x, const Q &y)
        : data(x.numerical_value_in(ref), y.numerical_value_in(ref))
    {
    }
    constexpr unit_mat(const Q &x, const Q &y, const Q &z)
        : data(x.numerical_value_in(ref), y.numerical_value_in(ref), z.numerical_value_in(ref))
    {
    }
    constexpr unit_mat(const Q &x, const Q &y, const Q &z, const Q &w)
        : data(x.numerical_value_in(ref), y.numerical_value_in(ref), z.numerical_value_in(ref),
               w.numerical_value_in(ref))
    {
    }
    constexpr unit_mat(const std::initializer_list<std::initializer_list<Q>> &list)
        : data(list.size(), list.begin()->size())
    {
        Eigen::Index r = 0;
        for (const auto &row : list)
        {
            Eigen::Index c = 0;
            for (const auto &val : row)
            {
                data(r, c) = val.numerical_value_in(ref);
                ++c;
            }
            ++r;
        }
    }

    constexpr unit_mat &operator=(const eigen_type &other)
    {
        data = other;
        return *this;
    }

    constexpr unit_mat &operator=(eigen_type &&other)
    {
        data = std::move(other);
        return *this;
    }

    constexpr decltype(auto) base(this auto &&self)
    {
        return std::forward_like<decltype(self)>(self.data);
    }

    constexpr auto x(this auto &&self)
    {
        return detail::to_reference<decltype(self)>(std::forward<decltype(self)>(self).base().x());
    }
    constexpr auto y(this auto &&self)
    {
        return detail::to_reference<decltype(self)>(std::forward<decltype(self)>(self).base().y());
    }
    constexpr auto z(this auto &&self)
    {
        return detail::to_reference<decltype(self)>(std::forward<decltype(self)>(self).base().z());
    }
    constexpr auto w(this auto &&self)
    {
        return detail::to_reference<decltype(self)>(std::forward<decltype(self)>(self).base().w());
    }

    constexpr auto rows() const { return base().rows(); }
    constexpr auto cols() const { return base().cols(); }

    constexpr auto begin(this auto &&self)
        requires(eigen_type::IsVectorAtCompileTime == 1)
    {
        return std::forward_like<decltype(self)>(self.data).begin();
    }
    constexpr auto end(this auto &&self)
        requires(eigen_type::IsVectorAtCompileTime == 1)
    {
        return std::forward_like<decltype(self)>(self.data).end();
    }

    auto transpose() const { return unit_mat{base().transpose()}; }
    auto determinant() const
        requires(_rows == _cols)
    {
        return base().determinant() * mp_units::pow<_cols>(ref);
    }
    auto inverse() const
        requires(_rows == _cols)
    {
        return unit_mat<quantity<mp_units::one / ref, rep_type>, _rows, _cols>{base().inverse()};
    }
    auto trace() const { return base().trace() * ref; }

    template <Quantity OtherQ> auto dot(const unit_mat<OtherQ, _rows, _cols> &other) const
    {
        return base().dot(other.base()) * ref * OtherQ::reference;
    }

    template <Quantity OtherQ> auto cross(const unit_mat<OtherQ, _rows, _cols> &other) const
    {
        using result_quantity = quantity<ref * OtherQ::reference, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base().cross(other.base())};
    }

    auto squared_norm() const { return base().squaredNorm() * ref * ref; }

    auto norm() const { return base().norm() * ref; }

    auto normalized() const
    {
        using result_quantity = quantity<ref / ref, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base().normalized()};
    }

    constexpr bool operator==(const unit_mat &other) const = default;
    constexpr bool operator!=(const unit_mat &other) const = default;

    constexpr auto operator[](this auto &&self, Eigen::Index index)
    {
        return detail::to_reference<decltype(self)>(
            std::forward<decltype(self)>(self).base()[index]);
    }
    constexpr auto operator[](this auto &&self, Eigen::Index row, Eigen::Index col)
    {
        return detail::to_reference<decltype(self)>(
            std::forward<decltype(self)>(self).base()(row, col));
    }

    template <Quantity OtherQ, int R, int C>
        requires(_cols == R)
    constexpr auto operator*(const unit_mat<OtherQ, R, C> &other) const
    {
        using result_quantity = quantity<ref * OtherQ::reference, rep_type>;
        return unit_mat<result_quantity, _rows, C>{base() * other.base()};
    }

    constexpr auto operator*(Quantity auto scalar) const
    {
        constexpr auto scalar_ref = std::remove_cvref_t<decltype(scalar)>::reference;
        using result_quantity = quantity<ref * scalar_ref, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base() *
                                                       scalar.numerical_value_in(scalar_ref)};
    }

    constexpr auto operator/(Quantity auto scalar) const
    {
        constexpr auto scalar_ref = std::remove_cvref_t<decltype(scalar)>::reference;
        using result_quantity = quantity<ref / scalar_ref, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base() /
                                                       scalar.numerical_value_in(scalar_ref)};
    }

    constexpr auto operator*(auto scalar) const
        requires(std::is_arithmetic_v<decltype(scalar)>)
    {
        return unit_mat<decltype(Q{} * scalar), _rows, _cols>(base() * scalar);
    }

    constexpr auto operator/(auto scalar) const
        requires(std::is_arithmetic_v<decltype(scalar)>)
    {
        return unit_mat<decltype(Q{} / scalar), _rows, _cols>(base() / scalar);
    }

    constexpr auto operator+(const unit_mat &other) const
    {
        return unit_mat{base() + other.base()};
    }

    constexpr auto operator-(const unit_mat &other) const
    {
        return unit_mat{base() - other.base()};
    }

    constexpr auto &operator+=(const unit_mat &other)
    {
        base() += other.base();
        return *this;
    }

    constexpr auto &operator-=(const unit_mat &other)
    {
        base() -= other.base();
        return *this;
    }

    constexpr auto &operator*=(QuantityOf<dimensionless> auto scalar)
    {
        base() *= scalar.numerical_value_in(dimensionless);
        return *this;
    }

    constexpr auto &operator/=(QuantityOf<dimensionless> auto scalar)
    {
        base() /= scalar.numerical_value_in(dimensionless);
        return *this;
    }

    constexpr auto &operator*=(auto scalar)
        requires(std::is_arithmetic_v<decltype(scalar)>)
    {
        base() *= scalar;
        return *this;
    }

    constexpr auto &operator/=(auto scalar)
        requires(std::is_arithmetic_v<decltype(scalar)>)
    {
        base() /= scalar;
        return *this;
    }

    constexpr auto operator-() const { return unit_mat{-base()}; }

    constexpr static auto ones() { return unit_mat{eigen_type::Ones()}; }

    constexpr static auto zero() { return unit_mat{eigen_type::Zero()}; }

    constexpr static auto identity()
        requires(_rows == _cols)
    {
        return unit_mat{eigen_type::Identity()};
    }

    constexpr static auto random() { return unit_mat{eigen_type::Random()}; }

    constexpr static auto constant(Q value)
    {
        return unit_mat{eigen_type::Constant(value.numerical_value_in(ref))};
    }

    constexpr static auto constant(Eigen::Index rows, Eigen::Index cols, Q value)
    {
        return unit_mat{eigen_type::Constant(rows, cols, value.numerical_value_in(ref))};
    }

private:
    eigen_type data;
};

template <typename T, Quantity Q, int Rows, int Cols>
    requires(std::is_arithmetic_v<T> || Quantity<T>)
auto operator*(T scalar, const unit_mat<Q, Rows, Cols> &matrix)
{
    return matrix * scalar;
}

} // namespace physkit
