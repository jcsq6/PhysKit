#pragma once
#include <Eigen/Dense>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/NumTraits.h>
#include <Eigen/src/Core/util/Meta.h>

#include <mp-units/framework.h>
#include <mp-units/systems/international.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/unit_symbols.h>

#include <mp-units/bits/hacks.h>
#include <mp-units/systems/si/units.h>
#include <type_traits>
#include <utility>

#include <initializer_list>

#include <Eigen/Core>

namespace physkit
{

using namespace mp_units;

template <Quantity Q, int _rows, int _cols>
class unit_mat
{
public:
    using value_type = Q;
    using rep_type = typename value_type::rep;
    using eigen_type = Eigen::Matrix<rep_type, _rows, _cols>;

    static constexpr auto ref = Q::reference;

    constexpr unit_mat() = default;
    constexpr unit_mat(const unit_mat&) = default;
    constexpr unit_mat& operator=(const unit_mat&) = default;
    constexpr unit_mat(unit_mat&&) = default;
    constexpr unit_mat& operator=(unit_mat&&) = default;
    constexpr ~unit_mat() = default;

    constexpr unit_mat(const eigen_type& data) : data(data) {}
    constexpr unit_mat(eigen_type&& data) : data(std::move(data)) {}

    constexpr unit_mat(Eigen::Index dim) : data(dim) {}
    constexpr unit_mat(Eigen::Index rows, Eigen::Index cols) : data(rows, cols) {}

    constexpr unit_mat(const Q &scalar) : data(scalar.numerical_value_in(ref)) {}
    constexpr unit_mat(const Q *data_ptr) : data(static_cast<rep_type*>(data_ptr)) {}

    constexpr unit_mat(const Q &x, const Q &y) : data(x.numerical_value_in(ref), y.numerical_value_in(ref)) {}
    constexpr unit_mat(const Q &x, const Q &y, const Q &z) : data(x.numerical_value_in(ref), y.numerical_value_in(ref), z.numerical_value_in(ref)) {}
    constexpr unit_mat(const Q &x, const Q &y, const Q &z, const Q &w) : data(x.numerical_value_in(ref), y.numerical_value_in(ref), z.numerical_value_in(ref), w.numerical_value_in(ref)) {}
    constexpr unit_mat(const std::initializer_list<std::initializer_list<Q>> &list) : data(list.size(), list.begin()->size())
    {
        Eigen::Index r = 0;
        for (const auto& row : list)
        {
            Eigen::Index c = 0;
            for (const auto& val : row)
            {
                data(r, c) = val.numerical_value_in(ref);
                ++c;
            }
            ++r;
        }
    }

    constexpr unit_mat &operator=(const eigen_type& other)
    {
        data = other;
        return *this;
    }

    constexpr unit_mat &operator=(eigen_type&& other)
    {
        data = std::move(other);
        return *this;
    }

    constexpr decltype(auto) base(this auto &&self) { return std::forward<decltype(self)>(self).data; }
    
    constexpr auto& x() { return base().x(); }
    constexpr auto& y() { return base().y(); }
    constexpr auto& z() { return base().z(); }
    constexpr auto& w() { return base().w(); }

    constexpr auto rows() const { return base().rows(); }
    constexpr auto cols() const { return base().cols(); }

    constexpr auto begin(this auto &&self) { return std::forward<decltype(self)>(self).data.begin(); }
    constexpr auto end(this auto &&self) { return std::forward<decltype(self)>(self).data.end(); }

    auto transpose() const { return unit_mat<Q, _cols, _rows>{base().transpose()}; }
    auto determinant() const { return base().determinant(); }
    auto inverse() const { return unit_mat<Q, _rows, _cols>{base().inverse()}; }
    auto trace() const { return base().trace(); }

    template <Quantity OtherQ>
    auto dot(const unit_mat<OtherQ, _rows, _cols>& other) const
    {
        using result_quantity = quantity<Q::reference * OtherQ::reference, rep_type>;
        return result_quantity(base().dot(other.base()));
    }

    template <Quantity OtherQ>
    auto cross(const unit_mat<OtherQ, _rows, _cols>& other) const
    {
        using result_quantity = quantity<Q::reference * OtherQ::reference, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base().cross(other.base())};
    }

    auto squared_norm() const
    {
        using result_quantity = quantity<Q::reference * Q::reference, rep_type>;
        return result_quantity(base().squaredNorm());
    }

    auto norm() const
    {
        return Q(base().norm());
    }

    auto normalized() const
    {
        using result_quantity = quantity<dimensionless, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base().normalized()};
    }

    constexpr bool operator==(const unit_mat& other) const = default;
    constexpr bool operator!=(const unit_mat& other) const = default;

    constexpr decltype(auto) operator[](this auto &&self, Eigen::Index index) { return std::forward<decltype(self)>(self).base()[index]; }
    constexpr decltype(auto) operator[](this auto &&self, Eigen::Index row, Eigen::Index col) { return std::forward<decltype(self)>(self).base()(row, col); }

    template <Quantity OtherQ, int R, int C>
    constexpr auto operator*(const unit_mat<OtherQ, R, C>& other) const
    {
        using result_quantity = quantity<Q::reference * OtherQ::reference, rep_type>;
        return unit_mat<result_quantity, _rows, C>{base() * other.base()};
    }

    constexpr auto operator*(const Quantity auto& scalar) const
    {
        constexpr auto scalar_ref = std::remove_cvref_t<decltype(scalar)>::reference;
        using result_quantity = quantity<Q::reference * scalar_ref, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base() * scalar.numerical_value_in(scalar_ref)};
    }

    constexpr auto operator/(const Quantity auto& scalar) const
    {
        constexpr auto scalar_ref = std::remove_cvref_t<decltype(scalar)>::reference;
        using result_quantity = quantity<Q::reference / scalar_ref, rep_type>;
        return unit_mat<result_quantity, _rows, _cols>{base() / scalar.numerical_value_in(scalar_ref)};
    }

    constexpr auto operator+(const unit_mat& other) const
    {
        return unit_mat{base() + other.base()};
    }

    constexpr auto operator-(const unit_mat& other) const
    {
        return unit_mat{base() - other.base()};
    }

    constexpr auto &operator+=(const unit_mat& other)
    {
        base() += other.base();
        return *this;
    }

    constexpr auto &operator-=(const unit_mat& other)
    {
        base() -= other.base();
        return *this;
    }

    constexpr auto &operator*=(const QuantityOf<dimensionless> auto& scalar)
    {
        base() *= scalar.numerical_value_in(dimensionless);
        return *this;
    }

    constexpr auto &operator/=(const QuantityOf<dimensionless> auto& scalar)
    {
        base() /= scalar.numerical_value_in(dimensionless);
        return *this;
    }

    constexpr auto operator-() const
    {
        return unit_mat{-base()};
        Eigen::Vector3d v;
    }

private:
    eigen_type data;
};
}
