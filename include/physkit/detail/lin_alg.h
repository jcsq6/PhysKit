#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mp-units/framework.h>
#include <mp-units/math.h>
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

template <Quantity Q, int _rows, int _cols> class unit_mat
{
public:
    using value_type = Q;
    using rep_type = typename value_type::rep;
    using eigen_type = Eigen::Matrix<rep_type, _rows, _cols>;

    static constexpr auto ref = Q::reference;

    constexpr unit_mat() = default;
    constexpr unit_mat(const unit_mat &) = default;
    constexpr unit_mat &operator=(const unit_mat &) = default;
    constexpr unit_mat(unit_mat &&) = default;
    constexpr unit_mat &operator=(unit_mat &&) = default;
    constexpr ~unit_mat() = default;

    template <Quantity OtherQ>
        requires(equivalent(OtherQ::reference, Q::reference))
    constexpr unit_mat(const unit_mat<OtherQ, _rows, _cols> &other)
        : M_data(other.base().template cast<rep_type>())
    {
    }

    template <Quantity OtherQ>
        requires(equivalent(OtherQ::reference, Q::reference))
    constexpr unit_mat &operator=(const unit_mat<OtherQ, _rows, _cols> &other)
    {
        M_data = other.base().template cast<rep_type>();
        return *this;
    }

    constexpr unit_mat(const eigen_type &data) : M_data(data) {}
    constexpr unit_mat(eigen_type &&data) : M_data(std::move(data)) {}

    constexpr unit_mat(Eigen::Index dim) : M_data(dim) {}
    constexpr unit_mat(Eigen::Index rows, Eigen::Index cols) : M_data(rows, cols) {}

    constexpr unit_mat(const Q &scalar) : M_data(scalar.numerical_value_in(ref)) {}
    // constexpr unit_mat(const Q *data_ptr) : M_data(static_cast<rep_type *>(data_ptr)) {}

    constexpr unit_mat(const Q &x, const Q &y)
        : M_data(x.numerical_value_in(ref), y.numerical_value_in(ref))
    {
    }
    constexpr unit_mat(const Q &x, const Q &y, const Q &z)
        : M_data(x.numerical_value_in(ref), y.numerical_value_in(ref), z.numerical_value_in(ref))
    {
    }
    constexpr unit_mat(const Q &x, const Q &y, const Q &z, const Q &w)
        : M_data(x.numerical_value_in(ref), y.numerical_value_in(ref), z.numerical_value_in(ref),
                 w.numerical_value_in(ref))
    {
    }
    constexpr unit_mat(const std::initializer_list<std::initializer_list<Q>> &list)
        : M_data(list.size(), list.begin()->size())
    {
        Eigen::Index r = 0;
        for (const auto &row : list)
        {
            Eigen::Index c = 0;
            for (const auto &val : row)
            {
                M_data(r, c) = val.numerical_value_in(ref);
                ++c;
            }
            ++r;
        }
    }

    constexpr unit_mat &operator=(const eigen_type &other)
    {
        M_data = other;
        return *this;
    }

    constexpr unit_mat &operator=(eigen_type &&other)
    {
        M_data = std::move(other);
        return *this;
    }

    constexpr decltype(auto) base(this auto &&self)
    {
        return std::forward_like<decltype(self)>(self.M_data);
    }

    constexpr Q x() const { return M_data.x() * ref; }
    constexpr Q y() const { return M_data.y() * ref; }
    constexpr Q z() const { return M_data.z() * ref; }
    constexpr Q w() const { return M_data.w() * ref; }

    constexpr void x(Q val) { M_data.x() = val.numerical_value_in(ref); }
    constexpr void y(Q val) { M_data.y() = val.numerical_value_in(ref); }
    constexpr void z(Q val) { M_data.z() = val.numerical_value_in(ref); }
    constexpr void w(Q val) { M_data.w() = val.numerical_value_in(ref); }

    template <std::size_t I> constexpr Q get() const { return M_data[I] * ref; }

    constexpr auto rows() const { return base().rows(); }
    constexpr auto cols() const { return base().cols(); }

    constexpr auto begin(this auto &&self)
        requires(eigen_type::IsVectorAtCompileTime == 1)
    {
        return std::forward_like<decltype(self)>(self.M_data).begin();
    }
    constexpr auto end(this auto &&self)
        requires(eigen_type::IsVectorAtCompileTime == 1)
    {
        return std::forward_like<decltype(self)>(self.M_data).end();
    }

    auto transpose() const { return unit_mat<Q, _cols, _rows>{base().transpose()}; }
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

    constexpr Q operator[](Eigen::Index index) const { return M_data[index] * ref; }

    constexpr Q operator[](Eigen::Index row, Eigen::Index col) const
    {
        return M_data(row, col) * ref;
    }

    constexpr void set(Eigen::Index index, Q val) { M_data[index] = val.numerical_value_in(ref); }

    constexpr void set(Eigen::Index row, Eigen::Index col, Q val)
    {
        M_data(row, col) = val.numerical_value_in(ref);
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
    eigen_type M_data;
};

template <typename T, Quantity Q, int Rows, int Cols>
    requires(std::is_arithmetic_v<T> || Quantity<T>)
auto operator*(T scalar, const unit_mat<Q, Rows, Cols> &matrix)
{
    return matrix * scalar;
}

template <Quantity Q> class unit_quat
{
public:
    using value_type = Q;
    using rep_type = typename value_type::rep;
    using eigen_type = Eigen::Quaternion<rep_type>;

    static constexpr auto ref = Q::reference;

    constexpr unit_quat() = default;
    constexpr unit_quat(const unit_quat &) = default;
    constexpr unit_quat &operator=(const unit_quat &) = default;
    constexpr unit_quat(unit_quat &&) = default;
    constexpr unit_quat &operator=(unit_quat &&) = default;
    constexpr ~unit_quat() = default;

    template <Quantity OtherQ>
        requires(equivalent(OtherQ::reference, Q::reference))
    constexpr unit_quat(const unit_quat<OtherQ> &other)
        : M_data(other.base().coeffs().template cast<rep_type>())
    {
    }

    template <Quantity OtherQ>
        requires(equivalent(OtherQ::reference, Q::reference))
    constexpr unit_quat &operator=(const unit_quat<OtherQ> &other)
    {
        M_data = eigen_type(other.base().coeffs().template cast<rep_type>());
        return *this;
    }

    constexpr unit_quat(const eigen_type &data) : M_data(data) {}
    constexpr unit_quat(eigen_type &&data) : M_data(std::move(data)) {}

    constexpr unit_quat(const Q &w, const Q &x, const Q &y, const Q &z)
        : M_data(w.numerical_value_in(ref), x.numerical_value_in(ref), y.numerical_value_in(ref),
                 z.numerical_value_in(ref))
    {
    }

    constexpr unit_quat(const unit_mat<Q, 3, 3> &rotation_matrix) : M_data(rotation_matrix.base())
    {
    }

    // Eigen: w(), x(), y(), z()
    constexpr Q w() const { return M_data.w() * ref; }
    constexpr Q x() const { return M_data.x() * ref; }
    constexpr Q y() const { return M_data.y() * ref; }
    constexpr Q z() const { return M_data.z() * ref; }

    constexpr void w(Q val) { M_data.w() = val.numerical_value_in(ref); }
    constexpr void x(Q val) { M_data.x() = val.numerical_value_in(ref); }
    constexpr void y(Q val) { M_data.y() = val.numerical_value_in(ref); }
    constexpr void z(Q val) { M_data.z() = val.numerical_value_in(ref); }

    unit_mat<Q, 3, 1> vec() const { return unit_mat<Q, 3, 1>{M_data.vec()}; }

    unit_mat<Q, 4, 1> coeffs() const { return unit_mat<Q, 4, 1>{M_data.coeffs()}; }

    constexpr decltype(auto) base(this auto &&self)
    {
        return std::forward_like<decltype(self)>(self.M_data);
    }

    template <Quantity OtherQ> auto operator*(const unit_quat<OtherQ> &other) const
    {
        using result_quantity = quantity<ref * OtherQ::reference, rep_type>;
        return unit_quat<result_quantity>{
            typename unit_quat<result_quantity>::eigen_type{M_data * other.base()}};
    }

    template <Quantity OtherQ>
        requires(equivalent(ref *OtherQ::reference, ref))
    auto &operator*=(const unit_quat<OtherQ> &other)
    {
        M_data *= other.base();
        return *this;
    }

    template <Quantity VecQ>
    auto operator*(const unit_mat<VecQ, 3, 1> &v) const
        requires(QuantityOf<Q, dimensionless>)
    {
        return unit_mat<VecQ, 3, 1>{M_data._transformVector(v.base())};
    }

    auto conjugate() const { return unit_quat{M_data.conjugate()}; }

    auto inverse() const
    {
        return unit_quat<quantity<mp_units::one / ref, rep_type>>{M_data.inverse()};
    }

    auto norm() const { return M_data.norm() * ref; }

    auto squared_norm() const { return M_data.squaredNorm() * ref * ref; }

    void normalize()
        requires(QuantityOf<Q, dimensionless>)
    {
        M_data.normalize();
    }

    auto normalized() const
    {
        using result_quantity = quantity<ref / ref, rep_type>;
        return unit_quat<result_quantity>{M_data.normalized()};
    }

    template <Quantity OtherQ> auto dot(const unit_quat<OtherQ> &other) const
    {
        return M_data.dot(other.base()) * ref * OtherQ::reference;
    }

    auto angular_distance(const unit_quat &other) const
        requires(QuantityOf<Q, dimensionless>)
    {
        return M_data.angularDistance(other.M_data) * si::radian;
    }

    auto to_rotation_matrix() const
        requires(QuantityOf<Q, dimensionless>)
    {
        return unit_mat<Q, 3, 3>{M_data.toRotationMatrix()};
    }

    auto slerp(rep_type t, const unit_quat &other) const
        requires(QuantityOf<Q, dimensionless>)
    {
        return unit_quat{M_data.slerp(t, other.M_data)};
    }

    auto &set_identity()
        requires(QuantityOf<Q, dimensionless>)
    {
        M_data.setIdentity();
        return *this;
    }

    template <Quantity VecQ1, Quantity VecQ2>
    auto &set_from_two_vectors(const unit_mat<VecQ1, 3, 1> &a, const unit_mat<VecQ2, 3, 1> &b)
        requires(QuantityOf<Q, dimensionless>)
    {
        M_data.setFromTwoVectors(a.base(), b.base());
        return *this;
    }

    bool is_approx(const unit_quat &other,
                   rep_type prec = Eigen::NumTraits<rep_type>::dummy_precision()) const
    {
        return M_data.isApprox(other.M_data, prec);
    }

    constexpr bool operator==(const unit_quat &other) const { return base() == other.base(); }

    constexpr bool operator!=(const unit_quat &other) const { return !(*this == other); }

    constexpr static auto identity()
        requires(QuantityOf<Q, dimensionless>)
    {
        return unit_quat{eigen_type::Identity()};
    }

    static auto unit_random()
        requires(QuantityOf<Q, dimensionless>)
    {
        return unit_quat{eigen_type::UnitRandom()};
    }

    template <Quantity AngleQ>
    static unit_quat from_angle_axis(const AngleQ &angle, const unit_mat<Q, 3, 1> &axis)
        requires(QuantityOf<Q, dimensionless>)
    {
        Eigen::AngleAxis<rep_type> aa(static_cast<rep_type>(angle.numerical_value_in(si::radian)),
                                      axis.base());
        return unit_quat{eigen_type{aa}};
    }

    template <Quantity VecQ1, Quantity VecQ2>
    static unit_quat from_two_vectors(const unit_mat<VecQ1, 3, 1> &a,
                                      const unit_mat<VecQ2, 3, 1> &b)
        requires(QuantityOf<Q, dimensionless>)
    {
        return unit_quat{eigen_type::FromTwoVectors(a.base(), b.base())};
    }

private:
    eigen_type M_data{rep_type(1), rep_type(0), rep_type(0), rep_type(0)};
};

} // namespace physkit

namespace std
{
template <physkit::Quantity Q, int Rows, int Cols>
struct tuple_size<physkit::unit_mat<Q, Rows, Cols>> : std::integral_constant<std::size_t, Rows>
{
};

template <std::size_t I, physkit::Quantity Q, int Rows, int Cols>
struct tuple_element<I, physkit::unit_mat<Q, Rows, Cols>>
{
    using type = Q;
};
} // namespace std
