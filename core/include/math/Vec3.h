#ifndef MATH_VEC3_H
#define MATH_VEC3_H

#include <cmath>

namespace core
{
namespace math
{

/// @brief Minimal 3D vector, dependency-free so the project always builds.
/// @note Swap in a full linear-algebra library (e.g. Eigen) later if/when the
///   math grows.
struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    /// @brief Default constructor. Zero-initializes all components.
    Vec3() = default;

    /// @brief Construct from individual components.
    /// @param x_ X component.
    /// @param y_ Y component.
    /// @param z_ Z component.
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    /// @brief Vector addition.
    /// @param o Vector to add.
    /// @return Resulting vector.
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }

    /// @brief Vector subtraction.
    /// @param o Vector to subtract.
    /// @return Resulting vector.
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }

    /// @brief Scalar multiplication.
    /// @param s Scalar value.
    /// @return Scaled vector.
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }

    /// @brief Scalar division.
    /// @param s Scalar value (must be non-zero).
    /// @return Scaled vector.
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    /// @brief In-place vector addition.
    /// @param o Vector to add.
    /// @return Reference to this vector.
    Vec3& operator+=(const Vec3& o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    /// @brief In-place vector subtraction.
    /// @param o Vector to subtract.
    /// @return Reference to this vector.
    Vec3& operator-=(const Vec3& o)
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    /// @brief Dot product.
    /// @param o Other vector.
    /// @return Scalar dot product.
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    /// @brief Cross product.
    /// @param o Other vector.
    /// @return Vector perpendicular to both inputs.
    Vec3 cross(const Vec3& o) const
    {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    /// @brief Squared Euclidean norm (avoids a sqrt).
    /// @return Squared magnitude.
    double normSq() const { return dot(*this); }

    /// @brief Euclidean norm (magnitude).
    /// @return Vector length.
    double norm() const { return std::sqrt(normSq()); }

    /// @brief Return a unit-length copy of this vector.
    /// @return Normalized vector, or the zero vector if norm < 1e-12.
    Vec3 normalized() const
    {
        const double n = norm();
        return n > 1e-12 ? *this / n : Vec3{0.0, 0.0, 0.0};
    }

    /// @brief Component-wise approximate equality.
    /// @param o Other vector.
    /// @param tol Per-component tolerance (default 1e-9).
    /// @return True if every component differs by at most @p tol.
    bool isApprox(const Vec3& o, double tol = 1e-9) const
    {
        return std::fabs(x - o.x) <= tol && std::fabs(y - o.y) <= tol && std::fabs(z - o.z) <= tol;
    }
};

/// @brief Scalar-first multiplication (commutative).
/// @param s Scalar value.
/// @param v Vector to scale.
/// @return Scaled vector.
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

} // namespace math
} // namespace core

#endif // MATH_VEC3_H
