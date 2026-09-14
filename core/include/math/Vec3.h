#ifndef MATH_VEC3_H
#define MATH_VEC3_H

#include <cmath>

namespace core
{
namespace math
{

// Minimal 3D vector, dependency-free so the project always builds. Swap in a
// full linear-algebra library (e.g. Eigen) later if/when the math grows.
struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& o)
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const
    {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    double normSq() const { return dot(*this); }
    double norm() const { return std::sqrt(normSq()); }
    Vec3 normalized() const
    {
        const double n = norm();
        return n > 1e-12 ? *this / n : Vec3{0.0, 0.0, 0.0};
    }

    bool isApprox(const Vec3& o, double tol = 1e-9) const
    {
        return std::fabs(x - o.x) <= tol && std::fabs(y - o.y) <= tol && std::fabs(z - o.z) <= tol;
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

} // namespace math
} // namespace core

#endif // MATH_VEC3_H
