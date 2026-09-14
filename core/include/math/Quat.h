#ifndef MATH_QUAT_H
#define MATH_QUAT_H

#include <cmath>

#include <math/Vec3.h>

namespace core
{
namespace math
{

/// @brief Unit quaternion, Hamilton convention (w real, vector x,y,z).
/// @details A vector v is rotated by q' = q * [0, v] * q*.
struct Quat
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    /// @brief Default constructor. Produces the identity quaternion.
    Quat() = default;

    /// @brief Construct from individual components.
    /// @param w_ Real (scalar) part.
    /// @param x_ i component.
    /// @param y_ j component.
    /// @param z_ k component.
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    /// @brief Return the identity quaternion (no rotation).
    /// @return Identity quaternion {1, 0, 0, 0}.
    static Quat identity() { return Quat{1.0, 0.0, 0.0, 0.0}; }

    /// @brief Construct a unit quaternion from an axis-angle representation.
    /// @param axis Rotation axis (normalized internally).
    /// @param radians Rotation angle in radians.
    /// @return Quaternion representing the rotation. Returns identity if @p axis
    ///   has near-zero norm.
    static Quat fromAxisAngle(const Vec3& axis, double radians)
    {
        const double n = axis.norm();
        if (n < 1e-12)
            return identity();
        const Vec3 a = axis / n;
        const double half = 0.5 * radians;
        const double s = std::sin(half);
        return Quat{std::cos(half), a.x * s, a.y * s, a.z * s};
    }

    /// @brief Build an aerospace yaw-pitch-roll attitude quaternion (NED frame order).
    /// @details Rotations applied about Down, +East (body-y), +body-x
    ///   respectively, giving a NED -> body rotation.
    /// @param yaw Rotation about Down [rad].
    /// @param pitch Rotation about +East [rad].
    /// @param roll Rotation about +body-x [rad].
    /// @return Quaternion representing the combined NED -> body rotation.
    static Quat fromYawPitchRoll(double yaw, double pitch, double roll)
    {
        const Quat qz = fromAxisAngle(Vec3{0.0, 0.0, 1.0}, yaw);   // about Down
        const Quat qy = fromAxisAngle(Vec3{0.0, 1.0, 0.0}, pitch); // about +East
        const Quat qx = fromAxisAngle(Vec3{1.0, 0.0, 0.0}, roll);  // about +x
        return qx * (qy * qz);
    }

    /// @brief Squared norm.
    /// @return w^2 + x^2 + y^2 + z^2.
    double normSq() const { return w * w + x * x + y * y + z * z; }

    /// @brief Conjugate (negate the vector part).
    /// @return Conjugate quaternion.
    Quat conjugate() const { return Quat{w, -x, -y, -z}; }

    /// @brief Inverse (assumes non-zero norm).
    /// @return Inverse quaternion. Returns identity if normSq < 1e-16.
    Quat inverse() const
    {
        const double ns = normSq();
        if (ns < 1e-16)
            return identity();
        const Quat c = conjugate();
        return Quat{c.w / ns, c.x / ns, c.y / ns, c.z / ns};
    }

    /// @brief Hamilton product (quaternion multiplication).
    /// @param o Right-hand operand.
    /// @return Product quaternion (*this * o).
    Quat operator*(const Quat& o) const
    {
        return Quat{w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                    w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }

    /// @brief Rotate a vector by this quaternion (assumes unit length).
    /// @param v Vector to rotate.
    /// @return Rotated vector.
    Vec3 rotate(const Vec3& v) const
    {
        const Quat p{0.0, v.x, v.y, v.z};
        const Quat r = *this * p * conjugate();
        return Vec3{r.x, r.y, r.z};
    }
};

} // namespace math
} // namespace core

#endif // MATH_QUAT_H
