#pragma once

#include <cmath>

#include "core/math/Vec3.h"

namespace slam {
namespace math {

// Unit quaternion, Hamilton convention (w real, vector x,y,z). A vector v is
// rotated by  q' = q * [0, v] * q*.
struct Quat
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Quat() = default;
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat identity() { return Quat{1.0, 0.0, 0.0, 0.0}; }

    static Quat fromAxisAngle(const Vec3& axis, double radians) {
        const double n = axis.norm();
        if (n < 1e-12)
            return identity();
        const Vec3 a = axis / n;
        const double half = 0.5 * radians;
        const double s = std::sin(half);
        return Quat{std::cos(half), a.x * s, a.y * s, a.z * s};
    }

    // Standard aerospace yaw-pitch-roll attitude built in NED frame order
    // (rotations about Down, +East/body-y, +body-x respectively), giving a
    // NED -> body rotation.
    static Quat fromYawPitchRoll(double yaw, double pitch, double roll) {
        const Quat qz = fromAxisAngle(Vec3{0.0, 0.0, 1.0}, yaw);   // about Down
        const Quat qy = fromAxisAngle(Vec3{0.0, 1.0, 0.0}, pitch); // about +East
        const Quat qx = fromAxisAngle(Vec3{1.0, 0.0, 0.0}, roll);  // about +x
        return qx * (qy * qz);
    }

    double normSq() const { return w * w + x * x + y * y + z * z; }
    Quat conjugate() const { return Quat{w, -x, -y, -z}; }
    Quat inverse() const {
        const double ns = normSq();
        if (ns < 1e-16)
            return identity();
        const Quat c = conjugate();
        return Quat{c.w / ns, c.x / ns, c.y / ns, c.z / ns};
    }

    Quat operator*(const Quat& o) const {
        return Quat{w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                    w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }

    // Rotate vector v by this quaternion (assumes unit length).
    Vec3 rotate(const Vec3& v) const {
        const Quat p{0.0, v.x, v.y, v.z};
        const Quat r = *this * p * conjugate();
        return Vec3{r.x, r.y, r.z};
    }
};

} // namespace math
} // namespace slam