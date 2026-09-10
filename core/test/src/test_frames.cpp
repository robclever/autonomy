#include <cassert>
#include <cmath>

#include <core/time/Frames.h>
#include <core/math/Quat.h>
#include <core/math/Vec3.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
void expectNear(const slam::math::Vec3& a, const slam::math::Vec3& b) {
    expectNear(a.x, b.x);
    expectNear(a.y, b.y);
    expectNear(a.z, b.z);
}
} // namespace

// NED/body frame and quaternion sanity checks.
int main() {
    using slam::math::Quat;
    using slam::math::Vec3;

    // Yaw 90 deg (north -> east) and back.
    const Quat q = Quat::fromYawPitchRoll(0.5 * kPi, 0.0, 0.0);
    expectNear(q.rotate(Vec3{1.0, 0.0, 0.0}), Vec3{0.0, 1.0, 0.0});
    expectNear(q.conjugate().rotate(Vec3{0.0, 1.0, 0.0}), Vec3{1.0, 0.0, 0.0});

    // Pitch up tilts the forward axis above the (down-positive) horizon.
    const Quat qPitch = Quat::fromYawPitchRoll(0.0, 0.3, 0.0);
    const Vec3 fwd = qPitch.rotate(Vec3{1.0, 0.0, 0.0});
    expectNear(std::asin(-fwd.z), 0.3, 1e-6);

    // Body <-> NED round-trip through a Transform.
    const slam::Pose pose{Vec3{10.0, -20.0, -500.0}, Quat::fromYawPitchRoll(0.7, -0.2, 0.1)};
    const slam::Transform nedToBody = slam::Transform::nedToBody(pose);
    const Vec3 pNed{1000.0, 30.0, -450.0};
    expectNear(nedToBody.applyInverseToPoint(nedToBody.applyToPoint(pNed)), pNed);

    // Forward-inverse transform consistency.
    const slam::Transform inv = nedToBody.inverse();
    expectNear(inv.applyToPoint(nedToBody.applyToPoint(pNed)), pNed);

    // A point directly ahead of the aircraft lands at azimuth 0 in body.
    const Vec3 aheadNed = pose.position + pose.orientation.conjugate().rotate(Vec3{500.0, 0.0, 0.0});
    const Vec3 aheadBody = nedToBody.applyToPoint(aheadNed);
    expectNear(std::atan2(aheadBody.y, aheadBody.x), 0.0, 1e-9);

    return 0;
}