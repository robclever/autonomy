#include <cassert>
#include <cmath>

#include <math/Quat.h>
#include <math/Vec3.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
void expectNear(const core::math::Vec3& a, const core::math::Vec3& b, double tol = kTol)
{
    expectNear(a.x, b.x, tol);
    expectNear(a.y, b.y, tol);
    expectNear(a.z, b.z, tol);
}
void expectNear(const core::math::Quat& a, const core::math::Quat& b, double tol = kTol)
{
    expectNear(a.w, b.w, tol);
    expectNear(a.x, b.x, tol);
    expectNear(a.y, b.y, tol);
    expectNear(a.z, b.z, tol);
}
} // namespace

/// @brief Unit tests for math/Quat.
int main()
{
    using core::math::Quat;
    using core::math::Vec3;

    /// @brief --- Construction ---
    {
        Quat q; // default
        expectNear(q.w, 1.0);
        expectNear(q.x, 0.0);
        expectNear(q.y, 0.0);
        expectNear(q.z, 0.0);
    }
    {
        Quat q(0.5, 0.5, 0.5, 0.5);
        expectNear(q.w, 0.5);
        expectNear(q.x, 0.5);
        expectNear(q.y, 0.5);
        expectNear(q.z, 0.5);
    }

    /// @brief --- identity() ---
    {
        Quat q = Quat::identity();
        expectNear(q, Quat(1.0, 0.0, 0.0, 0.0));
    }

    /// @brief --- fromAxisAngle ---
    {
        /// @brief 90 deg about +Z maps +X to +Y.
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.5 * kPi);
        expectNear(q.rotate(Vec3{1.0, 0.0, 0.0}), Vec3{0.0, 1.0, 0.0});
    }
    {
        /// @brief 90 deg about +Y maps +Z to +X.
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 1.0, 0.0}, 0.5 * kPi);
        expectNear(q.rotate(Vec3{0.0, 0.0, 1.0}), Vec3{1.0, 0.0, 0.0});
    }
    {
        /// @brief 90 deg about +X maps +Y to +Z.
        Quat q = Quat::fromAxisAngle(Vec3{1.0, 0.0, 0.0}, 0.5 * kPi);
        expectNear(q.rotate(Vec3{0.0, 1.0, 0.0}), Vec3{0.0, 0.0, 1.0});
    }
    {
        /// @brief Zero rotation angle gives identity.
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.0);
        expectNear(q, Quat::identity());
    }
    {
        /// @brief Zero-length axis returns identity (degenerate case).
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 0.0, 0.0}, 0.5 * kPi);
        expectNear(q, Quat::identity());
    }
    {
        /// @brief Non-unit axis is normalized internally.
        Quat qUnit = Quat::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.3);
        Quat qScaled = Quat::fromAxisAngle(Vec3{0.0, 0.0, 5.0}, 0.3);
        expectNear(qUnit, qScaled);
    }

    /// @brief --- fromYawPitchRoll ---
    {
        /// @brief Yaw 90 deg (about Down) maps North (+X) to East (+Y).
        Quat q = Quat::fromYawPitchRoll(0.5 * kPi, 0.0, 0.0);
        expectNear(q.rotate(Vec3{1.0, 0.0, 0.0}), Vec3{0.0, 1.0, 0.0});
    }
    {
        /// @brief Zero angles give identity.
        Quat q = Quat::fromYawPitchRoll(0.0, 0.0, 0.0);
        expectNear(q, Quat::identity());
    }

    /// @brief --- normSq ---
    {
        Quat q(1.0, 2.0, 3.0, 4.0);
        expectNear(q.normSq(), 30.0); // 1+4+9+16
    }
    {
        /// @brief Unit quaternion has normSq == 1.
        Quat q = Quat::fromAxisAngle(Vec3{1.0, 0.0, 0.0}, 0.7);
        expectNear(q.normSq(), 1.0, 1e-12);
    }

    /// @brief --- conjugate ---
    {
        Quat q(1.0, 2.0, 3.0, 4.0);
        Quat c = q.conjugate();
        expectNear(c, Quat(1.0, -2.0, -3.0, -4.0));
    }

    /// @brief --- inverse ---
    {
        /// @brief For a unit quaternion, inverse == conjugate.
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.5 * kPi);
        expectNear(q.inverse(), q.conjugate());
    }
    {
        /// @brief q * q.inverse() == identity.
        Quat q(1.0, 2.0, 3.0, 4.0);
        Quat product = q * q.inverse();
        expectNear(product, Quat::identity());
    }
    {
        /// @brief Zero quaternion inverse returns identity (degenerate case).
        Quat q(0.0, 0.0, 0.0, 0.0);
        expectNear(q.inverse(), Quat::identity());
    }

    /// @brief --- operator* (Hamilton product) ---
    {
        /// @brief Identity * q == q.
        Quat q = Quat::fromAxisAngle(Vec3{1.0, 0.0, 0.0}, 0.5);
        expectNear(Quat::identity() * q, q);
        expectNear(q * Quat::identity(), q);
    }
    {
        /// @brief Two 90-degree yaw rotations compose to 180-degree yaw.
        Quat q90 = Quat::fromYawPitchRoll(0.5 * kPi, 0.0, 0.0);
        Quat q180 = q90 * q90;
        expectNear(q180.rotate(Vec3{1.0, 0.0, 0.0}), Vec3{-1.0, 0.0, 0.0});
    }

    /// @brief --- rotate ---
    {
        /// @brief Identity rotation leaves vector unchanged.
        Quat q = Quat::identity();
        Vec3 v(1.0, 2.0, 3.0);
        expectNear(q.rotate(v), v);
    }
    {
        /// @brief 360-degree rotation leaves vector unchanged.
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, 2.0 * kPi);
        Vec3 v(1.0, 2.0, 3.0);
        expectNear(q.rotate(v), v, 1e-12);
    }
    {
        /// @brief Rotation preserves vector norm.
        Quat q = Quat::fromAxisAngle(Vec3{1.0, 1.0, 1.0}, 0.7);
        Vec3 v(3.0, -4.0, 5.0);
        expectNear(q.rotate(v).norm(), v.norm(), 1e-12);
    }
    {
        /// @brief Conjugate rotates in the opposite direction.
        Quat q = Quat::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.5 * kPi);
        Vec3 v(1.0, 0.0, 0.0);
        expectNear(q.conjugate().rotate(v), Vec3{0.0, -1.0, 0.0});
    }

    return 0;
}
