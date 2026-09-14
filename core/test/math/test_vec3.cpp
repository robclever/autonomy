#include <cassert>
#include <cmath>

#include <math/Vec3.h>

namespace
{
constexpr double kTol = 1e-9;

void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
void expectNear(const core::math::Vec3& a, const core::math::Vec3& b, double tol = kTol)
{
    expectNear(a.x, b.x, tol);
    expectNear(a.y, b.y, tol);
    expectNear(a.z, b.z, tol);
}
} // namespace

// Unit tests for math/Vec3.
int main()
{
    using core::math::Vec3;

    // --- Construction ---
    {
        Vec3 v; // default
        expectNear(v.x, 0.0);
        expectNear(v.y, 0.0);
        expectNear(v.z, 0.0);
    }
    {
        Vec3 v(1.0, 2.0, 3.0);
        expectNear(v.x, 1.0);
        expectNear(v.y, 2.0);
        expectNear(v.z, 3.0);
    }

    // --- Addition ---
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(4.0, 5.0, 6.0);
        Vec3 c = a + b;
        expectNear(c, Vec3(5.0, 7.0, 9.0));
    }

    // --- Subtraction ---
    {
        Vec3 a(4.0, 5.0, 6.0);
        Vec3 b(1.0, 2.0, 3.0);
        Vec3 c = a - b;
        expectNear(c, Vec3(3.0, 3.0, 3.0));
    }

    // --- Scalar multiplication (member) ---
    {
        Vec3 v(1.0, -2.0, 3.0);
        Vec3 r = v * 2.0;
        expectNear(r, Vec3(2.0, -4.0, 6.0));
    }

    // --- Scalar multiplication (free function, reverse order) ---
    {
        Vec3 v(1.0, -2.0, 3.0);
        Vec3 r = 3.0 * v;
        expectNear(r, Vec3(3.0, -6.0, 9.0));
    }

    // --- Scalar division ---
    {
        Vec3 v(6.0, -4.0, 2.0);
        Vec3 r = v / 2.0;
        expectNear(r, Vec3(3.0, -2.0, 1.0));
    }

    // --- Compound addition ---
    {
        Vec3 v(1.0, 2.0, 3.0);
        v += Vec3(10.0, 20.0, 30.0);
        expectNear(v, Vec3(11.0, 22.0, 33.0));
    }

    // --- Compound subtraction ---
    {
        Vec3 v(10.0, 20.0, 30.0);
        v -= Vec3(1.0, 2.0, 3.0);
        expectNear(v, Vec3(9.0, 18.0, 27.0));
    }

    // --- Dot product ---
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(4.0, 5.0, 6.0);
        expectNear(a.dot(b), 32.0); // 1*4 + 2*5 + 3*6
    }
    {
        // Orthogonal vectors have zero dot product.
        Vec3 x(1.0, 0.0, 0.0);
        Vec3 y(0.0, 1.0, 0.0);
        expectNear(x.dot(y), 0.0);
    }

    // --- Cross product ---
    {
        Vec3 x(1.0, 0.0, 0.0);
        Vec3 y(0.0, 1.0, 0.0);
        Vec3 z = x.cross(y);
        expectNear(z, Vec3(0.0, 0.0, 1.0));
    }
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(4.0, 5.0, 6.0);
        // a x b = (2*6-3*5, 3*4-1*6, 1*5-2*4) = (-3, 6, -3)
        expectNear(a.cross(b), Vec3(-3.0, 6.0, -3.0));
    }
    {
        // Cross product is anti-commutative: a x b == -(b x a).
        Vec3 a(1.0, -2.0, 3.0);
        Vec3 b(-4.0, 5.0, -6.0);
        expectNear(a.cross(b), b.cross(a) * -1.0);
    }

    // --- normSq ---
    {
        Vec3 v(3.0, 4.0, 0.0);
        expectNear(v.normSq(), 25.0);
    }

    // --- norm ---
    {
        Vec3 v(3.0, 4.0, 0.0);
        expectNear(v.norm(), 5.0);
    }

    // --- normalized ---
    {
        Vec3 v(0.0, 0.0, 5.0);
        Vec3 n = v.normalized();
        expectNear(n, Vec3(0.0, 0.0, 1.0));
        expectNear(n.norm(), 1.0);
    }
    {
        // Zero vector normalizes to zero.
        Vec3 v(0.0, 0.0, 0.0);
        Vec3 n = v.normalized();
        expectNear(n, Vec3(0.0, 0.0, 0.0));
    }

    // --- isApprox ---
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(1.0, 2.0, 3.0);
        assert(a.isApprox(b));
    }
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(1.0 + 1e-10, 2.0, 3.0);
        assert(a.isApprox(b)); // within default tolerance
    }
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(1.0 + 1e-6, 2.0, 3.0);
        assert(!a.isApprox(b)); // outside default tolerance
    }
    {
        Vec3 a(1.0, 2.0, 3.0);
        Vec3 b(1.0 + 1e-6, 2.0, 3.0);
        assert(a.isApprox(b, 1e-4)); // within custom tolerance
    }

    return 0;
}
