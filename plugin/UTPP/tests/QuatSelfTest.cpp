// Offline self-test for UTPP quaternion math (not shipped). Exit code 0 = all passed.
#include <cmath>
#include <cstdio>

#include "../src/QuatMath.hpp"

using namespace utpp::quat;

static int g_fail = 0;

static bool Near(float a, float b, float eps = 1e-5f)
{
    return std::fabs(a - b) <= eps;
}

static void ExpectQ(const char* aName, const float* q, float x, float y, float z, float w)
{
    const bool ok = Near(q[0], x) && Near(q[1], y) && Near(q[2], z) && Near(q[3], w);
    std::printf("[%s] %s: got (%.6f, %.6f, %.6f, %.6f) expected (%.6f, %.6f, %.6f, %.6f)\n", ok ? "PASS" : "FAIL",
                aName, q[0], q[1], q[2], q[3], x, y, z, w);
    if (!ok)
        ++g_fail;
}

static void ExpectTrue(const char* aName, bool v)
{
    std::printf("[%s] %s\n", v ? "PASS" : "FAIL", aName);
    if (!v)
        ++g_fail;
}

// Rotate vector v by unit quaternion q: q * (v,0) * conj(q).
static void Rotate(const Q& q, const float* v, float* out)
{
    const Q p{v[0], v[1], v[2], 0.0f};
    const Q c{-q.x, -q.y, -q.z, q.w};
    const Q r = Mul(Mul(q, p), c);
    out[0] = r.x;
    out[1] = r.y;
    out[2] = r.z;
}

int main()
{
    const float h = std::sqrt(0.5f);

    float q[4] = {0, 0, 0, 1};
    ExpectTrue("identity * 90deg Y returns true", ApplyLocalRotation(q, 1, 90.0f));
    ExpectQ("identity * 90deg Y", q, 0, h, 0, h);

    float q2[4] = {0, 0, 0, 1};
    ApplyLocalRotation(q2, 0, 90.0f);
    ExpectQ("identity * 90deg X", q2, h, 0, 0, h);
    ApplyLocalRotation(q2, 1, 90.0f);
    ExpectQ("qx90 * qy90 (local Y after X)", q2, 0.5f, 0.5f, 0.5f, 0.5f);

    float q3[4] = {0, 0, 0, 1};
    ApplyLocalRotation(q3, 1, 90.0f);
    ApplyLocalRotation(q3, 0, 90.0f);
    ExpectQ("qy90 * qx90 (order matters)", q3, 0.5f, 0.5f, -0.5f, 0.5f);

    float q4[4] = {0, 0, 0, 1};
    ApplyLocalRotation(q4, 2, -90.0f);
    ExpectQ("identity * -90deg Z", q4, 0, 0, -h, h);

    // Vector check: 90deg about Y maps +X to -Z.
    {
        Q r;
        AxisAngleDeg(1, 90.0f, r);
        const float v[3] = {1, 0, 0};
        float o[3];
        Rotate(r, v, o);
        ExpectTrue("90deg Y rotates +X to -Z", Near(o[0], 0) && Near(o[1], 0) && Near(o[2], -1));
    }

    // Normalization of a non-unit input.
    float q5[4] = {0, 0, 0, 2};
    ExpectTrue("non-unit input normalizes", ApplyLocalRotation(q5, 1, 90.0f));
    ExpectQ("(0,0,0,2) * 90deg Y normalized", q5, 0, h, 0, h);
    const float n = std::sqrt(q5[0] * q5[0] + q5[1] * q5[1] + q5[2] * q5[2] + q5[3] * q5[3]);
    ExpectTrue("result unit length", Near(n, 1.0f));

    // 360 deg -> -identity (same rotation).
    float q6[4] = {0, 0, 0, 1};
    ApplyLocalRotation(q6, 1, 360.0f);
    ExpectQ("identity * 360deg Y = -identity", q6, 0, 0, 0, -1);

    // Rejections leave memory untouched.
    float z[4] = {0, 0, 0, 0};
    ExpectTrue("zero quaternion rejected", !ApplyLocalRotation(z, 1, 30.0f));
    ExpectQ("zero quaternion untouched", z, 0, 0, 0, 0);
    float q7[4] = {0, 0, 0, 1};
    ExpectTrue("invalid axis rejected", !ApplyLocalRotation(q7, 3, 30.0f));
    ExpectTrue("NaN angle rejected", !ApplyLocalRotation(q7, 1, std::nanf("")));
    ExpectTrue("Inf angle rejected", !ApplyLocalRotation(q7, 1, INFINITY));
    ExpectQ("rejected input untouched", q7, 0, 0, 0, 1);
    float qn[4] = {std::nanf(""), 0, 0, 1};
    ExpectTrue("NaN quaternion rejected", !ApplyLocalRotation(qn, 1, 30.0f));

    std::printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
