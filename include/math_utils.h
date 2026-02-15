#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <algorithm>

#include "skse64/NiObjects.h"

#include "RE/havok.h"
#include "RE/offsets.h"


static const float g_minAllowedFingerAngle = 0;// 5 * 0.0174533; // 5 degrees

struct Triangle
{
    UInt16 vertexIndices[3];
};
static_assert(sizeof(Triangle) == 0x06);

struct Intersection
{
    BSTriShape *node; // the trishape where the intersected triangle resides
    Triangle tri; // triangle that was intersected
    float angle; // angle of the fingertip at intersection pt
};

struct Point2
{
    float x;
    float y;

    Point2();
    Point2(float X, float Y) : x(X), y(Y) { };

    Point2 Point2::operator- () const;
    Point2 Point2::operator+ (const Point2 &pt) const;

    Point2 Point2::operator- (const Point2 &pt) const;

    Point2 &Point2::operator+= (const Point2 &pt);
    Point2 &Point2::operator-= (const Point2 &pt);

    // Scalar operations
    Point2 Point2::operator* (float scalar) const;
    Point2 Point2::operator/ (float scalar) const;

    Point2 &Point2::operator*= (float scalar);
    Point2 &Point2::operator/= (float scalar);
};

namespace MathUtils
{
    struct Result
    {
        float sqrDistance;
        // barycentric coordinates for triangle.v[3]
        float parameter[3];
        NiPoint3 closest;
    };

    Result GetClosestPointOnTriangle(const NiPoint3 &point, const Triangle &triangle, uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset);

    struct PlanarFit2D
    {
        float yawRad = 0.f;          // CCW rotation in radians
        NiPoint3 offsetXY{ 0.f, 0.f, 0.f };  // planar translation
        NiPoint3 restCentroid{ 0.f, 0.f, 0.f };
        NiPoint3 physCentroid{ 0.f, 0.f, 0.f };
        NiPoint3 centroidOffset{ 0.f, 0.f, 0.f };
    };
}

inline float VectorLengthSquared(const NiPoint3 &vec) { return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z; }
inline float VectorLengthSquared(const Point2 &vec) { return vec.x * vec.x + vec.y * vec.y; }
inline float VectorLength(const NiPoint3 &vec) { return sqrtf(VectorLengthSquared(vec)); }
inline float VectorLength(const Point2 &vec) { return sqrtf(VectorLengthSquared(vec)); }
inline float DotProduct(const NiPoint3 &vec1, const NiPoint3 &vec2) { return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z; }
inline float DotProduct(const Point2 &vec1, const Point2 &vec2) { return vec1.x * vec2.x + vec1.y * vec2.y; }
inline float DotProduct(const NiQuaternion &q1, const NiQuaternion &q2) { return q1.m_fW * q2.m_fW + q1.m_fX * q2.m_fX + q1.m_fY * q2.m_fY + q1.m_fZ * q2.m_fZ; }
inline float DotProductSafe(const NiPoint3 &vec1, const NiPoint3 &vec2) { return std::clamp(DotProduct(vec1, vec2), -1.f, 1.f); }
inline float DotProductSafe(const NiQuaternion &q1, const NiQuaternion &q2) { return std::clamp(DotProduct(q1, q2), -1.f, 1.f); }
inline float QuaternionLength(const NiQuaternion &q) { return sqrtf(DotProduct(q, q)); }
NiPoint3 VectorNormalized(const NiPoint3 &vec);
NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2);
float CrossProduct2D(const NiPoint3 &vec1, const NiPoint3 &vec2);
NiMatrix33 MatrixFromAxisAngle(const NiPoint3 &axis, float theta);
std::pair<NiPoint3, float> MatrixToAxisAngle(const NiMatrix33 &mat);
NiPoint3 NiMatrixToYawPitchRoll(NiMatrix33 &mat);
NiPoint3 NiMatrixToEuler(NiMatrix33 &mat);
NiPoint3 MatrixToEuler(const NiMatrix33 &mat);
NiMatrix33 EulerToMatrix(const NiPoint3 &euler);
NiMatrix33 MatrixFromForwardVector(const NiPoint3 &forward, const NiPoint3 &world);
NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle);
NiPoint3 RotateVectorByQuaternion(const NiQuaternion &quat, const NiPoint3 &vec);
NiPoint3 ProjectVectorOntoPlane(const NiPoint3 &vector, const NiPoint3 &normal);
void NiMatrixToHkMatrix(const NiMatrix33 &niMat, hkMatrix3 &hkMat);
void HkMatrixToNiMatrix(const hkMatrix3 &hkMat, NiMatrix33 &niMat);
NiMatrix33 HkMatrixToNiMatrix(const hkMatrix3 &hkMat);
NiTransform hkTransformToNiTransform(const hkTransform &t, float scale);
hkTransform NiTransformTohkTransform(NiTransform &t);
NiTransform hkQsTransformToNiTransform(const hkQsTransform &in, bool useHavokScale = true);
hkQsTransform NiTransformTohkQsTransform(const NiTransform &in, bool useHavokScale = true);
hkTransform hkQsTransformTohkTransform(const hkQsTransform &in);
NiMatrix33 QuaternionToMatrix(const NiQuaternion &q);
inline NiQuaternion MatrixToQuaternion(const NiMatrix33 &m) { NiQuaternion q; NiMatrixToNiQuaternion(q, m); return q; }
inline NiQuaternion HkQuatToNiQuat(const hkQuaternion &quat) { return { quat.m_vec(3), quat.m_vec(0), quat.m_vec(1), quat.m_vec(2) }; }
inline hkQuaternion NiQuatToHkQuat(const NiQuaternion &quat) { return hkQuaternion(quat.m_fX, quat.m_fY, quat.m_fZ, quat.m_fW); }
inline NiPoint3 HkVectorToNiPoint(const hkVector4 &vec) { return { vec.getQuad().m128_f32[0], vec.getQuad().m128_f32[1], vec.getQuad().m128_f32[2] }; }
inline hkVector4 NiPointToHkVector(const NiPoint3 &pt) { return { pt.x, pt.y, pt.z, 0 }; };
inline NiTransform InverseTransform(const NiTransform &t) { NiTransform inverse; t.Invert(inverse); return inverse; }
inline NiPoint3 RightVector(const NiMatrix33 &r) { return { r.data[0][0], r.data[1][0], r.data[2][0] }; }
inline NiPoint3 ForwardVector(const NiMatrix33 &r) { return { r.data[0][1], r.data[1][1], r.data[2][1] }; }
inline NiPoint3 UpVector(const NiMatrix33 &r) { return { r.data[0][2], r.data[1][2], r.data[2][2] }; }
inline void SetRightVector(NiMatrix33 &r, const NiPoint3 &v) { r.data[0][0] = v.x; r.data[1][0] = v.y; r.data[2][0] = v.z; }
inline void SetForwardVector(NiMatrix33 &r, const NiPoint3 &v) { r.data[0][1] = v.x; r.data[1][1] = v.y; r.data[2][1] = v.z; }
inline void SetUpVector(NiMatrix33 &r, const NiPoint3 &v) { r.data[0][2] = v.x; r.data[1][2] = v.y; r.data[2][2] = v.z; }
NiQuaternion QuaternionIdentity();
NiQuaternion QuaternionNormalized(const NiQuaternion &q);
NiQuaternion QuaternionMultiply(const NiQuaternion &qa, const NiQuaternion &qb);
NiQuaternion QuaternionMultiply(const NiQuaternion &q, float multiplier);
NiQuaternion QuaternionInverse(const NiQuaternion &q);
inline float QuaternionAngle(const NiQuaternion &qa, const NiQuaternion &qb) { return 2.f * acosf(abs(DotProductSafe(qa, qb))); }
NiQuaternion slerp(const NiQuaternion &qa, const NiQuaternion &qb, double t);
NiQuaternion continuousSlerp(const NiQuaternion &qa, const NiQuaternion &qb, double t, const NiQuaternion &qPrev);
NiQuaternion lerp(const NiQuaternion &a, const NiQuaternion &b, float t);
inline NiPoint3 lerp(const NiPoint3 &a, const NiPoint3 &b, float t) { return a * (1.f - t) + b * t; }
inline float lerp(float a, float b, float t) { return a * (1.f - t) + b * t; }
inline double lerp(double a, double b, double t) { return a * (1.0 - t) + b * t; }
hkQsTransform lerphkQsTransform(hkQsTransform &a, hkQsTransform &b, double t);
float Determinant33(const NiMatrix33 &m);
NiPoint3 QuadraticFromPoints(const NiPoint2 &p1, const NiPoint2 &p2, const NiPoint2 &p3);
std::optional<NiTransform> AdvanceTransform(const NiTransform &currentTransform, const NiTransform &targetTransform, float posSpeed, float rotSpeed);
float AdvanceFloat(float a, float b, float speed, float *deltaOut);
NiPoint3 AdvanceVector(const NiPoint3 &a, const NiPoint3 &b, float speed, NiPoint3 *deltaOut);
NiMatrix33 AdvanceRotation(const NiMatrix33 &a, const NiMatrix33 &b, float speed, NiMatrix33 *deltaOut = nullptr);
inline float ConstrainAngle180(float x) { x = fmodf(x + M_PI, 2 * M_PI); if (x < 0) x += 2 * M_PI; return x - M_PI; }
inline NiPoint3 ConstrainAngle180(const NiPoint3 &euler) { return { ConstrainAngle180(euler.x), ConstrainAngle180(euler.y), ConstrainAngle180(euler.z) }; }
inline float ConstrainAngle360(float x) { x = fmod(x, 2 * M_PI); if (x < 0) x += 2 * M_PI; return x; }
inline float ConstrainAngleNegative360(float x) { return -ConstrainAngle360(-x); }
inline float AngleDifference(float angle1, float angle2)
{
    float diff = fmodf(angle2 - angle1 + M_PI, 2 * M_PI) - M_PI;
    return diff < -M_PI ? diff + 2 * M_PI : diff;
}

bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, const NiPoint3 &point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);

MathUtils::PlanarFit2D SolvePlanarYaw(const std::vector<NiPoint3> &restPos, const std::vector<NiPoint3> &physPos, const std::vector<float> &weights);
