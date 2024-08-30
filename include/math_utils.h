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
NiMatrix33 MatrixFromAxisAngle(const NiPoint3 &axis, float theta);
NiPoint3 NiMatrixToYawPitchRoll(NiMatrix33 &mat);
NiPoint3 NiMatrixToEuler(NiMatrix33 &mat);
NiPoint3 MatrixToEuler(const NiMatrix33 &mat);
NiMatrix33 EulerToMatrix(const NiPoint3 &euler);
NiMatrix33 MatrixFromForwardVector(NiPoint3 &forward, NiPoint3 &world);
NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle);
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
NiQuaternion QuaternionIdentity();
NiQuaternion QuaternionNormalized(const NiQuaternion &q);
NiQuaternion QuaternionMultiply(const NiQuaternion &qa, const NiQuaternion &qb);
NiQuaternion QuaternionMultiply(const NiQuaternion &q, float multiplier);
NiQuaternion QuaternionInverse(const NiQuaternion &q);
inline float QuaternionAngle(const NiQuaternion &qa, const NiQuaternion &qb) { return 2.f * acosf(abs(DotProductSafe(qa, qb))); }
NiQuaternion slerp(const NiQuaternion &qa, const NiQuaternion &qb, double t);
inline NiPoint3 lerp(const NiPoint3 &a, const NiPoint3 &b, float t) { return a * (1.f - t) + b * t; }
inline float lerp(float a, float b, float t) { return a * (1.f - t) + b * t; }
inline double lerp(double a, double b, double t) { return a * (1.0 - t) + b * t; }
hkQsTransform lerphkQsTransform(hkQsTransform &a, hkQsTransform &b, double t);
float Determinant33(const NiMatrix33 &m);
NiPoint3 QuadraticFromPoints(const NiPoint2 &p1, const NiPoint2 &p2, const NiPoint2 &p3);
std::optional<NiTransform> AdvanceTransform(const NiTransform &currentTransform, const NiTransform &targetTransform, float posSpeed, float rotSpeed);
float AdvanceFloat(float a, float b, float speed, float *deltaOut);
NiPoint3 AdvanceVector(const NiPoint3 &a, const NiPoint3 &b, float speed, NiPoint3 *deltaOut);
inline float ConstrainAngle180(float x) { x = fmodf(x + M_PI, 2 * M_PI); if (x < 0) x += 2 * M_PI; return x - M_PI; }
inline float ConstrainAngle360(float x) { x = fmod(x, 2 * M_PI); if (x < 0) x += 2 * M_PI; return x; }
inline float ConstrainAngleNegative360(float x) { return -ConstrainAngle360(-x); }
inline float AngleDifference(float angle1, float angle2)
{
    float diff = fmodf(angle2 - angle1 + 180.f, 360.f) - 180.f;
    return diff < -180.f ? diff + 360.f : diff;
}

bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, const NiPoint3 &point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);



namespace NiMathDouble
{
    class NiPoint3
    {
    public:
        long double	x;	// 0
        long double	y;	// 4
        long double	z;	// 8

        NiPoint3();
        NiPoint3(long double X, long double Y, long double Z) : x(X), y(Y), z(Z) { };
        NiPoint3(const ::NiPoint3 &pointSingle);

        ::NiPoint3 ToSingle() const { return ::NiPoint3(x, y, z); }

        // Negative
        NiPoint3 operator- () const;

        // Basic operations
        NiPoint3 operator+ (const NiPoint3 &pt) const;
        NiPoint3 operator- (const NiPoint3 &pt) const;

        NiPoint3 &operator+= (const NiPoint3 &pt);
        NiPoint3 &operator-= (const NiPoint3 &pt);

        // Scalar operations
        NiPoint3 operator* (long double fScalar) const;
        NiPoint3 operator/ (long double fScalar) const;

        NiPoint3 &operator*= (long double fScalar);
        NiPoint3 &operator/= (long double fScalar);
    };

    class NiMatrix33
    {
    public:
        union
        {
            long double	data[3][3];
            long double   arr[9];
        };

        NiMatrix33() = default;
        NiMatrix33(const ::NiMatrix33 &matSingle);

        ::NiMatrix33 ToSingle() const;

        void Identity();

        // Matric mult
        NiMatrix33 operator*(const NiMatrix33 &mat) const;

        // Vector mult
        NiPoint3 operator*(const NiPoint3 &pt) const;

        // Scalar multiplier
        NiMatrix33 operator*(long double fScalar) const;

        NiMatrix33 Transpose() const;
    };

    // 34
    class NiTransform
    {
    public:
        NiMatrix33	rot;	// 00
        NiPoint3	pos;	// 24
        long double		scale;	// 30

        NiTransform();
        NiTransform(const ::NiTransform &transformSingle);

        ::NiTransform ToSingle() const;

        // Multiply transforms
        NiTransform operator*(const NiTransform &rhs) const;

        // Transform point
        NiPoint3 operator*(const NiPoint3 &pt) const;

        // Invert
        void Invert(NiTransform &kDest) const;
    };

    struct NiQuaternion
    {
        double	m_fW;	// 0
        double	m_fX;	// 4
        double	m_fY;	// 8
        double	m_fZ;	// C

        NiQuaternion() : m_fW(1.0), m_fX(0.0), m_fY(0.0), m_fZ(0.0) { }
        NiQuaternion(const ::NiQuaternion &quatSingle);
        ::NiQuaternion ToSingle() const;
    };

    struct hkQsTransform
    {
        NiPoint3 m_translation;
        NiQuaternion m_rotation;
        NiPoint3 m_scale;

        hkQsTransform() : m_translation(0, 0, 0), m_rotation(), m_scale(1.0, 1.0, 1.0) { }
        hkQsTransform(const ::hkQsTransform &transformSingle);
        ::hkQsTransform ToSingle() const;
    };

    inline NiTransform InverseTransform(const NiTransform &t) { NiTransform inverse; t.Invert(inverse); return inverse; }
    inline float DotProduct(const NiPoint3 &vec1, const NiPoint3 &vec2) { return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z; }
    NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2);

    NiPoint3 RotateVectorByQuaternion(const NiQuaternion &quat, const NiPoint3 &vec);
    hkQsTransform hkQsTransform_Multiply(const hkQsTransform *a, const hkQsTransform &b);
    void hkbPoseLocalToPoseWorld(int a_numBones, const hkInt16 *a_parentIndices, const ::hkQsTransform &a_worldFromModel, const ::hkQsTransform *a_poseLocal, ::hkQsTransform *a_poseWorldOut);
    hkQsTransform GetRigidBodyTLocalTransform(bhkRigidBody *rigidBody, bool useHavokScale);
    hkQsTransform InversehkQsTransform(const hkQsTransform &t);
    void MapPoseWorldSpaceToPoseLocalSpace(int a_numPoses, SInt16 *a_parentIndices, const ::hkQsTransform *a_worldFromModel, const ::hkQsTransform *a_ragdollPoseWS, ::hkQsTransform *a_poseLocalSpaceOut);
}

