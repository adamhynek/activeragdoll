#pragma once

#include <optional>

#include <Common/Base/hkBase.h>
#include <Physics/Dynamics/Entity/hkpRigidBody.h>
#include <Physics/Dynamics/Phantom/hkpSimpleShapePhantom.h>
#include <Physics/Collide/Agent/hkpProcessCollisionInput.h>
#include <Physics/Collide/BroadPhase/3AxisSweep/hkp3AxisSweep.h>
#include <Physics/Collide/Filter/hkpCollisionFilter.h>
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>
#include <Physics/Dynamics/Collide/ContactListener/hkpContactPointEvent.h>
#include <Physics/Utilities/CharacterControl/CharacterProxy/hkpCharacterProxy.h>
#include <Physics/Dynamics/Constraint/Bilateral/Ragdoll/hkpRagdollConstraintData.h>

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"

#include "havok_ref_ptr.h"


namespace RE
{
    template <typename T>
    struct hkArray
    {
        enum
        {
            CAPACITY_MASK = int(0x3FFFFFFF),
            FLAG_MASK = int(0xC0000000),
            DONT_DEALLOCATE_FLAG = int(0x80000000), // Indicates that the storage is not the array's to delete
            FORCE_SIGNED = -1
        };

        T *m_data; // 00
        int m_size; // 08
        int m_capacityAndFlags; // 0C

        inline int getCapacity() const
        {
            return (m_capacityAndFlags & static_cast<int>(CAPACITY_MASK));
        }

        void pushBack(const T &t);
    };

}

enum class HavokProperty : hkUint32
{
    RootNode = 0, // NiNode - set for charcontroller rigidbodies
    Node = 1, // NiAVObject
    CollisionObject = 2, // bhkCollisionObject
    Character = 1000, // Character
    CharacterController = 1002, // bhkCharacterController
    TelekinesisDamageMult = 314159,
    TelekinesisMass = 314160,
};

struct bhkCollisionFilter : hkpCollisionFilter
{
    UInt32 unk48;
    UInt32 nextCollisionGroup; // 4C - this gets incremented when something gets added to the world - used for unique collision groups?
    UInt32 bipedBitfields[32]; // 50 - About 0x18 of them seem to be actually filled in
    UInt32 layerCollisionGroups[64]; // D0 - if zero, use the counter from the collision filter (4C) as the collision group - afaik only 3 have non-zero entries: 9 (trees), 11 (water), and 13 (terrain)
    UInt64 layerBitfields[64]; // 1D0 - only 56 are valid in vanilla - these are used to determine which layers collide with each other
    UInt64 triggerBitfield1; // 3D0 - bit x determines if phantoms on layer x should get triggers created for them. They are also tested against if a charcontroller with bit 15 set and bit 7 unset collides with it.
    UInt64 triggerBitfield2; // 3D8 - similar to above, but it's not used in the collision filter comparison or when phantoms are added to the world, so not sure where it's actually used.
    BSFixedString layerNames[64]; // 3E0 - only 56 are non-null
};
static_assert(offsetof(bhkCollisionFilter, nextCollisionGroup) == 0x4C);
static_assert(offsetof(bhkCollisionFilter, bipedBitfields) == 0x50);
static_assert(offsetof(bhkCollisionFilter, layerCollisionGroups) == 0xD0);
static_assert(offsetof(bhkCollisionFilter, layerBitfields) == 0x1D0);
static_assert(offsetof(bhkCollisionFilter, layerNames) == 0x3E0);

struct ahkpWorld : hkpWorld
{
    struct bhkWorld *m_userData; // 430
};
static_assert(offsetof(ahkpWorld, m_userData) == 0x430);

struct bhkRefObject : NiObject
{
    virtual void SetHavokObject(hkReferencedObject *object); // 25
    virtual void AddOrRemoveReferenceOnHavokObject(bool add); // 26
};

struct bhkSerializable : bhkRefObject
{
    virtual ahkpWorld *GetHavokWorld_1(); // 27
    virtual ahkpWorld *GetHavokWorld_2(); // 28
    virtual void	  MoveToWorld(struct bhkWorld *world); // 29
    virtual void	  RemoveFromCurrentWorld(); // 2A
    virtual void	  DestroyCinfo(bool destroy); // 2B
    virtual void	  Unk_2C(void); // 2C
    virtual void	  Unk_2D(void); // 2D
    virtual void	  InitHavokFromCinfo(void *cInfo); // 2E
    virtual void *GetOrCreateCinfo(bool &didCreateCinfo); // 2F
    virtual void	  Destroy(); // 30
    virtual void	  Unk_31(void *cInfo); // 31
};

struct bhkWorld : bhkSerializable
{
    virtual void Update(bool unk);  // 32
    virtual bool CastRay(hkpWorldRayCastInput &input); // 33 - actually takes in an extended hkpWorldRayCastInput that has the raycast output and stuff shoved at the end
    virtual bool HasSimulationIslands();  // 34
    virtual void AddHavok(NiAVObject *node, bool recurse, bool notify, UInt32 collisionGroup, bool ignoreBSXFlags);  // 35
    virtual void Unk_36(void);  // 36

    RE::hkRefPtr<ahkpWorld> world; // 10
    UInt8 unk18[0xC598 - 0x18];
    BSReadWriteLock worldLock; // C598
    UInt8 unkC5A0[0xC600 - 0xC5A0];
    // C530 is tArray<GraphPhysicsStepListener>
    // C560 is physicsStepListeners like bhkTrapListener, TESWindListener, BGSAcousticSpaceListener
    // C570 is bhkConstraintProjector
    // C5C0 is TESTrapListener
    // C5C8 is BGSAcousticSpaceListener
    // C5D0 is hkpSuspendInactiveAgentsUtil
    // C5D8 is some sort of counter
};
static_assert(offsetof(bhkWorld, world) == 0x10);
static_assert(offsetof(bhkWorld, worldLock) == 0xC598);
static_assert(sizeof(bhkWorld) == 0xC600);

struct bhkTrapListener : hkpEntityListener
{
    virtual void PhysicsStepCallback(float deltaTime); // 06

    UInt64 unk08;
    UInt64 unk10;
    hkpPhantomListener phantomListener; // 18
    struct TriggerEntry *triggers; // 20
    bool unk28;
};
static_assert(offsetof(bhkTrapListener, triggers) == 0x20);

struct bhkShape : bhkSerializable
{
    virtual void Unk_32(void); // 32
    virtual void Unk_33(void); // 33
    virtual struct bhkShapeCollection *GetCollection(); // 34 - { return 0; }
    virtual void Unk_35(void); // 35

    RE::hkRefPtr<hkpShape> shape; // 10
    UInt64 cinfo; // 18
    UInt32 materialId; // 20
    UInt32 filterInfo; // 24
};
static_assert(sizeof(bhkShape) == 0x28);

struct bhkTransformShape : bhkShape {};

struct bhkShapeCollection : bhkShape
{
    virtual UInt32 GetMaterialId(hkpShapeKey shapeKey); // 36
};

struct bhkSphereRepShape : bhkShape {};

struct bhkConvexShape : bhkSphereRepShape {};

struct bhkBoxShape : bhkConvexShape {};
static_assert(sizeof(bhkBoxShape) == 0x28);

struct bhkListShapeCinfo
{
    ~bhkListShapeCinfo();

    UInt32 unk00;
    UInt32 pad04;
    RE::hkArray<const hkpShape *> shapes; // 08
    RE::hkArray<hkUint32> filterInfos; // 18
};

struct bhkListShape : bhkShape
{
    bool hasNonTransformContainerChildShape;
    UInt8 pad29[7];
};
static_assert(sizeof(bhkListShape) == 0x30);

struct bhkConstraint : bhkSerializable
{
    virtual void SetRigidBodyA(hkpRigidBody *rigidBody); // 32
    virtual void SetRigidBodyB(hkpRigidBody *rigidBody); // 33
    virtual hkpRigidBody *GetRigidBodyA(); // 34
    virtual hkpRigidBody *GetRigidBodyB(); // 35
    virtual bool EnableOrDisableConstraint(char enable); // 36

    RE::hkRefPtr<hkpConstraintInstance> constraint; // 10
    struct hkConstraintCinfo *cinfo; // 18
};

struct bhkWorldObject : bhkSerializable
{
    virtual void Unk_32(void); // 32
};

struct bhkEntity : bhkWorldObject
{

};

struct bhkRigidBody : bhkEntity
{
    virtual hkVector4 &getPosition(hkVector4 &position); // 33
    virtual hkVector4 &getRotation(hkQuaternion &rotation); // 34
    virtual void setPosition(hkVector4 &position); // 35
    virtual void setRotation(hkQuaternion &rotation); // 36
    virtual void setPositionAndRotation(hkVector4 &pos, hkQuaternion &rot); // 37
    virtual hkVector4 &getCenterOfMassLocal(hkVector4 &centerOfMassLocal); // 38
    virtual hkVector4 &getCenterOfMassInWorld(hkVector4 &centerOfMassWorld); // 39
    virtual hkTransform &getTransform(hkTransform &transform); // 3A
    virtual void getAabbWorldspace(hkAabb &aabb); // 3B
    virtual void Unk_3C(void); // 3C

    RE::hkRefPtr<hkpRigidBody> hkBody; // 10
    struct bhkRigidBodyCinfo *cinfo; // 18
    UInt8 flags; // at least first byte are some flags? bit 2 is set -> has constraints?
    tArray<NiPointer<bhkConstraint>> constraints; // 28
};
static_assert(offsetof(bhkRigidBody, constraints) == 0x28);
static_assert(sizeof(bhkRigidBody) == 0x40);

struct bhkRigidBodyT : bhkRigidBody
{
    hkQuaternion rotation; // 40
    hkVector4 translation; // 50
};
static_assert(offsetof(bhkRigidBodyT, rotation) == 0x40);
static_assert(offsetof(bhkRigidBodyT, translation) == 0x50);

struct NiCollisionObject : NiObject
{
    virtual void SetNode(NiAVObject *node); // 25
    virtual void Update(NiAVObject::ControllerUpdateContext *ctx); // 26
    // These next 3 all return immediately
    virtual void Unk_27(void); // 27 - { return; }
    virtual void Unk_28(void); // 28 - { return; }
    virtual void Unk_29(void); // 29 - { return; }

    NiAVObject *node; // 10 - points back to the NiAVObject pointing to this
};

struct bhkNiCollisionObject : NiCollisionObject
{
    virtual void GetLinearVelocity(void); // 2A
    virtual void UpdateNodeTransformsFromCollision(void) = 0; // 2B
    virtual void UpdateCollisionFromNodeTransform(void) = 0; // 2C
    virtual void ZeroOutSmallVelocities(void) = 0; // 2D
    virtual void SetMotionType(UInt32 motionType, bhkRigidBody *rigidBody, bool activate) = 0; // 2E
    virtual void IsFixedOrKeyframed(void); // 2F
    virtual void Unk_30(void); // 30 - { return 1; }

    struct InitData
    {
        bhkWorld *world; // 00
        bool recurse = true; // 08 - whether to continue downstream of the current node
        UInt32 unk0C = 0; // 0C
        UInt32 notify = 1;  // 10 - If first bit is set, set kNotify and kSetLocal. If it isn't, unset kNotify. First bit is unset after first node is processed.
        UInt32 pad14;
        UInt32 collisionGroup = 0; // 18 - If set, use it as the initial collision group. If 0, get the next unused group from the bhkCollisionFilter, then set this to that.
        UInt32 pad1C;
        UInt32 dontBlend = 0; // 20 - If set, use node pos instead of blended pos for bhkBlendCollisionObject. If 0, use blended pos. After first node is processed, this is incremented.
        UInt32 pad24;
        UInt32 resetOnWorldChange = 1; // 28 - If changing worlds: If set, set kReset. If 0, unset kReset.
    };

    UInt32 flags; // 18 - flag 8 -> use blended pos (for bhkBlendCollisionObject) instead of node pos
    UInt32 pad1C; // 1C
    NiPointer<bhkWorldObject> body; // 20
};
static_assert(offsetof(bhkNiCollisionObject, body) == 0x20);

struct bhkCollisionObject : bhkNiCollisionObject
{
};

struct bhkBlendCollisionObject : bhkCollisionObject
{
    float blendStrength; // 28 - this affects how intensely to go from rigidBody position to node position. 0 means strictly follow rigidbody, 1 means strictly follow node.
    float unk2C;
    UInt32 motionType; // 30
    bhkWorld *world; // 38
    UInt32 unk40;
};
static_assert(sizeof(bhkBlendCollisionObject) == 0x48);

struct bhkSimpleShapePhantom : NiRefObject
{
    RE::hkRefPtr<hkpSimpleShapePhantom> phantom; // 10
};

struct bhkRigidBodyCinfo
{
    UInt32 collisionFilterInfo; // 00 - initd to 0
    hkpShape *shape; // 08 - initd to 0
    UInt8 unk10; // initd to 1
    UInt64 unk18; // initd to 0
    UInt32 unk20; // initd to 0
    float unk24; // initd to -0
    UInt8 unk28; // initd to 1
    UInt16 unk2A; // initd to -1 - quality type?
    hkpRigidBodyCinfo hkCinfo; // 30 - size == 0xE0
};
static_assert(offsetof(bhkRigidBodyCinfo, shape) == 0x08);
static_assert(offsetof(bhkRigidBodyCinfo, hkCinfo) == 0x30);
static_assert(sizeof(bhkRigidBodyCinfo) == 0x110);

struct hkConstraintCinfo
{
    ~hkConstraintCinfo();

    void *vtbl = 0; // 00
    RE::hkRefPtr<hkpConstraintData> constraintData = nullptr; // 08
    hkpConstraintInstance::ConstraintPriority priority = hkpConstraintInstance::ConstraintPriority::PRIORITY_INVALID; // 10
    UInt32 pad14 = 0;
    hkpRigidBody *rigidBodyA = nullptr; // 18
    hkpRigidBody *rigidBodyB = nullptr; // 20
};

typedef void(*_hkConstraintCinfo_CreateConstraintData)(hkConstraintCinfo *); // vfunc 4

struct hkMalleableConstraintCinfo : hkConstraintCinfo
{
    hkMalleableConstraintCinfo();
};

struct hkRagdollConstraintCinfo : hkConstraintCinfo
{
    hkRagdollConstraintCinfo();
};

struct hkFixedConstraintCinfo : hkConstraintCinfo
{
    UInt64 unk28 = 0;
    UInt8 unk30 = 0; // type or something
};

struct hkBallAndSocketConstraintCinfo : hkConstraintCinfo
{
    // These are not necessarily 16 byte aligned
    float pivotB[4]; // 28
    float pivotA[4]; // 38
};

struct bhkGroupConstraint : bhkConstraint
{
    UInt32 collisionGroup; // 20
    UInt32 pad24;
};
static_assert(sizeof(bhkGroupConstraint) == 0x28);

struct bhkMalleableConstraint : bhkConstraint {};
static_assert(sizeof(bhkMalleableConstraint) == 0x20);

struct bhkRagdollConstraint : bhkConstraint {};
static_assert(sizeof(bhkRagdollConstraint) == 0x20);

inline bool IsMotionTypeMoveable(UInt8 motionType) {
    return (
        motionType == hkpMotion::MotionType::MOTION_DYNAMIC ||
        motionType == hkpMotion::MotionType::MOTION_SPHERE_INERTIA ||
        motionType == hkpMotion::MotionType::MOTION_BOX_INERTIA ||
        motionType == hkpMotion::MotionType::MOTION_THIN_BOX_INERTIA
        );
}
inline bool IsMoveableEntity(hkpEntity *entity) { return IsMotionTypeMoveable(entity->m_motion.m_type); }

NiPoint3 hkpRigidBody_getPointVelocity(const hkpRigidBody *body, const hkVector4 &pos);

hkMemoryRouter &hkGetMemoryRouter();
inline void *hkHeapAlloc(int numBytes) {
    return hkGetMemoryRouter().heap().blockAlloc(numBytes);
}
inline void *hkStackAlloc(int numBytes) {
    return hkGetMemoryRouter().stack().blockAlloc(numBytes);
}

template <typename T>
inline T *hkStackAllocObjects(int numObjects) {
    int numBytes = numObjects * sizeof(T);
    return (T *)hkGetMemoryRouter().stack().blockAlloc(numBytes);
}
template <typename T>
inline void hkStackFreeObjects(T *objects, int numObjects) {
    int numBytes = numObjects * sizeof(T);
    hkGetMemoryRouter().stack().blockFree(objects, numBytes);
}

// Array of temporary objects allocated with the thread-local havok stack allocator
template <typename T>
struct hkStackArray
{
    hkStackArray(int numObjects)
    {
        m_data = hkStackAllocObjects<T>(numObjects);
        m_size = numObjects;
    }

    ~hkStackArray()
    {
        hkStackFreeObjects<T>(m_data, m_size);
    }

    inline T &operator[] (int i)
    {
        return m_data[i];
    }

    inline const T &operator[] (int i) const
    {
        return m_data[i];
    }

    T *m_data;
    int m_size;
};

template <typename T>
T *hkAllocReferencedObject() {
    T *allocated = (T *)hkHeapAlloc(sizeof(T));
    allocated->m_memSizeAndFlags = sizeof(T);
    return allocated;
}

void hkpWorld_removeContactListener(hkpWorld *_this, hkpContactListener *worldListener);
bool hkpWorld_hasContactListener(hkpWorld *_this, hkpContactListener *listener);
bool hkpWorld_hasWorldPostSimulationListener(hkpWorld *_this, hkpWorldPostSimulationListener *listener);
bool hkpWorld_hasEntityListener(hkpWorld *_this, hkpEntityListener *listener);
int hkpCharacterProxy_findCharacterProxyListener(hkpCharacterProxy *_this, hkpCharacterProxyListener *proxyListener);
float hkpContactPointEvent_getSeparatingVelocity(const hkpContactPointEvent &_this);
void hkpRagdollConstraintData_setPivotInWorldSpace(hkpRagdollConstraintData *constraint, const hkTransform &bodyATransform, const hkTransform &bodyBTransform, const hkVector4 &pivot);
bool bhkCollisionFilter_CompareFilterInfosEx(bhkCollisionFilter *_this, UInt32 filterInfoA, UInt32 filterInfoB, std::optional<UInt64> a_layerBitfield = std::nullopt);

void bhkMalleableConstraint_ctor(bhkMalleableConstraint *_this, hkMalleableConstraintCinfo *cInfo);
bhkMalleableConstraint *CreateMalleableConstraint(bhkConstraint *constraint, float strength);
hkpConstraintInstance *LimitedHingeToRagdollConstraint(hkpConstraintInstance *constraint);
bhkRagdollConstraint *ConvertToRagdollConstraint(bhkConstraint *constraint);

typedef void (*_hkp3AxisSweep_removeObject)(hkp3AxisSweep *, hkpBroadPhaseHandle *, hkArray<hkpBroadPhaseHandlePair> *);
typedef void (*_hkp3AxisSweep_removeObjectBatch)(hkp3AxisSweep *, hkArrayBase<hkpBroadPhaseHandle *> *, hkArray<hkpBroadPhaseHandlePair> *);

