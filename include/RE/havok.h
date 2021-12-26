#pragma once

#include <Common/Base/hkBase.h>
#include <Physics/Dynamics/Entity/hkpRigidBody.h>
#include <Physics/Dynamics/Phantom/hkpSimpleShapePhantom.h>
#include <Physics/Collide/Agent/hkpProcessCollisionInput.h>
#include <Physics/Collide/Filter/hkpCollisionFilter.h>
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"

#include "havok_ref_ptr.h"


struct bhkCollisionFilter : hkpCollisionFilter
{
	UInt32 unk48;
	UInt32 nextCollisionGroup; // 4C - this gets incremented when something gets added to the world - used for unique collision groups?
	UInt32 bipedBitfields[32]; // 50 - About 18 of them seem to be actually filled in
	UInt32 layerCollisionGroups[64]; // D0 - if zero, use the counter from the collision filter (4C) as the collision group - afaik only 3 have non-zero entries: 9 (trees), 11 (water), and 13 (terrain)
	UInt64 layerBitfields[64]; // 1D0 - only 56 are valid in vanilla - these are used to determine which layers collide with each other
	UInt64 unk3D0;
	UInt64 unk3D8;
	BSFixedString layerNames[64]; // 3E0 - only 56 are non-null
};
static_assert(offsetof(bhkCollisionFilter, nextCollisionGroup) == 0x4C);
static_assert(offsetof(bhkCollisionFilter, bipedBitfields) == 0x50);
static_assert(offsetof(bhkCollisionFilter, layerCollisionGroups) == 0xD0);
static_assert(offsetof(bhkCollisionFilter, layerBitfields) == 0x1D0);
static_assert(offsetof(bhkCollisionFilter, layerNames) == 0x3E0);

struct ahkpWorld : hkpWorld
{
	struct bhkWorld * m_userData; // 430
};
static_assert(offsetof(ahkpWorld, m_userData) == 0x430);

// bhkWorld pointer (exteriors) is at reloc<0x1f850d0>
// function that gets it from a TESObjectCELL is at reloc<276A90>

// Address of pointer that points to the bhkWorld pointer
// RelocAddr<bhkWorld ***> BHKWORLD(0x1f850d0); - world for _tamriel outside_ is here - does not work for interiors

struct bhkRefObject : NiObject
{
	virtual void SetHavokObject(void); // 25
	virtual void AddOrRemoveReference(void); // 26
};

struct bhkSerializable : bhkRefObject
{
	virtual ahkpWorld* GetHavokWorld_1(); // 27
	virtual ahkpWorld* GetHavokWorld_2(); // 28
	virtual void	  MoveToWorld(struct bhkWorld *world); // 29
	virtual void	  RemoveFromCurrentWorld(); // 2A
	virtual void	  Unk_2B(void); // 2B
	virtual void	  Unk_2C(void); // 2C
	virtual void	  Unk_2D(void); // 2D
	virtual void	  InitHavokFromCinfo(void *cInfo); // 2E
	virtual void	  GetSerializable(void); // 2F
	virtual void	  Unk_30(void); // 30
	virtual void	  Unk_31(void); // 31
};

struct bhkWorld : bhkSerializable
{
	virtual void Update(bool unk);  // 32
	virtual bool CastRay(hkpWorldRayCastInput& input); // 33 - actually takes in an extended hkpWorldRayCastInput that has the raycast output and stuff shoved at the end
	virtual bool HasSimulationIslands();  // 34
	virtual void Unk_35(void);  // 35
	virtual void Unk_36(void);  // 36

	ahkpWorld * world; // 10
	UInt8 unk18[0xC598 - 0x18];
	BSReadWriteLock worldLock; // C598
	// C530 is tArray<GraphPhysicsStepListener>
	// C570 is bhkConstraintProjector
	// C5C0 is TESTrapListener
	// C5C8 is BGSAcousticSpaceListener
	// C5D0 is hkpSuspendInactiveAgentsUtil
	// C5D8 is some sort of counter
};
static_assert(offsetof(bhkWorld, world) == 0x10);
static_assert(offsetof(bhkWorld, worldLock) == 0xC598);

struct bhkShape : bhkSerializable
{
	hkpShape *shape; // 10
	UInt64 unk18; // == 0?
	UInt32 materialId; // 20
	UInt32 pad28;
};
static_assert(sizeof(bhkShape) == 0x28);

struct bhkSphereRepShape : bhkShape {};

struct bhkConvexShape : bhkSphereRepShape {};

struct bhkBoxShape : bhkConvexShape {};
static_assert(sizeof(bhkBoxShape) == 0x28);

struct bhkConstraint : bhkSerializable
{
	hkpConstraintInstance *constraint; // 10
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
	virtual hkVector4 & getPosition(hkVector4 &position); // 33
	virtual hkVector4 & getRotation(hkQuaternion &rotation); // 34
	virtual void setPosition(hkVector4 &position); // 35
	virtual void setRotation(hkQuaternion &rotation); // 36
	virtual void setPositionAndRotation(hkVector4 &pos, hkQuaternion &rot); // 37
	virtual hkVector4 & getCenterOfMassLocal(hkVector4 &centerOfMassLocal); // 38
	virtual hkVector4 & getCenterOfMassInWorld(hkVector4 &centerOfMassWorld); // 39
	virtual hkTransform & getTransform(hkTransform &transform); // 3A
	virtual void getAabbWorldspace(hkAabb &aabb); // 3B
	virtual void Unk_3C(void); // 3C

	hkpRigidBody * hkBody; // 10
	UInt64 unk18;
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
	virtual void SetNode(NiAVObject* node); // 25
	virtual void Update(NiAVObject::ControllerUpdateContext *ctx); // 26
	// These next 3 all return immediately
	virtual void Unk_27(void); // 27 - { return; }
	virtual void Unk_28(void); // 28 - { return; }
	virtual void Unk_29(void); // 29 - { return; }

	NiAVObject * node; // 10 - points back to the NiAVObject pointing to this
};

struct bhkNiCollisionObject : NiCollisionObject
{
	virtual void GetLinearVelocity(void); // 2A
	virtual void UpdateNodeTransformsFromCollision(void) = 0; // 2B
	virtual void UpdateCollisionFromNodeTransform(void) = 0; // 2C
	virtual void ZeroOutSmallVelocities(void) = 0; // 2D
	virtual void SetMotionType(void) = 0; // 2E
	virtual void IsFixedOrKeyframed(void); // 2F
	virtual void Unk_30(void); // 30 - { return 1; }

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
	UInt64 unk38;
	UInt32 unk40;
};
static_assert(sizeof(bhkBlendCollisionObject) == 0x48);

struct bhkSimpleShapePhantom : NiRefObject
{
	hkpSimpleShapePhantom * phantom; // 10
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
	UInt32 unk10 = 0;
	UInt32 unk14 = 0;
	hkpRigidBody *rigidBodyA = nullptr; // 18
	hkpRigidBody *rigidBodyB = nullptr; // 20
};

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

struct bhkMalleableConstraint : bhkConstraint
{
	UInt64 unk18 = 0;
};
static_assert(sizeof(bhkMalleableConstraint) == 0x20);

struct bhkRagdollConstraint : bhkConstraint
{
	UInt64 unk18 = 0;
};
static_assert(sizeof(bhkRagdollConstraint) == 0x20);

void bhkMalleableConstraint_ctor(bhkMalleableConstraint *_this, hkMalleableConstraintCinfo *cInfo);
bhkMalleableConstraint * CreateMalleableConstraint(bhkConstraint *constraint, float strength);
hkpConstraintInstance * LimitedHingeToRagdollConstraint(hkpConstraintInstance *constraint);
bhkRagdollConstraint * ConvertToRagdollConstraint(bhkConstraint *constraint);
