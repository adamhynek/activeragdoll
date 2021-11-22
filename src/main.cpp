#include <functional>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <atomic>

#include "xbyak/xbyak.h"
#include "common/IDebugLog.h"  // IDebugLog
#include "skse64_common/skse_version.h"  // RUNTIME_VERSION
#include "skse64/PluginAPI.h"  // SKSEInterface, PluginInfo
#include "skse64/GameRTTI.h"
#include "skse64/GameSettings.h"
#include "skse64/NiNodes.h"
#include "skse64/NiObjects.h"
#include "skse64/NiExtraData.h"
#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/PapyrusActor.h"
#include "skse64/GameVR.h"
#include "skse64_common/SafeWrite.h"
#include "skse64_common/BranchTrampoline.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "RE/offsets.h"
#include "version.h"
#include "config.h"
#include "utils.h"
#include "math_utils.h"
#include "RE/havok.h"
#include "havok_ref_ptr.h"

#include <Physics/Dynamics/Entity/hkpRigidBody.h>
#include <Physics/Utilities/Collide/TriggerVolume/hkpTriggerVolume.h>
#include <Physics/Dynamics/Collide/ContactListener/hkpContactListener.h>
#include <Animation/Ragdoll/Instance/hkaRagdollInstance.h>
#include <Animation/Ragdoll/Controller/RigidBody/hkaRagdollRigidBodyController.h>
#include <Animation/Animation/Mapper/hkaSkeletonMapper.h>
#include <Physics/Dynamics/Constraint/Bilateral/Ragdoll/hkpRagdollConstraintData.h>
#include <Physics/Dynamics/Constraint/Bilateral/LimitedHinge/hkpLimitedHingeConstraintData.h>
#include <Physics/Dynamics/Constraint/Motor/Position/hkpPositionConstraintMotor.h>
#include <Physics/Dynamics/World/Extensions/hkpWorldExtension.h>


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;
SKSETrampolineInterface *g_trampoline = nullptr;

bool initComplete = false; // Whether hands have been initialized


// Potentially, could hook TESObjectREFR::InitHavok to set the collision layer for the weapon when we drop it so it doesn't collide with the bumper

// RightMeleeContactListener : hkpContactListener is at 0x2FFFDD0
// LeftMeleeContactListener : hkpContactListener is at 0x2FFFDD8
// May be flipped in left handed mode, I dunno

// 6B03D0 - inits some VR melee things in the PlayerCharacter from ini settings when switching weapons

// For setting transform of our trigger, probably hook at 0x6E5258

// 2AE5E0 - recursive check of node for collision when weapon swing - 2AFAA0 does the actual checks for a single node - 2ADFD0 is the one that does it on the BSFadeNode root of a TESObjectREFR
// ^^ Takes in BSFadeNode *, NiPoint3 * to position of the weapon hit? and checks which node within is the intersecting one?
// ^^ It gets passed the location of the WEAPON node during a weapon swing

// 60C808 - address of call to TESObjectREFR_EndHavokHit in Actor::KillImpl


struct HavokHitJob
{
	NiPoint3 position; // 00
	NiPoint3 direction; // 0C
	UInt32 refHandle; // 18
	UInt32 endTimeMilliseconds = -1; // 1C
};
static_assert(sizeof(HavokHitJob) == 0x20);

struct HavokHitJobs
{
	UInt64 unk00;
	// Actually this is a BSTSmallArray<HavokHitJob> from this point
	UInt32 unk08 = 0x80000000;
	UInt32 pad0C;
	HavokHitJob jobs[5]; // 10
	UInt32 numJobs = 5; // B0
};
static_assert(offsetof(HavokHitJobs, jobs) == 0x10);
static_assert(offsetof(HavokHitJobs, numJobs) == 0xB0);

struct hkbWorldFromModelModeData
{
	hkInt16 poseMatchingBone0; // 00
	hkInt16 poseMatchingBone1; // 02
	hkInt16 poseMatchingBone2; // 04
	UInt8 mode; // 06
};

struct hkbRagdollDriver : hkReferencedObject
{
	hkReal ragdollBlendOutTime; // 10
	hkbWorldFromModelModeData worldFromModelModeData; // 14
	hkBool autoAddRemoveRagdollToWorld; // 1C
	hkBool useAsynchronousStepping; // 1D
	hkQsTransform lastWorldFromModel; // 20
	hkbWorldFromModelModeData worldFromModelModeDataInternal; // 50
	hkArray<hkBool32> reportingWhenKeyframed; // 58 - Indexed by (boneIdx >> 5), and then you >> (boneIdx & 0x1F) & 1 to extract the specific bit
	UInt64 unk68; // numRagdollBones?
	hkPointerMap<hkReferencedObject *, hkBool32> attachedRigidBodyToIndexMap; // 70 - maybe
	struct hkbCharacter *character; // 80
	hkaRagdollInstance *ragdoll; // 88
	hkQsTransform *ragdollPoseWS; // 90
	hkaRagdollRigidBodyController *ragdollController; // 98
	hkQsTransform *ragdollPoseHiResLocal; // A0 - maybe
	hkQsTransform *lastPoseLocal; // A8
	SInt32 lastNumPoseLocal; // B0
	float lastFrameRigidBodyOnFraction; // B4
	float lastFramePoweredOnFraction; // B8
	float timeRigidBodyControllerActive; // BC
	float ragdollBlendOutTimeElapsed; // C0
	hkBool canAddRagdollToWorld; // C4
	hkBool shouldReinitializeRagdollController; // C5
	hkBool isEnabled; // C6
	hkBool isPoweredControllerEnabled; // C7
	hkBool isRigidBodyControllerEnabled; // C8
	hkBool wasRigidBodyControllerEnabledLastFrame; // C9
	hkBool ragdollPoseWasUsed; // CA
	hkBool allBonesKeyframed; // CB
};
static_assert(offsetof(hkbRagdollDriver, character) == 0x80);
static_assert(sizeof(hkbRagdollDriver) == 0xD0);

struct hkbCharacterSetup : hkReferencedObject
{
	hkArray<hkRefPtr<hkaSkeletonMapper>> m_retargetingSkeletonMappers; // 10
	hkRefPtr<hkaSkeleton> m_animationSkeleton; // 20
	hkRefPtr<hkaSkeletonMapper> m_ragdollToAnimationSkeletonMapper; // 28
	hkRefPtr<hkaSkeletonMapper> m_animationToRagdollSkeletonMapper; // 30
	hkRefPtr<struct hkbAnimationBindingSet> m_animationBindingSet; // 38
	hkRefPtr<struct hkbCharacterData> m_data; // 40
	UInt64 unk48; // probably either m_unscaledAnimationSkeleton or m_mirroredSkeleton
	hkRefPtr<struct hkbSymbolIdMap> m_characterPropertyIdMap; // 50
	mutable hkCriticalSection *m_criticalSection; // 58
};

struct hkbCharacter : hkReferencedObject
{
	// members
	hkArray<hkbCharacter*>      nearbyCharacters;           // 10
	std::int16_t                currentLOD;                 // 20
	std::int16_t                numTracksInLOD;             // 22
	std::uint32_t               pad24;                      // 24
	hkStringPtr                 name;                       // 28
	hkRefPtr<hkbRagdollDriver>  ragdollDriver;              // 30
	hkRefVariant                characterControllerDriver;  // 38
	hkRefPtr<struct hkbFootIkDriver> footIkDriver;               // 40
	hkRefPtr<struct hkbHandIkDriver> handIkDriver;               // 48
	hkRefPtr<hkbCharacterSetup> setup;                      // 50
	hkRefPtr<struct hkbBehaviorGraph>  behaviorGraph;              // 58
	hkRefPtr<struct hkbProjectData>    projectData;                // 60
	hkRefPtr<struct hkbAnimationBindingSet> animationBindingSet;        // 68
	hkRefVariant                raycastInterface;           // 70
	hkRefVariant                world;                      // 78
	hkRefVariant                eventQueue;                 // 80
	hkRefVariant                worldFromModel;             // 88
	const void**                poseLocal;                  // 90 - hkSimpleArray<hkRefVariant>
	std::int32_t                numPoseLocal;               // 98
	bool                        deleteWorldFromModel;       // 9C
	bool                        deletePoseLocal;            // 9D
	std::uint16_t               pad9E;                      // 9E
};
static_assert(sizeof(hkbCharacter) == 0xA0);

struct hkbContext
{
	hkbCharacter *character; // 00
	UInt64 unk08;
	UInt64 unk10;
	UInt64 unk18;
	UInt64 unk20;
	UInt64 unk28;
	bool unk30;
	ahkpWorld *world; // 38
	UInt64 unk40;
	UInt64 unk48;
};
static_assert(offsetof(hkbContext, world) == 0x38);

struct hkbGeneratorOutput
{
	enum class StandardTracks
	{
		TRACK_WORLD_FROM_MODEL, // 00
		TRACK_EXTRACTED_MOTION, // 01
		TRACK_POSE, // 02
		TRACK_FLOAT_SLOTS, // 03
		TRACK_RIGID_BODY_RAGDOLL_CONTROLS, // 04
		TRACK_RIGID_BODY_RAGDOLL_BLEND_TIME, // 05
		TRACK_POWERED_RAGDOLL_CONTROLS, // 06
		TRACK_POWERED_RAGDOLL_WORLD_FROM_MODEL_MODE, // 07
		TRACK_KEYFRAMED_RAGDOLL_BONES, // 08
		TRACK_KEYFRAME_TARGETS, // 09
		TRACK_ANIMATION_BLEND_FRACTION, // 0A
		TRACK_ATTRIBUTES, // 0B
		TRACK_FOOT_IK_CONTROLS, // 0C
		TRACK_CHARACTER_CONTROLLER_CONTROLS, // 0D
		TRACK_HAND_IK_CONTROLS_0, // 0E
		TRACK_HAND_IK_CONTROLS_1, // 0F
		TRACK_HAND_IK_CONTROLS_2, // 10
		TRACK_HAND_IK_CONTROLS_3, // 11
		TRACK_HAND_IK_CONTROLS_NON_BLENDABLE_0, // 12
		TRACK_HAND_IK_CONTROLS_NON_BLENDABLE_1, // 13
		TRACK_HAND_IK_CONTROLS_NON_BLENDABLE_2, // 14
		TRACK_HAND_IK_CONTROLS_NON_BLENDABLE_3, // 15
		TRACK_DOCKING_CONTROLS, // 16
		TRACK_AI_CONTROL_CONTROLS_BLENDABLE, // 17
		TRACK_AI_CONTROL_CONTROLS_NON_BLENDABLE, // 18
		NUM_STANDARD_TRACKS, // 19
	};

	enum class TrackTypes
	{
		TRACK_TYPE_REAL, // 0
		TRACK_TYPE_QSTRANSFORM, // 1
		TRACK_TYPE_BINARY, // 2
	};

	enum class TrackFlags
	{
		TRACK_FLAG_ADDITIVE_POSE = 1,
		TRACK_FLAG_PALETTE = 1 << 1,
		TRACK_FLAG_SPARSE = 1 << 2,
	};

	struct TrackHeader
	{
		hkInt16 m_capacity; // 00
		hkInt16 m_numData; // 02
		hkInt16 m_dataOffset; // 04
		hkInt16 m_elementSizeBytes; // 06
		hkReal m_onFraction; // 08
		hkFlags<TrackFlags, hkInt8> m_flags; // 0C
		hkEnum<TrackTypes, hkInt8> m_type; // 0D
	};
	static_assert(sizeof(TrackHeader) == 0x10);

	struct TrackMasterHeader
	{
		hkInt32 m_numBytes; // 00
		hkInt32 m_numTracks; // 04
		hkInt8 m_unused[8]; // 08
	};

	struct Tracks
	{
		struct TrackMasterHeader m_masterHeader; // 00
		struct TrackHeader m_trackHeaders[1]; // 10
	};

	struct Track
	{
		TrackHeader* m_header; // 00
		hkReal* m_data; // 08
	};

	struct Tracks* m_tracks; // 00
	bool m_deleteTracks; // 08
};

struct hkbBindable : hkReferencedObject
{
	hkRefPtr<struct hkbVariableBindingSet> variableBindingSet;  // 10
	hkArray<hkRefVariant>           cachedBindables;     // 18
	bool                            areBindablesCached;  // 28
	std::uint8_t                    pad29;               // 29
	std::uint16_t                   pad2A;               // 2A
	std::uint32_t                   pad2C;               // 2C
};
static_assert(sizeof(hkbBindable) == 0x30);

struct hkbNode : hkbBindable
{
	std::uint32_t                              userData;    // 30
	std::uint32_t                              pad34;       // 34
	hkStringPtr                                name;        // 38
	std::uint16_t                              id;          // 40
	std::uint8_t                               cloneState;  // 42
	std::uint8_t                               pad43;       // 43
	std::uint32_t                              pad44;       // 44
};
static_assert(sizeof(hkbNode) == 0x48);

struct hkbGenerator : hkbNode {};
struct hkbBehaviorGraph : hkbGenerator {};

struct BShkbAnimationGraph
{
	virtual ~BShkbAnimationGraph();
	virtual bool HasRagdollInterface();
	virtual bool AddRagdollToWorld();
	virtual bool RemoveRagdollFromWorld();

	UInt8 unk08[0xC0 - 0x08];
	hkbCharacter character; // C0
	UInt8 unk160[0x208 - 0x160];
	hkbBehaviorGraph *behaviorGraph; // 208
	Actor *holder; // 210
	NiNode *rootNode; // 218
	UInt8 unk220[0x238 - 0x220];
	bhkWorld *world; // 238
	// more...
};
static_assert(offsetof(BShkbAnimationGraph, character) == 0xC0);
static_assert(offsetof(BShkbAnimationGraph, holder) == 0x210);

template <class T, UInt32 N = 1>
struct BSTSmallArray
{
	union Data
	{
		T* heap;
		T local[N];
	};

	SInt32 heapSize = 0x80000000; // 00
	UInt32 unk04; // 04
	Data data; // 08
	UInt32 size;
};

class BSAnimationGraphManager :
	BSTEventSink<BSAnimationGraphEvent>, // 00
	BSIntrusiveRefCounted // 08
{
public:
	UInt8 unk10[0x40 - 0x10];
	BSTSmallArray<BSTSmartPointer<BShkbAnimationGraph>> graphs; // 40
	UInt8 unk58[0x98 - 0x58];
	SimpleLock updateLock; // 98
	SimpleLock dependentManagerLock; // A0
	UInt32 activeGraph; // A8
	UInt32 generateDepth; // A8
};
static_assert(offsetof(BSAnimationGraphManager, graphs) == 0x40);
static_assert(offsetof(BSAnimationGraphManager, updateLock) == 0x98);

typedef bool(*_hkConstraintCinfo_setConstraintData)(struct hkConstraintCinfo *_this, hkpConstraintData *data);
RelocAddr<_hkConstraintCinfo_setConstraintData> hkConstraintCinfo_setConstraintData(0xE3E6F0);
struct hkConstraintCinfo
{
	~hkConstraintCinfo() {
		hkConstraintCinfo_setConstraintData(this, nullptr);
	}

	void *vtbl = 0; // 00
	havokRefPtr<hkpConstraintData> constraintData = 0; // 08
	UInt32 unk10 = 0;
	UInt32 unk14 = 0;
	hkpRigidBody *rigidBodyA = nullptr; // 18
	hkpRigidBody *rigidBodyB = nullptr; // 20
};

auto hkMalleableConstraintCinfo_vtbl = RelocAddr<void *>(0x182C5F8);
struct hkMalleableConstraintCinfo : hkConstraintCinfo
{
	hkMalleableConstraintCinfo()
	{
		this->vtbl = hkMalleableConstraintCinfo_vtbl;
	}
};

struct hkFixedConstraintCinfo : hkConstraintCinfo
{
	UInt64 unk28 = 0;
	UInt8 unk30 = 0; // type or something
};

auto bhkMalleableConstraint_vtbl = RelocAddr<void *>(0x182C628);
struct bhkMalleableConstraint : bhkConstraint
{
	UInt64 unk18 = 0;
};
static_assert(sizeof(bhkMalleableConstraint) == 0x20);

struct AIProcessManager
{
	UInt8  unk000;                   // 008
	bool   enableDetection;          // 001 
	bool   unk002;                   // 002 
	UInt8  unk003;                   // 003
	UInt32 unk004;                   // 004
	bool   enableHighProcess;        // 008 
	bool   enableLowProcess;         // 009 
	bool   enableMiddleHighProcess;  // 00A 
	bool   enableMiddleLowProcess;   // 00B 
	bool   enableAISchedules;        // 00C 
	UInt8  unk00D;                   // 00D
	UInt8  unk00E;                   // 00E
	UInt8  unk00F;                   // 00F
	SInt32 numActorsInHighProcess;   // 010
	UInt32 unk014[(0x30 - 0x014) / sizeof(UInt32)];
	tArray<UInt32>  actorsHigh; // 030 
	tArray<UInt32>  actorsLow;  // 048 
	tArray<UInt32>  actorsMiddleLow; // 060
	tArray<UInt32>  actorsMiddleHigh; // 078
	UInt32  unk90[(0xF0 - 0x7C) / sizeof(UInt32)];
	tArray<void*> activeEffectShaders; // 108
									   //mutable BSUniqueLock			 activeEffectShadersLock; // 120
};

RelocPtr<AIProcessManager *> g_aiProcessManager(0x01F831B0);


typedef bool(*_bhkRefObject_ctor)(bhkRefObject *_this);
RelocAddr<_bhkRefObject_ctor> bhkRefObject_ctor(0xE306A0);

typedef bool(*_hkMalleableConstraintCinfo_Func4)(hkMalleableConstraintCinfo *_this);
RelocAddr<_hkMalleableConstraintCinfo_Func4> hkMalleableConstraintCinfo_Func4(0xE3DD20);

typedef bool(*_hkMalleableConstraintCinfo_setWrappedConstraintData)(hkMalleableConstraintCinfo *_this, hkpConstraintData *data);
RelocAddr<_hkMalleableConstraintCinfo_setWrappedConstraintData> hkMalleableConstraintCinfo_setWrappedConstraintData(0xE3DE70);

typedef bool(*_hkMalleableConstraintCinfo_setStrength)(hkMalleableConstraintCinfo *_this, float strength);
RelocAddr<_hkMalleableConstraintCinfo_setStrength> hkMalleableConstraintCinfo_setStrength(0xE3DE90);

typedef bool(*_Actor_IsInRagdollState)(Actor *_this);
RelocAddr<_Actor_IsInRagdollState> Actor_IsInRagdollState(0x5EBA50);

typedef void(*_BSAnimationGraphManager_HasRagdollInterface)(BSAnimationGraphManager *_this, bool *out);
RelocAddr<_BSAnimationGraphManager_HasRagdollInterface> BSAnimationGraphManager_HasRagdollInterface(0x20B320);

typedef void(*_BSAnimationGraphManager_AddRagdollToWorld)(BSAnimationGraphManager *_this, bool *a1);
RelocAddr<_BSAnimationGraphManager_AddRagdollToWorld> BSAnimationGraphManager_AddRagdollToWorld(0x5D1B50);

typedef void(*_BSAnimationGraphManager_RemoveRagdollFromWorld)(BSAnimationGraphManager *_this, bool *a1);
RelocAddr<_BSAnimationGraphManager_RemoveRagdollFromWorld> BSAnimationGraphManager_RemoveRagdollFromWorld(0x61A1E0);

typedef void(*_NiNode_AddOrRemoveMalleableConstraints)(NiNode *_this, bool a1, bool a2, bool a3);
RelocAddr<_NiNode_AddOrRemoveMalleableConstraints> NiNode_AddOrRemoveMalleableConstraints(0xE09CA0);

typedef void(*_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints)(BSAnimationGraphManager *_this, bool *a1);
RelocAddr<_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints> BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints(0x61A4E0);

typedef hkaRagdollInstance * (*_hkbRagdollDriver_getRagdollInterface)(hkbRagdollDriver *_this);
RelocAddr<_hkbRagdollDriver_getRagdollInterface> hkbRagdollDriver_getRagdollInterface(0xA25860);

typedef bhkConstraint * (*_ConstraintToFixedConstraint)(bhkConstraint *constraint, float strength, bool a3);
RelocAddr<_ConstraintToFixedConstraint> ConstraintToFixedConstraint(0xE63A20);

typedef void(*_hkpConstraintInstance_setEnabled)(hkpConstraintInstance *_this, bool enable);
RelocAddr<_hkpConstraintInstance_setEnabled> hkpConstraintInstance_setEnabled(0xAC06A0);

typedef bool(*_hkpConstraintInstance_isEnabled)(hkpConstraintInstance *_this, bool *enabled);
RelocAddr<_hkpConstraintInstance_isEnabled> hkpConstraintInstance_isEnabled(0xAC06D0);

typedef bool(*_hkpCollisionCallbackUtil_requireCollisionCallbackUtil)(hkpWorld *world);
RelocAddr<_hkpCollisionCallbackUtil_requireCollisionCallbackUtil> hkpCollisionCallbackUtil_requireCollisionCallbackUtil(0xAB8700);

typedef bool(*_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil)(hkpWorld *world);
RelocAddr<_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil> hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(0xB00C30);

typedef hkpWorldExtension * (*_hkpWorld_findWorldExtension)(hkpWorld *world, int id);
RelocAddr<_hkpWorld_findWorldExtension> hkpWorld_findWorldExtension(0xAB58F0);

typedef bool(*_IAnimationGraphManagerHolder_GetAnimationGraphManagerImpl)(IAnimationGraphManagerHolder *_this, BSTSmartPointer<BSAnimationGraphManager>& a_out);


std::unordered_map<UInt32, int> g_hitHandleCounts;
struct ContactListener : hkpContactListener
{
	virtual void contactPointCallback(const hkpContactPointEvent& evnt) override
	{
		return;

		if (evnt.m_contactPointProperties && (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED)) {
			// Early out
			return;
		}

		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body with our custom layer (hand, held object...)

		if (layerA == 56 && layerB == 56) return; // Both objects are on our custom layer

		hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;

		hkpMotion::MotionType motionType = hitRigidBody->getMotionType();

		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (!hitRefr) return;

		if (DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor)) {
			//g_hitActorHandle = GetOrCreateRefrHandle(hitRefr);
			//_MESSAGE("Hit actor");
		}

		//_MESSAGE("Collision");
	}

	virtual void collisionAddedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body with our custom layer (hand, held object...)

		if (layerA == 56 && layerB == 56) return; // Both objects are on our custom layer

		hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;

		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (!hitRefr) return;

		if (DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor)) {
			UInt32 handle = GetOrCreateRefrHandle(hitRefr);
			if (g_hitHandleCounts.count(handle) == 0) g_hitHandleCounts[handle] = 0;
			++g_hitHandleCounts[handle];

			//_MESSAGE("Collision added");
		}
	}

	virtual void collisionRemovedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body with our custom layer (hand, held object...)

		if (layerA == 56 && layerB == 56) return; // Both objects are on our custom layer

		hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;

		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (!hitRefr) return;

		if (DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor)) {
			UInt32 handle = GetOrCreateRefrHandle(hitRefr);

			//ASSERT_STR(g_hitHandleCounts.count(handle) != 0, "Received collision removed callback with no corresponding added callback");
			if (g_hitHandleCounts.count(handle) != 0) {
				int count = g_hitHandleCounts[handle];
				if (count <= 1) {
					g_hitHandleCounts.erase(handle);
				}
				else {
					g_hitHandleCounts[handle] = count - 1;
				}

				//_MESSAGE("Collision removed");
			}
		}
	}

	NiPointer<bhkWorld> world = nullptr;
};
ContactListener *contactListener = nullptr;


void bhkMalleableConstraint_ctor(bhkMalleableConstraint *_this, hkMalleableConstraintCinfo *cInfo)
{
	// Construct bhkMalleableConstraint
	bhkRefObject_ctor(_this);
	_this->unk18 = 0;
	*((void **)_this) = ((void *)(bhkMalleableConstraint_vtbl)); // set vtbl
	_this->InitHavokFromCinfo((void *)((UInt64)cInfo + 8)); // need to pass in cInfo + 8 for whatever reason. Within the function it subtracts 8 again...
}

bhkMalleableConstraint * CreateMalleableConstraint(bhkConstraint *constraint, float strength)
{
	if (DYNAMIC_CAST(constraint, bhkConstraint, bhkMalleableConstraint)) return nullptr; // already a malleable constraint

	hkMalleableConstraintCinfo cInfo;
	hkMalleableConstraintCinfo_Func4(&cInfo);
	hkMalleableConstraintCinfo_setWrappedConstraintData(&cInfo, constraint->constraint->m_data);
	cInfo.rigidBodyA = constraint->constraint->getRigidBodyA();
	cInfo.rigidBodyB = constraint->constraint->getRigidBodyB();
	hkMalleableConstraintCinfo_setStrength(&cInfo, strength);

	bhkMalleableConstraint *malleableConstraint = (bhkMalleableConstraint *)Heap_Allocate(sizeof(bhkMalleableConstraint));
	if (malleableConstraint) {
		bhkMalleableConstraint_ctor(malleableConstraint, &cInfo);
		return malleableConstraint;
	}

	return nullptr;
}

inline bool GetAnimationGraphManager(Actor *actor, BSTSmartPointer<BSAnimationGraphManager> &out)
{
	IAnimationGraphManagerHolder *animGraphManagerHolder = &actor->animGraphHolder;
	UInt64 *vtbl = *((UInt64 **)animGraphManagerHolder);
	return ((_IAnimationGraphManagerHolder_GetAnimationGraphManagerImpl)(vtbl[0x02]))(animGraphManagerHolder, out);
}

bool IsAddedToWorld(Actor *actor)
{
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
	if (!GetAnimationGraphManager(actor, animGraphManager)) return true;

	BSAnimationGraphManager *manager = animGraphManager.ptr;

	SimpleLocker lock(&manager->updateLock);

	for (int i = 0; i < manager->graphs.size; i++) {
		BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.heapSize >= 0 ? manager->graphs.data.heap[i] : manager->graphs.data.local[i];

		hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
		if (!driver) return false;
		hkaRagdollInstance *ragdoll = driver->ragdoll;
		if (!ragdoll) return false;
		if (!ragdoll->getWorld()) return false;
	}

	return true;
}

void ModifyConstraints(Actor *actor)
{
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
	if (!GetAnimationGraphManager(actor, animGraphManager)) return;

	BSAnimationGraphManager *manager = animGraphManager.ptr;

	SimpleLocker lock(&manager->updateLock);

	for (int i = 0; i < manager->graphs.size; i++) {
		BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.heapSize >= 0 ? manager->graphs.data.heap[i] : manager->graphs.data.local[i];

		hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
		if (!driver) return;
		hkaRagdollInstance *ragdoll = hkbRagdollDriver_getRagdollInterface(driver);
		if (!ragdoll) return;

		// Make sure bones are NOT keyframed to begin with, so that they get properly set in drivetopose to keyframed reporting.
		// Actually just set them all here anyways, just in case.
		for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
			NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable);
			if (node) {
				//if (std::string(node->m_name) == "NPC Head [Head]" || std::string(node->m_name) == "NPC R Hand [RHnd]" || std::string(node->m_name) == "NPC L Hand [LHnd]") {
					NiPointer<bhkRigidBody> wrapper = GetRigidBody(node);
					if (wrapper) {
						//bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
						bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_KEYFRAMED, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
						rigidBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
						bhkWorldObject_UpdateCollisionFilter(wrapper);
					}
				//}
			}
		}

		// Reduce strength of all constraints by wrapping them in malleable constraints with lower strength
		for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
			NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable);
			if (node) {
				NiPointer<bhkRigidBody> wrapper = GetRigidBody(node);
				if (wrapper) {
					for (int i = 0; i < wrapper->constraints.count; i++) {
						bhkConstraint *constraint = wrapper->constraints.entries[i];
						//bhkConstraint *fixedConstraint = ConstraintToFixedConstraint(constraint, 0.3f, false);
						bhkConstraint *malleableConstraint = CreateMalleableConstraint(constraint, 0.3f);
						if (malleableConstraint) {
							constraint->RemoveFromCurrentWorld();

							hkpWorld *hkWorld = wrapper->GetHavokWorld_1();
							bhkWorld *world = ((ahkpWorld*)hkWorld)->m_userData;
							malleableConstraint->MoveToWorld(world);
							wrapper->constraints.entries[i] = malleableConstraint;
						}
					}
				}
			}
		}
	}
}

bool AddRagdollToWorld(Actor *actor)
{
	bool isSittingOrSleepingOrMounted = actor->actorState.flags04 & 0x3C000;
	if (isSittingOrSleepingOrMounted || Actor_IsInRagdollState(actor)) return false;

	bool hasRagdollInterface = false;
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
	if (GetAnimationGraphManager(actor, animGraphManager)) {
		BSAnimationGraphManager_HasRagdollInterface(animGraphManager.ptr, &hasRagdollInterface);
	}

	if (hasRagdollInterface) {
		if (GetAnimationGraphManager(actor, animGraphManager)) {
			bool x = false;
			BSAnimationGraphManager_AddRagdollToWorld(animGraphManager.ptr, &x);

			ModifyConstraints(actor);

			x = false;
			BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints(animGraphManager.ptr, &x);
		}
	}

	return true;
}

bool RemoveRagdollFromWorld(Actor *actor)
{
	bool isSittingOrSleepingOrMounted = actor->actorState.flags04 & 0x3C000;
	if (isSittingOrSleepingOrMounted || Actor_IsInRagdollState(actor)) return false;

	bool hasRagdollInterface = false;
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
	if (GetAnimationGraphManager(actor, animGraphManager)) {
		BSAnimationGraphManager_HasRagdollInterface(animGraphManager.ptr, &hasRagdollInterface);
	}

	if (hasRagdollInterface) {
		if (GetAnimationGraphManager(actor, animGraphManager)) {
			bool x = false;
			BSAnimationGraphManager_RemoveRagdollFromWorld(animGraphManager.ptr, &x);

			//x = false;
			//BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints(animGraphManager.ptr, &x);
		}
	}

	return true;
}

void VisitNodes(NiAVObject *obj, std::function<void(NiAVObject *)> f)
{
	f(obj);

	NiNode *node = obj->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				VisitNodes(child, f);
			}
		}
	}
}

std::unordered_set<hkbRagdollDriver *> g_hitDrivers;
void ProcessHavokHitJobsHook()
{
	if (!initComplete) return;

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return;

	TESObjectCELL *cell = player->parentCell;
	if (!cell) return;

	NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell);
	if (!world) return;

	AIProcessManager *processManager = *g_aiProcessManager;
	if (!processManager) return;

	if (world != contactListener->world) {
		bhkWorld *oldWorld = contactListener->world;
		if (oldWorld) {
			_MESSAGE("Removing listener from old havok world");
			{
				BSWriteLocker lock(&oldWorld->worldLock);
				hkpWorldExtension *collisionCallbackExtension = hkpWorld_findWorldExtension(world->world, hkpKnownWorldExtensionIds::HK_WORLD_EXTENSION_COLLISION_CALLBACK);
				if (collisionCallbackExtension) {
					// There are times when the collision callback extension is gone even if we required it earlier...
					hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(world->world);
				}
				hkpWorld_removeContactListener(oldWorld->world, contactListener);
			}
		}

		_MESSAGE("Adding listener to new havok world");
		{
			BSWriteLocker lock(&world->worldLock);
			hkpCollisionCallbackUtil_requireCollisionCallbackUtil(world->world);
			hkpWorld_addContactListener(world->world, contactListener);
		}

		contactListener->world = world;
	}

	if (g_hitHandleCounts.size() > 0) {
		for (auto[handle, count] : g_hitHandleCounts) {
			//_MESSAGE("%d:\t%d", handle, count);
			NiPointer<TESObjectREFR> refr;
			UInt32 handleCopy = handle;
			if (LookupREFRByHandle(handleCopy, refr)) {
				Actor *hitActor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
				if (hitActor) {
					BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
					if (GetAnimationGraphManager(hitActor, animGraphManager)) {
						BSAnimationGraphManager *manager = animGraphManager.ptr;
						SimpleLocker lock(&manager->updateLock);
						for (int i = 0; i < manager->graphs.size; i++) {
							BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.heapSize >= 0 ? manager->graphs.data.heap[i] : manager->graphs.data.local[i];
							hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
							if (driver) {
								g_hitDrivers.insert(driver);
							}
						}
					}
				}
			}
		}
	}
	else {
		g_hitDrivers.clear();
	}

	// if (!g_enableRagdoll) return;

	for (UInt32 i = 0; i < processManager->actorsHigh.count; i++) {
		UInt32 actorHandle = processManager->actorsHigh[i];
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(actorHandle, refr) && refr != player) {
			Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
			if (!actor || !actor->GetNiNode()) continue;

			TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);

			bool shouldAddToWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale < 3.f;

			bool isAddedToWorld = IsAddedToWorld(actor);
			//_MESSAGE("%x %d", actor, isAddedToWorld);

			if (!isAddedToWorld && shouldAddToWorld) {
				AddRagdollToWorld(actor);
			}
			else if (isAddedToWorld && !shouldAddToWorld) {
				RemoveRagdollFromWorld(actor);
			}
		}
	}
}

inline hkReal * Track_getData(hkbGeneratorOutput &output, hkbGeneratorOutput::TrackHeader &header)
{
	return reinterpret_cast<hkReal*>(reinterpret_cast<char*>(output.m_tracks) + header.m_dataOffset);
}

inline hkInt8* Track_getIndices(hkbGeneratorOutput &output, hkbGeneratorOutput::TrackHeader &header)
{
	// must be sparse or pallette track
	int numDataBytes = HK_NEXT_MULTIPLE_OF(16, header.m_elementSizeBytes * header.m_capacity);
	return reinterpret_cast<hkInt8*>(Track_getData(output, header)) + numDataBytes;
}

void TryForceControls(hkbGeneratorOutput &output, hkbGeneratorOutput::TrackHeader &header)
{
	if (header.m_capacity > 0) {
		hkaKeyFrameHierarchyUtility::ControlData *data = (hkaKeyFrameHierarchyUtility::ControlData *)(Track_getData(output, header));
		data[0] = hkaKeyFrameHierarchyUtility::ControlData();

		hkInt8 *indices = Track_getIndices(output, header);
		for (int i = 0; i < header.m_capacity; i++) {
			indices[i] = 0;
		}

		header.m_numData = 1;
		header.m_onFraction = 1.f;
	}
}

void SetBonesKeyframedReporting(hkbRagdollDriver *driver, hkbGeneratorOutput& generatorOutput, hkbGeneratorOutput::TrackHeader &header)
{
	// - Set onFraction > 1.0f
	// - Set value of keyframed bones tracks to > 1.0f for bones we want keyframed, <= 1.0f for bones we don't want keyframed. Index of track data == index of bone.
	// - Set reportingWhenKeyframed in the ragdoll driver for the bones we care about

	header.m_onFraction = 1.1f;
	hkReal* data = Track_getData(generatorOutput, header);
	const hkaSkeleton *skeleton = driver->ragdoll->m_skeleton;
	for (int i = 0; i < skeleton->m_bones.getSize(); i++) { // TODO: We need to check the capacity of this track to see if we can fit all the bones? What about numData?
		//const hkaBone &bone = skeleton->m_bones[i];
		//if (std::string(bone.m_name.cString()) == "Ragdoll_NPC Head [Head]" || std::string(bone.m_name.cString()) == "Ragdoll_NPC L Hand [LHnd]" || std::string(bone.m_name.cString()) == "Ragdoll_NPC R Hand [RHnd]") {
		data[i] = 1.1f;
		// Indexed by (boneIdx >> 5), and then you >> (boneIdx & 0x1F) & 1 to extract the specific bit
		driver->reportingWhenKeyframed[i >> 5] |= (1 << (i & 0x1F));
		//}
	}
}

void PreDriveToPoseHook(hkbRagdollDriver *driver, UInt64 pad, const hkbContext& context, hkbGeneratorOutput& generatorOutput, hkReal deltaTime)
{
	int keyframedBonesTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_KEYFRAMED_RAGDOLL_BONES;
	int rigidBodyControlsTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_RIGID_BODY_RAGDOLL_CONTROLS;
	int poweredControlsTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POWERED_RAGDOLL_CONTROLS;

	hkInt32 numTracks = generatorOutput.m_tracks->m_masterHeader.m_numTracks;

	hkbGeneratorOutput::TrackHeader *keyframedBonesHeader = numTracks > keyframedBonesTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[keyframedBonesTrackId]) : nullptr;
	hkbGeneratorOutput::TrackHeader *rigidBodyHeader = numTracks > rigidBodyControlsTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[rigidBodyControlsTrackId]) : nullptr;
	hkbGeneratorOutput::TrackHeader *poweredHeader = numTracks > poweredControlsTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[poweredControlsTrackId]) : nullptr;

	if (keyframedBonesHeader && keyframedBonesHeader->m_onFraction > 0.f && g_hitDrivers.count(driver) == 0) { // Don't keyframe bones if we've collided with the actor
		SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);
	}

	bool isRigidBodyOn = rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f;
	bool isPoweredOn = poweredHeader && poweredHeader->m_onFraction > 0.f;

	if (!isRigidBodyOn && !isPoweredOn) {
		// No controls are active - try and force it to use the rigidbody controller
		TryForceControls(generatorOutput, *rigidBodyHeader);
	}
}

void PostDriveToPoseHook(hkbRagdollDriver *driver)
{
	return;


	// This hook is called right after hkbRagdollDriver::driveToPose()

	hkaRagdollInstance *ragdoll = hkbRagdollDriver_getRagdollInterface(driver);
	if (!ragdoll) return;
	if (!ragdoll->getWorld()) return;

	// Make head / hands keyframed to guarantee head tracking + hand positioning (for weapons, etc. - especially two-handed things)
	for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
		NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable);
		if (node) {
			if (std::string(node->m_name) == "NPC Head [Head]" || std::string(node->m_name) == "NPC R Hand [RHnd]" || std::string(node->m_name) == "NPC L Hand [LHnd]") {
				NiPointer<bhkRigidBody> wrapper = GetRigidBody(node);
				if (wrapper && wrapper->hkBody->m_motion.m_type != hkpMotion::MotionType::MOTION_KEYFRAMED) {
					//bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_KEYFRAMED, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
					//wrapper->hkBody->m_collidable.setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
					//bhkWorld_UpdateCollisionFilterOnEntity(((ahkpWorld *)ragdoll->getWorld())->m_userData, wrapper->hkBody);
				}
			}
		}
	}

	for (hkpConstraintInstance *constraint : ragdoll->m_constraints) {
		bool x;
		hkpConstraintInstance_isEnabled(constraint, &x);
		if (x) {
			//hkpConstraintInstance_setEnabled(constraint, false);
		}
	}
}


uintptr_t processHavokHitJobsHookedFuncAddr = 0;
auto processHavokHitJobsHookLoc = RelocAddr<uintptr_t>(0x6497E4);
auto processHavokHitJobsHookedFunc = RelocAddr<uintptr_t>(0x75AC20);

uintptr_t driveToPoseHookedFuncAddr = 0;
auto driveToPoseHookLoc = RelocAddr<uintptr_t>(0xB266AB);
auto driveToPoseHookedFunc = RelocAddr<uintptr_t>(0xA25B60);

hkbRagdollDriver *g_ragdollDriver = nullptr;

void PerformHooks(void)
{
	// First, set our addresses
	processHavokHitJobsHookedFuncAddr = processHavokHitJobsHookedFunc.GetUIntPtr();
	driveToPoseHookedFuncAddr = driveToPoseHookedFunc.GetUIntPtr();

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Original code
				mov(rax, processHavokHitJobsHookedFuncAddr);
				call(rax);

				push(rax);
				sub(rsp, 0x38); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space
				movsd(ptr[rsp + 0x20], xmm0);

				// Call our hook
				mov(rax, (uintptr_t)ProcessHavokHitJobsHook);
				call(rax);

				movsd(xmm0, ptr[rsp + 0x20]);
				add(rsp, 0x38);
				pop(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(processHavokHitJobsHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(processHavokHitJobsHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("ProcessHavokHitJobs hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Save ragdoll driver
				push(rax);
				mov(rax, (uintptr_t)&g_ragdollDriver);
				mov(ptr[rax], rcx);
				pop(rax);

				push(rax);
				push(rcx);
				push(rdx);
				push(r8);
				push(r9);
				push(r10);
				push(r11);
				sub(rsp, 0x88); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space
				movsd(ptr[rsp + 0x20], xmm0);
				movsd(ptr[rsp + 0x30], xmm1);
				movsd(ptr[rsp + 0x40], xmm2);
				movsd(ptr[rsp + 0x50], xmm3);
				movsd(ptr[rsp + 0x60], xmm4);
				movsd(ptr[rsp + 0x70], xmm5);

				// Call our pre hook
				mov(rax, (uintptr_t)PreDriveToPoseHook);
				call(rax);

				movsd(xmm0, ptr[rsp + 0x20]);
				movsd(xmm1, ptr[rsp + 0x30]);
				movsd(xmm2, ptr[rsp + 0x40]);
				movsd(xmm3, ptr[rsp + 0x50]);
				movsd(xmm4, ptr[rsp + 0x60]);
				movsd(xmm5, ptr[rsp + 0x70]);
				add(rsp, 0x88);
				pop(r11);
				pop(r10);
				pop(r9);
				pop(r8);
				pop(rdx);
				pop(rcx);
				pop(rax);

				// Original code
				mov(rax, driveToPoseHookedFuncAddr);
				call(rax);

				// Restore original ragdoll driver to pass to our hook
				mov(rax, (uintptr_t)&g_ragdollDriver);
				mov(rcx, ptr[rax]);

				// Call our post hook
				mov(rax, (uintptr_t)PostDriveToPoseHook);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(driveToPoseHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(driveToPoseHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("hkbRagdollDriver::driveToPose hook complete");
	}
}

bool TryHook()
{
	// This should be sized to the actual amount used by your trampoline
	static const size_t TRAMPOLINE_SIZE = 512;

	if (g_trampoline) {
		void* branch = g_trampoline->AllocateFromBranchPool(g_pluginHandle, TRAMPOLINE_SIZE);
		if (!branch) {
			_ERROR("couldn't acquire branch trampoline from SKSE. this is fatal. skipping remainder of init process.");
			return false;
		}

		g_branchTrampoline.SetBase(TRAMPOLINE_SIZE, branch);

		void* local = g_trampoline->AllocateFromLocalPool(g_pluginHandle, TRAMPOLINE_SIZE);
		if (!local) {
			_ERROR("couldn't acquire codegen buffer from SKSE. this is fatal. skipping remainder of init process.");
			return false;
		}

		g_localTrampoline.SetBase(TRAMPOLINE_SIZE, local);
	}
	else {
		if (!g_branchTrampoline.Create(TRAMPOLINE_SIZE)) {
			_ERROR("couldn't create branch trampoline. this is fatal. skipping remainder of init process.");
			return false;
		}
		if (!g_localTrampoline.Create(TRAMPOLINE_SIZE, nullptr))
		{
			_ERROR("couldn't create codegen buffer. this is fatal. skipping remainder of init process.");
			return false;
		}
	}

	PerformHooks();
	return true;
}


extern "C" {
	void OnDataLoaded()
	{
		contactListener = new ContactListener;

		initComplete = true;
		_MESSAGE("Successfully loaded all forms");
	}

	void OnInputLoaded()
	{

	}

	// Listener for SKSE Messages
	void OnSKSEMessage(SKSEMessagingInterface::Message* msg)
	{
		if (msg) {
			if (msg->type == SKSEMessagingInterface::kMessage_InputLoaded) {
				OnInputLoaded();
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_DataLoaded) {
				OnDataLoaded();
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoad) {
				// Get the VRIK plugin API
				//g_vrikInterface = vrikPluginApi::getVrikInterface001(g_pluginHandle, g_messaging);
			}
		}
	}

	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\MeleeVR.log");
		gLog.SetPrintLevel(IDebugLog::kLevel_DebugMessage);
		gLog.SetLogLevel(IDebugLog::kLevel_DebugMessage);

		_MESSAGE("MeleeVR v%s", DWBVR_VERSION_VERSTRING);

		info->infoVersion = PluginInfo::kInfoVersion;
		info->name = "ForcePullVR";
		info->version = DWBVR_VERSION_MAJOR;

		g_pluginHandle = skse->GetPluginHandle();

		if (skse->isEditor) {
			_FATALERROR("[FATAL ERROR] Loaded in editor, marking as incompatible!\n");
			return false;
		}
		else if (skse->runtimeVersion != RUNTIME_VR_VERSION_1_4_15) {
			_FATALERROR("[FATAL ERROR] Unsupported runtime version %08X!\n", skse->runtimeVersion);
			return false;
		}

		return true;
	}

	bool SKSEPlugin_Load(const SKSEInterface * skse)
	{	// Called by SKSE to load this plugin
		_MESSAGE("MeleeVR loaded");

		_MESSAGE("Registering for SKSE messages");
		g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
		g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

		g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
		if (!g_trampoline) {
			_WARNING("Couldn't get trampoline interface");
		}
		if (!TryHook()) {
			_ERROR("[CRITICAL] Failed to perform hooks");
			return false;
		}

		return true;
	}
};
