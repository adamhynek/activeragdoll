#include <functional>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <atomic>
#include <set>

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
#include "higgsinterface001.h"

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
#include <Physics/Utilities/CharacterControl/CharacterRigidBody/hkpCharacterRigidBodyListener.h>
#include <Physics/Utilities/CharacterControl/CharacterRigidBody/hkpCharacterRigidBody.h>
#include <Physics/Collide/Query/Collector/PointCollector/hkpAllCdPointCollector.h>
#include <Physics/Utilities/CharacterControl/StateMachine/hkpCharacterState.h>
#include <Physics/Utilities/CharacterControl/StateMachine/hkpCharacterContext.h>


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
static_assert(offsetof(hkbCharacter, behaviorGraph) == 0x58);
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
	UInt64 userData;    // 30
	hkStringPtr                                name;        // 38
	std::uint16_t                              id;          // 40
	std::uint8_t                               cloneState;  // 42
	std::uint8_t                               pad43;       // 43
	std::uint32_t                              pad44;       // 44
};
static_assert(offsetof(hkbNode, userData) == 0x30);
static_assert(sizeof(hkbNode) == 0x48);

struct hkbGenerator : hkbNode {};
struct hkbBehaviorGraph : hkbGenerator {};

struct bhkCharacterController : NiRefObject
{
	UInt8 unk10[0x70 - 0x10];
	hkVector4                                        forwardVec;                 // 070
	hkStepInfo                                       stepInfo;                   // 080
	hkVector4                                        outVelocity;                // 090
	hkVector4                                        initialVelocity;            // 0A0
	hkVector4                                        velocityMod;                // 0B0
	hkVector4                                        direction;                  // 0C0
	hkVector4                                        rotCenter;                  // 0D0
	hkVector4                                        pushDelta;                  // 0E0
	hkVector4                                        fakeSupportStart;           // 0F0
	hkVector4                                        up;                         // 100
	hkVector4                                        supportNorm;                // 110
	UInt8                                          collisionBound[0x150 - 0x120];             // 120
	UInt8                                          bumperCollisionBound[0x180 - 0x150];       // 150
	std::uint64_t                                    unk180;                     // 180
	std::uint64_t                                    unk188;                     // 188
	struct bhkICharOrientationController*                   orientationCtrl;            // 190
	std::uint64_t                                    pad198;                     // 198
	hkpSurfaceInfo                                   surfaceInfo;                // 1A0
	hkpCharacterContext                              context;                    // 1E0
	UInt32 flags;                      // 218
	hkpCharacterStateType                            wantState;                  // 218
	float                                            velocityTime;               // 220
	float                                            rotMod;                     // 224
	float                                            rotModTime;                 // 228
	float                                            calculatePitchTimer;        // 22C
	float                                            acrobatics;                 // 230
	float                                            center;                     // 234
	float                                            waterHeight;                // 238
	float                                            jumpHeight;                 // 23C
	float                                            fallStartHeight;            // 240
	float                                            fallTime;                   // 244
	float                                            gravity;                    // 248
	float                                            pitchAngle;                 // 24C
	float                                            rollAngle;                  // 250
	float                                            pitchMult;                  // 254
	float                                            scale;                      // 258
	float                                            swimFloatHeight;            // 25C
	float                                            actorHeight;                // 260
	float                                            speedPct;                   // 264
	std::uint32_t                                    pushCount;                  // 268
	std::uint32_t                                    unk26C;                     // 26C
	std::uint64_t                                    unk270;                     // 270
	std::uint64_t                                    unk278;                     // 278
	NiPointer<bhkShape>                              shapes[2];                  // 280
	std::uint64_t                                    unk290;                     // 290
	std::uint64_t                                    unk298;                     // 298
	std::uint64_t                                    unk2A0;                     // 2A0
	std::uint64_t                                    unk2A8;                     // 2A8
	hkRefPtr<hkpRigidBody>                           supportBody;                // 2B0
	float                                            bumpedForce;                // 2B8
	std::uint32_t                                    pad2BC;                     // 2BC
	hkRefPtr<hkpRigidBody>                           bumpedBody;                 // 2C0
	hkRefPtr<hkpRigidBody>                           bumpedCharCollisionObject;  // 2C8
	UInt8                     unk2D0[0x300 - 0x2D0];                     // 2D0
	std::uint64_t                                    unk300;                     // 300
	std::uint64_t                                    unk308;                     // 308
	std::uint64_t                                    unk310;                     // 310
	std::uint64_t                                    unk318;                     // 318
	std::uint64_t                                    unk320;                     // 320
	std::uint64_t                                    unk328;                     // 328
};
static_assert(offsetof(bhkCharacterController, context) == 0x1E0);
static_assert(sizeof(bhkCharacterController) == 0x330);

class bhkCharacterPointCollector : public hkpAllCdPointCollector
{
	UInt64 unk220;  // 220
	UInt64 unk228;  // 228
	UInt64 unk230;  // 230
	UInt64 unk238;  // 238
};
static_assert(sizeof(bhkCharacterPointCollector) == 0x240);

struct bhkCharacterRigidBody : bhkSerializable
{
	hkpCharacterRigidBody *characterRigidBody; // 10
	UInt64 unk18;
	bhkRigidBody *rigidBody; // 20
	NiAVObject *unk28; // 28 - MarkerX ??
	bhkCharacterPointCollector ignoredCollisionStartCollector;  // 30
};
static_assert(offsetof(bhkCharacterRigidBody, ignoredCollisionStartCollector) == 0x30);

struct bhkCharRigidBodyController :
	bhkCharacterController, // 00
	hkpCharacterRigidBodyListener // 330
{
	bhkCharacterRigidBody characterRigidBody; // 340
};
static_assert(offsetof(bhkCharRigidBodyController, characterRigidBody) == 0x340);

struct BShkbAnimationGraph
{
	void *vtbl; // 00
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
RelocPtr<float> g_bAlwaysDriveRagdoll(0x1EBE830);


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

typedef bool * (*_hkpConstraintInstance_isEnabled)(hkpConstraintInstance *_this, bool *enabled);
RelocAddr<_hkpConstraintInstance_isEnabled> hkpConstraintInstance_isEnabled(0xAC06D0);

typedef bool(*_hkpCollisionCallbackUtil_requireCollisionCallbackUtil)(hkpWorld *world);
RelocAddr<_hkpCollisionCallbackUtil_requireCollisionCallbackUtil> hkpCollisionCallbackUtil_requireCollisionCallbackUtil(0xAB8700);

typedef bool(*_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil)(hkpWorld *world);
RelocAddr<_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil> hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(0xB00C30);

typedef hkpWorldExtension * (*_hkpWorld_findWorldExtension)(hkpWorld *world, int id);
RelocAddr<_hkpWorld_findWorldExtension> hkpWorld_findWorldExtension(0xAB58F0);

typedef void(*_ahkpCharacterProxy_setLinearVelocity)(hkpCharacterProxy *_this, const hkVector4& vel);
RelocAddr<_ahkpCharacterProxy_setLinearVelocity> ahkpCharacterProxy_setLinearVelocity(0xAFA1F0);

typedef void(*_ahkpCharacterRigidBody_setLinearVelocity)(hkpCharacterRigidBody *_this, const hkVector4& newVel, hkReal timestep);
RelocAddr<_ahkpCharacterRigidBody_setLinearVelocity> ahkpCharacterRigidBody_setLinearVelocity(0xAF5B20);

typedef hkVector4 & (*_ahkpCharacterRigidBody_getLinearVelocity)(hkpCharacterRigidBody *_this);
RelocAddr<_ahkpCharacterRigidBody_getLinearVelocity> ahkpCharacterRigidBody_getLinearVelocity(0xAF5BB0);

typedef hkVector4 & (*_hkbBlendPoses)(UInt32 numData, const hkQsTransform *src, const hkQsTransform *dst, float amount, hkQsTransform *out);
RelocAddr<_hkbBlendPoses> hkbBlendPoses(0xB4DD80);

typedef bool(*_IAnimationGraphManagerHolder_GetAnimationGraphManagerImpl)(IAnimationGraphManagerHolder *_this, BSTSmartPointer<BSAnimationGraphManager>& a_out);


std::unordered_map<UInt32, int> g_hitHandleCounts;
struct ContactListener : hkpContactListener
{
	std::set<std::pair<hkpRigidBody *, hkpRigidBody*>> activeCollisions;

	std::pair<hkpRigidBody *, hkpRigidBody *> SortPair(hkpRigidBody *a, hkpRigidBody *b) {
		if ((uint64_t)a <= (uint64_t)b) return { a, b };
		else return { b, a };
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

			//hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;
			activeCollisions.insert(SortPair(rigidBodyA, rigidBodyB));

			//_MESSAGE("Collision added");
		}
	}

	virtual void collisionRemovedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		auto pair = SortPair(rigidBodyA, rigidBodyB);
		if (activeCollisions.count(pair)) {
			activeCollisions.erase(pair);

			UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
			hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;

			NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
			if (!hitRefr) return;

			if (DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor)) {
				UInt32 handle = GetOrCreateRefrHandle(hitRefr);

				if (g_hitHandleCounts.count(handle) > 0) {
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
	}

	NiPointer<bhkWorld> world = nullptr;
};
ContactListener *contactListener = nullptr;

enum class RagdollState
{
	Keyframed,
	BlendIn,
	Collide,
	StopCollide,
	BlendOut
};

struct RagdollData
{
	std::vector<hkQsTransform> blendInOutInitialPose;
	double stateChangedTime = 0.0;
	RagdollState state = RagdollState::Keyframed;
	bool isOn = false;
	bool firstFrameBlendInOut = false;
};

std::unordered_map<hkbRagdollDriver *, RagdollData> g_ragdollData;

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

std::unordered_map<hkbRagdollDriver *, hkQsTransform> g_hipBoneTransforms;

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
			BSAnimationGraphManager *manager = animGraphManager.ptr;

			{
				SimpleLocker lock(&manager->updateLock);
				for (int i = 0; i < manager->graphs.size; i++) {
					BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.heapSize >= 0 ? manager->graphs.data.heap[i] : manager->graphs.data.local[i];
					hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
					if (driver) {
						g_hipBoneTransforms.erase(driver);
						g_ragdollData[driver] = RagdollData();
					}
				}
			}

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
			BSAnimationGraphManager *manager = animGraphManager.ptr;

			{
				SimpleLocker lock(&manager->updateLock);
				for (int i = 0; i < manager->graphs.size; i++) {
					BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.heapSize >= 0 ? manager->graphs.data.heap[i] : manager->graphs.data.local[i];
					hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
					if (driver) {
						g_ragdollData.erase(driver);
					}
				}
			}

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

Actor * GetActorFromRagdollDriver(hkbRagdollDriver *driver)
{
	hkbCharacter *character = driver->character;
	if (!character) return nullptr;

	hkbBehaviorGraph *behaviorGraph = character->behaviorGraph;
	if (!behaviorGraph) return nullptr;

	BShkbAnimationGraph *graph = (BShkbAnimationGraph *)behaviorGraph->userData;
	if (!graph) return nullptr;

	return graph->holder;
}

bhkCharRigidBodyController * GetCharRigidBodyController(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return nullptr;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return nullptr;

	NiPointer<bhkCharacterController> controller = *((NiPointer<bhkCharacterController> *)&middleProcess->unk250);
	if (!controller) return nullptr;

	return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharRigidBodyController);
}

std::unordered_set<hkbRagdollDriver *> g_hitDrivers;
std::unordered_set<hkbRagdollDriver *> g_higgsDrivers;

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

	if (g_higgsInterface) {
		if (!g_higgsInterface->GetGrabbedObject(false) && !g_higgsInterface->GetGrabbedObject(true)) {
			g_higgsDrivers.clear();
		}
	}

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

			if (Config::options.enableBipedCollision) {
				bhkCollisionFilter *filter = (bhkCollisionFilter *)world->world->m_collisionFilter;
				filter->layerBitfields[8] |= (1 << 8); // enable biped->biped collision;
			}
		}

		contactListener->world = world;
	}

	if (g_hitDrivers.size() > 0) {
		g_hitDrivers.clear(); // clear first, then add any that are hit. Not thread-safe.
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

	// if (!g_enableRagdoll) return;

	for (UInt32 i = 0; i < processManager->actorsHigh.count; i++) {
		UInt32 actorHandle = processManager->actorsHigh[i];
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(actorHandle, refr) && refr != player) {
			Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
			if (!actor || !actor->GetNiNode()) continue;

			TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);

			bool shouldAddToWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale < Config::options.activeRagdollDistance;

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

int GetAnimBoneIndex(hkbCharacter *character, const std::string &boneName)
{
	hkaSkeleton *animSkeleton = character->setup->m_animationSkeleton;
	for (int i = 0; i < animSkeleton->m_bones.getSize(); i++) {
		const hkaBone &bone = animSkeleton->m_bones[i];
		if (boneName == bone.m_name.cString()) {
			return i;
		}
	}
	return -1;
}

hkaKeyFrameHierarchyUtility::Output g_stressOut[200]; // set in a hook during driveToPose(). Just reserve a bunch of space so it can handle any number of bones.

void PreDriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext& context, hkbGeneratorOutput& generatorOutput)
{
	int poseTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POSE;
	int worldFromModelTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_WORLD_FROM_MODEL;
	int keyframedBonesTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_KEYFRAMED_RAGDOLL_BONES;
	int rigidBodyControlsTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_RIGID_BODY_RAGDOLL_CONTROLS;
	int poweredControlsTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POWERED_RAGDOLL_CONTROLS;

	hkInt32 numTracks = generatorOutput.m_tracks->m_masterHeader.m_numTracks;

	hkbGeneratorOutput::TrackHeader *poseHeader = numTracks > poseTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[poseTrackId]) : nullptr;
	hkbGeneratorOutput::TrackHeader *worldFromModelHeader = numTracks > worldFromModelTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[worldFromModelTrackId]) : nullptr;
	hkbGeneratorOutput::TrackHeader *keyframedBonesHeader = numTracks > keyframedBonesTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[keyframedBonesTrackId]) : nullptr;
	hkbGeneratorOutput::TrackHeader *rigidBodyHeader = numTracks > rigidBodyControlsTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[rigidBodyControlsTrackId]) : nullptr;
	hkbGeneratorOutput::TrackHeader *poweredHeader = numTracks > poweredControlsTrackId ? &(generatorOutput.m_tracks->m_trackHeaders[poweredControlsTrackId]) : nullptr;

	bool isCollidedWith = g_hitDrivers.count(driver) || g_higgsDrivers.count(driver);

	double frameTime = GetTime();

	RagdollData &ragdollData = g_ragdollData[driver];

	bool isRigidBodyOn = rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f;
	bool isPoweredOn = poweredHeader && poweredHeader->m_onFraction > 0.f;

	ragdollData.isOn = true;
	if (!isRigidBodyOn) {
		ragdollData.isOn = false;
		ragdollData.state = RagdollState::Keyframed; // reset state
		return;
	}

	RagdollState state = ragdollData.state;
	if (state == RagdollState::Keyframed) {
		if (isCollidedWith) {
			ragdollData.stateChangedTime = frameTime;
			ragdollData.firstFrameBlendInOut = true;
			state = RagdollState::BlendIn;
		}
	}
	if (state == RagdollState::BlendIn) {
		if (!isCollidedWith) {
			// TODO: We need to make sure to blend out from the current lerped blended in pose. Currently we blend out from the actual ragdoll pose which has not been fully blended in (as we are currently still in the blendin state)
			ragdollData.stateChangedTime = frameTime;
			ragdollData.firstFrameBlendInOut = true;
			state = RagdollState::BlendOut;
		}
	}
	if (state == RagdollState::Collide) {
		if (!isCollidedWith) {
			ragdollData.stateChangedTime = frameTime;
			state = RagdollState::StopCollide;
		}
	}
	if (state == RagdollState::StopCollide) {
		if (isCollidedWith) {
			state = RagdollState::Collide;
		}
	}
	if (state == RagdollState::BlendOut) {
		if (isCollidedWith) {
			state = RagdollState::BlendIn;
		}
	}
	ragdollData.state = state;

	if (keyframedBonesHeader && keyframedBonesHeader->m_onFraction > 0.f) {
		if (state == RagdollState::Keyframed || (state == RagdollState::BlendOut && !ragdollData.firstFrameBlendInOut)) { // Don't keyframe bones if we've collided with the actor
			SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);
		}
		else {
			// Explicitly make bones not keyframed
			keyframedBonesHeader->m_onFraction = 0.f;
		}
	}

	if (!isRigidBodyOn && !isPoweredOn) {
		// No controls are active - try and force it to use the rigidbody controller
		if (rigidBodyHeader) {
			TryForceControls(generatorOutput, *rigidBodyHeader);
		}
	}

	if ((state == RagdollState::BlendIn || state == RagdollState::Collide || state == RagdollState::StopCollide) && rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f) {
		if (rigidBodyHeader->m_numData > 0) {
			float elapsedTime = (frameTime - ragdollData.stateChangedTime) * *g_globalTimeMultiplier;

			float hierarchyGain, velocityGain, positionGain;
			if (state == RagdollState::StopCollide) {
				float lerpAmount = elapsedTime / Config::options.stopCollideMaxTime;
				lerpAmount = std::clamp(lerpAmount, 0.f, 1.f);
				hierarchyGain = lerp(Config::options.collideHierarchyGain, Config::options.stopCollideHierarchyGain, lerpAmount);
				velocityGain = lerp(Config::options.collideVelocityGain, Config::options.stopCollideVelocityGain, lerpAmount);
				positionGain = lerp(Config::options.collidePositionGain, Config::options.stopCollidePositionGain, lerpAmount);
			}
			else {
				hierarchyGain = Config::options.collideHierarchyGain;
				velocityGain = Config::options.collideVelocityGain;
				positionGain = Config::options.collidePositionGain;
			}

			hkaKeyFrameHierarchyUtility::ControlData *data = (hkaKeyFrameHierarchyUtility::ControlData *)(Track_getData(generatorOutput, *rigidBodyHeader));
			for (int i = 0; i < rigidBodyHeader->m_numData; i++) {
				hkaKeyFrameHierarchyUtility::ControlData &elem = data[i];
				elem.m_hierarchyGain = hierarchyGain;
				elem.m_velocityGain = velocityGain;
				elem.m_positionGain = positionGain;
			}
		}
	}

	Actor *actor = GetActorFromRagdollDriver(driver);
	if (actor) {
		NiPointer<NiNode> root = actor->GetNiNode();
		if (root) {
			bhkCharRigidBodyController *controller = GetCharRigidBodyController(actor);
			if (controller) {

				if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
					NiPointer<bhkRigidBody> rb = GetFirstRigidBody(root);
					if (rb) {
						NiAVObject *collNode = GetNodeFromCollidable(&rb->hkBody->m_collidable);
						//std::string boneName = std::string("Ragdoll_") + collNode->m_name;
						_MESSAGE(collNode->m_name);

						int boneIndex = GetAnimBoneIndex(driver->character, collNode->m_name);
						if (boneIndex >= 0) {
							const hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);

							hkQsTransform *poseData = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);
							// TODO: Technically I think we need to apply the entire hierarchy of poses here, not just worldFromModel, but this _is_ the 'root' collision node...
							hkQsTransform poseT;
							poseT.setMul(worldFromModel, poseData[boneIndex]);
							
							if (Config::options.doRootMotion && state == RagdollState::Collide && g_hipBoneTransforms.count(driver)) {
								hkTransform actualT;
								rb->getTransform(actualT);

								NiPoint3 posePos = HkVectorToNiPoint(g_hipBoneTransforms[driver].m_translation) * *g_havokWorldScale;
								NiPoint3 actualPos = HkVectorToNiPoint(actualT.m_translation);
								NiPoint3 posDiff = actualPos - posePos;

								PrintVector(posDiff);
								hkpSurfaceInfo &surface = controller->surfaceInfo;
								if (surface.m_supportedState == hkpSurfaceInfo::SupportedState::SUPPORTED) {
									NiPoint3 supportNorm = HkVectorToNiPoint(surface.m_surfaceNormal);
									NiPoint3 posDiffInSupportPlane = ProjectVectorOntoPlane(posDiff, supportNorm);

									//PrintToFile(std::to_string(VectorLength(posDiffInSupportPlane)), "posdiff.txt");

									if (VectorLength(posDiffInSupportPlane) > Config::options.rootMotionMinOffset) {
										float deltaTime = *g_deltaTime;
										NiPoint3 vel = posDiffInSupportPlane / deltaTime;
										vel *= Config::options.rootMotionVelocityMultiplier;
										vel += HkVectorToNiPoint(ahkpCharacterRigidBody_getLinearVelocity(controller->characterRigidBody.characterRigidBody));
										ahkpCharacterRigidBody_setLinearVelocity(controller->characterRigidBody.characterRigidBody, NiPointToHkVector(vel), deltaTime);
									}
								}
							}

							g_hipBoneTransforms[driver] = poseT;
						}
					}
				}
			}
		}
	}
}

void PostDriveToPoseHook(hkbRagdollDriver *driver, hkbGeneratorOutput &inOut)
{
	// This hook is called right after hkbRagdollDriver::driveToPose()

	RagdollData &ragdollData = g_ragdollData[driver];
	if (!ragdollData.isOn) return;

	int numBones = driver->ragdoll->getNumBones();
	if (numBones <= 0) return;

	float totalStress = 0.f;
	for (int i = 0; i < driver->ragdoll->getNumBones(); i++) {
		totalStress += sqrtf(g_stressOut[i].m_stressSquared);
	}

	float avgStress = totalStress / numBones;
	_MESSAGE("stress: %.2f", avgStress);
	PrintToFile(std::to_string(avgStress), "stress.txt");

	RagdollState state = ragdollData.state;

	PrintToFile(std::to_string((int)state), "state.txt");

	if (state == RagdollState::StopCollide) {
		double frameTime = GetTime();
		float elapsedTime = (frameTime - ragdollData.stateChangedTime)  * *g_globalTimeMultiplier;
		float stressThreshold = lerp(Config::options.stopCollideStressThresholdStart, Config::options.stopCollideStressThresholdEnd, elapsedTime);
		if (avgStress <= stressThreshold || elapsedTime >= Config::options.stopCollideMaxTime) {
			ragdollData.stateChangedTime = frameTime;
			ragdollData.firstFrameBlendInOut = true;
			state = RagdollState::BlendOut;
		}
	}
	ragdollData.state = state;

	/*
	if (state == RagdollState::BlendOut) {
		bool x;
		for (hkpConstraintInstance *constraint : driver->ragdoll->m_constraints) {
			if (*hkpConstraintInstance_isEnabled(constraint, &x)) {
				hkpConstraintInstance_setEnabled(constraint, false);
			}
		}
	}
	*/
}

hkQsTransform g_animPose[200]; // set in a hook before postPhysics(). Just reserve a bunch of space so it can handle any number of bones.

void PrePostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
	// This hook is called right before hkbRagdollDriver::postPhysics()

	RagdollData &ragdollData = g_ragdollData[driver];
	if (!ragdollData.isOn) return;

	hkInt32 numTracks = inOut.m_tracks->m_masterHeader.m_numTracks;

	int poseTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POSE;
	hkbGeneratorOutput::TrackHeader *poseHeader = numTracks > poseTrackId ? &(inOut.m_tracks->m_trackHeaders[poseTrackId]) : nullptr;
	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *animPose = (hkQsTransform *)Track_getData(inOut, *poseHeader);
		// Copy anim pose track before postPhysics() as postPhysics() will overwrite it with the ragdoll pose
		memcpy(g_animPose, animPose, numPoses * sizeof(hkQsTransform));
	}
}

hkQsTransform g_scratchPose[200];
void PostPostPhysicsHook(hkbRagdollDriver *driver, hkbGeneratorOutput &inOut)
{
	// This hook is called right after hkbRagdollDriver::postPhysics()

	RagdollData &ragdollData = g_ragdollData[driver];
	if (!ragdollData.isOn) return;

	RagdollState state = ragdollData.state;

	if (state == RagdollState::BlendIn || state == RagdollState::BlendOut) {
		double elapsedTime = (GetTime() - ragdollData.stateChangedTime) * *g_globalTimeMultiplier;

		bool isBlendIn = state == RagdollState::BlendIn;

		double blendTime = isBlendIn ? Config::options.blendInTime : Config::options.blendOutTime;
		float lerpAmount = std::clamp(elapsedTime / blendTime, 0.0, 1.0);

		hkInt32 numTracks = inOut.m_tracks->m_masterHeader.m_numTracks;

		int poseTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POSE;
		hkbGeneratorOutput::TrackHeader *poseHeader = numTracks > poseTrackId ? &(inOut.m_tracks->m_trackHeaders[poseTrackId]) : nullptr;
		if (poseHeader && poseHeader->m_onFraction > 0.f) {
			int numPoses = poseHeader->m_numData;
			hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);
			if (ragdollData.firstFrameBlendInOut) {
				ragdollData.firstFrameBlendInOut = false;
				ragdollData.blendInOutInitialPose.reserve(numPoses);

				hkQsTransform *initialPose = isBlendIn ? g_animPose : poseOut;
				memcpy(ragdollData.blendInOutInitialPose.data(), initialPose, numPoses * sizeof(hkQsTransform));
			}

			if (isBlendIn) {
				// Save the pose before we write to it - at this point it is the high-res pose extracted from the ragdoll
				memcpy(g_scratchPose, poseOut, numPoses * sizeof(hkQsTransform));
				hkbBlendPoses(numPoses, ragdollData.blendInOutInitialPose.data(), g_scratchPose, lerpAmount, poseOut);
			}
			else {
				hkbBlendPoses(numPoses, ragdollData.blendInOutInitialPose.data(), g_animPose, lerpAmount, poseOut);
			}
		}

		// Need to change states only after acting on the previous state)
		if (elapsedTime >= blendTime) {
			state = isBlendIn ? RagdollState::Collide : RagdollState::Keyframed;
		}

		ragdollData.state = state;
	}
}


uintptr_t processHavokHitJobsHookedFuncAddr = 0;
auto processHavokHitJobsHookLoc = RelocAddr<uintptr_t>(0x6497E4);
auto processHavokHitJobsHookedFunc = RelocAddr<uintptr_t>(0x75AC20);

uintptr_t postPhysicsHookedFuncAddr = 0;
auto postPhysicsHookLoc = RelocAddr<uintptr_t>(0xB268DC);
auto postPhysicsHookedFunc = RelocAddr<uintptr_t>(0xA27730); // hkbRagdollDriver::postPhysics()

uintptr_t driveToPoseHookedFuncAddr = 0;
auto driveToPoseHookLoc = RelocAddr<uintptr_t>(0xB266AB);
auto driveToPoseHookedFunc = RelocAddr<uintptr_t>(0xA25B60); // hkbRagdollDriver::driveToPose()

uintptr_t controllerDriveToPoseHookedFuncAddr = 0;
auto controllerDriveToPoseHookLoc = RelocAddr<uintptr_t>(0xA26C05);
auto controllerDriveToPoseHookedFunc = RelocAddr<uintptr_t>(0xB4CFF0); // hkaRagdollRigidBodyController::driveToPose()

void PerformHooks(void)
{
	// First, set our addresses
	processHavokHitJobsHookedFuncAddr = processHavokHitJobsHookedFunc.GetUIntPtr();
	driveToPoseHookedFuncAddr = driveToPoseHookedFunc.GetUIntPtr();
	postPhysicsHookedFuncAddr = postPhysicsHookedFunc.GetUIntPtr();
	controllerDriveToPoseHookedFuncAddr = controllerDriveToPoseHookedFunc.GetUIntPtr();

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

				// Restore args to pass to our post hook
				mov(rcx, ptr[rsi + 0xF0]); // hkbRagdollDriver
				lea(rdx, ptr[rsp + 0x60]); // hkbGeneratorOutput

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

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

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
				mov(rax, (uintptr_t)PrePostPhysicsHook);
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
				mov(rax, postPhysicsHookedFuncAddr);
				call(rax);

				// Restore args to pass to our post hook
				mov(rcx, ptr[rsi + 0xF0]); // hkbRagdollDriver
				lea(rdx, ptr[rsp + 0x30]); // hkbGeneratorOutput

				// Call our post hook
				mov(rax, (uintptr_t)PostPostPhysicsHook);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(postPhysicsHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(postPhysicsHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("hkbRagdollDriver::postPhysics hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// The 4th arg to hkaRagdollRigidBodyController::driveToPose is a ptr which it sets to the output stress on the ragdoll.
				// The game passes 0 in this arg normally, which means the stress is not extracted. I want to know the stress though.
				mov(rax, (uintptr_t)&g_stressOut);
				mov(ptr[rsp + 0x20], rax);

				// Original code
				mov(rax, controllerDriveToPoseHookedFuncAddr);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(controllerDriveToPoseHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(controllerDriveToPoseHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("hkaRagdollRigidBodyController::driveToPose hook complete");
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

void HiggsGrab(bool isLeft, TESObjectREFR *grabbedRefr)
{
	Actor *actor = DYNAMIC_CAST(grabbedRefr, TESObjectREFR, Actor);
	if (actor) {
		BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
		if (GetAnimationGraphManager(actor, animGraphManager)) {
			BSAnimationGraphManager *manager = animGraphManager.ptr;
			SimpleLocker lock(&manager->updateLock);
			for (int i = 0; i < manager->graphs.size; i++) {
				BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.heapSize >= 0 ? manager->graphs.data.heap[i] : manager->graphs.data.local[i];
				hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
				if (driver) {
					g_higgsDrivers.insert(driver);
				}
			}
		}
	}
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
			else if (msg->type == SKSEMessagingInterface::kMessage_PostPostLoad) {
				// Get the HIGGS plugin API
				HiggsPluginAPI::GetHiggsInterface001(g_pluginHandle, g_messaging);
				if (g_higgsInterface) {
					_MESSAGE("Got higgs interface!");
					g_higgsInterface->AddGrabbedCallback(HiggsGrab);
				}
				else {
					_MESSAGE("Did not get higgs interface. This is okay.");
				}
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
		info->name = "MeleeVR";
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

		if (Config::ReadConfigOptions()) {
			_MESSAGE("Successfully read config parameters");
		}
		else {
			_WARNING("[WARNING] Failed to read config options. Using defaults instead.");
		}

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

		g_timer.Start();

		return true;
	}
};
