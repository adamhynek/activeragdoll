#pragma once

#include "skse64_common/Relocation.h"
#include "skse64/NiTypes.h"
#include "skse64/GameTypes.h"

#include "RE/havok.h"


typedef void(*_CleanupCloneList)(uintptr_t _this);
extern RelocAddr<_CleanupCloneList> CleanupCloneList1;
extern RelocAddr<_CleanupCloneList> CleanupCloneList2;

extern RelocPtr<UInt64> unk_141E703BC;
extern RelocPtr<UInt64> unk_141E703B8;

class NiCloningProcess
{
public:
	NiCloningProcess() {
		unk18 = unk_141E703BC;
		unk48 = unk_141E703B8;
	}

	~NiCloningProcess() {
		CleanupCloneList1((uintptr_t)&unk38);
		CleanupCloneList2((uintptr_t)&unk08);
	}

	UInt64 unk00 = 0;
	UInt64 unk08 = 0; // Start of clone list 1?
	UInt64 unk10 = 0;
	UInt64 * unk18; // initd to RelocAddr(0x1E703BC)
	UInt64 unk20 = 0;
	UInt64 unk28 = 0;
	UInt64 unk30 = 0;
	UInt64 unk38 = 0; // Start of clone list 2?
	UInt64 unk40 = 0;
	UInt64 * unk48; // initd to RelocAddr(0x1E703B8)
	UInt64 unk50 = 0;
	UInt64 unk58 = 0;
	UInt8 copyType = 1; // 60 - CopyType - default 1
	UInt8 m_eAffectedNodeRelationBehavior = 0; // 61 - CloneRelationBehavior - default 0
	UInt8 m_eDynamicEffectRelationBehavior = 0; // 62 - CloneRelationBehavior - default 0
	char m_cAppendChar = '$'; // 64 - default '$'
	NiPoint3 scale = { 1.0f, 1.0f, 1.0f }; // 0x68 - default {1, 1, 1}
};

struct VRMeleeData
{
	enum class SwingDirection : UInt32 {
		kNone = 0,
		kDown = 1,
		kLeft = 3,
		kRight = 4,
		kUp = 6
	};

	UInt64 unk00; // something to do with cooldown multipliers
	UInt64 unk08;
	NiPointer<bhkWorld> world; // 10
	NiPointer<NiNode> collisionNode; // 18
	NiPointer<NiAVObject> offsetNode; // 20
	UInt32 unk28; // default == 3? - could be an isLeft sort of thing
	UInt32 currentArrayOffset; // 2C - this is the offset in the arrays of the value for the current frame
	NiPoint3 position; // 30
	tArray<float> wandPositionDiffLengths; // 40 - lengths of values in subsequent array
	tArray<NiPoint3> wandPositionDiffs; // 58 - diffs of currentframe - lastframe of wandPositionsRoomspace
	tArray<NiPoint3> wandPositionsRoomspace; // 70
	tArray<Actor *> sweepActors; // 88 - which actors has the current swing hit. Only applies if the player has the 'Sweep' perk.
	float unkA0; // linearVelocityThreshold when main hand, 0 when offhand
	float linearVelocityThreshold; // A4
	float impactConfirmRumbleIntensity; // A8
	float impactConfirmRumbleDuration; // AC
	float impactRumbleIntensity; // B0
	float impactRumbleDuration; // B4
	float meleeForceMultLinear; // B8
	bool enableCollision; // BC - this value is read from and the collision node's collision is enabled/disabled
	bool applyImpulseOnHit; // BD - default true
	SwingDirection swingDirection; // C0 - default 0
	float cooldown; // C4 - gets set to the cooldown, then ticks down, can (and will) get negative - default 0
	float powerAttackCooldown; // C8 - default 0
	UInt32 unkCC;
};
static_assert(offsetof(VRMeleeData, collisionNode) == 0x18);
static_assert(offsetof(VRMeleeData, linearVelocityThreshold) == 0xA4);
static_assert(sizeof(VRMeleeData) == 0xD0);

struct CrosshairPickData
{
	UInt32 unk00;
	UInt32 leftHandle1; // 04
	UInt32 rightHandle1; // 08
	UInt32 unk0C;
	UInt32 leftHandle2; // 10
	UInt32 rightHandle2; // 14
	UInt32 unk18;
	UInt32 leftHandle3; // 1C
	UInt32 rightHandle3; // 20
	UInt8 unk24[0x50 - 0x24];
	bhkRigidBody *rigidBodies[2]; // 50 - slot depends on hand
	UInt8 unk60[0x78 - 0x60];
	NiPointer<bhkSimpleShapePhantom> sphere; // 78
};
static_assert(offsetof(CrosshairPickData, rigidBodies) == 0x50);
static_assert(offsetof(CrosshairPickData, sphere) == 0x78);

struct SoundData
{
	UInt32 id = -1; // 00
	UInt8 unk04 = 0;
	UInt32 unk08 = 0; // set to 1 when playing
};
static_assert(sizeof(SoundData) == 0x0C);

struct BGSImpactManager
{
	void *vtbl; // 00
	BSTEventSink<void> combatImpactEventSink; // 08 - BSTEventSink<BGSCombatImpactEvent>
	BSTEventSink<void> collisionSoundEventSink; // 10 - BSTEventSink<BGSCollisionSoundEvent>
	// sound event passes in ptr to skyrimhavokmaterial id in rdx (actually it's the 2 material ids, then at 0x14 (I think?) is the magnitude of the impact or something - it's used to determine which sound to play, high or low)
};

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

struct BGSAttackData : NiRefObject
{
	struct AttackData  // ATKD
	{
		enum class AttackFlag : UInt32
		{
			kNone = 0,
			kIgnoreWeapon = 1 << 0,
			kBashAttack = 1 << 1,
			kPowerAttack = 1 << 2,
			kChargeAttack = 1 << 3,
			kRotatingAttack = 1 << 4,
			kContinuousAttack = 1 << 5,
			kOverrideData = (UInt32)1 << 31
		};

		// members
		float                                       damageMult;     // 00
		float                                       attackChance;   // 04
		SpellItem*                                  attackSpell;    // 08
		UInt32										flags;          // 10
		float                                       attackAngle;    // 14
		float                                       strikeAngle;    // 18
		float                                       staggerOffset;  // 1C
		BGSKeyword*                                 attackType;     // 20
		float                                       knockDown;      // 28
		float                                       recoveryTime;   // 2C
		float                                       staminaMult;    // 30
		std::uint32_t                               pad34;          // 34
	};
	static_assert(sizeof(AttackData) == 0x38);

	BSFixedString event;  // 10 - ATKE
	AttackData    data;   // 18 - ATKD
};
static_assert(sizeof(BGSAttackData) == 0x50);

struct HitData
{
	enum class Flag : UInt32
	{
		kBlocked = 1 << 0,
		kBlockWithWeapon = 1 << 1,
		kBlockCandidate = 1 << 2,
		kCritical = 1 << 3,
		kCriticalOnDeath = 1 << 4,
		kFatal = 1 << 5,
		kDismemberLimb = 1 << 6,
		kExplodeLimb = 1 << 7,
		kCrippleLimb = 1 << 8,
		kDisarm = 1 << 9,
		kDisableWeapon = 1 << 10,
		kSneakAttack = 1 << 11,
		kIgnoreCritical = 1 << 12,
		kPredictDamage = 1 << 13,
		//kPredictBaseDamage = 1 << 14,
		kBash = 1 << 14,
		kTimedBash = 1 << 15,
		kPowerAttack = 1 << 16,

		kOffhand = 1 << 17,

		kMeleeAttack = 1 << 18,
		kRicochet = 1 << 19,
		kExplosion = 1 << 20
	};

	// members
	NiPoint3                 hitPosition;             // 00
	NiPoint3                 hitDirection;            // 0C
	UInt32                   aggressor;               // 18
	UInt32                   target;                  // 1C
	UInt32                   sourceRef;               // 20
	std::uint32_t            pad24;                   // 24
	NiPointer<BGSAttackData> attackData;              // 28
	TESObjectWEAP*           weapon;                  // 30
	std::uint64_t            unk38;                   // 38
	std::uint64_t            unk40;                   // 40
	std::uint32_t            unk48;                   // 48
	float                    healthDamage;            // 4C
	float                    totalDamage;             // 50
	float                    physicalDamage;          // 54
	float                    targetedLimbDamage;      // 58
	float                    percentBlocked;          // 5C
	float                    resistedPhysicalDamage;  // 60
	float                    resistedTypedDamage;     // 64
	float                    stagger;                 // 68
	float                    sneakAttackBonus;        // 6C
	float                    bonusHealthDamageMult;   // 70
	float                    pushBack;                // 74
	float                    reflectedDamage;         // 78
	float                    criticalDamageMult;      // 7C
	Flag                     flags;                   // 80
	std::uint32_t            equipIndex;              // 84
	std::uint32_t            material;                // 88
	std::uint32_t            damageLimb;              // 8C
};
static_assert(sizeof(HitData) == 0x90);

template <class T, UInt32 N = 1>
struct BSTSmallArray
{
	union Data
	{
		T* heap;
		T local[N];
	};

	inline T * GetData() { return heapSize >= 0 ? data.heap : data.local; }

	SInt32 heapSize = 0x80000000; // 00 - default -0.f
	UInt32 unk04; // 04
	Data data; // 08
	UInt32 size;
};

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
	BSTSmallArray<HavokHitJob, 5> jobs; // 08
};
static_assert(offsetof(HavokHitJobs, jobs.data) == 0x10);
static_assert(offsetof(HavokHitJobs, jobs.size) == 0xB0);

struct ActorCause
{
	UInt32 actor; // 00
	NiPoint3 origin; // 04
	UInt32 actorCauseID; // 10
	volatile mutable SInt32 refCount; // 14
};
static_assert(sizeof(ActorCause) == 0x18);

struct BSFadeNodeCuller
{
	struct Plane
	{
		NiPoint3 normal;
		float d;
	};

	NiCamera *camera; // 18

	Plane frustumCullingPlanes[6]; // 3C - near plane, far plane, ...

	struct BSMultiBoundNode *boundNode; // 128
	struct BSMultiBound *bound; // 130
	struct BSMultiBoundSphere *boundSphere; // 138
};

struct MovementMessageActorCollision
{
	void *vtbl; // 00
	UInt32 refCount = 0; // 08
	UInt32 pad0C;
	UInt32 handle; // 10
	UInt32 pad14;
};
static_assert(sizeof(MovementMessageActorCollision) == 0x18);

struct MovementControllerAI : BSIntrusiveRefCounted
{
	virtual ~MovementControllerAI(); // 00
	virtual void SetInterfaceByName(BSFixedString &interfaceName, IMovementInterface *interfacePtr); // 01
	virtual IMovementInterface * GetInterfaceByName_1(BSFixedString &interfaceName); // 02
	virtual IMovementInterface * GetInterfaceByName_2(BSFixedString &interfaceName); // 03
	virtual void ClearInterfaceByName(BSFixedString &interfaceName); // 04
	virtual IMovementState * GetCurrentMovementState(); // 05 - returns GetInterfaceByName_1("CurrentMovementState")
	virtual void InitializeInterfaces(); // 06
	// ...

	struct InterfaceEntry
	{
		BSFixedString name; // 00
		IMovementInterface *interfacePtr; // 08
	};

	UInt8 unk10[0x48 - 0x10]; // 10
	BSTSmallArray<InterfaceEntry, 11> interfaces; // 48
	BSReadWriteLock interfacesLock; // 108
	UInt64 unk110;
	UInt64 unk118;
};
static_assert(offsetof(MovementControllerAI, interfaces) == 0x48);
static_assert(offsetof(MovementControllerAI, interfacesLock) == 0x108);
static_assert(sizeof(MovementControllerAI) == 0x120);

struct MovementControllerNPC : MovementControllerAI
{
	IMovementInterface movementMessageInterface; // 120
	IMovementInterface movementMotionDrivenControl; // 128
	IMovementInterface movementSelectIdle; // 130
	IMovementInterface movementDirectControl; // 138
	IMovementInterface movementPlannerDirectControl; // 140
	IMovementInterface animationSetCallbackFunctor; // 148
	UInt64 movementMessageLock; // 150
	tArray<MovementMessageActorCollision *> movementMessages; // 158
	UInt8 unk170[0x1B8 - 0x170];
	Actor *actor; // 1B8
	UInt32 flags; // 1C0
	UInt32 unk1C4;
	UInt8 unk1C8;
	UInt8 unk1C9;
	UInt8 unk1CA;
	UInt8 keepOffsetFromActor; // 1CB
	UInt8 unk1CC;
	UInt8 unk1CD;
	UInt8 unk1CE;
	UInt8 unk1CF;
};
static_assert(offsetof(MovementControllerNPC, movementMessageLock) == 0x150);
static_assert(offsetof(MovementControllerNPC, movementMessages) == 0x158);
static_assert(offsetof(MovementControllerNPC, actor) == 0x1B8);
static_assert(sizeof(MovementControllerNPC) == 0x1D0);

enum class KnockState : UInt8
{
	Normal = 0,
	Ragdolled = 1,
	BeginGetUp = 5,
	GetUp = 6,
};

struct FOCollisionListener : hkpContactListener
{
	struct CollisionDamageEvent
	{
		NiPointer<bhkRigidBody> body; // 00
		float damage; // 04
		NiPointer<ActorCause> actorCause; // 08
	};
	static_assert(sizeof(CollisionDamageEvent) == 0x18);

	UInt64 unk08;
	tArray<CollisionDamageEvent> collisionEvents; // 10
};

struct CharacterCollisionHandler
{
	virtual ~CharacterCollisionHandler(); // 00
	virtual void HandleCharacterCollision(struct bhkCharacterController *collider, struct bhkCharacterController *collidee); // 01
};

struct ActionInput
{
	void *vtbl; // 00
	NiPointer<TESObjectREFR> source; // 08
	NiPointer<TESObjectREFR> target; // 10
	BGSAction *action; // 18
	UInt32 unk20;
};

struct TESActionData : ActionInput
{
	BSFixedString unk28{};
	BSFixedString unk30{};
	UInt64 unk38 = 0;
	UInt64 unk40 = 0;
	UInt64 unk48 = 0;
	UInt64 unk50 = 0;
	UInt64 unk58 = 0;
};

struct PackageLocation
{
	union Data
	{
		TESForm* object;
		UInt32 refHandle;
	};
	static_assert(sizeof(Data) == 0x8);

	void *vtbl; // 00
	UInt8 locType; // 08
	UInt32 rad; // 0C
	Data data; // 10
};
static_assert(sizeof(PackageLocation) == 0x18);

struct PackageTarget
{
	UInt8 targType; // 00
	UInt64 target; // 08
	UInt32 value; // 10
	UInt32 pad14; // 14
};
static_assert(sizeof(PackageTarget) == 0x18);

struct IMovementPlannerAgent
{
	void *vtbl; // 00
};

struct IMovementSetKeepOffsetFromActor
{
	void *vtbl; // 00
};

struct MovementAgent
{
	void *vtbl; // 00
	UInt32 unk08;
	IMovementState *movementState; // 10 - points to ActorState of Actor
};

struct MovementPlannerAgent : MovementAgent
{
	IMovementPlannerAgent unk18;
};

struct MovementPlannerAgentKeepOffset : MovementPlannerAgent
{
	struct KeepOffsetData
	{
		UInt32 targetHandle; // 00
		NiPoint3 offset; // 04
		NiPoint3 angleOffset; // 10
		float catchUpRadius; // 1C
		float followRadius; // 20
	};
	static_assert(sizeof(KeepOffsetData) == 0x24);

	IMovementSetKeepOffsetFromActor movementSet; // 20
	MovementControllerNPC *movementController; // 28
	BSReadWriteLock keepOffsetLock; // 30
	KeepOffsetData keepOffsetData; // 38
};
static_assert(sizeof(MovementPlannerAgentKeepOffset) == 0x60);

typedef void(*_IAnimationGraphManagerHolder_NotifyAnimationGraph)(IAnimationGraphManagerHolder *_this, const BSFixedString& a_eventName);
typedef void(*_Actor_WeaponSwingCallback)(Actor *_this);
typedef void(*_Actor_PauseCurrentDialogue)(Actor *_this);
typedef void(*_Actor_PutCreatedPackage)(Actor *_this, TESPackage *package, bool dontExitFurniture, bool a4);
typedef float(*_Actor_GetHeading)(Actor *_this, bool a_ignoreRaceSettings);
typedef ActorCause * (*_TESObjectREFR_GetActorCause)(TESObjectREFR *_this);
typedef void(*_TESObjectREFR_SetActorCause)(TESObjectREFR *_this, ActorCause* a_cause);
typedef bool(*_MagicTarget_IsInvulnerable)(MagicTarget *_this);
typedef void(*_ActorValueOwner_RestoreActorValue)(ActorValueOwner *_this, UInt32 modifier, UInt64 actorValue, float value);
typedef void(*_BSIntrusiveRefCounted_Destruct)(BSIntrusiveRefCounted *_this, UInt32 unk);
typedef bool(*_TESActionData_Process)(TESActionData *_this);
