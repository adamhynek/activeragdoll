#include <numeric>
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
#include <deque>

#include <Physics/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>
#include <Physics/Collide/Shape/Convex/Capsule/hkpCapsuleShape.h>
#include <Common/Base/Types/Geometry/hkStridedVertices.h>

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
#include "skse64/PapyrusKeyword.h"
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
#include "RE/havok_behavior.h"
#include "havok_ref_ptr.h"
#include "higgsinterface001.h"
#include "main.h"
#include "blender.h"


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;
SKSETrampolineInterface *g_trampoline = nullptr;
SKSETaskInterface *g_taskInterface = nullptr;

BGSKeyword *g_keyword_actorTypeAnimal = nullptr;
BGSKeyword *g_keyword_actorTypeNPC = nullptr;

bool g_isRightTriggerHeld = false;
bool g_isLeftTriggerHeld = false;


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


NiPointer<NiAVObject> GetWeaponCollisionOffsetNode(TESObjectWEAP *weapon, bool isLeft)
{
	PlayerCharacter *player = *g_thePlayer;
	UInt8 weaponType = weapon->gameData.type;

	if (!weapon || weaponType == TESObjectWEAP::GameData::kType_HandToHandMelee) {
		return isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftHandBone] : player->unk3F0[PlayerCharacter::Node::kNode_RightHandBone];
	}
	else if (weaponType == TESObjectWEAP::GameData::kType_Bow) {
		return player->unk538[PlayerCharacter::BowNode::kBowNode_BowRotationNode];
	}
	else if (weaponType == TESObjectWEAP::GameData::kType_CrossBow) {
		return player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftWeaponOffsetNode : PlayerCharacter::Node::kNode_RightWeaponOffsetNode];
	}
	else if (weaponType == TESObjectWEAP::GameData::kType_Staff) {
		return player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftStaffWeaponOffsetNode : PlayerCharacter::Node::kNode_RightStaffWeaponOffsetNode];
	}

	return player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftMeleeWeaponOffsetNode : PlayerCharacter::Node::kNode_RightMeleeWeaponOffsetNode];
}

void SwingWeapon(TESObjectWEAP *weapon, bool isLeft, bool setAttackState = true, bool playSound = true)
{
	PlayerCharacter *player = *g_thePlayer;

	if (setAttackState) {
		player->actorState.flags04 &= 0xFFFFFFFu; // zero out meleeAttackState
		player->actorState.flags04 |= 0x20000000u; // meleeAttackState = kSwing
	}

	if (playSound) {
		if (weapon) {
			if (weapon->type() < TESObjectWEAP::GameData::kType_Bow) { // not bow, staff, or crossbow
				BGSSoundDescriptorForm *sound = weapon->attackFailSound;
				if (sound) {
					NiPointer<NiAVObject> node = GetWeaponCollisionOffsetNode(weapon, isLeft);
					if (node) {
						PlaySoundAtNode(sound, node, {});
					}
				}
			}
		}
	}

	get_vfunc<_Actor_WeaponSwingCallback>(player, 0xF1)(player);

	if (player->processManager) {
		ActorProcess_IncrementAttackCounter(player->processManager, 1);
	}

	// Make noise
	int soundAmount = weapon ? TESObjectWEAP_GetSoundAmount(weapon) : TESNPC_GetSoundAmount((TESNPC *)player->baseForm);
	Actor_SetActionValue(player, soundAmount);

	if (player->unk158) {
		CombatController_sub_14050DEC0((void *)player->unk158);
	}
}

RelocAddr<void *> CheckHitEventsFunctor_vtbl(0x16E6BA0);
struct CheckHitEventsFunctor : IForEachScriptObjectFunctor
{
	struct Data
	{
		VMClassRegistry *registry; // 00
		hkpRigidBody *hitBody; // 08 - not sure if this is actually needed but it's here anyway
	};

	CheckHitEventsFunctor(Data *data, bool *result) : data(data), result(result) {
		set_vtbl(this, CheckHitEventsFunctor_vtbl);
	};

	Data *data; // 08 - points to stack addr of VMClassRegistry *
	bool *result; // 10
	UInt8 unk18[0x38 - 0x18];
	IForEachScriptObjectFunctor *next = this;
};
static_assert(offsetof(CheckHitEventsFunctor, data) == 0x08);

typedef void * (*_GetScriptEventSourceHolder)();
RelocAddr<_GetScriptEventSourceHolder> GetScriptEventSourceHolder(0x1964C0);

typedef void * (*_DispatchHitEvent)(void *scriptEventSourceHolder, TESHitEvent *hitEvent);
RelocAddr<_DispatchHitEvent> DispatchHitEvent(0x635AA0);

typedef void(*_VMClassRegistry_Destruct)(VMClassRegistry *_this, UInt32 unk);
bool DispatchHitEvents(TESObjectREFR *source, TESObjectREFR *target, hkpRigidBody *hitBody, TESForm *weapon)
{
	SkyrimVM *vm = *g_skyrimVM;
	VMClassRegistry *registry = vm->GetClassRegistry();
	if (registry) {
		// vm incref
		InterlockedIncrement(((UInt32 *)&registry->unk0004));

		IObjectHandlePolicy *handlePolicy = registry->GetHandlePolicy();
		UInt64 handle = handlePolicy->Create(target->formType, target);

		// Check if the object should dispatch hit events when hit, I think
		bool shouldDispatchHitEvent = false;
		CheckHitEventsFunctor::Data checkHitEventsFunctorData{ registry, hitBody };
		CheckHitEventsFunctor checkHitEventsFunctor{ &checkHitEventsFunctorData, &shouldDispatchHitEvent };

		registry->VisitScripts(handle, &checkHitEventsFunctor);

		if (shouldDispatchHitEvent) {
			// Increment refcounts before dispatching the hit event. The game does this... so I'll do it too.
			NiPointer<TESObjectREFR> incTarget = target;
			NiPointer<TESObjectREFR> incSource = source;

			if (!weapon) weapon = *g_unarmedWeapon;

			// Now dispatch the hit event
			TESHitEvent hitEvent{ source, target, weapon->formID, 0, 0 };

			void *scriptEventSourceHolder = GetScriptEventSourceHolder();
			DispatchHitEvent((void *)((UInt64)scriptEventSourceHolder + 0x5D8), &hitEvent);
		}

		// vm decref
		if (InterlockedExchangeSubtract(((UInt32 *)&registry->unk0004), (UInt32)1) == 1) {
			get_vfunc< _VMClassRegistry_Destruct>(registry, 0x0)(registry, 1);
		}

		return shouldDispatchHitEvent;
	}

	return false;
}

/*
void Hit()
{
	BGSAttackData *attackData = nullptr;
	PlayerCharacter_UpdateAndGetAttackData(player, *g_isUsingMotionControllers, isLeft, false, &attackData);

	// set hit pos / velocity

	if (hitCharacter && !isGhost && Character_CanHit(player, target)) {

	}
	else {
		if (hitRefr) {
			// DispatchHitEvents();
		}

		if (g_bPlayVRMeleeWorldImpactSounds) {
			// play impact effect sound
		}

		if (hitRefr) {
			if (IsMotionTypeMoveable()) {
				hitRefr->SetActorCause(player->GetActorCause());
			}
			BSTaskPool_QueueDestroyTask(hitRefr);
		}

		PlayRumble();
	}

	ApplyImpulse();
}
*/

void HitActor(Character *source, Character *target, TESForm *weapon, BGSAttackData *attackData, bool isOffhand, bool isPowerAttack)
{
	// Handle bow/crossbow/torch/shield bash (set attackstate to kBash)
	TESForm *offhandObj = source->GetEquippedObject(true);
	TESObjectARMO *equippedShield = (offhandObj && offhandObj->formType == kFormType_Armor) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectARMO) : nullptr;
	bool isShield = isOffhand && equippedShield;

	TESObjectLIGH *equippedLight = (offhandObj && offhandObj->formType == kFormType_Light) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectLIGH) : nullptr;
	bool isTorch = isOffhand && equippedLight;

	TESObjectWEAP *weap = DYNAMIC_CAST(weapon, TESForm, TESObjectWEAP);
	bool isBowOrCrossbow = weap && (weap->type() == TESObjectWEAP::GameData::kType_Bow || weap->type() == TESObjectWEAP::GameData::kType_CrossBow);

	bool isBash = isBowOrCrossbow || isShield || isTorch;

	source->actorState.flags04 &= 0xFFFFFFFu; // zero out meleeAttackState
	if (isBash) {
		source->actorState.flags04 |= 0x60000000u; // meleeAttackState = kBash
	}
	else {
		source->actorState.flags04 |= 0x20000000u; // meleeAttackState = kSwing
	}

	int dialogueSubtype = isBash ? 28 : (isPowerAttack ? 27 : 26); // 26 is attack, 27 powerattack, 28 bash
	UpdateDialogue(nullptr, source, target, 3, dialogueSubtype, false, nullptr);

	if (attackData && attackData->data.flags & UInt32(BGSAttackData::AttackData::AttackFlag::kPowerAttack)) {
		PlayerControls_sub_140705530(PlayerControls::GetSingleton(), isOffhand ? 45 : 49, 2);
		if (!get_vfunc<_MagicTarget_IsInvulnerable>(&source->magicTarget, 4)(&source->magicTarget)) {
			float staminaCost = ActorValueOwner_GetStaminaCostForAttackData(&source->actorValueOwner, attackData);
			if (staminaCost > 0.f) {
				float staminaBeforeHit = source->actorValueOwner.GetCurrent(26);

				// source->RestoreActorValue(kDamage, kStamina, -staminaCost)
				get_vfunc<_ActorValueOwner_RestoreActorValue>(&source->actorValueOwner, 6)(&source->actorValueOwner, 2, 26, -staminaCost);

				if (source->actorValueOwner.GetCurrent(26) <= 0.f) {
					// Out of stamina after the hit
					if (ActorProcessManager *process = source->processManager) {
						float regenRate = Actor_GetActorValueRegenRate(source, 26);
						ActorProcess_UpdateRegenDelay(process, 26, (staminaBeforeHit - staminaCost) / regenRate);
						FlashHudMenuMeter(26);
					}
				}
			}
		}
	}

	// Make noise
	int soundAmount = weapon ? TESObjectWEAP_GetSoundAmount(weap) : TESNPC_GetSoundAmount((TESNPC *)source->baseForm);
	Actor_SetActionValue(source, soundAmount);

	Character_HitTarget(source, target, nullptr, isOffhand);

	Actor_RemoveMagicEffectsDueToAction(source, -1); // removes invis/ethereal due to attacking

	source->actorState.flags08 &= 0xFFFFFFFu; // zero out attackState
}

bool HitRefr(Character *source, TESObjectREFR *target, TESForm *weapon, hkpRigidBody *hitBody, bool isLeft, bool isOffhand)
{
	source->actorState.flags04 &= 0xFFFFFFFu; // zero out meleeAttackState
	source->actorState.flags04 |= 0x20000000u; // meleeAttackState = kSwing

	float damage;
	{ // All this just to get the fricken damage
		InventoryEntryData *weaponEntry = ActorProcess_GetCurrentlyEquippedWeapon(source->processManager, isOffhand);

		HitData hitData;
		HitData_ctor(&hitData);
		HitData_populate(&hitData, source, nullptr, weaponEntry, isOffhand);

		damage = hitData.totalDamage;

		HitData_dtor(&hitData);
	}

	bool didDispatchHitEvent = DispatchHitEvents(source, target, hitBody, weapon);
	if (IsMoveableEntity(hitBody)) {
		TESObjectREFR_SetActorCause(target, TESObjectREFR_GetActorCause(source));
	}
	BSTaskPool_QueueDestroyTask(BSTaskPool::GetSingleton(), target, damage);

	source->actorState.flags04 &= 0xFFFFFFFu; // zero out attackState

	return didDispatchHitEvent;
}

float GetPhysicsDamage(float mass, float speed)
{
	/*
	// defaults
	g_fPhysicsDamage1Mass = 10.f;
	g_fPhysicsDamage2Mass = 50.f;
	g_fPhysicsDamage3Mass = 100.f;
	g_fPhysicsDamage1Damage = 1.f;
	g_fPhysicsDamage2Damage = 5.f;
	g_fPhysicsDamage3Damage = 10.f;
	g_fPhysicsDamageSpeedBase = 1.f;
	g_fPhysicsDamageSpeedMult = 0.0001f;
	g_fPhysicsDamageSpeedMin = 500.f;
	*/

	if (mass < Config::options.collisionDamageMinMass) return 0.f;
	if (speed < Config::options.collisionDamageMinSpeed) return 0.f;

	float damage = 0.f;
	if (mass >= *g_fPhysicsDamage1Mass) {
		if (mass >= *g_fPhysicsDamage2Mass) {
			if (mass >= *g_fPhysicsDamage3Mass) {
				damage = *g_fPhysicsDamage3Damage;
			}
			else {
				damage = lerp(*g_fPhysicsDamage2Damage, *g_fPhysicsDamage3Damage, (mass - *g_fPhysicsDamage2Mass) / (*g_fPhysicsDamage3Mass - *g_fPhysicsDamage2Mass));
			}
		}
		else {
			damage = lerp(*g_fPhysicsDamage1Damage, *g_fPhysicsDamage2Damage, (mass - *g_fPhysicsDamage1Mass) / (*g_fPhysicsDamage2Mass - *g_fPhysicsDamage1Mass));
		}
	}
	else {
		damage = lerp(0.f, *g_fPhysicsDamage1Damage, mass / *g_fPhysicsDamage1Mass);
	}

	float base = lerp(0.f, *g_fPhysicsDamageSpeedBase, min(1.f, speed / *g_fPhysicsDamageSpeedMin));
	float damageMult = base + (*g_fPhysicsDamageSpeedMult * speed);

	return damageMult * damage;
}

struct GenericJob
{
	virtual void Run() = 0;
};

struct PointImpulseJob : GenericJob
{
	hkpRigidBody *rigidBody{};
	NiPoint3 point{};
	NiPoint3 impulse{};
	UInt32 refrHandle{};

	PointImpulseJob(hkpRigidBody *rigidBody, const NiPoint3 &point, const NiPoint3 &impulse, UInt32 refrHandle) :
		rigidBody(rigidBody), point(point), impulse(impulse), refrHandle(refrHandle) {}

	virtual void Run() override
	{
		// Need to be safe since the job could run next frame where the rigidbody might not exist anymore
		if (NiPointer<TESObjectREFR> refr; LookupREFRByHandle(refrHandle, refr)) {
			NiPointer<NiNode> root = refr->GetNiNode();
			if (root && FindRigidBody(root, rigidBody)) {
				if (IsMoveableEntity(rigidBody)) {
					hkpEntity_activate(rigidBody);
					rigidBody->m_motion.applyPointImpulse(NiPointToHkVector(impulse), NiPointToHkVector(point));
					//_MESSAGE("Applied point impulse %.2f", VectorLength(impulse));
				}
			}
		}
	}
};

struct LinearImpulseJob : GenericJob
{
	hkpRigidBody *rigidBody{};
	NiPoint3 impulse{};
	UInt32 refrHandle{};

	LinearImpulseJob(hkpRigidBody *rigidBody, const NiPoint3 &impulse, UInt32 refrHandle) :
		rigidBody(rigidBody), impulse(impulse), refrHandle(refrHandle) {}

	virtual void Run() override
	{
		// Need to be safe since the job could run next frame where the rigidbody might not exist anymore
		if (NiPointer<TESObjectREFR> refr; LookupREFRByHandle(refrHandle, refr)) {
			NiPointer<NiNode> root = refr->GetNiNode();
			if (root && FindRigidBody(root, rigidBody)) {
				if (IsMoveableEntity(rigidBody)) {
					hkpEntity_activate(rigidBody);
					rigidBody->m_motion.applyLinearImpulse(NiPointToHkVector(impulse));
					//_MESSAGE("Applied linear impulse %.2f", VectorLength(impulse));
				}
			}
		}
	}
};

std::vector<std::unique_ptr<GenericJob>> g_prePhysicsStepJobs{};

template<class T, typename... Args>
void QueuePrePhysicsJob(Args&&... args)
{
	static_assert(std::is_base_of<GenericJob, T>::value);
	g_prePhysicsStepJobs.push_back(std::make_unique<T>(std::forward<Args>(args)...));
}

struct ControllerVelocityData
{
	std::deque<NiPoint3> velocities{ 5, NiPoint3() };
	NiPoint3 avgVelocity;
	float avgSpeed;

	void RecomputeAverageVelocity()
	{
		avgVelocity = std::accumulate(velocities.begin(), velocities.end(), NiPoint3()) / velocities.size();
	}

	void RecomputeAverageSpeed()
	{
		float speed = 0;
		for (NiPoint3 &velocity : velocities) {
			speed += VectorLength(velocity);
		}
		speed /= std::size(velocities);
		avgSpeed = speed;
	}

	void Recompute()
	{
		RecomputeAverageVelocity();
		RecomputeAverageSpeed();
	}
};
ControllerVelocityData g_controllerVelocities[2]; // one for each hand

std::unordered_set<Actor *> g_activeActors{};
std::unordered_set<UInt16> g_activeBipedGroups{};
std::unordered_set<UInt16> g_hittableCharControllerGroups{};
std::unordered_set<UInt16> g_selfCollidableBipedGroups{};

std::unordered_map<bhkRigidBody *, double> g_higgsLingeringRigidBodies{};
bhkRigidBody * g_rightHand = nullptr;
bhkRigidBody * g_leftHand = nullptr;
bhkRigidBody * g_rightWeapon = nullptr;
bhkRigidBody * g_leftWeapon = nullptr;
bhkRigidBody * g_rightHeldObject = nullptr;
bhkRigidBody * g_leftHeldObject = nullptr;
TESObjectREFR * g_rightHeldRefr = nullptr;
TESObjectREFR * g_leftHeldRefr = nullptr;
UInt16 g_rightHeldCollisionGroup;
UInt16 g_leftHeldCollisionGroup;
UInt32 g_higgsCollisionLayer = 56;

UInt16 g_playerCollisionGroup = 0;

inline bool IsLeftRigidBody(hkpRigidBody *rigidBody)
{
	bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
	if (!wrapper) return false;
	return wrapper == g_leftHand || wrapper == g_leftWeapon || wrapper == g_leftHeldObject;
}

inline bool IsWeaponRigidBody(hkpRigidBody *rigidBody)
{
	bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
	if (!wrapper) return false;
	return wrapper == g_leftWeapon || wrapper == g_rightWeapon;
}

inline bool IsHandRigidBody(hkpRigidBody *rigidBody)
{
	bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
	if (!wrapper) return false;
	return wrapper == g_leftHand || wrapper == g_rightHand;
}

inline bool IsHeldRigidBody(hkpRigidBody *rigidBody)
{
	bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
	if (!wrapper) return false;
	return wrapper == g_leftHeldObject || wrapper == g_rightHeldObject;
}

inline bool IsHiggsRigidBody(hkpRigidBody *rigidBody)
{
	if ((rigidBody->getCollidable()->getBroadPhaseHandle()->getCollisionFilterInfo() & 0x7f) != g_higgsCollisionLayer) {
		return false;
	}
	return IsHandRigidBody(rigidBody) || IsWeaponRigidBody(rigidBody) || IsHeldRigidBody(rigidBody);
}

inline bool IsHittableCharController(TESObjectREFR *refr)
{
	if (refr->formType == kFormType_Character) {
		if (Actor *hitActor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
			UInt32 filterInfo; Actor_GetCollisionFilterInfo(hitActor, filterInfo);
			if (g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(filterInfo >> 16)) {
				return true;
			}
		}
	}
	return false;
}

bool ShouldBumpActor(Actor *actor)
{
	if (Actor_IsRunning(actor) || Actor_IsGhost(actor) || actor->IsInCombat()) return false;
	if (!actor->race || actor->race->data.unk40 >= 2) return false; // race size is >= large

	if (Config::options.dontBumpAnimals && actor->race->keyword.HasKeyword(g_keyword_actorTypeAnimal)) return false;

	if (RelationshipRanks::GetRelationshipRank(actor, *g_thePlayer) > Config::options.bumpMaxRelationshipRank) return false;

	return true;
}

void TryBumpActor(Actor *actor, bool isLargeBump = false)
{
	if (!ShouldBumpActor(actor)) return;

	ActorProcessManager *process = actor->processManager;
	if (!process) return;

	PlayerCharacter *player = *g_thePlayer;
	NiPoint3 actorToPlayer = player->pos - actor->pos;
	float heading = GetHeadingFromVector(actorToPlayer);
	float bumpDirection = heading - get_vfunc<_Actor_GetHeading>(actor, 0xA5)(actor, false);

	ActorProcess_SetBumpState(process, isLargeBump ? 1 : 0);
	ActorProcess_SetBumpDirection(process, bumpDirection);
	Actor_GetBumped(actor, player, isLargeBump, false);
	ActorProcess_SetBumpDirection(process, 0.f);
}

struct BumpRequest
{
	double bumpTime = 0.0;
	bool consumed = false;
};
std::mutex g_bumpActorsLock;
std::unordered_map<Actor *, BumpRequest> g_bumpActors{};

void TryQueueBumpActor(Actor *actor)
{
	double now = GetTime();
	std::unique_lock lock(g_bumpActorsLock);
	if (auto it = g_bumpActors.find(actor); it == g_bumpActors.end() || now - it->second.bumpTime > Config::options.actorBumpCooldownTime) {
		g_bumpActors[actor] = { now, false };
	}
}

bool ShouldKeepOffset(Actor *actor)
{
	if (!Config::options.doKeepOffset) return false;
	if (Actor_IsGhost(actor)) return false;
	if (IsActorUsingFurniture(actor)) return false;

	if (!actor->race || actor->race->data.unk40 >= 2) return false; // race size is >= large

	//if (!actor->race->keyword.HasKeyword(g_keyword_actorTypeNPC)) return false;

	return true;
}

struct KeepOffsetData
{
	double lastAttemptTime = 0.0;
	bool success = false;
};
std::unordered_map<Actor *, KeepOffsetData> g_keepOffsetActors{};

bool ShouldRagdollOnGrab(Actor *actor)
{
	if (!Config::options.ragdollOnGrab) return false;

	TESRace *race = actor->race;
	if (!race) return false;

	if (Config::options.ragdollSmallRacesOnGrab && race->data.unk40 == 0) return true; // small race

	float health = actor->actorValueOwner.GetMaximum(24);
	if (health < Config::options.smallRaceHealthThreshold) return true;

	return false;
}

struct ContactListener : hkpContactListener, hkpWorldPostSimulationListener
{
	struct CollisionEvent
	{
		enum class Type
		{
			Added,
			Removed
		};

		hkpRigidBody *rbA = nullptr;
		hkpRigidBody *rbB = nullptr;
		Type type;
	};

	std::map<std::pair<hkpRigidBody *, hkpRigidBody *>, int> activeCollisions{};
	std::unordered_set<hkpRigidBody *> collidedRigidbodies{};
	std::unordered_set<TESObjectREFR *> collidedRefs{};

	struct CooldownData
	{
		double startTime = 0.0;
		double stoppedCollidingTime = 0.0;
	};
	std::unordered_map<TESObjectREFR *, CooldownData> hitCooldownTargets[2]{}; // each hand has its own cooldown
	std::map<std::pair<Actor *, hkpRigidBody *>, double> physicsHitCooldownTargets{};
	std::vector<CollisionEvent> events{};

	inline std::pair<hkpRigidBody *, hkpRigidBody *> SortPair(hkpRigidBody *a, hkpRigidBody *b) {
		if ((uint64_t)a <= (uint64_t)b) return { a, b };
		else return { b, a };
	}

	// 0 -> right, 1 -> left, 2-> both
	void PlayMeleeImpactRumble(int hand)
	{
		if (hand > 1) {
			VRMeleeData *meleeData = GetVRMeleeData(false);
			PlayRumble(true, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
			meleeData = GetVRMeleeData(true);
			PlayRumble(false, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
		}
		else {
			bool isLeft = hand == 1;
			VRMeleeData *meleeData = GetVRMeleeData(isLeft);
			PlayRumble(!isLeft, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
		}
	}

	NiPoint3 CalculateHitImpulse(hkpRigidBody *rigidBody, const NiPoint3 &hitVelocity, float impulseMult)
	{
		float massInv = rigidBody->getMassInv();
		float mass = massInv <= 0.001f ? 99999.f : 1.f / massInv;

		float impulseStrength = std::clamp(
			Config::options.hitImpulseBaseStrength + Config::options.hitImpulseProportionalStrength * powf(mass, Config::options.hitImpulseMassExponent),
			Config::options.hitImpulseMinStrength, Config::options.hitImpulseMaxStrength
		);

		float impulseSpeed = min(VectorLength(hitVelocity), Config::options.hitImpulseMaxVelocity); // limit the imparted velocity to some reasonable value
		NiPoint3 impulse = VectorNormalized(hitVelocity) * impulseSpeed * *g_havokWorldScale * mass; // This impulse will give the object the exact velocity it is hit with
		impulse *= impulseStrength; // Scale the velocity as we see fit
		impulse *= impulseMult;
		if (impulse.z < 0) {
			// Impulse points downwards somewhat, scale back the downward component so we don't get things shooting into the ground.
			impulse.z *= Config::options.hitImpulseDownwardsMultiplier;
		}

		return impulse;
	}

	void ApplyHitImpulse(Actor *actor, hkpRigidBody *rigidBody, const NiPoint3 &hitVelocity, const NiPoint3 position, float impulseMult)
	{
		UInt32 targetHandle = GetOrCreateRefrHandle(actor);
		// Apply linear impulse at the center of mass to all bodies within 2 ragdoll constraints
		ForEachRagdollDriver(actor, [this, rigidBody, hitVelocity, impulseMult, targetHandle](hkbRagdollDriver *driver) {
			ForEachAdjacentBody(driver, rigidBody, [this, driver, hitVelocity, impulseMult, targetHandle](hkpRigidBody *adjacentBody) {
				QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody, hitVelocity, impulseMult) * Config::options.hitImpulseDecayMult1, targetHandle);
				ForEachAdjacentBody(driver, adjacentBody, [this, driver, hitVelocity, impulseMult, targetHandle](hkpRigidBody *adjacentBody) {
					QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody, hitVelocity, impulseMult) * Config::options.hitImpulseDecayMult2, targetHandle);
					ForEachAdjacentBody(driver, adjacentBody, [this, hitVelocity, impulseMult, targetHandle](hkpRigidBody *adjacentBody) {
						QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody, hitVelocity, impulseMult) * Config::options.hitImpulseDecayMult3, targetHandle);
					});
				});
			});
		});

		// Apply a point impulse at the hit location to the body we actually hit
		QueuePrePhysicsJob<PointImpulseJob>(rigidBody, position, CalculateHitImpulse(rigidBody, hitVelocity, impulseMult), targetHandle);
	}

	void DoHit(TESObjectREFR *hitRefr, hkpRigidBody *hitRigidBody, hkpRigidBody *hittingRigidBody, const hkpContactPointEvent &evnt, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, TESForm *weapon, float impulseMult, bool isLeft, bool isOffhand, bool isTwoHanding)
	{
		PlayerCharacter *player = *g_thePlayer;
		if (hitRefr == player) return;

		bool isPowerAttack = isLeft ? g_isLeftTriggerHeld : g_isRightTriggerHeld;

		// Set attack data
		BGSAttackData *attackData = nullptr;
		PlayerCharacter_UpdateAndGetAttackData(player, *g_isUsingMotionControllers, isOffhand, isPowerAttack, &attackData);
		if (!attackData) return;

		if (isPowerAttack) {
			float staminaCost = ActorValueOwner_GetStaminaCostForAttackData(&player->actorValueOwner, attackData);
			float currentStamina = player->actorValueOwner.GetCurrent(26);
			if (staminaCost > 0.f && currentStamina <= 0.f) {
				// No stamina to power attack, so re-set the attackdata but this time explicitly set powerattack to false
				PlayerCharacter_UpdateAndGetAttackData(player, *g_isUsingMotionControllers, isOffhand, false, &attackData);
				isPowerAttack = false;
			}
		}

		// Hit position / velocity need to be set before Character::HitTarget() which at some point will read from them (during the HitData population)
		NiPoint3 *playerLastHitPosition = (NiPoint3 *)((UInt64)player + 0x6BC);
		*playerLastHitPosition = hitPosition;

		NiPoint3 *playerLastHitVelocity = (NiPoint3 *)((UInt64)player + 0x6C8);
		*playerLastHitVelocity = hitVelocity;

		Character *hitChar = DYNAMIC_CAST(hitRefr, TESObjectREFR, Character);
		if (hitChar && !Actor_IsGhost(hitChar) && Character_CanHit(player, hitChar)) {
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;

			HitActor(player, hitChar, weapon, attackData, isOffhand, isPowerAttack);

			PlayMeleeImpactRumble(isTwoHanding ? 2 : isLeft);

			if (Config::options.applyImpulseOnHit) {
				ApplyHitImpulse(hitChar, hitRigidBody, hitVelocity, hitPosition * *g_havokWorldScale, impulseMult);
			}
		}
		else {
			bool didDispatchHitEvent = HitRefr(player, hitRefr, weapon, hitRigidBody, isLeft, isOffhand);
			if (didDispatchHitEvent) {
				PlayMeleeImpactRumble(isTwoHanding ? 2 : isLeft);
			}

			if (hittingRigidBody->getQualityType() == hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING && !IsMoveableEntity(hitRigidBody)) {
				// Disable contact for keyframed/fixed objects in this case
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
		}

		double now = GetTime();
		hitCooldownTargets[isLeft][hitRefr] = { now, now };
	}

	void ApplyPhysicsDamage(Actor *source, Actor *target, bhkRigidBody *collidingBody, NiPoint3 &hitPos, NiPoint3 &hitNormal)
	{
		bhkCharacterController::CollisionEvent collisionEvent {
			collidingBody,
			hitPos * *g_inverseHavokWorldScale,
			hitNormal * *g_inverseHavokWorldScale,
			HkVectorToNiPoint(collidingBody->hkBody->getLinearVelocity()) * *g_inverseHavokWorldScale
		};

		float massInv = collidingBody->hkBody->getMassInv();
		float mass = massInv <= 0.001f ? 99999.f : 1.f / massInv;
		float speed = VectorLength(collisionEvent.bodyVelocity);
		float damage = GetPhysicsDamage(mass, speed);
		//_MESSAGE("%.2f", damage);

		if (damage > 0.f) {
			HitData hitData;
			HitData_ctor(&hitData);
			HitData_PopulateFromPhysicalHit(&hitData, source, target, damage, collisionEvent);
			// PopulateFromPhysicalHit moves the hit position out from the character a bit, but I don't like that.
			if (Config::options.showCollisionDamageHitFx) {
				hitData.hitPosition = collisionEvent.position;
			}
			else {
				hitData.hitPosition.z -= 10000.f; // move it far below to hide it
			}
			if (source) {
				hitData.aggressor = GetOrCreateRefrHandle(source);
			}
			if (hitData.totalDamage > 0.f) {
				Actor_GetHit(target, hitData);
				if (Config::options.physicsHitRecoveryTime > 0) {
					physicsHitCooldownTargets[{ target, collidingBody->hkBody }] = GetTime();
				}
			}
		}
	}

	virtual void contactPointCallback(const hkpContactPointEvent& evnt) {
		if (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED ||
			!evnt.m_contactPointProperties->isPotential()) {
			return;
		}

		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;

		if ((layerA == BGSCollisionLayer::kCollisionLayer_CharController && (layerB == BGSCollisionLayer::kCollisionLayer_Clutter || layerB == BGSCollisionLayer::kCollisionLayer_Weapon)) ||
			(layerB == BGSCollisionLayer::kCollisionLayer_CharController && (layerA == BGSCollisionLayer::kCollisionLayer_Clutter || layerA == BGSCollisionLayer::kCollisionLayer_Weapon))) {
			if (Config::options.disableClutterVsCharacterControllerCollisionForActiveActors) {
				hkpCollidable *charControllerCollidable = layerA == BGSCollisionLayer::kCollisionLayer_CharController ? &rigidBodyA->m_collidable : &rigidBodyB->m_collidable;
				if (NiPointer<TESObjectREFR> refr = GetRefFromCollidable(charControllerCollidable)) {
					if (refr->formType == kFormType_Character) {
						if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
							if (g_activeActors.count(actor)) {
								// We'll let the clutter object collide with the biped instead
								evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
								return;
							}
						}
					}
				}
			}
		}

		if (((layerA == BGSCollisionLayer::kCollisionLayer_Biped || layerA == BGSCollisionLayer::kCollisionLayer_BipedNoCC) && (layerB == BGSCollisionLayer::kCollisionLayer_Clutter || layerB == BGSCollisionLayer::kCollisionLayer_Weapon)) ||
			((layerB == BGSCollisionLayer::kCollisionLayer_Biped || layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC) && (layerA == BGSCollisionLayer::kCollisionLayer_Clutter || layerA == BGSCollisionLayer::kCollisionLayer_Weapon))) {
			if (NiPointer<TESObjectREFR> refrA = GetRefFromCollidable(&rigidBodyA->m_collidable)) {
				if (NiPointer<TESObjectREFR> refrB = GetRefFromCollidable(&rigidBodyB->m_collidable)) {
					bool isATarget = refrA->formType == kFormType_Character;
					Actor *actor = DYNAMIC_CAST(isATarget ? refrA : refrB, TESObjectREFR, Actor);
					if (!actor) {
						// Disable collision with biped objects that are not actors
						evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
						return;
					}

					if (Config::options.doClutterVsBipedCollisionDamage) {
						hkpRigidBody *hittingBody = isATarget ? rigidBodyB : rigidBodyA;
						if (!physicsHitCooldownTargets.count({ actor, hittingBody })) {
							bhkRigidBody *collidingRigidBody = (bhkRigidBody *)hittingBody->m_userData;
							Actor *aggressor = g_higgsLingeringRigidBodies.count(collidingRigidBody) ? *g_thePlayer : nullptr;
							ApplyPhysicsDamage(aggressor, actor, collidingRigidBody, HkVectorToNiPoint(evnt.m_contactPoint->getPosition()), HkVectorToNiPoint(evnt.m_contactPoint->getNormal()));
						}
					}
				}
			}
		}

		if (layerA != g_higgsCollisionLayer && layerB != g_higgsCollisionLayer) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

		if (layerA == g_higgsCollisionLayer && layerB == g_higgsCollisionLayer) {
			// Both objects are on the higgs layer
			if (!IsMoveableEntity(rigidBodyA) && !IsMoveableEntity(rigidBodyB)) {
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
			return;
		}

		hkpRigidBody *hitRigidBody = layerA == g_higgsCollisionLayer ? rigidBodyB : rigidBodyA;
		hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;

		bhkRigidBody *hittingRigidBodyWrapper = (bhkRigidBody *)hittingRigidBody->m_userData;
		if (!hittingRigidBodyWrapper) {
			if (hittingRigidBody->getQualityType() == hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING && !IsMoveableEntity(hitRigidBody)) {
				// It's not a hit, so disable contact for keyframed/fixed objects in this case
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
			return;
		}

		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (!hitRefr) {
			if (hittingRigidBody->getQualityType() == hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING && !IsMoveableEntity(hitRigidBody)) {
				// It's not a hit, so disable contact for keyframed/fixed objects in this case
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
			return;
		}

		UInt32 hitLayer = hitRigidBody == rigidBodyA ? layerA : layerB;

		bool isLeft = IsLeftRigidBody(hittingRigidBody);

		if (hitCooldownTargets[isLeft].count(hitRefr)) {
			// refr is currently under a hit cooldown, so disable the contact point and gtfo
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			return;
		}

		if (hitLayer == BGSCollisionLayer::kCollisionLayer_CharController && !IsHittableCharController(hitRefr)) {
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			return;
		}

		// A contact point of any sort confirms a collision, regardless of any collision added or removed events

		hkVector4 hkHitPos = evnt.m_contactPoint->getPosition();
		hkVector4 pointVelocity; hittingRigidBody->getPointVelocity(hkHitPos, pointVelocity);
		NiPoint3 hkHitVelocity = HkVectorToNiPoint(pointVelocity);
		float hitSpeed = VectorLength(hkHitVelocity);
		// TODO: Make the hit speed affect damage? (how?)
		bool isOffhand = *g_leftHandedMode ? !isLeft : isLeft;
		PlayerCharacter *player = *g_thePlayer;
		TESForm *equippedObj = player->GetEquippedObject(isOffhand);
		TESObjectWEAP *weap = DYNAMIC_CAST(equippedObj, TESForm, TESObjectWEAP);

		NiPoint3 handDirection = VectorNormalized(g_controllerVelocities[isLeft].avgVelocity);
		float handSpeedRoomspace = g_controllerVelocities[isLeft].avgSpeed;
		bool isTwoHanding = g_higgsInterface->IsTwoHanding();
		if (isTwoHanding) {
			handDirection = VectorNormalized(g_controllerVelocities[0].avgVelocity + g_controllerVelocities[1].avgVelocity);
			handSpeedRoomspace = max(g_controllerVelocities[0].avgSpeed, g_controllerVelocities[1].avgSpeed);
		}

		bool isStab = false, isPunch = false, isSwing = false;
		if (weap && CanWeaponStab(weap)) {
			// Check for stab
			// All stabbable weapons use the melee weapon offset node
			NiPointer<NiAVObject> weaponOffsetNode = player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftMeleeWeaponOffsetNode : PlayerCharacter::Node::kNode_RightMeleeWeaponOffsetNode];
			if (weaponOffsetNode) {
				// Use last frame's offset node transform because this frame is not over yet and it can still be modified by e.g. higgs two-handing
				NiPoint3 weaponForward = ForwardVector(weaponOffsetNode->m_oldWorldTransform.rot);
				float stabAmount = DotProduct(handDirection, weaponForward);
				//_MESSAGE("Stab amount: %.2f", stabAmount);
				if (stabAmount > Config::options.hitStabDirectionThreshold && hitSpeed > Config::options.hitStabSpeedThreshold) {
					isStab = true;
				}
			}
		}
		else if (!equippedObj || (weap && weap->type() == TESObjectWEAP::GameData::kType_HandToHandMelee)) {
			// Check for punch
			// For punching use the hand node
			NiPointer<NiAVObject> handNode = player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftHandBone: PlayerCharacter::Node::kNode_RightHandBone];
			if (handNode) {
				NiPoint3 punchVector = UpVector(handNode->m_worldTransform.rot); // in the direction of fingers when fingers are extended
				float punchAmount = DotProduct(handDirection, punchVector);
				//_MESSAGE("Punch amount: %.2f", punchAmount);
				if (punchAmount > Config::options.hitPunchDirectionThreshold && hitSpeed > Config::options.hitPunchSpeedThreshold) {
					isPunch = true;
				}
			}
		}
		
		if (!isStab && !isPunch && hitSpeed > Config::options.hitSwingSpeedThreshold) {
			isSwing = true;
		}

		// Thresholding on some (small) roomspace hand velocity helps prevent hits while moving around / turning

		bool doHit = (isSwing || isStab || isPunch);
		bool disableHit = handSpeedRoomspace < Config::options.hitRequiredHandSpeedRoomspace || (!player->actorState.IsWeaponDrawn() && Config::options.disableHitIfSheathed);

		if (doHit && !disableHit) {
			float havokWorldScale = *g_havokWorldScale;

			NiPoint3 hitPosition = HkVectorToNiPoint(hkHitPos) / havokWorldScale; // skyrim units
			NiPoint3 hitVelocity; // skyrim units
			if (isStab && Config::options.useHandVelocityForStabHitDirection) {
				hitVelocity = handDirection * handSpeedRoomspace / havokWorldScale;
			}
			else {
				hitVelocity = hkHitVelocity / havokWorldScale;
			}

			float impulseMult = isStab ? Config::options.hitStabImpulseMult : (isPunch ? Config::options.hitPunchImpulseMult : Config::options.hitSwingImpulseMult);
			DoHit(hitRefr, hitRigidBody, hittingRigidBody, evnt, hitPosition, hitVelocity, equippedObj, impulseMult, isLeft, isOffhand, isTwoHanding);
		}
		else if (hitRefr->formType == kFormType_Character && doHit && disableHit) {
			// Hit is disabled and we hit a character. Disable this contact point but don't disable future ones.
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
		}
		else {
			// It's not a hit, so disable contact for keyframed/fixed objects in this case
			if (hittingRigidBody->getQualityType() == hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING && !IsMoveableEntity(hitRigidBody)) {
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}

			if (Config::options.bumpActorsWhenTouched) {
				if (hitRefr->formType == kFormType_Character) {
					if (Actor *actor = DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor)) {
						TryQueueBumpActor(actor);
					}
				}
			}
		}
	}

	virtual void collisionAddedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != g_higgsCollisionLayer && layerB != g_higgsCollisionLayer) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

		if (layerA == g_higgsCollisionLayer && layerB == g_higgsCollisionLayer) return; // Both objects are on the higgs layer

		events.push_back({ rigidBodyA, rigidBodyB, CollisionEvent::Type::Added });
		//_MESSAGE("%d Added %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
	}

	virtual void collisionRemovedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		// Technically our objects could have changed layers or something between added and removed
		events.push_back({ rigidBodyA, rigidBodyB, CollisionEvent::Type::Removed });
		//_MESSAGE("%d Removed %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
	}

	virtual void postSimulationCallback(hkpWorld* world)
	{
		// First just accumulate adds/removes. Why? While Added always occurs before Removed for a single contact point,
		// a single pair of rigid bodies can have multiple contact points, and adds/removes between these different contact points can be non-deterministic.
		for (CollisionEvent &evnt : events) {
			if (evnt.type == CollisionEvent::Type::Added) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				int count = activeCollisions.count(pair) ? activeCollisions[pair] : 0;
				activeCollisions[pair] = count + 1;
			}
			else if (evnt.type == CollisionEvent::Type::Removed) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				int count = activeCollisions.count(pair) ? activeCollisions[pair] : 0;
				activeCollisions[pair] = count - 1;
			}
		}
		events.clear();

		// Clear out any collisions that are no longer active (or that were only removed, since we do events for any removes but only some adds)
		collidedRigidbodies.clear();
		collidedRefs.clear();
		for (auto it = activeCollisions.begin(); it != activeCollisions.end();) {
			auto[pair, count] = *it;
			if (count <= 0) {
				it = activeCollisions.erase(it);
			}
			else {
				auto[bodyA, bodyB] = pair;
				hkpRigidBody *collidedBody = IsHiggsRigidBody(bodyA) ? bodyB : bodyA;
				collidedRigidbodies.insert(collidedBody);

				if (TESObjectREFR *ref = GetRefFromCollidable(collidedBody->getCollidable())) {
					collidedRefs.insert(ref);
				}

				++it;
			}
		}

		double now = GetTime();

		// Now fill in the currently collided-with actors based on active collisions
		for (auto[pair, count] : activeCollisions) {
			auto[rigidBodyA, rigidBodyB] = pair;

			UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
			UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;

			hkpRigidBody *hitRigidBody = layerA == g_higgsCollisionLayer ? rigidBodyB : rigidBodyA;
			hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;

			NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
			if (hitRefr) {
				bool isLeft = IsLeftRigidBody(hittingRigidBody);
				if (hitCooldownTargets[isLeft].count(hitRefr)) {
					// refr is still collided with, so refresh its hit cooldown
					hitCooldownTargets[isLeft][hitRefr].stoppedCollidingTime = now;
				}
			}
		}

		// Clear out old hit cooldown targets
		for (auto &targets : hitCooldownTargets) { // For each hand's cooldown targets
			for (auto it = targets.begin(); it != targets.end();) {
				auto[target, cooldown] = *it;
				if ((now - cooldown.stoppedCollidingTime) * *g_globalTimeMultiplier > Config::options.hitCooldownTimeStoppedColliding ||
					(now - cooldown.startTime) * *g_globalTimeMultiplier > Config::options.hitCooldownTimeFallback)
					it = targets.erase(it);
				else
					++it;
			}
		}

		// Clear out old physics hit cooldown targets
		for (auto it = physicsHitCooldownTargets.begin(); it != physicsHitCooldownTargets.end();) {
			auto[target, hitTime] = *it;
			if ((now - hitTime) * *g_globalTimeMultiplier > Config::options.physicsHitRecoveryTime)
				it = physicsHitCooldownTargets.erase(it);
			else
				++it;
		}

		{
			std::unique_lock lock(g_bumpActorsLock);
			for (auto it = g_bumpActors.begin(); it != g_bumpActors.end();) {
				double bumpTime = it->second.bumpTime;
				if (now - bumpTime > Config::options.actorBumpCooldownTime)
					it = g_bumpActors.erase(it);
				else
					++it;
			}
		}
	}

	NiPointer<bhkWorld> world = nullptr;
};
ContactListener g_contactListener{};

struct PlayerCharacterProxyListener : hkpCharacterProxyListener
{
	// Called when the character interacts with another (non fixed or keyframed) rigid body.
	virtual void objectInteractionCallback(hkpCharacterProxy* proxy, const hkpCharacterObjectInteractionEvent& input, hkpCharacterObjectInteractionResult& output)
	{
		hkpRigidBody *hitBody = input.m_body;
		if (!hitBody) return;

		const hkpCollidable *collidable = hitBody->getCollidable();
		UInt32 layer = collidable->getBroadPhaseHandle()->getCollisionFilterInfo() & 0x7f;
		if (layer != BGSCollisionLayer::kCollisionLayer_Biped) return;

		NiPointer<TESObjectREFR> refr = GetRefFromCollidable(collidable);
		if (!refr) return;

		if (refr->formType != kFormType_Character) return;

		Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
		if (!actor) return;

		bhkCharacterController *controller = GetCharacterController(actor);
		if (!controller) return;

		CharacterCollisionHandler *collisionHandler = *g_characterCollisionHandler;
		if (!collisionHandler) return;

		bhkCharacterController *playerController = GetCharacterController(*g_thePlayer);
		if (!playerController) return;

		collisionHandler->HandleCharacterCollision(playerController, controller);
	}

	bhkCharProxyController *proxy = nullptr; // for reference purposes only, do not dereference
};
PlayerCharacterProxyListener g_characterProxyListener{};


using CollisionFilterComparisonResult = HiggsPluginAPI::IHiggsInterface001::CollisionFilterComparisonResult;
CollisionFilterComparisonResult CollisionFilterComparisonCallback(void *filter, UInt32 filterInfoA, UInt32 filterInfoB)
{
	UInt32 layerA = filterInfoA & 0x7f;
	UInt32 layerB = filterInfoB & 0x7f;

	if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped || layerA == BGSCollisionLayer::kCollisionLayer_BipedNoCC) && (layerB == BGSCollisionLayer::kCollisionLayer_Biped || layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC)) {
		// Biped vs. biped
		UInt16 groupA = filterInfoA >> 16;
		UInt16 groupB = filterInfoB >> 16;
		if (groupA == groupB) {
			// biped self-collision
			if (g_selfCollidableBipedGroups.count(groupA)) {
				return CollisionFilterComparisonResult::Continue; // will collide with all non-adjacent bones
			}
			else {
				return CollisionFilterComparisonResult::Ignore;
			}
		}
	}

	if (layerA != BGSCollisionLayer::kCollisionLayer_CharController && layerB != BGSCollisionLayer::kCollisionLayer_CharController) {
		// Neither collidee is a character controller
		return CollisionFilterComparisonResult::Continue;
	}

	if (layerA == BGSCollisionLayer::kCollisionLayer_CharController && layerB == BGSCollisionLayer::kCollisionLayer_CharController) {
		// Both collidees are character controllers. If one of them is the player, ignore the collision.
		UInt16 groupA = filterInfoA >> 16;
		UInt16 groupB = filterInfoB >> 16;
		if (groupA == g_playerCollisionGroup || groupB == g_playerCollisionGroup) {
			return CollisionFilterComparisonResult::Ignore;
		}
		return CollisionFilterComparisonResult::Continue;
	}

	// One of the collidees is a character controller

	UInt32 charControllerFilter = layerA == BGSCollisionLayer::kCollisionLayer_CharController ? filterInfoA : filterInfoB;
	UInt16 group = charControllerFilter >> 16;
	if (group != g_playerCollisionGroup) {
		// It's not the player

		UInt32 otherFilter = charControllerFilter == filterInfoA ? filterInfoB : filterInfoA;
		UInt16 otherGroup = otherFilter >> 16;
		if (otherGroup == g_playerCollisionGroup) {
			// Whatever collided with the charcontroller belongs to the player
			UInt32 otherLayer = otherFilter & 0x7f;
			if (otherLayer == g_higgsCollisionLayer) {
				// Higgs vs. non-player character controller
				if (g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(group)) {
					return CollisionFilterComparisonResult::Collide;
				}
				else {
					return CollisionFilterComparisonResult::Ignore;
				}
			}
		}

		return CollisionFilterComparisonResult::Continue;
	}

	// The character controller belongs to the player

	UInt32 otherFilter = charControllerFilter == filterInfoA ? filterInfoB : filterInfoA;
	UInt32 otherLayer = otherFilter & 0x7f;
	UInt16 otherGroup = otherFilter >> 16;

	if (otherGroup != g_playerCollisionGroup) {
		if (otherLayer == BGSCollisionLayer::kCollisionLayer_Biped || otherLayer == BGSCollisionLayer::kCollisionLayer_BipedNoCC) {
			// Collide with the biped unless we want to explicitly ignore them
			if (!Config::options.enablePlayerBipedCollision ||
				(g_rightHeldObject && otherGroup == g_rightHeldCollisionGroup) ||
				(g_leftHeldObject && otherGroup == g_leftHeldCollisionGroup)) {
				return CollisionFilterComparisonResult::Ignore;
			}

			if (!g_activeBipedGroups.count(otherGroup)) {
				// Disable collision with biped objects that are not actors
				return CollisionFilterComparisonResult::Ignore;
			}

			return CollisionFilterComparisonResult::Collide;
		}
	}

	return CollisionFilterComparisonResult::Continue;
}

void PrePhysicsStepCallback(void *world)
{
	// This hook is after all ragdolls' driveToPose(), and before the hkpWorld physics step

	// At this point we can apply any impulses / velocity adjustments without fear of them being overwritten

	for (auto &job : g_prePhysicsStepJobs) {
		job.get()->Run();
	}
	g_prePhysicsStepJobs.clear();

	// With the exe patched to not enable its melee collision, we still need to disable it once (after it's created)
	for (int i = 0; i < 2; i++) {
		VRMeleeData *meleeData = GetVRMeleeData(i);
		NiPointer<NiAVObject> collNode = meleeData->collisionNode;
		if (!collNode) continue;
		NiPointer<bhkRigidBody> rb = GetRigidBody(collNode);
		if (!rb) continue;
		if (!(rb->hkBody->m_collidable.getCollisionFilterInfo() >> 14 & 1)) {
			// collision is enabled
			rb->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= 0x4000; // disable collision
			bhkWorldObject_UpdateCollisionFilter(rb);
		}
	}
}

std::unordered_map<hkbRagdollDriver *, ActiveRagdoll> g_activeRagdolls{};

hkaKeyFrameHierarchyUtility::Output g_stressOut[200]; // set in a hook during driveToPose(). Just reserve a bunch of space so it can handle any number of bones.

hkArray<hkVector4> g_scratchHkArray{}; // We can't call the destructor of this ourselves, so this is a global array to be used at will and never deallocated.

bool IsAddedToWorld(Actor *actor)
{
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
	if (!GetAnimationGraphManager(actor, animGraphManager)) return false;

	BSAnimationGraphManager *manager = animGraphManager.ptr;
	{
		SimpleLocker lock(&manager->updateLock);

		if (manager->graphs.size <= 0) return false;

		for (int i = 0; i < manager->graphs.size; i++) {
			BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
			if (!graph.ptr->world) return false;

			hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
			if (!driver) return false;
			hkaRagdollInstance *ragdoll = driver->ragdoll;
			if (!ragdoll) return false;
			if (!ragdoll->getWorld()) return false;
		}
	}

	return true;
}

bool CanAddToWorld(Actor *actor)
{
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
	if (!GetAnimationGraphManager(actor, animGraphManager)) return false;

	BSAnimationGraphManager *manager = animGraphManager.ptr;
	{
		SimpleLocker lock(&manager->updateLock);

		if (manager->graphs.size <= 0) return false;

		for (int i = 0; i < manager->graphs.size; i++) {
			BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
			hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
			if (!driver) return false;
			hkaRagdollInstance *ragdoll = driver->ragdoll;
			if (!ragdoll) return false;
		}
	}

	return true;
}

void ModifyConstraints(Actor *actor)
{
	ForEachRagdollDriver(actor, [](hkbRagdollDriver *driver) {
		hkaRagdollInstance *ragdoll = hkbRagdollDriver_getRagdoll(driver);
		if (!ragdoll) return;

		for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
			NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable);
			if (node) {
				NiPointer<bhkRigidBody> wrapper = GetRigidBody(node);
				if (wrapper) {
					bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
				}
			}
		}

		if (Config::options.convertHingeConstraintsToRagdollConstraints) {
			// Convert any limited hinge constraints to ragdoll constraints so that they can be loosened properly
			for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
				NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable);
				if (node) {
					NiPointer<bhkRigidBody> wrapper = GetRigidBody(node);
					if (wrapper) {
						for (int i = 0; i < wrapper->constraints.count; i++) {
							bhkConstraint *constraint = wrapper->constraints.entries[i];
							if (constraint->constraint->getData()->getType() == hkpConstraintData::CONSTRAINT_TYPE_LIMITEDHINGE) {
								bhkRagdollConstraint *ragdollConstraint = ConvertToRagdollConstraint(constraint);
								if (ragdollConstraint) {
									constraint->RemoveFromCurrentWorld();

									bhkWorld *world = wrapper->GetHavokWorld_1()->m_userData;
									ragdollConstraint->MoveToWorld(world);
									wrapper->constraints.entries[i] = ragdollConstraint;
								}
							}
						}
					}
				}
			}
		}
	});
}

bool AddRagdollToWorld(Actor *actor)
{
	if (Actor_IsInRagdollState(actor)) return false;

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
					BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
					hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
					if (driver) {
						g_activeActors.insert(actor);

						g_activeRagdolls[driver] = ActiveRagdoll{};
						ActiveRagdoll &ragdoll = g_activeRagdolls[driver];

						double now = GetTime();
						Blender &blender = ragdoll.blender;
						blender.StartBlend(Blender::BlendType::AnimToRagdoll, now, Config::options.blendInTime);
						hkQsTransform *poseLocal = hkbCharacter_getPoseLocal(driver->character);
						blender.initialPose.assign(poseLocal, poseLocal + driver->character->numPoseLocal);
						blender.isFirstBlendFrame = false;

						ragdoll.stateChangedTime = now;
						ragdoll.state = RagdollState::BlendIn;

						if (!graph.ptr->world) {
							// World must be set before calling BShkbAnimationGraph::AddRagdollToWorld(), and is required for the graph to register its physics step listener (and hence call hkbRagdollDriver::driveToPose())
							graph.ptr->world = GetHavokWorldFromCell(actor->parentCell);
							ragdoll.shouldNullOutWorldWhenRemovingFromWorld = true;
						}
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
	if (Actor_IsInRagdollState(actor)) return false;

	bool hasRagdollInterface = false;
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
	if (GetAnimationGraphManager(actor, animGraphManager)) {
		BSAnimationGraphManager_HasRagdollInterface(animGraphManager.ptr, &hasRagdollInterface);
	}

	if (hasRagdollInterface) {
		// TODO: We should not remove the ragdoll from the world if it had the ragdoll added already when we added it (e.g. race allowragdollcollision flag).
		//       In that case we should also revert the motion type to keyframed since that's what it usually is in this scenario.
		bool x = false;
		BSAnimationGraphManager_RemoveRagdollFromWorld(animGraphManager.ptr, &x);

		if (GetAnimationGraphManager(actor, animGraphManager)) {
			BSAnimationGraphManager *manager = animGraphManager.ptr;
			{
				SimpleLocker lock(&manager->updateLock);
				for (int i = 0; i < manager->graphs.size; i++) {
					BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
					hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
					if (driver) {
						if (g_activeRagdolls.count(driver) && g_activeRagdolls[driver].shouldNullOutWorldWhenRemovingFromWorld) {
							graph.ptr->world = nullptr;
						}
						g_activeRagdolls.erase(driver);

						g_activeActors.erase(actor);
					}
				}
			}
		}
	}

	return true;
}

void DisableSyncOnUpdate(Actor *actor)
{
	if (Actor_IsInRagdollState(actor)) return;

	bool hasRagdollInterface = false;
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
	if (GetAnimationGraphManager(actor, animGraphManager)) {
		BSAnimationGraphManager_HasRagdollInterface(animGraphManager.ptr, &hasRagdollInterface);
	}

	if (hasRagdollInterface) {
		bool x[2] = { false, true };
		BSAnimationGraphManager_DisableOrEnableSyncOnUpdate(animGraphManager.ptr, x);
	}
}

void UpdateCollisionFilterOnAllBones(Actor *actor)
{
	if (Actor_IsInRagdollState(actor)) return;

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
					BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
					if (hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver) {
						if (hkaRagdollInstance *ragdoll = driver->ragdoll) {
							if (ahkpWorld *world = (ahkpWorld *)ragdoll->getWorld()) {
								bhkWorld *worldWrapper = world->m_userData;
								{
									BSWriteLocker lock(&worldWrapper->worldLock);

									for (hkpRigidBody *body : ragdoll->m_rigidBodies) {
										hkpWorld_UpdateCollisionFilterOnEntity(world, body, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_IGNORE_SHAPE_COLLECTIONS);
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

struct KeepOffsetTask : TaskDelegate
{
	static KeepOffsetTask * Create(UInt32 source, UInt32 target) {
		KeepOffsetTask * cmd = new KeepOffsetTask;
		if (cmd) {
			cmd->source = source;
			cmd->target = target;
		}
		return cmd;
	}

	virtual void Run() {
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(source, refr)) {
			if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
				Actor_KeepOffsetFromActor(actor, target, NiPoint3(0.f, 0.f, 0.f), NiPoint3(0.f, 0.f, 0.f), 150.f, 50.f);
			}
		}
	}

	virtual void Dispose() {
		delete this;
	}

	UInt32 source;
	UInt32 target;
};

void ProcessHavokHitJobsHook()
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return;

	TESObjectCELL *cell = player->parentCell;
	if (!cell) return;

	NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell);
	if (!world) return;

	AIProcessManager *processManager = *g_aiProcessManager;
	if (!processManager) return;

	{
		UInt32 filterInfo; Actor_GetCollisionFilterInfo(player, filterInfo);
		g_playerCollisionGroup = filterInfo >> 16;
	}

	if (world != g_contactListener.world) {
		if (NiPointer<bhkWorld> oldWorld = g_contactListener.world) {
			_MESSAGE("Removing listeners from old havok world");
			{
				BSWriteLocker lock(&oldWorld->worldLock);
				hkpWorldExtension *collisionCallbackExtension = hkpWorld_findWorldExtension(oldWorld->world, hkpKnownWorldExtensionIds::HK_WORLD_EXTENSION_COLLISION_CALLBACK);
				if (collisionCallbackExtension) {
					// There are times when the collision callback extension is gone even if we required it earlier...
					hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(oldWorld->world);
				}
				hkpWorld_removeContactListener(oldWorld->world, &g_contactListener);
				hkpWorld_removeWorldPostSimulationListener(oldWorld->world, &g_contactListener);
			}

			g_activeActors.clear();
			g_activeRagdolls.clear();
			g_activeBipedGroups.clear();
			g_hittableCharControllerGroups.clear();
			g_selfCollidableBipedGroups.clear();
			g_higgsLingeringRigidBodies.clear();
			g_keepOffsetActors.clear();
			g_contactListener = ContactListener{};
		}

		_MESSAGE("Havok world changed");
		{
			BSWriteLocker lock(&world->worldLock);

			hkpCollisionCallbackUtil_requireCollisionCallbackUtil(world->world);

			hkpWorld_addContactListener(world->world, &g_contactListener);
			hkpWorld_addWorldPostSimulationListener(world->world, &g_contactListener);

			bhkCollisionFilter *filter = (bhkCollisionFilter *)world->world->m_collisionFilter;

			if (Config::options.disableBipedCollisionWithWorld) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] = 0; // disable biped collision with anything
			}
			if (Config::options.enableBipedBipedCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped); // enable biped->biped collision;
			}
			if (Config::options.enableBipedClutterCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Clutter); // enable collision with clutter objects
			}
			if (Config::options.enableBipedWeaponCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Weapon);
			}
			if (Config::options.enableBipedDeadBipCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_DeadBip);
			}
			ReSyncLayerBitfields(filter, BGSCollisionLayer::kCollisionLayer_Biped);

			if (Config::options.enableBipedBipedCollisionNoCC) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_BipedNoCC] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC);
			}
			if (Config::options.enableBipedDeadBipCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_BipedNoCC] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_DeadBip);
			}
			ReSyncLayerBitfields(filter, BGSCollisionLayer::kCollisionLayer_BipedNoCC);
		}

		g_contactListener.world = world;
	}

	if (NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer)) {
		if (controller != g_characterProxyListener.proxy) {
			if (RE::hkRefPtr<hkpCharacterProxy> proxy = controller->proxy.characterProxy) {
				_MESSAGE("Player Character Proxy changed");

				BSWriteLocker lock(&world->worldLock);

				if (hkpCharacterProxy_findCharacterProxyListener(proxy, &g_characterProxyListener) == -1) {
					hkpCharacterProxy_addCharacterProxyListener(proxy, &g_characterProxyListener);
				}

				hkpListShape *listShape = ((hkpListShape*)proxy->m_shapePhantom->m_collidable.m_shape);

				if (Config::options.resizePlayerCharController) {
					// Shrink convex charcontroller shape
					g_scratchHkArray.clear();
					hkArray<hkVector4> &verts = g_scratchHkArray;

					hkpConvexVerticesShape *convexVerticesShape = ((hkpConvexVerticesShape *)listShape->m_childInfo[0].m_shape);
					hkpConvexVerticesShape_getOriginalVertices(convexVerticesShape, verts);

					// The charcontroller shape is composed of two vertically concentric "rings" with a single point above and below the top/bottom ring.
					// verts 0,2,6,10,12,14,15,17 are bottom ring, 8-9 are bottom/top points, 1,3,4,5,7,11,13,16 are top ring

					if (Config::options.adjustPlayerCharControllerBottomRingHeightToMaintainSlope) {
						// Move the bottom ring downwards so that the the slope between the bottom ring and the bottom point remains the same with the new ring radius.
						// This is to try and maintain the same stair-climbing behavior, though it could be an issue for very high steps since we move the bottom ring down.

						NiPoint3 bottomVert = HkVectorToNiPoint(verts[8]); // the single bottom point of the shape
						NiPoint3 bottomRingVert = HkVectorToNiPoint(verts[2]); // one of the points on the bottom ring of the shape

						float zOld = bottomRingVert.z - bottomVert.z;
						float rOld = VectorLength({ bottomRingVert.x, bottomRingVert.y });

						float rNew = Config::options.playerCharControllerRadius;
						float zNew = rNew * (zOld / rOld);
						float newBottomRingHeight = bottomVert.z + zNew;

						for (int i : {
							0, 2, 6, 10, 12, 14, 15, 17 // bottom ring
						}) {
							NiPoint3 vert = HkVectorToNiPoint(verts[i]);
							vert.z = newBottomRingHeight;
							verts[i] = NiPointToHkVector(vert);
						}
					}

					// Shrink the two rings of the charcontroller shape by moving the rings' vertices inwards
					for (int i : {
						1, 3, 4, 5, 7, 11, 13, 16, // top ring
							0, 2, 6, 10, 12, 14, 15, 17 // bottom ring
					}) {
						NiPoint3 vert = HkVectorToNiPoint(verts[i]);
						NiPoint3 newVert = vert;
						newVert.z = 0;
						newVert = VectorNormalized(newVert) * Config::options.playerCharControllerRadius;
						newVert.z = vert.z;

						verts[i] = NiPointToHkVector(newVert);
					}

					hkStridedVertices newVerts(verts);

					//hkpConvexVerticesShape::BuildConfig buildConfig{false, false, true, 0.05f, 0, 0.05f, 0.07f, -0.1f}; // defaults
					//hkpConvexVerticesShape::BuildConfig buildConfig{ true, false, true, 0.05f, 0, 0, 0, -0.1f }; // some havok func uses these values
					hkpConvexVerticesShape::BuildConfig buildConfig{ false, false, true, 0.05f, 0, 0.f, 0.f, -0.1f };

					hkpConvexVerticesShape *newShape = (hkpConvexVerticesShape *)hkHeapAlloc(sizeof(hkpConvexVerticesShape));
					hkpConvexVerticesShape_ctor(newShape, newVerts, buildConfig); // sets refcount to 1

					// it's actually a hkCharControllerShape not just a hkpConvexVerticesShape
					set_vtbl(newShape, hkCharControllerShape_vtbl);

					bhkShape *wrapper = (bhkShape*)convexVerticesShape->m_userData;
					wrapper->SetHavokObject(newShape);

					// The listshape does not use a hkRefPtr but it's still setup to add a reference upon construction and remove one on destruction
					listShape->m_childInfo[0].m_shape = newShape;
					hkReferencedObject_removeReference(convexVerticesShape); // this will usually call the dtor on the old shape

					// We don't need to remove a ref here, the ctor gave it a refcount of 1 and we assigned it to the listShape which isn't technically a hkRefPtr but still owns it (and the listShape's dtor will decref anyways)
					// hkReferencedObject_removeReference(newShape);
				}

				if (Config::options.resizePlayerCapsule) {
					// TODO: Am I accidentally modifying every npc's capsule too? I don't think so.
					// Shrink capsule shape too. It's active when weapons are unsheathed.
					float radius = Config::options.playerCapsuleRadius;
					hkpCapsuleShape *capsule = ((hkpCapsuleShape *)listShape->m_childInfo[1].m_shape);
					float originalRadius = capsule->m_radius;
					capsule->m_radius = radius;

					NiPoint3 vert0 = HkVectorToNiPoint(capsule->getVertex(0));
					NiPoint3 vert1 = HkVectorToNiPoint(capsule->getVertex(1));

					if (Config::options.centerPlayerCapsule) {
						vert0.x = 0.f;
						vert0.y = 0.f;
						vert1.x = 0.f;
						vert1.y = 0.f;
					}

					if (vert0.z < vert1.z) {
						// vert0 is the lower vertex
						vert1.z += originalRadius - radius;;
						vert0.z -= originalRadius - radius;
					}
					else {
						vert0.z += originalRadius - radius;;
						vert1.z -= originalRadius - radius;
					}
					capsule->setVertex(0, NiPointToHkVector(vert0));
					capsule->setVertex(1, NiPointToHkVector(vert1));
				}

				g_characterProxyListener.proxy = controller;
			}
		}
	}

	double now = GetTime();

	{ // Update higgs info
		NiPointer<bhkRigidBody> rightHand = (bhkRigidBody *)g_higgsInterface->GetHandRigidBody(false); // this one's a nipointer because we need to actually read from it
		g_rightHand = rightHand;
		g_leftHand = (bhkRigidBody *)g_higgsInterface->GetHandRigidBody(true);

		NiPointer<bhkRigidBody> rightWeapon = (bhkRigidBody *)g_higgsInterface->GetWeaponRigidBody(false);
		NiPointer<bhkRigidBody> leftWeapon = (bhkRigidBody *)g_higgsInterface->GetWeaponRigidBody(true);
		g_rightWeapon = rightWeapon;
		g_leftWeapon = leftWeapon;

		// TODO: We don't make held objects or hands keyframed_reporting, so those can't do hits on statics
		if (rightWeapon && rightWeapon->hkBody->getQualityType() != hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING) {
			rightWeapon->hkBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
			bhkWorld_UpdateCollisionFilterOnWorldObject(world, rightWeapon);
		}
		if (leftWeapon && leftWeapon->hkBody->getQualityType() != hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING) {
			leftWeapon->hkBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
			bhkWorld_UpdateCollisionFilterOnWorldObject(world, leftWeapon);
		}

		if (rightHand) {
			g_higgsCollisionLayer = rightHand->hkBody->m_collidable.getBroadPhaseHandle()->m_collisionFilterInfo & 0x7f;
		}

		g_rightHeldObject = (bhkRigidBody *)g_higgsInterface->GetGrabbedRigidBody(false);
		g_leftHeldObject = (bhkRigidBody *)g_higgsInterface->GetGrabbedRigidBody(true);

		g_rightHeldRefr = g_higgsInterface->GetGrabbedObject(false);
		g_leftHeldRefr = g_higgsInterface->GetGrabbedObject(true);

		if (g_rightHeldObject) {
			g_higgsLingeringRigidBodies[g_rightHeldObject] = now;
		}
		if (g_leftHeldObject) {
			g_higgsLingeringRigidBodies[g_leftHeldObject] = now;
		}

		// Clear out old dropped / thrown rigidbodies
		for (auto it = g_higgsLingeringRigidBodies.begin(); it != g_higgsLingeringRigidBodies.end();) {
			auto[target, hitTime] = *it;
			if ((now - hitTime) * *g_globalTimeMultiplier >= Config::options.thrownObjectLingerTime)
				it = g_higgsLingeringRigidBodies.erase(it);
			else
				++it;
		}
	}

	{ // Ensure our listener is the last one (will be called first)
		hkArray<hkpContactListener*> &listeners = world->world->m_contactListeners;
		if (listeners[listeners.getSize() - 1] != &g_contactListener) {
			BSWriteLocker lock(&world->worldLock);

			int numListeners = listeners.getSize();
			int listenerIndex = listeners.indexOf(&g_contactListener);
			if (listenerIndex >= 0) {
				for (int i = listenerIndex + 1; i < numListeners; ++i) {
					listeners[i - 1] = listeners[i];
				}
				listeners[numListeners - 1] = &g_contactListener;
			}
		}
	}

	for (UInt32 i = 0; i < processManager->actorsHigh.count; i++) {
		UInt32 actorHandle = processManager->actorsHigh[i];
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(actorHandle, refr) && refr != player) {
			Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
			if (!actor || !actor->GetNiNode()) continue;

			TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);

			UInt32 filterInfo; Actor_GetCollisionFilterInfo(actor, filterInfo);
			UInt16 collisionGroup = filterInfo >> 16;

			bool isHeld = actor == g_rightHeldRefr || actor == g_leftHeldRefr;
			if (isHeld) {
				if (!Actor_IsInRagdollState(actor)) {
					if (ShouldRagdollOnGrab(actor)) {
						if (ActorProcessManager *process = actor->processManager) {
							ActorProcess_PushActorAway(process, actor, player->pos, 0.f);
						}
					}
					else if (ShouldKeepOffset(actor)) {
						if (auto it = g_keepOffsetActors.find(actor); it == g_keepOffsetActors.end()) {
							// Wasn't grabbed before
							g_taskInterface->AddTask(KeepOffsetTask::Create(GetOrCreateRefrHandle(actor), GetOrCreateRefrHandle(player)));

							// At some point we probably want to do a better job of this and use offsets from the actor itself rather than the player
							//UInt32 handle = actorHandle;
							//Actor_KeepOffsetFromActor(actor, handle, NiPoint3(0.f, 100.f, 0.f), NiPoint3(0.f, 0.f, 0.f), 150.f, 0.f);

							g_keepOffsetActors[actor] = { now, false };
						}
						else {
							// Already in the set, so check if it actually succeeded at first
							KeepOffsetData &data = it->second;

							if (MovementControllerNPC *controller = GetMovementController(actor)) {
								InterlockedIncrement(&controller->m_refCount); // incref

								static BSFixedString keepOffsetFromActorStr("Keep Offset From Actor");
								if (!controller->GetInterfaceByName_2(keepOffsetFromActorStr)) {
									if (now - data.lastAttemptTime > Config::options.keepOffsetRetryInterval) {
										// Retry
										g_taskInterface->AddTask(KeepOffsetTask::Create(GetOrCreateRefrHandle(actor), GetOrCreateRefrHandle(player)));
										data.lastAttemptTime = now;
									}
								}
								else if (!data.success) {
									// To be sure, do a single additional attempt once we know the interface exists
									g_taskInterface->AddTask(KeepOffsetTask::Create(GetOrCreateRefrHandle(actor), GetOrCreateRefrHandle(player)));
									data.success = true;
								}

								// decref
								if (InterlockedExchangeSubtract(&controller->m_refCount, (UInt32)1) == 1) {
									get_vfunc< _BSIntrusiveRefCounted_Destruct>(controller, 0x0)(controller, 1);
								}
							}
						}
					}
				}

				// When an npc is grabbed, disable collision with them
				if (actor == g_rightHeldRefr) {
					g_rightHeldCollisionGroup = collisionGroup;
				}
				if (actor == g_leftHeldRefr) {
					g_leftHeldCollisionGroup = collisionGroup;
				}
			}
			else {
				if (g_keepOffsetActors.size() > 0 && g_keepOffsetActors.count(actor)) {
					Actor_ClearKeepOffsetFromActor(actor);
					g_keepOffsetActors.erase(actor);
				}
			}

			bool isHittableCharController = g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(collisionGroup);

			bool shouldAddToWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale < Config::options.activeRagdollStartDistance;
			bool shouldRemoveFromWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale > Config::options.activeRagdollEndDistance;

			bool isAddedToWorld = IsAddedToWorld(actor);
			bool isActiveActor = (g_activeActors.size() > 0 && g_activeActors.count(actor)) || isHittableCharController;
			bool canAddToWorld = CanAddToWorld(actor);
			
			if (shouldAddToWorld) {
				if (!isAddedToWorld || !isActiveActor) {
					if (canAddToWorld) {
						AddRagdollToWorld(actor);
						if (collisionGroup != 0) {
							g_activeBipedGroups.insert(collisionGroup);
						}
					}
					else {
						// There is no ragdoll instance, but we still need a way to hit the enemy, e.g. for the wisp (witchlight).
						// In this case, we need to register collisions against their charcontroller.
						g_hittableCharControllerGroups.insert(collisionGroup);
					}
				}

				if (g_activeActors.size() > 0 && g_activeActors.count(actor)) {
					// Sometimes the game re-enables sync-on-update e.g. when switching outfits, so we need to make sure it's disabled.
					DisableSyncOnUpdate(actor);

					// Force the game to run the animation graph update (and hence driveToPose, etc.)
					actor->flags2 |= (1 << 8);

					// Set whether we want biped self-collision for this actor
					if (Config::options.doBipedSelfCollision && collisionGroup != 0) {
						if (TESRace *race = actor->race) {
							const char *name = race->editorId;
							if ((Config::options.doBipedSelfCollisionForNPCs && race->keyword.HasKeyword(g_keyword_actorTypeNPC)) ||
								(name && Config::options.additionalSelfCollisionRaces.count(std::string_view(name)))) {

								if (g_contactListener.collidedRefs.count(actor) || isHeld) {
									if (!g_selfCollidableBipedGroups.count(collisionGroup)) {
										g_selfCollidableBipedGroups.insert(collisionGroup);
										UpdateCollisionFilterOnAllBones(actor);
									}
								}
								else {
									if (g_selfCollidableBipedGroups.count(collisionGroup)) {
										g_selfCollidableBipedGroups.erase(collisionGroup);
										UpdateCollisionFilterOnAllBones(actor);
									}
								}
							}
						}
					}
				}
			}
			else if (shouldRemoveFromWorld) {
				if (isAddedToWorld) {
					if (canAddToWorld) {
						RemoveRagdollFromWorld(actor);
						g_activeBipedGroups.erase(collisionGroup);
					}
					else {
						if (isHittableCharController) {
							g_hittableCharControllerGroups.erase(collisionGroup);
						}
					}
				}
			}
		}
	}
}

void TryForceRigidBodyControls(hkbGeneratorOutput &output, hkbGeneratorOutput::TrackHeader &header)
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

void TryForcePoweredControls(hkbGeneratorOutput &output, hkbGeneratorOutput::TrackHeader &header)
{
	if (header.m_capacity > 0) {
		hkbPoweredRagdollControlData *data = (hkbPoweredRagdollControlData *)(Track_getData(output, header));
		data[0] = hkbPoweredRagdollControlData{};

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
		data[i] = 1.1f; // anything > 1
		// Indexed by (boneIdx >> 5), and then you >> (boneIdx & 0x1F) & 1 to extract the specific bit
		driver->reportingWhenKeyframed[i >> 5] |= (1 << (i & 0x1F));
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

int GetRagdollBoneIndex(hkbCharacter *character, const std::string &boneName)
{
	const hkaSkeleton *skeleton = character->ragdollDriver->ragdoll->m_skeleton;
	for (int i = 0; i < skeleton->m_bones.getSize(); i++) {
		const hkaBone &bone = skeleton->m_bones[i];
		if (boneName == bone.m_name.cString()) {
			return i;
		}
	}
	return -1;
}

void PreDriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext& context, hkbGeneratorOutput& generatorOutput)
{
	Actor *actor = GetActorFromRagdollDriver(driver);
	if (!actor) return;

	hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POSE);
	hkbGeneratorOutput::TrackHeader *worldFromModelHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_WORLD_FROM_MODEL);
	hkbGeneratorOutput::TrackHeader *keyframedBonesHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_KEYFRAMED_RAGDOLL_BONES);
	hkbGeneratorOutput::TrackHeader *rigidBodyHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_RIGID_BODY_RAGDOLL_CONTROLS);
	hkbGeneratorOutput::TrackHeader *poweredHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POWERED_RAGDOLL_CONTROLS);

	if (!g_activeRagdolls.count(driver)) return;

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	ragdoll.deltaTime = deltaTime;

	double frameTime = GetTime();
	ragdoll.frameTime = frameTime;

	KnockState knockState = GetActorKnockState(actor);
	if (Config::options.blendWhenGettingUp) {
		if (ragdoll.knockState == KnockState::BeginGetUp && knockState == KnockState::GetUp) {
			// Went from starting to get up to actually getting up
			ragdoll.blender.StartBlend(Blender::BlendType::RagdollToCurrentRagdoll, frameTime, Config::options.getUpBlendTime);
		}
	}
	ragdoll.knockState = knockState;

	/*TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);
	if (std::string(name->name) == "Faendal") {
		hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);
		PrintToFile(std::to_string(VectorLength(HkVectorToNiPoint(worldFromModel.m_translation))), "worldfrommodel");
	}*/

	if (Actor_IsInRagdollState(actor) || IsActorGettingUp(actor)) return;

	bool isRigidBodyOn = rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f;
	bool isPoweredOn = poweredHeader && poweredHeader->m_onFraction > 0.f;

	if (!isRigidBodyOn && !isPoweredOn) {
		// No controls are active - try and force it to use the rigidbody controller
		if (rigidBodyHeader) {
			TryForceRigidBodyControls(generatorOutput, *rigidBodyHeader);
			isRigidBodyOn = rigidBodyHeader->m_onFraction > 0.f;
		}
	}

	if (isRigidBodyOn && !isPoweredOn) {
		if (poweredHeader) {
			TryForcePoweredControls(generatorOutput, *poweredHeader);
			isPoweredOn = poweredHeader->m_onFraction > 0.f;
			if (isPoweredOn) {
				poweredHeader->m_onFraction = Config::options.poweredControllerOnFraction;
				rigidBodyHeader->m_onFraction = 1.1f; // something > 1 makes the hkbRagdollDriver blend between the rigidbody and powered controllers
			}
		}
	}

	ragdoll.isOn = true;
	if (!isRigidBodyOn) {
		ragdoll.isOn = false;
		ragdoll.state = RagdollState::Idle; // reset state
		return;
	}

	if (Config::options.enableKeyframes) {
		double elapsedTime = (frameTime - ragdoll.stateChangedTime) * *g_globalTimeMultiplier;
		if (elapsedTime <= Config::options.blendInKeyframeTime) {
			if (keyframedBonesHeader && keyframedBonesHeader->m_onFraction > 0.f) {
				SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);
			}
		}
	}

	if (rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f && rigidBodyHeader->m_numData > 0) {
		hkaKeyFrameHierarchyUtility::ControlData *data = (hkaKeyFrameHierarchyUtility::ControlData *)(Track_getData(generatorOutput, *rigidBodyHeader));
		for (int i = 0; i < rigidBodyHeader->m_numData; i++) {
			hkaKeyFrameHierarchyUtility::ControlData &elem = data[i];
			elem.m_hierarchyGain = Config::options.hierarchyGain;
			elem.m_velocityGain = Config::options.velocityGain;
			elem.m_positionGain = Config::options.positionGain;
		}
	}

	if (poweredHeader && poweredHeader->m_onFraction > 0.f && poweredHeader->m_numData > 0) {
		hkbPoweredRagdollControlData *data = (hkbPoweredRagdollControlData *)(Track_getData(generatorOutput, *poweredHeader));
		for (int i = 0; i < poweredHeader->m_numData; i++) {
			hkbPoweredRagdollControlData &elem = data[i];
			elem.m_maxForce = Config::options.poweredMaxForce;
			elem.m_tau = Config::options.poweredTau;
			elem.m_damping = Config::options.poweredDaming;
			elem.m_proportionalRecoveryVelocity = Config::options.poweredProportionalRecoveryVelocity;
			elem.m_constantRecoveryVelocity = Config::options.poweredConstantRecoveryVelocity;
		}
	}

	if (Config::options.copyFootIkToPoseTrack) {
		// When the game does foot ik, the output of the foot ik is put into a temporary hkbGeneratorOutput and copied into the hkbCharacter.poseLocal.
		// However, the physics ragdoll driving is done on the hkbGeneratorOutput from hkbBehaviorGraph::generate() which does not have the foot ik incorporated.
		// So, copy the pose from hkbCharacter.poseLocal into the hkbGeneratorOutput pose track to have the ragdoll driving take the foot ik into account.
		hkbCharacter *character = driver->character;
		if (character && poseHeader && poseHeader->m_onFraction > 0.f) {
			BShkbAnimationGraph *graph = GetAnimationGraph(character);
			if (graph && graph->doFootIK) {
				if (character->footIkDriver && character->setup && character->setup->m_data && character->setup->m_data->m_footIkDriverInfo) {
					hkQsTransform *poseLocal = hkbCharacter_getPoseLocal(character);
					hkInt16 numPoses = poseHeader->m_numData;
					memcpy(Track_getData(generatorOutput, *poseHeader), poseLocal, numPoses * sizeof(hkQsTransform));
				}
			}
		}
	}

	if (Config::options.loosenRagdollContraintsToMatchPose) {
		if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
			hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);
			hkQsTransform *poseLocal = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

			int numPosesLow = driver->ragdoll->getNumBones();
			static std::vector<hkQsTransform> poseWorld{};
			poseWorld.resize(numPosesLow);
			hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld(driver, poseLocal, worldFromModel, poseWorld.data());

			// Set rigidbody transforms to the anim pose ones and save the old values
			static std::vector<hkTransform> savedTransforms{};
			savedTransforms.clear();
			for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
				hkpRigidBody *rb = driver->ragdoll->m_rigidBodies[i];
				hkQsTransform &transform = poseWorld[i];

				savedTransforms.push_back(rb->getTransform());
				rb->m_motion.getMotionState()->m_transform.m_translation = NiPointToHkVector(HkVectorToNiPoint(transform.m_translation) * *g_havokWorldScale);
				hkRotation_setFromQuat(&rb->m_motion.getMotionState()->m_transform.m_rotation, transform.m_rotation);
			}

			{ // Loosen ragdoll constraints to allow the anim pose
				if (!ragdoll.easeConstraintsAction) {
					hkpEaseConstraintsAction* easeConstraintsAction = (hkpEaseConstraintsAction *)hkHeapAlloc(sizeof(hkpEaseConstraintsAction));
					hkpEaseConstraintsAction_ctor(easeConstraintsAction, (const hkArray<hkpEntity*>&)(driver->ragdoll->getRigidBodyArray()), 0);
					ragdoll.easeConstraintsAction = easeConstraintsAction; // must do this after ctor since this increments the refcount
					hkReferencedObject_removeReference(ragdoll.easeConstraintsAction);
				}

				hkpEaseConstraintsAction_loosenConstraints(ragdoll.easeConstraintsAction);
			}

			// Restore rigidbody transforms
			for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
				hkpRigidBody *rb = driver->ragdoll->m_rigidBodies[i];
				rb->m_motion.getMotionState()->m_transform = savedTransforms[i];
			}
		}
	}

	// Root motion
	if (NiPointer<NiNode> root = actor->GetNiNode()) {
		if (bhkCharacterController *controller = GetCharacterController(actor)) {
			if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
				if (NiPointer<bhkRigidBody> rb = GetFirstRigidBody(root)) {
					NiAVObject *collNode = GetNodeFromCollidable(&rb->hkBody->m_collidable);
					//std::string boneName = std::string("Ragdoll_") + collNode->m_name;
					//_MESSAGE(collNode->m_name);

					if (int boneIndex = GetAnimBoneIndex(driver->character, collNode->m_name); boneIndex >= 0) {
						const hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);

						hkQsTransform *poseData = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);
						// TODO: Technically I think we need to apply the entire hierarchy of poses here, not just worldFromModel, but this is the root collision node so there shouldn't be much of a hierarchy here
						hkQsTransform poseT;
						poseT.setMul(worldFromModel, poseData[boneIndex]);

						if (Config::options.doWarp && ragdoll.hasHipBoneTransform) {
							hkTransform actualT;
							rb->getTransform(actualT);

							NiPoint3 posePos = HkVectorToNiPoint(ragdoll.hipBoneTransform.m_translation) * *g_havokWorldScale;
							NiPoint3 actualPos = HkVectorToNiPoint(actualT.m_translation);
							NiPoint3 posDiff = actualPos - posePos;

							if (VectorLength(posDiff) > Config::options.maxAllowedDistBeforeWarp) {
								if (keyframedBonesHeader && keyframedBonesHeader->m_onFraction > 0.f) {
									SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);
								}

								hkQsTransform *poseLocal = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

								int numPosesLow = driver->ragdoll->getNumBones();
								static std::vector<hkQsTransform> poseWorld{};
								poseWorld.resize(numPosesLow);
								hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld(driver, poseLocal, worldFromModel, poseWorld.data());

								// Set rigidbody transforms to the anim pose ones
								for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
									hkpRigidBody *rb = driver->ragdoll->m_rigidBodies[i];
									hkQsTransform &transform = poseWorld[i];

									hkTransform newTransform;
									newTransform.m_translation = NiPointToHkVector(HkVectorToNiPoint(transform.m_translation) * *g_havokWorldScale);
									hkRotation_setFromQuat(&newTransform.m_rotation, transform.m_rotation);

									rb->getRigidMotion()->setTransform(newTransform);
									hkpEntity_updateMovedBodyInfo(rb);
								}
							}
						}

						ragdoll.hipBoneTransform = poseT;
						ragdoll.hasHipBoneTransform = true;
					}
				}
			}
		}
	}
}

void PostDriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext& context, hkbGeneratorOutput& generatorOutput)
{
	// This hook is called right after hkbRagdollDriver::driveToPose()

	Actor *actor = GetActorFromRagdollDriver(driver);
	if (!actor) return;

	if (!g_activeRagdolls.count(driver)) return;

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	if (!ragdoll.isOn) return;

	int numBones = driver->ragdoll->getNumBones();
	if (numBones <= 0) return;
	ragdoll.stress.clear();

	float totalStress = 0.f;
	for (int i = 0; i < numBones; i++) {
		float stress = sqrtf(g_stressOut[i].m_stressSquared);
		ragdoll.stress.push_back(stress);
		totalStress += stress;
	}

	ragdoll.avgStress = totalStress / numBones;
	//_MESSAGE("stress: %.2f", avgStress);
	//PrintToFile(std::to_string(ragdoll.avgStress), "stress.txt");
}

void PrePostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
	// This hook is called right before hkbRagdollDriver::postPhysics()

	Actor *actor = GetActorFromRagdollDriver(driver);
	if (!actor) return;

	// All we're doing here is storing the anim pose, so it's fine to run this even if the actor is fully ragdolled or getting up.

	if (!g_activeRagdolls.count(driver)) return;

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	if (!ragdoll.isOn) return;

	hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(inOut, hkbGeneratorOutput::StandardTracks::TRACK_POSE);
	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *animPose = (hkQsTransform *)Track_getData(inOut, *poseHeader);
		// Copy anim pose track before postPhysics() as postPhysics() will overwrite it with the ragdoll pose
		ragdoll.animPose.assign(animPose, animPose + numPoses);
	}
}

void PostPostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
	// This hook is called right after hkbRagdollDriver::postPhysics()

	Actor *actor = GetActorFromRagdollDriver(driver);
	if (!actor) return;

	// All we're doing here is storing the ragdoll pose and blending, and we do want to have the option to blend even while getting up.

	if (!g_activeRagdolls.count(driver)) return;

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	if (!ragdoll.isOn) return;

	RagdollState state = ragdoll.state;

	//PrintToFile(std::to_string((int)state), "state.txt");

	hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(inOut, hkbGeneratorOutput::StandardTracks::TRACK_POSE);

	if (Config::options.loosenRagdollContraintsToMatchPose) {
		if (ragdoll.easeConstraintsAction) {
			// Restore constraint limits from before we loosened them
			// TODO: Can the character die between drivetopose and postphysics? If so, we should do this if the ragdoll character dies too.
			hkpEaseConstraintsAction_restoreConstraints(ragdoll.easeConstraintsAction, 0.f);
			ragdoll.easeConstraintsAction = nullptr;
		}
	}

	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);

		// Copy pose track now since postPhysics() just set it to the high-res ragdoll pose
		ragdoll.ragdollPose.assign(poseOut, poseOut + numPoses);
	}

	Blender &blender = ragdoll.blender;
	if (blender.isActive) {
		bool done = !Config::options.doBlending;
		if (!done) {
			done = blender.Update(ragdoll, *driver, inOut, ragdoll.frameTime);
		}
		if (done) {
			if (state == RagdollState::BlendIn) {
				state = RagdollState::Idle;
			}
			else if (state == RagdollState::BlendOut) {
				state = RagdollState::Idle;
			}
		}
	}

	if (Config::options.forceAnimPose) {
		if (poseHeader && poseHeader->m_onFraction > 0.f) {
			int numPoses = poseHeader->m_numData;
			hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);
			memcpy(poseOut, ragdoll.animPose.data(), numPoses * sizeof(hkQsTransform));
		}
	}
	else if (Config::options.forceRagdollPose) {
		if (poseHeader && poseHeader->m_onFraction > 0.f) {
			int numPoses = poseHeader->m_numData;
			hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);
			memcpy(poseOut, ragdoll.ragdollPose.data(), numPoses * sizeof(hkQsTransform));
		}
	}

	ragdoll.state = state;
}

void DriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext& context, hkbGeneratorOutput& generatorOutput)
{
	PreDriveToPoseHook(driver, deltaTime, context, generatorOutput);
	hkbRagdollDriver_driveToPose(driver, deltaTime, context, generatorOutput);
	PostDriveToPoseHook(driver, deltaTime, context, generatorOutput);
}

void PostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
	PrePostPhysicsHook(driver, context, inOut);
	hkbRagdollDriver_postPhysics(driver, context, inOut);
	PostPostPhysicsHook(driver, context, inOut);
}

void PreCullActorsHook(Actor *actor)
{
	if (!g_activeActors.count(actor)) return; // let the game decide

	UInt32 cullState = 7; // do not cull this actor
	
	actor->unk274 &= 0xFFFFFFF0;
	actor->unk274 |= cullState & 0xF;
	
	// It might be okay to just set the cull state above and still cull the root node (i.e. get rid of this line)
	//if (NiPointer<NiNode> root = actor->GetNiNode()) {
	//	root->m_flags &= ~(1 << 20);  // zero out bit 20 -> do not cull the actor's root node
	//}
}

void BShkbAnimationGraph_UpdateAnimation_Hook(BShkbAnimationGraph *_this, BShkbAnimationGraph::UpdateData *updateData, void *a3)
{
	Actor *actor = _this->holder;
	if (a3 && actor && g_activeActors.count(actor)) { // a3 is null if the graph is not active
		updateData->unk2A = true; // forces animation update (hkbGenerator::generate()) without skipping frames
	}

	BShkbAnimationGraph_UpdateAnimation(_this, updateData, a3);
}

void Actor_TakePhysicsDamage_Hook(Actor *_this, HitData &hitData)
{
	// We do our own physics damage for active actors, so don't have the game do its as well
	if (g_activeActors.count(_this)) return;
	Actor_GetHit(_this, hitData);
}

void MovementControllerUpdateHook(MovementControllerNPC *movementController, Actor *actor)
{

	// Do movement jobs here
	{
		std::unique_lock lock(g_bumpActorsLock);
		if (auto it = g_bumpActors.find(actor); it != g_bumpActors.end() && !it->second.consumed && GetTime() - it->second.bumpTime < Config::options.actorBumpCooldownTime) {
			TryBumpActor(actor, false);
			it->second.consumed = true;
		}
	}

	MovementControllerNPC_Update(movementController);
}


uintptr_t processHavokHitJobsHookedFuncAddr = 0;
auto processHavokHitJobsHookLoc = RelocAddr<uintptr_t>(0x6497E4);
auto processHavokHitJobsHookedFunc = RelocAddr<uintptr_t>(0x75AC20);

auto postPhysicsHookLoc = RelocAddr<uintptr_t>(0xB268DC);

auto driveToPoseHookLoc = RelocAddr<uintptr_t>(0xB266AB);

uintptr_t controllerDriveToPoseHookedFuncAddr = 0;
auto controllerDriveToPoseHookLoc = RelocAddr<uintptr_t>(0xA26C05);

auto potentiallyEnableMeleeCollisionLoc = RelocAddr<uintptr_t>(0x6E5366);

auto preCullActorsHookLoc = RelocAddr<uintptr_t>(0x69F4B9);

auto BShkbAnimationGraph_UpdateAnimation_HookLoc = RelocAddr<uintptr_t>(0xB1CB55);

auto Actor_TakePhysicsDamage_HookLoc = RelocAddr<uintptr_t>(0x61F6E7);

auto Character_MovementControllerUpdate_HookLoc = RelocAddr<uintptr_t>(0x5E086F);


void PerformHooks(void)
{
	// First, set our addresses
	processHavokHitJobsHookedFuncAddr = processHavokHitJobsHookedFunc.GetUIntPtr();
	controllerDriveToPoseHookedFuncAddr = hkaRagdollRigidBodyController_driveToPose.GetUIntPtr();

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

	if (Config::options.forceGenerateForActiveRagdolls) {
		g_branchTrampoline.Write5Call(BShkbAnimationGraph_UpdateAnimation_HookLoc.GetUIntPtr(), uintptr_t(BShkbAnimationGraph_UpdateAnimation_Hook));
		_MESSAGE("BShkbAnimationGraph::UpdateAnimation hook complete");
	}

	{
		g_branchTrampoline.Write5Call(driveToPoseHookLoc.GetUIntPtr(), uintptr_t(DriveToPoseHook));
		_MESSAGE("hkbRagdollDriver::driveToPose hook complete");
	}

	{
		g_branchTrampoline.Write5Call(postPhysicsHookLoc.GetUIntPtr(), uintptr_t(PostPhysicsHook));
		_MESSAGE("hkbRagdollDriver::postPhysics hook complete");
	}

	if (Config::options.doClutterVsBipedCollisionDamage) {
		g_branchTrampoline.Write5Call(Actor_TakePhysicsDamage_HookLoc.GetUIntPtr(), uintptr_t(Actor_TakePhysicsDamage_Hook));
		_MESSAGE("Actor take physics damage hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				mov(rdx, rsi); // the actor being updated is in rsi

				// Call our hook
				mov(rax, (uintptr_t)MovementControllerUpdateHook);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(Character_MovementControllerUpdate_HookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(Character_MovementControllerUpdate_HookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("MovementControllerNPC::Update hook complete");
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

	if (Config::options.disableCullingForActiveRagdolls) {
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				sub(rsp, 0x20); // 0x20 bytes for scratch space

				mov(rcx, rbp); // the actor being considered for culling is in rbp at this point

				// Call our hook
				mov(rax, (uintptr_t)PreCullActorsHook);
				call(rax);

				add(rsp, 0x20);

				// Original code
				mov(rbx, ptr[rsp + 0x78]);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(preCullActorsHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(preCullActorsHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("PreCullActors hook complete");
	}

	{
		UInt64 bytes = 0x000000A6E9; // turn the conditional jump in the exe into an unconditional jump
		SafeWriteBuf(potentiallyEnableMeleeCollisionLoc.GetUIntPtr(), &bytes, 5);
		_MESSAGE("Patched the game to no longer enable its melee collision");
	}
}

bool TryHook()
{
	// This should be sized to the actual amount used by your trampoline
	static const size_t TRAMPOLINE_SIZE = 1024;

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

bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return true;
	NiPointer<NiAVObject> hmdNode = player->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
	if (!hmdNode) return true;

	if (g_openVR && *g_openVR) {
		BSOpenVR *openVR = *g_openVR;
		vr_src::IVRSystem *vrSystem = openVR->vrSystem;
		if (vrSystem) {
			const vr_src::TrackedDeviceIndex_t hmdIndex = vr_src::k_unTrackedDeviceIndex_Hmd;
			const vr_src::TrackedDeviceIndex_t rightIndex = vrSystem->GetTrackedDeviceIndexForControllerRole(vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand);
			const vr_src::TrackedDeviceIndex_t leftIndex = vrSystem->GetTrackedDeviceIndexForControllerRole(vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand);

			if (unGamePoseArrayCount > hmdIndex && vrSystem->IsTrackedDeviceConnected(hmdIndex) && hmdNode) {
				vr_src::TrackedDevicePose_t &hmdPose = pGamePoseArray[hmdIndex];
				if (hmdPose.bDeviceIsConnected && hmdPose.bPoseIsValid && hmdPose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
					vr_src::HmdMatrix34_t &hmdMatrix = hmdPose.mDeviceToAbsoluteTracking;

					NiTransform hmdTransform;
					HmdMatrixToNiTransform(hmdTransform, hmdMatrix);

					// Use the transform between the openvr hmd pose and skyrim's hmdnode transform to get the transform from openvr space to skyrim worldspace
					NiMatrix33 openvrToSkyrimWorldTransform = hmdNode->m_worldTransform.rot * hmdTransform.rot.Transpose();

					bool isRightConnected = vrSystem->IsTrackedDeviceConnected(rightIndex);
					bool isLeftConnected = vrSystem->IsTrackedDeviceConnected(leftIndex);

					for (int i = hmdIndex + 1; i < unGamePoseArrayCount; i++) {
						if (i == rightIndex && isRightConnected) {
							vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
							if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {

								// SteamVR
								// +y is up
								// +x is to the right
								// -z is forward

								// Skyrim
								// +z is up
								// +x is to the right
								// +y is forward

								// So, SteamVR -> Skyrim
								// x <- x
								// y <- -z
								// z <- y

								NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
								NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };
								NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;
								g_controllerVelocities[0].velocities.pop_back();
								g_controllerVelocities[0].velocities.push_front(velocityWorldspace);
								g_controllerVelocities[0].Recompute();
							}
						}
						else if (i == leftIndex && isLeftConnected) {
							vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
							if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
								NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
								NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };
								NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;
								g_controllerVelocities[1].velocities.pop_back();
								g_controllerVelocities[1].velocities.push_front(velocityWorldspace);
								g_controllerVelocities[1].Recompute();
							}
						}
					}
				}
			}
		}
	}

	return true;
}

void ControllerStateCB(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, uint32_t unControllerStateSize, bool& state)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return;

	constexpr vr_src::ETrackedControllerRole rightControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand;
	vr_src::TrackedDeviceIndex_t rightController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(rightControllerRole);

	constexpr vr_src::ETrackedControllerRole leftControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand;
	vr_src::TrackedDeviceIndex_t leftController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(leftControllerRole);

	if (unControllerDeviceIndex == rightController) {
		// Check if the trigger is pressed
		const uint64_t triggerMask = vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
		g_isRightTriggerHeld = pControllerState->ulButtonPressed & triggerMask;
	}
	else if (unControllerDeviceIndex == leftController) {
		const uint64_t triggerMask = vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
		g_isLeftTriggerHeld = pControllerState->ulButtonPressed & triggerMask;
	}
}

void ShowErrorBox(const char *errorString)
{
	int msgboxID = MessageBox(
		NULL,
		(LPCTSTR)errorString,
		(LPCTSTR)"ActiveRagdoll Fatal Error",
		MB_ICONERROR | MB_OK | MB_TASKMODAL
	);
}

void ShowErrorBoxAndLog(const char *errorString)
{
	_ERROR(errorString);
	ShowErrorBox(errorString);
}

void ShowErrorBoxAndTerminate(const char *errorString)
{
	ShowErrorBoxAndLog(errorString);
	*((int *)0) = 0xDEADBEEF; // crash
}

extern "C" {
	void OnDataLoaded()
	{
		// With redone hit detection, these only affect weapon swing sounds/noise and stuff like the bloodskal blade
		*g_fMeleeLinearVelocityThreshold = Config::options.meleeSwingLinearVelocityThreshold;
		*g_fShieldLinearVelocityThreshold = Config::options.shieldSwingLinearVelocityThreshold;

		g_keyword_actorTypeAnimal = papyrusKeyword::GetKeyword(nullptr, BSFixedString("ActorTypeAnimal"));
		g_keyword_actorTypeNPC = papyrusKeyword::GetKeyword(nullptr, BSFixedString("ActorTypeNPC"));

		if (!g_keyword_actorTypeAnimal || !g_keyword_actorTypeNPC) {
			_ERROR("Failed to get keywords");
			return;
		}

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

					unsigned int higgsVersion = g_higgsInterface->GetBuildNumber();
					if (higgsVersion < 1040601) {
						ShowErrorBoxAndTerminate("[CRITICAL] HIGGS is present but is a lower version than is required by this mod. Get the latest version of HIGGS and try again.");
					}

					g_higgsInterface->AddCollisionFilterComparisonCallback(CollisionFilterComparisonCallback);
					g_higgsInterface->AddPrePhysicsStepCallback(PrePhysicsStepCallback);

					UInt64 bitfield = g_higgsInterface->GetHiggsLayerBitfield();
					bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped); // add collision with ragdoll of live characters
					bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC); // add collision with ragdoll of live characters using furniture
					g_higgsInterface->SetHiggsLayerBitfield(bitfield);

					g_higgsInterface->ForceWeaponCollisionEnabled(false);
					g_higgsInterface->ForceWeaponCollisionEnabled(true);
				}
				else {
					ShowErrorBoxAndTerminate("[CRITICAL] Did NOT get higgs interface. HIGGS is required for this mod.");
				}
			}
		}
	}

	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\activeragdoll.log");
		gLog.SetPrintLevel(IDebugLog::kLevel_DebugMessage);
		gLog.SetLogLevel(IDebugLog::kLevel_DebugMessage);

		_MESSAGE("activeragdoll v%s", ACTIVERAGDOLL_VERSION_VERSTRING);

		info->infoVersion = PluginInfo::kInfoVersion;
		info->name = "activeragdoll";
		info->version = ACTIVERAGDOLL_VERSION_MAJOR;

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
		_MESSAGE("activeragdoll loaded");

		if (Config::ReadConfigOptions()) {
			_MESSAGE("Successfully read config parameters");
		}
		else {
			_WARNING("[WARNING] Failed to read config options. Using defaults instead.");
		}

		_MESSAGE("Registering for SKSE messages");
		g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
		g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

		g_taskInterface = (SKSETaskInterface *)skse->QueryInterface(kInterface_Task);
		if (!g_taskInterface) {
			ShowErrorBoxAndLog("[CRITICAL] Could not get SKSE task interface");
			return false;
		}

		g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
		if (!g_trampoline) {
			_WARNING("Couldn't get trampoline interface");
		}
		if (!TryHook()) {
			ShowErrorBoxAndLog("[CRITICAL] Failed to perform hooks");
			return false;
		}

		g_vrInterface = (SKSEVRInterface *)skse->QueryInterface(kInterface_VR);
		if (!g_vrInterface) {
			ShowErrorBoxAndLog("[CRITICAL] Couldn't get SKSE VR interface. You probably have an outdated SKSE version.");
			return false;
		}
		g_vrInterface->RegisterForPoses(g_pluginHandle, 11, WaitPosesCB);
		g_vrInterface->RegisterForControllerState(g_pluginHandle, 11, ControllerStateCB);

		g_timer.Start();

		return true;
	}
};
