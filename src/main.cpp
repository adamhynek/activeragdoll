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

void HitActor(Character *source, Character *target, TESForm *weapon, bool isOffhand)
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

	int dialogueSubtype = isBash ? 28 : 26; // 26 is attack, 27 powerattack, 28 bash
	UpdateDialogue(nullptr, source, target, 3, dialogueSubtype, false, nullptr);

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

struct PrePhysicsStepJob
{
	virtual void Run() = 0;
};

struct PointImpulseJob : PrePhysicsStepJob
{
	hkpRigidBody *rigidBody{};
	NiPoint3 point{};
	NiPoint3 impulse{};
	UInt32 refrHandle{};

	PointImpulseJob(hkpRigidBody *rigidBody, NiPoint3 &point, NiPoint3 &impulse, UInt32 refrHandle) : rigidBody(rigidBody), point(point), impulse(impulse), refrHandle(refrHandle) {}

	virtual void Run() override
	{
		// Need to be safe since the job could run next frame where the rigidbody might not exist anymore
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(refrHandle, refr)) {
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

struct LinearImpulseJob : PrePhysicsStepJob
{
	hkpRigidBody *rigidBody{};
	NiPoint3 impulse{};
	UInt32 refrHandle{};

	LinearImpulseJob(hkpRigidBody *rigidBody, NiPoint3 &impulse, UInt32 refrHandle) : rigidBody(rigidBody), impulse(impulse), refrHandle(refrHandle) {}

	virtual void Run() override
	{
		// Need to be safe since the job could run next frame where the rigidbody might not exist anymore
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(refrHandle, refr)) {
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

std::vector<std::unique_ptr<PrePhysicsStepJob>> g_prePhysicsStepJobs{};

template<class T, typename... Args>
void QueuePrePhysicsJob(Args&&... args)
{
	static_assert(std::is_base_of<PrePhysicsStepJob, T>::value);
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

std::unordered_set<Actor *> g_charControllerActors{};

struct ContactListener : hkpContactListener, hkpWorldPostSimulationListener
{
	struct CollisionEvent
	{
		enum class Type
		{
			ADDED,
			REMOVED
		};

		hkpRigidBody *rbA = nullptr;
		hkpRigidBody *rbB = nullptr;
		Type type;
	};

	std::map<std::pair<hkpRigidBody *, hkpRigidBody*>, int> activeCollisions{};
	std::unordered_map<UInt32, double> hitReactionTargets{};
	std::unordered_map<TESObjectREFR *, double> hitCooldownTargets[2]{}; // each hand has its own cooldown
	std::unordered_set<hkbRagdollDriver *> activeDrivers;
	std::vector<CollisionEvent> events{};

	inline std::pair<hkpRigidBody *, hkpRigidBody *> SortPair(hkpRigidBody *a, hkpRigidBody *b) {
		if ((uint64_t)a <= (uint64_t)b) return { a, b };
		else return { b, a };
	}

	void DoHit(TESObjectREFR *hitRefr, hkpRigidBody *hitRigidBody, hkpRigidBody *hittingRigidBody, const hkpContactPointEvent &evnt, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, TESForm *weapon, float impulseMult, bool isLeft, bool isOffhand)
	{
		PlayerCharacter *player = *g_thePlayer;
		if (hitRefr == player) return;

		// Set attack data
		BGSAttackData *attackData = nullptr;
		PlayerCharacter_UpdateAndGetAttackData(player, *g_isUsingMotionControllers, isOffhand, false, &attackData);
		if (!attackData) return;

		// TODO: Consider using hand velocity for hit velocity instead of the actual point velocity at the hit
		// Hit position / velocity need to be set before Character::HitTarget() which at some point will read from them (during the HitData population)
		NiPoint3 *playerLastHitPosition = (NiPoint3 *)((UInt64)player + 0x6BC);
		*playerLastHitPosition = hitPosition;

		NiPoint3 *playerLastHitVelocity = (NiPoint3 *)((UInt64)player + 0x6C8);
		*playerLastHitVelocity = hitVelocity;

		UInt32 targetHandle = GetOrCreateRefrHandle(hitRefr);
		double now = GetTime();

		// TODO: Different impulse for different weapon types? (e.g. dagger is not much, but two-hander is much)
		float massInv = hitRigidBody->getMassInv();
		float mass = massInv <= 0.001f ? 99999.f : 1.f / massInv;

		float impulseStrength = std::clamp(
			Config::options.hitImpulseBaseStrength + Config::options.hitImpulseProportionalStrength * powf(mass, Config::options.hitImpulseMassExponent),
			Config::options.hitImpulseMinStrength, Config::options.hitImpulseMaxStrength
		);

		float havokWorldScale = *g_havokWorldScale;
		NiPoint3 impulse = hitVelocity * havokWorldScale * mass; // This impulse will give the object the exact velocity it is hit with
		impulse *= impulseStrength; // Scale the velocity as we see fit
		impulse *= impulseMult;
		if (impulse.z < 0) {
			// Impulse points downwards somewhat, scale back the downward component so we don't get things shooting into the ground.
			impulse.z *= Config::options.hitImpulseDownwardsMultiplier;
		}

		Character *hitChar = DYNAMIC_CAST(hitRefr, TESObjectREFR, Character);
		if (hitChar && !Actor_IsGhost(hitChar) && Character_CanHit(player, hitChar)) {
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;

			HitActor(player, hitChar, weapon, isOffhand);

			VRMeleeData *meleeData = GetVRMeleeData(isLeft);
			PlayRumble(!isLeft, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);

			if (Config::options.applyImpulseOnHit) {
				// Apply linear impulse at the center of mass to all bodies within 2 ragdoll constraints
				ForEachRagdollDriver(hitChar, [hitRigidBody, &impulse, targetHandle](hkbRagdollDriver *driver) {
					ForEachAdjacentBody(driver, hitRigidBody, [driver, &impulse, targetHandle](hkpRigidBody *adjacentBody) {
						QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, impulse * Config::options.hitImpulseDecayMult1, targetHandle);
						ForEachAdjacentBody(driver, adjacentBody, [&impulse, targetHandle](hkpRigidBody *semiAdjacentBody) {
							QueuePrePhysicsJob<LinearImpulseJob>(semiAdjacentBody, impulse * Config::options.hitImpulseDecayMult2, targetHandle);
						});
					});
				});

				// Apply a point impulse at the hit location to the body we actually hit
				QueuePrePhysicsJob<PointImpulseJob>(hitRigidBody, hitPosition * havokWorldScale, impulse, targetHandle);

				hitReactionTargets[targetHandle] = now;
			}
		}
		else {
			bool didDispatchHitEvent = HitRefr(player, hitRefr, weapon, hitRigidBody, isLeft, isOffhand);
			if (didDispatchHitEvent) {
				VRMeleeData *meleeData = GetVRMeleeData(isLeft);
				PlayRumble(!isLeft, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
			}

			if (!IsMoveableEntity(hitRigidBody)) {
				// Disable contact for keyframed/fixed objects in this case
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
		}

		hitCooldownTargets[isLeft][hitRefr] = now;
	}

	inline bool IsHittableCharController(TESObjectREFR *refr)
	{
		if (refr->formType == kFormType_Character) {
			if (Actor *hitActor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
				if (g_charControllerActors.size() > 0 && g_charControllerActors.count(hitActor)) {
					return true;
				}
			}
		}
		return false;
	}

	virtual void contactPointCallback(const hkpContactPointEvent& evnt) {
		if (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED) return;

		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

		if (layerA == 56 && layerB == 56) {
			// Both objects are on the higgs layer
			if (!IsMoveableEntity(rigidBodyA) && !IsMoveableEntity(rigidBodyB)) {
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
			return;
		}

		hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;
		hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;

		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (!hitRefr) {
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			return;
		}

		UInt32 hitLayer = hitRigidBody == rigidBodyA ? layerA : layerB;
		UInt8 ragdollBits = (hittingRigidBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 8) & 0x1f;
		bool isLeft = ragdollBits == 5 || ragdollBits == 6; // stupid. Right hand would be 3.

		if (hitCooldownTargets[isLeft].count(hitRefr)) {
			// refr is currently under a hit cooldown, so disable the contact point and gtfo
			evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			return;
		}
		else if (hitLayer == BGSCollisionLayer::kCollisionLayer_CharController) {
			if (!IsHittableCharController(hitRefr)) {
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
				return;
			}
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
		if (g_higgsInterface->IsTwoHanding()) {
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
				_MESSAGE("Stab amount: %.2f", stabAmount);
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
				_MESSAGE("Punch amount: %.2f", punchAmount);
				if (punchAmount > Config::options.hitPunchDirectionThreshold && hitSpeed > Config::options.hitPunchSpeedThreshold) {
					isPunch = true;
				}
			}
		}
		
		if (!isStab && !isPunch && hitSpeed > Config::options.hitSwingSpeedThreshold) {
			isSwing = true;
		}

		// Thresholding on some (small) roomspace hand velocity helps prevent hits while moving around / turning

		bool doHit = (isSwing || isStab || isPunch) && handSpeedRoomspace > Config::options.hitRequiredHandSpeedRoomspace;
		bool disableHit = !player->actorState.IsWeaponDrawn() && Config::options.disableHitIfSheathed;

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
			DoHit(hitRefr, hitRigidBody, hittingRigidBody, evnt, hitPosition, hitVelocity, equippedObj, impulseMult, isLeft, isOffhand);
		}
		else {
			if (!IsMoveableEntity(hitRigidBody)) {
				// It's not a hit, so disable contact for keyframed/fixed objects in this case
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
			}
		}
	}

	virtual void collisionAddedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

		if (layerA == 56 && layerB == 56) return; // Both objects are on the higgs layer

		events.push_back({ rigidBodyA, rigidBodyB, CollisionEvent::Type::ADDED });
		//_MESSAGE("%d Added %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
	}

	virtual void collisionRemovedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		// Technically our objects could have changed layers or something between added and removed
		events.push_back({ rigidBodyA, rigidBodyB, CollisionEvent::Type::REMOVED });
		//_MESSAGE("%d Removed %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
	}

	virtual void postSimulationCallback(hkpWorld* world)
	{
		// First just accumulate adds/removes. Why? While ADDED always occurs before REMOVED for a single contact point,
		// a single pair of rigid bodies can have multiple contact points, and adds/removes between these different contact points can be non-deterministic.
		for (CollisionEvent &evnt : events) {
			if (evnt.type == CollisionEvent::Type::ADDED) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				int count = activeCollisions.count(pair) ? activeCollisions[pair] : 0;
				activeCollisions[pair] = count + 1;
			}
			else if (evnt.type == CollisionEvent::Type::REMOVED) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				int count = activeCollisions.count(pair) ? activeCollisions[pair] : 0;
				activeCollisions[pair] = count - 1;
			}
		}
		events.clear();

		// Clear out any collisions that are no longer active (or that were only removed, since we do events for any removes but only some adds)
		for (auto it = activeCollisions.begin(); it != activeCollisions.end();) {
			auto[pair, count] = *it;
			if (count <= 0)
				it = activeCollisions.erase(it);
			else
				++it;
		}

		// Clear first, then add any that are hit. Not thread-safe obviously.
		activeDrivers.clear();

		double now = GetTime();

		// Process hit targets (not hit cooldown targets) and condider them the same as if they were collided with.
		// Fill in active drivers with any hit targets that haven't expired yet.
		for (auto it = hitReactionTargets.begin(); it != hitReactionTargets.end();) {
			auto[handle, hitTime] = *it;
			if ((now - hitTime) * *g_globalTimeMultiplier >= Config::options.hitReactionTime)
				it = hitReactionTargets.erase(it);
			else {
				UInt32 handleCopy = handle;
				NiPointer<TESObjectREFR> refr;
				if (LookupREFRByHandle(handleCopy, refr)) {
					Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
					if (actor) {
						ForEachRagdollDriver(actor, [this](hkbRagdollDriver *driver) {
							activeDrivers.insert(driver);
						});
					}
				}
				++it;
			}
		}

		// Now fill in the currently collided-with actors based on active collisions
		for (auto[pair, count] : activeCollisions) {
			auto[rigidBodyA, rigidBodyB] = pair;

			UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
			UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;

			hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;
			hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;

			NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
			if (hitRefr) {
				Actor *hitActor = DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor);
				if (hitActor) {
					ForEachRagdollDriver(hitActor, [this](hkbRagdollDriver *driver) {
						activeDrivers.insert(driver);
					});
				}

				UInt8 ragdollBits = (hittingRigidBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 8) & 0x1f;
				bool isLeft = ragdollBits == 5 || ragdollBits == 6; // stupid. Right hand would be 3.

				if (hitCooldownTargets[isLeft].count(hitRefr)) {
					// refr is still collided with, so refresh its hit cooldown
					hitCooldownTargets[isLeft][hitRefr] = now;
				}
			}
		}

		// Clear out old hit cooldown targets
		for (auto &targets : hitCooldownTargets) { // For each hand's cooldown targets
			for (auto it = targets.begin(); it != targets.end();) {
				auto[target, hitTime] = *it;
				if ((now - hitTime) * *g_globalTimeMultiplier >= Config::options.hitRecoveryTime)
					it = targets.erase(it);
				else
					++it;
			}
		}
	}

	NiPointer<bhkWorld> world = nullptr;
};
ContactListener g_contactListener{};

std::unordered_map<hkbRagdollDriver *, ActiveRagdoll> g_activeRagdolls{};
std::unordered_set<hkbRagdollDriver *> g_higgsDrivers{};

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
		hkaRagdollInstance *ragdoll = hkbRagdollDriver_getRagdollInterface(driver);
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
	bool isSittingOrSleepingOrMounted = false;// actor->actorState.flags04 & 0x3C000; // also true when e.g. using furniture (leaning against a wall, etc.)
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
					BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
					hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
					if (driver) {
						g_activeRagdolls[driver] = ActiveRagdoll{};

						if (!graph.ptr->world) {
							// World must be set before calling BShkbAnimationGraph::AddRagdollToWorld(), and is required for the graph to register its physics step listener (and hence call hkbRagdollDriver::driveToPose())
							graph.ptr->world = GetHavokWorldFromCell(actor->parentCell);
							g_activeRagdolls[driver].shouldNullOutWorldWhenRemovingFromWorld = true;
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
	bool isSittingOrSleepingOrMounted = false;// actor->actorState.flags04 & 0x3C000;
	if (isSittingOrSleepingOrMounted || Actor_IsInRagdollState(actor)) return false;

	bool hasRagdollInterface = false;
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
	if (GetAnimationGraphManager(actor, animGraphManager)) {
		BSAnimationGraphManager_HasRagdollInterface(animGraphManager.ptr, &hasRagdollInterface);
	}

	if (hasRagdollInterface) {
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
					}
				}
			}

			//x = false;
			//BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints(animGraphManager.ptr, &x);
		}
	}

	return true;
}

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

	if (!g_higgsInterface->GetGrabbedObject(false) && !g_higgsInterface->GetGrabbedObject(true)) {
		g_higgsDrivers.clear();
	}

	if (world != g_contactListener.world) {
		bhkWorld *oldWorld = g_contactListener.world;
		if (oldWorld) {
			_MESSAGE("Removing listener from old havok world");
			{
				BSWriteLocker lock(&oldWorld->worldLock);
				hkpWorldExtension *collisionCallbackExtension = hkpWorld_findWorldExtension(world->world, hkpKnownWorldExtensionIds::HK_WORLD_EXTENSION_COLLISION_CALLBACK);
				if (collisionCallbackExtension) {
					// There are times when the collision callback extension is gone even if we required it earlier...
					hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(world->world);
				}
				hkpWorld_removeContactListener(oldWorld->world, &g_contactListener);
				hkpWorld_removeWorldPostSimulationListener(world->world, &g_contactListener);
			}

			g_activeRagdolls.clear();
			g_charControllerActors.clear();
			g_higgsDrivers.clear();
			g_contactListener = ContactListener{};
		}

		_MESSAGE("Havok world changed");
		{
			BSWriteLocker lock(&world->worldLock);

			hkpCollisionCallbackUtil_requireCollisionCallbackUtil(world->world);

			hkpWorld_addContactListener(world->world, &g_contactListener);
			hkpWorld_addWorldPostSimulationListener(world->world, &g_contactListener);

			bhkCollisionFilter *filter = (bhkCollisionFilter *)world->world->m_collisionFilter;

			if (Config::options.enableBipedBipedCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped); // enable biped->biped collision;
			}

			if (Config::options.disableBipedGroundCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] &= ~((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Ground); // disable biped->ground collision;
				ReSyncLayerBitfields(filter, filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped]);
			}

			if (Config::options.enablePlayerBipedCollision) {
				// TODO: Instead of doing this, we may need to keep the player on the charcontroller layer and disable contact points from the contact listener,
				//       since the game does check specific things for specifically the charcontroller layer, and if the player charcontroller is not on that layer that may cause issues.

				// Add a new layer for the player that will not collide with charcontrollers but will collide with the biped layer instead
				UInt64 bitfield = filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_CharController]; // copy of L_CHARCONTROLLER layer bitfield

				bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped); // add collision with ragdoll of live characters
				bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC); // add collision with ragdoll of live characters with no charcontroller (likely they are in an animation)
				bitfield &= ~((UInt64)1 << BGSCollisionLayer::kCollisionLayer_CharController); // remove collision with character controllers

				// Layer 56 is taken (higgs) so use 57
				filter->layerBitfields[57] = bitfield;
				filter->layerNames[57] = BSFixedString("L_PLAYERCAPSULE");
				// Set whether other layers should collide with our new layer
				ReSyncLayerBitfields(filter, bitfield);

				if (NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer)) {
					hkpListShape *listShape = ((hkpListShape*)controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_shape);

					{ // Shrink convex charcontroller shape
						g_scratchHkArray.clear();
						hkArray<hkVector4> &verts = g_scratchHkArray;

						hkpConvexVerticesShape *convexVerticesShape = ((hkpConvexVerticesShape *)listShape->m_childInfo[0].m_shape);
						hkpConvexVerticesShape_getOriginalVertices(convexVerticesShape, verts);

						// Shrink the two "rings" of the charcontroller shape by moving the rings' vertices inwards
						// verts 0,2,6,10,12,14,15,17 are bottom ring, 8-9 are bottom/top points, 1,3,4,5,7,11,13,16 are top ring
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

						// set vtbl
						static auto hkCharControllerShape_vtbl = RelocAddr<void *>(0x1838E78);
						set_vtbl(newShape, hkCharControllerShape_vtbl);

						bhkShape *wrapper = (bhkShape*)convexVerticesShape->m_userData;
						wrapper->SetHavokObject(newShape);

						// The listshape does not use a hkRefPtr but it's still setup to add a reference upon construction and remove one on destruction
						listShape->m_childInfo[0].m_shape = newShape;
						hkReferencedObject_removeReference(convexVerticesShape); // this will usually call the dtor on the old shape

						// We don't need to remove a ref here, the ctor gave it a refcount of 1 and we assigned it to the listShape which isn't technically a hkRefPtr but still owns it (and the listShape's dtor will decref anyways)
						// hkReferencedObject_removeReference(newShape);
					}

					{ // Shrink capsule shape too. It's active when weapons are unsheathed.
						float radius = Config::options.playerCapsuleRadius;
						hkpCapsuleShape *capsule = ((hkpCapsuleShape *)listShape->m_childInfo[1].m_shape);
						float originalRadius = capsule->m_radius;
						capsule->m_radius = radius;

						NiPoint3 vert0 = HkVectorToNiPoint(capsule->getVertex(0));
						NiPoint3 vert1 = HkVectorToNiPoint(capsule->getVertex(1));

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

					controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f; // zero out collision layer
					controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= 57; // set layer to the layer we just created
					bhkWorld_UpdateCollisionFilterOnWorldObject(world, (bhkWorldObject *)controller->proxy.characterProxy->m_shapePhantom->m_userData);
				}
			}
		}

		g_contactListener.world = world;
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

	if (NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer)) {
		hkUint32 &filterInfo = controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
		UInt32 layer = filterInfo & 0x7f;

		bhkCollisionFilter *filter = (bhkCollisionFilter *)world->world->m_collisionFilter;
		UInt64 bitfield = filter->layerBitfields[57];

		bool updateFilter = false;

		if (g_higgsDrivers.size() > 0) {
			// When something is grabbed, disable collision with bipeds (since we're holding one)
			if (bitfield & ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped)) {
				bitfield &= ~((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped);
				updateFilter = true;
			}
			if (bitfield & ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC)) {
				bitfield &= ~((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC);
				updateFilter = true;
			}
		}
		else if (Config::options.enablePlayerBipedCollision) {
			// Nothing grabbed, make sure we're colliding with bipeds again
			if (!(bitfield & ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped))) {
				bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped);
				updateFilter = true;
			}
			if (!(bitfield & ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC))) {
				bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_BipedNoCC);
				updateFilter = true;
			}
		}

		if (updateFilter) {
			{
				BSWriteLocker lock(&world->worldLock);
				filter->layerBitfields[57] = bitfield;
				ReSyncLayerBitfields(filter, bitfield);
			}
			bhkWorld_UpdateCollisionFilterOnWorldObject(world, (bhkWorldObject *)controller->proxy.characterProxy->m_shapePhantom->m_userData);
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

			bool shouldAddToWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale < Config::options.activeRagdollStartDistance;
			bool shouldRemoveFromWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale > Config::options.activeRagdollEndDistance;

			bool isAddedToWorld = IsAddedToWorld(actor) || g_charControllerActors.count(actor);
			bool canAddToWorld = CanAddToWorld(actor);
			
			if (shouldAddToWorld) {
				if (!isAddedToWorld) {
					if (canAddToWorld) {
						AddRagdollToWorld(actor);
					}
					else {
						// There is no ragdoll instance, but we still need a way to hit the enemy, e.g. for the wisp (witchlight).
						// In this case, we need to register collisions against their charcontroller.
						g_charControllerActors.insert(actor);
					}
				}
			}
			else if (shouldRemoveFromWorld) {
				if (isAddedToWorld) {
					if (canAddToWorld) {
						RemoveRagdollFromWorld(actor);
					}
					else {
						if (g_charControllerActors.count(actor)) {
							g_charControllerActors.erase(actor);
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

void PreDriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext& context, hkbGeneratorOutput& generatorOutput)
{
	Actor *actor = GetActorFromRagdollDriver(driver);
	if (!actor || Actor_IsInRagdollState(actor)) return;

	hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POSE);
	hkbGeneratorOutput::TrackHeader *worldFromModelHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_WORLD_FROM_MODEL);
	hkbGeneratorOutput::TrackHeader *keyframedBonesHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_KEYFRAMED_RAGDOLL_BONES);
	hkbGeneratorOutput::TrackHeader *rigidBodyHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_RIGID_BODY_RAGDOLL_CONTROLS);
	hkbGeneratorOutput::TrackHeader *poweredHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POWERED_RAGDOLL_CONTROLS);

	bool isCollidedWith = g_contactListener.activeDrivers.count(driver) || g_higgsDrivers.count(driver);

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	ragdoll.deltaTime = deltaTime;

	double frameTime = GetTime();
	ragdoll.frameTime = frameTime;

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
		ragdoll.wantKeyframe = true;
		ragdoll.state = RagdollState::Keyframed; // reset state
		return;
	}

	RagdollState state = ragdoll.state;
	if (state == RagdollState::Keyframed) {
		if (isCollidedWith) {
			ragdoll.stateChangedTime = frameTime;
			ragdoll.blender.StartBlend(Blender::BlendType::AnimToRagdoll, frameTime, Config::options.blendInTime);
			state = RagdollState::BlendIn;
		}
	}
	if (state == RagdollState::Dynamic) {
		if (!isCollidedWith) {
			double stressLerp = max(0.0, (double)ragdoll.avgStress - Config::options.blendOutDurationStressMin) / (Config::options.blendOutDurationStressMax - Config::options.blendOutDurationStressMin);
			double blendDuration = lerp(Config::options.blendOutDurationMin, Config::options.blendOutDurationMax, stressLerp);
			ragdoll.blender.StartBlend(Blender::BlendType::CurrentRagdollToAnim, frameTime, Blender::PowerCurve(blendDuration, Config::options.blendOutBlendPower));
			ragdoll.stateChangedTime = frameTime;
			state = RagdollState::BlendOut;
		}
	}
	if (state == RagdollState::BlendOut) {
		if (isCollidedWith) {
			ragdoll.blender.StartBlend(Blender::BlendType::AnimToRagdoll, frameTime, Config::options.blendInTime);
			state = RagdollState::BlendIn;
		}
	}
	ragdoll.state = state;

	if (Config::options.keyframeBones) {
		if (keyframedBonesHeader && keyframedBonesHeader->m_onFraction > 0.f) {
			if (state == RagdollState::Keyframed && ragdoll.wantKeyframe) { // Don't keyframe bones if we've collided with the actor
				ragdoll.wantKeyframe = false;
				SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);
			}
			else {
				// Explicitly make bones not keyframed
				keyframedBonesHeader->m_onFraction = 0.f;
			}
		}
	}

	//SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);

	if ((state == RagdollState::BlendIn || state == RagdollState::Dynamic || state == RagdollState::BlendOut)) {
		if (rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f && rigidBodyHeader->m_numData > 0) {
			hkaKeyFrameHierarchyUtility::ControlData *data = (hkaKeyFrameHierarchyUtility::ControlData *)(Track_getData(generatorOutput, *rigidBodyHeader));
			for (int i = 0; i < rigidBodyHeader->m_numData; i++) {
				hkaKeyFrameHierarchyUtility::ControlData &elem = data[i];
				if (state == RagdollState::BlendOut) {
					elem.m_hierarchyGain = Config::options.blendOutHierarchyGain;
					elem.m_velocityGain = Config::options.blendOutVelocityGain;
					elem.m_positionGain = Config::options.blendOutPositionGain;
				}
				else {
					elem.m_hierarchyGain = Config::options.hierarchyGain;
					elem.m_velocityGain = Config::options.velocityGain;
					elem.m_positionGain = Config::options.positionGain;
				}
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
			/*
			static hkQsTransform prevWorldFromModel;
			PrintToFile(std::to_string(VectorLength(HkVectorToNiPoint(worldFromModel.m_translation) - HkVectorToNiPoint(prevWorldFromModel.m_translation))), "worldfrommodel.txt");
			prevWorldFromModel = worldFromModel;

			static NiTransform prevWorldFromModelNode;
			if (NiPointer<NiAVObject> root = actor->GetNiNode()) {
				//worldFromModel.m_translation = NiPointToHkVector(root->m_worldTransform.pos);
				//worldFromModel.m_rotation = NiQuatToHkQuat(MatrixToQuaternion(root->m_worldTransform.rot));

				PrintToFile(std::to_string(VectorLength(root->m_worldTransform.pos - prevWorldFromModelNode.pos)), "worldfrommodelnode.txt");
				prevWorldFromModelNode = root->m_worldTransform;

				PrintToFile(std::to_string(VectorLength(root->m_worldTransform.pos - HkVectorToNiPoint(worldFromModel.m_translation))), "worldfrommodelcomp.txt");
			}*/

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
				rb->m_motion.getMotionState()->m_transform.m_translation = transform.m_translation;
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

	if (NiPointer<NiNode> root = actor->GetNiNode()) {
		if (bhkCharRigidBodyController *controller = GetCharRigidBodyController(actor)) {
			if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
				if (NiPointer<bhkRigidBody> rb = GetFirstRigidBody(root)) {
					NiAVObject *collNode = GetNodeFromCollidable(&rb->hkBody->m_collidable);
					//std::string boneName = std::string("Ragdoll_") + collNode->m_name;
					//_MESSAGE(collNode->m_name);

					if (int boneIndex = GetAnimBoneIndex(driver->character, collNode->m_name); boneIndex >= 0) {
						const hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);

						hkQsTransform *poseData = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);
						// TODO: Technically I think we need to apply the entire hierarchy of poses here, not just worldFromModel, but this _is_ the 'root' collision node...
						hkQsTransform poseT;
						poseT.setMul(worldFromModel, poseData[boneIndex]);
							
						if (Config::options.doRootMotion && state == RagdollState::Dynamic && ragdoll.hasHipBoneTransform) {
							hkTransform actualT;
							rb->getTransform(actualT);

							NiPoint3 posePos = HkVectorToNiPoint(ragdoll.hipBoneTransform.m_translation) * *g_havokWorldScale;
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
	if (!actor || Actor_IsInRagdollState(actor)) return;

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
	if (!actor || Actor_IsInRagdollState(actor)) return;

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
	if (!actor || Actor_IsInRagdollState(actor)) return;

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

		if (ragdoll.state == RagdollState::Keyframed) {
			// When in keyframed state, force the output pose to be the anim pose
			memcpy(poseOut, ragdoll.animPose.data(), numPoses * sizeof(hkQsTransform));
		}
	}

	Blender &blender = ragdoll.blender;
	if (blender.isActive) {
		bool done = !Config::options.doBlending;
		if (!done) {
			done = blender.Update(ragdoll, inOut, ragdoll.frameTime);
		}
		if (done) {
			if (state == RagdollState::BlendIn) {
				state = RagdollState::Dynamic;
			}
			else if (state == RagdollState::BlendOut) {
				ragdoll.wantKeyframe = true;
				state = RagdollState::Keyframed;
			}
		}
	}

	if (Config::options.forceRagdollPose) {
		if (poseHeader && poseHeader->m_onFraction > 0.f) {
			int numPoses = poseHeader->m_numData;
			hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);
			memcpy(poseOut, ragdoll.ragdollPose.data(), numPoses * sizeof(hkQsTransform));
		}
	}

	ragdoll.state = state;
}

void PrePhysicsStepHook()
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
	if (!IsAddedToWorld(actor)) return; // let the game decide

	UInt32 cullState = 7; // do not cull this actor
	
	actor->unk274 &= 0xFFFFFFF0;
	actor->unk274 |= cullState & 0xF;
	
	if (NiPointer<NiNode> root = actor->GetNiNode()) {
		// TODO: it might be okay to just set the cull state above and still cull the root node (i.e. get rid of this line)
		root->m_flags &= ~(1 << 20);  // zero out bit 20 -> do not cull the actor's root node
	}
}

void BShkbAnimationGraph_UpdateAnimation_Hook(BShkbAnimationGraph *_this, BShkbAnimationGraph::UpdateData *updateData, void *a3)
{
	Actor *actor = _this->holder;
	if (a3 && actor && IsAddedToWorld(actor)) { // a3 is null if the graph is not active
		updateData->unk2A = true; // forces animation update (hkbGenerator::generate()) without skipping frames
	}

	BShkbAnimationGraph_UpdateAnimation(_this, updateData, a3);
}


uintptr_t processHavokHitJobsHookedFuncAddr = 0;
auto processHavokHitJobsHookLoc = RelocAddr<uintptr_t>(0x6497E4);
auto processHavokHitJobsHookedFunc = RelocAddr<uintptr_t>(0x75AC20);

auto postPhysicsHookLoc = RelocAddr<uintptr_t>(0xB268DC);

auto driveToPoseHookLoc = RelocAddr<uintptr_t>(0xB266AB);

uintptr_t controllerDriveToPoseHookedFuncAddr = 0;
auto controllerDriveToPoseHookLoc = RelocAddr<uintptr_t>(0xA26C05);

auto potentiallyEnableMeleeCollisionLoc = RelocAddr<uintptr_t>(0x6E5366);

auto prePhysicsStepHookLoc = RelocAddr<uintptr_t>(0xDFB709);

auto preCullActorsHookLoc = RelocAddr<uintptr_t>(0x69F4B9);

auto BShkbAnimationGraph_UpdateAnimation_HookLoc = RelocAddr<uintptr_t>(0xB1CB55);


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

	{
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

				// Call our hook
				mov(rax, (uintptr_t)PrePhysicsStepHook);
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
				mov(rcx, r13);
				test(r14b, r14b);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(prePhysicsStepHookLoc.GetUIntPtr() + 6);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write6Branch(prePhysicsStepHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Pre-physics-step hook complete");
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

void HiggsGrab(bool isLeft, TESObjectREFR *grabbedRefr)
{
	Actor *actor = DYNAMIC_CAST(grabbedRefr, TESObjectREFR, Actor);
	if (actor) {
		ForEachRagdollDriver(actor, [](hkbRagdollDriver *driver) {
			g_higgsDrivers.insert(driver);
		});
	}
}

bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
	if (!initComplete) return true;

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

extern "C" {
	void OnDataLoaded()
	{
		// With redone hit detection, these only affect weapon swing sounds/noise and stuff like the bloodskal blade
		*g_fMeleeLinearVelocityThreshold = Config::options.meleeSwingLinearVelocityThreshold;
		*g_fShieldLinearVelocityThreshold = Config::options.shieldSwingLinearVelocityThreshold;

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

		g_vrInterface = (SKSEVRInterface *)skse->QueryInterface(kInterface_VR);
		if (!g_vrInterface) {
			_ERROR("[CRITICAL] Couldn't get SKSE VR interface. You probably have an outdated SKSE version.");
			return false;
		}
		g_vrInterface->RegisterForPoses(g_pluginHandle, 11, WaitPosesCB);

		g_timer.Start();

		return true;
	}
};
