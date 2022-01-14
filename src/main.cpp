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


void SwingWeapon(TESObjectWEAP *weapon, bool isLeft, bool setAttackState = true, bool playSound = true)
{
	PlayerCharacter *player = *g_thePlayer;

	if (setAttackState) {
		player->actorState.flags08 &= 0xFFFFFFFu; // zero out meleeAttackState
		player->actorState.flags08 |= 0x20000000u; // meleeAttackState = kSwing
	}

	if (playSound) {
		if (weapon) {
			if (weapon->type() < TESObjectWEAP::GameData::kType_Bow) { // not bow, staff, or crossbow
				BGSSoundDescriptorForm *sound = weapon->attackFailSound;
				if (sound) {
					NiPointer<NiAVObject> handNode = isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftHandBone] : player->unk3F0[PlayerCharacter::Node::kNode_RightHandBone];
					if (handNode) {
						PlaySoundAtNode(sound, handNode, {});
					}
				}
			}
		}
	}

	UInt64 *vtbl = *((UInt64 **)player);
	((_Actor_WeaponSwingCallback)(vtbl[0xF1]))(player);

	if (player->processManager) {
		ActorProcess_IncrementAttackCounter(player->processManager, 1);
	}

	int soundAmount = weapon ? TESObjectWEAP_GetSoundAmount(weapon) : TESNPC_GetSoundAmount((TESNPC *)player->baseForm);
	Actor_SetActionValue(player, soundAmount);

	if (player->unk158) {
		CombatController_sub_14050DEC0((void *)player->unk158);
	}
}

void HitActor(Character *target, bool isLeft, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity)
{
	bool isOffhand = *g_leftHandedMode ? !isLeft : isLeft;

	PlayerCharacter *player = *g_thePlayer;

	TESObjectWEAP *mainWeapon = GetEquippedWeapon(player, false);

	TESForm *offhandObj = player->GetEquippedObject(true);
	TESObjectWEAP *offhandWeapon = nullptr;
	if (offhandObj) {
		offhandWeapon = DYNAMIC_CAST(offhandObj, TESForm, TESObjectWEAP);
	}

	TESObjectWEAP *weapon = isOffhand ? offhandWeapon : mainWeapon;

	bool canHit = !Actor_IsGhost(target) && Character_CanHit(*g_thePlayer, target);
	if (!canHit) {
		// TODO: Actually test this code path

		// potentially play impact effect/sound
		// - get bgsblockbashdata and material of thing hit

		// if (isMotionTypeMoveable(bodyB))
		//   hitRefr->SetActorCause(player->GetActorCause())

		SwingWeapon(weapon, isLeft, true);
		player->actorState.flags08 &= 0xFFFFFFFu; // zero out attackState since SwingWeapon sets it
		return;
	}

	// if isOffhand and no right hand weapon and offhand weapon is bow, set weapon to offhand weapon (bow)

	// Handle bow/crossbow/torch/shield bash (set attackstate to kBash)
	TESObjectARMO *equippedShield = (offhandObj && offhandObj->formType == kFormType_Armor) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectARMO) : nullptr;
	bool isShield = isOffhand && equippedShield;

	TESObjectLIGH *equippedLight = (offhandObj && offhandObj->formType == kFormType_Light) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectLIGH) : nullptr;
	bool isTorch = isOffhand && equippedLight;

	bool isBowOrCrossbow = weapon && (weapon->type() == TESObjectWEAP::GameData::kType_Bow || weapon->type() == TESObjectWEAP::GameData::kType_CrossBow);

	bool isBash = isBowOrCrossbow || isShield || isTorch;
	if (isBash) {
		player->actorState.flags08 &= 0xFFFFFFFu; // zero out meleeAttackState
		player->actorState.flags08 |= 0x60000000u; // attackState = kBash
	}

	BGSAttackData *attackData = nullptr;
	PlayerCharacter_UpdateAndGetAttackData(player, *g_isUsingMotionControllers, isLeft, false, &attackData);

	SwingWeapon(weapon, isLeft, false);

	int dialogueSubtype = isBash ? 28 : 26; // 26 is attack, 27 powerattack, 28 bash
	UpdateDialogue(nullptr, player, target, 3, dialogueSubtype, false, nullptr);

	// Hit position / velocity need to be set before Character::HitTarget() which at some point will read from them (during the HitData population)
	NiPoint3 *playerLastHitPosition = (NiPoint3 *)((UInt64)player + 0x6BC);
	*playerLastHitPosition = hitPosition;

	NiPoint3 *playerLastHitVelocity = (NiPoint3 *)((UInt64)player + 0x6C8);
	*playerLastHitVelocity = hitVelocity;

	// TODO: We get bow/crossbow shooting sounds / fx (and damage, perhaps?) when bashing with a bow/crossbow for some reason, even though in the base game's hit detection I think we get the bash sound.
	Character_HitTarget(player, target, nullptr, isOffhand);

	Actor_RemoveMagicEffectsDueToAction(player, -1); // removes invis/ethereal due to attacking

	// rumble(isRight, vrMeleeData.impactConfirmRumbleIntensity, veMeleeData.impactConfirmRumbleDuration) // sub_140C59440()

	player->actorState.flags08 &= 0xFFFFFFFu; // zero out attackState

	// if (isMotionTypeMoveable(bodyB)) apply impulse
}

struct PointImpulseJob
{
	hkpRigidBody *rigidBody{};
	NiPoint3 point{};
	NiPoint3 impulse{};
	UInt32 refrHandle{};

	void Run()
	{
		// Need to be safe since the job could run next frame where the rigidbody might not exist anymore
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(refrHandle, refr)) {
			NiPointer<NiNode> root = refr->GetNiNode();
			if (root && FindRigidBody(root, rigidBody)) {
				hkpEntity_activate(rigidBody);
				rigidBody->m_motion.applyPointImpulse(NiPointToHkVector(impulse), NiPointToHkVector(point));
				_MESSAGE("Applied point impulse %.2f", VectorLength(impulse));
			}
		}
	}
};

struct LinearImpulseJob
{
	hkpRigidBody *rigidBody{};
	NiPoint3 impulse{};
	UInt32 refrHandle{};

	void Run()
	{
		// Need to be safe since the job could run next frame where the rigidbody might not exist anymore
		NiPointer<TESObjectREFR> refr;
		if (LookupREFRByHandle(refrHandle, refr)) {
			NiPointer<NiNode> root = refr->GetNiNode();
			if (root && FindRigidBody(root, rigidBody)) {
				hkpEntity_activate(rigidBody);
				rigidBody->m_motion.applyLinearImpulse(NiPointToHkVector(impulse));
				_MESSAGE("Applied linear impulse %.2f", VectorLength(impulse));
			}
		}
	}
};

std::vector<PointImpulseJob> g_pointImpulsejobs{};
std::vector<LinearImpulseJob> g_linearImpulsejobs{};

struct ContactListener : hkpContactListener, hkpWorldPostSimulationListener
{
	struct Event
	{
		enum class Type
		{
			TOI,
			ADDED,
			REMOVED
		};

		hkpRigidBody *rbA = nullptr;
		hkpRigidBody *rbB = nullptr;
		Type type;
	};

	std::map<std::pair<hkpRigidBody *, hkpRigidBody*>, int> activeCollisions{};
	std::unordered_map<UInt32, double> hitReactionTargets{};
	std::unordered_map<Actor *, double> hitCooldownTargets[2]{}; // each hand has its own cooldown
	std::unordered_set<hkbRagdollDriver *> activeDrivers;
	std::vector<Event> events{};

	std::pair<hkpRigidBody *, hkpRigidBody *> SortPair(hkpRigidBody *a, hkpRigidBody *b) {
		if ((uint64_t)a <= (uint64_t)b) return { a, b };
		else return { b, a };
	}

	void DoHit(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, const hkpContactPointEvent &evnt)
	{
		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;

		hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;
		hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;

		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (!hitRefr) return;

		Character *target = DYNAMIC_CAST(hitRefr, TESObjectREFR, Character);
		if (!target) return;

		UInt8 ragdollBits = (hittingRigidBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 8) & 0x1f;
		bool isLeft = ragdollBits == 5; // stupid. Right hand would be 3.

		float havokWorldScale = *g_havokWorldScale;

		hkVector4 hkHitPos = evnt.m_contactPoint->getPosition();
		NiPoint3 hitPosition = HkVectorToNiPoint(hkHitPos) / havokWorldScale;

		//NiPoint3 hitVelocity = HkVectorToNiPoint(evnt.m_contactPoint->getSeparatingNormal()) * hkpContactPointEvent_getSeparatingVelocity(evnt) * *g_inverseHavokWorldScale;
		//if (!isHitRigidBodyA) hitVelocity *= -1;
		hkVector4 pointVelocity; hittingRigidBody->getPointVelocity(hkHitPos, pointVelocity);
		NiPoint3 hitVelocity = HkVectorToNiPoint(pointVelocity) / havokWorldScale;

		HitActor(target, isLeft, hitPosition, hitVelocity);

		// TODO: We _should_ incorporate the mass somewhat, so that heavier objects get moved less but will still move somewhat. As is, this is fine for human ragdolls but dwarven spheres basically do not react to hits at all.
		// TODO: Different impulse for different weapon types? (e.g. dagger is not much, but two-hander is much)
		NiPoint3 impulse = hitVelocity * havokWorldScale * Config::options.hitImpulseMult;

		UInt32 targetHandle = GetOrCreateRefrHandle(target);

		// Apply linear impulse at the center of mass to all bodies within 2 ragdoll constraints
		ForEachRagdollDriver(target, [hitRigidBody, &impulse, targetHandle](hkbRagdollDriver *driver) {
			ForEachAdjacentBody(driver, hitRigidBody, [driver, &impulse, targetHandle](hkpRigidBody *adjacentBody) {
				g_linearImpulsejobs.push_back({ adjacentBody, impulse * Config::options.hitImpulseDecayMult1, targetHandle });

				ForEachAdjacentBody(driver, adjacentBody, [&impulse, targetHandle](hkpRigidBody *semiAdjacentBody) {
					g_linearImpulsejobs.push_back({ semiAdjacentBody, impulse * Config::options.hitImpulseDecayMult2, targetHandle });
				});
			});
		});

		// Apply a point impulse at the hit location to the body we actually hit
		g_pointImpulsejobs.push_back({ hitRigidBody, HkVectorToNiPoint(hkHitPos), impulse, targetHandle });

		double now = GetTime();
		hitReactionTargets[targetHandle] = now;
		hitCooldownTargets[isLeft][target] = now;
	}

	virtual void contactPointCallback(const hkpContactPointEvent& evnt) {
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

		if (layerA == 56 && layerB == 56) return; // Both objects are on the higgs layer

		hkpRigidBody *hitRigidBody = layerA == 56 ? rigidBodyB : rigidBodyA;
		NiPointer<TESObjectREFR> hitRefr = GetRefFromCollidable(&hitRigidBody->m_collidable);
		if (hitRefr) {
			Character *target = DYNAMIC_CAST(hitRefr, TESObjectREFR, Character);
			if (target) {
				hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;
				UInt8 ragdollBits = (hittingRigidBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 8) & 0x1f;
				bool isLeft = ragdollBits == 5; // stupid. Right hand would be 3.

				if (hitCooldownTargets[isLeft].count(target)) {
					// Actor is currently under a hit cooldown, so disable the contact point and gtfo
					evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
					return;
				}
			}
		}

		// TODO: Only do hit detection for actors probably (or at least, do something other than just disabling the contact point for non-actors)

		// A contact point of any sort confirms a collision
		if (evnt.isToi()) {
			hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;
			hkVector4 pointVelocity; hittingRigidBody->getPointVelocity(evnt.m_contactPoint->getPosition(), pointVelocity);
			float hitSpeed = VectorLength(HkVectorToNiPoint(pointVelocity));
			//float separatingSpeed = hkpContactPointEvent_getSeparatingVelocity(evnt); // along collision normal
			// TODO: We could make the required speed high when the hand is not moving in roomspace and lower it the faster the hand is moving in roomspace. I think this would lead to fewer accidental hits?
			if (fabs(hitSpeed) > Config::options.hitSeparatingSpeedThreshold) {
				// For TOIs, they can happen before the collisionAddedCallback so we need to disable them now no matter if they're in activeCollisions or not
				evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
				events.push_back({ rigidBodyA, rigidBodyB, Event::Type::TOI });
				_MESSAGE("%d TOI contact pt fast %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);

				// Do damage or something
				DoHit(rigidBodyA, rigidBodyB, evnt);

				return;
			}
			_MESSAGE("%d TOI contact pt slow %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
		}
		else {
			// Not toi, so it will be an active collision if it's relevant as these occur on the frame after collisionAddedCallback (apart from EXPAND_MANIFOLD which is dependent on the TOI event)
			auto pair = SortPair(rigidBodyA, rigidBodyB);
			if (activeCollisions.count(pair)) {
				_MESSAGE("%d Contact pt %d %x %x", *g_currentFrameCounter, (int)evnt.m_type, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
				hkpRigidBody *hittingRigidBody = hitRigidBody == rigidBodyA ? rigidBodyB : rigidBodyA;
				hkVector4 pointVelocity; hittingRigidBody->getPointVelocity(evnt.m_contactPoint->getPosition(), pointVelocity);
				float hitSpeed = VectorLength(HkVectorToNiPoint(pointVelocity));
				//float separatingSpeed = hkpContactPointEvent_getSeparatingVelocity(evnt); // along collision normal
				if (fabs(hitSpeed) > Config::options.hitSeparatingSpeedThreshold) {
					evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
					// Do damage or something
					_MESSAGE("%d fast %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
					DoHit(rigidBodyA, rigidBodyB, evnt);
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
		if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

		if (layerA == 56 && layerB == 56) return; // Both objects are on the higgs layer

		events.push_back({ rigidBodyA, rigidBodyB, Event::Type::ADDED });
		_MESSAGE("%d Added %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
	}

	virtual void collisionRemovedCallback(const hkpCollisionEvent& evnt)
	{
		hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
		hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

		// Technically our objects could have changed layers or something between added and removed
		events.push_back({ rigidBodyA, rigidBodyB, Event::Type::REMOVED });
		_MESSAGE("%d Removed %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
	}

	virtual void postSimulationCallback(hkpWorld* world)
	{
		// First just accumulate adds/removes. Why? While ADDED always occurs before REMOVED for a single contact point,
		// a single pair of rigid bodies can have multiple contact points, and adds/removes between these different contact points can be non-deterministic.
		for (Event &evnt : events) {
			if (evnt.type == Event::Type::ADDED) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				int count = activeCollisions.count(pair) ? activeCollisions[pair] : 0;
				activeCollisions[pair] = count + 1;
			}
			else if (evnt.type == Event::Type::REMOVED) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				int count = activeCollisions.count(pair) ? activeCollisions[pair] : 0;
				activeCollisions[pair] = count - 1;
			}
		}

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

		// Process hit targets (not hit cooldown targets) and condider the same as if they were collided with.
		// Fill in active drivers with any hit targets that haven't expired yet.
		for (auto it = hitReactionTargets.begin(); it != hitReactionTargets.end();) {
			auto[handle, hitTime] = *it;
			if (now - hitTime >= Config::options.hitReactionTime)
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
				if (DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor)) {
					Actor *hitActor = DYNAMIC_CAST(hitRefr, TESObjectREFR, Actor);
					if (hitActor) {
						ForEachRagdollDriver(hitActor, [this](hkbRagdollDriver *driver) {
							activeDrivers.insert(driver);
						});

						UInt8 ragdollBits = (hittingRigidBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 8) & 0x1f;
						bool isLeft = ragdollBits == 5; // stupid. Right hand would be 3.

						if (hitCooldownTargets[isLeft].count(hitActor)) {
							// Actor is still collided with, so refresh its hit cooldown
							hitCooldownTargets[isLeft][hitActor] = now;
						}
					}
				}
			}
		}

		// Clear out old hit cooldown targets
		for (auto &targets : hitCooldownTargets) { // For each hand's cooldown targets
			for (auto it = targets.begin(); it != targets.end();) {
				auto[target, hitTime] = *it;
				if (now - hitTime >= Config::options.hitRecoveryTime)
					it = targets.erase(it);
				else
					++it;
			}
		}

		// Now process TOIs
		// TODO: I don't think I need this anymore, nor the adding of the contact points to this event list during the contact point event.
		for (Event &evnt : events) {
			if (evnt.type == Event::Type::TOI) {
				auto pair = SortPair(evnt.rbA, evnt.rbB);
				if (activeCollisions.count(pair)) {
					// At the end of this frame's sim, we are collided with the rigidbody

					_MESSAGE("%d TOI entered", *g_currentFrameCounter);
				}
				else {
					// TOI entered and left (no collision added)

					_MESSAGE("%d TOI entered and left", *g_currentFrameCounter);
				}
			}
		}

		events.clear();
	}

	NiPointer<bhkWorld> world = nullptr;
};
ContactListener *contactListener = nullptr;

std::unordered_map<hkbRagdollDriver *, ActiveRagdoll> g_activeRagdolls;
std::unordered_set<hkbRagdollDriver *> g_higgsDrivers;

hkaKeyFrameHierarchyUtility::Output g_stressOut[200]; // set in a hook during driveToPose(). Just reserve a bunch of space so it can handle any number of bones.

hkArray<hkVector4> g_scratchHkArray{}; // We can't call the destructor of this ourselves, so this is a global array to be used at will and never deallocated.


bool IsAddedToWorld(Actor *actor)
{
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
	if (!GetAnimationGraphManager(actor, animGraphManager)) return true;

	BSAnimationGraphManager *manager = animGraphManager.ptr;

	SimpleLocker lock(&manager->updateLock);

	for (int i = 0; i < manager->graphs.size; i++) {
		BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
		if (!graph.ptr->world) return false;

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
	ForEachRagdollDriver(actor, [](hkbRagdollDriver *driver) {
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
					bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
					//bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_KEYFRAMED, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
					//rigidBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
					//bhkWorldObject_UpdateCollisionFilter(wrapper);
				}
				//}
			}
		}

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

		if (Config::options.malleableConstraintStrength < 1.f) {
			// Reduce strength of all constraints by wrapping them in malleable constraints with lower strength
			for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
				NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable);
				if (node) {
					NiPointer<bhkRigidBody> wrapper = GetRigidBody(node);
					if (wrapper) {
						for (int i = 0; i < wrapper->constraints.count; i++) {
							bhkConstraint *constraint = wrapper->constraints.entries[i];
							//bhkConstraint *fixedConstraint = ConstraintToFixedConstraint(constraint, 0.3f, false);
							bhkConstraint *malleableConstraint = CreateMalleableConstraint(constraint, Config::options.malleableConstraintStrength);
							if (malleableConstraint) {
								constraint->RemoveFromCurrentWorld();

								bhkWorld *world = wrapper->GetHavokWorld_1()->m_userData;
								malleableConstraint->MoveToWorld(world);
								wrapper->constraints.entries[i] = malleableConstraint;
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
	else {
		// TODO: If there is no ragdoll instance, then we still need a way to hit the enemy, e.g. for wisp (witchlight). In this case, I guess we need to register collisions against their charcontroller body or something.
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
				hkpWorld_removeWorldPostSimulationListener(world->world, contactListener);
			}
		}

		_MESSAGE("Adding listener to new havok world");
		{
			BSWriteLocker lock(&world->worldLock);

			hkpCollisionCallbackUtil_requireCollisionCallbackUtil(world->world);
			hkpWorld_addContactListener(world->world, contactListener);
			hkpWorld_addWorldPostSimulationListener(world->world, contactListener);

			bhkCollisionFilter *filter = (bhkCollisionFilter *)world->world->m_collisionFilter;

			if (Config::options.enableBipedBipedCollision) {
				filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= (1 << BGSCollisionLayer::kCollisionLayer_Biped); // enable biped->biped collision;
			}

			if (Config::options.enablePlayerBipedCollision) {
				// Add a new layer for the player that will not collide with charcontrollers but will collide with the biped layer instead
				UInt64 bitfield = filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_CharController]; // copy of L_CHARCONTROLLER layer bitfield

				bitfield |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Biped); // add collision with l_biped (ragdoll of live characters)
				bitfield &= ~((UInt64)1 << BGSCollisionLayer::kCollisionLayer_CharController); // remove collision with character controllers

				// Layer 56 is taken (higgs) so use 57
				filter->layerBitfields[57] = bitfield;
				filter->layerNames[57] = BSFixedString("L_PLAYERCAPSULE");
				// Set whether other layers should collide with our new layer
				ReSyncLayerBitfields(filter, bitfield);

				NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer);
				if (controller) {
					hkpListShape *listShape = ((hkpListShape*)controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_shape);

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
					*((void **)newShape) = ((void *)(hkCharControllerShape_vtbl));

					bhkShape *wrapper = (bhkShape*)convexVerticesShape->m_userData;
					wrapper->SetHavokObject(newShape);

					// The listshape does not use a hkRefPtr but it's still setup to add a reference upon construction and remove one on destruction
					listShape->m_childInfo[0].m_shape = newShape;
					hkReferencedObject_removeReference(convexVerticesShape); // this will usually call the dtor on the old shape

					// We don't need to remove a ref here, the ctor gave it a refcount of 1 and we assigned it to the listShape which isn't technically a hkRefPtr but still owns it (and the listShape's dtor will decref anyways)
					// hkReferencedObject_removeReference(newShape);

					/*
					// This technically works, but causes bugs such as falling through the floor
					float radius = 0.1f;
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

					hkpListShape_disableChild(listShape, 0);
					hkpListShape_enableChild(listShape, 1);
					*/

					controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f; // zero out collision layer
					controller->proxy.characterProxy->m_shapePhantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= 57; // set layer to the layer we just created
					bhkWorld_UpdateCollisionFilterOnWorldObject(world, (bhkWorldObject *)controller->proxy.characterProxy->m_shapePhantom->m_userData);
				}
			}
		}

		contactListener->world = world;
	}

	if (g_higgsInterface) {
		NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer);
		if (controller) {
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

	bool isCollidedWith = contactListener->activeDrivers.count(driver) || g_higgsDrivers.count(driver);

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];

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

	if ((state == RagdollState::BlendIn || state == RagdollState::Dynamic || state == RagdollState::BlendOut)) {
		if (rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f && rigidBodyHeader->m_numData > 0) {
			float hierarchyGain, velocityGain, positionGain;
			if (state == RagdollState::BlendOut) {
				hierarchyGain = Config::options.blendOutHierarchyGain;
				velocityGain = Config::options.blendOutVelocityGain;
				positionGain = Config::options.blendOutPositionGain;
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

	if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
		const hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);

		int numPosesHigh = poseHeader->m_numData;
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
						//_MESSAGE(collNode->m_name);

						int boneIndex = GetAnimBoneIndex(driver->character, collNode->m_name);
						if (boneIndex >= 0) {
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
}

void PostDriveToPoseHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
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

void PostPostPhysicsHook(hkbRagdollDriver *driver, hkbGeneratorOutput &inOut)
{
	// This hook is called right after hkbRagdollDriver::postPhysics()

	Actor *actor = GetActorFromRagdollDriver(driver);
	if (!actor || Actor_IsInRagdollState(actor)) return;

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	if (!ragdoll.isOn) return;

	RagdollState state = ragdoll.state;

	//PrintToFile(std::to_string((int)state), "state.txt");

	hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(inOut, hkbGeneratorOutput::StandardTracks::TRACK_POSE);

	if (ragdoll.easeConstraintsAction) {
		// Restore constraint limits from before we loosened them
		// TODO: Can the character die between drivetopose and postphysics? If so, we should do this if the ragdoll character dies too.
		hkpEaseConstraintsAction_restoreConstraints(ragdoll.easeConstraintsAction, 0.f);
		ragdoll.easeConstraintsAction = nullptr;
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

	for (LinearImpulseJob &job : g_linearImpulsejobs) {
		job.Run();
	}
	g_linearImpulsejobs.clear();

	for (PointImpulseJob &job : g_pointImpulsejobs) {
		job.Run();
	}
	g_pointImpulsejobs.clear();
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

auto prePhysicsStepHookLoc = RelocAddr<uintptr_t>(0xDFB709);

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
				lea(rdx, ptr[rsp + 0x60]); // hkbContext
				mov(r8, r12); // hkbGeneratorOutput

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

extern "C" {
	void OnDataLoaded()
	{
		contactListener = new ContactListener;

		*g_fMeleeLinearVelocityThreshold = 99999.f;
		*g_fShieldLinearVelocityThreshold = 99999.f;

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
