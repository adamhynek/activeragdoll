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
			activeCollisions.insert(SortPair(rigidBodyA, rigidBodyB)); // TODO: Can there be multiple collisions between the same entity pair?

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

std::unordered_map<hkbRagdollDriver *, ActiveRagdoll> g_activeRagdolls;


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
						ActiveRagdoll data = ActiveRagdoll();
						g_activeRagdolls[driver] = data;
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
						g_activeRagdolls.erase(driver);
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

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];

	double frameTime = GetTime();
	ragdoll.frameTime = frameTime;

	bool isRigidBodyOn = rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f;
	bool isPoweredOn = poweredHeader && poweredHeader->m_onFraction > 0.f;

	if (!isRigidBodyOn && !isPoweredOn) {
		// No controls are active - try and force it to use the rigidbody controller
		if (rigidBodyHeader) {
			TryForceControls(generatorOutput, *rigidBodyHeader);
			isRigidBodyOn = rigidBodyHeader->m_onFraction > 0.f;
		}
	}

	ragdoll.isOn = true;
	if (!isRigidBodyOn) {
		ragdoll.isOn = false;
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
	if (state == RagdollState::Collide) {
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

	if (keyframedBonesHeader && keyframedBonesHeader->m_onFraction > 0.f) {
		if (state == RagdollState::Keyframed) { // Don't keyframe bones if we've collided with the actor
			//SetBonesKeyframedReporting(driver, generatorOutput, *keyframedBonesHeader);
		}
		else {
			// Explicitly make bones not keyframed
			//keyframedBonesHeader->m_onFraction = 0.f;
		}
	}

	if ((state == RagdollState::BlendIn || state == RagdollState::Collide || state == RagdollState::BlendOut) && rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f) {
		if (rigidBodyHeader->m_numData > 0) {
			float elapsedTime = (frameTime - ragdoll.stateChangedTime) * *g_globalTimeMultiplier;

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
	}

	if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
		const hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);

		int numPosesHigh = poseHeader->m_numData;
		hkQsTransform *poseLocal = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

		int numPosesLow = driver->ragdoll->getNumBones();
		std::vector<hkQsTransform> poseWorld(numPosesLow);
		hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld(driver, poseLocal, worldFromModel, poseWorld.data());

		// Set rigidbody transforms to the anim pose ones and save the old values
		std::vector<hkTransform> savedTransforms{};
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
						//_MESSAGE(collNode->m_name);

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

void PostDriveToPoseHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
	// This hook is called right after hkbRagdollDriver::driveToPose()

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
	PrintToFile(std::to_string(ragdoll.avgStress), "stress.txt");
}

void PrePostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
	// This hook is called right before hkbRagdollDriver::postPhysics()

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	if (!ragdoll.isOn) return;

	hkInt32 numTracks = inOut.m_tracks->m_masterHeader.m_numTracks;

	int poseTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POSE;
	hkbGeneratorOutput::TrackHeader *poseHeader = numTracks > poseTrackId ? &(inOut.m_tracks->m_trackHeaders[poseTrackId]) : nullptr;
	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *animPose = (hkQsTransform *)Track_getData(inOut, *poseHeader);
		// Copy anim pose track before postPhysics() as postPhysics() will overwrite it with the ragdoll pose
		ragdoll.animPose.assign(animPose, animPose + numPoses);
	}
}

std::vector<float> weights{};
std::vector<float> scratchStresses{};
std::vector<hkQsTransform> g_scratchPose{};
void PostPostPhysicsHook(hkbRagdollDriver *driver, hkbGeneratorOutput &inOut)
{
	// This hook is called right after hkbRagdollDriver::postPhysics()

	ActiveRagdoll &ragdoll = g_activeRagdolls[driver];
	if (!ragdoll.isOn) return;

	RagdollState state = ragdoll.state;

	PrintToFile(std::to_string((int)state), "state.txt");

	hkInt32 numTracks = inOut.m_tracks->m_masterHeader.m_numTracks;
	int poseTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POSE;
	hkbGeneratorOutput::TrackHeader *poseHeader = numTracks > poseTrackId ? &(inOut.m_tracks->m_trackHeaders[poseTrackId]) : nullptr;

	{ // Restore constraint limits from before we loosened them
		// TODO: Can the character die between drivetopose and postphysics? If so, we should do this if the ragdoll character dies too.
		hkpEaseConstraintsAction_restoreConstraints(ragdoll.easeConstraintsAction, 0.f);
		ragdoll.easeConstraintsAction = nullptr;
	}

	if (state == RagdollState::BlendIn && ragdoll.frameTime == ragdoll.stateChangedTime) {
		if (poseHeader && poseHeader->m_onFraction > 0.f) {
			int numPoses = poseHeader->m_numData;
			hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);
			hkaSkeleton *animSkeleton = driver->character->setup->m_animationSkeleton; // TODO: null checks
			ragdoll.restStress.clear();
			for (int i = 0; i < numPoses; i++) {
				hkQsTransform &animPose = ragdoll.animPose[i];
				hkQsTransform &restPose = poseOut[i];
				float stress = QuaternionAngle(HkQuatToNiQuat(animPose.m_rotation), HkQuatToNiQuat(restPose.m_rotation));
				ragdoll.restStress.push_back(stress);
			}
		}
	}

	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);
		hkaSkeleton *animSkeleton = driver->character->setup->m_animationSkeleton; // TODO: null checks
		if (ragdoll.state == RagdollState::Keyframed) {
			memcpy(poseOut, ragdoll.animPose.data(), numPoses * sizeof(hkQsTransform));
		}
		else {
			weights.clear();
			scratchStresses.clear();
			for (int i = 0; i < numPoses; i++) {
				//float stress = ragdoll.stress[i];
				hkQsTransform &animPose = ragdoll.animPose[i];
				hkQsTransform &pose = poseOut[i];

				//float stress = VectorLength(HkVectorToNiPoint(animPose.m_translation) - HkVectorToNiPoint(pose.m_translation));
				float stress = abs(QuaternionAngle(HkQuatToNiQuat(animPose.m_rotation), HkQuatToNiQuat(pose.m_rotation)) - ragdoll.restStress[i]);
				scratchStresses.push_back(stress);
			}
			for (int i = 0; i < numPoses; i++) {
				float stress = 0.f;
				int parent = i;
				// propagate stress from parent nodes
				while (parent != -1) {
					stress = max(stress, scratchStresses[parent]);
					parent = animSkeleton->m_parentIndices[parent];
				}

				float weight = stress * Config::options.blendStressWeight;
				weight = std::clamp(weight, 0.f, 1.f); // 0 means full anim pose, 1 means full ragdoll/blended pose
				weights.push_back(weight);
			}

			g_scratchPose.assign(poseOut, poseOut + numPoses);
			BlendPoses(ragdoll.animPose.data(), g_scratchPose.data(), poseOut, weights.data(), numPoses);
		}
	}

	Blender &blender = ragdoll.blender;
	if (blender.isActive) {
		bool done = blender.Update(ragdoll, inOut, ragdoll.frameTime);
		if (done) {
			if (state == RagdollState::BlendIn)
				state = RagdollState::Collide;
			else if (state == RagdollState::BlendOut)
				state = RagdollState::Keyframed;
		}
	}

	ragdoll.state = state;
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
