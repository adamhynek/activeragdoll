#pragma once

#include <unordered_set>

#include "RE/havok.h"
#include "RE/havok_behavior.h"

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "common/ITimer.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
//extern double g_deltaTime;

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx);
void UpdateKeyframedNodeTransform(NiAVObject *node, const NiTransform &transform);

NiAVObject * GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

bool IsAllowedCollidable(const hkpCollidable *collidable);
bool HasGeometryChildren(NiAVObject *obj);

bool IsTwoHanded(const TESObjectWEAP *weap);
bool IsBow(const TESObjectWEAP *weap);
TESObjectWEAP * GetEquippedWeapon(Actor *actor, bool isOffhand);

double GetTime();

void PrintVector(const NiPoint3 &p);
void PrintSceneGraph(NiAVObject *node);
void PrintToFile(std::string entry, std::string filename);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

inline VRMeleeData * GetVRMeleeData(bool isLeft) { return (VRMeleeData *)((UInt64)*g_thePlayer + 0x710 + (isLeft ? sizeof(VRMeleeData) : 0)); };

inline bool CanWeaponStab(TESObjectWEAP *weapon)
{
	UInt8 type = weapon->type();
	return (
		type == TESObjectWEAP::GameData::kType_OneHandSword ||
		type == TESObjectWEAP::GameData::kType_OneHandDagger ||
		type == TESObjectWEAP::GameData::kType_TwoHandSword
	);
};

bhkCollisionObject * GetCollisionObject(NiAVObject *obj);
NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj);
NiPointer<bhkRigidBody> GetFirstRigidBody(NiAVObject *root);
bool FindRigidBody(NiAVObject *root, hkpRigidBody *query);
void ForEachRagdollDriver(Actor *actor, std::function<void(hkbRagdollDriver *)> f);
void ForEachAdjacentBody(hkbRagdollDriver *driver, hkpRigidBody *body, std::function<void(hkpRigidBody *)> f);
bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node);
bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target);
bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node);
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target);
void GetAllSkinnedNodes(NiAVObject *root, std::unordered_set<NiAVObject *> &skinnedNodes);
UInt32 PlaySoundAtNode(BGSSoundDescriptorForm *sound, NiAVObject *node, const NiPoint3 &location);
void PlayPhysicsSound(hkpCollidable *collidable, const NiPoint3 &location, bool loud);
ActorCause * TESObjectREFR_GetActorCause(TESObjectREFR *refr);
void TESObjectREFR_SetActorCause(TESObjectREFR *refr, ActorCause* cause);
