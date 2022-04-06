#pragma once

#include <unordered_set>
#include <set>

#include "RE/havok.h"
#include "RE/havok_behavior.h"

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "common/ITimer.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
//extern double g_deltaTime;

inline void set_vtbl(void *object, void *vtbl) { *((void **)object) = ((void *)(vtbl)); }
inline UInt64 * get_vtbl(void *object) { return *((UInt64 **)object); }

inline void set_vfunc(void *object, UInt64 index, std::uintptr_t vfunc)
{
	UInt64 *vtbl = get_vtbl(object);
	vtbl[index] = vfunc;
}

template<class T>
inline T get_vfunc(void *object, UInt64 index)
{
	UInt64 *vtbl = get_vtbl(object);
	return (T)(vtbl[index]);
}

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

inline void ltrim(std::string &s) { s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !::isspace(ch); })); }
inline void rtrim(std::string &s) { s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !::isspace(ch); }).base(), s.end()); }
inline void trim(std::string &s) { ltrim(s); rtrim(s); }
std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim);

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
MovementControllerNPC * GetMovementController(Actor *actor);
ActorCause * TESObjectREFR_GetActorCause(TESObjectREFR *refr);
void TESObjectREFR_SetActorCause(TESObjectREFR *refr, ActorCause* cause);
KnockState GetActorKnockState(Actor *actor);
bool IsActorGettingUp(Actor *actor);
bool IsActorUsingFurniture(Actor *actor);
