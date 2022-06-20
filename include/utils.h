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

inline void set_vfunc(void *object, UInt64 index, std::uintptr_t vfunc) {
	UInt64 *vtbl = get_vtbl(object);
	vtbl[index] = vfunc;
}

template<typename T>
inline T get_vfunc(void *object, UInt64 index) {
	UInt64 *vtbl = get_vtbl(object);
	return (T)(vtbl[index]);
}

#define CALL_VFUNC(T, object, index, ...) get_vfunc<T>(object, index)(object, __VA_ARGS__)

template<class T>
inline EventDispatcher<T> * GetDispatcher(UInt64 offset) {
	return (EventDispatcher<T> *)((UInt64)GetEventDispatcherList() + offset);
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
bool IsHoldingTwoHandedWeapon(Actor *actor);
bool IsOneHandedWeapon(TESObjectWEAP *weapon);
bool IsUnarmed(TESForm *equippedObject);

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

inline bool CanWeaponStab(TESObjectWEAP *weapon) {
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
void ModSpeedMult(Actor *actor, float amount);
void PlayMeleeImpactRumble(int hand);
MovementControllerNPC * GetMovementController(Actor *actor);
ActorCause * TESObjectREFR_GetActorCause(TESObjectREFR *refr);
void TESObjectREFR_SetActorCause(TESObjectREFR *refr, ActorCause* cause);
inline UInt32 GetVehicleHandle(Actor *actor) { return actor->unk1E8; }
UInt32 GetHorseHandle(TESObjectREFR *actor);
KnockState GetActorKnockState(Actor *actor);
inline bool IsReanimating(Actor *actor) { return (actor->actorState.flags08 >> 4) & 1; }
inline bool IsSwimming(Actor *actor) { return (actor->actorState.flags04 >> 10) & 1; }
inline bool IsStaggered(Actor *actor) { return (actor->actorState.flags08 >> 13) & 1; }
inline bool IsSleeping(Actor *actor) { return ((actor->actorState.flags04 >> 14) & 0xF) == 7; }
bool IsActorGettingUp(Actor *actor);
float GetAVPercentage(Actor *actor, UInt32 av);
bool SendAction(Actor *source, TESObjectREFR *target, BGSAction *action);
void TriggerDialogue(Character *source, Character *target, int dialogueSubtype, bool interruptDialogue);
void ExitFurniture(Actor *actor);
bool HasKeepOffsetInterface(Actor * actor);
void Actor_GetBumpedEx(Actor *actor, Actor *bumper, bool isLargeBump, bool exitFurniture, bool pauseCurrentDialogue, bool triggerDialogue);
void Actor_SayToEx(Actor *source, Actor *target, TESTopic *topic, TESTopicInfo *topicInfo = nullptr);
inline TESTopic * GetCurrentTopic(Actor *actor) { if (ActorProcessManager *process = actor->processManager) { return *(TESTopic **)((UInt64)process + 0x128); } return nullptr; }
TESTopicInfo * GetRandomTopicInfo(std::vector<UInt32> &topicInfoIDs, UInt32 exclude1 = 0, UInt32 exclude2 = 0);
inline void DamageAV(Actor *actor, UInt32 av, float value) { get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 2, av, value); }
inline bool IsActorUsingFurniture(Actor *actor) { return actor->actorState.flags04 & 0x3C000; }
inline bool IsTeammate(Actor *actor) { return actor->flags1 >> 26 & 1; }
inline UInt32 ActorProcess_GetCommandingActor(ActorProcessManager *process) { return process->middleProcess ? process->middleProcess->unk218 : *g_invalidRefHandle; }
inline UInt32 GetCommandingActor(Actor *actor) { return actor->processManager ? ActorProcess_GetCommandingActor(actor->processManager) : *g_invalidRefHandle; }
bool IsInFaction(Actor *actor, TESFaction *faction);
bool IsCalmed(Actor *actor);
NiPointer<NiAVObject> GetFirstPersonHandNode(bool isLeft);
bool IsHandWithinConeFromHmd(bool isLeft, float halfAngle);

constexpr int GetDialogueTypeFromSubtype(int subtype)
{
	if (subtype < 3) return 0; // kPlayerDialogue
	if (subtype < 14) return 1; // kCommandDialogue
	if (subtype < 15) return 2; // kSceneDialogue
	if (subtype < 26) return 4; // kFavors
	if (subtype < 55) return 3; // kCombat
	if (subtype < 66) return 5; // kDetection
	if (subtype < 75) return 6; // kService
	else return 7; // kMiscellaneous
}
