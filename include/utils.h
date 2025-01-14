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
inline UInt64 *get_vtbl(void *object) { return *((UInt64 **)object); }

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
inline EventDispatcher<T> *GetDispatcher(UInt64 offset) {
    return (EventDispatcher<T> *)((UInt64)GetEventDispatcherList() + offset);
}

template<class T>
inline T *GetDefaultObject(int index) {
    bool *objectsInit = (bool *)((UInt64)g_defaultObjectManager.GetUIntPtr() + 0xBA8);
    if (objectsInit[index]) {
        return (T *)g_defaultObjectManager->objects[index];
    }
    return nullptr;
}

NiAVObject *GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject *root, NiAVObject::ControllerUpdateContext *ctx);
void UpdateKeyframedNodeTransform(NiAVObject *node, const NiTransform &transform);

NiAVObject *GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo *modInfo, UInt32 formLower);

bool IsAllowedCollidable(const hkpCollidable *collidable);
bool HasGeometryChildren(NiAVObject *obj);

bool IsTwoHanded(const TESObjectWEAP *weap);
bool IsBow(const TESObjectWEAP *weap);
TESObjectWEAP *GetEquippedWeapon(Actor *actor, bool isOffhand);
bool IsHoldingTwoHandedWeapon(Actor *actor);
bool IsOneHandedWeapon(TESObjectWEAP *weapon);
bool IsUnarmed(TESForm *equippedObject);
bool ShouldBashBasedOnWeapon(Actor *actor, bool isOffhand, bool isLeft, bool allowWeaponBash, const NiPoint3 *hitPosition);

double GetTime();

void PrintVector(const NiPoint3 &p);
void PrintSceneGraph(NiAVObject *node);
void PrintToFile(std::string entry, std::string filename);

bool VisitNodes(NiAVObject *parent, std::function<bool(NiAVObject *, int)> functor, int depth = 0);

inline void ltrim(std::string &s) { s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !::isspace(ch); })); }
inline void rtrim(std::string &s) { s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !::isspace(ch); }).base(), s.end()); }
inline void trim(std::string &s) { ltrim(s); rtrim(s); }
std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

inline VRMeleeData *GetVRMeleeData(bool isLeft) { return (VRMeleeData *)((UInt64)*g_thePlayer + 0x710 + (isLeft ? sizeof(VRMeleeData) : 0)); };

inline bool CanWeaponStab(TESObjectWEAP *weapon) {
    UInt8 type = weapon->type();
    return (
        type == TESObjectWEAP::GameData::kType_OneHandSword ||
        type == TESObjectWEAP::GameData::kType_OneHandDagger ||
        type == TESObjectWEAP::GameData::kType_TwoHandSword
        );
};

inline NiPointer<NiAVObject> GetWandNode(bool isLeft) { return isLeft ? (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_LeftWandNode] : (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_RightWandNode]; }

NiPointer<bhkCollisionObject> GetCollisionObject(NiAVObject *obj);
NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj);
NiPointer<bhkRigidBody> GetFirstRigidBody(NiAVObject *root);
NiPointer<bhkRigidBody> FindRigidBody(NiAVObject *root, bhkRigidBody *query);
NiPointer<NiAVObject> GetClosestParentWithCollision(NiAVObject *node, bool ignoreSelf = false);
void ForEachRagdollDriver(BSAnimationGraphManager *graphManager, std::function<void(hkbRagdollDriver *)> f);
void ForEachRagdollDriver(Actor *actor, std::function<void(hkbRagdollDriver *)> f);
void ForEachAnimationGraph(BSAnimationGraphManager *graphManager, std::function<void(BShkbAnimationGraph *)> f);
void ForEachAdjacentBody(NiAVObject *root, bhkRigidBody *body, std::function<void(bhkRigidBody *, int)> f, int waves);
NiTransform GetRigidBodyTLocalTransform(bhkRigidBody *rigidBody, bool useHavokScale = true);
bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node);
bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target);
bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node);
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target);
void GetAllSkinnedNodes(NiAVObject *root, std::unordered_set<NiAVObject *> &skinnedNodes);
void PlayTopicInfoWithoutActorChecks(TESTopicInfo *topicInfo, Actor *source, TESObjectREFR *target);
bool TESTopicInfo_EvaluateConditionsEx(TESTopicInfo *topicInfo, Actor *source, TESObjectREFR *target, const std::vector<UInt16> &skipConditions);
void PlayDialogueWithoutActorChecks(int subType, Actor *source, TESObjectREFR *target);
UInt32 PlaySoundAtNode(BGSSoundDescriptorForm *sound, NiAVObject *node, const NiPoint3 &location);
void PlayPhysicsSound(hkpCollidable *collidable, const NiPoint3 &location, bool loud);
void ModSpeedMult(Actor *actor, float amount);
void PlayMeleeImpactRumble(int hand);
MovementControllerNPC *GetMovementController(Actor *actor);
ActorCause *TESObjectREFR_GetActorCause(TESObjectREFR *refr);
void TESObjectREFR_SetActorCause(TESObjectREFR *refr, ActorCause *cause);
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
void TriggerDialogueByType(Character *source, Character *target, int dialogueSubtype, bool interruptDialogue);
void ExitFurniture(Actor *actor);
bool HasKeepOffsetInterface(Actor *actor);
void Actor_GetBumpedEx(Actor *actor, Actor *bumper, bool isLargeBump, bool exitFurniture, bool pauseCurrentDialogue, bool triggerDialogue);
void Actor_SayToEx(Actor *source, Actor *target, TESTopic *topic, TESTopicInfo *topicInfo = nullptr);
inline TESTopic *GetCurrentTopic(Actor *actor) { if (ActorProcessManager *process = actor->processManager) { return *(TESTopic **)((UInt64)process + 0x128); } return nullptr; }
TESTopicInfo *GetRandomTopicInfo(const std::vector<TESTopicInfo *> &topicInfos, TESTopicInfo *exclude1 = nullptr, TESTopicInfo *exclude2 = nullptr);
std::vector<TESTopicInfo *> EvaluateTopicInfoConditions(const std::vector<UInt32> &topicInfoIDs, Actor *source, Actor *target, const std::vector<UInt16> &skipConditions);
inline void DamageAV(Actor *actor, UInt32 av, float value) { get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 2, av, value); }
inline bool IsActorUsingFurniture(Actor *actor) { return actor->actorState.flags04 & 0x3C000; }
inline bool IsTeammate(Actor *actor) { return actor->flags1 >> 26 & 1; }
inline UInt32 ActorProcess_GetCommandingActor(ActorProcessManager *process) { return process->middleProcess ? process->middleProcess->unk218 : *g_invalidRefHandle; }
inline UInt32 GetCommandingActor(Actor *actor) { return actor->processManager ? ActorProcess_GetCommandingActor(actor->processManager) : *g_invalidRefHandle; }
bool IsInFaction(Actor *actor, TESFaction *faction);
bool IsCalmed(Actor *actor);
NiPointer<NiAVObject> GetFirstPersonHandNode(bool isLeft);
NiPointer<NiAVObject> GetWeaponCollisionOffsetNode(TESObjectWEAP *weapon, bool isLeft);
bool IsHandWithinConeFromHmd(bool isLeft, float halfAngle);
inline UInt32 GetEnabledInputs() { return UInt32(InputManager::GetSingleton()->unk138); }
inline bool AreCombatControlsEnabled() { return GetEnabledInputs() & UInt32(EnabledInputs::fighting); }
inline UInt32 GetAttackState(Actor *actor) { return (actor->actorState.flags04 >> 28) & 0xF; }
inline void SetAttackState(Actor *actor, UInt32 attackState) {
    actor->actorState.flags04 &= 0xFFFFFFFu; // zero out attackState
    actor->actorState.flags04 |= attackState << 28;
}

inline UInt8 GetPartNumber(UInt32 collisionFilterInfo) { return (collisionFilterInfo >> 8) & 0x1f; }
inline void SetPartNumber(hkUint32 &collisionFilterInfo, UInt8 partNumber) {
    collisionFilterInfo &= ~(0x1f << 8);
    collisionFilterInfo |= ((partNumber & 0x1f) << 8);
}
inline UInt8 GetPartNumber(hkpRigidBody *rigidBody) { return GetPartNumber(rigidBody->getCollisionFilterInfo()); }
inline bool IsRagdollHandFilter(UInt32 collisionFilterInfo) { UInt8 partNumber = GetPartNumber(collisionFilterInfo); return partNumber == 13 || partNumber == 7; } // right hand || left hand
inline bool IsRagdollHandRigidBody(hkpRigidBody *rigidBody) { return IsRagdollHandFilter(rigidBody->getCollisionFilterInfo()); }

inline UInt32 GetCollisionLayer(UInt32 collisionFilterInfo) { return collisionFilterInfo & 0x7f; }
inline void SetCollisionLayer(hkUint32 &collisionFilterInfo, UInt32 layer) {
    collisionFilterInfo &= ~(0x7f); // zero out layer
    collisionFilterInfo |= (layer & 0x7f); // set layer to the same as a dead ragdoll
}
inline UInt32 GetCollisionLayer(hkpRigidBody *rigidBody) { return GetCollisionLayer(rigidBody->getCollisionFilterInfo()); }
inline void SetCollisionLayer(hkpRigidBody *rigidBody, UInt32 layer) { return SetCollisionLayer(rigidBody->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo, layer); }

inline UInt16 GetCollisionGroup(UInt32 collisionFilterInfo) { return collisionFilterInfo >> 16; }
inline UInt16 GetCollisionGroup(hkpRigidBody *rigidBody) { return GetCollisionGroup(rigidBody->getCollisionFilterInfo()); }

inline UInt64 GetAttackActionId(bool isOffhand) { return isOffhand ? 45 : 49; } // 45 and 49 are kActionLeftAttack and kActionRightAttack
inline UInt64 GetPowerAttackActionId(bool isOffhand) { return isOffhand ? 69 : 70; } // 69 and 70 are kActionLeftPowerAttack and kActionRightPowerAttack

inline int GetAttackDialogueSubtype(bool isPowerAttack, bool isBash) { return isBash ? 28 : (isPowerAttack ? 27 : 26); } // 26 is attack, 27 powerattack, 28 bash

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
