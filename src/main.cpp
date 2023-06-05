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
#include <Physics/Dynamics/Constraint/Bilateral/BallAndSocket/hkpBallAndSocketConstraintData.h>

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
#include "menu_checker.h"
#include "pluginapi.h"
#include "papyrusapi.h"


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;
SKSETrampolineInterface *g_trampoline = nullptr;
SKSETaskInterface *g_taskInterface = nullptr;
SKSEPapyrusInterface *g_papyrus = nullptr;

BGSKeyword *g_keyword_actorTypeAnimal = nullptr;
BGSKeyword *g_keyword_actorTypeNPC = nullptr;

vr_src::VRControllerAxis_t g_rightStick;
vr_src::VRControllerAxis_t g_leftStick;


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

typedef void(*_VMClassRegistry_Destruct)(VMClassRegistry *_this, UInt32 unk);
bool DispatchHitEvents(TESObjectREFR *source, TESObjectREFR *target, hkpRigidBody *hitBody, TESForm *weapon)
{
    SkyrimVM *vm = *g_skyrimVM;
    if (VMClassRegistry *registry = vm->GetClassRegistry()) {
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
            TESHitEvent hitEvent{ target, source, weapon->formID, 0, 0 };
            GetDispatcher<TESHitEvent>(0x5D8)->SendEvent(&hitEvent);
        }

        // vm decref
        if (InterlockedExchangeSubtract(((UInt32 *)&registry->unk0004), (UInt32)1) == 1) {
            get_vfunc<_VMClassRegistry_Destruct>(registry, 0x0)(registry, 1);
        }

        return shouldDispatchHitEvent;
    }

    return false;
}

float GetPhysicsDamage(float mass, float speed, float minMass, float minSpeed)
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

    if (mass < minMass) return 0.f;
    if (speed < minSpeed) return 0.f;

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

struct DelayedJob : GenericJob
{
    DelayedJob(std::function<void(void)> job, double delay) :
        job(job)
    {
        runTime = g_currentFrameTime + delay;
    }

    virtual void Run() override
    {
        job();
    }

    double runTime = 0.0;
    std::function<void(void)> job;
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


void UpdateCollisionFilterOnAllBones(Actor *actor)
{
    if (Actor_IsInRagdollState(actor)) return;

    bool hasRagdollInterface = false;
    BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
    if (GetAnimationGraphManager(actor, animGraphManager)) {
        BSAnimationGraphManager_HasRagdoll(animGraphManager.ptr, &hasRagdollInterface);
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

std::vector<std::unique_ptr<GenericJob>> g_prePhysicsStepJobs{};

template<class T, typename... Args>
void QueuePrePhysicsJob(Args&&... args)
{
    static_assert(std::is_base_of<GenericJob, T>::value);
    g_prePhysicsStepJobs.push_back(std::make_unique<T>(std::forward<Args>(args)...));
}

std::vector<std::unique_ptr<DelayedJob>> g_delayedJobs{};

void QueueDelayedJob(std::function<void(void)> job, double delay)
{
    g_delayedJobs.push_back(std::make_unique<DelayedJob>(job, delay));
}

void RunDelayedJobs(double now)
{
    for (int i = 0; i < g_delayedJobs.size(); ++i) {
        DelayedJob *job = g_delayedJobs[i].get();
        if (now >= job->runTime) {
            job->Run();

            // Constant-time removal
            std::swap(g_delayedJobs[i], g_delayedJobs.back());
            g_delayedJobs.pop_back();

            --i; // so that the increment brings it back to the current value
        }
    }
}

struct ControllerTrackingData
{
    std::deque<NiPoint3> velocities{ 50, NiPoint3() };
    NiPoint3 avgVelocity;
    float avgSpeed;

    std::deque<NiPoint3> positionsRoomspace{ 50, NiPoint3() };
    NiPoint2 angularVelocity;

    NiPoint3 GetAverageVector(std::deque<NiPoint3> &vectors, int numFrames)
    {
        NiPoint3 averageVector = NiPoint3();

        int i = 0;
        for (NiPoint3 &vector : vectors) {
            averageVector += vector;
            if (++i >= numFrames) {
                break;
            }
        }

        return averageVector / i;
    }

    float GetAverageVectorLength(std::deque<NiPoint3> &vectors, int numFrames)
    {
        float length = 0;

        int i = 0;
        for (NiPoint3 &vector : vectors) {
            length += VectorLength(vector);
            if (++i >= numFrames) {
                break;
            }
        }
        return length /= i;
    }

    int GetNumSmoothingFrames()
    {
        int numSmoothingFrames = Config::options.numControllerVelocitySmoothingFrames;
        float smoothingMultiplier = 0.011f / *g_deltaTime; // Half the number of frames at 45fps compared to 90fps, etc.
        return round(float(numSmoothingFrames) * smoothingMultiplier);
    }

    int GetNumControllerAngularVelocityTrackingFrames()
    {
        int numFrames = Config::options.numControllerAngularVelocityTrackingFrames;
        float smoothingMultiplier = 0.011f / *g_deltaTime; // Half the number of frames at 45fps compared to 90fps, etc.
        return round(float(numFrames) * smoothingMultiplier);
    }

    void ComputeAverageVelocity()
    {
        int numSmoothingFrames = GetNumSmoothingFrames();
        avgVelocity = GetAverageVector(velocities, numSmoothingFrames);
    }

    void ComputeAverageSpeed()
    {
        int numSmoothingFrames = GetNumSmoothingFrames();
        avgSpeed = GetAverageVectorLength(velocities, numSmoothingFrames);
    }

    void ComputeAngularVelocity()
    {
        int numFrames = GetNumControllerAngularVelocityTrackingFrames();

        NiPointer<NiAVObject> hmdNode = (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
        NiPoint3 hmdLocalPos = hmdNode->m_localTransform.pos;

        NiPoint2 eulerSums = { 0.f, 0.f };

        int i = 0;
        NiPoint3 *prev = nullptr;
        for (NiPoint3 &positionRoomspace : positionsRoomspace) {
            if (prev) {
                // "prev" in this case is actually the _newer_ value, as this deque is ordered from new to old
                NiPoint3 hmdToController = VectorNormalized(positionRoomspace - hmdLocalPos);
                NiMatrix33 rot = MatrixFromForwardVector(hmdToController, NiPoint3(0.f, 0.f, 1.f));
                NiPoint3 olderEuler = NiMatrixToYawPitchRoll(rot); // z x y

                hmdToController = VectorNormalized(*prev - hmdLocalPos);
                rot = MatrixFromForwardVector(hmdToController, NiPoint3(0.f, 0.f, 1.f));
                NiPoint3 newerEuler = NiMatrixToYawPitchRoll(rot); // z x y

                NiPoint3 eulerDiff = newerEuler - olderEuler;
                eulerSums.x += eulerDiff.x; // yaw
                eulerSums.y += eulerDiff.y; // pitch
            }

            prev = &positionRoomspace;

            if (++i >= numFrames) {
                break;
            }
        }

        angularVelocity = eulerSums;
    }
};
ControllerTrackingData g_controllerData[2]; // one for each hand

std::unordered_set<Actor *> g_activeActors{};
std::unordered_set<UInt16> g_activeBipedGroups{};
std::unordered_set<UInt16> g_noPlayerCharControllerCollideGroups{};
std::unordered_set<UInt16> g_hittableCharControllerGroups{};
std::unordered_set<UInt16> g_selfCollidableBipedGroups{};

std::unordered_map<bhkRigidBody *, double> g_higgsLingeringRigidBodies{};
bhkRigidBody *g_rightHand = nullptr;
bhkRigidBody *g_leftHand = nullptr;
bhkRigidBody *g_rightWeapon = nullptr;
bhkRigidBody *g_leftWeapon = nullptr;
bhkRigidBody *g_rightHeldObject = nullptr;
bhkRigidBody *g_leftHeldObject = nullptr;
TESObjectREFR *g_rightHeldRefr = nullptr;
TESObjectREFR *g_leftHeldRefr = nullptr;
UInt16 g_rightHeldActorCollisionGroup = 0;
UInt16 g_leftHeldActorCollisionGroup = 0;
UInt32 g_higgsCollisionLayer = 56;

bhkRigidBody *g_prevRightHeldObject = nullptr;
bhkRigidBody *g_prevLeftHeldObject = nullptr;

UInt16 g_playerCollisionGroup = 0;

inline bool IsLeftRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return wrapper == g_leftHand || wrapper == g_leftWeapon || wrapper == g_leftHeldObject;
}

inline bool IsPlayerWeaponRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return wrapper == g_leftWeapon || wrapper == g_rightWeapon;
}

inline bool IsPlayerHandRigidBody(hkpRigidBody *rigidBody)
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
    return IsPlayerHandRigidBody(rigidBody) || IsPlayerWeaponRigidBody(rigidBody) || IsHeldRigidBody(rigidBody);
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
    if (!Config::options.enableBump) return false;
    if (Actor_IsRunning(actor) || Actor_IsGhost(actor) || actor->IsInCombat() || Actor_IsInRagdollState(actor)) return false;
    if (!actor->race || actor->race->data.unk40 >= 2) return false; // race size is >= large
    return true;
}

bool ShouldShoveActor(Actor *actor)
{
    if (Actor_IsGhost(actor) || Actor_IsInRagdollState(actor)) return false;
    if (!Config::options.enableShoveFromFurniture && IsActorUsingFurniture(actor)) return false;
    if (!actor->race || actor->race->data.unk40 >= 2) return false; // race size is >= large or is child
    return true;
}

void BumpActor(Actor *actor, float bumpDirection, bool isLargeBump = false, bool exitFurniture = false, bool pauseCurrentDialogue = true, bool triggerDialogue = true)
{
    ActorProcessManager *process = actor->processManager;
    if (!process) return;

    ActorProcess_SetBumpState(process, isLargeBump ? 1 : 0);
    ActorProcess_SetBumpDirection(process, bumpDirection);
    Actor_GetBumpedEx(actor, *g_thePlayer, isLargeBump, exitFurniture, pauseCurrentDialogue, triggerDialogue);
    ActorProcess_SetBumpDirection(process, 0.f);
}

struct BumpRequest
{
    NiPoint3 bumpDirectionVec{};
    float bumpDirection = 0.f;
    bool isLargeBump = false;
    bool exitFurniture = false;
    bool pauseCurrentDialogue = true;
    bool triggerDialogue = true;
};
std::mutex g_bumpActorsLock;
std::unordered_map<Actor *, BumpRequest> g_bumpActors{};
std::unordered_map<Actor *, double> g_shovedActors{};

std::mutex g_shoveTimesLock;
struct ShoveData
{
    NiPoint3 shoveDirection{};
    double shovedTime = 0.0;
};
std::unordered_map<IAnimationGraphManagerHolder *, ShoveData> g_shoveData{};
std::mutex g_shoveAnimLock;
std::unordered_map<IAnimationGraphManagerHolder *, double> g_shoveAnimTimes{};

void QueueBumpActor(Actor *actor, NiPoint3 direction, float bumpDirection, bool isLargeBump, bool exitFurniture, bool pauseCurrentDialogue = true, bool triggerDialogue = true)
{
    std::scoped_lock lock(g_bumpActorsLock);
    auto it = g_bumpActors.find(actor);
    if (it == g_bumpActors.end()) {
        g_bumpActors[actor] = { direction, bumpDirection, isLargeBump, exitFurniture, pauseCurrentDialogue, triggerDialogue };
    }
}

void QueueBumpActor(Actor *actor, NiPoint3 direction, bool isLargeBump, bool exitFurniture, bool pauseCurrentDialogue = true, bool triggerDialogue = true)
{
    float heading = GetHeadingFromVector(-direction);
    float bumpDirection = heading - get_vfunc<_Actor_GetHeading>(actor, 0xA5)(actor, false);
    QueueBumpActor(actor, direction, bumpDirection, isLargeBump, exitFurniture, pauseCurrentDialogue, triggerDialogue);
}

void StaggerActor(Actor *actor, NiPoint3 direction, float magnitude)
{
    // Stagger direction goes from 0 to 1, where 0/1 is stagger backwards, and 0.5 is stagger forwards
    float heading = GetHeadingFromVector(-direction);
    float staggerHeading = heading - get_vfunc<_Actor_GetHeading>(actor, 0xA5)(actor, false);
    float staggerDirection = ConstrainAngle360(staggerHeading) / (2.f * M_PI);

    static BSFixedString staggerMagnitudeStr("staggerMagnitude");
    IAnimationGraphManagerHolder_SetAnimationVariableFloat(&actor->animGraphHolder, staggerMagnitudeStr, magnitude);

    static BSFixedString staggerDirectionStr("staggerDirection");
    IAnimationGraphManagerHolder_SetAnimationVariableFloat(&actor->animGraphHolder, staggerDirectionStr, staggerDirection);

    static BSFixedString staggerStartStr("staggerStart");
    get_vfunc<_IAnimationGraphManagerHolder_NotifyAnimationGraph>(&actor->animGraphHolder, 0x1)(&actor->animGraphHolder, staggerStartStr);
}

bool ShouldKeepOffset(Actor *actor)
{
    if (!Config::options.doKeepOffset) return false;
    if (Actor_IsGhost(actor)) return false;
    if (IsActorUsingFurniture(actor)) return false;

    if (!actor->race || actor->race->data.unk40 >= 2) return false; // race size is >= large

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

    float health = actor->actorValueOwner.GetCurrent(24);
    if (health < Config::options.smallRaceHealthThreshold) return true;

    return false;
}


int g_numSkipAnimationFrames = 0;

struct SwingHandler
{
    enum class SwingState
    {
        None,
        PreSwing,
        Swing,
    };

    bool isLeft;
    SwingState swingState = SwingState::None;
    float lastSwingSpeed = 0.f;
    bool wasLastSwingPowerAttack = false;
    bool wasLastSwingBash = false;
    bool didLastSwingFail = false;
    float swingCooldown = 0.f;
    float swingDuration = 0.f;
    float lastDeltaTime = 1.f;

    SwingHandler(bool isLeft) : isLeft(isLeft) {}

    inline bool IsSwingCoolingDown()
    {
        return swingCooldown > 0.f;
    }

    inline bool IsSwingActive()
    {
        return swingDuration > 0.f;
    }

    inline bool IsPowerAttackActive()
    {
        return IsSwingActive() && wasLastSwingPowerAttack;
    }

    inline bool IsBashActive()
    {
        return IsSwingActive() && wasLastSwingBash;
    }

    void DeductSwingStamina(float staminaCost)
    {
        PlayerCharacter *player = *g_thePlayer;

        float staminaBeforeHit = player->actorValueOwner.GetCurrent(26);

        DamageAV(player, 26, -staminaCost);

        if (player->actorValueOwner.GetCurrent(26) <= 0.f) {
            // Out of stamina after the swing
            if (ActorProcessManager *process = player->processManager) {
                float regenRate = Actor_GetActorValueRegenRate(player, 26);
                ActorProcess_UpdateRegenDelay(process, 26, (staminaCost - staminaBeforeHit) / regenRate);
                FlashHudMenuMeter(26);
            }
        }
    }

    // Return true if power attack succeeded, false if regular attack
    bool TrySwingPowerAttack(bool isOffhand, bool isBash, bool isWeaponSwingWhileBlockingWithShield)
    {
        PlayerCharacter *player = *g_thePlayer;

        BGSAttackData *attackData = nullptr;
        PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, true, &attackData);
        if (!attackData) return false;

        float staminaCost = ActorValueOwner_GetStaminaCostForAttackData(&player->actorValueOwner, attackData);
        float currentStamina = player->actorValueOwner.GetCurrent(26);
        if (staminaCost > 0.f && currentStamina <= 0.f) {
            if (!isBash) {
                // No stamina to power attack, so re-set the attackdata but this time explicitly set powerattack to false
                PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, false, &attackData);
                // Now do a regular attack (overwrites the attack data too)
                PlayerControls_SendAction(PlayerControls::GetSingleton(), GetAttackActionId(isOffhand), 2);
            }
            FlashHudMenuMeter(26);
            return false;
        }

        if (isWeaponSwingWhileBlockingWithShield) {
            // Stop blocking with the shield right before performing the swing action
            PlayerControls_SendAction(PlayerControls::GetSingleton(), 47, 2); // kActionLeftRelease
        }

        // We send this first, as it is a regular attack and executes some side effects, and does not deduct stamina since we don't make this a power attack.
        // It also only triggers a regular attack animation event, which we can send a power attack anim event on top of, which is not possible the other way around.
        // This sends the ActionLeftAttack/ActionRightAttack action, which sets the last BGSAttackData (overrides us from before...) to the regular attackStart attackdata.
        // If it's a shield (bash), this will make us block/unblock instantly, which may be desirable as we are bashing?
        PlayerControls_SendAction(PlayerControls::GetSingleton(), GetAttackActionId(isOffhand), 2);

        // Do this again after the above func overwrote the attackData
        PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, true, &attackData);

        if (!get_vfunc<_MagicTarget_IsInvulnerable>(&player->magicTarget, 4)(&player->magicTarget) && staminaCost > 0.f) {
            DeductSwingStamina(staminaCost);
        }

        if (SpellItem *attackSpell = attackData->data.attackSpell) {
            Actor_DoCombatSpellApply(player, attackSpell, nullptr);
        }

        get_vfunc<_IAnimationGraphManagerHolder_NotifyAnimationGraph>(&player->animGraphHolder, 0x1)(&player->animGraphHolder, attackData->event);

        if (!isBash) { // bash dialogue will be handled if the bash actually hits
            // Trigger dialogue here, since WeaponSwingCallback() won't properly handle the powerattack case, and it won't interrupt us if we do it here.
            TriggerDialogueByType(player, nullptr, 27, false); // 27 == powerAttack
        }

        *((UInt8 *)player + 0x12D7) |= 0x1C;

        return true;
    }

    // Return true if bash succeeded, false if it didn't (and hence no attack should occur)
    bool TryBash(bool isOffhand)
    {
        PlayerCharacter *player = *g_thePlayer;

        BGSAttackData *attackData = nullptr;
        PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, false, &attackData);
        if (!attackData) return false;

        float staminaCost = ActorValueOwner_GetStaminaCostForAttackData(&player->actorValueOwner, attackData);
        float currentStamina = player->actorValueOwner.GetCurrent(26);
        if (staminaCost > 0.f && currentStamina <= 0.f) {
            // No stamina to bash
            if (Config::options.failBashWhenOutOfStamina) {
                SetAttackState(player, 0);
                FlashHudMenuMeter(26);
                return false;
            }
        }
        else {
            // Enough stamina
            if (!get_vfunc<_MagicTarget_IsInvulnerable>(&player->magicTarget, 4)(&player->magicTarget) && staminaCost > 0.f) {
                DeductSwingStamina(staminaCost);
            }
        }

        if (SpellItem *attackSpell = attackData->data.attackSpell) {
            Actor_DoCombatSpellApply(player, attackSpell, nullptr);
        }

        // Play bash animation
        get_vfunc<_IAnimationGraphManagerHolder_NotifyAnimationGraph>(&player->animGraphHolder, 0x1)(&player->animGraphHolder, attackData->event);

        return true;
    }

    void SwingWeapon(TESObjectWEAP *weapon, bool isOffhand, bool isBash, bool playSound = true)
    {
        PlayerCharacter *player = *g_thePlayer;

        if (playSound) {
            if (weapon) {
                if (weapon->type() < TESObjectWEAP::GameData::kType_Bow) { // not bow, staff, or crossbow
                    if (BGSSoundDescriptorForm *sound = weapon->attackFailSound) {
                        if (NiPointer<NiAVObject> node = GetWeaponCollisionOffsetNode(weapon, isOffhand)) {
                            PlaySoundAtNode(sound, node, {});
                        }
                    }
                }
            }
        }

        if (!isBash) {
            get_vfunc<_Actor_WeaponSwingCallback>(player, 0xF1)(player); // Reads from last attackData and applies kApplyWeaponSwingSpell entrypoint (which could read from attackState)
        }

        if (ActorProcessManager *process = player->processManager) {
            ActorProcess_IncrementAttackCounter(process, 1);
        }

        // Make noise
        int soundAmount = weapon ? TESObjectWEAP_GetSoundAmount(weapon) : TESNPC_GetSoundAmount((TESNPC *)player->baseForm);
        Actor_SetActionValue(player, soundAmount);

        if (player->unk158) {
            CombatController_SetLastAttackTimeToNow((void *)player->unk158);
        }
    }

    void Swing(bool isHit = false, bool isStab = false, const NiPoint3 *hitPosition = nullptr)
    {
        bool isOffhand = isLeft != *g_leftHandedMode;

        PlayerCharacter *player = *g_thePlayer;

        VRMeleeData *meleeData = GetVRMeleeData(isLeft);
        NiPoint2 angularVelocity = g_controllerData[isLeft].angularVelocity;
        if (angularVelocity.y > 0.f) {
            // Positive y means swinging down
            angularVelocity.y *= Config::options.swingDownwardsSpeedMultipler;
        }

        // The power attack direction is based on swinging direction. If we are moving forwards and it's a standing power attack (swinging down) then it's a forwards power attack.
        vr_src::VRControllerAxis_t &movementStick = *g_leftHandedMode ? g_rightStick : g_leftStick;
        bool isMovingForward = movementStick.y > Config::options.forwardsPowerAttackStickForwardThreshold;
        meleeData->swingDirection = VRMeleeData::GetSwingDirectionFromAngularVelocities(angularVelocity.x, angularVelocity.y);
        if (isStab) {
            // Treat a stab as an in-place (downwards) attack. Below it can be converted to a forwards attack.
            meleeData->swingDirection = VRMeleeData::SwingDirection::kDown;
        }
        if (isMovingForward && meleeData->swingDirection == VRMeleeData::SwingDirection::kDown) {
            // Turn a standing power attack into a forwards power attack if the left joystick is being pushed sufficiently forward
            meleeData->swingDirection = VRMeleeData::SwingDirection::kForward;
        }

        TESForm *offhandObj = player->GetEquippedObject(true);
        TESObjectARMO *equippedShield = (offhandObj && offhandObj->formType == kFormType_Armor) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectARMO) : nullptr;
        bool isShield = isOffhand && equippedShield;

        TESObjectLIGH *equippedLight = (offhandObj && offhandObj->formType == kFormType_Light) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectLIGH) : nullptr;
        bool isTorch = isOffhand && equippedLight;

        TESObjectWEAP *weapon = GetEquippedWeapon(player, isOffhand);
        // Realistically, it can't be a staff as staffs don't have melee collision (yet...?)
        bool isStaffOrBowOrCrossbow = weapon && weapon->type() >= TESObjectWEAP::GameData::kType_Bow;

        bool canPowerAttack = !isTorch && !isStaffOrBowOrCrossbow; // unarmed, melee weapon, or shield

        bool isTriggerHeld = isOffhand ? PlayerControls_IsTriggerHeldOffHand(PlayerControls::GetSingleton()) : PlayerControls_IsTriggerHeldMainHand(PlayerControls::GetSingleton());

        bool allowWeaponBash = Config::options.enableWeaponBash && isHit && !isTriggerHeld && hitPosition && !isStab;
        bool isBash = ShouldBashBasedOnWeapon(player, isOffhand, isLeft, allowWeaponBash, hitPosition);

        bool isWeaponSwingWhileBlockingWithShield = Actor_IsBlocking(player) && !isBash && !isOffhand && equippedShield;

        // Need to set attackState before UpdateAndGetAttackData()
        UInt32 newAttackState = isBash ? 6 : 2; // kBash if bashing else kSwing
        SetAttackState(player, newAttackState);

        didLastSwingFail = false;
        wasLastSwingBash = false;
        if (isTriggerHeld && canPowerAttack) {
            bool didPowerAttackSucceed = TrySwingPowerAttack(isOffhand, isBash, isWeaponSwingWhileBlockingWithShield); // this also sets the attackData
            if (!didPowerAttackSucceed && isBash) {
                bool didBashSucceed = TryBash(isOffhand);
                if (didBashSucceed) {
                    wasLastSwingBash = true;
                }
                else {
                    Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
                    if (BGSSoundDescriptorForm *magicFailSound = GetDefaultObject<BGSSoundDescriptorForm>(128)) {
                        PlaySoundAtNode(magicFailSound, player->GetNiNode(), {});
                    }
                }
                didLastSwingFail = !didBashSucceed;
            }
            if (didPowerAttackSucceed && isBash) {
                wasLastSwingBash = true;
            }
            wasLastSwingPowerAttack = didPowerAttackSucceed;
        }
        else {
            wasLastSwingPowerAttack = false;

            if (isBash) {
                bool didBashSucceed = TryBash(isOffhand);
                if (didBashSucceed) {
                    wasLastSwingBash = true;
                }
                else {
                    Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
                    if (BGSSoundDescriptorForm *magicFailSound = GetDefaultObject<BGSSoundDescriptorForm>(128)) {
                        PlaySoundAtNode(magicFailSound, player->GetNiNode(), {});
                    }
                }
                didLastSwingFail = !didBashSucceed;
            }
            else {
                if (isWeaponSwingWhileBlockingWithShield) {
                    // Stop blocking with the shield right before performing the swing action
                    PlayerControls_SendAction(PlayerControls::GetSingleton(), 47, 2); // kActionLeftRelease
                }

                // This sends the ActionLeftAttack/ActionRightAttack action, which sets the last BGSAttackData (overrides us from before...) to the regular attackStart attackdata.
                // That means it notifies the anim graph with attackStart/attackStartLeftHand too.
                // It also sets the attackState to kDraw (1)
                PlayerControls_SendAction(PlayerControls::GetSingleton(), GetAttackActionId(isOffhand), 2);
            }
        }

        if (!didLastSwingFail) {
            // Set the attackState again after it gets set to kDraw by PlayerControls_SendAction()
            SetAttackState(player, newAttackState);

            SwingWeapon(weapon, isOffhand, isBash); // Reads from last attackData and attackState

            g_numSkipAnimationFrames = isBash ? Config::options.swingSkipAnimFramesBash : Config::options.swingSkipAnimFrames; // skip the animation
        }

        swingState = didLastSwingFail ? SwingState::None : SwingState::Swing;

        swingCooldown = Config::options.swingCooldown;
        swingDuration = Config::options.swingDuration;
    }

    void UpdateWeaponSwing(float deltaTime)
    {
        lastDeltaTime = deltaTime;
        swingCooldown -= deltaTime;
        swingDuration -= deltaTime;

        PlayerCharacter *player = *g_thePlayer;

        UInt32 attackState = GetAttackState(player);

        if (swingState == SwingState::Swing) {
            if (!IsSwingActive()) {
                if (attackState == 2 || attackState == 6) { // swing or bash
                    SetAttackState(player, 0);
                }
                swingState = SwingState::None;
            }
        }

        VRMeleeData *meleeData = GetVRMeleeData(isLeft);
        if (!meleeData->collisionNode) return;
        if (!player->actorState.IsWeaponDrawn()) return;

        if (!AreCombatControlsEnabled()) return;
        if (Actor_IsInRagdollState(player) || IsSwimming(player) || IsStaggered(player)) return;

        NiPointer<NiAVObject> hmdNode = (*g_thePlayer)->unk3F0[PlayerCharacter::kNode_HmdNode];
        if (!hmdNode) return;

        ControllerTrackingData &controllerData = g_controllerData[isLeft];
        float speed = controllerData.avgSpeed;

        NiPoint3 handDirection = VectorNormalized(controllerData.avgVelocity);
        NiPoint3 hmdForward = ForwardVector(hmdNode->m_worldTransform.rot);
        float handInHmdDirection = DotProduct(handDirection, hmdForward);

        if (swingState == SwingState::None) {
            if (swingCooldown <= 0.f) {
                if (speed > Config::options.swingLinearVelocityThreshold) {
                    swingState = SwingState::PreSwing;
                }
            }
        }

        if (swingState == SwingState::PreSwing) {
            if (speed < lastSwingSpeed) {
                // We are past the peak of the swing
                if (attackState == 0 && handInHmdDirection > Config::options.swingRequiredHandHmdDirection) {
                    Swing();
                }
                else {
                    // No swinging allowed while e.g. firing a crossbow, or swinging extremely backwards
                    swingState = SwingState::None;
                }
            }
        }

        lastSwingSpeed = speed;
    }
};
SwingHandler g_rightSwingHandler{ false };
SwingHandler g_leftSwingHandler{ true };


bool g_isInPlanckHit = false;

void HitActor(Character *source, Character *target, NiAVObject *hitNode, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, bool isLeft, bool isOffhand, bool isPowerAttack, bool isBash)
{
    // Handle bow/crossbow/torch/shield bash (set attackstate to kBash)
    TESForm *offhandObj = source->GetEquippedObject(true);
    TESObjectARMO *equippedShield = (offhandObj && offhandObj->formType == kFormType_Armor) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectARMO) : nullptr;
    bool isShield = isOffhand && equippedShield;

    SetAttackState(source, isBash ? 6 : 2); // kBash : kSwing

    if (isPowerAttack && isShield) {
        // power bash
        if (BGSAction *blockAnticipateAction = GetDefaultObject<BGSAction>(74)) {
            // This is cool, as it does not drain stamina or anything as was done as part of the swing. This will just do the actual bash recoil on the target.
            SendAction(source, target, blockAnticipateAction);
        }
        PlayerCharacter *player = *g_thePlayer;
        if (source == player) {
            *((UInt8 *)player + 0x12D7) |= 0x1C;
        }
    }

    // Do combat dialogue here (after the swing) in addition to during the swing, as this will handle bash dialogue which I'm fairly sure needs a target.
    int dialogueSubtype = isBash ? 28 : (isPowerAttack ? 27 : 26); // 26 is attack, 27 powerattack, 28 bash
    TriggerDialogueByType(source, target, dialogueSubtype, false);

#ifdef _DEBUG
    if (isPowerAttack) {
        _MESSAGE("%d Detect hit power attack", *g_currentFrameCounter);
    }
    else {
        _MESSAGE("%d Detect hit", *g_currentFrameCounter);
    }
#endif // _DEBUG

    // Set last hit data to be read from at the time that the hit event is fired
    PlanckPluginAPI::PlanckHitData &lastHitData = g_interface001.lastHitData;
    lastHitData = {};
    if (hitNode) {
        lastHitData.node = hitNode;
        lastHitData.nodeName = hitNode->m_name;
    }
    lastHitData.position = hitPosition;
    lastHitData.velocity = hitVelocity;
    lastHitData.isLeft = isLeft;

    g_isInPlanckHit = true;
    Character_HitTarget(source, target, nullptr, isOffhand); // Populates HitData and so relies on attackState/attackData
    g_isInPlanckHit = false;

    Actor_RemoveMagicEffectsDueToAction(source, -1); // removes invis/ethereal due to attacking

    SwingHandler &swingHandler = isLeft ? g_leftSwingHandler : g_rightSwingHandler;
    if (!swingHandler.IsSwingActive()) {
        SetAttackState(source, 0);
    }
}

void DoDestructibleDamage(Character *source, TESObjectREFR *target, bool isOffhand)
{
    float damage;
    { // All this just to get the fricken damage
        InventoryEntryData *weaponEntry = ActorProcess_GetCurrentlyEquippedWeapon(source->processManager, isOffhand);

        HitData hitData;
        HitData_ctor(&hitData);
        HitData_populate(&hitData, source, nullptr, weaponEntry, isOffhand);

        damage = hitData.totalDamage;

        HitData_dtor(&hitData);
    }

    BSTaskPool_QueueDestructibleDamageTask(BSTaskPool::GetSingleton(), target, damage);
}

void HitRefr(Character *source, TESObjectREFR *target, bool setCause, bool isLeft, bool isOffhand)
{
    SetAttackState(source, 2); // kSwing

    if (setCause) {
        TESObjectREFR_SetActorCause(target, TESObjectREFR_GetActorCause(source));
    }

    DoDestructibleDamage(source, target, isOffhand);

    SwingHandler &swingHandler = isLeft ? g_leftSwingHandler : g_rightSwingHandler;
    if (!swingHandler.IsSwingActive()) {
        SetAttackState(source, 0);
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

    float impulseSpeed = min(VectorLength(hitVelocity), Config::options.hitImpulseMaxVelocity / *g_globalTimeMultiplier); // limit the imparted velocity to some reasonable value
    NiPoint3 impulse = VectorNormalized(hitVelocity) * impulseSpeed * *g_havokWorldScale * mass; // This impulse will give the object the exact velocity it is hit with
    impulse *= impulseStrength; // Scale the velocity as we see fit
    impulse *= impulseMult;
    if (impulse.z < 0) {
        // Impulse points downwards somewhat, scale back the downward component so we don't get things shooting into the ground.
        impulse.z *= Config::options.hitImpulseDownwardsMultiplier;
    }

    return impulse;
}

void ApplyHitImpulse(bhkWorld *world, Actor *actor, hkpRigidBody *rigidBody, const NiPoint3 &hitVelocity, const NiPoint3 &position, float impulseMult)
{
    UInt32 targetHandle = GetOrCreateRefrHandle(actor);
    // Apply linear impulse at the center of mass to all bodies within 2 ragdoll constraints
    ForEachRagdollDriver(actor, [world, rigidBody, hitVelocity, impulseMult, targetHandle](hkbRagdollDriver *driver) {
        // No deadlock here as the BSAnimationGraphManager update lock has already been acquired
        BSReadLocker lock(&world->worldLock);
        ForEachAdjacentBody(driver, rigidBody, [driver, hitVelocity, impulseMult, targetHandle](hkpRigidBody *adjacentBody) {
            QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody, hitVelocity, impulseMult) * Config::options.hitImpulseDecayMult1, targetHandle);
            ForEachAdjacentBody(driver, adjacentBody, [driver, hitVelocity, impulseMult, targetHandle](hkpRigidBody *adjacentBody) {
                QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody, hitVelocity, impulseMult) * Config::options.hitImpulseDecayMult2, targetHandle);
                ForEachAdjacentBody(driver, adjacentBody, [hitVelocity, impulseMult, targetHandle](hkpRigidBody *adjacentBody) {
                    QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody, hitVelocity, impulseMult) * Config::options.hitImpulseDecayMult3, targetHandle);
                    });
                });
            });
        });

    // Apply a point impulse at the hit location to the body we actually hit
    QueuePrePhysicsJob<PointImpulseJob>(rigidBody, position, CalculateHitImpulse(rigidBody, hitVelocity, impulseMult), targetHandle);
}

BGSMaterialType *GetImpactMaterial(hkpRigidBody *hitRigidBody, const hkpContactPointEvent &evnt, const NiPoint3 &hitPosition)
{
    UInt32 materialId = 0;
    UInt32 hitLayer = hitRigidBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
    if (hitLayer == BGSCollisionLayer::kCollisionLayer_Ground) {
        materialId = TES_GetLandMaterialId(*g_tes, hitPosition);
    }
    else { // Not the ground
        if (bhkShape *shape = (bhkShape *)hitRigidBody->getCollidable()->getShape()->m_userData) {
            if (hkpShapeKey *shapeKey = evnt.getShapeKeys(evnt.getBody(1) == hitRigidBody)) {
                materialId = bhkShape_GetMaterialId(shape, *shapeKey);
            }
        }
    }

    return GetMaterialType(materialId);
}

BGSImpactData *GetImpactData(BGSMaterialType *material, TESForm *weapon, bool isBash)
{
    BGSImpactDataSet *impactSet = nullptr;

    if (isBash) {
        BGSBlockBashData *blockBashData = TESForm_GetBlockBashData(weapon);
        impactSet = blockBashData ? blockBashData->impact : (*g_unarmedWeapon)->impactDataSet;
    }
    else { // Not bash
        TESForm *weaponOrUnarmed = weapon ? weapon : *g_unarmedWeapon;
        if (weaponOrUnarmed->formType == kFormType_Weapon) {
            if (TESObjectWEAP *weap = DYNAMIC_CAST(weaponOrUnarmed, TESForm, TESObjectWEAP)) {
                impactSet = weap->impactDataSet;
            }
        }
    }

    if (!impactSet) return nullptr;

    return BGSImpactDataSet_GetImpactData(impactSet, material);
}

void PlayImpactEffects(TESObjectCELL *cell, BGSImpactData *impact, NiPoint3 hitPosition, const NiPoint3 &hitNormal)
{
    if (!impact) return;

    if (Config::options.playMeleeWorldImpactSounds) {
        ImpactSoundData impactSoundData{
            impact,
            &hitPosition,
            nullptr,
            nullptr,
            nullptr,
            true,
            false,
            false,
            nullptr
        };
        BGSImpactManager_PlayImpactSound(*g_impactManager, impactSoundData);
    }

    if (Config::options.playMeleeImpactEffects) {
        TESObjectCELL_PlaceParticleEffect(cell, impact->duration, impact->model.GetModelName(), hitNormal, hitPosition, 1.f, 7, nullptr);
    }
}


struct DoActorHitTask : TaskDelegate
{
    void DoActorHit(Character *hitChar, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, NiPointer<NiAVObject> hitNode, float impulseMult, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab)
    {
        PlayerCharacter *player = *g_thePlayer;

        SwingHandler &swingHandler = isLeft ? g_leftSwingHandler : g_rightSwingHandler;
        if (!swingHandler.IsSwingCoolingDown()) {
            // There was no recent swing, so perform one now
            swingHandler.Swing(true, isStab, &hitPosition);
        }

        if (swingHandler.didLastSwingFail) {
            PlayRumble(!isLeft, Config::options.swingFailedRumbleIntensity, Config::options.swingFailedRumbleDuration);
            return;
        }

        // We already figured out if we had enough stamina, and deducted stamina, in the swing, so no need to check it here
        bool isPowerAttack = swingHandler.wasLastSwingPowerAttack; // either the last recent swing, or the one we just performed
        bool isBash = swingHandler.wasLastSwingBash;

        BGSAttackData *attackData = nullptr;
        PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, isPowerAttack, &attackData);
        if (!attackData) return;

        // Hit position / velocity need to be set before Character::HitTarget() which at some point will read from them (during the HitData population)
        NiPoint3 *playerLastHitPosition = (NiPoint3 *)((UInt64)player + 0x6BC);
        *playerLastHitPosition = hitPosition;

        NiPoint3 *playerLastHitVelocity = (NiPoint3 *)((UInt64)player + 0x6C8);
        *playerLastHitVelocity = hitVelocity;

        HitActor(player, hitChar, hitNode, hitPosition, hitVelocity, isLeft, isOffhand, isPowerAttack, isBash);

        PlayMeleeImpactRumble(isTwoHanding ? 2 : isLeft);

        if (Config::options.applyImpulseOnHit) {
            if (isPowerAttack) {
                impulseMult *= Config::options.powerAttackImpulseMultiplier;
            }

            if (NiPointer<NiAVObject> root = hitChar->GetNiNode()) {
                if (NiPointer<bhkWorld> world = GetHavokWorldFromCell(player->parentCell)) {
                    if (DoesRefrHaveNode(hitChar, hitNode)) {
                        if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(hitNode)) {
                            ApplyHitImpulse(world, hitChar, rigidBody->hkBody, hitVelocity, hitPosition * *g_havokWorldScale, impulseMult);
                        }
                    }
                }
            }
        }
    }

    DoActorHitTask(UInt32 actorHandle, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, NiPointer<NiAVObject> hitNode, float impulseMult, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab)
        : hitActorHandle(actorHandle), hitPosition(hitPosition), hitVelocity(hitVelocity), hitNode(hitNode), impulseMult(impulseMult), isLeft(isLeft), isOffhand(isOffhand), isTwoHanding(isTwoHanding), isStab(isStab) {}

    static DoActorHitTask *Create(UInt32 actorHandle, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, NiPointer<NiAVObject> hitNode, float impulseMult, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab) {
        return new DoActorHitTask(actorHandle, hitPosition, hitVelocity, hitNode, impulseMult, isLeft, isOffhand, isTwoHanding, isStab);
    }

    virtual void Run() {
        NiPointer<TESObjectREFR> refr;
        if (LookupREFRByHandle(hitActorHandle, refr)) {
            if (Character *actor = DYNAMIC_CAST(refr, TESObjectREFR, Character)) {
                DoActorHit(actor, hitPosition, hitVelocity, hitNode, impulseMult, isLeft, isOffhand, isTwoHanding, isStab);
            }
        }
    }

    virtual void Dispose() {
        delete this;
    }

    UInt32 hitActorHandle;
    NiPoint3 hitPosition;
    NiPoint3 hitVelocity;
    NiPointer<NiAVObject> hitNode;
    float impulseMult;
    bool isLeft;
    bool isOffhand;
    bool isTwoHanding;
    bool isStab;
};


struct DoRefrHitTask : TaskDelegate
{
    void DoRefrHit(TESObjectREFR *hitRefr, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, BGSMaterialType *hitMaterial, TESForm *weapon, bool setCause, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab)
    {
        PlayerCharacter *player = *g_thePlayer;

        SwingHandler &swingHandler = isLeft ? g_leftSwingHandler : g_rightSwingHandler;
        if (!swingHandler.IsSwingCoolingDown()) {
            // There was no recent swing, so perform one now
            swingHandler.Swing(true, isStab, &hitPosition);
        }

        if (swingHandler.didLastSwingFail) {
            PlayRumble(!isLeft, Config::options.swingFailedRumbleIntensity, Config::options.swingFailedRumbleDuration);
            return;
        }

        // We already figured out if we had enough stamina, and deducted stamina, in the swing, so no need to check it here
        bool isPowerAttack = swingHandler.wasLastSwingPowerAttack; // either the last recent swing, or the one we just performed
        bool isBash = swingHandler.wasLastSwingBash;

        BGSAttackData *attackData = nullptr;
        PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, isPowerAttack, &attackData);

        NiPoint3 *playerLastHitPosition = (NiPoint3 *)((UInt64)player + 0x6BC);
        *playerLastHitPosition = hitPosition;

        NiPoint3 *playerLastHitVelocity = (NiPoint3 *)((UInt64)player + 0x6C8);
        *playerLastHitVelocity = hitVelocity;

        HitRefr(player, hitRefr, setCause, isLeft, isOffhand);

        BGSImpactData *impact = GetImpactData(hitMaterial, weapon, isBash);
        PlayImpactEffects(player->parentCell, impact, hitPosition, VectorNormalized(-hitVelocity));

        PlayMeleeImpactRumble(isTwoHanding ? 2 : isLeft);
    }

    DoRefrHitTask(UInt32 refrHandle, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, BGSMaterialType *hitMaterial, TESForm *weapon, bool setCause, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab)
        : hitRefrHandle(refrHandle), hitPosition(hitPosition), hitVelocity(hitVelocity), hitMaterial(hitMaterial), setCause(setCause), weapon(weapon), isLeft(isLeft), isOffhand(isOffhand), isTwoHanding(isTwoHanding), isStab(isStab) {}

    static DoRefrHitTask *Create(UInt32 actorHandle, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, BGSMaterialType *hitMaterial, TESForm *weapon, bool setCause, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab) {
        return new DoRefrHitTask(actorHandle, hitPosition, hitVelocity, hitMaterial, weapon, setCause, isLeft, isOffhand, isTwoHanding, isStab);
    }

    virtual void Run() {
        NiPointer<TESObjectREFR> refr;
        if (LookupREFRByHandle(hitRefrHandle, refr)) {
            DoRefrHit(refr, hitPosition, hitVelocity, hitMaterial, weapon, setCause, isLeft, isOffhand, isTwoHanding, isStab);
        }
    }

    virtual void Dispose() {
        delete this;
    }

    UInt32 hitRefrHandle;
    NiPoint3 hitPosition;
    NiPoint3 hitVelocity;
    BGSMaterialType *hitMaterial;
    TESForm *weapon;
    bool setCause;
    bool isLeft;
    bool isOffhand;
    bool isTwoHanding;
    bool isStab;
};


struct PotentiallyConvertBipedObjectToDeadBipTask : TaskDelegate
{
    PotentiallyConvertBipedObjectToDeadBipTask(UInt32 refrHandle, bhkRigidBody *rigidBody)
        : refrHandle(refrHandle), rigidBody(rigidBody) {}

    static PotentiallyConvertBipedObjectToDeadBipTask *Create(UInt32 refrHandle, bhkRigidBody *rigidBody) {
        return new PotentiallyConvertBipedObjectToDeadBipTask(refrHandle, rigidBody);
    }

    virtual void Run() {
        NiPointer<TESObjectREFR> refr;
        if (!LookupREFRByHandle(refrHandle, refr)) return;

        Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
        if (!actor) return;

        NiPointer<NiNode> root = refr->GetNiNode();
        hkpRigidBody *body = rigidBody->hkBody;
        if (!root || !body || !FindRigidBody(root, body)) return;

        bool isRagdollRigidBody = false;
        // This is a task because we acquire the animation graph manager lock in here, which can cause a deadlock in some cases
        ForEachRagdollDriver(actor, [this, body, &isRagdollRigidBody](hkbRagdollDriver *driver) {
            if (hkaRagdollInstance *ragdoll = driver->ragdoll) {
                if (ragdoll->m_rigidBodies.indexOf(body) >= 0) {
                    isRagdollRigidBody = true;
                }
            }
            });
        if (isRagdollRigidBody) return;

        body->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo &= ~(0x7f); // zero out layer
        body->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo |= (BGSCollisionLayer::kCollisionLayer_DeadBip & 0x7f); // set layer to the same as a dead ragdoll
        bhkWorldObject_UpdateCollisionFilter(rigidBody);
    }

    virtual void Dispose() {
        delete this;
    }

    UInt32 refrHandle;
    NiPointer<bhkRigidBody> rigidBody;
};


float g_savedMinSoundVel;
double g_restoreSoundVelTime = 0.0;

struct PhysicsListener :
    hkpContactListener,
    hkpWorldPostSimulationListener,
    hkpEntityListener
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
    std::unordered_set<TESObjectREFR *> collidedRefs[2]{};
    std::unordered_set<TESObjectREFR *> handCollidedRefs[2]{};

    struct CooldownData
    {
        double startTime = 0.0;
        double stoppedCollidingTime = 0.0;
    };
    std::unordered_map<TESObjectREFR *, CooldownData> hitCooldownTargets[2]{}; // each hand has its own cooldown
    std::unordered_map<TESObjectREFR *, double> collisionCooldownTargets[2]{};
    std::map<std::pair<Actor *, hkpRigidBody *>, double> physicsHitCooldownTargets{};
    std::vector<CollisionEvent> events{};

    std::unordered_map<TESObjectREFR *, double> physicalBlockTimes{};

    inline std::pair<hkpRigidBody *, hkpRigidBody *> SortPair(hkpRigidBody *a, hkpRigidBody *b) {
        if ((uint64_t)a <= (uint64_t)b) return { a, b };
        else return { b, a };
    }

    inline bool IsCollided(TESObjectREFR *refr)
    {
        return collidedRefs[0].count(refr) || collidedRefs[1].count(refr);
    }

    void DoHit(TESObjectREFR *hitRefr, hkpRigidBody *hitRigidBody, hkpRigidBody *hittingRigidBody, const hkpContactPointEvent &evnt, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity,
        TESForm *weapon, float impulseMult, bool isLeft, bool isOffhand, bool isTwoHanding, bool isStab)
    {
        PlayerCharacter *player = *g_thePlayer;
        if (hitRefr == player) return;

        hitCooldownTargets[isLeft][hitRefr] = { g_currentFrameTime, g_currentFrameTime };

        Character *hitChar = DYNAMIC_CAST(hitRefr, TESObjectREFR, Character);
        if (hitChar && !Actor_IsGhost(hitChar) && Character_CanHit(player, hitChar)) {
            evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;

            NiPointer<NiAVObject> hitNode = GetNodeFromCollidable(hitRigidBody->getCollidable());
            g_taskInterface->AddTask(DoActorHitTask::Create(GetOrCreateRefrHandle(hitChar), hitPosition, hitVelocity, hitNode, impulseMult, isLeft, isOffhand, isTwoHanding, isStab));
        }
        else {
            if (hittingRigidBody->getQualityType() == hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING && !IsMoveableEntity(hitRigidBody)) {
                // Disable contact for keyframed/fixed objects in this case
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
            }

            bool didDispatchHitEvent = DispatchHitEvents(player, hitRefr, hitRigidBody, weapon);

            BGSDestructibleObjectForm *destructible = TESForm_GetDestructibleObjectForm(hitRefr->baseForm);
            bool isDestructible = destructible && destructible->data;

            if (didDispatchHitEvent || isDestructible) {
                BGSMaterialType *material = GetImpactMaterial(hitRigidBody, evnt, hitPosition);
                g_taskInterface->AddTask(DoRefrHitTask::Create(GetOrCreateRefrHandle(hitRefr), hitPosition, hitVelocity, material, weapon, IsMoveableEntity(hitRigidBody), isLeft, isOffhand, isTwoHanding, isStab));
            }
        }
    }

    void ApplyPhysicsDamage(Actor *source, Actor *target, bhkRigidBody *collidingBody, NiPoint3 &hitPos, NiPoint3 &hitNormal, float minMass, float minSpeed)
    {
        bhkCharacterController::CollisionEvent collisionEvent{
            collidingBody,
            hitPos * *g_inverseHavokWorldScale,
            hitNormal * *g_inverseHavokWorldScale,
            HkVectorToNiPoint(collidingBody->hkBody->getLinearVelocity()) * *g_inverseHavokWorldScale
        };

        float massInv = collidingBody->hkBody->getMassInv();
        float mass = massInv <= 0.001f ? 99999.f : 1.f / massInv;
        float speed = VectorLength(collisionEvent.bodyVelocity);
        float damage = GetPhysicsDamage(mass, speed, minMass, minSpeed);

        _DMESSAGE("Physics damage: %.2f", damage);

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
                    physicsHitCooldownTargets[{ target, collidingBody->hkBody }] = g_currentFrameTime;
                }
            }
        }
    }

    virtual void contactPointCallback(const hkpContactPointEvent &evnt) override {
        if (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED ||
            !evnt.m_contactPointProperties->isPotential())
        {
            return;
        }

        hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
        hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

        UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
        UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;

        if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped && layerB != g_higgsCollisionLayer) ||
            (layerB == BGSCollisionLayer::kCollisionLayer_Biped && layerA != g_higgsCollisionLayer))
        {
            if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped && IsRagdollHandRigidBody(rigidBodyA)) || (layerB == BGSCollisionLayer::kCollisionLayer_Biped && IsRagdollHandRigidBody(rigidBodyB))) {
                // One of the collidees is a weapon rigidbody on the biped layer, and it didn't collide with a higgs rigidbody
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }
        }

        if ((layerA == BGSCollisionLayer::kCollisionLayer_CharController && (layerB == BGSCollisionLayer::kCollisionLayer_Clutter || layerB == BGSCollisionLayer::kCollisionLayer_Weapon)) ||
            (layerB == BGSCollisionLayer::kCollisionLayer_CharController && (layerA == BGSCollisionLayer::kCollisionLayer_Clutter || layerA == BGSCollisionLayer::kCollisionLayer_Weapon)))
        {
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
            ((layerB == BGSCollisionLayer::kCollisionLayer_Biped || layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC) && (layerA == BGSCollisionLayer::kCollisionLayer_Clutter || layerA == BGSCollisionLayer::kCollisionLayer_Weapon)))
        {
            if (NiPointer<TESObjectREFR> refrA = GetRefFromCollidable(&rigidBodyA->m_collidable)) {
                if (NiPointer<TESObjectREFR> refrB = GetRefFromCollidable(&rigidBodyB->m_collidable)) {
                    bool isATarget = refrA->formType == kFormType_Character;
                    Actor *actor = DYNAMIC_CAST(isATarget ? refrA : refrB, TESObjectREFR, Actor);
                    if (!actor) {
                        // Disable collision with biped objects that are not actors
                        evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                        return;
                    }

                    hkpRigidBody *otherBody = isATarget ? rigidBodyB : rigidBodyA;
                    if (Config::options.disableBipedClutterCollisionWithNonMoveableObjects && !IsMoveableEntity(otherBody) && g_activeActors.count(actor)) {
                        evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                        return;
                    }

                    if (Config::options.doClutterVsBipedCollisionDamage) {
                        hkpRigidBody *hittingBody = isATarget ? rigidBodyB : rigidBodyA;
                        if (!physicsHitCooldownTargets.count({ actor, hittingBody })) {
                            bhkRigidBody *collidingRigidBody = (bhkRigidBody *)hittingBody->m_userData;
                            // Different hit speed / mass thresholds for stuff thrown by the player vs random stuff colliding (for non-player thrown stuff, just use the base game's minimums)
                            Actor *aggressor = g_higgsLingeringRigidBodies.count(collidingRigidBody) ? *g_thePlayer : nullptr;
                            float minMass = aggressor ? Config::options.collisionDamageMinMassPlayerInflicted : *g_fPhysicsDamage1Mass;
                            float minSpeed = aggressor ? Config::options.collisionDamageMinSpeedPlayerInflicted : *g_fPhysicsDamageSpeedMin;
                            ApplyPhysicsDamage(aggressor, actor, collidingRigidBody, HkVectorToNiPoint(evnt.m_contactPoint->getPosition()), HkVectorToNiPoint(evnt.m_contactPoint->getNormal()), minMass, minSpeed);
                        }
                    }
                }
            }
        }

        if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped || layerA == BGSCollisionLayer::kCollisionLayer_BipedNoCC || layerA == BGSCollisionLayer::kCollisionLayer_DeadBip) &&
            (layerB == BGSCollisionLayer::kCollisionLayer_Biped || layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC || layerB == BGSCollisionLayer::kCollisionLayer_DeadBip))
        {
            if (Config::options.overrideSoundVelForRagdollCollisions) {
                // Disable collision sounds for this frame
                *g_fMinSoundVel = Config::options.ragdollSoundVel;
            }

            UInt16 groupA = rigidBodyA->getCollidable()->m_broadPhaseHandle.m_collisionFilterInfo >> 16;
            UInt16 groupB = rigidBodyB->getCollidable()->m_broadPhaseHandle.m_collisionFilterInfo >> 16;

            if (groupA != groupB) {
                if (NiPointer<TESObjectREFR> refrA = GetRefFromCollidable(&rigidBodyA->m_collidable)) {
                    if (NiPointer<TESObjectREFR> refrB = GetRefFromCollidable(&rigidBodyB->m_collidable)) {
                        if (refrA != refrB) {
                            if (g_interface001.IsRagdollCollisionIgnored(refrA) || g_interface001.IsRagdollCollisionIgnored(refrB)) {
                                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                                return;
                            }

                            if (Config::options.stopRagdollNonSelfCollisionForCloseActors) {
                                if (VectorLength(refrA->pos - refrB->pos) < Config::options.closeActorMinDistance) {
                                    // Disable collision between bipeds whose references are roughly in the same position
                                    evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                                    return;
                                }
                            }

                            if (Config::options.stopRagdollNonSelfCollisionForActorsWithVehicle) {
                                if (Actor *actorA = DYNAMIC_CAST(refrA, TESObjectREFR, Actor)) {
                                    if (Actor *actorB = DYNAMIC_CAST(refrB, TESObjectREFR, Actor)) {
                                        if (GetVehicleHandle(actorA) != *g_invalidRefHandle && GetVehicleHandle(actorB) != *g_invalidRefHandle) {
                                            evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                                            return;
                                        }
                                    }
                                }
                            }
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

        PlayerCharacter *player = *g_thePlayer;
        if (Actor_IsInRagdollState(player) || IsSwimming(player) || IsStaggered(player)) {
            evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
            return;
        }

        UInt32 hitLayer = hitRigidBody == rigidBodyA ? layerA : layerB;

        if (hitLayer == BGSCollisionLayer::kCollisionLayer_Biped && IsRagdollHandRigidBody(hitRigidBody) && IsPlayerWeaponRigidBody(hittingRigidBody)) {
            // Physical block
            physicalBlockTimes[hitRefr] = g_currentFrameTime;
            return;
        }

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
        TESForm *equippedObj = player->GetEquippedObject(isOffhand);
        TESObjectWEAP *weap = DYNAMIC_CAST(equippedObj, TESForm, TESObjectWEAP);

        NiPoint3 handDirection = VectorNormalized(g_controllerData[isLeft].avgVelocity);
        float handSpeedRoomspace = g_controllerData[isLeft].avgSpeed;
        bool isTwoHanding = g_higgsInterface->IsTwoHanding();
        if (isTwoHanding) {
            handDirection = VectorNormalized(g_controllerData[0].avgVelocity + g_controllerData[1].avgVelocity);
            handSpeedRoomspace = max(g_controllerData[0].avgSpeed, g_controllerData[1].avgSpeed);
        }

        bool isStab = false, isPunch = false, isSwing = false;
        if (weap && CanWeaponStab(weap)) {
            // Check for stab
            // All stabbable weapons use the melee weapon offset node
            if (NiPointer<NiAVObject> weaponOffsetNode = player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftMeleeWeaponOffsetNode : PlayerCharacter::Node::kNode_RightMeleeWeaponOffsetNode]) {
                // Use last frame's offset node transform because this frame is not over yet and it can still be modified by e.g. higgs two-handing
                NiPoint3 weaponForward = ForwardVector(weaponOffsetNode->m_oldWorldTransform.rot);
                float stabAmount = DotProduct(handDirection, weaponForward);
                _DMESSAGE("Stab amount: %.2f", stabAmount);
                if (stabAmount > Config::options.hitStabDirectionThreshold && hitSpeed > Config::options.hitStabSpeedThreshold) {
                    isStab = true;
                }
            }
        }
        else if (!equippedObj || (weap && weap->type() == TESObjectWEAP::GameData::kType_HandToHandMelee)) {
            // Check for punch
            // For punching use the hand node
            if (NiPointer<NiAVObject> handNode = player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftHandBone : PlayerCharacter::Node::kNode_RightHandBone]) {
                NiPoint3 punchVector = UpVector(handNode->m_worldTransform.rot); // in the direction of fingers when fingers are extended
                float punchAmount = DotProduct(handDirection, punchVector);
                _DMESSAGE("Punch amount: %.2f", punchAmount);
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
                hitVelocity = (handDirection * handSpeedRoomspace / *g_globalTimeMultiplier) / havokWorldScale;
            }
            else {
                hitVelocity = hkHitVelocity / havokWorldScale;
            }

            float impulseMult = isStab ? Config::options.hitStabImpulseMult : (isPunch ? Config::options.hitPunchImpulseMult : Config::options.hitSwingImpulseMult);
            DoHit(hitRefr, hitRigidBody, hittingRigidBody, evnt, hitPosition, hitVelocity, equippedObj, impulseMult, isLeft, isOffhand, isTwoHanding, isStab);
        }
        else if (hitRefr->formType == kFormType_Character && doHit && disableHit) {
            // Hit is disabled and we hit a character. Disable this contact point but don't disable future ones.
            evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
            return;
        }
        else {
            // It's not a hit

            if (hittingRigidBody->getQualityType() == hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING && !IsMoveableEntity(hitRigidBody)) {
                // Disable contact for keyframed / fixed objects in this case
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }

            if (hitLayer == BGSCollisionLayer::kCollisionLayer_CharController) {
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }

            if (collisionCooldownTargets[isLeft].count(hitRefr)) {
                // refr shouldn't be collided with, but should still be able to be hit (so all code above still runs)
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }
        }
    }

    virtual void collisionAddedCallback(const hkpCollisionEvent &evnt) override
    {
        hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
        hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

        UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
        UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
        if (layerA != g_higgsCollisionLayer && layerB != g_higgsCollisionLayer) return; // Every collision we care about involves a body on the higgs layer (hand, held object...)

        if (layerA == g_higgsCollisionLayer && layerB == g_higgsCollisionLayer) return; // Both objects are on the higgs layer

        events.push_back({ rigidBodyA, rigidBodyB, CollisionEvent::Type::Added });
        //_DMESSAGE("%d Added %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
    }

    virtual void collisionRemovedCallback(const hkpCollisionEvent &evnt) override
    {
        hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
        hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

        // Technically our objects could have changed layers or something between added and removed
        events.push_back({ rigidBodyA, rigidBodyB, CollisionEvent::Type::Removed });
        //_DMESSAGE("%d Removed %x %x", *g_currentFrameCounter, (UInt64)rigidBodyA, (UInt64)rigidBodyB);
    }

    virtual void postSimulationCallback(hkpWorld *world) override
    {
        // Restore the game's original value for fMinSoundVel after any contact callbacks would have been called.
        if (g_currentFrameTime >= g_restoreSoundVelTime) {
            *g_fMinSoundVel = g_savedMinSoundVel;
        }

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
        for (int isLeft = 0; isLeft < 2; ++isLeft) {
            collidedRefs[isLeft].clear();
            handCollidedRefs[isLeft].clear();
        }
        for (auto it = activeCollisions.begin(); it != activeCollisions.end();) {
            auto [pair, count] = *it;
            if (count <= 0) {
                it = activeCollisions.erase(it);
            }
            else {
                ++it;
            }
        }

        // Now fill in the currently collided-with actors based on active collisions
        for (auto [pair, count] : activeCollisions) {
            auto [bodyA, bodyB] = pair;
            hkpRigidBody *collidedBody = IsHiggsRigidBody(bodyA) ? bodyB : bodyA;
            collidedRigidbodies.insert(collidedBody);

            if (TESObjectREFR *collidedRef = GetRefFromCollidable(collidedBody->getCollidable())) {
                hkpRigidBody *collidingBody = collidedBody == bodyA ? bodyB : bodyA;
                bool isLeft = IsLeftRigidBody(collidingBody);

                collidedRefs[isLeft].insert(collidedRef);
                if (IsPlayerHandRigidBody(collidingBody)) {
                    handCollidedRefs[isLeft].insert(collidedRef);
                }

                if (hitCooldownTargets[isLeft].count(collidedRef)) {
                    // refr is still collided with, so refresh its hit cooldown
                    hitCooldownTargets[isLeft][collidedRef].stoppedCollidingTime = g_currentFrameTime;
                }

                if (GetCollisionLayer(collidedBody) == BGSCollisionLayer::kCollisionLayer_Biped && IsRagdollHandRigidBody(collidedBody) && IsPlayerWeaponRigidBody(collidingBody)) {
                    physicalBlockTimes[collidedRef] = g_currentFrameTime;
                }
            }
        }

        // Clear out old hit cooldown targets
        for (auto &targets : hitCooldownTargets) { // For each hand's cooldown targets
            for (auto it = targets.begin(); it != targets.end();) {
                auto [target, cooldown] = *it;
                if ((g_currentFrameTime - cooldown.stoppedCollidingTime) > Config::options.hitCooldownTimeStoppedColliding ||
                    (g_currentFrameTime - cooldown.startTime) > Config::options.hitCooldownTimeFallback)
                    it = targets.erase(it);
                else
                    ++it;
            }
        }

        // Clear out old physics hit cooldown targets
        for (auto it = physicsHitCooldownTargets.begin(); it != physicsHitCooldownTargets.end();) {
            auto [target, hitTime] = *it;
            if ((g_currentFrameTime - hitTime) * *g_globalTimeMultiplier > Config::options.physicsHitRecoveryTime)
                it = physicsHitCooldownTargets.erase(it);
            else
                ++it;
        }

        // Clear out old collision cooldown targets
        for (auto &targets : collisionCooldownTargets) { // For each hand's cooldown targets
            for (auto it = targets.begin(); it != targets.end();) {
                auto [target, disabledTime] = *it;
                if ((g_currentFrameTime - disabledTime) > Config::options.collisionCooldownTime)
                    it = targets.erase(it);
                else
                    ++it;
            }
        }
    }

    virtual void entityAddedCallback(hkpEntity *entity) override {
        if (!Config::options.convertNonRagdollBipedObjectsToDeadBip) return;

        hkpRigidBody *rigidBody = hkpGetRigidBody(entity->getCollidable());
        if (!rigidBody) return;

        UInt32 layer = rigidBody->getCollisionFilterInfo() & 0x7f;
        if (layer != BGSCollisionLayer::kCollisionLayer_Biped) return;

        // Make sure it's a standalone object that belongs to a node (has a collision object) and isn't the rigidbody of the character controller
        if (!rigidBody->hasProperty((hkUint32)HavokProperty::CollisionObject) || rigidBody->hasProperty((hkUint32)HavokProperty::CharacterController)) return;

        NiPointer<TESObjectREFR> refr = GetRefFromCollidable(rigidBody->getCollidable());
        if (!refr) return;

        if (refr->formType != kFormType_Character) return;

        bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
        if (!wrapper) return;

        g_taskInterface->AddTask(PotentiallyConvertBipedObjectToDeadBipTask::Create(GetOrCreateRefrHandle(refr), wrapper));
    }

    NiPointer<bhkWorld> world = nullptr;
};
PhysicsListener g_physicsListener{};

struct PlayerCharacterProxyListener : hkpCharacterProxyListener
{
    // Called when the character interacts with another (non fixed or keyframed) rigid body.
    virtual void objectInteractionCallback(hkpCharacterProxy *proxy, const hkpCharacterObjectInteractionEvent &input, hkpCharacterObjectInteractionResult &output)
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

        output.m_objectImpulse = NiPointToHkVector(HkVectorToNiPoint(output.m_objectImpulse) * Config::options.playerVsBipedInteractionImpulseMultiplier);

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
        else {
            // Biped vs. another biped
            if (Config::options.doBipedNonSelfCollision) {
                return CollisionFilterComparisonResult::Continue;
            }
            else {
                return CollisionFilterComparisonResult::Ignore;
            }
        }
    }

    if (layerA == BGSCollisionLayer::kCollisionLayer_Ground || layerB == BGSCollisionLayer::kCollisionLayer_Ground) {
        // If the layer is the ground layer, and the group is the player group, then this is actually the linear cast that checks for footsteps,
        // and we handle this the same way as if it was the player controller
        UInt16 groupA = filterInfoA >> 16;
        UInt16 groupB = filterInfoB >> 16;
        if (layerA == BGSCollisionLayer::kCollisionLayer_Ground && groupA == g_playerCollisionGroup) {
            filterInfoA &= ~(0x7f); // zero out layer
            filterInfoA |= (BGSCollisionLayer::kCollisionLayer_CharController & 0x7f); // set layer to charcontroller for the duration of this function
            layerA = filterInfoA & 0x7f;
        }
        if (layerB == BGSCollisionLayer::kCollisionLayer_Ground && groupB == g_playerCollisionGroup) {
            filterInfoB &= ~(0x7f); // zero out layer
            filterInfoB |= (BGSCollisionLayer::kCollisionLayer_CharController & 0x7f); // set layer to charcontroller for the duration of this function
            layerB = filterInfoB & 0x7f;
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
            UInt16 otherGroup = groupA == g_playerCollisionGroup ? groupB : groupA;
            if (g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(otherGroup)) {
                // Still collide the player with hittable character controllers
                return CollisionFilterComparisonResult::Continue;
            }
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
                (g_rightHeldActorCollisionGroup && otherGroup == g_rightHeldActorCollisionGroup) ||
                (g_leftHeldActorCollisionGroup && otherGroup == g_leftHeldActorCollisionGroup)) {
                return CollisionFilterComparisonResult::Ignore;
            }

            if (!g_activeBipedGroups.count(otherGroup)) {
                // Disable collision with biped objects that are not actors
                return CollisionFilterComparisonResult::Ignore;
            }

            if (IsRagdollHandFilter(otherFilter)) {
                // Disable collision between the player capsule and ragdoll hands (weapons)
                return CollisionFilterComparisonResult::Ignore;
            }

            if (g_noPlayerCharControllerCollideGroups.count(otherGroup)) {
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


bool g_isRightHandAggressivelyPositioned = false;
bool g_isLeftHandAggressivelyPositioned = false;
bool g_isMenuOpen = false;

struct NPCData
{
    enum class State
    {
        Normal,
        SomewhatMiffed,
        VeryMiffed,
        Assaulted,
        Hostile,
    };

    State state = State::Normal;
    double dialogueTime = 0.0;
    double bumpTime = 0.0;
    double lastGrabbedTouchedTime = 0.0;
    float accumulatedGrabbedTime = 0.f;
    TESTopic *lastSaidTopic = nullptr;
    UInt32 lastSaidTopicID = 0;
    UInt32 secondLastSaidTopicID = 0;
    float lastSaidDialogueDuration = -1.f;
    float lastVoiceTimer = -1.f;
    bool isSpeaking = false;

    void TriggerDialogue(Character *character, TESTopic *topic, TESTopicInfo *topicInfo)
    {
        Actor_SayToEx(character, *g_thePlayer, topic, topicInfo);

        if (topicInfo) {
            secondLastSaidTopicID = lastSaidTopicID;
            lastSaidTopicID = topicInfo->formID;
        }

        lastSaidTopic = topic;
    }

    void TryTriggerDialogue(Character *character, bool high, bool isShoved)
    {
        if (Actor_IsInRagdollState(character) || IsSleeping(character)) return;

        if (isShoved) {
            if (TESTopicInfo *topicInfo = GetRandomTopicInfo(Config::options.shoveTopicInfos, lastSaidTopicID, secondLastSaidTopicID)) {
                TriggerDialogue(character, (TESTopic *)topicInfo->unk14, topicInfo);
            }
            dialogueTime = g_currentFrameTime;
            return;
        }

        if (isSpeaking || g_currentFrameTime - dialogueTime <= Config::options.aggressionDialogueInitMaxTime) return;

        {
            std::scoped_lock lock(g_interface001.aggressionTopicsLock);
            std::unordered_map<Actor *, TESTopic *> &topics = high ? g_interface001.highAggressionTopics : g_interface001.lowAggressionTopics;
            if (auto it = topics.find(character); it != topics.end()) {
                // First, check if this specific actor has an assigned topic
                TriggerDialogue(character, it->second, nullptr);
                dialogueTime = g_currentFrameTime;
                return;
            }
            else if (auto it = topics.find(nullptr); it != topics.end()) {
                // If the specific actor does not have an assigned topic, see if there is a general topic set for all actors
                TriggerDialogue(character, it->second, nullptr);
                dialogueTime = g_currentFrameTime;
                return;
            }
        }

        // There are no assigned topics, so use our topic info sets
        std::vector<UInt32> &topicInfoIDs = high ? Config::options.aggressionHighTopicInfos : Config::options.aggressionLowTopicInfos;
        if (TESTopicInfo *topicInfo = GetRandomTopicInfo(topicInfoIDs, lastSaidTopicID, secondLastSaidTopicID)) {
            TriggerDialogue(character, (TESTopic *)topicInfo->unk14, topicInfo);
        }

        dialogueTime = g_currentFrameTime;
    }

    void TryBump(Character *character, bool exitFurniture, bool force = false)
    {
        if (Actor_IsInRagdollState(character)) return;

        if (force || g_currentFrameTime - bumpTime > Config::options.aggressionBumpCooldownTime) {
            QueueBumpActor(character, character->pos - (*g_thePlayer)->pos, false, exitFurniture, false, false);
            bumpTime = g_currentFrameTime;
        }
    }

    void StateUpdate(Character *character, bool isShoved)
    {
        if (character->IsDead(1)) return;
        if (Config::options.summonsSkipAggression && GetCommandingActor(character) == *g_playerHandle) return;
        if ((Config::options.followersSkipAggression && IsTeammate(character)) ||
            (RelationshipRanks::GetRelationshipRank(character->baseForm, (*g_thePlayer)->baseForm) > Config::options.aggressionMaxRelationshipRank))
        {
            // Still do some dialogue when shoved even if it won't grow aggression for them
            if (isShoved && !Actor_IsInRagdollState(character) && !IsSleeping(character)) {
                if (TESTopicInfo *topicInfo = GetRandomTopicInfo(Config::options.shoveTopicInfos, lastSaidTopicID, secondLastSaidTopicID)) {
                    TriggerDialogue(character, (TESTopic *)topicInfo->unk14, topicInfo);
                }
            }
            return;
        }

        TESTopic *currentTopic = GetCurrentTopic(character);
        float voiceTimer = character->unk108;
        float dialogueCooldown = isSpeaking ? lastSaidDialogueDuration + Config::options.aggressionDialogueCooldown : Config::options.aggressionDialogueCooldownFallback;

        if (!isSpeaking && currentTopic == lastSaidTopic && voiceTimer != -1.f) {
            // Just started speaking after us making the actor speak
            lastSaidDialogueDuration = voiceTimer;
            isSpeaking = true;
        }
        else if (isSpeaking && (voiceTimer == -1.f || g_currentFrameTime - dialogueTime > dialogueCooldown || currentTopic != lastSaidTopic)) {
            // Just stopped speaking or started saying something else
            isSpeaking = false;
        }

        float deltaTime = *g_deltaTime;

        bool isGrabbed = g_leftHeldRefr == character || g_rightHeldRefr == character;
        bool isTouchedRight = g_physicsListener.collidedRefs[0].count(character) && g_isRightHandAggressivelyPositioned;
        bool isTouchedLeft = g_physicsListener.collidedRefs[1].count(character) && g_isLeftHandAggressivelyPositioned;
        bool isInteractedWith = isGrabbed || isTouchedLeft || isTouchedRight || isShoved;

        PlayerCharacter *player = *g_thePlayer;

        // These two are to not do aggression if they are in... certain scenes...
        bool sharesPlayerPosition = Config::options.stopAggressionForCloseActors && VectorLength(character->pos - player->pos) < Config::options.closeActorMinDistance;
        bool isInVehicle = Config::options.stopAggressionForActorsWithVehicle && GetVehicleHandle(character) != *g_invalidRefHandle;
        bool isSpecial = sharesPlayerPosition || isInVehicle;

        bool canPlayerAggress = !Actor_IsInRagdollState(player) && !IsSwimming(player) && !IsStaggered(player) && (!Config::options.dontDoAggressionWhileMenusAreOpen || !g_isMenuOpen);
        bool isCalmed = Config::options.calmedActorsDontAccumulateAggression && IsCalmed(character);

        bool isAggressivelyInteractedWith = isInteractedWith && canPlayerAggress && !isSpecial && !isCalmed;

        if (isAggressivelyInteractedWith) {
            accumulatedGrabbedTime += isShoved ? Config::options.shoveAggressionImpact : deltaTime;
            lastGrabbedTouchedTime = g_currentFrameTime;
        }
        else if (g_currentFrameTime - lastGrabbedTouchedTime >= Config::options.aggressionStopDelay) {
            accumulatedGrabbedTime -= deltaTime;
        }
        accumulatedGrabbedTime = std::clamp(accumulatedGrabbedTime, 0.f, Config::options.aggressionMaxAccumulatedGrabTime);

        bool isHostile = Actor_IsHostileToActor(character, player);

        if (state == State::Normal) {
            if (isHostile) {
                state = State::Hostile;
            }
            else if (accumulatedGrabbedTime > Config::options.aggressionRequiredGrabTimeLow) {
                state = State::SomewhatMiffed;
            }
        }

        if (state == State::SomewhatMiffed) {
            if (isHostile) {
                state = State::Hostile;
            }
            else if (accumulatedGrabbedTime <= Config::options.aggressionRequiredGrabTimeLow) {
                state = State::Normal;
            }
            else if (accumulatedGrabbedTime > Config::options.aggressionRequiredGrabTimeHigh) {
                state = State::VeryMiffed;
            }
            else if (isAggressivelyInteractedWith) {
                // Constantly try to say something
                TryTriggerDialogue(character, false, isShoved);
                if (ShouldBumpActor(character) && !IsActorUsingFurniture(character)) {
                    TryBump(character, false);
                }
            }
        }

        if (state == State::VeryMiffed) {
            if (isHostile) {
                state = State::Hostile;
            }
            else if (accumulatedGrabbedTime <= Config::options.aggressionRequiredGrabTimeHigh) {
                state = State::SomewhatMiffed;
            }
            else if (accumulatedGrabbedTime > Config::options.aggressionRequiredGrabTimeAssault) {
                Actor_SendAssaultAlarm(0, 0, character);
                isHostile = Actor_IsHostileToActor(character, player); // need to update this after assaulting the actor
                if (isHostile) {
                    state = State::Assaulted;
                }
            }
            else if (isAggressivelyInteractedWith) {
                // Constantly try to say something
                TryTriggerDialogue(character, true, isShoved);
                if (ShouldBumpActor(character)) {
                    TryBump(character, Config::options.stopUsingFurnitureOnHighAggression);
                }
            }
        }

        if (state == State::Hostile) {
            if (!isHostile) {
                accumulatedGrabbedTime = 0.f;
                state = State::Normal;
            }
        }

        if (state == State::Assaulted) {
            if (!isHostile) {
                accumulatedGrabbedTime = 0.f;
                state = State::Normal;
            }
            else if (VectorLength(character->pos - player->pos) >= Config::options.aggressionStopCombatAlarmDistance) {
                // We're far enough away from the assaulted actor so make them forgive us
                Actor_StopCombatAlarm(0, 0, player);
                accumulatedGrabbedTime = 0.f;
                state = State::Normal;
            }
        }

        lastVoiceTimer = voiceTimer;
    }
};

std::unordered_map<Actor *, NPCData> g_npcs{};

void TryUpdateNPCState(Actor *actor, bool isShoved)
{
    if (!Config::options.doAggression) return;

    auto it = g_npcs.find(actor);
    if (it == g_npcs.end()) {
        // Not in the map yet
        TESRace *race = actor->race;
        if (!race) return;
        if (!race->keyword.HasKeyword(g_keyword_actorTypeNPC)) return;
        if (race->editorId && Config::options.aggressionExcludeRaces.count(std::string_view(race->editorId))) return;

        g_npcs[actor] = NPCData{};
    }
    else if (Character *character = DYNAMIC_CAST(actor, Actor, Character)) {
        NPCData &data = it->second;
        data.StateUpdate(character, isShoved);
    }
}

std::unordered_map<hkbRagdollDriver *, std::shared_ptr<ActiveRagdoll>> g_activeRagdolls{};
std::unordered_map<Actor *, std::shared_ptr<ActorData>> g_actorData{};

std::shared_ptr<ActiveRagdoll> GetActiveRagdollFromDriver(hkbRagdollDriver *driver)
{
    auto it = g_activeRagdolls.find(driver);
    if (it == g_activeRagdolls.end()) return nullptr;
    return it->second;
}

std::shared_ptr<ActorData> GetActorData(Actor *actor)
{
    auto it = g_actorData.find(actor);
    if (it == g_actorData.end()) return nullptr;
    return it->second;
}

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
    if (TESRace *race = actor->race) {
        const char *name = race->editorId;
        if (name && Config::options.excludeRaces.count(std::string_view(name))) {
            return false;
        }
    }

    {
        std::scoped_lock lock(g_interface001.ignoredActorsLock);
        if (g_interface001.ignoredActors.count(actor)) return false;
    }

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
            if (NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable)) {
                if (NiPointer<bhkRigidBody> wrapper = GetRigidBody(node)) {
                    bhkRigidBody_setMotionType(wrapper, hkpMotion::MotionType::MOTION_DYNAMIC);

                    hkRealTohkUFloat8(rigidBody->getRigidMotion()->getMotionState()->m_maxLinearVelocity, Config::options.ragdollBoneMaxLinearVelocity);
                    hkRealTohkUFloat8(rigidBody->getRigidMotion()->getMotionState()->m_maxAngularVelocity, Config::options.ragdollBoneMaxAngularVelocity);
                }
            }
        }

        if (Config::options.convertHingeConstraintsToRagdollConstraints) {
            // Convert any limited hinge constraints to ragdoll constraints so that they can be loosened properly
            for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
                if (NiPointer<NiAVObject> node = GetNodeFromCollidable(&rigidBody->m_collidable)) {
                    if (NiPointer<bhkRigidBody> wrapper = GetRigidBody(node)) {
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

struct hkInt16PairHash {
    std::size_t operator()(const std::pair<hkInt16, hkInt16> &pair) const {
        return std::hash<hkInt16>()(pair.first) ^ std::hash<hkInt16>()(pair.second);
    }
};

void SynchronizeAndFixupRagdollAndAnimSkeletonMappers(hkbRagdollDriver *driver)
{
    hkbCharacter *character = driver->character;
    if (!character) return;

    hkbCharacterSetup *setup = character->setup;
    if (!setup) return;

    hkaSkeletonMapper *animToRagdollMapper = setup->m_animationToRagdollSkeletonMapper;
    if (!animToRagdollMapper) return;

    hkaSkeletonMapper *ragdollToAnimMapper = setup->m_ragdollToAnimationSkeletonMapper;
    if (!ragdollToAnimMapper) return;

    if (animToRagdollMapper->m_mapping.m_simpleMappings.m_size != ragdollToAnimMapper->m_mapping.m_simpleMappings.m_size) {
        const char *nameStr = "";
        if (Actor *actor = GetActorFromCharacter(character)) {
            if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
                nameStr = name->name.data;
            }
        }
        _WARNING("Warning: '%s' mapping size mismatch between animToRagdoll and ragdollToAnim mappers (%d != %d)", nameStr, animToRagdollMapper->m_mapping.m_simpleMappings.m_size, ragdollToAnimMapper->m_mapping.m_simpleMappings.m_size);
    }

    // Create a set of pairs of bones that are mapped to each other for both the animation and ragdoll skeleton mappers

    std::unordered_map<std::pair<hkInt16, hkInt16>, hkQsTransform, hkInt16PairHash> mappedBonesAnimToRagdoll; // anim, ragdoll

    for (hkaSkeletonMapperData::SimpleMapping &mapping : animToRagdollMapper->m_mapping.m_simpleMappings) {
        std::pair<hkInt16, hkInt16> pair(mapping.m_boneA, mapping.m_boneB);
        auto it = mappedBonesAnimToRagdoll.find(pair);
        if (it != mappedBonesAnimToRagdoll.end()) {
            _WARNING("animtoragdoll: duplicate mapping for bones %d and %d", mapping.m_boneA, mapping.m_boneB);
        }
        else {
            mappedBonesAnimToRagdoll[pair] = mapping.m_aFromBTransform;
        }
    }

    std::unordered_map<std::pair<hkInt16, hkInt16>, hkQsTransform, hkInt16PairHash> mappedBonesRagdollToAnim; // anim, ragdoll

    for (hkaSkeletonMapperData::SimpleMapping &mapping : ragdollToAnimMapper->m_mapping.m_simpleMappings) {
        std::pair<hkInt16, hkInt16> pair(mapping.m_boneB, mapping.m_boneA);
        auto it = mappedBonesRagdollToAnim.find(pair);
        if (it != mappedBonesRagdollToAnim.end()) {
            _WARNING("ragdolltoanim: duplicate mapping for bones %d and %d", mapping.m_boneA, mapping.m_boneB);
        }
        else {
            mappedBonesRagdollToAnim[pair] = mapping.m_aFromBTransform;
        }
    }

    // Now compute the intersection of the two sets of mapped bones

    std::unordered_set<std::pair<hkInt16, hkInt16>, hkInt16PairHash> mappedBones;

    for (auto it = mappedBonesAnimToRagdoll.begin(); it != mappedBonesAnimToRagdoll.end(); it++) {
        if (mappedBonesRagdollToAnim.find((*it).first) != mappedBonesRagdollToAnim.end()) {
            mappedBones.insert((*it).first);
        }
    }

    // Now replace the mappings in the skeleton mappers with the intersection

    int i = 0;
    for (auto it = mappedBones.begin(); it != mappedBones.end(); it++) {
        std::pair<hkInt16, hkInt16> pair = *it;

        {
            hkaSkeletonMapperData::SimpleMapping &mapping = animToRagdollMapper->m_mapping.m_simpleMappings[i];
            mapping.m_boneA = pair.first;
            mapping.m_boneB = pair.second;
            mapping.m_aFromBTransform = mappedBonesAnimToRagdoll[pair];
        }

        {
            hkaSkeletonMapperData::SimpleMapping &mapping = ragdollToAnimMapper->m_mapping.m_simpleMappings[i];
            mapping.m_boneA = pair.second;
            mapping.m_boneB = pair.first;
            mapping.m_aFromBTransform = mappedBonesRagdollToAnim[pair];
        }

        i++;
    }

    // The new size will be <= the old size, so we know the capacity is sufficient
    animToRagdollMapper->m_mapping.m_simpleMappings.m_size = mappedBones.size();
    ragdollToAnimMapper->m_mapping.m_simpleMappings.m_size = mappedBones.size();
}

bool AddRagdollToWorld(Actor *actor)
{
    if (Actor_IsInRagdollState(actor)) return false;

    bool hasRagdollInterface = false;
    BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
    if (GetAnimationGraphManager(actor, animGraphManager)) {
        BSAnimationGraphManager_HasRagdoll(animGraphManager.ptr, &hasRagdollInterface);
    }

    if (hasRagdollInterface) {
        if (GetAnimationGraphManager(actor, animGraphManager)) {
            BSAnimationGraphManager *manager = animGraphManager.ptr;

            TESObjectCELL *parentCell = actor->parentCell;

            {
                SimpleLocker lock(&manager->updateLock);
                for (int i = 0; i < manager->graphs.size; i++) {
                    BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
                    if (hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver) {
                        g_activeActors.insert(actor);

                        std::shared_ptr<ActiveRagdoll> activeRagdoll(new ActiveRagdoll());
                        g_activeRagdolls[driver] = activeRagdoll;

                        if (Config::options.blendInWhenAddingToWorld) {
                            Blender &blender = activeRagdoll->blender;
                            blender.StartBlend(Blender::BlendType::AnimToRagdoll, g_currentFrameTime, Config::options.blendInTime);

                            hkQsTransform *poseLocal = hkbCharacter_getPoseLocal(driver->character);
                            blender.initialPose.assign(poseLocal, poseLocal + driver->character->numPoseLocal);
                            blender.isFirstBlendFrame = false;
                        }

                        activeRagdoll->stateChangedTime = g_currentFrameTime;
                        activeRagdoll->state = RagdollState::BlendIn;

                        if (!graph.ptr->world && parentCell) {
                            // World must be set before calling BShkbAnimationGraph::AddRagdollToWorld(), and is required for the graph to register its physics step listener (and hence call hkbRagdollDriver::driveToPose())
                            graph.ptr->world = GetHavokWorldFromCell(parentCell);
                            activeRagdoll->shouldNullOutWorldWhenRemovingFromWorld = true;
                        }

                        if (Config::options.fixupSkeletonMappings) {
                            SynchronizeAndFixupRagdollAndAnimSkeletonMappers(driver);
                        }
                    }
                }
            }

            if (parentCell) {
                if (NiPointer<bhkWorld> world = GetHavokWorldFromCell(parentCell)) {
#ifdef _DEBUG
                    if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
                        _MESSAGE("%d %s: Add ragdoll to world", *g_currentFrameCounter, name->name);
                    }
#endif // _DEBUG

                    BSWriteLocker lock(&world->worldLock);

                    bool x = false;
                    BSAnimationGraphManager_AddRagdollToWorld(animGraphManager.ptr, &x);

                    ModifyConstraints(actor);

                    x = false;
                    BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints(animGraphManager.ptr, &x);
                }
            }
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
        BSAnimationGraphManager_HasRagdoll(animGraphManager.ptr, &hasRagdollInterface);
    }

    if (hasRagdollInterface) {
        // TODO: We should not remove the ragdoll from the world if it had the ragdoll added already when we added it (e.g. race allowragdollcollision flag).
        //       In that case we should also revert the motion type to keyframed since that's what it usually is in this scenario.

#ifdef _DEBUG
        if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
            _MESSAGE("%d %s: Remove ragdoll from world", *g_currentFrameCounter, name->name);
        }
#endif // _DEBUG

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
                        if (std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver)) {
                            if (ragdoll && ragdoll->shouldNullOutWorldWhenRemovingFromWorld) {
                                graph.ptr->world = nullptr;
                            }
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
        BSAnimationGraphManager_HasRagdoll(animGraphManager.ptr, &hasRagdollInterface);
    }

    if (hasRagdollInterface) {
        bool x[2] = { false, true };
        BSAnimationGraphManager_DisableOrEnableSyncOnUpdate(animGraphManager.ptr, x);
    }
}

void EnableGravity(Actor *actor)
{
    bool hasRagdollInterface = false;
    BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
    if (GetAnimationGraphManager(actor, animGraphManager)) {
        BSAnimationGraphManager_HasRagdoll(animGraphManager.ptr, &hasRagdollInterface);
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

                                    for (hkpRigidBody *rigidBody : ragdoll->m_rigidBodies) {
                                        rigidBody->setGravityFactor(1.f);
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
    static KeepOffsetTask *Create(UInt32 source, UInt32 target, NiPoint3 &offset = NiPoint3(), NiPoint3 &offsetAngle = NiPoint3(), float catchUpRadius = 150.f, float followRadius = 50.f) {
        KeepOffsetTask *cmd = new KeepOffsetTask;
        if (cmd) {
            cmd->source = source;
            cmd->target = target;
            cmd->offset = offset;
            cmd->offsetAngle = offsetAngle;
            cmd->catchUpRadius = catchUpRadius;
            cmd->followRadius = followRadius;
        }
        return cmd;
    }

    virtual void Run() {
        NiPointer<TESObjectREFR> refr;
        if (LookupREFRByHandle(source, refr)) {
            if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
                Actor_KeepOffsetFromActor(actor, target, offset, offsetAngle, catchUpRadius, followRadius);
            }
        }
    }

    virtual void Dispose() {
        delete this;
    }

    UInt32 source;
    UInt32 target;
    NiPoint3 offset;
    NiPoint3 offsetAngle;
    float catchUpRadius;
    float followRadius;
};

struct ClearKeepOffsetTask : TaskDelegate
{
    static ClearKeepOffsetTask *Create(UInt32 source) {
        ClearKeepOffsetTask *cmd = new ClearKeepOffsetTask;
        if (cmd) {
            cmd->source = source;
        }
        return cmd;
    }

    virtual void Run() {
        NiPointer<TESObjectREFR> refr;
        if (LookupREFRByHandle(source, refr)) {
            if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
                Actor_ClearKeepOffsetFromActor(actor);
                Actor_EvaluatePackage(actor, false, false);
            }
        }
    }

    virtual void Dispose() {
        delete this;
    }

    UInt32 source;
};

float g_savedSpeedReduction = 0.f;

float GetSpeedReduction(Actor *actor)
{
    if (!Config::options.doSpeedReduction) return 0.f;

    if (Actor_IsInRagdollState(actor)) return 0.f;

    PlayerCharacter *player = *g_thePlayer;

    float massReduction = Config::options.mediumRaceSpeedReduction;
    if (TESRace *race = actor->race) {
        UInt32 raceSize = actor->race->data.unk40;
        if (raceSize == 0) massReduction = Config::options.smallRaceSpeedReduction;
        if (raceSize == 2) massReduction = Config::options.largeRaceSpeedReduction;
        if (raceSize >= 3) massReduction = Config::options.extraLargeRaceSpeedReduction;
    }

    float healthPercent = std::clamp(GetAVPercentage(actor, 24), 0.f, 1.f);
    float playerStaminaPercent = std::clamp(GetAVPercentage(player, 26), 0.f, 1.f);

    float reduction = massReduction * (healthPercent * Config::options.speedReductionHealthInfluence + (1.f - Config::options.speedReductionHealthInfluence));
    reduction = lerp(reduction, Config::options.maxSpeedReduction, 1.f - playerStaminaPercent);

    if (IsTeammate(actor)) {
        reduction *= Config::options.followerSpeedReductionMultiplier;
    }

    return std::clamp(reduction, 0.f, Config::options.maxSpeedReduction);
}

float GetGrabbedStaminaCost(Actor *actor)
{
    if (Config::options.followersSkipStaminaCost && IsTeammate(actor)) return 0.f;

    if (RelationshipRanks::GetRelationshipRank(actor->baseForm, (*g_thePlayer)->baseForm) > Config::options.grabbedstaminaDrainMaxRelationshipRank) return 0.f;

    if (Actor_IsInRagdollState(actor)) {
        KnockState knockState = GetActorKnockState(actor);
        bool isKnockedDown = knockState != KnockState::Normal && knockState != KnockState::GetUp; // knocked down and not getting up

        bool isReanimating = actor->IsDead(1) && IsReanimating(actor);

        if (!isKnockedDown && !isReanimating) return 0.f;
    }

    bool isHostile = Actor_IsHostileToActor(actor, *g_thePlayer);
    float initialCost = isHostile ? Config::options.grabbedActorHostileStaminaCost : Config::options.grabbedActorStaminaCost;
    float healthPercent = std::clamp(GetAVPercentage(actor, 24), 0.f, 1.f);
    float cost = initialCost * (healthPercent * Config::options.grabbedActorStaminaCostHealthInfluence + (1.f - Config::options.grabbedActorStaminaCostHealthInfluence));

    return cost;
}

bool g_rightDropAttempted = false;
bool g_leftDropAttempted = false;

void DropObject(bool isLeft)
{
    bool &dropAttempted = isLeft ? g_leftDropAttempted : g_rightDropAttempted;
    if (!dropAttempted) {
        g_higgsInterface->DisableHand(isLeft);
        dropAttempted = true;
    }
}

void UpdateHiggsDrop()
{
    if (g_rightDropAttempted && !g_rightHeldRefr) {
        g_higgsInterface->EnableHand(false);
        g_rightDropAttempted = false;
    }
    if (g_leftDropAttempted && !g_leftHeldRefr) {
        g_higgsInterface->EnableHand(true);
        g_leftDropAttempted = false;
    }
}

double g_lastHeldTime = 0.0;

void UpdateSpeedReduction()
{
    bool rightHasHeld = g_rightHeldRefr;
    bool leftHasHeld = g_leftHeldRefr;
    int numHeld = int(rightHasHeld) + int(leftHasHeld);

    bool rightCostStamina = false;
    bool leftCostStamina = false;

    float speedReduction = 0.f;
    float staminaCost = 0.f;
    if (numHeld > 0) {
        // We are holding at least 1 object
        if (rightHasHeld && leftHasHeld && g_rightHeldRefr == g_leftHeldRefr) {
            // Both hands are holding the same refr (ex. different limbs of the same body). We don't want to double up on the slowdown.
            if (Actor *actor = DYNAMIC_CAST(g_rightHeldRefr, TESObjectREFR, Actor)) {
                speedReduction += GetSpeedReduction(actor);
                float cost = GetGrabbedStaminaCost(actor);
                if (cost > 0.f) {
                    rightCostStamina = true;
                    leftCostStamina = true;
                    staminaCost += cost;
                }
            }
        }
        else {
            if (rightHasHeld) {
                if (Actor *actor = DYNAMIC_CAST(g_rightHeldRefr, TESObjectREFR, Actor)) {
                    speedReduction += GetSpeedReduction(actor);
                    float cost = GetGrabbedStaminaCost(actor);
                    if (cost > 0.f) {
                        rightCostStamina = true;
                        staminaCost += cost;
                    }
                }
            }
            if (leftHasHeld) {
                if (Actor *actor = DYNAMIC_CAST(g_leftHeldRefr, TESObjectREFR, Actor)) {
                    speedReduction += GetSpeedReduction(actor);
                    float cost = GetGrabbedStaminaCost(actor);
                    if (cost > 0.f) {
                        leftCostStamina = true;
                        staminaCost += cost;
                    }
                }
            }
        }

        speedReduction = std::clamp(speedReduction, 0.f, Config::options.maxSpeedReduction);
        g_lastHeldTime = g_currentFrameTime;
    }
    else if (g_currentFrameTime - g_lastHeldTime < Config::options.slowMovementFadeOutTime) {
        speedReduction = g_savedSpeedReduction * (1.f - (g_currentFrameTime - g_lastHeldTime) / Config::options.slowMovementFadeOutTime);
    }

    if (speedReduction != g_savedSpeedReduction) {
        PlayerCharacter *player = *g_thePlayer;

        // First just undo our previous speed change
        ModSpeedMult(player, g_savedSpeedReduction);

        // Now modify the speed based on what we have held
        ModSpeedMult(player, -speedReduction);

        g_savedSpeedReduction = speedReduction;
    }

    if (staminaCost > 0.f) {
        PlayerCharacter *player = *g_thePlayer;

        float cost = staminaCost * *g_deltaTime;
        DamageAV(player, 26, -cost);

        if (player->actorValueOwner.GetCurrent(26) <= 0.f) {
            // Out of stamina
            if (rightCostStamina) {
                // drop right object
                DropObject(false);
            }
            if (leftCostStamina) {
                // drop left object
                DropObject(true);
            }

            FlashHudMenuMeter(26);

            if (Config::options.playSoundOnGrabStaminaDepletion) {
                if (BGSSoundDescriptorForm *shoutFailSound = GetDefaultObject<BGSSoundDescriptorForm>(129)) {
                    PlaySoundAtNode(shoutFailSound, player->GetNiNode(), {});
                }
            }
        }
    }
}

bool UpdateActorShove(Actor *actor)
{
    if (!Config::options.enableActorShove) return false;

    {
        std::scoped_lock lock(g_shoveTimesLock);
        if (auto it = g_shoveData.find(&actor->animGraphHolder); it != g_shoveData.end()) {
            ShoveData &shoveData = it->second;
            double shovedTime = shoveData.shovedTime;
            if (g_currentFrameTime - shovedTime > Config::options.shoveWaitForBumpTimeBeforeStagger) {
                {
                    std::scoped_lock lock2(g_shoveAnimLock);
                    auto it2 = g_shoveAnimTimes.find(&actor->animGraphHolder);
                    if (it2 == g_shoveAnimTimes.end() || it2->second < shovedTime) {
                        // Large bump anim didn't play or it played before we shoved them (from a previous shove)
                        StaggerActor(actor, shoveData.shoveDirection, Config::options.shoveStaggerMagnitude);
                    }
                    g_shoveAnimTimes.erase(&actor->animGraphHolder);
                }
                g_shoveData.erase(&actor->animGraphHolder);
            }
        }
    }

    PlayerCharacter *player = *g_thePlayer;
    if (Actor_IsInRagdollState(player) || IsSwimming(player) || IsStaggered(player) || g_isMenuOpen) return false;
    if (Config::options.disableShoveWhileWeaponsDrawn && player->actorState.IsWeaponDrawn()) return false;

    for (int isLeft = 0; isLeft < 2; ++isLeft) {
        ControllerTrackingData &controllerData = g_controllerData[isLeft];

        if (controllerData.avgSpeed > Config::options.shoveSpeedThreshold) {
            if (g_physicsListener.handCollidedRefs[isLeft].count(actor) && ShouldShoveActor(actor)) {
                if (!g_shovedActors.count(actor)) {
                    float staminaCost = Config::options.shoveStaminaCost;
                    float staminaBeforeHit = player->actorValueOwner.GetCurrent(26);
                    if (staminaBeforeHit > 0.f || staminaCost <= 0.f) {
                        // Shove costs no stamina, or we have enough stamina
                        DamageAV(player, 26, -staminaCost);

                        if (player->actorValueOwner.GetCurrent(26) <= 0.f) {
                            // Out of stamina after the shove
                            if (ActorProcessManager *process = player->processManager) {
                                float regenRate = Actor_GetActorValueRegenRate(player, 26);
                                ActorProcess_UpdateRegenDelay(process, 26, (staminaCost - staminaBeforeHit) / regenRate);
                                FlashHudMenuMeter(26);
                            }
                        }

                        NiPoint3 shoveDirection = VectorNormalized(controllerData.avgVelocity);
                        QueueBumpActor(actor, shoveDirection, true, false, false, false);

                        if (Config::options.playShovePhysicsSound) {
                            if (NiPointer<bhkRigidBody> rigidBody = GetFirstRigidBody(GetTorsoNode(actor))) {
                                if (NiPointer<NiAVObject> handNode = GetFirstPersonHandNode(isLeft)) {
                                    PlayPhysicsSound(rigidBody->hkBody->getCollidableRw(), handNode->m_worldTransform.pos, true);
                                }
                            }
                        }
                    }
                    else {
                        // Not enough stamina
                        if (Config::options.playSoundOnShoveNoStamina) {
                            if (BGSSoundDescriptorForm *magicFailSound = GetDefaultObject<BGSSoundDescriptorForm>(128)) {
                                PlaySoundAtNode(magicFailSound, player->GetNiNode(), {});
                            }
                        }
                        FlashHudMenuMeter(26);
                    }

                    PlayRumble(!isLeft, Config::options.shoveRumbleIntensity, Config::options.shoveRumbleDuration);
                    // Ignore future contact points for a bit to make things less janky
                    g_physicsListener.collisionCooldownTargets[isLeft][actor] = g_currentFrameTime;

                    if (g_controllerData[!isLeft].avgSpeed > Config::options.shoveSpeedThreshold) {
                        PlayRumble(isLeft, Config::options.shoveRumbleIntensity, Config::options.shoveRumbleDuration);
                        g_physicsListener.collisionCooldownTargets[!isLeft][actor] = g_currentFrameTime;
                    }

                    g_shovedActors[actor] = g_currentFrameTime;

                    return true;
                }
            }
        }
    }

    return false;
}

void ResetObjects()
{
    g_npcs.clear();
    g_activeActors.clear();
    g_activeRagdolls.clear();
    g_actorData.clear();
    g_activeBipedGroups.clear();
    g_noPlayerCharControllerCollideGroups.clear();
    g_hittableCharControllerGroups.clear();
    g_selfCollidableBipedGroups.clear();
    g_higgsLingeringRigidBodies.clear();
    g_keepOffsetActors.clear();
    g_bumpActors.clear();
    g_shoveData.clear();
    g_shoveAnimTimes.clear();
    g_shovedActors.clear();
    g_physicsListener = PhysicsListener{};
}

double g_worldChangedTime = 0.0;


void PlayerCharacter_UpdateWeaponSwing_Hook(PlayerCharacter *player, float deltaTime)
{
    g_rightSwingHandler.UpdateWeaponSwing(deltaTime);
    g_leftSwingHandler.UpdateWeaponSwing(deltaTime);
}

void PreVrikPreHiggsCallback()
{
    // Wand nodes are up to date at this point

    for (int i = 0; i < 2; i++) {
        bool isLeft = bool(i);
        NiAVObjectPtr wandNode = GetWandNode(isLeft);
        NiPoint3 wandPositionRoomspace = wandNode->m_localTransform.pos;

        ControllerTrackingData &controllerData = g_controllerData[i];
        controllerData.positionsRoomspace.pop_back();
        controllerData.positionsRoomspace.push_front(wandPositionRoomspace);
        controllerData.ComputeAngularVelocity();
    }
}

NiTransform g_initialGrabTransforms[2]{};
NiTransform g_initialHeldWeaponLocalTransforms[2]{};

void ObjectGrabbedCallback(bool isLeft, TESObjectREFR *grabbedRefr)
{
    g_initialGrabTransforms[isLeft] = g_higgsInterface->GetGrabTransform(isLeft);
}

_ProcessHavokHitJobs ProcessHavokHitJobs_Original = 0;
void ProcessHavokHitJobsHook(HavokHitJobs *havokHitJobs)
{
    ProcessHavokHitJobs(havokHitJobs);

    PlayerCharacter *player = *g_thePlayer;
    if (!player || !player->GetNiNode()) return;

    TESObjectCELL *cell = player->parentCell;
    if (!cell) return;

    NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell);
    if (!world) return;

    AIProcessManager *processManager = *g_aiProcessManager;
    if (!processManager) return;

#ifdef _DEBUG
    Config::ReloadIfModified();
#endif // _DEBUG

    g_currentFrameTime = GetTime();

    {
        UInt32 filterInfo; Actor_GetCollisionFilterInfo(player, filterInfo);
        g_playerCollisionGroup = filterInfo >> 16;
    }

    if (world != g_physicsListener.world) {
        if (NiPointer<bhkWorld> oldWorld = g_physicsListener.world) {
            _MESSAGE("%d: Removing listeners from old havok world", *g_currentFrameCounter);
            {
                BSWriteLocker lock(&oldWorld->worldLock);
                if (hkpWorldExtension *collisionCallbackExtension = hkpWorld_findWorldExtension(oldWorld->world, hkpKnownWorldExtensionIds::HK_WORLD_EXTENSION_COLLISION_CALLBACK)) {
                    // There are times when the collision callback extension is gone even if we required it earlier...
                    hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(oldWorld->world);
                }
                hkpWorld_removeContactListener(oldWorld->world, &g_physicsListener);
                hkpWorld_removeWorldPostSimulationListener(oldWorld->world, &g_physicsListener);
                hkpWorld_removeEntityListener(oldWorld->world, &g_physicsListener);
            }

            ResetObjects();
        }

        _MESSAGE("%d: Havok world changed", *g_currentFrameCounter);
        {
            BSWriteLocker lock(&world->worldLock);

            hkpCollisionCallbackUtil_requireCollisionCallbackUtil(world->world);

            hkpWorld_addContactListener(world->world, &g_physicsListener);
            hkpWorld_addWorldPostSimulationListener(world->world, &g_physicsListener);
            hkpWorld_addEntityListener(world->world, &g_physicsListener);

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
            if (Config::options.enableBipedProjectileCollision) {
                filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Projectile);
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

        g_physicsListener.world = world;
        g_worldChangedTime = g_currentFrameTime;
    }

    if (NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer)) {
        if (controller != g_characterProxyListener.proxy) {
            if (RE::hkRefPtr<hkpCharacterProxy> proxy = controller->proxy.characterProxy) {
                _MESSAGE("%d: Player Character Proxy changed", *g_currentFrameCounter);

                BSWriteLocker lock(&world->worldLock);

                if (hkpCharacterProxy_findCharacterProxyListener(proxy, &g_characterProxyListener) == -1) {
                    hkpCharacterProxy_addCharacterProxyListener(proxy, &g_characterProxyListener);
                }

                // Normally, the player has a list shape that has both a hkCharControllerShape and a capsule shape, but in werewolf / vampire lord, they have only the charcontroller shape.
                hkpListShape *listShape = DYNAMIC_CAST(proxy->m_shapePhantom->m_collidable.m_shape, hkpShape, hkpListShape);
                hkpConvexVerticesShape *convexVerticesShape = DYNAMIC_CAST(listShape ? listShape->m_childInfo[0].m_shape : proxy->m_shapePhantom->m_collidable.m_shape, hkpShape, hkpConvexVerticesShape);
                hkpCapsuleShape *capsule = DYNAMIC_CAST(listShape ? listShape->m_childInfo[1].m_shape : proxy->m_shapePhantom->m_collidable.m_shape, hkpShape, hkpCapsuleShape);

                if (Config::options.resizePlayerCharController && convexVerticesShape) {
                    // Shrink convex charcontroller shape
                    g_scratchHkArray.clear();
                    hkArray<hkVector4> &verts = g_scratchHkArray;

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
                        float oldSlope = zOld / rOld;

                        float rNew = Config::options.playerCharControllerRadius;
                        float zNew = rNew * oldSlope;
                        float zAdjustment = zNew - zOld;

                        float maxAdjustment = Config::options.playerCharControllerBottomRingMaxHeightAdjustment;
                        zAdjustment = std::clamp(zAdjustment, -maxAdjustment, maxAdjustment);
                        zNew = zOld + zAdjustment;

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

                    hkpConvexVerticesShape *newShape = hkAllocReferencedObject<hkpConvexVerticesShape>();
                    hkpConvexVerticesShape_ctor(newShape, newVerts, buildConfig); // sets refcount to 1

                    // it's actually a hkCharControllerShape not just a hkpConvexVerticesShape
                    set_vtbl(newShape, hkCharControllerShape_vtbl);

                    bhkShape *wrapper = (bhkShape *)convexVerticesShape->m_userData;
                    wrapper->SetHavokObject(newShape);

                    // The listshape does not use a hkRefPtr but it's still setup to add a reference upon construction and remove one on destruction
                    if (listShape) {
                        listShape->m_childInfo[0].m_shape = newShape;
                        hkReferencedObject_removeReference(convexVerticesShape); // this will usually call the dtor on the old shape

                        // We don't need to remove a ref here, the ctor gave it a refcount of 1 and we assigned it to the listShape which isn't technically a hkRefPtr but still owns it (and the listShape's dtor will decref anyways)
                        // hkReferencedObject_removeReference(newShape);
                    }
                    else {
                        proxy->m_shapePhantom->setShape(newShape);
                        hkReferencedObject_removeReference(newShape);
                    }
                }

                if (Config::options.resizePlayerCapsule && capsule) {
                    // TODO: Am I accidentally modifying every npc's capsule too? I don't think so.
                    // Shrink capsule shape too. It's active when weapons are unsheathed.
                    float radius = Config::options.playerCapsuleRadius;
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

    { // Update higgs info
        NiPointer<bhkRigidBody> rightHand = (bhkRigidBody *)g_higgsInterface->GetHandRigidBody(false);
        NiPointer<bhkRigidBody> leftHand = (bhkRigidBody *)g_higgsInterface->GetHandRigidBody(true);
        g_rightHand = rightHand;
        g_leftHand = leftHand;

        NiPointer<bhkRigidBody> rightWeapon = (bhkRigidBody *)g_higgsInterface->GetWeaponRigidBody(false);
        NiPointer<bhkRigidBody> leftWeapon = (bhkRigidBody *)g_higgsInterface->GetWeaponRigidBody(true);
        g_rightWeapon = rightWeapon;
        g_leftWeapon = leftWeapon;

        // TODO: We don't make held objects keyframed_reporting, so those can't do hits on statics
        if (rightWeapon && rightWeapon->hkBody->getQualityType() != hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING) {
            rightWeapon->hkBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
            bhkWorld_UpdateCollisionFilterOnWorldObject(world, rightWeapon);
        }
        if (leftWeapon && leftWeapon->hkBody->getQualityType() != hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING) {
            leftWeapon->hkBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
            bhkWorld_UpdateCollisionFilterOnWorldObject(world, leftWeapon);
        }

        if (rightHand && rightHand->hkBody->getQualityType() != hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING) {
            rightHand->hkBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
            bhkWorld_UpdateCollisionFilterOnWorldObject(world, rightHand);
        }
        if (leftHand && leftHand->hkBody->getQualityType() != hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING) {
            leftHand->hkBody->setQualityType(hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING);
            bhkWorld_UpdateCollisionFilterOnWorldObject(world, leftHand);
        }

        if (rightHand) {
            g_higgsCollisionLayer = rightHand->hkBody->m_collidable.getBroadPhaseHandle()->m_collisionFilterInfo & 0x7f;
        }

        g_prevLeftHeldObject = g_leftHeldObject;
        g_prevRightHeldObject = g_rightHeldObject;

        g_rightHeldObject = (bhkRigidBody *)g_higgsInterface->GetGrabbedRigidBody(false);
        g_leftHeldObject = (bhkRigidBody *)g_higgsInterface->GetGrabbedRigidBody(true);

        g_rightHeldRefr = g_higgsInterface->GetGrabbedObject(false);
        g_leftHeldRefr = g_higgsInterface->GetGrabbedObject(true);

        if (g_rightHeldObject) {
            g_higgsLingeringRigidBodies[g_rightHeldObject] = g_currentFrameTime;
        }
        if (g_leftHeldObject) {
            g_higgsLingeringRigidBodies[g_leftHeldObject] = g_currentFrameTime;
        }

        // Clear out old dropped / thrown rigidbodies
        for (auto it = g_higgsLingeringRigidBodies.begin(); it != g_higgsLingeringRigidBodies.end();) {
            auto [target, hitTime] = *it;
            if ((g_currentFrameTime - hitTime) * *g_globalTimeMultiplier >= Config::options.thrownObjectLingerTime)
                it = g_higgsLingeringRigidBodies.erase(it);
            else
                ++it;
        }
    }

    UpdateHiggsDrop();

    // Do this after we've updated higgs things
    UpdateSpeedReduction();

    RunDelayedJobs(g_currentFrameTime);

    { // Ensure our listener is the last one (will be called first)
        hkArray<hkpContactListener *> &listeners = world->world->m_contactListeners;
        if (listeners[listeners.getSize() - 1] != &g_physicsListener) {
            BSWriteLocker lock(&world->worldLock);

            int numListeners = listeners.getSize();
            int listenerIndex = listeners.indexOf(&g_physicsListener);
            if (listenerIndex >= 0) {
                for (int i = listenerIndex + 1; i < numListeners; ++i) {
                    listeners[i - 1] = listeners[i];
                }
                listeners[numListeners - 1] = &g_physicsListener;
            }
        }
    }

    { // Clear out old shoved actors
        for (auto it = g_shovedActors.begin(); it != g_shovedActors.end();) {
            auto [actor, shovedTime] = *it;
            if ((g_currentFrameTime - shovedTime) > Config::options.shoveCooldown)
                it = g_shovedActors.erase(it);
            else
                ++it;
        }
    }

    if (g_currentFrameTime - g_worldChangedTime < Config::options.worldChangedWaitTime) return;

    // Do this before any NPC state updates
    g_isRightHandAggressivelyPositioned = IsHandWithinConeFromHmd(false, Config::options.aggressionRequiredHandWithinHmdConeHalfAngle);
    g_isLeftHandAggressivelyPositioned = IsHandWithinConeFromHmd(true, Config::options.aggressionRequiredHandWithinHmdConeHalfAngle);

    g_isMenuOpen = MenuChecker::isGameStopped();

    UInt16 rightHeldCollisionGroup = 0;
    UInt16 leftHeldCollisionGroup = 0;

    for (UInt32 i = 0; i < processManager->actorsHigh.count; i++) {
        UInt32 actorHandle = processManager->actorsHigh[i];
        NiPointer<TESObjectREFR> refr;
        if (LookupREFRByHandle(actorHandle, refr) && refr != player) {
            Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
            if (!actor || !actor->GetNiNode()) continue;

            std::shared_ptr<ActorData> actorData = GetActorData(actor);
            if (!actorData) {
                g_actorData[actor] = std::shared_ptr<ActorData>(new ActorData());
            }

            bool didShove = UpdateActorShove(actor);

            TryUpdateNPCState(actor, didShove);

#ifdef _DEBUG
            TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);
            /*if (std::string_view(name->name) == "Faendal") {
                g_interface001.SetAggressionLowTopic(actor, (TESTopic*)LookupFormByID(0x000142B5));
                g_interface001.SetAggressionLowTopic(nullptr, (TESTopic*)LookupFormByID(0x000142B2));
                if (fmod(g_currentFrameTime, 10.0) < 5.0) {
                    g_interface001.AddIgnoredActor(actor);
                }
                else {
                    g_interface001.RemoveIgnoredActor(actor);
                }
            }*/
#endif // _DEBUG

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
                            g_taskInterface->AddTask(KeepOffsetTask::Create(GetOrCreateRefrHandle(actor), *g_playerHandle));
                            g_keepOffsetActors[actor] = { g_currentFrameTime, false };
                        }
                        else {
                            // Already in the set, so check if it actually succeeded at first
                            KeepOffsetData &data = it->second;

                            if (GetMovementController(actor) && !HasKeepOffsetInterface(actor)) {
                                if (g_currentFrameTime - data.lastAttemptTime > Config::options.keepOffsetRetryInterval) {
                                    // Retry

                                    if (Config::options.bumpActorIfKeepOffsetFails) {
                                        // Try to get them unstuck by bumping them
                                        QueueBumpActor(actor, { 0.f, 0.f, 0.f }, 0.f, false, false, false, false);
                                    }

                                    g_taskInterface->AddTask(KeepOffsetTask::Create(GetOrCreateRefrHandle(actor), *g_playerHandle));
                                    data.lastAttemptTime = g_currentFrameTime;
                                }
                            }
                            else {
                                if (!data.success) {
                                    // To be sure, do a single additional attempt once we know the interface exists
                                    g_taskInterface->AddTask(KeepOffsetTask::Create(GetOrCreateRefrHandle(actor), *g_playerHandle));
                                    data.success = true;
                                }
                                else { // KeepOffset interface exists and has succeeded
                                    ForEachRagdollDriver(actor, [actor, player](hkbRagdollDriver *driver) {
                                        if (std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver)) {
                                            NiPoint3 offset = { 0.f, 0.f, 0.f };
                                            /*
                                            NiPoint3 rootOffsetXY = ragdoll->rootOffset;
                                            rootOffsetXY.z = 0.f;
                                            if (VectorLength(rootOffsetXY) >= Config::options.keepOffsetMinPosDifference) {
                                                NiTransform refrTransform; TESObjectREFR_GetTransformIncorporatingScale(actor, refrTransform);
                                                NiPoint3 offsetWS = refrTransform.pos + rootOffsetXY * Config::options.keepOffsetPosDifferenceMultiplier;
                                                offset = InverseTransform(refrTransform) * offsetWS;
                                            }
                                            */

                                            NiPoint3 offsetAngle = { 0.f, 0.f, 0.f };
                                            if (fabsf(ConstrainAngle180(ragdoll->rootOffsetAngle)) >= Config::options.keepOffsetMinAngleDifference) {
                                                // offsetAngle.z < PI -> clockwise, >= PI -> counter-clockwise
                                                offsetAngle.z = ConstrainAngle180(ragdoll->rootOffsetAngle) * Config::options.keepOffsetAngleDifferenceMultiplier;
                                            }

                                            UInt32 actorHandle = GetOrCreateRefrHandle(actor);
                                            UInt32 playerHandle = *g_playerHandle;
                                            g_taskInterface->AddTask(KeepOffsetTask::Create(actorHandle, playerHandle, offset, offsetAngle, 150.f, 50.f));
                                            //g_taskInterface->AddTask(KeepOffsetTask::Create(actorHandle, actorHandle, offset, offsetAngle, 150.f, 0.f));
                                        }
                                        });
                                }
                            }
                        }
                    }
                }

                // When an npc is grabbed, disable collision with them
                if (actor == g_rightHeldRefr) {
                    rightHeldCollisionGroup = collisionGroup;
                }
                if (actor == g_leftHeldRefr) {
                    leftHeldCollisionGroup = collisionGroup;
                }
            }
            else {
                if (g_keepOffsetActors.size() > 0 && g_keepOffsetActors.count(actor)) {
                    g_taskInterface->AddTask(ClearKeepOffsetTask::Create(GetOrCreateRefrHandle(actor)));
                    g_keepOffsetActors.erase(actor);
                }
            }

            bool isHittableCharController = g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(collisionGroup);

            bool shouldAddToWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale < Config::options.activeRagdollStartDistance;
            bool shouldRemoveFromWorld = VectorLength(actor->pos - player->pos) * *g_havokWorldScale > Config::options.activeRagdollEndDistance;

            bool isAddedToWorld = IsAddedToWorld(actor);
            bool isActiveActor = g_activeActors.count(actor);
            bool isProcessedActor = isActiveActor || isHittableCharController;
            bool canAddToWorld = CanAddToWorld(actor);

            if (shouldAddToWorld) {
                if ((!isAddedToWorld || !isProcessedActor) && canAddToWorld) {
                    AddRagdollToWorld(actor);
                    if (collisionGroup != 0) {
                        g_activeBipedGroups.insert(collisionGroup);
                        if (isHittableCharController) {
                            g_hittableCharControllerGroups.erase(collisionGroup);
                        }
                    }
                }

                if (!canAddToWorld) {
                    if (isActiveActor) {
                        // Someone in range went from having a ragdoll or not being excluded, to not having a ragdoll or being excluded
                        RemoveRagdollFromWorld(actor);
                        g_activeBipedGroups.erase(collisionGroup);
                        isActiveActor = false;
                    }

                    // There is no ragdoll instance, but we still need a way to hit the enemy, e.g. for the wisp (witchlight).
                    // In this case, we need to register collisions against their charcontroller.
                    if (collisionGroup != 0) {
                        g_hittableCharControllerGroups.insert(collisionGroup);
                    }
                }

                if (isActiveActor) {
                    if (collisionGroup != 0) {
                        g_activeBipedGroups.insert(collisionGroup);

                        if (
                            (Config::options.disablePlayerSummonCollision && GetCommandingActor(actor) == *g_playerHandle) ||
                            (Config::options.disablePlayerFollowerCollision && IsTeammate(actor))
                            ) {
                            g_noPlayerCharControllerCollideGroups.insert(collisionGroup);
                        }
                        else {
                            g_noPlayerCharControllerCollideGroups.erase(collisionGroup);
                        }
                    }

                    // Sometimes the game re-enables sync-on-update e.g. when switching outfits, so we need to make sure it's disabled.
                    DisableSyncOnUpdate(actor);

                    if (Config::options.forceAnimationUpdateForActiveActors) {
                        // Force the game to run the animation graph update (and hence driveToPose, etc.)
                        actor->flags2 |= (1 << 8);
                    }

                    // Set whether we want biped self-collision for this actor
                    if (Config::options.doBipedSelfCollision && collisionGroup != 0) {
                        if (TESRace *race = actor->race) {
                            const char *name = race->editorId;
                            if ((Config::options.doBipedSelfCollisionForNPCs && race->keyword.HasKeyword(g_keyword_actorTypeNPC)) ||
                                (name && Config::options.additionalSelfCollisionRaces.count(std::string_view(name)))) {

                                if (g_physicsListener.IsCollided(actor) || isHeld) {
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

                    /*
                                        {
                                            bool isDead = actor->IsDead(1);

                                            {
                                                if (BipedModel *biped = actor->GetBipedSmall()) {
                                                    if (Biped *bipedData = biped->bipedData) {
                                                        // TODO: This doesn't handle weapons equipped in the offhand. We want to include those, but likely ignore shields.
                                                        // Shield / offhand is index 9.

                                                        UInt8 drawState = actor->actorState.flags08 >> 5 & 0x7;
                                                        bool isWeaponDrawn = drawState == 3;
                                                        bool shouldAddToWorld = isWeaponDrawn || isDead;

                                                        for (int i = 32; i < 42; i++) {
                                                            if (NiPointer<NiAVObject> geomNode = bipedData->unk10[i].object) {
                                                                if (shouldAddToWorld) {
                                                                    actorData->weaponRoot = geomNode;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }

                                                if (NiPointer<NiAVObject> weaponRoot = actorData->weaponRoot) {
                                                    if (NiPointer<bhkRigidBody> weaponBody = GetRigidBody(weaponRoot)) {
                                                        bool isWeaponValid = false;

                                                        if (NiPointer<NiAVObject> parent = GetClosestParentWithCollision(weaponRoot, true)) {
                                                            if (NiPointer<bhkRigidBody> handBody = GetRigidBody(parent)) {
                                                                if (IsRagdollHandRigidBody(handBody->hkBody)) {
                                                                    if (bhkShape *handShapeWrapper = (bhkShape *)handBody->hkBody->m_collidable.m_shape->m_userData) {
                                                                        if (bhkShape *weaponShapeWrapper = (bhkShape *)weaponBody->hkBody->m_collidable.m_shape->m_userData) {
                                                                            isWeaponValid = true;

                                                                            if (weaponShapeWrapper != actorData->clonedFromShape) {
                                                                                NiCloningProcess cloningProcess = NiCloningProcess();

                                                                                _MESSAGE("clone");

                                                                                if (bhkRigidBodyT *rigidBodyT = DYNAMIC_CAST(weaponBody, bhkRigidBody, bhkRigidBodyT)) {
                                                                                    // TODO: Handle this - probably the best thing to do is just apply the bhkRigidBodyT transform to the transform shape directly
                                                                                    _MESSAGE("bhkRigidBodyT");
                                                                                }

                                                                                // The weapon local transform actually changes during play, so we need to change the transform (or re-create the transform shape) whenever the weapon node local transform changes too much.

                                                                                bhkShape *clonedShape = (bhkShape *)NiObject_Clone(weaponShapeWrapper, &cloningProcess);
                                                                                bhkTransformShape *transformShapeWrapper = CreatebhkTransformShape();
                                                                                hkpTransformShape *transformShape = hkAllocReferencedObject<hkpTransformShape>();

                                                                                NiTransform weaponLocalTransform = weaponRoot->m_parent->m_localTransform;
                                                                                hkTransform transform = NiTransformTohkTransform(weaponLocalTransform);

                                                                                hkpTransformShape_ctor(transformShape, clonedShape->shape, transform);
                                                                                transformShapeWrapper->shape = transformShape;
                                                                                hkReferencedObject_removeReference(transformShape);
                                                                                transformShape->m_userData = (hkUlong)transformShapeWrapper;

                                                                                transformShapeWrapper->materialId = clonedShape->materialId;

                                                                                // Create a list shape with both the original hand shape and the cloned weapon shape - this way we can easily toggle off the weapon shape when they die/sheathe.
                                                                                // TODO: We need to restore just the original hand node when the actor is killed or drops the weapon.

                                                                                if (!actorData->handShape) {
                                                                                    actorData->handShape = handShapeWrapper;
                                                                                }

                                                                                bhkListShape *listShape = (bhkListShape *)Heap_Allocate(sizeof(bhkListShape));

                                                                                {
                                                                                    bhkListShapeCinfo cinfo{};

                                                                                    cinfo.shapes.pushBack(actorData->handShape->shape); // add the original hand shape
                                                                                    cinfo.shapes.pushBack(transformShape); // add the new weapon shape

                                                                                    bhkListShape_ctor(listShape, &cinfo);
                                                                                }

                                                                                {
                                                                                    BSWriteLocker lock(&world->worldLock);

                                                                                    handBody->RemoveFromCurrentWorld();

                                                                                    handShapeWrapper->DecRef();
                                                                                    handBody->hkBody->setShape(listShape->shape); // Could this be causing crashes? We do lock the world here, but perhaps that's getting queued for later and not done on the spot?
                                                                                    listShape->IncRef();

                                                                                    bhkWorld_AddEntity(world, handBody->hkBody);
                                                                                }

                                                                                actorData->combinedShape = listShape;

                                                                                actorData->clonedFromShape = weaponShapeWrapper;
                                                                            }

                                                                            if (actorData->combinedShape) {
                                                                                hkpListShape *listShape = (hkpListShape *)actorData->combinedShape->shape.val();
                                                                                if (hkpTransformShape *shape = DYNAMIC_CAST(listShape->getChildShapeInl(1), hkpShape, hkpTransformShape)) {
                                                                                    NiTransform weaponLocalTransform = weaponRoot->m_parent->m_localTransform;
                                                                                    hkTransform transform = NiTransformTohkTransform(weaponLocalTransform);

                                                                                    // This is not recommended as collision agents may have cached data for this shape assuming the previous transform. But, it's kind of the best we can do.
                                                                                    hkpTransformShape_setTransform(shape, transform);

                                                                                    if (g_rightHeldObject == handBody) {
                                                                                        if (g_rightHeldObject != g_prevRightHeldObject) {
                                                                                            g_initialHeldWeaponLocalTransforms[false] = weaponLocalTransform;
                                                                                        }

                                                                                        // Keep Constant: hand in the space of the weapon
                                                                                        // hand in space of body = weapon in space of body * hand in space of weapon
                                                                                        // initial hand in space of weapon = inverse(initial weapon in space of body) * initial hand in space of body
                                                                                        // final hand in space of weapon = inverse(final weapon in space of body) * (final hand in space of body)
                                                                                        // => inverse(initial weapon in space of body) * initial hand in space of body = inverse(final weapon in space of body) * final hand in space of body
                                                                                        // => final hand in space of body = final weapon in space of body * inverse(initial weapon in space of body) * initial hand in space of body

                                                                                        NiTransform initialBodyToHand = InverseTransform(g_initialGrabTransforms[false]);
                                                                                        NiTransform newBodyToHand = weaponLocalTransform * InverseTransform(g_initialHeldWeaponLocalTransforms[false]) * initialBodyToHand;
                                                                                        g_higgsInterface->SetGrabTransform(false, InverseTransform(newBodyToHand));
                                                                                    }

                                                                                    if (g_leftHeldObject == handBody) {
                                                                                        if (g_leftHeldObject != g_prevLeftHeldObject) {
                                                                                            g_initialHeldWeaponLocalTransforms[true] = weaponLocalTransform;
                                                                                        }

                                                                                        NiTransform initialBodyToHand = InverseTransform(g_initialGrabTransforms[true]);
                                                                                        NiTransform newBodyToHand = weaponLocalTransform * InverseTransform(g_initialHeldWeaponLocalTransforms[true]) * initialBodyToHand;
                                                                                        g_higgsInterface->SetGrabTransform(true, InverseTransform(newBodyToHand));
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }

                                                        if (!isWeaponValid || isDead) {
                                                            // The weapon is not in the hand (not drawn, etc.), or the actor is dead

                                                            if (actorData->combinedShape) {
                                                                hkpListShape *listShape = (hkpListShape *)actorData->combinedShape->shape.val();
                                                                // Disable the child shape which is the weapon shape (keep the hand shape active)
                                                                if (listShape->isChildEnabled(1)) {
                                                                    hkpListShape_disableChild(listShape, 1);
                                                                }
                                                            }
                                                        }
                                                        else {
                                                            if (actorData->combinedShape) {
                                                                hkpListShape *listShape = (hkpListShape *)actorData->combinedShape->shape.val();
                                                                // Enable the child shape which is the weapon shape
                                                                if (!listShape->isChildEnabled(1)) {
                                                                    hkpListShape_enableChild(listShape, 1);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                    */


                }
            }
            else if (shouldRemoveFromWorld) {
                if (isAddedToWorld && canAddToWorld) {
                    RemoveRagdollFromWorld(actor);
                    g_activeBipedGroups.erase(collisionGroup);
                    g_noPlayerCharControllerCollideGroups.erase(collisionGroup);
                }
                else if (isHittableCharController) {
                    g_hittableCharControllerGroups.erase(collisionGroup);
                }
            }
        }
    }

    g_rightHeldActorCollisionGroup = rightHeldCollisionGroup;
    g_leftHeldActorCollisionGroup = leftHeldCollisionGroup;
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

void SetBonesKeyframedReporting(hkbRagdollDriver *driver, hkbGeneratorOutput &generatorOutput, hkbGeneratorOutput::TrackHeader &header)
{
    // - Set onFraction > 1.0f
    // - Set value of keyframed bones tracks to > 1.0f for bones we want keyframed, <= 1.0f for bones we don't want keyframed. Index of track data == index of bone.
    // - Set reportingWhenKeyframed in the ragdoll driver for the bones we care about

    header.m_onFraction = 1.1f;
    hkReal *data = Track_getData(generatorOutput, header);
    const hkaSkeleton *skeleton = driver->ragdoll->m_skeleton;
    for (int i = 0; i < skeleton->m_bones.getSize(); i++) { // TODO: We need to check the capacity of this track to see if we can fit all the bones? What about numData?
        data[i] = 1.1f; // anything > 1
        // Indexed by (boneIdx >> 5), and then you >> (boneIdx & 0x1F) & 1 to extract the specific bit
        driver->reportingWhenKeyframed[i >> 5] |= (1 << (i & 0x1F));
    }
}


void PreDriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext &context, hkbGeneratorOutput &generatorOutput)
{
    Actor *actor = GetActorFromRagdollDriver(driver);
    if (!actor) return;

    hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POSE);
    hkbGeneratorOutput::TrackHeader *worldFromModelHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_WORLD_FROM_MODEL);
    hkbGeneratorOutput::TrackHeader *keyframedBonesHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_KEYFRAMED_RAGDOLL_BONES);
    hkbGeneratorOutput::TrackHeader *rigidBodyHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_RIGID_BODY_RAGDOLL_CONTROLS);
    hkbGeneratorOutput::TrackHeader *poweredHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POWERED_RAGDOLL_CONTROLS);
    hkbGeneratorOutput::TrackHeader *poweredWorldFromModelModeHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POWERED_RAGDOLL_WORLD_FROM_MODEL_MODE);

    std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver);
    if (!ragdoll) return;

    ragdoll->deltaTime = deltaTime;

    KnockState prevKnockState = ragdoll->knockState;
    ragdoll->knockState = GetActorKnockState(actor);

    bool isRigidBodyOn = rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f;
    bool isPoweredOn = poweredHeader && poweredHeader->m_onFraction > 0.f;

    bool isComputingWorldFromModel = false;
    bool isUsingRootBoneAsWorldFromModel = false;
    if (poweredWorldFromModelModeHeader && poweredWorldFromModelModeHeader->m_onFraction > 0.f) {
        hkbWorldFromModelModeData &worldFromModelMode = *(hkbWorldFromModelModeData *)Track_getData(generatorOutput, *poweredWorldFromModelModeHeader);
        if (worldFromModelMode.mode == hkbWorldFromModelModeData::WorldFromModelMode::WORLD_FROM_MODEL_MODE_COMPUTE) {
            isComputingWorldFromModel = true;
        }
        else if (worldFromModelMode.mode == hkbWorldFromModelModeData::WorldFromModelMode::WORLD_FROM_MODEL_MODE_USE_ROOT_BONE) {
            isUsingRootBoneAsWorldFromModel = true;
        }
    }

    if (isComputingWorldFromModel && !ragdoll->wasComputingWorldFromModel) {
        // Went from not computing worldfrommodel to computing it
        if (Config::options.fadeInComputedWorldFromModel) {
            ragdoll->stickyWorldFromModel = ragdoll->worldFromModel; // We haven't updated ragdoll->worldFromModel yet this frame, so this actually the previous worldFromModel
            ragdoll->worldFromModelFadeInTime = g_currentFrameTime;
            ragdoll->fadeInWorldFromModel = true;
        }
    }
    else if (!isComputingWorldFromModel && ragdoll->wasComputingWorldFromModel) {
        // Went from computing worldfrommodel to not computing it any more
        if (Config::options.fadeOutComputedWorldFromModel) {
            ragdoll->stickyWorldFromModel = ragdoll->worldFromModel; // We haven't updated ragdoll->worldFromModel yet this frame, so this actually the previous worldFromModel
            ragdoll->worldFromModelFadeOutTime = g_currentFrameTime;
            ragdoll->fadeOutWorldFromModel = true;
        }
    }
    ragdoll->wasComputingWorldFromModel = isComputingWorldFromModel;

    if (ragdoll->fadeOutWorldFromModel) {
        if (worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
            hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);

            double elapsedTime = (g_currentFrameTime - ragdoll->worldFromModelFadeOutTime) * *g_globalTimeMultiplier;
            double elapsedTimeFraction = elapsedTime / Config::options.computeWorldFromModelFadeOutTime;

            if (elapsedTimeFraction <= 1.0) {
                worldFromModel = lerphkQsTransform(ragdoll->stickyWorldFromModel, worldFromModel, elapsedTimeFraction);
            }
            else {
                ragdoll->fadeOutWorldFromModel = false;
            }
        }
    }

    if (prevKnockState == KnockState::Ragdoll && ragdoll->knockState == KnockState::BeginGetUp) {
#ifdef _DEBUG
        if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
            _MESSAGE("%d %s: Begin get up", *g_currentFrameCounter, name->name);
        }
#endif // _DEBUG
    }
    else if (prevKnockState == KnockState::BeginGetUp && ragdoll->knockState == KnockState::GetUp) {
        if (Config::options.disableCollisionSoundsWhenGettingUp) {
            *g_fMinSoundVel = Config::options.ragdollSoundVel;
            g_restoreSoundVelTime = g_currentFrameTime + Config::options.getUpDisableCollisionSoundsTime;
        }
#ifdef _DEBUG
        if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
            _MESSAGE("%d %s: Get up", *g_currentFrameCounter, name->name);
        }
#endif // _DEBUG
    }

    if (worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
        hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);
        ragdoll->worldFromModel = worldFromModel;
    }

    bool isInRagdollState = Actor_IsInRagdollState(actor);

    bool isPoweredOnly = !isRigidBodyOn && isPoweredOn;
    if (isPoweredOnly) {
        bool allNoForce = true;
        if (poweredHeader->m_numData > 0) {
            hkbPoweredRagdollControlData *data = (hkbPoweredRagdollControlData *)(Track_getData(generatorOutput, *poweredHeader));
            for (int i = 0; i < poweredHeader->m_numData; i++) {
                hkbPoweredRagdollControlData &elem = data[i];
                if (elem.m_maxForce > 0.f) {
                    allNoForce = false;
                }
            }
        }

        if (Config::options.knockDownAfterBuggedGetUp) {
            if (allNoForce && isUsingRootBoneAsWorldFromModel && !isInRagdollState) {
                if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
                    _MESSAGE("%s bugged out while getting up. Knocking them down again.", name->name);
                }
                if (ActorProcessManager *process = actor->processManager) {
                    ActorProcess_PushActorAway(process, actor, (*g_thePlayer)->pos, 0.f);
                }
            }
        }

        if (allNoForce) {
            // Only powered constraints are active and they are effectively disabled
            return;
        }
    }

    if (!isRigidBodyOn && (!isPoweredOn || !isComputingWorldFromModel)) {
        // No controls are active, or powered only and not computing world from model - try and force it to use the rigidbody controller
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

    if (isPoweredOn && ragdoll->disableConstraintMotorsForOneFrame) {
        ragdoll->disableConstraintMotorsForOneFrame = false;
        poweredHeader->m_onFraction = 0.f;
        isPoweredOn = false;
    }

    if (Config::options.blendWhenGettingUp) {
        if (prevKnockState == KnockState::BeginGetUp && ragdoll->knockState == KnockState::GetUp) {
            ragdoll->blender.StartBlend(Blender::BlendType::RagdollToCurrentRagdoll, g_currentFrameTime, Config::options.getUpBlendInTime, true);
            ragdoll->blender.initialPose = ragdoll->ragdollPose; // the previous ragdoll pose
            ragdoll->blender.isFirstBlendFrame = false;
        }
    }
    ragdoll->wasRigidBodyOn = isRigidBodyOn;

    if (rigidBodyHeader && rigidBodyHeader->m_onFraction > 0.f && rigidBodyHeader->m_numData > 0) {
        hkaKeyFrameHierarchyUtility::ControlData *data = (hkaKeyFrameHierarchyUtility::ControlData *)(Track_getData(generatorOutput, *rigidBodyHeader));

        double snapElapsedTime = (g_currentFrameTime - ragdoll->stateChangedTime) * *g_globalTimeMultiplier;
        bool useSnapController = snapElapsedTime <= Config::options.addToWorldSnapTime;

        for (int i = 0; i < rigidBodyHeader->m_numData; i++) {
            hkaKeyFrameHierarchyUtility::ControlData &elem = data[i];
            elem.m_hierarchyGain = Config::options.hierarchyGain;
            elem.m_velocityGain = Config::options.velocityGain;
            elem.m_positionGain = Config::options.positionGain;

            if (useSnapController) {
                elem.m_snapGain = 1.f;
                elem.m_snapMaxLinearDistance = 100.f;
                elem.m_snapMaxAngularDistance = 100.f;
                elem.m_snapMaxLinearVelocity = 100.f;
                elem.m_snapMaxAngularVelocity = 100.f;
            }
        }
    }

    if (poweredHeader && poweredHeader->m_onFraction > 0.f && poweredHeader->m_numData > 0) {
        hkbPoweredRagdollControlData *data = (hkbPoweredRagdollControlData *)(Track_getData(generatorOutput, *poweredHeader));
        if (isComputingWorldFromModel) {
            for (int i = 0; i < poweredHeader->m_numData; i++) {
                hkbPoweredRagdollControlData &elem = data[i];
                elem.m_maxForce = Config::options.getUpMaxForce;
                elem.m_tau = Config::options.getUpTau;
                elem.m_damping = Config::options.getUpDaming;
                elem.m_proportionalRecoveryVelocity = Config::options.getUpProportionalRecoveryVelocity;
                elem.m_constantRecoveryVelocity = Config::options.getUpConstantRecoveryVelocity;
            }
        }
        else {
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

    ragdoll->isOn = isRigidBodyOn;
    if (!ragdoll->isOn) {
        ragdoll->state = RagdollState::Idle; // reset state
        return;
    }

    if (Actor_IsInRagdollState(actor)) return;

    if (isPoweredOnly) {
        // Don't want to do foot ik / constraint loosening / disabling gravity when powered only
        return;
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
            hkQsTransform *highResPoseLocal = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

            hkStackArray<hkQsTransform> lowResPoseWorld(driver->ragdoll->getNumBones());
            MapHighResPoseLocalToLowResPoseWorld(driver, worldFromModel, highResPoseLocal, lowResPoseWorld.m_data);

            // Set rigidbody transforms to the anim pose ones and save the old values
            static std::vector<hkTransform> savedTransforms{};
            savedTransforms.clear();
            for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
                hkpRigidBody *rb = driver->ragdoll->m_rigidBodies[i];
                hkQsTransform &transform = lowResPoseWorld[i];

                savedTransforms.push_back(rb->getTransform());
                rb->m_motion.getMotionState()->m_transform.m_translation = transform.m_translation;
                hkRotation_setFromQuat(&rb->m_motion.getMotionState()->m_transform.m_rotation, transform.m_rotation);
            }

            if (!ragdoll->easeConstraintsAction) {
                // Loosen ragdoll constraints to allow the anim pose
                hkpEaseConstraintsAction *easeConstraintsAction = hkAllocReferencedObject<hkpEaseConstraintsAction>();
                hkpEaseConstraintsAction_ctor(easeConstraintsAction, (const hkArray<hkpEntity *>&)(driver->ragdoll->getRigidBodyArray()), 0);
                ragdoll->easeConstraintsAction = easeConstraintsAction; // must do this after ctor since this increments the refcount
                hkReferencedObject_removeReference(ragdoll->easeConstraintsAction);

                // Loosen constraint pivots first
                if (Config::options.loosenRagdollConstraintPivots) {
                    ragdoll->originalConstraintPivots.clear();

                    for (hkpConstraintInstance *constraint : driver->ragdoll->getConstraintArray()) {
                        if (constraint->getData()->getType() == hkpConstraintData::CONSTRAINT_TYPE_RAGDOLL) {
                            hkpRagdollConstraintData *data = (hkpRagdollConstraintData *)constraint->getData();

                            if (constraint->getInternal()) { // needed to tell master from slave
                                hkpRigidBody *bodyA = (hkpRigidBody *)constraint->getEntityA();
                                hkpRigidBody *bodyB = (hkpRigidBody *)constraint->getEntityB();

                                hkVector4 pivotAbodySpace = data->m_atoms.m_transforms.m_transformA.m_translation;
                                hkVector4 pivotBbodySpace = data->m_atoms.m_transforms.m_transformB.m_translation;
                                ragdoll->originalConstraintPivots[constraint] = { pivotAbodySpace, pivotBbodySpace };

                                hkVector4 pivotA; hkVector4_setTransformedPos(pivotA, bodyA->getTransform(), pivotAbodySpace);
                                hkVector4 pivotB; hkVector4_setTransformedPos(pivotB, bodyB->getTransform(), pivotBbodySpace);

                                hkVector4 slavePivot = bodyA == constraint->getSlaveEntity() ? pivotA : pivotB;

                                hkpRagdollConstraintData_setPivotInWorldSpace(data, bodyA->getTransform(), bodyB->getTransform(), slavePivot);
                            }
                        }
                    }
                }

                // Loosen angular constraints second
                hkpEaseConstraintsAction_loosenConstraints(ragdoll->easeConstraintsAction);
            }

            // Restore rigidbody transforms
            for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
                hkpRigidBody *rb = driver->ragdoll->m_rigidBodies[i];
                rb->m_motion.getMotionState()->m_transform = savedTransforms[i];
            }
        }
    }

    if (Config::options.disableGravityForActiveRagdolls) {
        for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
            hkpRigidBody *rigidBody = driver->ragdoll->m_rigidBodies[i];
            rigidBody->setGravityFactor(0.f);
        }
    }

    // Root motion
    if (!Config::options.skipRootCalculations) {
        if (bhkCharacterController *controller = GetCharacterController(actor)) {
            if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
                if (driver->ragdoll->getNumBones() > 0) {
                    if (hkpRigidBody *rootRigidBody = driver->ragdoll->getRigidBodyOfBone(0)) {
                        if (NiPointer<NiAVObject> rootNode = GetNodeFromCollidable(rootRigidBody->getCollidable())) {

                            const hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);
                            hkQsTransform *highResPoseLocal = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

                            hkStackArray<hkQsTransform> lowResPoseWorld(driver->ragdoll->getNumBones());
                            MapHighResPoseLocalToLowResPoseWorld(driver, worldFromModel, highResPoseLocal, lowResPoseWorld.m_data);

                            hkQsTransform poseT = lowResPoseWorld[0];

                            if (ragdoll->rootBoneTransform) { // We compare against last frame's pose transform since the rigidbody transforms aren't updated yet for this frame until after the physics step.
                                hkTransform actualT = rootRigidBody->getTransform();

                                NiPoint3 posePos = HkVectorToNiPoint(ragdoll->rootBoneTransform->m_translation);
                                NiPoint3 actualPos = HkVectorToNiPoint(actualT.m_translation);
                                ragdoll->rootOffset = actualPos - posePos;

                                NiPoint3 poseForward = ForwardVector(QuaternionToMatrix(QuaternionNormalized(HkQuatToNiQuat(ragdoll->rootBoneTransform->m_rotation))));
                                NiMatrix33 actualRot; HkMatrixToNiMatrix(actualT.m_rotation, actualRot);
                                NiPoint3 actualForward = ForwardVector(actualRot);
                                float poseAngle = atan2f(poseForward.x, poseForward.y);
                                float actualAngle = atan2f(actualForward.x, actualForward.y);
                                ragdoll->rootOffsetAngle = AngleDifference(poseAngle, actualAngle);

                                if (
                                    Config::options.doWarp &&
                                    (!Config::options.disableWarpWhenGettingUp || ragdoll->knockState != KnockState::GetUp) &&
                                    VectorLength(ragdoll->rootOffset) > Config::options.maxAllowedDistBeforeWarp
                                    ) {
#ifdef _DEBUG
                                    TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);
                                    _MESSAGE("%d %s: Warp", *g_currentFrameCounter, name->name);
#endif // _DEBUG

                                    // Set rigidbody transforms to the anim pose ones
                                    for (int i = 0; i < lowResPoseWorld.m_size; i++) {
                                        hkpRigidBody *rb = driver->ragdoll->getRigidBodyOfBone(i);
                                        if (!rb) continue;

                                        hkQsTransform &transform = lowResPoseWorld[i];

                                        hkTransform newTransform;
                                        newTransform.m_translation = NiPointToHkVector(HkVectorToNiPoint(transform.m_translation));
                                        hkRotation_setFromQuat(&newTransform.m_rotation, transform.m_rotation);

                                        rb->getRigidMotion()->setTransform(newTransform);
                                        hkpEntity_updateMovedBodyInfo(rb);

                                        rb->getRigidMotion()->setLinearVelocity(NiPointToHkVector(NiPoint3()));
                                        rb->getRigidMotion()->setAngularVelocity(NiPointToHkVector(NiPoint3()));
                                    }

                                    // Reset all ragdoll controller state to stop it from going crazy
                                    hkbRagdollDriver_reset(driver);
                                }
                            }

                            ragdoll->rootBoneTransform = poseT;
                        }
                    }
                }
            }
        }
    }
}

void PostDriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext &context, hkbGeneratorOutput &generatorOutput)
{
    // This hook is called right after hkbRagdollDriver::driveToPose()

    Actor *actor = GetActorFromRagdollDriver(driver);
    if (!actor) return;

    std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver);
    if (!ragdoll) return;

    if (!ragdoll->isOn) return;

    int numBones = driver->ragdoll->getNumBones();
    if (numBones <= 0) return;
    ragdoll->stress.clear();

    float totalStress = 0.f;
    for (int i = 0; i < numBones; i++) {
        float stress = sqrtf(g_stressOut[i].m_stressSquared);
        ragdoll->stress.push_back(stress);
        totalStress += stress;
    }

    ragdoll->avgStress = totalStress / numBones;
    //_MESSAGE("stress: %.2f", avgStress);
    //PrintToFile(std::to_string(ragdoll->avgStress), "stress.txt");

    NiPointer<bhkWorld> world = context.world->m_userData;
    if (!world) return;

    /*if (BipedModel *biped = actor->GetBipedSmall()) {
        if (Biped *bipedData = biped->bipedData) {
            // TODO: This doesn't handle weapons equipped in the offhand. We want to include those, but likely ignore shields.
            // Shield / offhand is index 9.

            UInt8 drawState = actor->actorState.flags08 >> 5 & 0x7;
            bool isWeaponDrawn = drawState == 3;
            bool isDead = actor->IsDead(1);
            bool shouldAddToWorld = isWeaponDrawn || isDead;

            for (int i = 32; i < 42; i++) {
                if (NiPointer<NiAVObject> geomNode = bipedData->unk10[i].object) {
                    if (shouldAddToWorld) {
                        ForEachRagdollDriver(actor, [geomNode](hkbRagdollDriver *driver) {
                            if (std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver)) {
                                ragdoll->weaponRoot = geomNode;
                            }
                        });
                    }

                    VisitNodes(geomNode, [i, isDead, shouldAddToWorld, world](NiAVObject *node, int depth) -> bool {
                        if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(node)) {
                            if (shouldAddToWorld) {
                                if (!rigidBody->hkBody->getWorld()) {
                                    _MESSAGE("%d Add: %s", i, node->m_name);
                                    bhkWorld_AddEntity(world, rigidBody->hkBody);

                                    _MESSAGE("%d", GetPartNumber(rigidBody->hkBody));

                                    // Part numbers
                                    // 20 == WEAPON
                                    // 18 == SHIELD
                                }

                                if (!isDead) {
                                    //if (rigidBody->hkBody->getMotionType() != hkpMotion::MotionType::MOTION_KEYFRAMED) {
                                    //	bhkRigidBody_setMotionType(rigidBody, hkpMotion::MotionType::MOTION_KEYFRAMED);
                                    //}
                                    if (!IsMoveableEntity(rigidBody->hkBody)) {
                                        bhkRigidBody_setMotionType(rigidBody, hkpMotion::MotionType::MOTION_DYNAMIC);
                                    }

                                    if (GetCollisionLayer(rigidBody->hkBody) != BGSCollisionLayer::kCollisionLayer_Biped) {
                                        SetCollisionLayer(rigidBody->hkBody, BGSCollisionLayer::kCollisionLayer_Biped);
                                        bhkWorld_UpdateCollisionFilterOnWorldObject(world, rigidBody);
                                    }
                                }
                            }
                            else {
                                if (rigidBody->hkBody->getWorld()) {
                                    _MESSAGE("%d Remove: %s", i, node->m_name);
                                    rigidBody->RemoveFromCurrentWorld();
                                }
                            }
                        }
                        return false;
                    });
                }
            }
        }
    }*/

    //if (NiPointer<NiAVObject> weaponRoot = ragdoll->weaponRoot) {
    //	if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(weaponRoot)) {

            /*
            TODO:
            Instead of this, we could "replace" the hand rigidbodies with the equivalent weapon rigidbody. Or, replace the physics shapes of the hand rigidbodies?

            - clone weapon rigidbody
            - replace hand rigidbody with the cloned one (remove old one from the world, add the new one)

            - swap back to the old hand rigidbody when they die (and drop the weapon)?

            */



            /*
            // TODO: In order to have the weapon act upon the ragdoll, we need to introduce a constraint to binds the weapon to the hand
            // TODO: This should actually be the closest ragdoll rigidbody with collision
            // TODO: We also want to make sure it's actually a hand, as there can be frames when initially unsheathing where it is still parented to the pelvis
            if (NiPointer<NiAVObject> parent = GetClosestParentWithCollision(weaponRoot, true)) {
                // assume the parent is the hand
                if (NiPointer<bhkRigidBody> handBody = GetRigidBody(parent)) {
                    NiTransform handTransform = hkTransformToNiTransform(handBody->hkBody->getTransform(), 1.f);
                    NiTransform weaponTransform = hkTransformToNiTransform(rigidBody->hkBody->getTransform(), 1.f);

                    //NiTransform handToWeapon = InverseTransform(handTransform) * weaponTransform;
                    NiTransform handToWeapon = InverseTransform(parent->m_worldTransform) * weaponRoot->m_worldTransform;
                    NiTransform desiredWeapon = handTransform * handToWeapon;

                    // figure out where the hand is trying to move (current + velocity * deltaTime)

                    // apply hard keyframe to weapon to reach the transform that it would be at if the hand were at its desired transform

                    hkTransform hkDesiredWeapon = NiTransformTohkTransform(desiredWeapon);
                    hkpKeyFrameUtility_applyHardKeyFrame(hkDesiredWeapon.m_translation, NiQuatToHkQuat(MatrixToQuaternion(HkMatrixToNiMatrix(hkDesiredWeapon.m_rotation))), 1.f / *g_deltaTime, rigidBody->hkBody);
                }
            }
            */
            //}
        //}

    if (Config::options.disableConstraints) {
        for (hkpConstraintInstance *constraint : driver->ragdoll->m_constraints) {
            hkpConstraintInstance_setEnabled(constraint, false);
        }
    }
}

hkbRagdollDriver *g_currentPostPhysicsDriver = nullptr;

void PrePostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &generatorOutput)
{
    // This hook is called right before hkbRagdollDriver::postPhysics()

    g_currentPostPhysicsDriver = driver;

    Actor *actor = GetActorFromRagdollDriver(driver);
    if (!actor) return;

    // All we're doing here is storing the anim pose, so it's fine to run this even if the actor is fully ragdolled or getting up.

    std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver);
    if (!ragdoll) return;

    hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POSE);
    if (poseHeader && poseHeader->m_onFraction > 0.f) {
        int numPoses = poseHeader->m_numData;
        hkQsTransform *animPose = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);
        // Copy anim pose track before postPhysics() as postPhysics() will overwrite it with the ragdoll pose
        ragdoll->animPose.assign(animPose, animPose + numPoses);
    }
}

void PostPostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &generatorOutput)
{
    // This hook is called right after hkbRagdollDriver::postPhysics()

    g_currentPostPhysicsDriver = nullptr;

    Actor *actor = GetActorFromRagdollDriver(driver);
    if (!actor) return;

    // All we're doing here is storing the ragdoll pose and blending, and we do want to have the option to blend even while getting up.

    std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver);
    if (!ragdoll) return;

    hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_POSE);

    if (poseHeader && poseHeader->m_onFraction > 0.f) {
        int numPoses = poseHeader->m_numData;
        hkQsTransform *poseOut = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

        // Copy pose track now since postPhysics() just set it to the high-res ragdoll pose
        ragdoll->ragdollPose.assign(poseOut, poseOut + numPoses);
    }

    hkbGeneratorOutput::TrackHeader *worldFromModelHeader = GetTrackHeader(generatorOutput, hkbGeneratorOutput::StandardTracks::TRACK_WORLD_FROM_MODEL);
    if (worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
        hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);
        ragdoll->worldFromModelPostPhysics = worldFromModel;
    }

    if (!ragdoll->isOn) return;

    if (ragdoll->fadeInWorldFromModel) {
        double elapsedTime = (g_currentFrameTime - ragdoll->worldFromModelFadeInTime) * *g_globalTimeMultiplier;
        double elapsedTimeFraction = elapsedTime / Config::options.computeWorldFromModelFadeInTime;
        if (elapsedTimeFraction > 1.0) {
            ragdoll->fadeInWorldFromModel = false;
        }
    }

    RagdollState state = ragdoll->state;

    if (Config::options.loosenRagdollContraintsToMatchPose) {
        if (ragdoll->easeConstraintsAction) {
            // Restore constraint limits from before we loosened them
            // TODO: Can the character die between drivetopose and postphysics? If so, we should do this if the ragdoll character dies too.

            hkpEaseConstraintsAction_restoreConstraints(ragdoll->easeConstraintsAction, 0.f);
            ragdoll->easeConstraintsAction = nullptr;

            if (Config::options.loosenRagdollConstraintPivots) {
                for (hkpConstraintInstance *constraint : driver->ragdoll->getConstraintArray()) {
                    if (constraint->getData()->getType() == hkpConstraintData::CONSTRAINT_TYPE_RAGDOLL) {
                        hkpRagdollConstraintData *data = (hkpRagdollConstraintData *)constraint->getData();

                        if (auto it = ragdoll->originalConstraintPivots.find(constraint); it != ragdoll->originalConstraintPivots.end()) {
                            data->m_atoms.m_transforms.m_transformA.m_translation = it->second.first;
                            data->m_atoms.m_transforms.m_transformB.m_translation = it->second.second;
                        }
                    }
                }
            }
        }
    }

    if (Config::options.disableGravityForActiveRagdolls) {
        for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
            hkpRigidBody *rigidBody = driver->ragdoll->m_rigidBodies[i];
            rigidBody->setGravityFactor(1.f);
        }
    }

    // TODO: Figure out blending fully in world space

    Blender &blender = ragdoll->blender;
    if (blender.isActive) {
        bool done = !Config::options.doBlending;
        if (!done) {
            done = blender.Update(*ragdoll, *driver, generatorOutput, g_currentFrameTime);
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
        if (poseHeader && poseHeader->m_onFraction > 0.f && ragdoll->animPose.data()) {
            int numPoses = poseHeader->m_numData;
            hkQsTransform *poseOut = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);
            memcpy(poseOut, ragdoll->animPose.data(), ragdoll->animPose.size() * sizeof(hkQsTransform));
        }
    }
    else if (Config::options.forceRagdollPose) {
        if (poseHeader && poseHeader->m_onFraction > 0.f && ragdoll->ragdollPose.data()) {
            int numPoses = poseHeader->m_numData;
            hkQsTransform *poseOut = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);
            memcpy(poseOut, ragdoll->ragdollPose.data(), ragdoll->ragdollPose.size() * sizeof(hkQsTransform));
        }
    }

    ragdoll->state = state;
}

_hkbRagdollDriver_driveToPose hkbRagdollDriver_driveToPose_Original = 0;
void DriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext &context, hkbGeneratorOutput &generatorOutput)
{
    PreDriveToPoseHook(driver, deltaTime, context, generatorOutput);
    hkbRagdollDriver_driveToPose_Original(driver, deltaTime, context, generatorOutput);
    PostDriveToPoseHook(driver, deltaTime, context, generatorOutput);
}

_hkbRagdollDriver_postPhysics hkbRagdollDriver_postPhysics_Original = 0;
void PostPhysicsHook(hkbRagdollDriver *driver, const hkbContext &context, hkbGeneratorOutput &inOut)
{
    PrePostPhysicsHook(driver, context, inOut);
    hkbRagdollDriver_postPhysics_Original(driver, context, inOut);
    PostPostPhysicsHook(driver, context, inOut);
}

void PreCullActorsHook(Actor *actor)
{
    if (!g_activeActors.count(actor)) return; // let the game decide

    UInt32 cullState = 7; // do not cull this actor

    actor->unk274 &= 0xFFFFFFF0;
    actor->unk274 |= cullState & 0xF;
}

_BShkbAnimationGraph_UpdateAnimation BShkbAnimationGraph_UpdateAnimation_Original = 0;
void BShkbAnimationGraph_UpdateAnimation_Hook(BShkbAnimationGraph *_this, BShkbAnimationGraph::UpdateData *updateData, void *a3)
{
    Actor *actor = _this->holder;
    if (a3 && actor && g_activeActors.count(actor)) { // a3 is null if the graph is not active
        updateData->unk2A = true; // forces animation update (hkbGenerator::generate()) without skipping frames
    }

    BShkbAnimationGraph_UpdateAnimation_Original(_this, updateData, a3);
}

_Actor_GetHit Actor_TakePhysicsDamage_Original = 0;
void Actor_TakePhysicsDamage_Hook(Actor *_this, HitData &hitData)
{
    // We do our own physics damage for active actors, so don't have the game do its as well
    if (g_activeActors.count(_this)) return;
    Actor_TakePhysicsDamage_Original(_this, hitData);
}

_Actor_EndHavokHit Actor_KillEndHavokHit_Original = 0;
void Actor_KillEndHavokHit_Hook(HavokHitJobs *havokHitJobs, Actor *_this)
{
    // The actor is dying so undo anything we've done to it
    if (g_activeActors.count(_this)) {
        if (Config::options.disableGravityForActiveRagdolls) {
            EnableGravity(_this);
        }
    }

    Actor_KillEndHavokHit_Original(havokHitJobs, _this);
}

void MovementControllerUpdateHook(MovementControllerNPC *movementController, Actor *actor)
{

    // Do movement jobs here
    {
        std::scoped_lock lock(g_bumpActorsLock);
        if (auto it = g_bumpActors.find(actor); it != g_bumpActors.end()) {
            BumpActor(actor, it->second.bumpDirection, it->second.isLargeBump, it->second.exitFurniture, it->second.pauseCurrentDialogue, it->second.triggerDialogue);
            if (it->second.isLargeBump) {
                std::scoped_lock lock2(g_shoveTimesLock);
                g_shoveData[&actor->animGraphHolder] = { it->second.bumpDirectionVec, g_currentFrameTime };
            }
            g_bumpActors.erase(it);
        }
    }

    MovementControllerNPC_Update(movementController);
}

void ActorProcess_ExitFurniture_RemoveCollision_Hook(BSTaskPool *taskPool, NiAVObject *root)
{
    TESObjectREFR *refr = NiAVObject_GetOwner(root);
    if (g_activeActors.count(static_cast<Actor *>(refr))) return;
    BSTaskPool_QueueRemoveCollisionFromWorld(taskPool, root);
}

void ActorProcess_ExitFurniture_ResetRagdoll_Hook(BSAnimationGraphManager *manager, bool *a2, Actor *actor)
{
    if (g_activeActors.count(actor)) {
        if (Config::options.disableConstraintMotorsOnFurnitureExit) {
            ForEachRagdollDriver(manager, [](hkbRagdollDriver *driver) {
                if (std::shared_ptr<ActiveRagdoll> activeRagdoll = GetActiveRagdollFromDriver(driver)) {
                    activeRagdoll->disableConstraintMotorsForOneFrame = true;
                }
                });
        }

        return;
    }
    BSAnimationGraphManager_ResetRagdoll(manager, a2);
}

void ActorProcess_EnterFurniture_SetWorld_Hook(BSAnimationGraphManager *manager, UInt64 *a2, Actor *actor)
{
    if (g_activeActors.count(actor)) return;
    BSAnimationGraphManager_SetWorld(manager, a2);
}

bool IsPhysicallyBlocked(Actor *attacker)
{
    if (auto it = g_physicsListener.physicalBlockTimes.find(attacker); it != g_physicsListener.physicalBlockTimes.end()) {
        _MESSAGE("%d: %.2f", *g_currentFrameCounter, g_currentFrameTime - it->second);
        if (g_currentFrameTime - it->second < Config::options.physicalBlockWindow) {
            return true;
        }
    }

    return false;
}

_HitData_populate Character_HitTarget_HitData_populate_Original = 0;
void Character_HitTarget_HitData_Populate_Hook(HitData *hitData, Actor *srcRefr, Actor *targetRefr, InventoryEntryData *weapon, bool isOffhand)
{
    if (targetRefr == *g_thePlayer) {
        // TODO: This works, but the game still does the cone check so I think it's possible it does't work if we aren't looking at the enemy?

        if (IsPhysicallyBlocked(srcRefr)) {
            bool wasBlocking = Actor_IsBlocking(targetRefr);

            static BSFixedString sIsBlocking("IsBlocking");
            IAnimationGraphManagerHolder_SetAnimationVariableBool(&targetRefr->animGraphHolder, sIsBlocking, true);

            Character_HitTarget_HitData_populate_Original(hitData, srcRefr, targetRefr, weapon, isOffhand);

            IAnimationGraphManagerHolder_SetAnimationVariableBool(&targetRefr->animGraphHolder, sIsBlocking, wasBlocking);
        }
        else {
            Character_HitTarget_HitData_populate_Original(hitData, srcRefr, targetRefr, weapon, isOffhand);
        }
    }
    else {
        Character_HitTarget_HitData_populate_Original(hitData, srcRefr, targetRefr, weapon, isOffhand);
    }

    if (g_isInPlanckHit) {
        // Set an unused bit to signify this hit will have additional info from planck
        hitData->flags |= (1 << 30);
    }
}

void ScriptEventSourceHolder_DispatchHitEventFromHitData_DispatchHitEvent_Hook(EventDispatcher<TESHitEvent> *dispatcher, TESHitEvent *hitEvent, HitData *hitData)
{
    if (hitData->flags >> 30 & 1) {
#ifdef _DEBUG
        _MESSAGE("%d Dispatch hit", *g_currentFrameCounter);
#endif // _DEBUG

        PlanckPluginAPI::PlanckHitEvent extendedHitEvent{ hitEvent->target, hitEvent->caster, hitEvent->sourceFormID, hitEvent->projectileFormID, hitEvent->flags };
        extendedHitEvent.hitData = hitData;
        extendedHitEvent.extendedHitData = g_interface001.lastHitData;
        extendedHitEvent.flags &= ~(0xFFFFFF00); // zero out top 3 bytes
        extendedHitEvent.flags |= PlanckPluginAPI::hitEventMagicNumber; // hopefully this never occurs randomly...

        g_interface001.currentHitEvent = &extendedHitEvent;
        dispatcher->SendEvent(&extendedHitEvent);
        g_interface001.currentHitEvent = nullptr;
    }
    else {
        dispatcher->SendEvent(hitEvent);
    }
}

// Hooks the IAnimationGraphManagerHolder::GetAnimationVariableBool vfunc call in the papyrus GetAnimationVariableBool func
bool Papyrus_IAnimationGraphManagerHolder_GetAnimationVariableBool_Hook(IAnimationGraphManagerHolder *_this, const BSFixedString &a_variableName, bool &a_out)
{
    IAnimationGraphManagerHolder *playerAnimGraphHolder = &(*g_thePlayer)->animGraphHolder;
    if (_this == playerAnimGraphHolder) {
        if (Config::options.spoofbAllowRotationDuringSwing) {
            static BSFixedString bAllowRotationStr("bAllowRotation");
            if (a_variableName == bAllowRotationStr) {
                if (g_rightSwingHandler.IsPowerAttackActive() || g_leftSwingHandler.IsPowerAttackActive()) {
                    a_out = true;
                    return true;
                }
            }
        }

        if (Config::options.spoofIsBashingDuringSwing) {
            static BSFixedString IsBashingStr("IsBashing");
            if (a_variableName == IsBashingStr) {
                if (g_rightSwingHandler.IsBashActive() || g_leftSwingHandler.IsBashActive()) {
                    a_out = true;
                    return true;
                }
            }
        }
    }

    return get_vfunc<_IAnimationGraphManagerHolder_GetAnimationVariableBool>(_this, 0x12)(_this, a_variableName, a_out);
}

// Return true to let the base game run its code, false to return immediately
bool WeaponRightSwingHandler_Handle_Hook(void *_this, Actor *actor)
{
    if (actor == *g_thePlayer) {
#ifdef _DEBUG
        _MESSAGE("%d WeaponRightSwing", *g_currentFrameCounter);
#endif // _DEBUG
        // Don't do the weapon swing sound for the player

        return false;

        // OLD
        SetAttackState(actor, 2); // kSwing

        get_vfunc<_Actor_WeaponSwingCallback>(actor, 0xF1)(actor);

        return false;
    }
    return true;
}

// Return true to let the base game run its code, false to return immediately
bool WeaponLeftSwingHandler_Handle_Hook(void *_this, Actor *actor)
{
    if (actor == *g_thePlayer) {
#ifdef _DEBUG
        _MESSAGE("%d WeaponLeftSwing", *g_currentFrameCounter);
#endif // _DEBUG
        // Don't do the weapon swing sound for the player

        return false;

        // OLD
        SetAttackState(actor, 2); // kSwing

        get_vfunc<_Actor_WeaponSwingCallback>(actor, 0xF1)(actor);

        return false;
    }
    return true;
}

// HitFrameHandler::Handle hook for disabling animation-driven power attack hits for the player
_HitFrameHandler_Handle g_originalHitFrameHandlerHandle = nullptr;
static RelocPtr<_HitFrameHandler_Handle> HitFrameHandler_Handle_vtbl(0x16F5148); // 0x16F5140 + 8
bool HitFrameHandler_Handle_Hook(void *_this, Actor *actor, BSFixedString *side) // side == "Left" when offhand, "" when main hand
{
    if (actor == *g_thePlayer) {
#ifdef _DEBUG
        _MESSAGE("%d HitFrame", *g_currentFrameCounter);
#endif // _DEBUG
        // Ignore hitframe power attack hits for the player

        static std::string left("Left");
        bool isLeft = left == side->c_str();

        SwingHandler &swingHandler = isLeft ? g_leftSwingHandler : g_rightSwingHandler;
        return swingHandler.IsSwingActive(); // swing happened recently

        // OLD
        if (GetAttackState(actor) != 2) { // != kSwing
            return false;
        }

        SetAttackState(actor, 3); // kSwing
        return true;
    }
    return g_originalHitFrameHandlerHandle(_this, actor, side);
}

_AttackHandler_Handle g_originalAttackWinStartHandlerHandle = nullptr;
static RelocPtr<_AttackHandler_Handle> AttackWinStartHandler_Handle_vtbl(0x16F4FC8); // 0x16F4FC0 + 8
bool AttackWinStartHandler_Handle_Hook(void *_this, Actor *actor)
{
    if (actor == *g_thePlayer) {
#ifdef _DEBUG
        _MESSAGE("%d AttackWinStart", *g_currentFrameCounter);
#endif // _DEBUG

        return true;
    }
    return g_originalAttackWinStartHandlerHandle(_this, actor);
}

_AttackHandler_Handle g_originalAttackWinEndHandlerHandle = nullptr;
static RelocPtr<_AttackHandler_Handle> AttackWinEndHandler_Handle_vtbl(0x16F4FE0); // 0x16F4FD8 + 8
bool AttackWinEndHandler_Handle_Hook(void *_this, Actor *actor)
{
    if (actor == *g_thePlayer) {
#ifdef _DEBUG
        _MESSAGE("%d AttackWinEnd", *g_currentFrameCounter);
#endif // _DEBUG

        return true;
    }
    return g_originalAttackWinEndHandlerHandle(_this, actor);
}

_AttackHandler_Handle g_originalAttackStopHandlerHandle = nullptr;
static RelocPtr<_AttackHandler_Handle> AttackStopHandler_Handle_vtbl(0x16F4FF8); // 0x16F4FF0 + 8
bool AttackStopHandler_Handle_Hook(void *_this, Actor *actor)
{
    if (actor == *g_thePlayer) {
#ifdef _DEBUG
        _MESSAGE("%d AttackStop", *g_currentFrameCounter);
#endif // _DEBUG

        // Still unset attackState and attackData for non-swings (switching weapons, crossbow shots?)
        if (g_leftSwingHandler.IsSwingActive() || g_rightSwingHandler.IsSwingActive()) {
            // swing happened recently, so don't do anything
            return true;
        }
    }
    return g_originalAttackStopHandlerHandle(_this, actor);
}

_Actor_IsRagdollMovingSlowEnoughToGetUp Actor_IsRagdollMovingSlowEnoughToGetUp_Original = 0;
bool Actor_IsRagdollMovingSlowEnoughToGetUp_Hook(Actor *actor)
{
    if (g_rightHeldRefr == actor || g_leftHeldRefr == actor) {
        if (Config::options.resetFallStateWhenRagdollIsGrabbed) {
            if (bhkCharacterController *controller = GetCharacterController(actor)) {
                // Reset fall height/time while we have them grabbed, so that grabbing them and carrying them somewhere lower than they were ragdolled doesn't trigger fall damage
                hkVector4 controllerPos; controller->GetPositionImpl(controllerPos, true);
                controller->fallTime = 0.f;
                controller->fallStartHeight = HkVectorToNiPoint(controllerPos).z * *g_inverseHavokWorldScale;
            }
        }
        if (Config::options.preventGetUpWhenRagdollIsGrabbed) {
            if (Config::options.resetKnockedDownTimerWhenRagdollIsGrabbed) {
                if (ActorProcessManager *process = actor->processManager) {
                    *(float *)&process->middleProcess->unk2B0 = *g_fExplosionKnockStateExplodeDownTime; // reset knocked down timer - it's 15 seconds by default
                }
            }
            return false;
        }
    }

    return Actor_IsRagdollMovingSlowEnoughToGetUp_Original(actor);
}

_BSAnimationGraphManager_RemoveRagdollFromWorld GetUpEnd_BSAnimationGraphManager_RemoveRagdollFromWorld_Original = 0;
void GetUpEnd_RemoveRagdollFromWorld_Hook(BSAnimationGraphManager *graphManager, bool *result)
{
    {
        SimpleLocker lock(&graphManager->updateLock);
        for (int i = 0; i < graphManager->graphs.size; i++) {
            BSTSmartPointer<BShkbAnimationGraph> graph = graphManager->graphs.GetData()[i];
            if (!graph.ptr) continue;

            Actor *actor = graph.ptr->holder;
            if (!actor) continue;

            if (g_activeActors.count(actor)) {
                *result = true; // act as if we successfully removed the ragdoll from the world
                return;
            }
        }
    }

    GetUpEnd_BSAnimationGraphManager_RemoveRagdollFromWorld_Original(graphManager, result);
}

_NiNode_SetMotionTypeDownwards GetUpEnd_NiNode_SetMotionTypeDownwards_Original = 0;
void GetUpEnd_NiNode_SetMotionTypeKeyframed_Hook(NiNode *node, UInt32 motionType, bool a3, bool a4, UInt32 a5)
{
    TESObjectREFR *refr = NiAVObject_GetOwner(node);
    if (refr && g_activeActors.count(static_cast<Actor *>(refr))) return;

    GetUpEnd_NiNode_SetMotionTypeDownwards_Original(node, motionType, a3, a4, a5);
}

_BSTaskPool_QueueRemoveCollisionFromWorld GetUpEndHandler_Handle_RemoveCollision_BSTaskPool_QueueRemoveCollisionFromWorld_Original = 0;
void GetUpEndHandler_Handle_RemoveCollision_Hook(BSTaskPool *taskPool, NiAVObject *node)
{
    TESObjectREFR *refr = NiAVObject_GetOwner(node);
    if (refr && g_activeActors.count(static_cast<Actor *>(refr))) return;

    GetUpEndHandler_Handle_RemoveCollision_BSTaskPool_QueueRemoveCollisionFromWorld_Original(taskPool, node);
}

_hkaPoseMatchingUtility_computeReferenceFrame hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_Original = 0;
void hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_Hook(hkaPoseMatchingUtility *_this, const hkQsTransform *animPoseModelSpace, const hkQsTransform *ragdollPoseWorldSpace, hkQsTransform &animWorldFromModel, hkQsTransform &ragdollWorldFromModel)
{
    hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_Original(_this, animPoseModelSpace, ragdollPoseWorldSpace, animWorldFromModel, ragdollWorldFromModel);

    if (hkbRagdollDriver *driver = g_currentPostPhysicsDriver) {
        if (Actor *actor = GetActorFromRagdollDriver(driver)) {
            if (std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver)) {
                if (ragdoll->fadeInWorldFromModel) {
                    double elapsedTime = (g_currentFrameTime - ragdoll->worldFromModelFadeInTime) * *g_globalTimeMultiplier;
                    double elapsedTimeFraction = elapsedTime / Config::options.computeWorldFromModelFadeInTime;
                    if (elapsedTimeFraction <= 1.0) {
                        animWorldFromModel = lerphkQsTransform(ragdoll->stickyWorldFromModel, animWorldFromModel, elapsedTimeFraction);
                        ragdollWorldFromModel = lerphkQsTransform(ragdoll->stickyWorldFromModel, ragdollWorldFromModel, elapsedTimeFraction);
                    }
                }
            }
        }
    }
}

_hkpMotion_approxTransformAt hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_Original = 0;
void hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_Hook(hkpMotion *_this, hkTime time, hkTransform &out)
{
    hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_Original(_this, time, out);

    if (!Config::options.applyRigidBodyTWhenReadingRigidBodies) return;

    hkpRigidBody *rigidBody = (hkpRigidBody *)(((UInt64)_this) - 0x150);
    if (bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData) {
        NiTransform rigidBodyTLocalTransform = GetRigidBodyTLocalTransform(wrapper);

        NiTransform transform = hkTransformToNiTransform(out, 1.f);
        transform = transform * InverseTransform(rigidBodyTLocalTransform);
        out = NiTransformTohkTransform(transform);
    }
}

_hkaSkeletonMapper_mapPose MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_1_Original = 0;
void MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_Hook_1(hkaSkeletonMapper *_this, const hkQsTransform *poseAModelSpace, const hkQsTransform *originalPoseBLocalSpace, hkQsTransform *poseBModelSpaceInOut, UInt32 source)
{
    UInt32 sourceOverride = Config::options.dontRestrictBoneLengthsWhenMappingFromRagdollToAnim ? hkaSkeletonMapper::ConstraintSource::NO_CONSTRAINTS : source;
    MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_1_Original(_this, poseAModelSpace, originalPoseBLocalSpace, poseBModelSpaceInOut, sourceOverride);
}

_hkaSkeletonMapper_mapPose MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_2_Original = 0;
void MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_Hook_2(hkaSkeletonMapper *_this, const hkQsTransform *poseAModelSpace, const hkQsTransform *originalPoseBLocalSpace, hkQsTransform *poseBModelSpaceInOut, UInt32 source)
{
    UInt32 sourceOverride = Config::options.dontRestrictBoneLengthsWhenMappingFromRagdollToAnim ? hkaSkeletonMapper::ConstraintSource::NO_CONSTRAINTS : source;
    MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_2_Original(_this, poseAModelSpace, originalPoseBLocalSpace, poseBModelSpaceInOut, sourceOverride);
}

_hkaSkeletonMapper_mapPose hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_Original = 0;
void hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_Hook(hkaSkeletonMapper *_this, const hkQsTransform *poseAModelSpace, const hkQsTransform *originalPoseBLocalSpace, hkQsTransform *poseBModelSpaceInOut, UInt32 source)
{
    UInt32 sourceOverride = Config::options.dontRestrictBoneLengthsWhenMappingFromAnimToRagdoll ? hkaSkeletonMapper::ConstraintSource::NO_CONSTRAINTS : source;
    hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_Original(_this, poseAModelSpace, originalPoseBLocalSpace, poseBModelSpaceInOut, sourceOverride);
}

_hkaRagdollRigidBodyController_driveToPose hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_Original = 0;
void hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_Hook(hkaRagdollRigidBodyController *_this, hkReal deltaTime, hkQsTransform *poseLocalSpace, const hkQsTransform &worldFromModel, hkaKeyFrameHierarchyUtility::Output *stressOut)
{
    if (Config::options.applyRigidBodyTWhenWritingRigidBodies) {
        ApplyRigidBodyTTransformsToPose(_this->m_ragdollInstance, worldFromModel, poseLocalSpace, poseLocalSpace);
    }

    // The 4th arg to hkaRagdollRigidBodyController::driveToPose is a ptr which it sets to the output stress on the ragdoll.
    // The game passes 0 in this arg normally, which means the stress is not extracted, so we extract it ourselves.
    hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_Original(_this, deltaTime, poseLocalSpace, worldFromModel, g_stressOut);
}

auto processHavokHitJobsHookLoc = RelocAddr<uintptr_t>(0x6497E4);

auto PlayerCharacter_UpdateWeaponSwing_HookLoc = RelocAddr<uintptr_t>(0x6ABCA4);

auto postPhysicsHookLoc = RelocAddr<uintptr_t>(0xB268DC);

auto driveToPoseHookLoc = RelocAddr<uintptr_t>(0xB266AB);

auto hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_HookLoc = RelocAddr<uintptr_t>(0xA26C05);

auto potentiallyEnableMeleeCollisionLoc = RelocAddr<uintptr_t>(0x6E5366);

auto preCullActorsHookLoc = RelocAddr<uintptr_t>(0x69F4B9);

auto BShkbAnimationGraph_UpdateAnimation_HookLoc = RelocAddr<uintptr_t>(0xB1CB55);

auto Actor_TakePhysicsDamage_HookLoc = RelocAddr<uintptr_t>(0x61F6E7);

auto Character_MovementControllerUpdate_HookLoc = RelocAddr<uintptr_t>(0x5E086F);

auto Actor_KillEndHavokHit_HookLoc = RelocAddr<uintptr_t>(0x60C808);

auto ActorProcess_ExitFurniture_RemoveCollision_HookLoc = RelocAddr<uintptr_t>(0x687FF7);
auto ActorProcess_ExitFurniture_ResetRagdoll_HookLoc = RelocAddr<uintptr_t>(0x688061);
auto ActorProcess_EnterFurniture_SetWorld_HookLoc = RelocAddr<uintptr_t>(0x68E344); // sets the world to null when entering furniture

auto Character_HitTarget_HitData_Populate_HookLoc = RelocAddr<uintptr_t>(0x631CA7);
auto Actor_GetHit_DispatchHitEventFromHitData_HookLoc = RelocAddr<uintptr_t>(0x62F3DA);
auto ScriptEventSourceHolder_DispatchHitEventFromHitData_DispatchHitEvent_HookLoc = RelocAddr<uintptr_t>(0x636742);

auto Papyrus_IAnimationGraphManagerHolder_GetAnimationVariableBool_HookLoc = RelocAddr<uintptr_t>(0x9CE8D0);

auto Actor_IsRagdollMovingSlowEnoughToGetUp_HookLoc = RelocAddr<uintptr_t>(0x687164);

auto GetUpStart_ZeroOutPitchRoll_Loc = RelocAddr<uintptr_t>(0x74D65B);

auto GetUpEnd_RemoveRagdollFromWorld_HookLoc = RelocAddr<uintptr_t>(0x68727D);
auto GetUpEnd_SetMotionTypeKeyframed_HookLoc = RelocAddr<uintptr_t>(0x6872D0);
auto GetUpEndHandler_Handle_RemoveCollision_HookLoc = RelocAddr<uintptr_t>(0x74D816);

auto hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_HookLoc = RelocAddr<uintptr_t>(0xA2921E);

auto hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_HookLoc = RelocAddr<uintptr_t>(0xB51008);

auto MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_HookLoc_1 = RelocAddr<uintptr_t>(0xA374B6);

auto MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_HookLoc_2 = RelocAddr<uintptr_t>(0xA37533);

auto hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_HookLoc = RelocAddr<uintptr_t>(0xA28005);


std::uintptr_t Write5Call(std::uintptr_t a_src, std::uintptr_t a_dst)
{
    const auto disp = reinterpret_cast<std::int32_t *>(a_src + 1);
    const auto nextOp = a_src + 5;
    const auto func = nextOp + *disp;

    g_branchTrampoline.Write5Call(a_src, a_dst);

    return func;
}

void PerformHooks(void)
{
    {
        std::uintptr_t originalFunc = Write5Call(processHavokHitJobsHookLoc.GetUIntPtr(), uintptr_t(ProcessHavokHitJobsHook));
        ProcessHavokHitJobs_Original = (_ProcessHavokHitJobs)originalFunc;
        _MESSAGE("ProcessHavokHitJobs hook complete");
    }

    {
        g_branchTrampoline.Write5Call(PlayerCharacter_UpdateWeaponSwing_HookLoc.GetUIntPtr(), uintptr_t(PlayerCharacter_UpdateWeaponSwing_Hook));
        _MESSAGE("PlayerCharacter::UpdateWeaponSwing hook complete");
    }

    if (Config::options.forceGenerateForActiveRagdolls) {
        std::uintptr_t originalFunc = Write5Call(BShkbAnimationGraph_UpdateAnimation_HookLoc.GetUIntPtr(), uintptr_t(BShkbAnimationGraph_UpdateAnimation_Hook));
        BShkbAnimationGraph_UpdateAnimation_Original = (_BShkbAnimationGraph_UpdateAnimation)originalFunc;
        _MESSAGE("BShkbAnimationGraph::UpdateAnimation hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(driveToPoseHookLoc.GetUIntPtr(), uintptr_t(DriveToPoseHook));
        hkbRagdollDriver_driveToPose_Original = (_hkbRagdollDriver_driveToPose)originalFunc;
        _MESSAGE("hkbRagdollDriver::driveToPose hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(postPhysicsHookLoc.GetUIntPtr(), uintptr_t(PostPhysicsHook));
        hkbRagdollDriver_postPhysics_Original = (_hkbRagdollDriver_postPhysics)originalFunc;
        _MESSAGE("hkbRagdollDriver::postPhysics hook complete");
    }

    if (Config::options.doClutterVsBipedCollisionDamage) {
        std::uintptr_t originalFunc = Write5Call(Actor_TakePhysicsDamage_HookLoc.GetUIntPtr(), uintptr_t(Actor_TakePhysicsDamage_Hook));
        Actor_TakePhysicsDamage_Original = (_Actor_GetHit)originalFunc;
        _MESSAGE("Actor take physics damage hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(Actor_KillEndHavokHit_HookLoc.GetUIntPtr(), uintptr_t(Actor_KillEndHavokHit_Hook));
        Actor_KillEndHavokHit_Original = (_Actor_EndHavokHit)originalFunc;
        _MESSAGE("Actor kill end havok hit hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(Character_HitTarget_HitData_Populate_HookLoc.GetUIntPtr(), uintptr_t(Character_HitTarget_HitData_Populate_Hook));
        Character_HitTarget_HitData_populate_Original = (_HitData_populate)originalFunc;
        _MESSAGE("Character_HitTarget_HitData_Populate hook complete");
    }

    {
        g_branchTrampoline.Write6Call(Papyrus_IAnimationGraphManagerHolder_GetAnimationVariableBool_HookLoc.GetUIntPtr(), uintptr_t(Papyrus_IAnimationGraphManagerHolder_GetAnimationVariableBool_Hook));
        _MESSAGE("Papyrus_IAnimationGraphManagerHolder_GetAnimationVariableBool hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(Actor_IsRagdollMovingSlowEnoughToGetUp_HookLoc.GetUIntPtr(), uintptr_t(Actor_IsRagdollMovingSlowEnoughToGetUp_Hook));
        Actor_IsRagdollMovingSlowEnoughToGetUp_Original = (_Actor_IsRagdollMovingSlowEnoughToGetUp)originalFunc;
        _MESSAGE("Actor_IsRagdollMovingSlowEnoughToGetUp hook complete");
    }

    if (Config::options.dontResetPitchRollWhenGettingUp) {
        char *nops = "\x90\x90\x90\x90\x90\x90\x90\x90\x90\x90\x90"; // "\x90" * 11
        SafeWriteBuf(GetUpStart_ZeroOutPitchRoll_Loc.GetUIntPtr(), nops, 11);
        _MESSAGE("NOP'd out zeroing out of pitch/roll when starting to get up");
    }

    if (Config::options.dontRemoveRagdollWhenDoneGettingUp) {
        std::uintptr_t originalFunc = Write5Call(GetUpEnd_RemoveRagdollFromWorld_HookLoc.GetUIntPtr(), uintptr_t(GetUpEnd_RemoveRagdollFromWorld_Hook));
        GetUpEnd_BSAnimationGraphManager_RemoveRagdollFromWorld_Original = (_BSAnimationGraphManager_RemoveRagdollFromWorld)originalFunc;

        originalFunc = Write5Call(GetUpEnd_SetMotionTypeKeyframed_HookLoc.GetUIntPtr(), uintptr_t(GetUpEnd_NiNode_SetMotionTypeKeyframed_Hook));
        GetUpEnd_NiNode_SetMotionTypeDownwards_Original = (_NiNode_SetMotionTypeDownwards)originalFunc;

        originalFunc = Write5Call(GetUpEndHandler_Handle_RemoveCollision_HookLoc.GetUIntPtr(), uintptr_t(GetUpEndHandler_Handle_RemoveCollision_Hook));
        GetUpEndHandler_Handle_RemoveCollision_BSTaskPool_QueueRemoveCollisionFromWorld_Original = (_BSTaskPool_QueueRemoveCollisionFromWorld)originalFunc;

        _MESSAGE("Stopped the game from removing/resetting the ragdoll when a character has finished getting up");
    }

    {
        std::uintptr_t originalFunc = Write5Call(hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_HookLoc.GetUIntPtr(), uintptr_t(hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_Hook));
        hkbRagdollDriver_extractRagdollPoseInternal_hkaPoseMatchingUtility_computeReferenceFrame_Original = (_hkaPoseMatchingUtility_computeReferenceFrame)originalFunc;
        _MESSAGE("hkbRagdollDriver::extractRagdollPoseInternal hkaPoseMatchingUtility::computeReferenceFrame hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_HookLoc.GetUIntPtr(), uintptr_t(hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_Hook));
        hkaRagdollInstance_getApproxWorldFromBoneTransformAt_hkpMotion_approxTransformAt_Original = (_hkpMotion_approxTransformAt)originalFunc;
        _MESSAGE("hkaRagdollInstance::getApproxWorldFromBoneTransformAt hkpMotion::approxTransformAt hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_HookLoc_1.GetUIntPtr(), uintptr_t(MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_Hook_1));
        MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_1_Original = (_hkaSkeletonMapper_mapPose)originalFunc;

        originalFunc = Write5Call(MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_HookLoc_2.GetUIntPtr(), uintptr_t(MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_Hook_2));
        MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_2_Original = (_hkaSkeletonMapper_mapPose)originalFunc;

        _MESSAGE("MapRagdollPoseToAnimPoseModelSpace hkaSkeletonMapper::mapPose hooks complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_HookLoc.GetUIntPtr(), uintptr_t(hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_Hook));
        hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal_hkaSkeletonMapper_mapPose_Original = (_hkaSkeletonMapper_mapPose)originalFunc;
        _MESSAGE("hkbRagdollDriver::mapHighResPoseLocalToLowResPoseLocal hkaSkeletonMapper::mapPose hook complete");
    }

    {
        std::uintptr_t originalFunc = Write5Call(hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_HookLoc.GetUIntPtr(), uintptr_t(hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_Hook));
        hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_Original = (_hkaRagdollRigidBodyController_driveToPose)originalFunc;
        _MESSAGE("hkbRagdollDriver::driveToPose hkaRagdollRigidBodyController::driveToPose hook complete");
    }

    {
        struct Code : Xbyak::CodeGenerator {
            Code(void *buf) : Xbyak::CodeGenerator(256, buf)
            {
                Xbyak::Label jumpBack;

                mov(r8, ptr[rsp + 0x88]); // the HitData is at rsp + 0x88

                // Call our hook
                mov(rax, (uintptr_t)ScriptEventSourceHolder_DispatchHitEventFromHitData_DispatchHitEvent_Hook);
                call(rax);

                // Jump back to whence we came (+ the size of the initial branch instruction)
                jmp(ptr[rip + jumpBack]);

                L(jumpBack);
                dq(ScriptEventSourceHolder_DispatchHitEventFromHitData_DispatchHitEvent_HookLoc.GetUIntPtr() + 5);
            }
        };

        void *codeBuf = g_localTrampoline.StartAlloc();
        Code code(codeBuf);
        g_localTrampoline.EndAlloc(code.getCurr());

        g_branchTrampoline.Write5Branch(ScriptEventSourceHolder_DispatchHitEventFromHitData_DispatchHitEvent_HookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

        _MESSAGE("ScriptEventSourceHolder_DispatchHitEventFromHitData_DispatchHitEvent hook complete");
    }

    { // SKSE hooks the swing handlers and calls the native function directly instead of nicely through vtable chaining, so I need to do these manual hooks.
        struct Code : Xbyak::CodeGenerator {
            Code(void *buf) : Xbyak::CodeGenerator(256, buf)
            {
                Xbyak::Label jumpBack, skip;

                // Save Actor * argument and align the stack
                push(rdx);

                sub(rsp, 0x20); // 0x20 bytes for scratch space

                // Call our hook
                mov(rax, (uintptr_t)WeaponRightSwingHandler_Handle_Hook);
                call(rax);

                add(rsp, 0x20);

                pop(rdx);

                // Return immediately if our hook returns false, otherwise continue the function
                test(al, al);
                jz(skip);

                // Original code
                push(rbx);
                sub(rsp, 0x20);

                // Jump back to whence we came (+ the size of the initial branch instruction)
                jmp(ptr[rip + jumpBack]);

                L(skip);
                ret();

                L(jumpBack);
                dq(WeaponRightSwingHandler_Handle.GetUIntPtr() + 6);
            }
        };

        void *codeBuf = g_localTrampoline.StartAlloc();
        Code code(codeBuf);
        g_localTrampoline.EndAlloc(code.getCurr());

        g_branchTrampoline.Write6Branch(WeaponRightSwingHandler_Handle.GetUIntPtr(), uintptr_t(code.getCode()));

        _MESSAGE("WeaponRightSwingHandler::Handle hook complete");
    }

    {
        struct Code : Xbyak::CodeGenerator {
            Code(void *buf) : Xbyak::CodeGenerator(256, buf)
            {
                Xbyak::Label jumpBack, skip;

                // Save Actor * argument and align the stack
                push(rdx);

                sub(rsp, 0x20); // 0x20 bytes for scratch space

                // Call our hook
                mov(rax, (uintptr_t)WeaponLeftSwingHandler_Handle_Hook);
                call(rax);

                add(rsp, 0x20);

                pop(rdx);

                // Return immediately if our hook returns false, otherwise continue the function
                test(al, al);
                jz(skip);

                // Original code
                push(rbx);
                sub(rsp, 0x20);

                // Jump back to whence we came (+ the size of the initial branch instruction)
                jmp(ptr[rip + jumpBack]);

                L(skip);
                ret();

                L(jumpBack);
                dq(WeaponLeftSwingHandler_Handle.GetUIntPtr() + 6);
            }
        };

        void *codeBuf = g_localTrampoline.StartAlloc();
        Code code(codeBuf);
        g_localTrampoline.EndAlloc(code.getCurr());

        g_branchTrampoline.Write6Branch(WeaponLeftSwingHandler_Handle.GetUIntPtr(), uintptr_t(code.getCode()));

        _MESSAGE("WeaponLeftSwingHandler::Handle hook complete");
    }

    if (Config::options.seamlessFurnitureTransition) {
        {
            g_branchTrampoline.Write5Call(ActorProcess_ExitFurniture_RemoveCollision_HookLoc.GetUIntPtr(), uintptr_t(ActorProcess_ExitFurniture_RemoveCollision_Hook));
            _MESSAGE("ActorProcess TransitionFurnitureState QueueRemoveCollision hook complete");
        }

        {
            struct Code : Xbyak::CodeGenerator {
                Code(void *buf) : Xbyak::CodeGenerator(256, buf)
                {
                    Xbyak::Label jumpBack;

                    mov(r8, rdi); // the actor is in rdi

                    // Call our hook
                    mov(rax, (uintptr_t)ActorProcess_ExitFurniture_ResetRagdoll_Hook);
                    call(rax);

                    // Jump back to whence we came (+ the size of the initial branch instruction)
                    jmp(ptr[rip + jumpBack]);

                    L(jumpBack);
                    dq(ActorProcess_ExitFurniture_ResetRagdoll_HookLoc.GetUIntPtr() + 5);
                }
            };

            void *codeBuf = g_localTrampoline.StartAlloc();
            Code code(codeBuf);
            g_localTrampoline.EndAlloc(code.getCurr());

            g_branchTrampoline.Write5Branch(ActorProcess_ExitFurniture_ResetRagdoll_HookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

            _MESSAGE("ActorProcess ExitFurniture ResetRagdoll hook complete");
        }

        {
            struct Code : Xbyak::CodeGenerator {
                Code(void *buf) : Xbyak::CodeGenerator(256, buf)
                {
                    Xbyak::Label jumpBack;

                    mov(r8, rbp); // the actor is in rdi

                    // Call our hook
                    mov(rax, (uintptr_t)ActorProcess_EnterFurniture_SetWorld_Hook);
                    call(rax);

                    // Jump back to whence we came (+ the size of the initial branch instruction)
                    jmp(ptr[rip + jumpBack]);

                    L(jumpBack);
                    dq(ActorProcess_EnterFurniture_SetWorld_HookLoc.GetUIntPtr() + 5);
                }
            };

            void *codeBuf = g_localTrampoline.StartAlloc();
            Code code(codeBuf);
            g_localTrampoline.EndAlloc(code.getCurr());

            g_branchTrampoline.Write5Branch(ActorProcess_EnterFurniture_SetWorld_HookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

            _MESSAGE("ActorProcess EnterFurniture BSAnimationGraphManager::SetWorld hook complete");
        }
    }

    {
        struct Code : Xbyak::CodeGenerator {
            Code(void *buf) : Xbyak::CodeGenerator(256, buf)
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

        void *codeBuf = g_localTrampoline.StartAlloc();
        Code code(codeBuf);
        g_localTrampoline.EndAlloc(code.getCurr());

        g_branchTrampoline.Write5Branch(Character_MovementControllerUpdate_HookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

        _MESSAGE("MovementControllerNPC::Update hook complete");
    }

    if (Config::options.disableCullingForActiveRagdolls) {
        struct Code : Xbyak::CodeGenerator {
            Code(void *buf) : Xbyak::CodeGenerator(256, buf)
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

        void *codeBuf = g_localTrampoline.StartAlloc();
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
        void *branch = g_trampoline->AllocateFromBranchPool(g_pluginHandle, TRAMPOLINE_SIZE);
        if (!branch) {
            _ERROR("couldn't acquire branch trampoline from SKSE. this is fatal. skipping remainder of init process.");
            return false;
        }

        g_branchTrampoline.SetBase(TRAMPOLINE_SIZE, branch);

        void *local = g_trampoline->AllocateFromLocalPool(g_pluginHandle, TRAMPOLINE_SIZE);
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

bool WaitPosesCB(vr_src::TrackedDevicePose_t *pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t *pGamePoseArray, uint32_t unGamePoseArrayCount)
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

                    NiTransform hmdTransform; HmdMatrixToNiTransform(hmdTransform, hmdMatrix);

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
                                g_controllerData[0].velocities.pop_back();
                                g_controllerData[0].velocities.push_front(velocityWorldspace);
                                g_controllerData[0].ComputeAverageVelocity();
                                g_controllerData[0].ComputeAverageSpeed();
                            }
                        }
                        else if (i == leftIndex && isLeftConnected) {
                            vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
                            if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
                                NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
                                NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };
                                NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;
                                g_controllerData[1].velocities.pop_back();
                                g_controllerData[1].velocities.push_front(velocityWorldspace);
                                g_controllerData[1].ComputeAverageVelocity();
                                g_controllerData[1].ComputeAverageSpeed();
                            }
                        }
                    }
                }
            }
        }
    }

    return true;
}

void ControllerStateCB(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, uint32_t unControllerStateSize, bool &state)
{
    PlayerCharacter *player = *g_thePlayer;
    if (!player || !player->GetNiNode()) return;

    constexpr vr_src::ETrackedControllerRole rightControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand;
    vr_src::TrackedDeviceIndex_t rightController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(rightControllerRole);

    constexpr vr_src::ETrackedControllerRole leftControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand;
    vr_src::TrackedDeviceIndex_t leftController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(leftControllerRole);

    if (g_openVR && *g_openVR) {
        BSOpenVR *openVR = *g_openVR;
        vr_src::IVRSystem *vrSystem = openVR->vrSystem;

        if (unControllerDeviceIndex == rightController) {
            for (int i = 0; i < vr_src::k_unControllerStateAxisCount; i++) {
                vr_src::EVRControllerAxisType axisType = static_cast<vr_src::EVRControllerAxisType>(vrSystem->GetInt32TrackedDeviceProperty(unControllerDeviceIndex, static_cast<vr_src::ETrackedDeviceProperty>(vr_src::ETrackedDeviceProperty::Prop_Axis0Type_Int32 + i)));
                if (axisType == vr_src::EVRControllerAxisType::k_eControllerAxis_Joystick) {
                    g_rightStick = pControllerState->rAxis[i];
                    break;
                }
            }
        }
        else if (unControllerDeviceIndex == leftController) {
            for (int i = 0; i < vr_src::k_unControllerStateAxisCount; i++) {
                vr_src::EVRControllerAxisType axisType = static_cast<vr_src::EVRControllerAxisType>(vrSystem->GetInt32TrackedDeviceProperty(unControllerDeviceIndex, static_cast<vr_src::ETrackedDeviceProperty>(vr_src::ETrackedDeviceProperty::Prop_Axis0Type_Int32 + i)));
                if (axisType == vr_src::EVRControllerAxisType::k_eControllerAxis_Joystick) {
                    g_leftStick = pControllerState->rAxis[i];
                    break;
                }
            }
        }
    }
}

void ShowErrorBox(const char *errorString)
{
    int msgboxID = MessageBox(
        NULL,
        (LPCTSTR)errorString,
        (LPCTSTR)"PLANCK Fatal Error",
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

// Animation graph hook for detecting if certain animations were played
_IAnimationGraphManagerHolder_NotifyAnimationGraph g_originalNotifyAnimationGraph = nullptr;
static RelocPtr<_IAnimationGraphManagerHolder_NotifyAnimationGraph> Character_IAnimationGraphManagerHolder_NotifyAnimationGraph_vtbl(0x16D7780); // Character
bool IAnimationGraphManagerHolder_NotifyAnimationGraph_Hook(IAnimationGraphManagerHolder *_this, const BSFixedString &animationName)
{
    bool accepted = g_originalNotifyAnimationGraph(_this, animationName);
    if (accepted) {
        // Event was accepted by the graph
        std::string_view view(animationName.c_str());
        if (
            view == "NPC_BumpFromFront" ||
            view == "NPC_BumpedFromRight" ||
            view == "NPC_BumpedFromBack" ||
            view == "NPC_BumpedFromLeft"
            ) {
                {
                    std::scoped_lock lock(g_shoveAnimLock);
                    g_shoveAnimTimes[_this] = g_currentFrameTime;
                }
        }
    }
    return accepted;
}

_IAnimationGraphManagerHolder_NotifyAnimationGraph g_originalPlayerCharacterNotifyAnimationGraph = nullptr;
static RelocPtr<_IAnimationGraphManagerHolder_NotifyAnimationGraph> PlayerCharacter_IAnimationGraphManagerHolder_NotifyAnimationGraph_vtbl(0x016E2BF8); // PlayerCharacter
bool PlayerCharacter_NotifyAnimationGraph_Hook(IAnimationGraphManagerHolder *_this, const BSFixedString &animationName)
{
    bool accepted = g_originalPlayerCharacterNotifyAnimationGraph(_this, animationName);
    if (accepted) {
        // Event was accepted by the graph
        _MESSAGE("%p: %s", _this, animationName.c_str());
    }
    return accepted;
}

// PlayerCharacter::UpdateAnimation hook for altering the player's animation rate
_Actor_UpdateAnimation g_originalPCUpdateAnimation = nullptr;
static RelocPtr<_Actor_UpdateAnimation> PlayerCharacter_UpdateAnimation_vtbl(0x16E2618); // 0x16E2230 + 0x7D * 8
void PlayerCharacter_UpdateAnimation_Hook(Actor *_this, float deltaTime)
{
    if (g_numSkipAnimationFrames > 0) {
        --g_numSkipAnimationFrames;
        deltaTime = Config::options.skipAnimationDeltaTime;
    }
    g_originalPCUpdateAnimation(_this, deltaTime);
}


extern "C" {
    void OnDataLoaded()
    {
        g_savedMinSoundVel = *g_fMinSoundVel;

        g_keyword_actorTypeAnimal = papyrusKeyword::GetKeyword(nullptr, BSFixedString("ActorTypeAnimal"));
        g_keyword_actorTypeNPC = papyrusKeyword::GetKeyword(nullptr, BSFixedString("ActorTypeNPC"));
        if (!g_keyword_actorTypeAnimal || !g_keyword_actorTypeNPC) {
            _ERROR("Failed to get keywords");
            return;
        }

        if (MenuManager *menuManager = MenuManager::GetSingleton()) {
            menuManager->MenuOpenCloseEventDispatcher()->AddEventSink(&MenuChecker::menuEvent);
        }

        _MESSAGE("Successfully loaded all forms");
    }

    void OnInputLoaded()
    {

    }

    // Listener for SKSE Messages
    void OnSKSEMessage(SKSEMessagingInterface::Message *msg)
    {
        if (msg) {
            if (msg->type == SKSEMessagingInterface::kMessage_InputLoaded) {
                OnInputLoaded();
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_DataLoaded) {
                OnDataLoaded();
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_PostLoad) {
                // Register our own mod api listener
                g_messaging->RegisterListener(g_pluginHandle, nullptr, PlanckPluginAPI::ModMessageHandler);
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_PostPostLoad) {
                // Get the HIGGS plugin API
                HiggsPluginAPI::GetHiggsInterface001(g_pluginHandle, g_messaging);
                if (g_higgsInterface) {
                    _MESSAGE("Got higgs interface!");

                    unsigned int higgsVersion = g_higgsInterface->GetBuildNumber();
                    if (higgsVersion < 1050100) {
                        ShowErrorBoxAndTerminate("[CRITICAL] HIGGS is present but is a lower version than is required by this mod. Get the latest version of HIGGS and try again.");
                    }

                    g_higgsInterface->AddCollisionFilterComparisonCallback(CollisionFilterComparisonCallback);
                    g_higgsInterface->AddPrePhysicsStepCallback(PrePhysicsStepCallback);
                    g_higgsInterface->AddPreVrikPreHiggsCallback(PreVrikPreHiggsCallback);
                    g_higgsInterface->AddGrabbedCallback(ObjectGrabbedCallback);

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
            else if (msg->type == SKSEMessagingInterface::kMessage_PostLoadGame || msg->type == SKSEMessagingInterface::kMessage_NewGame) {
                bool loadSucceeded = (bool)msg->data;
                if (loadSucceeded) {
                    ResetObjects();
                }
            }
        }
    }

    bool SKSEPlugin_Query(const SKSEInterface *skse, PluginInfo *info)
    {
        gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\activeragdoll.log");
        gLog.SetPrintLevel(IDebugLog::kLevel_Message);
        gLog.SetLogLevel(IDebugLog::kLevel_Message);

        _MESSAGE("PLANCK v%s", ACTIVERAGDOLL_VERSION_VERSTRING);

        info->infoVersion = PluginInfo::kInfoVersion;
        info->name = "PLANCK";
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

    bool SKSEPlugin_Load(const SKSEInterface *skse)
    {	// Called by SKSE to load this plugin
        _MESSAGE("PLANCK loaded");

        if (Config::ReadConfigOptions()) {
            _MESSAGE("Successfully read config parameters");
        }
        else {
            _WARNING("[WARNING] Failed to read some config options");
        }

        gLog.SetPrintLevel((IDebugLog::LogLevel)Config::options.logLevel);
        gLog.SetLogLevel((IDebugLog::LogLevel)Config::options.logLevel);

        _MESSAGE("Registering for SKSE messages");
        g_messaging = (SKSEMessagingInterface *)skse->QueryInterface(kInterface_Messaging);
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

        g_papyrus = (SKSEPapyrusInterface *)skse->QueryInterface(kInterface_Papyrus);
        if (!g_papyrus) {
            ShowErrorBoxAndLog("[CRITICAL] Couldn't get Papyrus interface");
            return false;
        }
        if (g_papyrus->Register(PapyrusAPI::RegisterPapyrusFuncs)) {
            _MESSAGE("Successfully registered papyrus functions");
        }

        g_originalNotifyAnimationGraph = *Character_IAnimationGraphManagerHolder_NotifyAnimationGraph_vtbl;
        SafeWrite64(Character_IAnimationGraphManagerHolder_NotifyAnimationGraph_vtbl.GetUIntPtr(), uintptr_t(IAnimationGraphManagerHolder_NotifyAnimationGraph_Hook));
#ifdef _DEBUG
        g_originalPlayerCharacterNotifyAnimationGraph = *PlayerCharacter_IAnimationGraphManagerHolder_NotifyAnimationGraph_vtbl;
        SafeWrite64(PlayerCharacter_IAnimationGraphManagerHolder_NotifyAnimationGraph_vtbl.GetUIntPtr(), uintptr_t(PlayerCharacter_NotifyAnimationGraph_Hook));
#endif // _DEBUG

        g_originalPCUpdateAnimation = *PlayerCharacter_UpdateAnimation_vtbl;
        SafeWrite64(PlayerCharacter_UpdateAnimation_vtbl.GetUIntPtr(), uintptr_t(PlayerCharacter_UpdateAnimation_Hook));

        g_originalHitFrameHandlerHandle = *HitFrameHandler_Handle_vtbl;
        SafeWrite64(HitFrameHandler_Handle_vtbl.GetUIntPtr(), uintptr_t(HitFrameHandler_Handle_Hook));

        g_originalAttackWinStartHandlerHandle = *AttackWinStartHandler_Handle_vtbl;
        SafeWrite64(AttackWinStartHandler_Handle_vtbl.GetUIntPtr(), uintptr_t(AttackWinStartHandler_Handle_Hook));

        g_originalAttackWinEndHandlerHandle = *AttackWinEndHandler_Handle_vtbl;
        SafeWrite64(AttackWinEndHandler_Handle_vtbl.GetUIntPtr(), uintptr_t(AttackWinEndHandler_Handle_Hook));

        g_originalAttackStopHandlerHandle = *AttackStopHandler_Handle_vtbl;
        SafeWrite64(AttackStopHandler_Handle_vtbl.GetUIntPtr(), uintptr_t(AttackStopHandler_Handle_Hook));

        g_timer.Start();

        return true;
    }
};
