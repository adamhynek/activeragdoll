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
#include <shared_mutex>

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

std::vector<UInt16> g_dialogueSkipConditions = { 0x4D, 0x90, 0x91, 0x120 }; // 4D = GetRandomPercent, 90 = GetTrespassWarningLevel, 91 = IsTrespassing, 120 = GetFriendHit


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
        bool shouldDispatchHitEventBasedOnScript = false;
        CheckHitEventsFunctor::Data checkHitEventsFunctorData{ registry, hitBody };
        CheckHitEventsFunctor checkHitEventsFunctor{ &checkHitEventsFunctorData, &shouldDispatchHitEventBasedOnScript };

        registry->VisitScripts(handle, &checkHitEventsFunctor);

        bool isMoveable = IsMoveableEntity(hitBody);
        bool dispatchHitEvent = shouldDispatchHitEventBasedOnScript || (Config::options.hitAnyMoveableObjects && isMoveable) || (Config::options.hitAnyStaticObjects && !isMoveable);

        if (dispatchHitEvent) {
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

        return dispatchHitEvent;
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
    bhkRigidBody *rigidBody{};
    NiPoint3 point{};
    NiPoint3 impulse{};
    UInt32 refrHandle{};

    PointImpulseJob(bhkRigidBody *rigidBody, const NiPoint3 &point, const NiPoint3 &impulse, UInt32 refrHandle) :
        rigidBody(rigidBody), point(point), impulse(impulse), refrHandle(refrHandle) {}

    virtual void Run() override
    {
        // Need to be safe since the job could run next frame where the rigidbody might not exist anymore
        if (NiPointer<TESObjectREFR> refr; LookupREFRByHandle(refrHandle, refr)) {
            NiPointer<NiNode> root = refr->GetNiNode();
            if (root && FindRigidBody(root, rigidBody)) {
                if (IsMoveableEntity(rigidBody->hkBody)) {
                    hkpEntity_activate(rigidBody->hkBody);
                    rigidBody->hkBody->m_motion.applyPointImpulse(NiPointToHkVector(impulse), NiPointToHkVector(point));
                    //_MESSAGE("Applied point impulse %.2f", VectorLength(impulse));
                }
            }
        }
    }
};

struct LinearImpulseJob : GenericJob
{
    bhkRigidBody *rigidBody{};
    NiPoint3 impulse{};
    UInt32 refrHandle{};

    LinearImpulseJob(bhkRigidBody *rigidBody, const NiPoint3 &impulse, UInt32 refrHandle) :
        rigidBody(rigidBody), impulse(impulse), refrHandle(refrHandle) {}

    virtual void Run() override
    {
        // Need to be safe since the job could run next frame where the rigidbody might not exist anymore
        if (NiPointer<TESObjectREFR> refr; LookupREFRByHandle(refrHandle, refr)) {
            NiPointer<NiNode> root = refr->GetNiNode();
            if (root && FindRigidBody(root, rigidBody)) {
                if (IsMoveableEntity(rigidBody->hkBody)) {
                    hkpEntity_activate(rigidBody->hkBody);
                    rigidBody->hkBody->m_motion.applyLinearImpulse(NiPointToHkVector(impulse));
                    //_MESSAGE("Applied linear impulse %.2f", VectorLength(impulse));
                }
            }
        }
    }
};


std::unordered_map<hkbRagdollDriver *, std::shared_ptr<ActiveRagdoll>> g_activeRagdolls{};

std::shared_ptr<ActiveRagdoll> GetActiveRagdollFromDriver(hkbRagdollDriver *driver)
{
    auto it = g_activeRagdolls.find(driver);
    if (it == g_activeRagdolls.end()) return nullptr;
    return it->second;
}

std::shared_mutex g_activeActorsLock{};
std::unordered_set<Actor *> g_activeActors{};

bool IsActiveActor(TESObjectREFR *refr)
{
    std::shared_lock lock(g_activeActorsLock);
    return g_activeActors.find((Actor *)refr) != g_activeActors.end();
}


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
std::mutex g_delayedJobsLock{};

void QueueDelayedJob(std::function<void(void)> job, double delay)
{
    std::scoped_lock lock(g_delayedJobsLock);
    g_delayedJobs.push_back(std::make_unique<DelayedJob>(job, delay));
}

void RunDelayedJobs(double now)
{
    std::scoped_lock lock(g_delayedJobsLock);

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

    NiPoint3 GetMaxVelocity(const std::deque<NiPoint3> &velocities, int numFrames)
    {
        int size = min(numFrames, velocities.size());

        float largestSpeed = -1;
        int largestIndex = -1;
        for (int i = 0; i < size; i++) {
            const NiPoint3 &velocity = velocities[i];
            float speed = VectorLength(velocity);
            if (speed > largestSpeed) {
                largestSpeed = speed;
                largestIndex = i;
            }
        }

        if (largestIndex == 0) {
            // Max is the first value
            return velocities[0];
        }
        else if (largestIndex == size - 1) {
            // Max is the last value
            return velocities[largestIndex];
        }
        else {
            // Regular case - avg 3 values centered at the peak
            return (velocities[largestIndex - 1] + velocities[largestIndex] + velocities[largestIndex + 1]) / 3;
        }
    }

    NiPoint3 GetMaxRecentVelocity()
    {
        return GetMaxVelocity(velocities, 5);
    }

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
        float smoothingMultiplier = 0.011f / *g_secondsSinceLastFrame_Unmultiplied; // Half the number of frames at 45fps compared to 90fps, etc.
        return round(float(numSmoothingFrames) * smoothingMultiplier);
    }

    int GetNumControllerAngularVelocityTrackingFrames()
    {
        int numFrames = Config::options.numControllerAngularVelocityTrackingFrames;
        float smoothingMultiplier = 0.011f / *g_secondsSinceLastFrame_Unmultiplied; // Half the number of frames at 45fps compared to 90fps, etc.
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

std::unordered_set<UInt16> g_activeBipedGroups{};
std::unordered_set<UInt16> g_noPlayerCharControllerCollideGroups{};
std::unordered_set<UInt16> g_hittableCharControllerGroups{};
std::unordered_set<UInt16> g_selfCollidableBipedGroups{};

std::unordered_map<bhkRigidBody *, double> g_higgsLingeringRigidBodies{};
std::unordered_map<TESObjectREFR *, double> g_higgsLingeringRefrs[2]{}; // [right hand, left hand]
bhkRigidBody *g_rightHand = nullptr;
bhkRigidBody *g_leftHand = nullptr;
bhkRigidBody *g_rightWeapon = nullptr;
bhkRigidBody *g_leftWeapon = nullptr;
bhkRigidBody *g_rightHeldRigidBody = nullptr;
bhkRigidBody *g_leftHeldRigidBody = nullptr;
TESObjectREFR *g_rightHeldRefr = nullptr;
TESObjectREFR *g_leftHeldRefr = nullptr;
UInt16 g_rightHeldActorCollisionGroup = 0;
UInt16 g_leftHeldActorCollisionGroup = 0;
UInt32 g_higgsCollisionLayer = 56;

bhkRigidBody *g_prevRightHeldRigidBody = nullptr;
bhkRigidBody *g_prevLeftHeldRigidBody = nullptr;
TESObjectREFR *g_prevRightHeldRefr = nullptr;
TESObjectREFR *g_prevLeftHeldRefr = nullptr;

NiTransform g_currentGrabTransforms[2]{};

UInt16 g_playerCollisionGroup = 0;

inline bool IsLeftRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return wrapper == g_leftHand || wrapper == g_leftWeapon || wrapper == g_leftHeldRigidBody;
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
    return wrapper == g_leftHeldRigidBody || wrapper == g_rightHeldRigidBody;
}

inline bool IsHiggsRigidBody(hkpRigidBody *rigidBody)
{
    UInt32 collisionLayer = GetCollisionLayer(rigidBody);
    bool isHeld = IsHeldRigidBody(rigidBody);
    if (collisionLayer != g_higgsCollisionLayer && !isHeld) {
        return false;
    }
    return IsPlayerHandRigidBody(rigidBody) || IsPlayerWeaponRigidBody(rigidBody) || isHeld;
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

std::unordered_map<Actor *, double> g_footYankedActors{};

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


std::mutex g_temporaryIgnoredActorsLock;
std::unordered_map<Actor *, double> g_temporaryIgnoredActors;
void AddTemporaryIgnoredActor(Actor *actor, double duration)
{
    std::scoped_lock lock(g_temporaryIgnoredActorsLock);
    g_temporaryIgnoredActors[actor] = g_currentFrameTime + duration;
}

bool IsTemporaryIgnoredActor(Actor *actor)
{
    std::scoped_lock lock(g_temporaryIgnoredActorsLock);
    return g_temporaryIgnoredActors.size() > 0 && g_temporaryIgnoredActors.count(actor);
}

void UpdateTemporaryIgnoredActors()
{
    std::scoped_lock lock(g_temporaryIgnoredActorsLock);
    for (auto it = g_temporaryIgnoredActors.begin(); it != g_temporaryIgnoredActors.end();) {
        if (g_currentFrameTime > it->second) {
            it = g_temporaryIgnoredActors.erase(it);
        }
        else {
            ++it;
        }
    }
}


NiPointer<NiAVObject> GetGrabbedNode(Actor *actor, bool isLeft)
{
    TESObjectREFR *heldRefr = !isLeft ? g_rightHeldRefr : g_leftHeldRefr;
    if (heldRefr == actor) {
        if (bhkRigidBody *heldRigidBody = isLeft ? g_leftHeldRigidBody : g_rightHeldRigidBody) {
            if (NiPointer<NiNode> root = actor->GetNiNode()) {
                if (NiPointer<bhkRigidBody> rigidBody = FindRigidBody(root, heldRigidBody)) { // make sure it's still there to be safe
                    if (NiPointer<NiAVObject> heldNode = GetNodeFromCollidable(rigidBody->hkBody->getCollidable())) {
                        return heldNode;
                    }
                }
            }
        }
    }
    return nullptr;
}

bool ShouldKeepOffset(Actor *actor)
{
    if (!Config::options.doKeepOffset) return false;
    if (Actor_IsGhost(actor)) return false;
    if (IsActorUsingFurniture(actor)) return false;

    if (!actor->race || actor->race->data.unk40 >= 2) return false; // race size is >= large

    if (Config::options.disableKeepOffsetWhenFootHeld) {
        for (int isLeft = 0; isLeft < 2; ++isLeft) {
            if (NiPointer<NiAVObject> heldNode = GetGrabbedNode(actor, isLeft)) {
                if (heldNode->m_name && Config::options.footNodeNames.count(heldNode->m_name)) {
                    // Holding foot -> don't keep offset
                    return false;
                }
            }
        }
    }

    return true;
}

struct KeepOffsetData
{
    double lastAttemptTime = 0.0;
    bool success = false;
};
std::unordered_map<Actor *, KeepOffsetData> g_keepOffsetActors{};

std::optional<float> GetStress(Actor *actor, const hkpRigidBody *desiredBody)
{
    std::optional<float> stress = std::nullopt;

    ForEachRagdollDriver(actor, [desiredBody, &stress](hkbRagdollDriver *driver) -> void {
        if (std::shared_ptr<ActiveRagdoll> activeRagdoll = GetActiveRagdollFromDriver(driver)) {
            if (hkaRagdollInstance *ragdoll = driver->ragdoll) {
                if (ahkpWorld *world = (ahkpWorld *)ragdoll->getWorld()) {
                    for (int i = 0; i < ragdoll->getNumBones(); ++i) {
                        if (ragdoll->m_rigidBodies[i] == desiredBody) {
                            stress = activeRagdoll->stress[i];
                            return;
                        }
                    }
                }
            }
        }
    });

    return stress;
}

std::optional<float> GetBoneDisplacement(Actor *actor, const hkpRigidBody *desiredBody, bool isLeft)
{
    std::optional<float> displacement = std::nullopt;
    ForEachRagdollDriver(actor, [desiredBody, isLeft, &displacement](hkbRagdollDriver *driver) -> void {
        if (std::shared_ptr<ActiveRagdoll> activeRagdoll = GetActiveRagdollFromDriver(driver)) {
            if (hkaRagdollInstance *ragdoll = driver->ragdoll) {
                if (ahkpWorld *world = (ahkpWorld *)ragdoll->getWorld()) {
                    for (int i = 0; i < ragdoll->getNumBones(); ++i) {
                        if (ragdoll->m_rigidBodies[i] == desiredBody) {
                            NiTransform handToNode = g_currentGrabTransforms[isLeft];
                            NiTransform nodeToHand = InverseTransform(handToNode);

                            NiTransform animPose = hkQsTransformToNiTransform(activeRagdoll->lowResAnimPoseWS[i]);
                            NiTransform rbPose = hkTransformToNiTransform(desiredBody->getTransform(), animPose.scale);

                            NiTransform handOnAnimPose = animPose * nodeToHand;
                            NiTransform handOnRbPose = rbPose * nodeToHand;

                            NiPoint3 delta = handOnAnimPose.pos - handOnRbPose.pos;
                            displacement = VectorLength(delta);
                            return;
                        }
                    }
                }
            }
        }
    });

    return displacement;
}

bool CanRagdoll(Actor *actor)
{
    if (!Config::options.ragdollOnGrab) return false;

    TESRace *race = actor->race;
    if (!race) return false;

    if (race->data.unk40 >= 2) return false; // race size is >= large

    if (!Actor_CanBeKnockedDown(actor)) return false; // checks stuff like kNoKnockdowns flag

    if (Actor_IsGhost(actor)) return false;

    if (IsTemporaryIgnoredActor(actor)) return false;

    return true;
}

bool IsAnimal(TESRace *race)
{
    return race->keyword.HasKeyword(g_keyword_actorTypeAnimal);
}

enum class ForceRagdollType
{
    None,
    RagdollOnGrab,
    FootYank,
    RightHandedYank,
    LeftHandedYank,
    TwoHandedYank,
    KeepOffset,
};
ForceRagdollType ShouldRagdollOnGrabOrFootYank(Actor *actor)
{
    if (!CanRagdoll(actor)) return ForceRagdollType::None;

    TESRace *race = actor->race;
    if (Config::options.ragdollSmallRacesOnGrab && race->data.unk40 == 0) return ForceRagdollType::RagdollOnGrab; // small race
    
    float health = actor->actorValueOwner.GetCurrent(24);
    if (health < Config::options.smallRaceHealthThreshold) return ForceRagdollType::RagdollOnGrab;

    if (Config::options.ragdollOnFootYank && !IsAnimal(race)) {
        for (int isLeft = 0; isLeft < 2; ++isLeft) {
            if (NiPointer<NiAVObject> heldNode = GetGrabbedNode(actor, isLeft)) {
                if (heldNode->m_name && Config::options.footNodeNames.count(heldNode->m_name)) {
                    if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(heldNode)) {
                        if (std::optional<float> stress = GetBoneDisplacement(actor, rigidBody->hkBody, isLeft)) {
                            if (*stress > Config::options.footYankRequiredStressAmount) {
                                return ForceRagdollType::FootYank;
                            }
                        }
                    }
                }
            }
        }
    }

    return ForceRagdollType::None;
}


struct LetGoHandData
{
    NiPoint3 velocity;
    TESObjectREFR * refr;
    bhkRigidBody * rigidBody;
    double time;
};
LetGoHandData g_letGoHandData[2]{};

ForceRagdollType GetDesiredForceRagdollType(Actor *actor, bool isHeld)
{
    bool wasHeldRight = actor == g_prevRightHeldRefr;
    bool wasHeldLeft = actor == g_prevLeftHeldRefr;
    bool wasHeld = wasHeldRight || wasHeldLeft;

    if (isHeld) {
        if (!Actor_IsInRagdollState(actor)) {
            ForceRagdollType ragdollType = ShouldRagdollOnGrabOrFootYank(actor);
            if (ragdollType != ForceRagdollType::None) {
                return ragdollType;
            }
            else if (ShouldKeepOffset(actor)) {
                return ForceRagdollType::KeepOffset;
            }
        }
    }
    else if (wasHeld) {
        if (!Actor_IsInRagdollState(actor) && CanRagdoll(actor)) {
            if (Config::options.ragdollOnYank && !IsAnimal(actor->race)) {
                auto [rightVelocity, rightDroppedRefr, rightDroppedRigidBody, rightLetGoTime] = g_letGoHandData[0];
                auto [leftVelocity, leftDroppedRefr, leftDroppedRigidBody, leftLetGoTime] = g_letGoHandData[1];

                if (rightDroppedRefr == actor && leftDroppedRefr == actor && g_currentFrameTime - rightLetGoTime < Config::options.yankTwoHandedTimeWindow && g_currentFrameTime - leftLetGoTime < Config::options.yankTwoHandedTimeWindow) {
                    // ADD UP hand velocity for the impulse
                    // Take MAX velocity to check the threshold (so you still need at least one hand above the threshold, but for impulse you can get up to 2x the normal impulse)
                    NiPoint3 yankVelocity = VectorLength(rightVelocity) >= VectorLength(leftVelocity) ? rightVelocity : leftVelocity;
                    if (VectorLength(yankVelocity) > Config::options.yankRequiredHandSpeedRoomspace) {
                        return ForceRagdollType::TwoHandedYank;
                    }
                }
                else if (!Config::options.requireTwoHandsToYank) {
                    if (wasHeldRight && VectorLength(rightVelocity) > Config::options.yankRequiredHandSpeedRoomspace) {
                        return ForceRagdollType::RightHandedYank;
                    }
                    else if (wasHeldLeft && VectorLength(leftVelocity) > Config::options.yankRequiredHandSpeedRoomspace) {
                        return ForceRagdollType::LeftHandedYank;
                    }
                }
            }
        }
    }

    return ForceRagdollType::None;
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
            Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
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
        float requiredStamina = Config::options.requireFullPowerAttackStamina ? staminaCost : 0.f;

        if (staminaCost > 0.f && currentStamina <= requiredStamina) {
            if (!isBash) {
                // No stamina to power attack, so re-set the attackdata but this time explicitly set powerattack to false
                PlayerCharacter_UpdateAndGetAttackData(player, isLeft, isOffhand, false, &attackData);
                // Now do a regular attack (overwrites the attack data too)
                PlayerControls_SendAction(PlayerControls::GetSingleton(), GetAttackActionId(isOffhand), 2);
            }
            Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
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
        float requiredStamina = Config::options.requireFullBashStamina ? staminaCost : 0.f;

        if (staminaCost > 0.f && currentStamina <= requiredStamina) {
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
bool g_isCurrentHitFatal = false;

bool HitActor(Character *source, Character *target, NiAVObject *hitNode, const NiPoint3 &hitPosition, const NiPoint3 &hitVelocity, bool isLeft, bool isOffhand, bool isPowerAttack, bool isBash)
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

    g_isCurrentHitFatal = false;

    g_isInPlanckHit = true;
    Character_HitTarget(source, target, nullptr, isOffhand); // Populates HitData and so relies on attackState/attackData
    g_isInPlanckHit = false;

    bool isFatal = g_isCurrentHitFatal;

    Actor_RemoveMagicEffectsDueToAction(source, -1); // removes invis/ethereal due to attacking

    SwingHandler &swingHandler = isLeft ? g_leftSwingHandler : g_rightSwingHandler;
    if (!swingHandler.IsSwingActive()) {
        SetAttackState(source, 0);
    }

    return isFatal;
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

    BSTaskPool_QueueDamageObjectTask(BSTaskPool::GetSingleton(), target, damage, false, source);
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

void ApplyHitImpulse(bhkWorld *world, Actor *actor, bhkRigidBody *rigidBody, const NiPoint3 &hitVelocity, const NiPoint3 &position, float impulseMult)
{
    UInt32 targetHandle = GetOrCreateRefrHandle(actor);

    {
        BSReadLocker lock(&world->worldLock);

        // Need to be safe and make sure the rigidBody is actually here
        NiPointer<NiNode> root = actor->GetNiNode();
        if (root && !FindRigidBody(root, rigidBody)) {
            return;
        }

        // Apply a point impulse at the hit location to the body we actually hit
        QueuePrePhysicsJob<PointImpulseJob>(rigidBody, position, CalculateHitImpulse(rigidBody->hkBody, hitVelocity, impulseMult), targetHandle);

        // Apply linear impulse at the center of mass to all bodies within 2 ragdoll constraints
        ForEachAdjacentBody(root, rigidBody,
            [hitVelocity, impulseMult, targetHandle](bhkRigidBody *adjacentBody, int wave) {
                float impulseDecayMult;
                if (wave == 0) {
                    return; // Skip the first wave, as we already applied a POINT impulse to the hit body
                }
                else if (wave == 1) {
                    impulseDecayMult = Config::options.hitImpulseDecayMult1;
                }
                else if (wave == 2) {
                    impulseDecayMult = Config::options.hitImpulseDecayMult2;
                }
                else {
                    impulseDecayMult = Config::options.hitImpulseDecayMult3;
                }
                QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, CalculateHitImpulse(adjacentBody->hkBody, hitVelocity, impulseMult) * impulseDecayMult, targetHandle);
            },
            3
        );
    }
}

NiPoint3 CalculateYankImpulse(hkpRigidBody *rigidBody, const NiPoint3 &velocity)
{
    float massInv = rigidBody->getMassInv();
    float mass = massInv <= 0.001f ? 99999.f : 1.f / massInv;
    NiPoint3 impulse = velocity * mass; // This impulse will give the object the exact velocity it is hit with
    impulse.x *= Config::options.yankImpulseHorizontalMultiplier;
    impulse.y *= Config::options.yankImpulseHorizontalMultiplier;
    impulse.z *= (impulse.z >= 0.f) ? Config::options.yankImpulseUpwardsMultiplier : Config::options.yankImpulseDownwardsMultiplier;
    return impulse;
}

void ApplyYankImpulse(bhkWorld *world, Actor *actor, bhkRigidBody *rigidBody, const NiPoint3 &velocity)
{
    UInt32 targetHandle = GetOrCreateRefrHandle(actor);

    {
        BSReadLocker lock(&world->worldLock);

        // Need to be safe and make sure the rigidBody is actually here
        NiPointer<NiNode> root = actor->GetNiNode();
        if (root && !FindRigidBody(root, rigidBody)) {
            return;
        }

        ForEachAdjacentBody(root, rigidBody,
            [velocity, targetHandle](bhkRigidBody *adjacentBody, int wave) {
                NiPoint3 impulse = CalculateYankImpulse(adjacentBody->hkBody, velocity);
                impulse *= powf(Config::options.yankImpulseDecayExponent, wave);
                QueuePrePhysicsJob<LinearImpulseJob>(adjacentBody, impulse, targetHandle);
            },
            Config::options.yankImpulseBoneRadius
        );
    }
}

BGSMaterialType *GetImpactMaterial(hkpRigidBody *hitRigidBody, const hkpContactPointEvent &evnt, const NiPoint3 &hitPosition)
{
    UInt32 materialId = 0;
    UInt32 hitLayer = GetCollisionLayer(hitRigidBody);
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

        bool isFatal = HitActor(player, hitChar, hitNode, hitPosition, hitVelocity, isLeft, isOffhand, isPowerAttack, isBash);

        PlayMeleeImpactRumble(isTwoHanding ? 2 : isLeft);

        if (Config::options.applyImpulseOnHit) {
            if (isPowerAttack) {
                impulseMult *= Config::options.powerAttackImpulseMultiplier;
            }

            if (NiPointer<NiAVObject> root = hitChar->GetNiNode()) {
                if (NiPointer<bhkWorld> world = GetHavokWorldFromCell(player->parentCell)) {
                    if (DoesRefrHaveNode(hitChar, hitNode)) {
                        if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(hitNode)) {
                            if (isFatal) {
                                // If the hit was fatal, apply a smaller impulse so they don't go flying
                                impulseMult *= Config::options.fatalHitImpulseMultiplier;
                            }
                            else if (Actor_IsInRagdollState(hitChar)) {
                                impulseMult *= Config::options.ragdolledHitImpulseMultiplier;
                            }
                            ApplyHitImpulse(world, hitChar, rigidBody, hitVelocity, hitPosition * *g_havokWorldScale, impulseMult);
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
#ifdef _DEBUG
        _MESSAGE("%d: Refr hit", *g_currentFrameCounter);
#endif // _DEBUG

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
        if (!root || !FindRigidBody(root, rigidBody)) return;

        bool isRagdollRigidBody = false;
        // This is a task because we acquire the animation graph manager lock in here, which can cause a deadlock in some cases
        ForEachRagdollDriver(actor, [this, &isRagdollRigidBody](hkbRagdollDriver *driver) {
            if (hkaRagdollInstance *ragdoll = driver->ragdoll) {
                if (ragdoll->m_rigidBodies.indexOf(rigidBody->hkBody) >= 0) {
                    isRagdollRigidBody = true;
                }
            }
        });
        if (isRagdollRigidBody) return;

        rigidBody->hkBody->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo &= ~(0x7f); // zero out layer
        rigidBody->hkBody->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo |= (BGSCollisionLayer::kCollisionLayer_DeadBip & 0x7f); // set layer to the same as a dead ragdoll
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

UInt64 g_originalBipedLayerBitfield = 0;
UInt64 g_originalBipedNoCCLayerBitfield = 0;

bool ShouldIgnoreNonActorBipedCollision(UInt32 layerA, UInt32 layerB)
{
    if (layerA == BGSCollisionLayer::kCollisionLayer_Biped) {
        if (!(g_originalBipedLayerBitfield & ((UInt64)1 << layerB))) {
            return true;
        }
    }
    else if (layerA == BGSCollisionLayer::kCollisionLayer_BipedNoCC) {
        if (!(g_originalBipedNoCCLayerBitfield & ((UInt64)1 << layerB))) {
            return true;
        }
    }
    else if (layerB == BGSCollisionLayer::kCollisionLayer_Biped) {
        if (!(g_originalBipedLayerBitfield & ((UInt64)1 << layerA))) {
            return true;
        }
    }
    else if (layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC) {
        if (!(g_originalBipedNoCCLayerBitfield & ((UInt64)1 << layerA))) {
            return true;
        }
    }

    return false;
}

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

    UInt32 lastPhysicsDamageFormId = 0;

    void IgnoreCollisionForSeconds(bool isLeft, Actor *actor, double duration)
    {
        collisionCooldownTargets[isLeft][actor] = g_currentFrameTime + duration;
    }

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
                if (NiPointer<TESObjectREFR> hittingRefr = GetRefFromCollidable(collidingBody->hkBody->getCollidable())) {
                    if (TESForm *baseForm = hittingRefr->baseForm) {
                        // set an unused flag to signify that this is physics damage
                        hitData.flags |= (1 << 29);
                        lastPhysicsDamageFormId = baseForm->formID;
                    }
                }

                Actor_GetHit(target, hitData);
                if (Config::options.physicsHitRecoveryTime > 0) {
                    physicsHitCooldownTargets[{ target, collidingBody->hkBody }] = g_currentFrameTime;
                }

                if (Config::options.damageHittingObjectOnPhysicalHit) {
                    if (NiPointer<TESObjectREFR> hittingRefr = GetRefFromCollidable(collidingBody->hkBody->getCollidable())) {
                        BSTaskPool_QueueDamageObjectTask(BSTaskPool::GetSingleton(), hittingRefr, Config::options.hittingObjectSelfDamage, false, source);
                    }
                }
            }

            HitData_dtor(&hitData);
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

        UInt32 layerA = GetCollisionLayer(rigidBodyA);
        UInt32 layerB = GetCollisionLayer(rigidBodyB);

        /*if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped && layerB != g_higgsCollisionLayer) ||
            (layerB == BGSCollisionLayer::kCollisionLayer_Biped && layerA != g_higgsCollisionLayer))
        {
            if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped && IsRagdollHandRigidBody(rigidBodyA)) || (layerB == BGSCollisionLayer::kCollisionLayer_Biped && IsRagdollHandRigidBody(rigidBodyB))) {
                // One of the collidees is a weapon rigidbody on the biped layer, and it didn't collide with a higgs rigidbody
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }
        }*/

        if ((layerA == BGSCollisionLayer::kCollisionLayer_CharController && IsMoveableEntity(rigidBodyB)) ||
            (layerB == BGSCollisionLayer::kCollisionLayer_CharController && IsMoveableEntity(rigidBodyA)))
        {
            // Note: Traps like falling boulders DO still apply themselves even if we ignore the contact point here.
            if (Config::options.disableClutterVsCharacterControllerCollisionForActiveActors) {
                const hkpCollidable *charControllerCollidable = layerA == BGSCollisionLayer::kCollisionLayer_CharController ? rigidBodyA->getCollidable() : rigidBodyB->getCollidable();
                if (NiPointer<TESObjectREFR> refr = GetRefFromCollidable(charControllerCollidable)) {
                    if (refr->formType == kFormType_Character) {
                        if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
                            if (IsActiveActor(actor)) {
                                // We'll let the clutter object collide with the biped instead
                                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                                return;
                            }
                        }
                    }
                }
            }
        }

        bool isABiped = layerA == BGSCollisionLayer::kCollisionLayer_Biped || layerA == BGSCollisionLayer::kCollisionLayer_BipedNoCC;
        bool isBBiped = layerB == BGSCollisionLayer::kCollisionLayer_Biped || layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC;

        // Note: We COULD disable all collisions with non-moveable objects on any layer, but I'd like to keep the door open to have collision with statics. But not static clutter objects.
        if ((isABiped && (layerB == BGSCollisionLayer::kCollisionLayer_Clutter || layerB == BGSCollisionLayer::kCollisionLayer_Weapon || layerB == BGSCollisionLayer::kCollisionLayer_Trap)) ||
            (isBBiped && (layerA == BGSCollisionLayer::kCollisionLayer_Clutter || layerA == BGSCollisionLayer::kCollisionLayer_Weapon || layerA == BGSCollisionLayer::kCollisionLayer_Trap)))
        {
            hkpRigidBody *bipedBody = isABiped ? rigidBodyA : rigidBodyB;

            if (NiPointer<TESObjectREFR> bipedRefr = GetRefFromCollidable(bipedBody->getCollidable())) {
                if (bipedRefr->formType == kFormType_Character) {
                    Actor *actor = DYNAMIC_CAST(bipedRefr, TESObjectREFR, Actor);
                    hkpRigidBody *hittingBody = isABiped ? rigidBodyB : rigidBodyA;

                    if (Config::options.disableBipedClutterCollisionWithNonMoveableObjects && !IsMoveableEntity(hittingBody) && !IsHiggsRigidBody(hittingBody) && IsActiveActor(actor)) {
                        evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                        return;
                    }
                }
            }
        }

        if ((isABiped && IsMoveableEntity(rigidBodyB)) || (isBBiped && IsMoveableEntity(rigidBodyA))) {
            UInt16 groupA = GetCollisionGroup(rigidBodyA);
            UInt16 groupB = GetCollisionGroup(rigidBodyB);

            if (groupA != groupB) { // Make sure this is not the biped colliding with itself, since biped objects are moveable
                if (NiPointer<TESObjectREFR> refrA = GetRefFromCollidable(rigidBodyA->getCollidable())) {
                    if (NiPointer<TESObjectREFR> refrB = GetRefFromCollidable(rigidBodyB->getCollidable())) {
                        bool isAActor = refrA->formType == kFormType_Character;
                        bool isBActor = refrB->formType == kFormType_Character;

                        if (!isAActor && !isBActor) {
                            // Neither object is an actor - potentially disable collision with biped objects that are not actors
                            if (ShouldIgnoreNonActorBipedCollision(layerA, layerB)) {
                                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                                return;
                            }
                        }
                        else if (!(isAActor && isBActor)) {
                            // Exactly one of the objects is an actor

                            Actor *actor = DYNAMIC_CAST(isAActor ? refrA : refrB, TESObjectREFR, Actor);
                            hkpRigidBody *hittingBody = isAActor ? rigidBodyB : rigidBodyA;

                            if (Config::options.doClutterVsBipedCollisionDamage && !IsHiggsRigidBody(hittingBody) && !physicsHitCooldownTargets.count({ actor, hittingBody })) {
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
        }

        if ((isABiped || layerA == BGSCollisionLayer::kCollisionLayer_DeadBip) && (isBBiped || layerB == BGSCollisionLayer::kCollisionLayer_DeadBip)) {
            if (Config::options.overrideSoundVelForRagdollCollisions) {
                // Disable collision sounds for this frame
                *g_fMinSoundVel = Config::options.ragdollSoundVel;
            }

            UInt16 groupA = GetCollisionGroup(rigidBodyA);
            UInt16 groupB = GetCollisionGroup(rigidBodyB);

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

        bool isAhiggs = IsHiggsRigidBody(rigidBodyA);
        bool isBhiggs = IsHiggsRigidBody(rigidBodyB);
        if (!isAhiggs && !isBhiggs) return; // Every collision we care about involves a rigidbody involved with higgs (hand, weapon, held object)

        bool isAheld = IsHeldRigidBody(rigidBodyA);
        bool isBheld = IsHeldRigidBody(rigidBodyB);

        if (isAhiggs && isBhiggs) {
            // Both objects are higgs rigidbodies - don't let them hit each other
            if (!IsMoveableEntity(rigidBodyA) && !IsMoveableEntity(rigidBodyB)) {
                // Both nonmoveable likely implies hand vs hand, or weapon vs hand, or weapon vs weapon
                // We set those to keyframed_reporting and if we don't disable contact, it'll trigger haptics and such
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }
            else if (isAheld && isBheld) {
                // Both are held objects - don't let them hit each other
                // TODO: We could try to handle this properly
                return;
            }
            else if (!isAheld && !isBheld) {
                // Only allow hits vs held objects. Still let the collision happen though.
                return;
            }
        }

        // Since neither object is held, this means at least one of them is a weapon or hand
        bool isAhandOrWeapon = isAhiggs && !isAheld;
        hkpRigidBody *hitRigidBody = isAhandOrWeapon ? rigidBodyB : rigidBodyA;
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

        bool isHittingObjectHeld = IsHeldRigidBody(hittingRigidBody);

        if (g_higgsLingeringRefrs[isLeft].count(hitRefr)) {
            // This refr is currently held / was recently dropped in this hand. Don't eat the collision but don't allow the hit.
            if (!isHittingObjectHeld && collisionCooldownTargets[isLeft].count(hitRefr)) {
                // refr shouldn't be hit OR collided with
                evnt.m_contactPointProperties->m_flags |= hkpContactPointProperties::CONTACT_IS_DISABLED;
                return;
            }
            return;
        }

        if (Config::options.preventActorSelfHits) {
            NiPointer<TESObjectREFR> hittingRefr = GetRefFromCollidable(&hittingRigidBody->m_collidable);
            if (hittingRefr && hittingRefr == hitRefr && hitRefr->formType == kFormType_Character) {
                // Thing that hit is from the same refr as the thing getting hit.
                // We don't want this, to prevent stuff like if you grab someone's arm and "hit" them with it.
                // Don't disable contact in this case, just don't proceed with the hit.
                return;
            }
        }

        // A contact point of any sort confirms a collision, regardless of any collision added or removed events

        hkVector4 hkHitPos = evnt.m_contactPoint->getPosition();
        NiPoint3 hittingObjPointVelocity = hkpRigidBody_getPointVelocity(hittingRigidBody, hkHitPos);
        NiPoint3 hitObjPointVelocity = hkpRigidBody_getPointVelocity(hitRigidBody, hkHitPos);

        NiPoint3 hkHitVelocity = hittingObjPointVelocity - hitObjPointVelocity;

        float hitSpeed = VectorLength(hkHitVelocity);

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

            if (!isHittingObjectHeld && collisionCooldownTargets[isLeft].count(hitRefr)) {
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

        bool isAhiggs = IsHiggsRigidBody(rigidBodyA);
        bool isBhiggs = IsHiggsRigidBody(rigidBodyB);
        if (!isAhiggs && !isBhiggs) return; // Every collision we care about involves a rigidbody involved with higgs (hand, weapon, held object)

        if (isAhiggs && isBhiggs) return; // Both objects are on the higgs layer

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

            // There is a case where we touch someone with a held object and then let go.
            // The active collision will remain even though we are no longer holding the object.
            // This can happen even if the held "object" is part of the character's ragdoll.
            // So keep the collision tracked (in case we re-grab the object) but don't truly count it. It will be removed either way when the collision is removed, held or not.

            bool isAhiggs = IsHiggsRigidBody(bodyA);
            bool isBhiggs = IsHiggsRigidBody(bodyB);
            if (!isAhiggs && !isBhiggs) continue;

            hkpRigidBody *collidedBody = isAhiggs ? bodyB : bodyA;

            // Don't count collisions with the character controller, only the ragdoll
            if (GetCollisionLayer(collidedBody) == BGSCollisionLayer::kCollisionLayer_CharController) continue;

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
                auto [target, cooldownUntil] = *it;
                if (g_currentFrameTime > cooldownUntil)
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

        UInt32 layer = GetCollisionLayer(rigidBody);
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

    bhkWorld *world = nullptr;
};
PhysicsListener g_physicsListener{};

struct PlayerCharacterProxyListener : hkpCharacterProxyListener
{
    // Called when the character interacts with another (non fixed or keyframed) rigid body.
    virtual void objectInteractionCallback(hkpCharacterProxy *proxy, const hkpCharacterObjectInteractionEvent &input, hkpCharacterObjectInteractionResult &output)
    {
        hkpRigidBody *hitBody = input.m_body;
        if (!hitBody) return;

        UInt32 layer = GetCollisionLayer(hitBody);
        if (layer != BGSCollisionLayer::kCollisionLayer_Biped && layer != BGSCollisionLayer::kCollisionLayer_BipedNoCC) return;

        NiPointer<TESObjectREFR> refr = GetRefFromCollidable(hitBody->getCollidable());
        if (!refr) return;

        if (refr->formType != kFormType_Character) return;

        Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
        if (!actor) return;

        output.m_objectImpulse = NiPointToHkVector(HkVectorToNiPoint(output.m_objectImpulse) * Config::options.playerVsBipedInteractionImpulseMultiplier);
        if (layer != BGSCollisionLayer::kCollisionLayer_Biped) return;

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
    UInt32 layerA = GetCollisionLayer(filterInfoA);
    UInt32 layerB = GetCollisionLayer(filterInfoB);

    if ((layerA == BGSCollisionLayer::kCollisionLayer_Biped || layerA == BGSCollisionLayer::kCollisionLayer_BipedNoCC) && (layerB == BGSCollisionLayer::kCollisionLayer_Biped || layerB == BGSCollisionLayer::kCollisionLayer_BipedNoCC)) {
        // Biped vs. biped
        UInt16 groupA = GetCollisionGroup(filterInfoA);
        UInt16 groupB = GetCollisionGroup(filterInfoB);
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
        UInt16 otherGroup = GetCollisionGroup(otherFilter);
        if (otherGroup == g_playerCollisionGroup) {
            // Whatever collided with the charcontroller belongs to the player
            UInt32 otherLayer = GetCollisionLayer(otherFilter);
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
    UInt32 otherLayer = GetCollisionLayer(otherFilter);
    UInt16 otherGroup = GetCollisionGroup(otherFilter);

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

    {
        // Set lingering rigidbodies while we hold them
        if (g_rightHeldRigidBody) {
            g_higgsLingeringRigidBodies[g_rightHeldRigidBody] = g_currentFrameTime;
        }
        if (g_leftHeldRigidBody) {
            g_higgsLingeringRigidBodies[g_leftHeldRigidBody] = g_currentFrameTime;
        }

        // Clear out old dropped / thrown rigidbodies
        for (auto it = g_higgsLingeringRigidBodies.begin(); it != g_higgsLingeringRigidBodies.end();) {
            auto [target, lastHeldTime] = *it;
            if ((g_currentFrameTime - lastHeldTime) * *g_globalTimeMultiplier >= Config::options.thrownObjectLingerTime)
                it = g_higgsLingeringRigidBodies.erase(it);
            else
                ++it;
        }


        // Lingering refrs are those that were just dropped
        if (g_rightHeldRefr) {
            g_higgsLingeringRefrs[0][g_rightHeldRefr] = g_currentFrameTime;
        }
        if (g_leftHeldRefr) {
            g_higgsLingeringRefrs[1][g_leftHeldRefr] = g_currentFrameTime;
        }

        for (int isLeft = 0; isLeft < 2; isLeft++) {
            auto &lingeringRefrs = g_higgsLingeringRefrs[isLeft];
            for (auto it = lingeringRefrs.begin(); it != lingeringRefrs.end();) {
                auto [target, lastHeldTime] = *it;
                if ((g_currentFrameTime - lastHeldTime) * *g_globalTimeMultiplier >= Config::options.thrownObjectIgnoreHitTime) {
                    it = lingeringRefrs.erase(it);
                }
                else
                    ++it;
            }
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
    TESTopicInfo *lastSaidTopicInfo = nullptr;
    TESTopicInfo *secondLastSaidTopicInfo = nullptr;
    float lastSaidDialogueDuration = -1.f;
    float lastVoiceTimer = -1.f;
    bool isSpeaking = false;

    void TriggerDialogue(Character *character, TESTopic *topic, TESTopicInfo *topicInfo)
    {
        Actor_SayToEx(character, *g_thePlayer, topic, topicInfo);

        if (topicInfo) {
            secondLastSaidTopicInfo = lastSaidTopicInfo;
            lastSaidTopicInfo = topicInfo;
        }

        lastSaidTopic = topic;
    }

    void TryTriggerDialogue(Character *character, bool high, bool isShoved)
    {
        if (Actor_IsInRagdollState(character) || IsSleeping(character)) return;

        if (isShoved) {
            std::vector<TESTopicInfo *> topicInfos = EvaluateTopicInfoConditions(Config::options.shoveTopicInfos, character, *g_thePlayer, g_dialogueSkipConditions);
            if (TESTopicInfo *topicInfo = GetRandomTopicInfo(topicInfos, lastSaidTopicInfo, secondLastSaidTopicInfo)) {
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
        std::vector<TESTopicInfo *> topicInfos = EvaluateTopicInfoConditions(topicInfoIDs, character, *g_thePlayer, g_dialogueSkipConditions);
        if (TESTopicInfo *topicInfo = GetRandomTopicInfo(topicInfos, lastSaidTopicInfo, secondLastSaidTopicInfo)) {
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

    void StateUpdate(Character *character, bool isShoved, bool wasJustRagdolled)
    {
        if (character->IsDead(1)) return;
        if (Config::options.summonsSkipAggression && GetCommandingActor(character) == *g_playerHandle) return;

        {
            std::scoped_lock lock(g_interface001.aggressionIgnoredActorsLock);
            if (g_interface001.aggressionIgnoredActors.count(character)) return;
        }

        if ((Config::options.followersSkipAggression && IsTeammate(character)) ||
            (RelationshipRanks::GetRelationshipRank(character->baseForm, (*g_thePlayer)->baseForm) > Config::options.aggressionMaxRelationshipRank))
        {
            // Still do some dialogue when shoved even if it won't grow aggression for them
            if (isShoved && !Actor_IsInRagdollState(character) && !IsSleeping(character)) {
                std::vector<TESTopicInfo *> topicInfos = EvaluateTopicInfoConditions(Config::options.shoveTopicInfos, character, *g_thePlayer, g_dialogueSkipConditions);
                if (TESTopicInfo *topicInfo = GetRandomTopicInfo(topicInfos, lastSaidTopicInfo, secondLastSaidTopicInfo)) {
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

        PlayerCharacter *player = *g_thePlayer;

        // These two are to not do aggression if they are in... certain scenes...
        bool sharesPlayerPosition = Config::options.stopAggressionForCloseActors && VectorLength(character->pos - player->pos) < Config::options.closeActorMinDistance;
        bool isInVehicle = Config::options.stopAggressionForActorsWithVehicle && GetVehicleHandle(character) != *g_invalidRefHandle;
        bool isSpecial = sharesPlayerPosition || isInVehicle;

        bool canPlayerAggress = !Actor_IsInRagdollState(player) && !IsSwimming(player) && !IsStaggered(player) && (!Config::options.dontDoAggressionWhileMenusAreOpen || !g_isMenuOpen);
        bool isCalmed = Config::options.calmedActorsDontAccumulateAggression && IsCalmed(character);

        bool isGrabbed = g_leftHeldRefr == character || g_rightHeldRefr == character;
        bool isTouchedRight = g_physicsListener.collidedRefs[0].count(character) && g_isRightHandAggressivelyPositioned;
        bool isTouchedLeft = g_physicsListener.collidedRefs[1].count(character) && g_isLeftHandAggressivelyPositioned;
        bool isInteractedWith = isGrabbed || isTouchedLeft || isTouchedRight;

        bool isAggressivelyInteractedWith = (isInteractedWith && canPlayerAggress) || isShoved || wasJustRagdolled;
        bool accumulateAggression = isAggressivelyInteractedWith && !isSpecial && !isCalmed;

        if (accumulateAggression) {
            if (wasJustRagdolled) {
                accumulatedGrabbedTime += Config::options.ragdollAggressionImpact;
            }
            else if (isShoved) {
                accumulatedGrabbedTime += Config::options.shoveAggressionImpact;
            }
            else {
                accumulatedGrabbedTime += deltaTime;
            }
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
            else if (accumulateAggression) {
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
            else if (accumulateAggression) {
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

void TryUpdateNPCState(Actor *actor, bool isShoved, bool wasJustRagdolled)
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
        data.StateUpdate(character, isShoved, wasJustRagdolled);
    }
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

bool IsAddableToWorld(Actor *actor)
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

    if (IsTemporaryIgnoredActor(actor)) return false;

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

                    if (Config::options.activateActorsOnAdd) {
                        bhkRigidBody_setActivated(wrapper, true);
                    }
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
        /*if (mapping.m_boneA == 0) {
            if (VectorLength(HkVectorToNiPoint(mapping.m_aFromBTransform.m_translation)) > 0.1f) {
                _WARNING("ragdolltoanim: root bone translation is not zero");
            }
        }*/
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

    /*if (Actor *actor = GetActorFromCharacter(character)) {
        if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
            _MESSAGE(name->name.data);
        }
    }*/

    int i = 0;
    for (auto it = mappedBones.begin(); it != mappedBones.end(); it++) {
        std::pair<hkInt16, hkInt16> pair = *it;

        {
            hkaSkeletonMapperData::SimpleMapping &mapping = animToRagdollMapper->m_mapping.m_simpleMappings[i];
            mapping.m_boneA = pair.first;
            mapping.m_boneB = pair.second;
            mapping.m_aFromBTransform = mappedBonesAnimToRagdoll[pair];
            //mapping.m_aFromBTransform.m_translation = NiPointToHkVector(NiPoint3());
            //mapping.m_aFromBTransform.m_rotation = NiQuatToHkQuat(QuaternionIdentity());
            //mapping.m_aFromBTransform.m_scale = NiPointToHkVector(NiPoint3(1.0f, 1.0f, 1.0f));

            /*NiPoint3 translation = HkVectorToNiPoint(mapping.m_aFromBTransform.m_translation);
            NiPoint3 euler = MatrixToEuler(QuaternionToMatrix(HkQuatToNiQuat(mapping.m_aFromBTransform.m_rotation)));
            _MESSAGE("%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\t%s", translation.x, translation.y, translation.z, euler.x, euler.y, euler.z, setup->m_animationSkeleton->m_bones[mapping.m_boneA].m_name.cString());*/
        }

        {
            hkaSkeletonMapperData::SimpleMapping &mapping = ragdollToAnimMapper->m_mapping.m_simpleMappings[i];
            mapping.m_boneA = pair.second;
            mapping.m_boneB = pair.first;
            mapping.m_aFromBTransform = mappedBonesRagdollToAnim[pair];
            //mapping.m_aFromBTransform.m_translation = NiPointToHkVector(NiPoint3());
            //mapping.m_aFromBTransform.m_rotation = NiQuatToHkQuat(QuaternionIdentity());
            //mapping.m_aFromBTransform.m_scale = NiPointToHkVector(NiPoint3(1.0f, 1.0f, 1.0f));
        }

        i++;
    }

    // The new size will be <= the old size, so we know the capacity is sufficient
    animToRagdollMapper->m_mapping.m_simpleMappings.m_size = mappedBones.size();
    ragdollToAnimMapper->m_mapping.m_simpleMappings.m_size = mappedBones.size();
}


int g_lastActorAddedToWorldFrame = 0;

bool AddRagdollToWorld(Actor *actor)
{
    if (!Config::options.processRagdolledActors && Actor_IsInRagdollState(actor)) return false;

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

    g_lastActorAddedToWorldFrame = *g_currentFrameCounter;

    return true;
}


void CleanupActiveRagdollTracking(Actor *actor)
{
    BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
    if (GetAnimationGraphManager(actor, animGraphManager)) {
        BSAnimationGraphManager *manager = animGraphManager.ptr;
        {
            SimpleLocker lock(&manager->updateLock);
            for (int i = 0; i < manager->graphs.size; i++) {
                BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
                if (hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver) {
                    if (std::shared_ptr<ActiveRagdoll> ragdoll = GetActiveRagdollFromDriver(driver)) {
                        if (ragdoll && ragdoll->shouldNullOutWorldWhenRemovingFromWorld) {
                            graph.ptr->world = nullptr;
                        }
                    }
                    g_activeRagdolls.erase(driver);

                    {
                        std::unique_lock lock(g_activeActorsLock);
                        g_activeActors.erase(actor);
                    }
                }
            }
        }
    }
}

void CleanupActiveGroupTracking(UInt32 collisionGroup)
{
    g_activeBipedGroups.erase(collisionGroup);
    g_noPlayerCharControllerCollideGroups.erase(collisionGroup);
}

bool RemoveRagdollFromWorld(Actor *actor)
{
    bool isInRagdollState = Actor_IsInRagdollState(actor);
    if (!Config::options.processRagdolledActors && isInRagdollState) return false;

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

        if (!isInRagdollState) {
            bool x = false;
            BSAnimationGraphManager_RemoveRagdollFromWorld(animGraphManager.ptr, &x);
        }

        CleanupActiveRagdollTracking(actor);
    }

    return true;
}

void DisableOrEnableSyncOnUpdate(Actor *actor, bool disableElseEnable)
{
    if (Actor_IsInRagdollState(actor)) return;

    bool hasRagdollInterface = false;
    BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 }; // need to init this to 0 or we crash
    if (GetAnimationGraphManager(actor, animGraphManager)) {
        BSAnimationGraphManager_HasRagdoll(animGraphManager.ptr, &hasRagdollInterface);
    }

    if (hasRagdollInterface) {
        bool x[2] = { false, disableElseEnable };
        BSAnimationGraphManager_DisableOrEnableSyncOnUpdate(animGraphManager.ptr, x);
    }
}

void RemoveActorFromWorldIfActive(Actor *actor)
{
    bool isActiveActor = IsActiveActor(actor);
    bool isAddedToWorld = IsAddedToWorld(actor);

    UInt32 filterInfo; Actor_GetCollisionFilterInfo(actor, filterInfo);
    UInt16 collisionGroup = filterInfo >> 16;

    bool isHittableCharController = g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(collisionGroup);

    if (isAddedToWorld && isActiveActor) {

#ifdef _DEBUG
        TESFullName *fullName = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);
        _MESSAGE("%d %s detached", *g_currentFrameCounter, fullName->name.data);
#endif // _DEBUG

        RemoveRagdollFromWorld(actor);
        CleanupActiveGroupTracking(collisionGroup);
    }
    else if (isHittableCharController) {
        g_hittableCharControllerGroups.erase(collisionGroup);
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
        bool isReallyDead = actor->IsDead(1) && !IsReanimating(actor);
        bool isParalyzed = actor->flags1 & (1 << 31);
        bool isUnconscious = (actor->actorState.flags04 & 0x1E00000) == 0x600000;
        if (isReallyDead || isParalyzed || isUnconscious) return 0.f;
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

void UpdateSpeedReductionAndStaminaDrain()
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

    PlayerCharacter *player = *g_thePlayer;
    bool isPlayerGodMode = get_vfunc<_MagicTarget_IsInvulnerable>(&player->magicTarget, 4)(&player->magicTarget);
    if (!isPlayerGodMode && staminaCost > 0.f) {
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
                Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
                if (BGSSoundDescriptorForm *shoutFailSound = GetDefaultObject<BGSSoundDescriptorForm>(129)) {
                    PlaySoundAtNode(shoutFailSound, player->GetNiNode(), {});
                }
            }
        }
    }
}

bool TryStaminaAction(float staminaCost)
{
    PlayerCharacter *player = *g_thePlayer;
    bool isPlayerGodMode = get_vfunc<_MagicTarget_IsInvulnerable>(&player->magicTarget, 4)(&player->magicTarget);

    float staminaBeforeHit = player->actorValueOwner.GetCurrent(26);
    if (isPlayerGodMode || staminaBeforeHit > 0.f || staminaCost <= 0.f) {
        // Costs no stamina, or we have enough stamina

        if (!isPlayerGodMode) {
            DamageAV(player, 26, -staminaCost);

            if (player->actorValueOwner.GetCurrent(26) <= 0.f) {
                // Out of stamina after the action
                if (ActorProcessManager *process = player->processManager) {
                    float regenRate = Actor_GetActorValueRegenRate(player, 26);
                    ActorProcess_UpdateRegenDelay(process, 26, (staminaCost - staminaBeforeHit) / regenRate);
                }
                Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
                FlashHudMenuMeter(26);
            }
        }

        return true;
    }
    else {
        // Not enough stamina
        if (BGSSoundDescriptorForm *magicFailSound = GetDefaultObject<BGSSoundDescriptorForm>(128)) {
            PlaySoundAtNode(magicFailSound, player->GetNiNode(), {});
        }
        Actor_TriggerMiscDialogue(player, 100, false); // kOutofBreath
        FlashHudMenuMeter(26);

        return false;
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
                    if (TryStaminaAction(Config::options.shoveStaminaCost)) {
                        NiPoint3 shoveDirection = VectorNormalized(controllerData.avgVelocity);
                        QueueBumpActor(actor, shoveDirection, true, false, false, false);

                        if (NiPointer<bhkRigidBody> rigidBody = GetFirstRigidBody(GetTorsoNode(actor))) {
                            if (NiPointer<NiAVObject> handNode = GetFirstPersonHandNode(isLeft)) {
                                PlayPhysicsSound(rigidBody->hkBody->getCollidableRw(), handNode->m_worldTransform.pos, true);
                            }
                        }
                    }

                    PlayRumble(!isLeft, Config::options.shoveRumbleIntensity, Config::options.shoveRumbleDuration);
                    // Ignore future contact points for a bit to make things less janky
                    g_physicsListener.IgnoreCollisionForSeconds(isLeft, actor, Config::options.shoveCollisionCooldownTime);

                    if (g_controllerData[!isLeft].avgSpeed > Config::options.shoveSpeedThreshold) {
                        PlayRumble(isLeft, Config::options.shoveRumbleIntensity, Config::options.shoveRumbleDuration);
                        g_physicsListener.IgnoreCollisionForSeconds(!isLeft, actor, Config::options.shoveCollisionCooldownTime);
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
#ifdef _DEBUG
    _MESSAGE("%d Reset objects", *g_currentFrameCounter);
#endif // _DEBUG

    if (Config::options.removeActiveActorsOnLoad) {
        if (AIProcessManager *processManager = *g_aiProcessManager) {
            for (UInt32 i = 0; i < processManager->actorsHigh.count; i++) {
                UInt32 actorHandle = processManager->actorsHigh[i];
                NiPointer<TESObjectREFR> refr;
                if (LookupREFRByHandle(actorHandle, refr) && refr != *g_thePlayer) {
                    Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor);
                    if (!actor || !actor->GetNiNode()) {
                        continue;
                    }

                    RemoveActorFromWorldIfActive(actor);
                }
            }
        }
    }

    {
        std::unique_lock lock(g_activeActorsLock);
        g_activeActors.clear();
    }

    g_npcs.clear();
    g_activeRagdolls.clear();
    g_activeBipedGroups.clear();
    g_noPlayerCharControllerCollideGroups.clear();
    g_hittableCharControllerGroups.clear();
    g_selfCollidableBipedGroups.clear();
    g_higgsLingeringRigidBodies.clear();
    g_higgsLingeringRefrs[0].clear();
    g_higgsLingeringRefrs[1].clear();
    g_keepOffsetActors.clear();
    g_bumpActors.clear();
    g_shoveData.clear();
    g_shoveAnimTimes.clear();
    g_shovedActors.clear();
    g_footYankedActors.clear();
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

void OnHiggsGrab(bool isLeft, TESObjectREFR *grabbedRefr)
{
    g_initialGrabTransforms[isLeft] = g_higgsInterface->GetGrabTransform(isLeft);
}


void PlayRagdollSound(Actor *actor)
{
    // Why do some complicated stuff here like evaluate a subset of conditions on a set of topic infos, instead of just playing a dialogue category like kHit?
    // Because kHit for example, has voice lines like "That's all you've got?!" which is not appropriate here.
    // The reason we DO want to evaluate conditions at all is that certain lines cannot be said by children, etc. and we need to check that.
    // We use the conditions as a filter, after which we choose a random one to play, as opposed to playing an entire topic.
    // Playing a topic could refuse to play a topicinfo if the random percent condition on it failed, even if it's the only otherwise-passing topicinfo in the entire topic.
    std::vector<TESTopicInfo *> topicInfos = EvaluateTopicInfoConditions(Config::options.ragdollTopicInfos, actor, *g_thePlayer, g_dialogueSkipConditions);
    if (topicInfos.empty()) {
        PlayDialogueWithoutActorChecks(Config::options.ragdollSoundFallbackDialogueSubtype, actor, *g_thePlayer);
    }
    else {
        if (TESTopicInfo *topicInfo = GetRandomTopicInfo(topicInfos)) {
            PlayTopicInfoWithoutActorChecks(topicInfo, actor, *g_thePlayer);
        }
    }
}

void RagdollActor(Actor *actor)
{
    if (ActorProcessManager *process = actor->processManager) {
        if (Config::options.playRagdollSound) {
            PlayRagdollSound(actor);
        }
        ActorProcess_PushActorAway(process, actor, (*g_thePlayer)->pos, 0.f);
    }
}

void UpdateKeepOffset(Actor *actor, bool keepOffset)
{
    if (keepOffset) {
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
                    ForEachRagdollDriver(actor, [actor](hkbRagdollDriver *driver) {
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
    else {
        if (g_keepOffsetActors.size() > 0 && g_keepOffsetActors.count(actor)) {
            g_taskInterface->AddTask(ClearKeepOffsetTask::Create(GetOrCreateRefrHandle(actor)));
            g_keepOffsetActors.erase(actor);
        }
    }
}

void UpdateHiggsInfo(bhkWorld *world)
{
    NiPointer<bhkRigidBody> rightHand = (bhkRigidBody *)g_higgsInterface->GetHandRigidBody(false);
    NiPointer<bhkRigidBody> leftHand = (bhkRigidBody *)g_higgsInterface->GetHandRigidBody(true);
    g_rightHand = rightHand;
    g_leftHand = leftHand;

    NiPointer<bhkRigidBody> rightWeapon = (bhkRigidBody *)g_higgsInterface->GetWeaponRigidBody(false);
    NiPointer<bhkRigidBody> leftWeapon = (bhkRigidBody *)g_higgsInterface->GetWeaponRigidBody(true);
    g_rightWeapon = rightWeapon;
    g_leftWeapon = leftWeapon;

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
        g_higgsCollisionLayer = GetCollisionLayer(rightHand->hkBody);
    }

    g_prevRightHeldRigidBody = g_rightHeldRigidBody;
    g_prevLeftHeldRigidBody = g_leftHeldRigidBody;

    g_rightHeldRigidBody = (bhkRigidBody *)g_higgsInterface->GetGrabbedRigidBody(false);
    g_leftHeldRigidBody = (bhkRigidBody *)g_higgsInterface->GetGrabbedRigidBody(true);

    g_prevRightHeldRefr = g_rightHeldRefr;
    g_prevLeftHeldRefr = g_leftHeldRefr;

    g_rightHeldRefr = g_higgsInterface->GetGrabbedObject(false);
    g_leftHeldRefr = g_higgsInterface->GetGrabbedObject(true);

    g_currentGrabTransforms[0] = g_higgsInterface->GetGrabTransform(false);
    g_currentGrabTransforms[1] = g_higgsInterface->GetGrabTransform(true);

    // Check if we let go of something
    if (!g_rightHeldRigidBody && g_prevRightHeldRigidBody) {
        g_letGoHandData[0] = { g_controllerData[0].GetMaxRecentVelocity(), g_prevRightHeldRefr, g_prevRightHeldRigidBody, g_currentFrameTime };
    }
    if (!g_leftHeldRigidBody && g_prevLeftHeldRigidBody) {
        g_letGoHandData[1] = { g_controllerData[1].GetMaxRecentVelocity(), g_prevLeftHeldRefr, g_prevLeftHeldRigidBody, g_currentFrameTime };
    }
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

    UpdateTemporaryIgnoredActors();

    if (world != g_physicsListener.world) {
        if (g_physicsListener.world) {
            ResetObjects();
        }

        _MESSAGE("%d: Havok world changed", *g_currentFrameCounter);
        {
            BSWriteLocker lock(&world->worldLock);

            if (!hkpWorld_findWorldExtension(world->world, hkpKnownWorldExtensionIds::HK_WORLD_EXTENSION_COLLISION_CALLBACK)) {
                hkpCollisionCallbackUtil_requireCollisionCallbackUtil(world->world);
            }

            if (!hkpWorld_hasContactListener(world->world, &g_physicsListener)) {
                hkpWorld_addContactListener(world->world, &g_physicsListener);
            }
            if (!hkpWorld_hasWorldPostSimulationListener(world->world, &g_physicsListener)) {
                hkpWorld_addWorldPostSimulationListener(world->world, &g_physicsListener);
            }
            if (!hkpWorld_hasEntityListener(world->world, &g_physicsListener)) {
                hkpWorld_addEntityListener(world->world, &g_physicsListener);
            }

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
            if (Config::options.enableBipedPropsCollision) {
                filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Props);
            }
            if (Config::options.enableBipedTrapCollision) {
                filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Trap);
            }
            if (Config::options.enableBipedSpellCollision) {
                filter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped] |= ((UInt64)1 << BGSCollisionLayer::kCollisionLayer_Spell);
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

    UpdateHiggsInfo(world);
    UpdateHiggsDrop();

    // Do this after we've updated higgs things
    UpdateSpeedReductionAndStaminaDrain();

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

    { // Clear out old foot yank actors
        for (auto it = g_footYankedActors.begin(); it != g_footYankedActors.end();) {
            auto [actor, shovedTime] = *it;
            if ((g_currentFrameTime - shovedTime) > Config::options.footYankCooldown)
                it = g_footYankedActors.erase(it);
            else
                ++it;
        }
    }

    if (g_currentFrameTime - g_worldChangedTime < Config::options.worldChangedWaitTime) {
        return;
    }

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
            if (!actor || !actor->GetNiNode()) {
                continue;
            }

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
            UInt16 collisionGroup = GetCollisionGroup(filterInfo);

            bool isHeldLeft = actor == g_leftHeldRefr;
            bool isHeldRight = actor == g_rightHeldRefr;
            bool isHeld = isHeldLeft || isHeldRight;

            ForceRagdollType desiredForceRagdollType = GetDesiredForceRagdollType(actor, isHeld);

            // When an npc is grabbed, disable collision with them
            if (isHeldRight) {
                rightHeldCollisionGroup = collisionGroup; // ignore collision with player capsule
                g_physicsListener.IgnoreCollisionForSeconds(false, actor, Config::options.droppedActorIgnoreCollisionTime); // ignore collision with hands/weapons
            }
            if (isHeldLeft) {
                leftHeldCollisionGroup = collisionGroup;
                g_physicsListener.IgnoreCollisionForSeconds(true, actor, Config::options.droppedActorIgnoreCollisionTime);
            }

            bool didRagdoll = false;
            if (desiredForceRagdollType == ForceRagdollType::RagdollOnGrab) {
                RagdollActor(actor);
                didRagdoll = true;
            }
            else if (desiredForceRagdollType == ForceRagdollType::FootYank) {
                if (!g_footYankedActors.count(actor)) {
                    g_footYankedActors[actor] = g_currentFrameTime;

                    if (TryStaminaAction(Config::options.footYankStaminaCost)) {
                        RagdollActor(actor);
                        didRagdoll = true;
                    }
                }
            }
            else if (desiredForceRagdollType == ForceRagdollType::RightHandedYank || desiredForceRagdollType == ForceRagdollType::LeftHandedYank || desiredForceRagdollType == ForceRagdollType::TwoHandedYank) {
                float staminaCost = Actor_IsHostileToActor(actor, player) ? Config::options.yankHostileStaminaCost : Config::options.yankStaminaCost;
                if (TryStaminaAction(staminaCost)) {
                    if (desiredForceRagdollType == ForceRagdollType::RightHandedYank || desiredForceRagdollType == ForceRagdollType::TwoHandedYank) {
                        ApplyYankImpulse(world, actor, g_letGoHandData[0].rigidBody, g_letGoHandData[0].velocity);
                    }
                    if (desiredForceRagdollType == ForceRagdollType::LeftHandedYank || desiredForceRagdollType == ForceRagdollType::TwoHandedYank) {
                        ApplyYankImpulse(world, actor, g_letGoHandData[1].rigidBody, g_letGoHandData[1].velocity);
                    }

                    g_shovedActors[actor] = g_currentFrameTime; // Prevent shoving right when letting go. This needs to be before UpdateActorShove.

                    RagdollActor(actor);
                    didRagdoll = true;
                }
            }

            UpdateKeepOffset(actor, desiredForceRagdollType == ForceRagdollType::KeepOffset);


            bool didShove = UpdateActorShove(actor);

            TryUpdateNPCState(actor, didShove, didRagdoll);


            bool isHittableCharController = g_hittableCharControllerGroups.size() > 0 && g_hittableCharControllerGroups.count(collisionGroup);

            bool shouldBeActive = VectorLength(actor->pos - player->pos) * *g_havokWorldScale < Config::options.activeRagdollStartDistance;
            bool shouldBeInactive = VectorLength(actor->pos - player->pos) * *g_havokWorldScale > Config::options.activeRagdollEndDistance;

            bool isAddedToWorld = IsAddedToWorld(actor);
            bool isActiveActor = IsActiveActor(actor);
            bool isProcessedActor = isActiveActor || isHittableCharController;
            bool isAddableToWorld = IsAddableToWorld(actor);

            //isAddableToWorld = isAddableToWorld && (*g_currentFrameCounter % 1000) > 500;

            bool isAllowedToAddToWorld = *g_currentFrameCounter - g_lastActorAddedToWorldFrame > Config::options.minFramesBetweenActorAdds;

            if (shouldBeActive) {
                if ((!isAddedToWorld || !isProcessedActor) && isAddableToWorld && isAllowedToAddToWorld) {
                    AddRagdollToWorld(actor);
                    if (collisionGroup != 0) {
                        g_activeBipedGroups.insert(collisionGroup);
                        if (isHittableCharController) {
                            g_hittableCharControllerGroups.erase(collisionGroup);
                        }
                    }
                }

                if (!isAddableToWorld) {
                    if (isActiveActor) {
                        // Someone in range went from having a ragdoll or not being excluded, to not having a ragdoll or being excluded
                        RemoveRagdollFromWorld(actor);
                        CleanupActiveGroupTracking(collisionGroup);
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
                    DisableOrEnableSyncOnUpdate(actor, true);

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
                }
            }
            else if (shouldBeInactive) {
                if (isAddedToWorld && isAddableToWorld && isActiveActor) {
                    RemoveRagdollFromWorld(actor);
                    CleanupActiveGroupTracking(collisionGroup);
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


struct SavedConstraintData
{
    NiPoint3 pivotA;
    NiPoint3 pivotB;
    float coneMaxAngle;
    float planesMinAngle;
    float planesMaxAngle;
    float twistMinAngle;
    float twistMaxAngle;
};

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

    /*
    NiPoint3 rootBoneMappingTranslation = { 0.f, 0.f, 0.f };
    if (hkbCharacterSetup *setup = driver->character->setup) {
        if (hkaSkeletonMapper *mapper = setup->m_animationToRagdollSkeletonMapper) {
            for (int i = 0; i < mapper->m_mapping.m_simpleMappings.m_size; i++) {
                hkaSkeletonMapperData::SimpleMapping &mapping = mapper->m_mapping.m_simpleMappings[i];
                if (mapping.m_boneB == 0) {
                    rootBoneMappingTranslation = HkVectorToNiPoint(mapping.m_aFromBTransform.m_translation);
                    if (worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
                        if (poseHeader && poseHeader->m_onFraction > 0.f && worldFromModelHeader && worldFromModelHeader->m_onFraction > 0.f) {
                            hkQsTransform &worldFromModel = *(hkQsTransform *)Track_getData(generatorOutput, *worldFromModelHeader);
                            hkQsTransform *highResPoseLocal = (hkQsTransform *)Track_getData(generatorOutput, *poseHeader);

                            NiTransform rootBoneTransform = hkQsTransformToNiTransform(highResPoseLocal[mapping.m_boneA], false);
                            rootBoneTransform = hkQsTransformToNiTransform(worldFromModel, false) * rootBoneTransform;

                            NiPoint3 correction = rootBoneTransform.rot * rootBoneMappingTranslation;

                            worldFromModel.m_translation = NiPointToHkVector(HkVectorToNiPoint(worldFromModel.m_translation) + correction);
                        }
                    }
                }
            }
        }
    }
    */

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

    bool wasInRagdollState = ragdoll->isInRagdollState;
    ragdoll->isInRagdollState = Actor_IsInRagdollState(actor);

    if ((!wasInRagdollState && ragdoll->isInRagdollState) ||
        (prevKnockState != KnockState::GetUp && ragdoll->knockState == KnockState::GetUp))
    {
        ragdoll->loosenConstraintsStartTime = g_currentFrameTime;
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
                rb->m_motion.getMotionState()->m_transform = hkQsTransformTohkTransform(transform);
            }

            // Save the constraint limits in case we need them in a bit
            static std::unordered_map<hkpConstraintInstance *, SavedConstraintData> previousConstraintData{};
            previousConstraintData.clear();

            for (hkpConstraintInstance *constraint : driver->ragdoll->getConstraintArray()) {
                if (constraint->getData()->getType() == hkpConstraintData::CONSTRAINT_TYPE_RAGDOLL) {
                    hkpRagdollConstraintData *data = (hkpRagdollConstraintData *)constraint->getData();

                    SavedConstraintData savedData;
                    savedData.pivotA = HkVectorToNiPoint(data->m_atoms.m_transforms.m_transformA.m_translation);
                    savedData.pivotB = HkVectorToNiPoint(data->m_atoms.m_transforms.m_transformB.m_translation);
                    savedData.coneMaxAngle = data->m_atoms.m_coneLimit.m_maxAngle;
                    savedData.planesMinAngle = data->m_atoms.m_planesLimit.m_minAngle;
                    savedData.planesMaxAngle = data->m_atoms.m_planesLimit.m_maxAngle;
                    savedData.twistMinAngle = data->m_atoms.m_twistLimit.m_minAngle;
                    savedData.twistMaxAngle = data->m_atoms.m_twistLimit.m_maxAngle;
                    previousConstraintData[constraint] = savedData;
                }
            }

            if (ragdoll->easeConstraintsAction) {
                // Restore constraint limits from before we loosened them last time

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

                                hkVector4 masterPivot = bodyA == constraint->getSlaveEntity() ? pivotB : pivotA;

                                hkpRagdollConstraintData_setPivotInWorldSpace(data, bodyA->getTransform(), bodyB->getTransform(), masterPivot);
                            }
                        }
                    }
                }

                // Loosen angular constraints second
                if (!ragdoll->isInRagdollState) {
                    hkpEaseConstraintsAction_loosenConstraints(ragdoll->easeConstraintsAction);
                }
            }

            // Restore rigidbody transforms
            for (int i = 0; i < driver->ragdoll->m_rigidBodies.getSize(); i++) {
                hkpRigidBody *rb = driver->ragdoll->m_rigidBodies[i];
                rb->m_motion.getMotionState()->m_transform = savedTransforms[i];
            }

            if (Config::options.graduallyLoosenConstraintsWhileRagdolled && (ragdoll->isInRagdollState || ragdoll->knockState != KnockState::Normal)) {
                float maxDelta = 0.f;
                float maxDeltaPivot = 0.f;
                bool shouldActivate = false;
                // Step the constraint limits by a fixed amount from the previous value towards the value they were just set to
                for (hkpConstraintInstance *constraint : driver->ragdoll->getConstraintArray()) {
                    if (constraint->getData()->getType() == hkpConstraintData::CONSTRAINT_TYPE_RAGDOLL) {
                        hkpRagdollConstraintData *data = (hkpRagdollConstraintData *)constraint->getData();

                        if (auto it = previousConstraintData.find(constraint); it != previousConstraintData.end()) {
                            SavedConstraintData &savedData = it->second;

                            // Note: Don't clamp the elapsed time fraction, so that the speed can go unbounded
                            float elapsedTimeFraction = (g_currentFrameTime - ragdoll->loosenConstraintsStartTime) / Config::options.loosenConstraintsRampUpTime;
                            float angularStep = lerp(Config::options.loosenConstraintsSpeedAngularStart, Config::options.loosenConstraintsSpeedAngularEnd, elapsedTimeFraction) * deltaTime;
                            float linearStep = lerp(Config::options.loosenConstraintsSpeedLinearStart, Config::options.loosenConstraintsSpeedLinearEnd, elapsedTimeFraction) * deltaTime;

                            // Angular limits
                            float delta;
                            data->m_atoms.m_coneLimit.m_maxAngle = AdvanceFloat(savedData.coneMaxAngle, data->m_atoms.m_coneLimit.m_maxAngle, angularStep, &delta);
                            if (delta > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDelta = max(maxDelta, delta);
                            data->m_atoms.m_planesLimit.m_minAngle = AdvanceFloat(savedData.planesMinAngle, data->m_atoms.m_planesLimit.m_minAngle, angularStep, &delta);
                            if (delta > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDelta = max(maxDelta, delta);
                            data->m_atoms.m_planesLimit.m_maxAngle = AdvanceFloat(savedData.planesMaxAngle, data->m_atoms.m_planesLimit.m_maxAngle, angularStep, &delta);
                            if (delta > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDelta = max(maxDelta, delta);
                            data->m_atoms.m_twistLimit.m_minAngle = AdvanceFloat(savedData.twistMinAngle, data->m_atoms.m_twistLimit.m_minAngle, angularStep, &delta);
                            if (delta > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDelta = max(maxDelta, delta);
                            data->m_atoms.m_twistLimit.m_maxAngle = AdvanceFloat(savedData.twistMaxAngle, data->m_atoms.m_twistLimit.m_maxAngle, angularStep, &delta);
                            if (delta > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDelta = max(maxDelta, delta);

                            // Pivots
                            NiPoint3 deltaPivot;
                            data->m_atoms.m_transforms.m_transformA.m_translation = NiPointToHkVector(AdvanceVector(savedData.pivotA, HkVectorToNiPoint(data->m_atoms.m_transforms.m_transformA.m_translation), linearStep, &deltaPivot));
                            if (VectorLength(deltaPivot) > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDeltaPivot = max(maxDeltaPivot, VectorLength(deltaPivot));
                            data->m_atoms.m_transforms.m_transformB.m_translation = NiPointToHkVector(AdvanceVector(savedData.pivotB, HkVectorToNiPoint(data->m_atoms.m_transforms.m_transformB.m_translation), linearStep, &deltaPivot));
                            if (VectorLength(deltaPivot) > Config::options.loosenConstraintsActivateDeltaThreshold) shouldActivate = true;
                            maxDeltaPivot = max(maxDeltaPivot, VectorLength(deltaPivot));
                        }
                    }
                }

                if (shouldActivate) {
                    for (hkpRigidBody *rigidBody : driver->ragdoll->m_rigidBodies) {
                        if (bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData) {
                            bhkRigidBody_setActivated(wrapper, true);
                        }
                    }
                }
            }
        }
    }

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
            if (allNoForce && isUsingRootBoneAsWorldFromModel && !ragdoll->isInRagdollState) {
                if (TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName)) {
                    _MESSAGE("%s bugged out while getting up. Knocking them down again.", name->name);
                }
                if (ActorProcessManager *process = actor->processManager) {
                    ActorProcess_PushActorAway(process, actor, (*g_thePlayer)->pos, 0.f);
                }
            }
        }

        if (allNoForce) {
            // Only powered constraints are active and they are effectively disabled. This is the typical case for dead/ragdolled actors.
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
        if (isComputingWorldFromModel || ragdoll->isInRagdollState) {
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
        // Don't want to do foot ik / disabling gravity when powered only
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

                            ragdoll->lowResAnimPoseWS.assign(lowResPoseWorld.m_data, lowResPoseWorld.m_data + lowResPoseWorld.m_size);

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
                                    {
                                        TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);
                                        _MESSAGE("%d %s: Warp %.2f m", *g_currentFrameCounter, name->name, VectorLength(ragdoll->rootOffset));
                                    }

                                    AddTemporaryIgnoredActor(actor, Config::options.warpDisableActorTime);
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

    if (Config::options.disableConstraints || (actor->race && Config::options.disableConstraintsRaces.count(actor->race->editorId.c_str()))) {
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
    if (!IsActiveActor(actor)) return; // let the game decide

    UInt32 cullState = 7; // do not cull this actor

    actor->unk274 &= 0xFFFFFFF0;
    actor->unk274 |= cullState & 0xF;
}

_BShkbAnimationGraph_UpdateAnimation BShkbAnimationGraph_UpdateAnimation_Original = 0;
void BShkbAnimationGraph_UpdateAnimation_Hook(BShkbAnimationGraph *_this, BShkbAnimationGraph::UpdateData *updateData, void *a3)
{
    Actor *actor = _this->holder;
    if (a3 && actor && IsActiveActor(actor)) { // a3 is null if the graph is not active
        updateData->unk2A = true; // forces animation update (hkbGenerator::generate()) without skipping frames
    }

    BShkbAnimationGraph_UpdateAnimation_Original(_this, updateData, a3);
}

_Actor_GetHit Actor_TakePhysicsDamage_Original = 0;
void Actor_TakePhysicsDamage_Hook(Actor *_this, HitData &hitData)
{
    // We do our own physics damage for active actors, so don't have the game do its as well
    if (IsActiveActor(_this)) return;
    Actor_TakePhysicsDamage_Original(_this, hitData);
}

_Actor_EndHavokHit Actor_KillEndHavokHit_Original = 0;
void Actor_KillEndHavokHit_Hook(HavokHitJobs *havokHitJobs, Actor *_this)
{
    // The actor is dying so undo anything we've done to it
    if (IsActiveActor(_this)) {
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


struct RemoveNonRagdollRigidBodiesFromWorldTask : TaskDelegate
{
    static RemoveNonRagdollRigidBodiesFromWorldTask *Create(UInt32 refHandle)
    {
        RemoveNonRagdollRigidBodiesFromWorldTask *cmd = new RemoveNonRagdollRigidBodiesFromWorldTask;
        if (cmd) {
            cmd->handle = refHandle;
        }
        return cmd;
    }

    virtual void Run() {
        NiPointer<TESObjectREFR> refr;
        if (LookupREFRByHandle(handle, refr)) {
            if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
                BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
                if (GetAnimationGraphManager(actor, animGraphManager)) {
                    BSAnimationGraphManager *manager = animGraphManager.ptr;

                    bool x = false;
                    BSAnimationGraphManager_RemoveNonRagdollRigidBodiesFromWorld(manager, &x);
                }
            }
        }
    }

    virtual void Dispose() {
        delete this;
    }

    UInt32 handle;
};


void ActorProcess_ExitFurniture_RemoveCollision_Hook(BSTaskPool *taskPool, NiAVObject *root)
{
    TESObjectREFR *refr = NiAVObject_GetOwner(root);
    if (IsActiveActor(refr)) {
        UInt32 handle = GetOrCreateRefrHandle(refr);
        g_taskInterface->AddTask(RemoveNonRagdollRigidBodiesFromWorldTask::Create(handle));
        return;
    }

    BSTaskPool_QueueRemoveCollisionFromWorld(taskPool, root);
}

void ActorProcess_ExitFurniture_ResetRagdoll_Hook(BSAnimationGraphManager *manager, bool *a2, Actor *actor)
{
    if (IsActiveActor(actor)) {
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
    if (IsActiveActor(actor)) {
        // We don't want to run RemoveNonRagdollRigidBodiesFromWorld immediately since the stuff is still yet to be added by AddHavok by a task.
        // So we need to make sure we run RemoveNonRagdollRigidBodiesFromWorld at least 1 frame later than the next task queue run.
        UInt32 handle = GetOrCreateRefrHandle(actor);
        auto job = [handle]() {
            g_taskInterface->AddTask(RemoveNonRagdollRigidBodiesFromWorldTask::Create(handle));
        };
        QueueDelayedJob(job, 0.1);
        return;
    }
    BSAnimationGraphManager_SetWorld(manager, a2);
}

bool IsPhysicallyBlocked(Actor *attacker)
{
    if (!Config::options.enablePhysicalBlock) return false;

    return false;
}

_HitData_populate Character_HitTarget_HitData_populate_Original = 0;
void Character_HitTarget_HitData_Populate_Hook(HitData *hitData, Actor *srcRefr, Actor *targetRefr, InventoryEntryData *weapon, bool isOffhand)
{
    if (targetRefr == *g_thePlayer) {
        // TODO: This works, but the game still does the cone check so I think it's possible it doesn't work if we aren't looking at the enemy?

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

        g_isCurrentHitFatal = hitData->flags & 0x20;
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
        if (hitData->flags >> 29 & 1) {
            // It's from physics damage - set the source form to the form of the object that hit them
            hitEvent->sourceFormID = g_physicsListener.lastPhysicsDamageFormId;
        }

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

_Actor_SetPosition g_Actor_SetPosition_Original = nullptr;
static RelocPtr<_Actor_SetPosition> Actor_SetPosition_vtbl(0x16D7338);
void Actor_SetPosition_Hook(Actor *actor, const NiPoint3 &pos, bool setCharControllerToo)
{
#ifdef _DEBUG
    TESFullName *name = DYNAMIC_CAST(actor->baseForm, TESForm, TESFullName);
    _MESSAGE("%d SetPosition %s", *g_currentFrameCounter, name->name);
#endif // _DEBUG

    if (Config::options.disableActorOnSetPositionTime > 0.0 && Config::options.disableActorOnSetPositionTimeJitter > 0.0) {
        double ignoreDuration = Config::options.disableActorOnSetPositionTime + GetRandomNumberInRange(0.f, Config::options.disableActorOnSetPositionTimeJitter);
        AddTemporaryIgnoredActor(actor, ignoreDuration);
    }

    g_Actor_SetPosition_Original(actor, pos, setCharControllerToo);
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
    else {
        if (ActorProcessManager *process = actor->processManager) {
            float knockedDownTimer = *(float *)&process->middleProcess->unk2B0;
            if (*g_fExplosionKnockStateExplodeDownTime - knockedDownTimer < Config::options.getUpMinTimeRagdolled) {
                return false;
            }
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

            if (IsActiveActor(actor)) {
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
    if (refr && IsActiveActor(refr)) return;

    GetUpEnd_NiNode_SetMotionTypeDownwards_Original(node, motionType, a3, a4, a5);
}

_BSTaskPool_QueueRemoveCollisionFromWorld GetUpEndHandler_Handle_RemoveCollision_BSTaskPool_QueueRemoveCollisionFromWorld_Original = 0;
void GetUpEndHandler_Handle_RemoveCollision_Hook(BSTaskPool *taskPool, NiAVObject *node)
{
    TESObjectREFR *refr = NiAVObject_GetOwner(node);
    if (refr && IsActiveActor(refr)) {
        UInt32 handle = GetOrCreateRefrHandle(refr);
        g_taskInterface->AddTask(RemoveNonRagdollRigidBodiesFromWorldTask::Create(handle));
        return;
    }

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
    const hkQsTransform *originalPoseBLocalSpaceOverride = originalPoseBLocalSpace;
    MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_1_Original(_this, poseAModelSpace, originalPoseBLocalSpaceOverride, poseBModelSpaceInOut, sourceOverride);
}

_hkaSkeletonMapper_mapPose MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_2_Original = 0;
void MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_Hook_2(hkaSkeletonMapper *_this, const hkQsTransform *poseAModelSpace, const hkQsTransform *originalPoseBLocalSpace, hkQsTransform *poseBModelSpaceInOut, UInt32 source)
{
    UInt32 sourceOverride = Config::options.dontRestrictBoneLengthsWhenMappingFromRagdollToAnim ? hkaSkeletonMapper::ConstraintSource::NO_CONSTRAINTS : source;
    const hkQsTransform *originalPoseBLocalSpaceOverride = originalPoseBLocalSpace;
    MapRagdollPoseToAnimPoseModelSpace_hkaSkeletonMapper_mapPose_2_Original(_this, poseAModelSpace, originalPoseBLocalSpaceOverride, poseBModelSpaceInOut, sourceOverride);
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

typedef void(*_bhkCollisionFilter_SetFromBGSCollisionLayers)();
_bhkCollisionFilter_SetFromBGSCollisionLayers bhkCollisionFilter_SetFromBGSCollisionLayers_Original = 0;
RelocAddr<uintptr_t> bhkCollisionFilter_SetFromBGSCollisionLayers_HookLoc(0x5BF24F);
void bhkCollisionFilter_SetFromBGSCollisionLayers_Hook()
{
    bhkCollisionFilter_SetFromBGSCollisionLayers_Original();

    bhkCollisionFilter *collisionFilter = *g_collisionFilter;
    g_originalBipedLayerBitfield = collisionFilter->layerBitfields[BGSCollisionLayer::kCollisionLayer_Biped];
    g_originalBipedNoCCLayerBitfield = collisionFilter->layerBitfields[BGSCollisionLayer::kCollisionLayer_BipedNoCC];
}

typedef void(*_Actor_UpdateFromCharacterControllerAndFootIK)(Actor *actor);
_Actor_UpdateFromCharacterControllerAndFootIK UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_Original = 0;
RelocAddr<uintptr_t> UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_HookLoc(0x703367);
void UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_Hook(Actor *actor)
{
    UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_Original(actor);

    if (bhkCharacterController *controller = GetCharacterController(actor)) {
        bool needsOrientationUpdate = fabs(controller->pitchAngle) > 0.001f || fabs(controller->rollAngle) > 0.001f;
        bool wouldDoOrientationUpdate = controller->calculatePitchTimer > 0.f && controller->doOrientationUpdate && ((controller->flags & 0x6000000) != 0 || controller->pitchAngle != 0.f || controller->rollAngle != 0.f);
        if (needsOrientationUpdate && !wouldDoOrientationUpdate) {
            if (Config::options.enableForcedOrientationUpdate) {
                controller->doOrientationUpdate = true;
                controller->calculatePitchTimer = 5.f; // 5 is the default in the base game too
            }
        }
    }
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
        std::uintptr_t originalFunc = Write5Call(bhkCollisionFilter_SetFromBGSCollisionLayers_HookLoc.GetUIntPtr(), uintptr_t(bhkCollisionFilter_SetFromBGSCollisionLayers_Hook));
        bhkCollisionFilter_SetFromBGSCollisionLayers_Original = (_bhkCollisionFilter_SetFromBGSCollisionLayers)originalFunc;
    }

    {
        std::uintptr_t originalFunc = Write5Call(UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_HookLoc.GetUIntPtr(), uintptr_t(UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_Hook));
        UpdateRagdollPostPhysics_Actor_UpdateFromCharacterControllerAndFootIK_Original = (_Actor_UpdateFromCharacterControllerAndFootIK)originalFunc;
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

class CellAttachDetachHandler : public BSTEventSink<TESCellAttachDetachEvent> {
public:
    virtual EventResult	ReceiveEvent(TESCellAttachDetachEvent *evn, EventDispatcher<TESCellAttachDetachEvent> *dispatcher)
    {
        bool attached = evn->attached;
        TESObjectREFR *refr = evn->reference;

        if (attached) return EventResult::kEvent_Continue; // We only care about detached

        if (refr->formType == kFormType_Character) {
            if (Actor *actor = DYNAMIC_CAST(refr, TESObjectREFR, Actor)) {
                RemoveActorFromWorldIfActive(actor);
            }
        }

        return EventResult::kEvent_Continue;
    }
};
CellAttachDetachHandler g_cellAttachDetachHandler;


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

        EventDispatcherList *eventDispatcherList = GetEventDispatcherList();
        if (eventDispatcherList) {
            eventDispatcherList->unk1B8.AddEventSink(&g_cellAttachDetachHandler);
        }
        else {
            ShowErrorBoxAndTerminate("Failed to get event dispatcher list");
            return;
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
                    if (higgsVersion < 1060000) {
                        ShowErrorBoxAndTerminate("[CRITICAL] HIGGS is present but is a lower version than is required by this mod. Get the latest version of HIGGS and try again.");
                    }

                    g_higgsInterface->AddCollisionFilterComparisonCallback(CollisionFilterComparisonCallback);
                    g_higgsInterface->AddPrePhysicsStepCallback(PrePhysicsStepCallback);
                    g_higgsInterface->AddPreVrikPreHiggsCallback(PreVrikPreHiggsCallback);
                    g_higgsInterface->AddGrabbedCallback(OnHiggsGrab);

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

        {
            g_Actor_SetPosition_Original = *Actor_SetPosition_vtbl;
            SafeWrite64(Actor_SetPosition_vtbl.GetUIntPtr(), uintptr_t(Actor_SetPosition_Hook));
        }

        g_timer.Start();

        return true;
    }
};

