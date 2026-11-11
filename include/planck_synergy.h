#ifndef PLANCK_SYNERGY_H
#define PLANCK_SYNERGY_H
/* Additive PLANCK revision 2. C11 / C++20, Windows x64, default packing (8).
 * Obtain GetApiFunction(2) through the existing SKSE PLANCK message transport.
 * All strings are UTF-8, all records must set size = sizeof(record).
 * No handles are persistent. Never free callback context until removal returns OK.
 * Callback functions must not throw, block, write engine objects or call inventory.
 * See docs/synergy-api.md for phase, ownership and capability limits. */
#include <stdint.h>
#pragma pack(push,8)
#if defined(_MSC_VER)
#define PLANCK_CALL __cdecl
#else
#define PLANCK_CALL
#endif
#ifdef __cplusplus
extern "C" {
#endif
typedef uint64_t PlanckToken;
typedef uint32_t PlanckStatus;
enum { PLANCK_OK=0, PLANCK_INVALID=1, PLANCK_STALE=2, PLANCK_NOT_FOUND=3,
    PLANCK_BUFFER=4, PLANCK_BUSY=5, PLANCK_UNSUPPORTED=6, PLANCK_LIMIT=7,
    PLANCK_CONFLICT=8, PLANCK_CANCELLED=9, PLANCK_REENTRANT=10 };
enum { PLANCK_PHASE_MAIN=1, PLANCK_PHASE_PRE_PHYSICS=2, PLANCK_PHASE_HIT=3,
    PLANCK_PHASE_LIFECYCLE=4 };
enum { PLANCK_ACTOR_CHANGED=1, PLANCK_ENDPOINT_INVALIDATED=2, PLANCK_HIT_CANDIDATE=4,
    PLANCK_HIT_FINALIZED=8, PLANCK_HIT_COMMITTED=PLANCK_HIT_FINALIZED, PLANCK_REQUEST_RESULT=16, PLANCK_RESET=32 };
enum { PLANCK_CAP_ACTORS=1, PLANCK_CAP_ENDPOINTS=2, PLANCK_CAP_HIT_POLICY=4,
    PLANCK_CAP_ACTOR_POLICY=8, PLANCK_CAP_CAPSULE_GEOMETRY=16, PLANCK_CAP_SAMPLE_TIME=32 };
enum { PLANCK_HIT_MELEE=1, PLANCK_HIT_CLUTTER=2 };
enum { PLANCK_HIT_PENDING=0, PLANCK_HIT_ALLOWED=1, PLANCK_HIT_DENIED=2,
    PLANCK_HIT_CANCELLED=3 };
enum { PLANCK_POLICY_SUPPRESS_HITS=1 };
enum { PLANCK_REQUEST_ACCEPTED=0, PLANCK_REQUEST_COMPLETED=1,
    PLANCK_REQUEST_REJECTED=2, PLANCK_REQUEST_CANCELLED=3 };
typedef struct PlanckVec3 { float x,y,z; } PlanckVec3;
typedef struct PlanckClientIdentity { uint32_t size; uint32_t reserved; char id[64]; } PlanckClientIdentity;
typedef struct PlanckActorRef {
    uint64_t epoch, actorGeneration; uint32_t handle, reserved;
} PlanckActorRef;
typedef struct PlanckBodyRef {
    PlanckActorRef actor; uint64_t bodyGeneration; uint32_t index, reserved;
} PlanckBodyRef;
typedef struct PlanckActorPhysics {
    uint32_t size, bodyCount; PlanckActorRef actor;
    uint64_t sample, mainFrame, physicsTick; double timeSeconds;
    uint32_t phase, ragdolled;
    /* Optional output tail from CopyActorPhysics. Zero means no source clock.
     * Check returned size before reading. QPC-based monotonic nanoseconds. */
    uint64_t observedAtNs;
} PlanckActorPhysics;
typedef struct PlanckBodyPhysics {
    uint32_t size, capsule; PlanckBodyRef body; uint64_t sample, geometryRevision;
    /* World and body-local axes use Skyrim/Havok axes, distances meters.
     * Rotation is xyzw, worldFromBody. Name is copied, truncated to 95 bytes. */
    PlanckVec3 worldPosition; float worldRotation[4];
    PlanckVec3 capsuleA, capsuleB; float capsuleRadius;
    char boneName[96];
} PlanckBodyPhysics;
typedef struct PlanckEndpointQuery {
    uint32_t size, reserved; PlanckBodyRef body; PlanckVec3 localPoint; float reserved2;
} PlanckEndpointQuery;
typedef struct PlanckEndpoint {
    uint32_t size, reserved; PlanckToken lease; PlanckBodyRef body;
    PlanckVec3 localPoint; float reserved2;
} PlanckEndpoint;
typedef struct PlanckActorPolicy {
    uint32_t size, flags; PlanckActorRef actor;
    /* Commutative restrictions: deny wins, smallest damage scale wins [0,1]. */
    float damageScale; uint32_t hitKinds;
} PlanckActorPolicy;
typedef struct PlanckHitSnapshot {
    uint32_t size, kind; uint64_t hitId, epoch, mainFrame, physicsTick;
    PlanckActorRef target; uint32_t sourceHandle, outcome;
    PlanckVec3 position, velocity; float proposedDamage, damageScale;
    /* proposedDamage=-1 for melee: engine damage is not yet populated.
     * Melee policy supports veto; damageScale is supported for clutter only. */
    uint32_t physicalHand, phase; /* 0 right, 1 left, 2 not applicable */
    uint64_t threadId; /* Provider-local process thread identity, not an OS handle. */
} PlanckHitSnapshot;
typedef struct PlanckHitDecision { uint32_t size, deny; float damageScale; uint32_t reserved; } PlanckHitDecision;
typedef struct PlanckHitPolicy {
    uint32_t size, hitKinds; PlanckActorRef actor; /* handle=0 means all actors */
} PlanckHitPolicy;
typedef struct PlanckEvent {
    uint32_t size, type; uint64_t sequence, epoch, subject;
    PlanckActorRef actor; uint32_t phase, reserved; uint64_t threadId;
} PlanckEvent;
typedef void (PLANCK_CALL *PlanckEventCallback)(const PlanckEvent*, void*);
typedef void (PLANCK_CALL *PlanckHitPolicyCallback)(const PlanckHitSnapshot*, PlanckHitDecision*, void*);
typedef struct PlanckGeometryRequest {
    uint32_t size, reserved; PlanckBodyRef body; uint64_t expectedRevision;
    /* Body-local meters. Direct native capsules only; positive radius <=10m.
     * Accepted work is applied by PLANCK at its safe graph/world phase. A
     * completed change persists until another proposal/body replacement;
     * releasing its request token does not roll back applied geometry. */
    PlanckVec3 capsuleA, capsuleB; float radius; uint32_t reserved2;
} PlanckGeometryRequest;
typedef struct PlanckRequestResult {
    uint32_t size, stage; PlanckToken request, lease; uint64_t epoch;
    PlanckStatus status; uint32_t reserved; uint64_t geometryRevision;
} PlanckRequestResult;
typedef struct PlanckSynergyAPI {
    uint32_t size, version; uint64_t capabilities;
    PlanckStatus (PLANCK_CALL *RegisterClient)(const PlanckClientIdentity*, PlanckToken*);
    PlanckStatus (PLANCK_CALL *ReleaseClient)(PlanckToken);
    PlanckStatus (PLANCK_CALL *Subscribe)(PlanckToken,uint64_t,PlanckEventCallback,void*,PlanckToken*);
    PlanckStatus (PLANCK_CALL *Unsubscribe)(PlanckToken,PlanckToken);
    PlanckStatus (PLANCK_CALL *CopyActorPhysics)(PlanckToken,uint32_t,PlanckActorPhysics*,PlanckBodyPhysics*,uint32_t,uint32_t*);
    PlanckStatus (PLANCK_CALL *AcquireAttachmentEndpoint)(PlanckToken,const PlanckEndpointQuery*,PlanckEndpoint*);
    PlanckStatus (PLANCK_CALL *ValidateAttachmentEndpoint)(PlanckToken,PlanckToken,PlanckEndpoint*);
    PlanckStatus (PLANCK_CALL *ReleaseLease)(PlanckToken,PlanckToken);
    PlanckStatus (PLANCK_CALL *RequestActorPolicy)(PlanckToken,const PlanckActorPolicy*,PlanckToken*);
    PlanckStatus (PLANCK_CALL *RegisterHitPolicy)(PlanckToken,const PlanckHitPolicy*,PlanckHitPolicyCallback,void*,PlanckToken*);
    PlanckStatus (PLANCK_CALL *CopyHitSnapshot)(PlanckToken,uint64_t,PlanckHitSnapshot*);
    PlanckStatus (PLANCK_CALL *RequestActorGeometry)(PlanckToken,const PlanckGeometryRequest*,PlanckToken*);
    PlanckStatus (PLANCK_CALL *GetRequestResult)(PlanckToken,PlanckToken,PlanckRequestResult*);
    PlanckStatus (PLANCK_CALL *CancelRequest)(PlanckToken,PlanckToken);
    /* Compatibility for clients with a 72-byte actor snapshot. Check table
     * size and PLANCK_CAP_SAMPLE_TIME before calling.
     * QPC-based monotonic nanoseconds at body capture, for this exact sample. */
    PlanckStatus (PLANCK_CALL *CopyActorSampleTime)(PlanckToken,PlanckActorRef,uint64_t sample,uint64_t* observedAtNs);
} PlanckSynergyAPI;
#ifdef __cplusplus
}
#endif
#pragma pack(pop)
#endif
