#ifndef PLANCK_DRIVE_API_H
#define PLANCK_DRIVE_API_H
#include "planck_synergy.h"
#pragma pack(push,8)
#ifdef __cplusplus
extern "C" {
#endif
/* GetApiFunction(4), version1. Reuses revision2 client tokens, Subscribe,
 * Unsubscribe and ReleaseLease. Every record sets size; reserved fields zero.
 * No native pointer or caller-owned array survives a call. */
enum { PLANCK_DRIVE_OBSERVED=64, PLANCK_DRIVE_POLICY_INVALIDATED=128 };
enum { PLANCK_PHASE_DRIVE=5 };
enum { PLANCK_DRIVE_BLOCK_PIVOT_LOOSENING=1 };
enum { PLANCK_DRIVE_HIERARCHY_GAIN=1, PLANCK_DRIVE_VELOCITY_GAIN=2,
    PLANCK_DRIVE_POSITION_GAIN=4 };
enum { PLANCK_DRIVE_POLICY_ACCEPTED=0, PLANCK_DRIVE_POLICY_OBSERVED=1,
    PLANCK_DRIVE_POLICY_STALE=2 };
enum { PLANCK_DRIVE_WEIGHT_SATURATED=1 };

typedef struct PlanckDriveWeight {
    uint32_t size,reserved; PlanckBodyRef body; float multiplier; uint32_t reserved2;
} PlanckDriveWeight;

typedef struct PlanckDrivePolicy {
    uint32_t size,flags; PlanckActorRef actor;
    uint32_t gainMask,weightCount;
    /* Selected gains [0,1]; unselected fields must be zero. Multipliers [0,8].
     * Exact body refs identify weights; unmentioned bodies retain multiplier1.
     * Only one numerical/weight policy may own an actor at once (CONFLICT).
     * Block-pivot-only leases compose by OR and can coexist. */
    float hierarchyGain,velocityGain,positionGain; uint32_t reserved;
} PlanckDrivePolicy;

typedef struct PlanckDrivePolicyState {
    uint32_t size,state; PlanckToken lease; PlanckActorRef actor;
    uint64_t lastObservedSample;
} PlanckDrivePolicyState;

typedef struct PlanckDriveBody {
    uint32_t size,flags; PlanckBodyRef body;
    float requestedMultiplier,engineWeight,effectiveWeight,hierarchyFraction;
} PlanckDriveBody;

typedef struct PlanckDriveSnapshot {
    uint32_t size,bodyCount; PlanckActorRef actor;
    uint64_t sample,mainFrame,physicsTick; PlanckToken numericalPolicy;
    uint32_t phase,blockPivotLoosening; PlanckStatus status; uint32_t reserved;
    float hierarchyGain,velocityGain,positionGain; uint32_t paletteCount;
} PlanckDriveSnapshot;

typedef struct PlanckDriveAPI {
    uint32_t size,version;
    PlanckStatus (PLANCK_CALL *RequestActorDrivePolicy)(PlanckToken,
        const PlanckDrivePolicy*,const PlanckDriveWeight*,PlanckToken*);
    PlanckStatus (PLANCK_CALL *CopyActorDrive)(PlanckToken,uint32_t,
        PlanckDriveSnapshot*,PlanckDriveBody*,uint32_t,uint32_t*);
    PlanckStatus (PLANCK_CALL *CopyDrivePolicyState)(PlanckToken,PlanckToken,PlanckDrivePolicyState*);
} PlanckDriveAPI;
#ifdef __cplusplus
}
#endif
#pragma pack(pop)
#endif
