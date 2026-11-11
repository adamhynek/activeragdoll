#pragma once
#include "planck_synergy.h"

#pragma pack(push, 8)
#ifdef __cplusplus
extern "C" {
#endif
/* Request 6. This is PLANCK's own world listener, before its response filters.
 * A sample confirms a reported potential contact, not a solver impulse or hit.
 * Register a revision-2 client; release the watch with ReleaseLease.
 * Reset, body replacement and loss of the observed actor make a watch stale. */
enum { PLANCK_PREFILTER_POTENTIAL=1, PLANCK_CONTACT_HISTORY_CAPACITY=64 };
typedef struct PlanckContactSample {
    uint32_t size, otherFormId; /* Reference form ID, zero for unresolvable world geometry. */
    PlanckBodyRef body, otherBody; /* otherBody is available if independently watched. */
    uint64_t sequence, mainFrame, physicsTick, observedAtNs, geometryRevision;
    uint32_t shapeKey, otherShapeKey; /* Proven direct-list capsule index; UINT32_MAX if unavailable. */
    PlanckVec3 positionMetres, normalTowardBody;
    PlanckVec3 bodyPointVelocity, otherPointVelocity; /* m/s at the same world point. */
    uint64_t contactId, otherGeometryRevision; /* PLANCK domain: (body.actor.epoch, contactId). */
    uint32_t stage, reserved;
} PlanckContactSample;
typedef struct PlanckContactsAPI {
    uint32_t size, version;
    PlanckStatus (PLANCK_CALL *WatchBody)(PlanckToken, PlanckBodyRef, PlanckToken* watch);
    /* Latest observation since afterSequence. NOT_FOUND means no newer sample.
     * Each watch starts at sequence 1. A gap means intermediate contacts were skipped;
     * CopyContact can recover them while they remain in the bounded history. */
    PlanckStatus (PLANCK_CALL *CopyLatest)(PlanckToken, PlanckToken watch, uint64_t afterSequence, PlanckContactSample*);
    /* Re-resolve the exact named pose generation (PLANCK_QUERY_ACTOR_POSE)
     * on the SKSE main task thread. Returns its current revision-2 body ID. */
    PlanckStatus (PLANCK_CALL *ResolvePoseBody)(PlanckToken, uint32_t actorHandle, const char* bone,
        uint64_t expectedPoseGeneration, PlanckBodyRef*);
    /* Additive version-1 tail: check table size before reading/calling this member.
     * Copy the exact (epoch, contactId) observed by this owned watch. Retains the
     * latest PLANCK_CONTACT_HISTORY_CAPACITY observations per watch. NOT_FOUND
     * means unobserved/evicted; STALE means old epoch or invalidated body/watch.
     * No nearest/latest substitution. Failures leave output unchanged. History
     * is discarded on ReleaseLease, client closure, reset or body replacement. */
    PlanckStatus (PLANCK_CALL *CopyContact)(PlanckToken, PlanckToken watch, uint64_t epoch,
        uint64_t contactId, PlanckContactSample*);
} PlanckContactsAPI;
#ifdef __cplusplus
}
#endif
#pragma pack(pop)
