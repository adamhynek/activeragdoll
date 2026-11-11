#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { PLANCK_QUERY_ACTOR_POSE = 0x41504F53 }; /* Proposal; request through GetApiFunction. */
enum { PLANCK_POSE_OK, PLANCK_POSE_INVALID, PLANCK_POSE_UNAVAILABLE, PLANCK_POSE_BUSY, PLANCK_POSE_AMBIGUOUS };

typedef struct PlanckBodyPose {
    uint64_t observedAtNs; /* Windows QPC-based monotonic nanoseconds at the locked read. */
    float positionMetres[3];
    float rotationXyzw[4];
} PlanckBodyPose;

typedef struct PlanckActorPoseAPI {
    uint32_t size;
    uint32_t version;
    /* Call from an SKSE main-thread task before taking graph/physics locks.
     * Supply a current actor handle and a NUL-terminated, case-sensitive bone name.
     * OK returns one copied observation. No native identity or continuing lease. */
    uint32_t (*CopyBodyPose)(uint32_t actorHandle, const char* boneName, PlanckBodyPose* out);
} PlanckActorPoseAPI;

#ifdef __cplusplus
}
#endif
