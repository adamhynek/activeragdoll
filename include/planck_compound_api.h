#ifndef PLANCK_COMPOUND_API_H
#define PLANCK_COMPOUND_API_H
/* Additive GetApiFunction(5), version 1. Existing revision-2 clients, events,
 * request results, cancellation and ReleaseLease own these requests too.
 * Copied body-local geometry uses meters and Havok axes. No native pointers. */
#include "planck_synergy.h"
#pragma pack(push,8)
#ifdef __cplusplus
extern "C" {
#endif
enum { PLANCK_SHAPE_UNKNOWN=0, PLANCK_SHAPE_CAPSULE=1, PLANCK_SHAPE_LIST=2 };
enum { PLANCK_COMPOUND_COPY=1, PLANCK_COMPOUND_APPLY=2 };
typedef struct PlanckBodyGeometry {
    uint32_t size,shapeKind; PlanckBodyRef body;
    uint64_t sample,geometryRevision; uint32_t childCount,reserved;
} PlanckBodyGeometry;
typedef struct PlanckShapeChild {
    uint32_t size,index,shapeKind,enabled;
    uint32_t collisionFilter,reserved; PlanckVec3 capsuleA,capsuleB;
    float radius; uint32_t reserved2;
} PlanckShapeChild;
typedef struct PlanckCapsuleEdit {
    uint32_t size,index; PlanckVec3 capsuleA,capsuleB;
    float radius; uint32_t reserved;
} PlanckCapsuleEdit;
typedef struct PlanckCompoundProposal {
    uint32_t size,editCount; PlanckBodyRef body;
    uint64_t expectedRevision,reserved;
} PlanckCompoundProposal;
typedef struct PlanckCompoundGeometryAPI {
    uint32_t size,version; uint64_t capabilities;
    PlanckStatus (PLANCK_CALL *CopyBodyGeometry)(PlanckToken,PlanckBodyRef,
        PlanckBodyGeometry*,PlanckShapeChild*,uint32_t,uint32_t*);
    /* Atomic for one existing body/list, at most 256 distinct capsule edits.
     * Child indices are valid only for this exact body + geometryRevision.
     * Topology, enabled state, filter and material are preserved. */
    PlanckStatus (PLANCK_CALL *RequestCapsuleEdits)(PlanckToken,
        const PlanckCompoundProposal*,const PlanckCapsuleEdit*,PlanckToken*);
} PlanckCompoundGeometryAPI;
#ifdef __cplusplus
}
#endif
#pragma pack(pop)
#endif
