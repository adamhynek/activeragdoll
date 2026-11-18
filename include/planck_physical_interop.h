#ifndef PLANCK_PHYSICAL_INTEROP_H
#define PLANCK_PHYSICAL_INTEROP_H
#include "planck_synergy.h"
#pragma pack(push,8)
#ifdef __cplusplus
extern "C" {
#endif
/* Native-provider bridge, requested as GetApiFunction(3). Unlike the copied
 * gameplay API, this opt-in table exposes callback-scoped engine addresses.
 * Do not retain, free or dereference them after the visitor returns. The
 * visitor executes on PLANCK's observed actor-maintenance thread with graph
 * then world write ownership, outside the API registry mutex. Only compatible
 * Havok native adapters should consume this table. */
typedef struct PlanckNativeEndpoint {
    uint32_t size, phase;
    PlanckEndpoint endpoint;
    uint64_t rigidBody, world; /* borrowed hkpRigidBody*, hkpWorld*, Windows x64 */
    PlanckVec3 worldPosition; float worldRotation[4];
    uint64_t sample, geometryRevision;
} PlanckNativeEndpoint;

typedef PlanckStatus (PLANCK_CALL *PlanckEndpointVisitor)(const PlanckNativeEndpoint*,void*);

typedef struct PlanckPhysicalInteropAPI {
    uint32_t size, version;
    PlanckStatus (PLANCK_CALL *WithAttachmentEndpoint)(PlanckToken client,PlanckToken lease,
        PlanckEndpointVisitor visitor,void* context);
} PlanckPhysicalInteropAPI;
#ifdef __cplusplus
}
#endif
#pragma pack(pop)
#endif
