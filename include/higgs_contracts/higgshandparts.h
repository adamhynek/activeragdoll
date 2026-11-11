#ifndef HIGGS_HAND_PARTS_H
#define HIGGS_HAND_PARTS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#define HIGGS_HAND_PART_LIMIT 8u
#define HIGGS_HAND_SOURCE_LIMIT 17u
enum { HIGGS_PART_BOX = 1 };
enum { HIGGS_PART_PENDING = 1, HIGGS_PART_ACTIVE = 2, HIGGS_PART_ENDING = 3,
       HIGGS_PART_RELEASED = 4, HIGGS_PART_BROKEN = 5 };
enum { HIGGS_PART_REASON_OWNER = 1, HIGGS_PART_REASON_EPOCH = 2,
       HIGGS_PART_REASON_TRACKING = 3, HIGGS_PART_REASON_MODEL = 4,
       HIGGS_PART_REASON_TIMEOUT = 5, HIGGS_PART_REASON_NATIVE = 6 };
enum { HIGGS_PART_BODY_PRESENT = 1, HIGGS_PART_COLLISION_ENABLED = 2,
       HIGGS_PART_KEYFRAME_ISSUED = 4, HIGGS_PART_ACTUAL_POSE = 8 };
enum { HIGGS_PART_SOURCE_VALID = 1 };
#pragma pack(push, 8)
typedef struct HIGGS_HandPartSource {
    uint32_t sourceId, flags;
    float position[3], rotation[9], modelScale;
} HIGGS_HandPartSource;
typedef struct HIGGS_HandPartSources {
    uint32_t structSize, physicalHand, sourceCount, reserved;
    uint64_t epoch, sampleId, observedAtNs, modelGeneration, handGeneration;
    uint64_t trackingGeneration, trackingObservationId, trackedAtNs;
    HIGGS_HandPartSource sources[HIGGS_HAND_SOURCE_LIMIT];
} HIGGS_HandPartSources;
typedef struct HIGGS_HandPartDefinition {
    uint32_t partId, shape, sourceId, reserved;
    float localPosition[3], localRotation[9], halfExtentsMetres[3];
} HIGGS_HandPartDefinition;
typedef struct HIGGS_HandPartSetRequest {
    uint32_t structSize, physicalHand, partCount, durationMs;
    uint64_t epoch, sampleId, modelGeneration, expectedRevision, setId;
    HIGGS_HandPartDefinition parts[HIGGS_HAND_PART_LIMIT];
} HIGGS_HandPartSetRequest;
typedef struct HIGGS_HandPartBody {
    uint32_t partId, flags, bodyKey, reserved;
    uint64_t bodyGeneration;
    float position[3], rotation[9], targetPosition[3], targetRotation[9];
} HIGGS_HandPartBody;
typedef struct HIGGS_HandPartSetState {
    uint32_t structSize, stage, reason, physicalHand;
    uint64_t owner, setId, epoch, requestRevision, appliedRevision,
        modelGeneration, sourceSampleId, observedAtNs, expiresAtNs;
    uint32_t bodyCount, reserved;
    HIGGS_HandPartBody bodies[HIGGS_HAND_PART_LIMIT];
} HIGGS_HandPartSetState;
typedef struct HIGGS_HandPartAttribution {
    uint32_t structSize, physicalHand, partId, bodyKey;
    uint64_t epoch, bodyGeneration, setId, appliedRevision, owner;
} HIGGS_HandPartAttribution;
typedef struct HIGGS_HandPartContact {
    uint32_t structSize, reserved;
    uint64_t contactSequence;
    HIGGS_HandPartAttribution part;
} HIGGS_HandPartContact;
typedef struct HIGGS_HandPartsInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopySources)(HIGGS_HandClient, uint32_t, HIGGS_HandPartSources*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *SetParts)(HIGGS_HandClient, const HIGGS_HandPartSetRequest*, HIGGS_HandPartSetState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *EndParts)(HIGGS_HandClient, uint64_t, uint64_t);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyParts)(HIGGS_HandClient, uint64_t, HIGGS_HandPartSetState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *ResolveBody)(HIGGS_HandClient, uint64_t, uint32_t, uint64_t, HIGGS_HandPartAttribution*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyContactAttribution)(HIGGS_HandClient, uint64_t, uint64_t, HIGGS_HandPartContact*);
} HIGGS_HandPartsInterface;
#pragma pack(pop)
/* Copied POD v1. Capabilities: bit0 fitted BOX parts, bit1 bone sources, bit2
   resolved-palm-local sources, bit3 live body attribution, bit4 contact attribution.
   CopyContactAttribution matches the exact ObservationInterface contact sequence
   and epoch, from a bounded512 ring; missing/evicted records return NOT_FOUND.
   It retains the identity at that contact even after the native body was removed.
   No thumb/palm replacement,
   capsule factory, arbitrary driver, provider collision-filter or damage policy.
   One exclusive provider set per physical hand; 1..8 unique nonzero part IDs.
   Source0 is the actual resolved HIGGS palm body; source1 is the skeletal hand;
   sources2..16 are thumb/index/middle/ring/little joints0..2 in that order.
   Sources have world positions in game units and proper row-major rotations.
   localPosition uses game units in source axes, with NO implicit model scaling;
   fitted half extents are metres. modelScale is evidence for the provider's fitter.
   SetParts copies a proposal matching the current source epoch/sample/model, max
   age100ms. setId=expectedRevision=0 creates; updates require both exact values.
   Updating renews a 1..60000ms lease. HIGGS owns keyframing, movement compensation,
   native filter policy and teardown; PPB owns fitting. Collision follows HIGGS's
   palm enablement, so holding/two-hand transitions do not create a second actuator.
   ACTIVE/appliedRevision means native bodies and keyframe commands were applied,
   NOT that physics reached the requested pose. ACTUAL_POSE and target are separate.
   Geometry-size changes recreate only the changed native body, changing its
   bodyGeneration. bodyKey is observed Havok UID, epoch-scoped; bodyGeneration is
   this API's nonreused runtime lifetime. ResolveBody requires both for ABA safety.
   EndParts is asynchronous; RELEASED/BROKEN confirms owned body removal. Loading,
   model replacement, tracking loss, timeout and unregister invalidate the set.
   No callbacks run under the service registry lock. Never persist runtime IDs. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandPartsInterfaceFn)(uint32_t,uint32_t,HIGGS_HandPartsInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandPartsInterface(uint32_t,uint32_t,HIGGS_HandPartsInterface*);
#ifdef __cplusplus
}
#endif
#endif
