#pragma once
#include "planck_synergy.h"

#pragma pack(push, 8)
#ifdef __cplusplus
extern "C" {
#endif
enum { PLANCK_ADMITTED=1, PLANCK_HANDLER_INVOKED=2, PLANCK_POPULATED_MELEE=4,
    PLANCK_SCALE_APPLIED=8, PLANCK_SCALE_UNAVAILABLE=16, PLANCK_FACTOR_SELECTED=32 };
typedef struct PlanckHitEvidence {
    uint32_t size, facts;
    uint64_t hitId, epoch, contactId; /* zero contactId = causal contact unavailable */
    float populatedTotal, appliedFactor; /* Sources: POPULATED_MELEE and FACTOR_SELECTED. */
    char selectedRule[64];
} PlanckHitEvidence;
/* Return 0 to pass or 1 with a finite factor in [0,1]. The source/target and
 * motion belong to the same PLANCK hit. Only populated ordinary melee damage
 * is eligible. Terminal/limb/critical flags are never recomputed here. */
typedef uint32_t (PLANCK_CALL *PlanckMeleeRule)(const PlanckHitSnapshot*, float* factor, void*);
typedef struct PlanckHitApplicationAPI {
    uint32_t size, version;
    PlanckStatus (PLANCK_CALL *CopyEvidence)(PlanckToken, uint64_t hitId, PlanckHitEvidence*);
    PlanckStatus (PLANCK_CALL *RegisterMeleeRule)(PlanckToken, const char* stableRuleId,
        PlanckMeleeRule, void*, PlanckToken* registration);
} PlanckHitApplicationAPI;
#ifdef __cplusplus
}
#endif
#pragma pack(pop)
