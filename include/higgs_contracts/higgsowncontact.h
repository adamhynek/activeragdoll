#ifndef HIGGS_OWN_CONTACT_H
#define HIGGS_OWN_CONTACT_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#define HIGGS_OWN_CONTACT_VERSION 1u
#define HIGGS_OWN_CONTACT_CAP_HELD UINT64_C(1)
#define HIGGS_OWN_CONTACT_CAP_EQUIPPED_UID UINT64_C(2)
#define HIGGS_OWN_CONTACT_CAP_EQUIPPED_COLLIDER UINT64_C(4)
enum { HIGGS_CONTACT_ITEM_HELD=1, HIGGS_CONTACT_ITEM_EQUIPPED=2 };
enum { HIGGS_CONTACT_CLASS_WEAPON=1,HIGGS_CONTACT_CLASS_SHIELD=2,HIGGS_CONTACT_CLASS_OTHER=4 };
enum { HIGGS_COSMETIC_SOUND=1,HIGGS_COSMETIC_HAPTIC=2 };
enum { HIGGS_COSMETIC_ACTIVE=1,HIGGS_COSMETIC_ENDED=2 };
enum { HIGGS_COSMETIC_VERIFIED_PAIR=1,HIGGS_COSMETIC_SERVICE_NO_DAMAGE=2,HIGGS_COSMETIC_NATIVE_RESPONSE_DISABLED=4,HIGGS_COSMETIC_PLANCK_HIT_PATH_SKIPPED=8 };
enum { HIGGS_CONTACT_SOURCE_AVAILABLE=1,HIGGS_CONTACT_SOURCE_ABSENT=2,HIGGS_CONTACT_SOURCE_UNSUPPORTED=3,HIGGS_CONTACT_SOURCE_AMBIGUOUS=4,HIGGS_CONTACT_SOURCE_NOT_READY=5 };
enum { HIGGS_CONTACT_PROVENANCE_HELD_REFERENCE=1,HIGGS_CONTACT_PROVENANCE_EQUIPPED_UID=2,HIGGS_CONTACT_PROVENANCE_EQUIPPED_COLLIDER=4 };
#pragma pack(push,8)
typedef struct HIGGS_ContactItem {
    uint32_t structSize,physicalHand,source,itemClass,baseFormId,worldRefId,provenance,reserved;
    uint64_t itemInstance,itemGeneration,bodyGeneration,handGeneration;
} HIGGS_ContactItem;
typedef struct HIGGS_CosmeticContactPolicy {
    uint32_t structSize,feedbackKinds;
    uint64_t epoch,leaseId,expectedRevision;
    uint32_t leftSources,rightSources,leftClasses,rightClasses;
    float minRelativeSpeed,minClosingSpeed;
    uint32_t cooldownMs,leaseMs,maxObservationAgeMs,provenanceMask,reserved;
} HIGGS_CosmeticContactPolicy;
typedef struct HIGGS_CosmeticContactState {
    uint32_t structSize,stage;
    uint64_t leaseId,owner,epoch,revision,expiresAtNs;
    uint32_t reason,reserved;
} HIGGS_CosmeticContactState;
typedef struct HIGGS_CosmeticContact {
    uint32_t structSize,flags;
    uint64_t sequence,epoch,leaseId,policyRevision,producerObservation,producerObservedAtNs,observedAtNs,mainFrame,physicsTick;
    HIGGS_ContactItem left,right;
    float positionMetres[3],rightToLeftNormal[3],leftPointVelocity[3],rightPointVelocity[3],relativeSpeed,closingSpeed;
    uint32_t feedbackKinds,reserved;
} HIGGS_CosmeticContact;
typedef struct HIGGS_CosmeticContactQuery {
    uint32_t structSize,reserved;
    uint64_t epoch,afterSequence,leaseId;
    uint32_t feedbackKinds,reserved2;
} HIGGS_CosmeticContactQuery;
typedef struct HIGGS_ContactItemFrame {
    uint32_t structSize,itemCount;
    uint64_t epoch,observationId,observedAtNs;
    uint32_t sources[4]; /* left-held,left-equipped,right-held,right-equipped */
    HIGGS_ContactItem items[4];
} HIGGS_ContactItemFrame;
typedef struct HIGGS_CosmeticContactInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *RequestPolicy)(HIGGS_HandClient,const HIGGS_CosmeticContactPolicy*,HIGGS_CosmeticContactState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *EndPolicy)(HIGGS_HandClient,uint64_t,uint64_t);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyPolicy)(HIGGS_HandClient,uint64_t,HIGGS_CosmeticContactState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyContacts)(HIGGS_HandClient,const HIGGS_CosmeticContactQuery*,HIGGS_CosmeticContact*,uint32_t,uint32_t*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyItems)(HIGGS_HandClient,HIGGS_ContactItemFrame*);
} HIGGS_CosmeticContactInterface;
#pragma pack(pop)
/* Passive, opt-in cosmetic evidence only. No input grant, damage, solver change,
   audio or hardware pulse is produced by requesting a policy. Source masks use
   HELD=1 / EQUIPPED=2; class masks are WEAPON=1 / SHIELD=2 / OTHER=4.
   Identities name HIGGS-owned contact-item lifetimes, not serialized inventory IDs.
   Both endpoints have proved provider-owned bodies and nonreused generations.
   Provenance masks explicitly select retained HELD_REFERENCE episodes, existing
   unambiguous EQUIPPED_UID evidence, or HIGGS-owned EQUIPPED_COLLIDER episodes.
   Missing/duplicate worn UID cannot satisfy UID provenance; a policy may instead
   opt into collider provenance. A collider episode is not inventory identity.
   Old/model/world/hand changes and unsupported equipment fail closed. Sources
   and positions are sampled observations, not virtual IK goals. Publication
   age bounds are not proof of fresh hardware callbacks or controller tracking.
   Producer velocities are measured at the SAME contact point (including angular
   motion), in world-axis metres/s. Normal points right item -> left item.
   Closing speed is max(0,-dot(left-right velocity,normal)); relative speed is
   |left-right|, never the maximum of independent body speeds.
   Consumer polls in its safe phase, selects sound/assets, and schedules haptics
   through HIGGS's ordinary haptics service. A copied event is admitted evidence,
   not proof that sound/pulse was delivered. SERVICE_NO_DAMAGE means this service
   applies no damage; it does not certify a caller's separate combat outcome.
   The other flags report PLANCK's actual response-disable/hit-path branch.
   mainFrame/physicsTick are producer-assigned observations, not service timers.
   Coverage is contacts observed by the connected PLANCK listener, not all world
   manifolds. Capability bits name endpoint provenance, not contact enumeration.
   Old/ended/replaced-source events are not returned. CopyContacts retains at
   most 512 newest admitted events across leases; afterSequence is exclusive,
   and overflow drops oldest events. Required count is the current matching
   count (not a total-ever count); short capacity returns BUFFER_TOO_SMALL.
   CopyItems includes its producer time and availability of each source; callers
   must inspect these, not infer freshness from an earlier successful copy.
   Histories/leases are bounded; no external callback is retained.
   Exact v1 sized PODs, packing8, no native pointer in this gameplay interface. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryCosmeticContactInterfaceFn)(uint32_t,uint32_t,HIGGS_CosmeticContactInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryCosmeticContactInterface(uint32_t,uint32_t,HIGGS_CosmeticContactInterface*);
#ifdef __cplusplus
}
#endif
#endif
