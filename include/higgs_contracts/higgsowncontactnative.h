#ifndef HIGGS_OWN_CONTACT_NATIVE_H
#define HIGGS_OWN_CONTACT_NATIVE_H
#include "higgsowncontact.h"
#ifdef __cplusplus
extern "C" {
#endif
enum {HIGGS_NATIVE_OWN_CONTACT_PLANCK=1};
enum {HIGGS_NATIVE_OWN_RESPONSE_DISABLED=1,HIGGS_NATIVE_OWN_HIT_PATH_SKIPPED=2};
#pragma pack(push,8)
typedef struct HIGGS_NativeOwnContact {
    uint32_t structSize,producer;
    uint64_t bodyA,bodyB,world;
    float positionMetres[3],bToANormal[3],pointVelocityA[3],pointVelocityB[3];
    uint64_t mainFrame,physicsTick;
    uint32_t contactFlags,reserved;
} HIGGS_NativeOwnContact;
typedef struct HIGGS_OwnContactNativeInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *Observe)(const HIGGS_NativeOwnContact*,uint64_t* lastSequence);
} HIGGS_OwnContactNativeInterface;
#pragma pack(pop)
/* Physics-owner bridge. Caller owns both borrowed live bodies and their common
   world for the complete call. Addresses are comparison keys only: HIGGS does
   not dereference caller keys, acquire engine/broker locks, change contacts,
   invoke consumers or return a native pointer. PLANCK supplies actual paired
   contact-point velocities at its own-item classification branch. Flags report
   actual response-disable/hit-path decisions; mixed held/equipped forwarding
   preserves the original continuation. HIGGS resolves BOTH endpoints from one
   retained immutable snapshot. mainFrame/physicsTick belong to the producer;
   neither a key nor these counters proves ownership outside this scoped call.
   OK means one or more opted-in copied evidence records; NOT_FOUND means no
   matching policy/pair; no status authorizes damage or native contact response.
   Early/portable capability0 -> NOT_READY. Never cache/serialize native keys. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryOwnContactNativeInterfaceFn)(uint32_t,uint32_t,HIGGS_OwnContactNativeInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryOwnContactNativeInterface(uint32_t,uint32_t,HIGGS_OwnContactNativeInterface*);
#ifdef __cplusplus
}
#endif
#endif
