#ifndef HIGGSNATIVEINTEROP_H
#define HIGGSNATIVEINTEROP_H
#include "higgsabitypes.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)

#define HIGGS_HAND_PARTS_NATIVE_ABI_VERSION 2u
typedef struct HIGGS_HandPartsNativeInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities; /* bit0 read-only borrowed-body attribution. */
    HIGGS_HandStatus(HIGGS_HAND_CALL *ResolveBody)(uint64_t nativeBody,HIGGS_HandPartAttribution*);
} HIGGS_HandPartsNativeInterface;
/* Opt-in native physics-provider bridge, NOT a gameplay/body-lifetime API.
   The caller already holds a valid live hkpRigidBody and its own world/lifetime
   scope; nativeBody is that object's address used ONLY as a comparison key.
   HIGGS never dereferences the input address, calls the broker, acquires an engine
   lock, mutates physics or returns a native pointer. An immutable attribution
   snapshot retains owned wrappers while comparing, preventing address reuse.
   Output is copied metadata; epoch/bodyGeneration are runtime identities. A
   prepared disabled body can have appliedRevision0. Never cache, deserialize or
   manufacture nativeBody keys. Call only while the borrowed object is valid.
   Portable/early builds advertise capability0 and return NOT_READY; retry after
   native maintenance installs the actual resolver. Exact v2 table. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandPartsNativeInterfaceFn)(uint32_t,uint32_t,HIGGS_HandPartsNativeInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandPartsNativeInterface(uint32_t,uint32_t,HIGGS_HandPartsNativeInterface*);

enum {HIGGS_NATIVE_OWN_CONTACT_PLANCK=1};
enum {HIGGS_NATIVE_OWN_RESPONSE_DISABLED=1,HIGGS_NATIVE_OWN_HIT_PATH_SKIPPED=2};
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

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif
