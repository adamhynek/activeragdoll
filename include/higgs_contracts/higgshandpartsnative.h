#ifndef HIGGS_HAND_PARTS_NATIVE_H
#define HIGGS_HAND_PARTS_NATIVE_H
#include "higgshandparts.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push,8)
typedef struct HIGGS_HandPartsNativeInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities; /* bit0 read-only borrowed-body attribution. */
    HIGGS_HandStatus(HIGGS_HAND_CALL *ResolveBody)(uint64_t nativeBody,HIGGS_HandPartAttribution*);
} HIGGS_HandPartsNativeInterface;
#pragma pack(pop)
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
   native maintenance installs the actual resolver. Exact v1 table. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandPartsNativeInterfaceFn)(uint32_t,uint32_t,HIGGS_HandPartsNativeInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandPartsNativeInterface(uint32_t,uint32_t,HIGGS_HandPartsNativeInterface*);
#ifdef __cplusplus
}
#endif
#endif
