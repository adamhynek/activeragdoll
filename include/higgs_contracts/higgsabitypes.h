#ifndef HIGGS_ABI_TYPES_H
#define HIGGS_ABI_TYPES_H
#include <stdint.h>
#if defined(_WIN32)
#define HIGGS_HAND_CALL __cdecl
#else
#define HIGGS_HAND_CALL
#endif

typedef uint32_t HIGGS_HandStatus;
typedef uint64_t HIGGS_HandClient;
typedef uint64_t HIGGS_HandRegistration;
typedef uint64_t HIGGS_HandExecutor;
typedef uint64_t HIGGS_HandGrant;
typedef uint64_t HIGGS_HandRequestId;
enum
{
    HIGGS_HAND_OK = 0,
    HIGGS_HAND_INVALID_ARGUMENT = 1,
    HIGGS_HAND_UNSUPPORTED_VERSION = 2,
    HIGGS_HAND_BUFFER_TOO_SMALL = 3,
    HIGGS_HAND_INVALID_CLIENT = 4,
    HIGGS_HAND_WRONG_OWNER = 5,
    HIGGS_HAND_NOT_READY = 6,
    HIGGS_HAND_STALE = 7,
    HIGGS_HAND_CAPACITY = 8,
    HIGGS_HAND_WRONG_PHASE = 9,
    HIGGS_HAND_CONFLICT = 10,
    HIGGS_HAND_NOT_FOUND = 11,
    HIGGS_HAND_PENDING = 12,
    HIGGS_HAND_UNSUPPORTED = 13,
    HIGGS_HAND_INTERNAL_ERROR = 14
};
enum
{
    HIGGS_HAND_LEFT = 1,
    HIGGS_HAND_RIGHT = 2,
    HIGGS_HAND_BOTH = 3
};

#pragma pack(push, 8)
typedef struct HIGGS_HandPartAttribution {
    uint32_t structSize, physicalHand, partId, bodyKey;
    uint64_t epoch, bodyGeneration, setId, appliedRevision, owner; /* Public registration label. */
} HIGGS_HandPartAttribution;
#pragma pack(pop)
#endif
