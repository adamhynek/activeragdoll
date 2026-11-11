#ifndef HIGGS_HAND_API_H
#define HIGGS_HAND_API_H
/* Additive, copied C ABI. Official001, restrictions v1 and motion v1 are unchanged. */
#include <stdint.h>
#if defined(_WIN32)
#define HIGGS_HAND_CALL __cdecl
#else
#define HIGGS_HAND_CALL
#endif
#ifdef __cplusplus
extern "C"
{
#endif
#define HIGGS_HAND_ABI_VERSION 1u
#define HIGGS_HAND_CAP_SNAPSHOT UINT64_C(1)
#define HIGGS_HAND_CAP_INPUT_BROKER UINT64_C(2)
#define HIGGS_HAND_CAP_OWNED_GRANTS UINT64_C(4)
#define HIGGS_HAND_CAP_EXECUTOR_RECEIPTS UINT64_C(8)
#define HIGGS_HAND_CAP_PHYSICAL_REQUESTS UINT64_C(16)
#define HIGGS_HAND_CAP_INTERACTION_EVENTS UINT64_C(32)
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
    /* occupancy is meaningful ONLY when both HELD_KNOWN and EQUIPMENT_KNOWN are set. */
    enum
    {
        HIGGS_OCCUPANCY_EMPTY = 0,
        HIGGS_OCCUPANCY_GAME_EQUIPPED = 1,
        HIGGS_OCCUPANCY_HIGGS_HELD = 2,
        HIGGS_OCCUPANCY_BOTH = 3
    };
    enum
    {
        HIGGS_AVAILABILITY_UNKNOWN = 0,
        HIGGS_AVAILABILITY_READY = 1,
        HIGGS_AVAILABILITY_RESERVED = 2,
        HIGGS_AVAILABILITY_UNAVAILABLE = 3
    };
    enum
    {
        HIGGS_OBS_HELD_KNOWN = 1,
        HIGGS_OBS_EQUIPMENT_KNOWN = 2,
        HIGGS_OBS_PALM_VALID = 4,
        HIGGS_OBS_TRACKING_VALID = 8,
        HIGGS_OBS_SECONDARY_SUPPORT = 16
    };
    enum
    {
        HIGGS_HAND_REASON_NONE = 0,
        HIGGS_HAND_REASON_OBSERVATION = 1,
        HIGGS_HAND_REASON_RESTRICTION = 2,
        HIGGS_HAND_REASON_MENU = 4,
        HIGGS_HAND_REASON_TRACKING = 8,
        HIGGS_HAND_REASON_EXISTING_USE = 16
    };
    enum
    {
        HIGGS_INPUT_GRIP = 1,
        HIGGS_INPUT_TRIGGER = 2
    };
    enum
    {
        HIGGS_INPUT_DOWN = 1,
        HIGGS_INPUT_UP = 2
    };
    enum
    {
        HIGGS_DECISION_USER_PRIORITY = 1u,
        HIGGS_DECISION_CLIENT_PRIORITY = 2u,
        HIGGS_DECISION_REGISTRATION_ORDER = 4u,
        HIGGS_DECISION_CANDIDATE_ORDER = 8u
    };
    enum
    {
        HIGGS_ACTION_ACCEPTED = 1,
        HIGGS_ACTION_COMMITTED = 2,
        HIGGS_ACTION_COMPLETED = 3,
        HIGGS_ACTION_REJECTED = 4,
        HIGGS_ACTION_CANCELLED = 5
    };
    enum
    {
        HIGGS_RECEIPT_INVENTORY_COMMITTED = 1,
        HIGGS_RECEIPT_PHYSICAL_CONFIRMED = 2,
        HIGGS_RECEIPT_COMPENSATED = 4
    };
    enum
    {
        HIGGS_EVENT_GRABBED = 1,
        HIGGS_EVENT_RELEASED = 2,
        HIGGS_EVENT_SECONDARY_STARTED = 3,
        HIGGS_EVENT_SECONDARY_ENDED = 4,
        HIGGS_EVENT_ACTION_RESULT = 5,
        HIGGS_EVENT_RESET = 6,
        HIGGS_EVENT_CONSUMPTION_REQUESTED = 7,
        HIGGS_EVENT_STASH_REQUESTED = 8
    };
    enum
    {
        HIGGS_PHYSICAL_GRAB = 1,
        HIGGS_PHYSICAL_RELEASE = 2,
        HIGGS_PHYSICAL_SECONDARY_SUPPORT = 3 /* existing HIGGS close weapon selection only. */
    };

#pragma pack(push, 8)
    typedef struct HIGGS_HandClientInfo
    {
        uint32_t structSize;
        int32_t userPriority;
        char name[64];
    } HIGGS_HandClientInfo;
    typedef struct HIGGS_HandRecord
    {
        uint32_t physicalHand, observationMask, occupancy, availability;
        uint32_t unavailableReasons, heldRefId, equippedFormId, reserved;
        uint64_t generation, poseObservationId, poseObservedAtNs;
        HIGGS_HandGrant grant;
        HIGGS_HandClient owner;
        float palmPosition[3]; /* Skyrim world units, world axes, resolved HIGGS palm. */
        float palmRotation[9]; /* row-major, no VRIK final-pose guarantee. */
    } HIGGS_HandRecord;
    typedef struct HIGGS_HandSnapshot
    {
        uint32_t structSize, worldReady;
        uint64_t epoch, generation, observationId, sampledAtNs;
        HIGGS_HandRecord hands[2]; /* physical left, right; copied together after hand update. */
    } HIGGS_HandSnapshot;
    typedef struct HIGGS_HandInput
    {
        uint32_t structSize, physicalHand, button, edge;
        uint64_t inputId, epoch, handGeneration, poseObservationId, observedAtNs;
        HIGGS_HandSnapshot snapshot; /* Exact retained observation used for recognition. */
    } HIGGS_HandInput;
    typedef struct HIGGS_ActionCandidate
    {
        uint32_t structSize, physicalHands;
        uint64_t inputId, epoch, handGeneration;
        HIGGS_HandExecutor executor;
        uint64_t actionId, contextId, contextGeneration;
        uint32_t grantDurationMs, reserved; /* 1..60000. Priority comes from client registration. */
    } HIGGS_ActionCandidate;
    typedef struct HIGGS_ActionRequest
    {
        uint32_t structSize, physicalHands;
        HIGGS_HandRequestId requestId;
        HIGGS_HandClient owner;
        HIGGS_HandExecutor executor;
        HIGGS_HandGrant grant;
        uint64_t epoch, inputId, actionId, contextId, contextGeneration, poseObservationId;
    } HIGGS_ActionRequest;
    typedef struct HIGGS_ActionResult
    {
        uint32_t structSize, stage;
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch, inputId;
        uint32_t receiptFlags, itemRefId, quantity, reason;
    } HIGGS_ActionResult;
    typedef struct HIGGS_InputDecision
    {
        uint32_t structSize, consumed;
        uint64_t inputId, epoch;
        HIGGS_HandClient owner;
        HIGGS_HandGrant grant;
        HIGGS_HandRequestId requestId;
        int32_t effectivePriority;
        uint32_t reason;
    } HIGGS_InputDecision;
    typedef struct HIGGS_HandEvent
    {
        uint32_t structSize, type, physicalHand, itemRefId;
        uint64_t eventId, epoch, observationId, poseObservationId, observedAtNs;
        HIGGS_ActionResult result; /* only ACTION_RESULT; physical changes are observations. */
        HIGGS_HandRecord hand;     /* exact observation for grab/release/support events. */
    } HIGGS_HandEvent;
    typedef struct HIGGS_PhysicalRequest
    {
        uint32_t structSize, kind, physicalHand, itemRefId;
        uint64_t epoch, handGeneration;
        HIGGS_HandGrant grant;
        uint32_t timeoutMs, reserved;
    } HIGGS_PhysicalRequest;
    typedef void(HIGGS_HAND_CALL *HIGGS_RecognizerCallback)(const HIGGS_HandInput *, void *);
    typedef void(HIGGS_HAND_CALL *HIGGS_ExecutorCallback)(const HIGGS_ActionRequest *, void *);
    typedef void(HIGGS_HAND_CALL *HIGGS_HandEventCallback)(const HIGGS_HandEvent *, void *);
    typedef struct HIGGS_HandInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterClient)(const HIGGS_HandClientInfo *, HIGGS_HandClient *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *UnregisterClient)(HIGGS_HandClient);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopySnapshot)(HIGGS_HandClient, HIGGS_HandSnapshot *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubscribeRecognizer)(HIGGS_HandClient, HIGGS_RecognizerCallback, void *,
                                                               HIGGS_HandRegistration *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterExecutor)(HIGGS_HandClient, HIGGS_ExecutorCallback, void *,
                                                            HIGGS_HandExecutor *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubscribeEvents)(HIGGS_HandClient, HIGGS_HandEventCallback, void *,
                                                           HIGGS_HandRegistration *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RemoveRegistration)(HIGGS_HandClient, uint64_t);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubmitCandidate)(HIGGS_HandClient, const HIGGS_ActionCandidate *,
                                                           HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *ReportResult)(HIGGS_HandClient, const HIGGS_ActionResult *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyActionResult)(HIGGS_HandClient, HIGGS_HandRequestId,
                                                            HIGGS_ActionResult *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyInputDecision)(HIGGS_HandClient, uint64_t, HIGGS_InputDecision *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *ReleaseGrant)(HIGGS_HandClient, HIGGS_HandGrant);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestPhysical)(HIGGS_HandClient, const HIGGS_PhysicalRequest *,
                                                           HIGGS_HandRequestId *);
    } HIGGS_HandInterface;
#pragma pack(pop)
    /* All copied reads and registration/receipt calls serialize and may run on any
     thread. Recognizers/executors run synchronously on the controller callback
     thread BEFORE HIGGS defaults. Recognizers may ONLY read/submit candidates or
     mutate registrations; inventory mutation belongs to the winning executor.
     SubmitCandidate is valid only inside that client's recognizer for this input.
     External executors must queue engine work to their safe phase and report actual
     results; callbacks do not authorize unsafe engine calls. No pointers are retained
     except registered callbacks/context, which must remain alive until drainage.
     RemoveRegistration/UnregisterClient from outside ALL provider callbacks drain
     in-flight calls before OK. From any provider callback removal deactivates at
     once and returns PENDING: call again outside callbacks for confirmed drainage.
     A removed token remains a tombstone until drainage is confirmed. Registering
     during dispatch takes effect on the NEXT input/event. Never unload on PENDING.
     Epoch changes invalidate grants/pending requests; registration survives loads.
     Deadlines use provider steady clock. Consumers must renew by a new action after
     expiry; no hardware timestamp or callback-silence freshness is promised.
     Physical requests are queued, dispatched in HIGGS's main update and completed
     only after an actual matching hold transition. Acceptance is not completion.
     Successful outputs copy known bytes only; failures leave output unchanged except
     BUFFER_TOO_SMALL sets structSize. No C++ exceptions may cross callbacks.
     Exact v1 table only; sizes/layout immutable. Stop callers before DLL unload. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandInterfaceFn)(uint32_t, uint32_t, HIGGS_HandInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandInterface(uint32_t version, uint32_t tableSize,
                                                              HIGGS_HandInterface *);
#ifdef __cplusplus
}
#endif
#endif
