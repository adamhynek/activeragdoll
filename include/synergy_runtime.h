#pragma once
#include "planck_synergy.h"
#include "planck_physical_interop.h"
#include "synergy_drive_runtime.h"
#include "planck_compound_api.h"
#include <cstdint>
#include <vector>
namespace PlanckSynergy {
    struct ShapeChildSample { uintptr_t identity=0;PlanckShapeChild state{}; };
    struct BodySample {
        uintptr_t identity=0,lineage=0;PlanckBodyPhysics state{};
        // List identities are comparison keys, never native pointers to resolve.
        uintptr_t listIdentity=0;std::vector<ShapeChildSample> children;
    };
    // Native callers publish while holding graph -> world ownership. Stored
    // identities are comparison keys only; no native pointer is resolved here.
    struct DeferredEvent { uint32_t type; uint64_t subject; PlanckActorRef actor; uint32_t phase; };
    using DeferredEvents = std::vector<DeferredEvent>;
    const PlanckSynergyAPI* GetAPI();
    const PlanckPhysicalInteropAPI* GetPhysicalInteropAPI();
    const PlanckCompoundGeometryAPI* GetCompoundGeometryAPI();
    using NativeEndpointResolver = PlanckStatus(*)(PlanckToken,PlanckToken,
        const PlanckEndpoint&,PlanckEndpointVisitor,void*);
    // Install once at plugin startup. The resolver acquires graph -> world,
    // freshly resolves the actor/body, then calls VisitResolvedEndpoint.
    void InstallNativeEndpointResolver(NativeEndpointResolver resolver);
    using NativeGeometryExecutor = PlanckStatus(*)(PlanckToken requestId,const BodySample&,
        const PlanckGeometryRequest&,void*);
    constexpr PlanckStatus NativeGeometryPending=0x80000001u;
    // Capture/publication, then this pass, occur under one graph -> world write
    // acquisition. Only the freshly captured sample is given to the executor.
    void ProcessActorGeometry(uint32_t handle,uintptr_t modelIdentity,
        const std::vector<BodySample>& liveBodies,NativeGeometryExecutor,void*,DeferredEvents*);
    void SetGeometryExecutorAvailable(bool available);
    using NativeCompoundExecutor=PlanckStatus(*)(PlanckToken,const BodySample&,
        const PlanckCompoundProposal&,const std::vector<PlanckCapsuleEdit>&,void*,BodySample* applied);
    void SetCompoundExecutorAvailable(bool available);
    void ProcessActorCompoundGeometry(uint32_t handle,uintptr_t modelIdentity,
        const std::vector<BodySample>& liveBodies,NativeCompoundExecutor,void*,DeferredEvents*);
    PlanckStatus VisitResolvedEndpoint(PlanckToken client,PlanckToken lease,
        uintptr_t modelIdentity,uintptr_t bodyIdentity,uintptr_t lineage,
        const PlanckNativeEndpoint&,PlanckEndpointVisitor,void*);
    uint64_t MonotonicNowNs();
    void PublishActor(uint32_t handle, uintptr_t modelIdentity, bool ragdolled,
        uint64_t frame, uint64_t tick, double time, const std::vector<BodySample>& bodies,
        DeferredEvents* deferred = nullptr, uint64_t observedAtNs = 0);
    void InvalidateActor(uint32_t handle, DeferredEvents* deferred = nullptr);
    void DispatchEvents(const DeferredEvents& deferred);
    void InvalidateBody(uintptr_t identity);
    bool HasClients();
    uint64_t CaptureHitEpoch();
    bool IsHitEpochCurrent(uint64_t epoch);
    void Reset();
    void SetSuspended(bool suspended);
    PlanckHitSnapshot BeginHit(uint32_t kind, uint32_t target, uint32_t source,
        PlanckVec3 position, PlanckVec3 velocity, float damage, uint32_t hand,
        uint64_t frame, uint64_t tick);
    // Called exactly once by a successful native commitment or cancellation path.
    void FinishHit(uint64_t hitId, bool committed);
}
