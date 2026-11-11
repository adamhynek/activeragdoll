#pragma once
#include "planck_drive_api.h"
#include <cstdint>
#include <vector>
namespace PlanckSynergy {
    struct DriveBonePlan {uintptr_t identity=0;PlanckBodyRef body{};float multiplier=1.f;};
    struct DrivePlan {
        PlanckActorRef actor{};uintptr_t lineage=0;
        PlanckToken numericalPolicy=0;uint32_t flags=0,gainMask=0,weightPolicy=0;
        float hierarchyGain=0,velocityGain=0,positionGain=0;
        std::vector<DriveBonePlan> bones;
    };
    struct DriveExecution {
        PlanckStatus status=PLANCK_NOT_FOUND;
        float hierarchyGain=0,velocityGain=0,positionGain=0;uint32_t paletteCount=0;
        bool pivotPolicyObserved=false;
        std::vector<PlanckDriveBody> bodies;
    };
    using NativeDriveAction=void(*)(const DrivePlan&,DriveExecution&,void*);
    const PlanckDriveAPI* GetDriveAPI();
    bool IsDrivePlanCurrent(const DrivePlan&);
    // Existing drive hook already owns the graph/world phase. The native gate
    // prevents reset/replacement during this admitted action; no registry mutex
    // is held while executing native code. All callbacks remain copied-only.
    void ExecuteNativeDrive(uintptr_t model,uintptr_t lineage,const uintptr_t* bodies,uint32_t count,
        uint64_t frame,uint64_t tick,NativeDriveAction,void*);
}
