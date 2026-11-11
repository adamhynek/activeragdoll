#pragma once
#include "synergy_drive_runtime.h"
#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vector>

namespace PlanckSynergy {
    inline thread_local const DrivePlan* currentDrivePlan=nullptr;
    inline thread_local DriveExecution* currentDriveExecution=nullptr;

    class ScopedDriveInvocation {
    public:
        ScopedDriveInvocation(const DrivePlan& plan,DriveExecution& execution):
            beforePlan_(currentDrivePlan),beforeExecution_(currentDriveExecution){
            currentDrivePlan=&plan;currentDriveExecution=&execution;
        }
        ~ScopedDriveInvocation(){currentDrivePlan=beforePlan_;currentDriveExecution=beforeExecution_;}
        ScopedDriveInvocation(const ScopedDriveInvocation&)=delete;
        ScopedDriveInvocation& operator=(const ScopedDriveInvocation&)=delete;
    private:
        const DrivePlan* beforePlan_;DriveExecution* beforeExecution_;
    };

    inline bool MayLoosenPivots(bool configured,uintptr_t lineage){
        if(currentDrivePlan&&currentDriveExecution&&currentDrivePlan->actor.handle&&currentDrivePlan->lineage==lineage&&IsDrivePlanCurrent(*currentDrivePlan)){
            currentDriveExecution->pivotPolicyObserved=true;
            if(currentDrivePlan->flags&PLANCK_DRIVE_BLOCK_PIVOT_LOOSENING)return false;
        }
        return configured;
    }

    // Controller is the engine's existing hkaRagdollRigidBodyController. The
    // template also lets the actual adapter execute against explicit CPU seams.
    // It never owns/changes the controller's bodies, pose, allocation or driver.
    template<class Controller> class ScopedDriveControls {
        using Control=std::remove_reference_t<decltype(std::declval<Controller&>().m_controlDataPalette[0])>;
    public:
        explicit ScopedDriveControls(Controller& controller):controller_(controller){
            auto* plan=currentDrivePlan;auto* execution=currentDriveExecution;
            if(!plan||!execution||!plan->actor.handle)return;
            execution->status=PLANCK_STALE;
            if(!IsDrivePlanCurrent(*plan))return;
            const auto count=plan->bones.size();
            if(reinterpret_cast<uintptr_t>(controller_.m_ragdollInstance)!=plan->lineage||
                !controller_.m_ragdollInstance||count==0||count>256||
                controller_.m_bodyData.m_numRigidBodies!=static_cast<int>(count)||!controller_.m_bodyData.m_rigidBodies)return;
            for(size_t i=0;i<count;++i)if(reinterpret_cast<uintptr_t>(controller_.m_bodyData.m_rigidBodies[i])!=plan->bones[i].identity)return;
            auto& palette=controller_.m_controlDataPalette;const int n=palette.getSize();
            if(n<=0||n>256){execution->status=PLANCK_LIMIT;return;}
            savedPalette_.assign(palette.begin(),palette.end());
            const bool adjustWeights=plan->weightPolicy||(plan->gainMask&PLANCK_DRIVE_HIERARCHY_GAIN);
            savedWeights_=controller_.m_bodyData.m_boneWeights;
            std::vector<PlanckDriveBody> observed;observed.reserve(count);weights_.reserve(count);
            for(size_t i=0;i<count;++i){
                // The public mapping is the input to the upcoming native call.
                // BodyData's private cache may still describe the previous call.
                const int index=controller_.m_bodyIndexToPaletteIndex.getSize()==static_cast<int>(count)?
                    controller_.m_bodyIndexToPaletteIndex[static_cast<int>(i)]:0;
                if(index<0||index>=n){execution->status=PLANCK_INVALID;return;}
                const float hierarchy=plan->gainMask&PLANCK_DRIVE_HIERARCHY_GAIN?plan->hierarchyGain:palette[index].m_hierarchyGain;
                const float engine=savedWeights_?savedWeights_[i]:1.f;
                if(!std::isfinite(hierarchy)||hierarchy<0.f||!std::isfinite(engine)||engine<0.f){execution->status=PLANCK_INVALID;return;}
                const float requested=engine*plan->bones[i].multiplier;
                const float effective=adjustWeights&&hierarchy>0.f?(std::min)(requested,1.f/hierarchy):adjustWeights?requested:engine;
                if(!std::isfinite(effective)){execution->status=PLANCK_INVALID;return;}
                weights_.push_back(effective);
                observed.push_back({sizeof(PlanckDriveBody),uint32_t(adjustWeights&&effective<requested?PLANCK_DRIVE_WEIGHT_SATURATED:0),
                    plan->bones[i].body,plan->bones[i].multiplier,engine,effective,hierarchy*effective});
            }
            execution->paletteCount=static_cast<uint32_t>(n);
            execution->hierarchyGain=plan->gainMask&PLANCK_DRIVE_HIERARCHY_GAIN?plan->hierarchyGain:palette[0].m_hierarchyGain;
            execution->velocityGain=plan->gainMask&PLANCK_DRIVE_VELOCITY_GAIN?plan->velocityGain:palette[0].m_velocityGain;
            execution->positionGain=plan->gainMask&PLANCK_DRIVE_POSITION_GAIN?plan->positionGain:palette[0].m_positionGain;
            execution->bodies=std::move(observed);
            for(int i=0;i<n;++i){
                if(plan->gainMask&PLANCK_DRIVE_HIERARCHY_GAIN)palette[i].m_hierarchyGain=plan->hierarchyGain;
                if(plan->gainMask&PLANCK_DRIVE_VELOCITY_GAIN)palette[i].m_velocityGain=plan->velocityGain;
                if(plan->gainMask&PLANCK_DRIVE_POSITION_GAIN)palette[i].m_positionGain=plan->positionGain;
            }
            paletteApplied_=plan->gainMask!=0;
            gainMask_=plan->gainMask;
            if(adjustWeights){controller_.m_bodyData.m_boneWeights=weights_.data();weightsApplied_=true;}
            execution->status=PLANCK_OK;
        }
        ~ScopedDriveControls(){
            if(weightsApplied_)controller_.m_bodyData.m_boneWeights=savedWeights_;
            if(paletteApplied_)for(size_t i=0;i<savedPalette_.size();++i){
                auto& current=controller_.m_controlDataPalette[static_cast<int>(i)];const auto& saved=savedPalette_[i];
                if(gainMask_&PLANCK_DRIVE_HIERARCHY_GAIN)current.m_hierarchyGain=saved.m_hierarchyGain;
                if(gainMask_&PLANCK_DRIVE_VELOCITY_GAIN)current.m_velocityGain=saved.m_velocityGain;
                if(gainMask_&PLANCK_DRIVE_POSITION_GAIN)current.m_positionGain=saved.m_positionGain;
            }
        }
        ScopedDriveControls(const ScopedDriveControls&)=delete;
        ScopedDriveControls& operator=(const ScopedDriveControls&)=delete;
    private:
        Controller& controller_;std::vector<Control> savedPalette_;std::vector<float> weights_;
        const float* savedWeights_=nullptr;bool paletteApplied_=false,weightsApplied_=false;uint32_t gainMask_=0;
    };
}
