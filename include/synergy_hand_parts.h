#pragma once
#include "higgs_contracts/higgshandpartsnative.h"
#include <atomic>

namespace PlanckSynergy {
    // Installed at PostPostLoad and immutable thereafter. HIGGS remains loaded
    // for PLANCK's lifetime, just as with the existing official001 dependency.
    using HandPartResolver=HIGGS_HandStatus(HIGGS_HAND_CALL*)(uint64_t,HIGGS_HandPartAttribution*);
    inline std::atomic<HandPartResolver> handPartResolver{nullptr};

    inline bool InstallHandPartResolver(const HIGGS_HandPartsNativeInterface& api){
        if(api.structSize<sizeof(api)||api.abiVersion!=1||!api.ResolveBody)return false;
        // A valid early table can have capabilities zero until HIGGS's first
        // native maintenance. ResolveBody then returns NOT_READY; retry on use.
        handPartResolver.store(api.ResolveBody,std::memory_order_release);return true;
    }

    inline uint32_t ResolvePhysicalHandPart(uintptr_t borrowedBody){
        const auto resolve=handPartResolver.load(std::memory_order_acquire);
        if(!resolve||!borrowedBody)return 0;
        HIGGS_HandPartAttribution result{};result.structSize=sizeof(result);
        try{if(resolve(static_cast<uint64_t>(borrowedBody),&result)!=HIGGS_HAND_OK)return 0;}catch(...){return 0;}
        if(result.structSize<sizeof(result)||(result.physicalHand!=HIGGS_HAND_LEFT&&result.physicalHand!=HIGGS_HAND_RIGHT)||
            !result.epoch||!result.bodyGeneration||!result.partId||!result.setId||!result.owner)return 0;
        return result.physicalHand;
    }

    class HandPartContactScope;
    inline thread_local const HandPartContactScope* currentHandPartContact=nullptr;

    // One copied observation serves admission and side interpretation throughout
    // a borrowed contact/pair. Provider reset cannot turn a known left part into
    // a right-hand bool fallback halfway through the same contact processing.
    class HandPartContactScope {
    public:
        HandPartContactScope(uintptr_t first,uintptr_t second):previous_(currentHandPartContact),
            first_(first),second_(second),firstHand_(ResolvePhysicalHandPart(first)),secondHand_(ResolvePhysicalHandPart(second)){
            currentHandPartContact=this;
        }
        ~HandPartContactScope(){currentHandPartContact=previous_;}
        HandPartContactScope(const HandPartContactScope&)=delete;
        HandPartContactScope& operator=(const HandPartContactScope&)=delete;
        bool Copy(uintptr_t body,uint32_t& hand)const{
            if(body==first_){hand=firstHand_;return true;}
            if(body==second_){hand=secondHand_;return true;}
            return false;
        }
    private:
        const HandPartContactScope* previous_;
        uintptr_t first_,second_;uint32_t firstHand_,secondHand_;
    };

    inline uint32_t PhysicalHandPart(uintptr_t borrowedBody){
        uint32_t hand=0;if(currentHandPartContact&&currentHandPartContact->Copy(borrowedBody,hand))return hand;
        return ResolvePhysicalHandPart(borrowedBody);
    }
}
