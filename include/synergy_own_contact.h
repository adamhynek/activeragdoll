#pragma once
#include "higgs_contracts/higgsowncontactnative.h"
#include <atomic>
namespace PlanckSynergy {
using OwnContactObserver=HIGGS_HandStatus(HIGGS_HAND_CALL*)(const HIGGS_NativeOwnContact*,uint64_t*);
inline std::atomic<OwnContactObserver> ownContactObserver{nullptr};
inline bool HasOwnContactObserver(){return ownContactObserver.load(std::memory_order_acquire)!=nullptr;}
inline bool InstallOwnContactObserver(const HIGGS_OwnContactNativeInterface& api){
    if(api.structSize<sizeof(api)||api.abiVersion!=1||!api.Observe)return false;
    // Capability0 is a valid early table; the owner returns NOT_READY until
    // its native identity publication is installed. Never invent a fallback.
    ownContactObserver.store(api.Observe,std::memory_order_release);return true;
}
inline void ObserveOwnContact(const HIGGS_NativeOwnContact& input)noexcept{
    if(const auto call=ownContactObserver.load(std::memory_order_acquire)){
        uint64_t sequence=0;try{call(&input,&sequence);}catch(...){}
    }
}
}
