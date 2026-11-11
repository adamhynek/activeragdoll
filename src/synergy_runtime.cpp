#include "synergy_runtime.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace PlanckSynergy {
namespace {
constexpr size_t MaxClients=64, MaxResources=512, MaxBodies=256, MaxActors=512, MaxHits=256;
constexpr uint64_t EventMask=255;
struct Client { std::string id; bool closing=false; };
struct ActorState {
    uintptr_t model=0;PlanckActorPhysics state{};std::vector<BodySample> bodies;
    PlanckDriveSnapshot drive{};std::vector<PlanckDriveBody> driveBodies;
};
enum class Kind { Subscription, Endpoint, ActorPolicy, HitPolicy, Geometry, DrivePolicy };
struct Resource {
    Kind kind{}; PlanckToken owner=0; bool active=true; uint32_t inFlight=0;
    uint64_t mask=0; PlanckEventCallback event=nullptr; PlanckHitPolicyCallback hit=nullptr;
    void* context=nullptr;
    PlanckEndpoint endpoint{}; PlanckActorPolicy actorPolicy{}; PlanckHitPolicy hitPolicy{};
    PlanckGeometryRequest geometry{};PlanckRequestResult result{};bool dispatched=false,nativeCommitted=false;
    bool compound=false;std::vector<PlanckCapsuleEdit> edits;
    PlanckDrivePolicy drivePolicy{};std::vector<PlanckDriveWeight> driveWeights;
    uint64_t lastDriveSample=0;
};
struct Registry {
    std::mutex lock;
    // Never acquire graph/world ownership while holding this gate. Native
    // visitors enter with both locks already held; reset/invalidation waits
    // only for the admitted, synchronous native operation to finish.
    std::recursive_mutex nativeGate;
    NativeEndpointResolver endpointResolver=nullptr;
    bool geometryAvailable=false,compoundAvailable=false;
    uint64_t epoch=1, serial=1, sample=0, eventSequence=0;
    bool exhausted=false, suspended=false;
    std::map<PlanckToken,Client> clients;
    std::map<uint32_t,ActorState> actors;
    std::map<PlanckToken,std::shared_ptr<Resource>> resources;
    std::map<uint64_t,PlanckHitSnapshot> hits;
    std::deque<uint64_t> hitOrder;
    PlanckToken Next() {
        if (exhausted || serial==(std::numeric_limits<uint64_t>::max)()) { exhausted=true; return 0; }
        return serial++;
    }
};
std::atomic<bool> clientsRegistered{false};
Registry& State() { static Registry value; return value; }
thread_local uint32_t callbackDepth=0;
thread_local uint64_t threadToken=0;
uint64_t ThreadToken(Registry& registry){if(!threadToken)threadToken=registry.Next();return threadToken;}
struct CallbackScope { CallbackScope(){++callbackDepth;} ~CallbackScope(){--callbackDepth;} };
template<class T> bool Sized(const T* value) { return value && value->size>=sizeof(T); }
bool Finite(PlanckVec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
bool LocalPoint(PlanckVec3 v){return Finite(v)&&std::abs(v.x)<=100.f&&std::abs(v.y)<=100.f&&std::abs(v.z)<=100.f;}
bool Valid(PlanckActorRef v){return v.epoch&&v.actorGeneration&&v.handle&&!v.reserved;}
bool Valid(PlanckBodyRef v){return Valid(v.actor)&&v.bodyGeneration&&!v.reserved;}
bool Scale(float value) { return std::isfinite(value)&&value>=0.f&&value<=1.f; }
bool Same(PlanckActorRef a,PlanckActorRef b) { return a.epoch==b.epoch&&a.actorGeneration==b.actorGeneration&&a.handle==b.handle; }
bool Same(PlanckBodyRef a,PlanckBodyRef b) { return Same(a.actor,b.actor)&&a.bodyGeneration==b.bodyGeneration&&a.index==b.index; }
bool SameChildren(const BodySample& a,const BodySample& b){
    if(a.listIdentity!=b.listIdentity||a.children.size()!=b.children.size())return false;
    for(size_t i=0;i<a.children.size();++i){
        const auto& x=a.children[i];const auto& y=b.children[i];
        if(x.identity!=y.identity||x.state.index!=y.state.index||x.state.shapeKind!=y.state.shapeKind||
            x.state.enabled!=y.state.enabled||x.state.collisionFilter!=y.state.collisionFilter||x.state.radius!=y.state.radius||
            std::memcmp(&x.state.capsuleA,&y.state.capsuleA,sizeof(PlanckVec3))||
            std::memcmp(&x.state.capsuleB,&y.state.capsuleB,sizeof(PlanckVec3)))return false;
    }
    return true;
}
PlanckStatus ClientStatus(Registry& r,PlanckToken client,bool closing=false) {
    auto it=r.clients.find(client);
    return it==r.clients.end()?PLANCK_INVALID:(!closing&&it->second.closing?PLANCK_BUSY:PLANCK_OK);
}
BodySample* FindBody(Registry& r,PlanckBodyRef ref) {
    if(r.suspended)return nullptr;
    auto it=r.actors.find(ref.actor.handle);
    if (it==r.actors.end()||!Same(it->second.state.actor,ref.actor)||ref.index>=it->second.bodies.size()) return nullptr;
    auto& body=it->second.bodies[ref.index];
    return Same(body.state.body,ref)?&body:nullptr;
}
bool CurrentActor(Registry& r,PlanckActorRef actor) {
    if(r.suspended)return false;
    auto it=r.actors.find(actor.handle);
    return it!=r.actors.end()&&Same(actor,it->second.state.actor);
}
std::shared_ptr<Resource> Owned(Registry& r,PlanckToken owner,PlanckToken id) {
    auto it=r.resources.find(id);
    return it!=r.resources.end()&&it->second->owner==owner?it->second:nullptr;
}
void Emit(uint32_t type,uint64_t subject,PlanckActorRef actor,uint32_t phase) {
    auto& r=State(); std::vector<PlanckToken> callbacks; PlanckEvent ev{};
    {
        std::lock_guard guard(r.lock);
        ev={sizeof(ev),type,++r.eventSequence,r.epoch,subject,actor,phase,0,ThreadToken(r)};
        for(auto& [id,res]:r.resources) if(res->kind==Kind::Subscription&&res->active&&(res->mask&type)) callbacks.push_back(id);
    }
    // The engine owns lifecycle dispatch; recursive client emission is unavailable.
    // A malicious/internal recursive engine call is bounded, and never holds this lock.
    if(callbackDepth>=8) return;
    for(auto id:callbacks) {
        std::shared_ptr<Resource> res;
        {
            std::lock_guard guard(r.lock); auto it=r.resources.find(id);
            if(it==r.resources.end()||!it->second->active) continue;
            res=it->second; ++res->inFlight;
        }
        bool failed=false;
        { CallbackScope scope; auto copy=ev; try {res->event(&copy,res->context);} catch(...) {failed=true;} }
        { std::lock_guard guard(r.lock); --res->inFlight; if(failed)res->active=false; }
    }
}
void RetireActorLocked(Registry& r,PlanckActorRef actor,std::vector<uint64_t>& endpoints,std::vector<uint64_t>& cancelled) {
    for(auto& [id,res]:r.resources) {
        if(res->kind==Kind::Endpoint&&Same(res->endpoint.body.actor,actor)&&res->active) {res->active=false;endpoints.push_back(id);}
        if(res->kind==Kind::ActorPolicy&&Same(res->actorPolicy.actor,actor)) res->active=false;
        if(res->kind==Kind::Geometry&&Same(res->geometry.body.actor,actor)&&res->result.stage==PLANCK_REQUEST_ACCEPTED){
            res->active=false;res->nativeCommitted=false;res->result.stage=PLANCK_REQUEST_CANCELLED;res->result.status=PLANCK_STALE;cancelled.push_back(id);
        }
    }
}
void RetireDrivePoliciesLocked(Registry& r,PlanckActorRef actor,std::vector<uint64_t>& leases){
    for(auto& [id,res]:r.resources)if(res->kind==Kind::DrivePolicy&&res->active&&Same(res->drivePolicy.actor,actor)){
        res->active=false;leases.push_back(id);
    }
}
PlanckStatus RegisterClient(const PlanckClientIdentity* info,PlanckToken* output) {
    if(output)*output=0;
    if(!Sized(info)||!output||info->reserved||!info->id[0]||!std::memchr(info->id,0,sizeof(info->id)))return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::lock_guard guard(r.lock);
    if(r.clients.size()>=MaxClients)return PLANCK_LIMIT;
    for(auto& [id,c]:r.clients)if(c.id==info->id)return PLANCK_CONFLICT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    r.clients.emplace(id,Client{info->id,false});
    clientsRegistered.store(true, std::memory_order_release);
    *output=id;return PLANCK_OK;
}
PlanckStatus ReleaseClient(PlanckToken owner) {
    auto& r=State();std::lock_guard guard(r.lock);
    auto status=ClientStatus(r,owner,true);if(status)return status;
    r.clients.at(owner).closing=true;
    bool busy=false;
    for(auto& [id,res]:r.resources)if(res->owner==owner){res->active=false;busy|=res->inFlight!=0||res->nativeCommitted;}
    if(busy)return PLANCK_BUSY; // Call again; no later callback may start.
    std::erase_if(r.resources,[owner](auto& pair){return pair.second->owner==owner;});
    r.clients.erase(owner);
    if (r.clients.empty()) {
        clientsRegistered.store(false, std::memory_order_release);
        r.actors.clear();
        r.hits.clear();
        r.hitOrder.clear();
    }
    return PLANCK_OK;
}
PlanckStatus Subscribe(PlanckToken owner,uint64_t mask,PlanckEventCallback callback,void* context,PlanckToken* output) {
    if(output)*output=0;
    if(!output||!callback||!mask||(mask&~EventMask))return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(r.resources.size()>=MaxResources)return PLANCK_LIMIT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::Subscription;res->owner=owner;res->mask=mask;res->event=callback;res->context=context;
    r.resources.emplace(id,res);*output=id;return PLANCK_OK;
}
PlanckStatus Unsubscribe(PlanckToken owner,PlanckToken id) {
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner,true);if(status)return status;
    auto res=Owned(r,owner,id);if(!res||(res->kind!=Kind::Subscription&&res->kind!=Kind::HitPolicy))return PLANCK_INVALID;
    res->active=false;if(res->inFlight)return PLANCK_BUSY;
    r.resources.erase(id);return PLANCK_OK;
}
PlanckStatus CopyActorPhysics(PlanckToken owner,uint32_t actor,PlanckActorPhysics* output,PlanckBodyPhysics* bodies,uint32_t capacity,uint32_t* required) {
    constexpr size_t originalSize = offsetof(PlanckActorPhysics, observedAtNs);
    if (!output || output->size < originalSize || !required || (capacity && !bodies)) return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(r.suspended)return PLANCK_STALE;
    auto it=r.actors.find(actor);if(it==r.actors.end())return PLANCK_NOT_FOUND;
    *required=static_cast<uint32_t>(it->second.bodies.size());
    if(capacity<*required)return PLANCK_BUFFER;
    for(uint32_t i=0;i<*required;++i)if(!Sized(&bodies[i]))return PLANCK_INVALID;
    const auto bytes = output->size >= sizeof(*output) ? sizeof(*output) : originalSize;
    std::memcpy(output, &it->second.state, bytes);
    output->size = static_cast<uint32_t>(bytes);
    for(uint32_t i=0;i<*required;++i)bodies[i]=it->second.bodies[i].state;
    return PLANCK_OK;
}
PlanckStatus CopyActorSampleTime(PlanckToken owner, PlanckActorRef actor, uint64_t sample, uint64_t* observedAtNs) {
    if (!observedAtNs) return PLANCK_INVALID;
    *observedAtNs = 0;
    if (!Valid(actor) || !sample) return PLANCK_INVALID;
    auto& registry = State();
    std::lock_guard lock(registry.lock);
    if (auto status = ClientStatus(registry, owner); status != PLANCK_OK) return status;
    const auto it = registry.actors.find(actor.handle);
    if (registry.suspended || it == registry.actors.end() || !Same(it->second.state.actor, actor) ||
        it->second.state.sample != sample) return PLANCK_STALE;
    if (!it->second.state.observedAtNs) return PLANCK_NOT_FOUND;
    *observedAtNs = it->second.state.observedAtNs;
    return PLANCK_OK;
}
PlanckStatus AcquireEndpoint(PlanckToken owner,const PlanckEndpointQuery* query,PlanckEndpoint* output) {
    if(!Sized(query)||!Sized(output)||query->reserved||query->reserved2||!Valid(query->body)||!LocalPoint(query->localPoint))return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(!FindBody(r,query->body))return PLANCK_STALE;
    if(r.resources.size()>=MaxResources)return PLANCK_LIMIT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::Endpoint;res->owner=owner;
    res->endpoint={sizeof(PlanckEndpoint),0,id,query->body,query->localPoint,0};
    r.resources.emplace(id,res);*output=res->endpoint;return PLANCK_OK;
}
PlanckStatus ValidateEndpoint(PlanckToken owner,PlanckToken id,PlanckEndpoint* output) {
    if(!Sized(output))return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    auto res=Owned(r,owner,id);if(!res||res->kind!=Kind::Endpoint)return PLANCK_INVALID;
    if(!res->active||!FindBody(r,res->endpoint.body))return PLANCK_STALE;
    *output=res->endpoint;return PLANCK_OK;
}
PlanckStatus ReleaseLease(PlanckToken owner,PlanckToken id) {
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner,true);if(status)return status;
    auto res=Owned(r,owner,id);if(!res||(res->kind!=Kind::Endpoint&&res->kind!=Kind::ActorPolicy&&res->kind!=Kind::Geometry&&res->kind!=Kind::DrivePolicy))return PLANCK_INVALID;
    if(res->inFlight||res->dispatched||res->nativeCommitted)return PLANCK_BUSY;
    r.resources.erase(id);return PLANCK_OK;
}
PlanckStatus RequestPolicy(PlanckToken owner,const PlanckActorPolicy* policy,PlanckToken* output) {
    if(output)*output=0;
    if(!Sized(policy)||!output||!Valid(policy->actor)||(policy->flags&~PLANCK_POLICY_SUPPRESS_HITS)||!Scale(policy->damageScale)||!policy->hitKinds||(policy->hitKinds&~3u))return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(!CurrentActor(r,policy->actor))return PLANCK_STALE;
    if(r.resources.size()>=MaxResources)return PLANCK_LIMIT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::ActorPolicy;res->owner=owner;res->actorPolicy=*policy;
    r.resources.emplace(id,res);*output=id;return PLANCK_OK;
}
PlanckStatus RegisterHitPolicy(PlanckToken owner,const PlanckHitPolicy* policy,PlanckHitPolicyCallback callback,void* context,PlanckToken* output) {
    if(output)*output=0;
    if(!Sized(policy)||!output||!callback||!policy->hitKinds||(policy->hitKinds&~3u)||
        (policy->actor.handle?!Valid(policy->actor):(policy->actor.epoch||policy->actor.actorGeneration||policy->actor.reserved)))return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(policy->actor.handle&&!CurrentActor(r,policy->actor))return PLANCK_STALE;
    size_t count=0;for(auto& [id,res]:r.resources)if(res->kind==Kind::HitPolicy)++count;
    if(count>=16||r.resources.size()>=MaxResources)return PLANCK_LIMIT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::HitPolicy;res->owner=owner;res->hitPolicy=*policy;res->hit=callback;res->context=context;
    r.resources.emplace(id,res);*output=id;return PLANCK_OK;
}
PlanckStatus CopyHit(PlanckToken owner,uint64_t id,PlanckHitSnapshot* output) {
    if(!Sized(output))return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    auto it=r.hits.find(id);if(it==r.hits.end())return PLANCK_NOT_FOUND;
    *output=it->second;return PLANCK_OK;
}
PlanckStatus RequestGeometry(PlanckToken owner,const PlanckGeometryRequest* proposal,PlanckToken* output) {
    if(output)*output=0;
    if(!Sized(proposal)||!output||proposal->reserved||proposal->reserved2||!Valid(proposal->body)||!LocalPoint(proposal->capsuleA)||
        !LocalPoint(proposal->capsuleB)||!std::isfinite(proposal->radius)||proposal->radius<=0.f||proposal->radius>10.f||!proposal->expectedRevision)
        return PLANCK_INVALID;
    const double dx=double(proposal->capsuleA.x)-proposal->capsuleB.x;
    const double dy=double(proposal->capsuleA.y)-proposal->capsuleB.y;
    const double dz=double(proposal->capsuleA.z)-proposal->capsuleB.z;
    if(dx*dx+dy*dy+dz*dz<=1e-12)return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(!r.geometryAvailable)return PLANCK_UNSUPPORTED;
    auto* body=FindBody(r,proposal->body);if(!body)return PLANCK_STALE;
    if(!body->state.capsule)return PLANCK_UNSUPPORTED;
    if(body->state.geometryRevision!=proposal->expectedRevision)return PLANCK_STALE;
    if(body->state.geometryRevision==(std::numeric_limits<uint64_t>::max)())return PLANCK_LIMIT;
    for(const auto& [id,res]:r.resources)if(res->kind==Kind::Geometry&&(res->active||res->dispatched||res->nativeCommitted)&&
        res->result.stage==PLANCK_REQUEST_ACCEPTED&&Same(res->geometry.body,proposal->body))return PLANCK_CONFLICT;
    if(r.resources.size()>=MaxResources)return PLANCK_LIMIT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::Geometry;res->owner=owner;res->geometry=*proposal;
    res->result={sizeof(PlanckRequestResult),PLANCK_REQUEST_ACCEPTED,id,0,r.epoch,PLANCK_OK,0,body->state.geometryRevision};
    r.resources.emplace(id,res);*output=id;return PLANCK_OK;
}
PlanckStatus GetResult(PlanckToken owner,PlanckToken id,PlanckRequestResult* output) {
    if(!Sized(output))return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner,true);if(status)return status;
    auto res=Owned(r,owner,id);if(!res||res->kind!=Kind::Geometry)return PLANCK_INVALID;
    *output=res->result;return PLANCK_OK;
}
PlanckStatus CopyBodyGeometry(PlanckToken owner,PlanckBodyRef ref,PlanckBodyGeometry* output,
    PlanckShapeChild* children,uint32_t capacity,uint32_t* count){
    if(count)*count=0;
    if(!Valid(ref)||!Sized(output)||!count||(capacity&&!children))return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    auto* body=FindBody(r,ref);if(!body)return PLANCK_STALE;
    if(!body->listIdentity||body->children.empty())return PLANCK_UNSUPPORTED;
    *count=static_cast<uint32_t>(body->children.size());
    if(capacity<*count)return PLANCK_BUFFER;
    for(uint32_t i=0;i<*count;++i)if(!Sized(&children[i]))return PLANCK_INVALID;
    *output={sizeof(*output),PLANCK_SHAPE_LIST,ref,body->state.sample,body->state.geometryRevision,*count,0};
    for(uint32_t i=0;i<*count;++i)children[i]=body->children[i].state;
    return PLANCK_OK;
}
PlanckStatus RequestCompoundGeometry(PlanckToken owner,const PlanckCompoundProposal* proposal,
    const PlanckCapsuleEdit* edits,PlanckToken* output){
    if(output)*output=0;
    if(!output||!Sized(proposal)||!Valid(proposal->body)||!proposal->expectedRevision||proposal->reserved||
        !proposal->editCount||proposal->editCount>256||!edits)return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    std::vector<PlanckCapsuleEdit> copied;copied.reserve(proposal->editCount);
    for(uint32_t i=0;i<proposal->editCount;++i){
        const auto& edit=edits[i];
        if(!Sized(&edit)||edit.reserved||!LocalPoint(edit.capsuleA)||!LocalPoint(edit.capsuleB)||
            !std::isfinite(edit.radius)||edit.radius<=0.f||edit.radius>10.f)return PLANCK_INVALID;
        const double dx=double(edit.capsuleA.x)-edit.capsuleB.x,dy=double(edit.capsuleA.y)-edit.capsuleB.y,dz=double(edit.capsuleA.z)-edit.capsuleB.z;
        if(dx*dx+dy*dy+dz*dz<=1e-12||std::any_of(copied.begin(),copied.end(),[&](const auto& row){return row.index==edit.index;}))return PLANCK_INVALID;
        copied.push_back(edit);
    }
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(!r.compoundAvailable)return PLANCK_UNSUPPORTED;
    auto* body=FindBody(r,proposal->body);if(!body)return PLANCK_STALE;
    if(!body->listIdentity||body->children.empty())return PLANCK_UNSUPPORTED;
    if(body->state.geometryRevision!=proposal->expectedRevision)return PLANCK_STALE;
    if(body->state.geometryRevision==(std::numeric_limits<uint64_t>::max)())return PLANCK_LIMIT;
    for(const auto& edit:copied)if(edit.index>=body->children.size()||body->children[edit.index].state.shapeKind!=PLANCK_SHAPE_CAPSULE)return PLANCK_UNSUPPORTED;
    for(const auto& [id,res]:r.resources)if(res->kind==Kind::Geometry&&(res->active||res->dispatched||res->nativeCommitted)&&
        res->result.stage==PLANCK_REQUEST_ACCEPTED&&Same(res->geometry.body,proposal->body))return PLANCK_CONFLICT;
    if(r.resources.size()>=MaxResources)return PLANCK_LIMIT;
    auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::Geometry;res->owner=owner;res->compound=true;res->edits=std::move(copied);
    res->geometry.body=proposal->body;res->geometry.expectedRevision=proposal->expectedRevision;
    res->result={sizeof(PlanckRequestResult),PLANCK_REQUEST_ACCEPTED,id,0,r.epoch,PLANCK_OK,0,body->state.geometryRevision};
    r.resources.emplace(id,res);*output=id;return PLANCK_OK;
}
PlanckStatus CancelRequest(PlanckToken owner,PlanckToken id) {
    auto& r=State();PlanckActorRef actor{};bool emit=false;
    {
        std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner,true);if(status)return status;
        auto res=Owned(r,owner,id);if(!res||res->kind!=Kind::Geometry)return PLANCK_INVALID;
        if(res->dispatched||res->nativeCommitted)return PLANCK_BUSY;
        if(res->result.stage==PLANCK_REQUEST_ACCEPTED){
            res->active=false;res->result.stage=PLANCK_REQUEST_CANCELLED;res->result.status=PLANCK_CANCELLED;
            actor=res->geometry.body.actor;emit=true;
        }
    }
    if(emit)Emit(PLANCK_REQUEST_RESULT,id,actor,PLANCK_PHASE_LIFECYCLE);
    return PLANCK_OK;
}
PlanckStatus RequestDrivePolicy(PlanckToken owner,const PlanckDrivePolicy* proposal,
    const PlanckDriveWeight* weights,PlanckToken* output){
    if(output)*output=0;
    if(!Sized(proposal)||!output||!Valid(proposal->actor)||proposal->reserved||
        (proposal->flags&~PLANCK_DRIVE_BLOCK_PIVOT_LOOSENING)||(proposal->gainMask&~7u)||
        proposal->weightCount>MaxBodies||(proposal->weightCount&&!weights)||
        !(proposal->flags||proposal->gainMask||proposal->weightCount))return PLANCK_INVALID;
    const float gains[]{proposal->hierarchyGain,proposal->velocityGain,proposal->positionGain};
    for(unsigned i=0;i<3;++i)if((proposal->gainMask&(1u<<i))?!Scale(gains[i]):gains[i]!=0.f)return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    std::vector<PlanckDriveWeight> copied;
    for(uint32_t i=0;i<proposal->weightCount;++i){
        const auto& w=weights[i];
        if(!Sized(&w)||w.reserved||w.reserved2||!Valid(w.body)||!Same(w.body.actor,proposal->actor)||
            !std::isfinite(w.multiplier)||w.multiplier<0.f||w.multiplier>8.f)return PLANCK_INVALID;
        for(const auto& before:copied)if(Same(before.body,w.body))return PLANCK_INVALID;
        copied.push_back(w);
    }
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(!CurrentActor(r,proposal->actor))return PLANCK_STALE;
    for(const auto& w:copied)if(!FindBody(r,w.body))return PLANCK_STALE;
    if(proposal->gainMask||proposal->weightCount)
        for(const auto& [id,res]:r.resources)if(res->kind==Kind::DrivePolicy&&(res->active||res->inFlight)&&
            Same(res->drivePolicy.actor,proposal->actor)&&(res->drivePolicy.gainMask||res->drivePolicy.weightCount))return PLANCK_CONFLICT;
    if(r.resources.size()>=MaxResources)return PLANCK_LIMIT;auto id=r.Next();if(!id)return PLANCK_LIMIT;
    auto res=std::make_shared<Resource>();res->kind=Kind::DrivePolicy;res->owner=owner;
    res->drivePolicy=*proposal;res->driveWeights=std::move(copied);r.resources.emplace(id,res);*output=id;return PLANCK_OK;
}
PlanckStatus CopyDrive(PlanckToken owner,uint32_t actor,PlanckDriveSnapshot* output,
    PlanckDriveBody* bodies,uint32_t capacity,uint32_t* required){
    if(!Sized(output)||!required||(capacity&&!bodies))return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
    if(r.suspended)return PLANCK_STALE;auto it=r.actors.find(actor);
    if(it==r.actors.end()||!it->second.drive.sample)return PLANCK_NOT_FOUND;
    *required=static_cast<uint32_t>(it->second.driveBodies.size());if(capacity<*required)return PLANCK_BUFFER;
    for(uint32_t i=0;i<*required;++i)if(!Sized(&bodies[i]))return PLANCK_INVALID;
    *output=it->second.drive;for(uint32_t i=0;i<*required;++i)bodies[i]=it->second.driveBodies[i];return PLANCK_OK;
}
PlanckStatus CopyDrivePolicy(PlanckToken owner,PlanckToken lease,PlanckDrivePolicyState* output){
    if(!Sized(output))return PLANCK_INVALID;
    auto& r=State();std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner,true);if(status)return status;
    auto res=Owned(r,owner,lease);if(!res||res->kind!=Kind::DrivePolicy)return PLANCK_INVALID;
    const uint32_t phase=!res->active||!CurrentActor(r,res->drivePolicy.actor)?PLANCK_DRIVE_POLICY_STALE:
        res->lastDriveSample?PLANCK_DRIVE_POLICY_OBSERVED:PLANCK_DRIVE_POLICY_ACCEPTED;
    *output={sizeof(PlanckDrivePolicyState),phase,lease,res->drivePolicy.actor,res->lastDriveSample};return PLANCK_OK;
}
PlanckStatus WithEndpoint(PlanckToken owner,PlanckToken lease,PlanckEndpointVisitor visitor,void* context) {
    if(!visitor)return PLANCK_INVALID;
    if(callbackDepth)return PLANCK_REENTRANT;
    auto& r=State();std::shared_ptr<Resource> resource;PlanckEndpoint endpoint{};
    NativeEndpointResolver resolver=nullptr;
    {
        std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
        resource=Owned(r,owner,lease);
        if(!resource||resource->kind!=Kind::Endpoint)return PLANCK_INVALID;
        if(!resource->active||!FindBody(r,resource->endpoint.body))return PLANCK_STALE;
        resolver=r.endpointResolver;if(!resolver)return PLANCK_UNSUPPORTED;
        endpoint=resource->endpoint;++resource->inFlight;
    }
    PlanckStatus status=PLANCK_LIMIT;
    try{status=resolver(owner,lease,endpoint,visitor,context);}catch(...){status=PLANCK_LIMIT;}
    {std::lock_guard guard(r.lock);--resource->inFlight;}
    return status;
}
// All C entry points contain allocation/implementation exceptions. Callback errors
// are isolated separately, outside the registry lock.
template<auto Fn> struct Boundary;
template<class R,class... A,R(*Fn)(A...)> struct Boundary<Fn> {
    static R PLANCK_CALL Call(A... args) noexcept {try{return Fn(args...);}catch(...){return PLANCK_LIMIT;}}
};
PlanckSynergyAPI api={sizeof(PlanckSynergyAPI),2,15 | PLANCK_CAP_SAMPLE_TIME,
    Boundary<RegisterClient>::Call,Boundary<ReleaseClient>::Call,Boundary<Subscribe>::Call,Boundary<Unsubscribe>::Call,
    Boundary<CopyActorPhysics>::Call,Boundary<AcquireEndpoint>::Call,Boundary<ValidateEndpoint>::Call,
    Boundary<ReleaseLease>::Call,Boundary<RequestPolicy>::Call,Boundary<RegisterHitPolicy>::Call,Boundary<CopyHit>::Call,
    Boundary<RequestGeometry>::Call,Boundary<GetResult>::Call,Boundary<CancelRequest>::Call,Boundary<CopyActorSampleTime>::Call};
const PlanckPhysicalInteropAPI physicalAPI={sizeof(PlanckPhysicalInteropAPI),1,Boundary<WithEndpoint>::Call};
const PlanckDriveAPI driveAPI={sizeof(PlanckDriveAPI),1,Boundary<RequestDrivePolicy>::Call,
    Boundary<CopyDrive>::Call,Boundary<CopyDrivePolicy>::Call};
PlanckCompoundGeometryAPI compoundAPI={sizeof(PlanckCompoundGeometryAPI),1,PLANCK_COMPOUND_COPY,
    Boundary<CopyBodyGeometry>::Call,Boundary<RequestCompoundGeometry>::Call};
}
const PlanckSynergyAPI* GetAPI(){return &api;}
const PlanckPhysicalInteropAPI* GetPhysicalInteropAPI(){return &physicalAPI;}
bool HasClients() { return clientsRegistered.load(std::memory_order_acquire); }
uint64_t CaptureHitEpoch() {
    if (!HasClients()) return 0;
    auto& r = State();
    std::lock_guard lock(r.lock);
    return r.epoch;
}
bool IsHitEpochCurrent(uint64_t epoch) {
    auto& r = State();
    std::lock_guard lock(r.lock);
    return !r.suspended && epoch == r.epoch;
}
const PlanckDriveAPI* GetDriveAPI(){return &driveAPI;}
const PlanckCompoundGeometryAPI* GetCompoundGeometryAPI(){return &compoundAPI;}
bool IsDrivePlanCurrent(const DrivePlan& plan){
    auto& r=State();std::lock_guard guard(r.lock);if(!CurrentActor(r,plan.actor))return false;
    for(const auto& expected:plan.bones){auto* body=FindBody(r,expected.body);
        if(!body||body->identity!=expected.identity||body->lineage!=plan.lineage)return false;}
    return !plan.bones.empty();
}
void ExecuteNativeDrive(uintptr_t model,uintptr_t lineage,const uintptr_t* bodies,uint32_t count,
    uint64_t frame,uint64_t tick,NativeDriveAction action,void* context){
    if(!action)return;auto& r=State();std::unique_lock nativeGuard(r.nativeGate);
    DrivePlan plan;DriveExecution execution;std::vector<std::shared_ptr<Resource>> flight;
    {
        std::lock_guard guard(r.lock);
        if(!r.suspended&&model&&lineage&&bodies&&count&&count<=MaxBodies)
            for(auto& [handle,actor]:r.actors){
                if(actor.model!=model)continue;
                std::vector<DriveBonePlan> matched;
                for(uint32_t i=0;i<count;++i){
                    auto found=std::find_if(actor.bodies.begin(),actor.bodies.end(),[&](const auto& body){return body.identity==bodies[i]&&body.lineage==lineage;});
                    if(!bodies[i]||found==actor.bodies.end())break;
                    if(std::any_of(matched.begin(),matched.end(),[&](const auto& previous){return previous.identity==bodies[i];}))break;
                    matched.push_back({bodies[i],found->state.body,1.f});
                }
                if(matched.size()!=count)continue;
                plan.actor=actor.state.actor;plan.lineage=lineage;plan.bones=std::move(matched);break;
            }
        if(plan.actor.handle)for(auto& [id,res]:r.resources)if(res->kind==Kind::DrivePolicy&&res->active&&Same(res->drivePolicy.actor,plan.actor)){
            plan.flags|=res->drivePolicy.flags;
            if(res->drivePolicy.gainMask||res->drivePolicy.weightCount){
                plan.numericalPolicy=id;plan.gainMask=res->drivePolicy.gainMask;
                plan.weightPolicy=res->drivePolicy.weightCount!=0;
                plan.hierarchyGain=res->drivePolicy.hierarchyGain;plan.velocityGain=res->drivePolicy.velocityGain;plan.positionGain=res->drivePolicy.positionGain;
                for(const auto& w:res->driveWeights)for(auto& bone:plan.bones)if(Same(w.body,bone.body))bone.multiplier=w.multiplier;
            }
            ++res->inFlight;flight.push_back(res);
        }
    }
    std::exception_ptr failure;
    try{action(plan,execution,context);}catch(...){execution.status=PLANCK_CANCELLED;failure=std::current_exception();}
    uint64_t sample=0;PlanckActorRef actorRef=plan.actor;
    {
        std::lock_guard guard(r.lock);
        for(auto& resource:flight)--resource->inFlight;
        auto actor=r.actors.find(plan.actor.handle);
        if(plan.actor.handle&&actor!=r.actors.end()&&CurrentActor(r,plan.actor)){
            sample=r.Next();
            if(sample){
                bool coherent=execution.bodies.size()==plan.bones.size();
                if(coherent)for(size_t i=0;i<plan.bones.size();++i)if(!Same(execution.bodies[i].body,plan.bones[i].body)){coherent=false;break;}
                if(!coherent){execution.bodies.clear();if(execution.status==PLANCK_OK)execution.status=PLANCK_INVALID;}
                auto& current=actor->second;
                current.drive={sizeof(PlanckDriveSnapshot),static_cast<uint32_t>(execution.bodies.size()),plan.actor,
                    sample,frame,tick,plan.numericalPolicy,PLANCK_PHASE_DRIVE,
                    uint32_t((plan.flags&PLANCK_DRIVE_BLOCK_PIVOT_LOOSENING)!=0),execution.status,0,
                    execution.hierarchyGain,execution.velocityGain,execution.positionGain,execution.paletteCount};
                current.driveBodies=std::move(execution.bodies);
                for(auto& resource:flight)if(execution.status==PLANCK_OK||
                    (execution.pivotPolicyObserved&&!resource->drivePolicy.gainMask&&!resource->drivePolicy.weightCount))resource->lastDriveSample=sample;
            }
        }
    }
    nativeGuard.unlock();
    if(sample)Emit(PLANCK_DRIVE_OBSERVED,sample,actorRef,PLANCK_PHASE_DRIVE);
    if(failure)std::rethrow_exception(failure);
}
void InstallNativeEndpointResolver(NativeEndpointResolver resolver){auto& r=State();std::lock_guard guard(r.lock);r.endpointResolver=resolver;}
void SetGeometryExecutorAvailable(bool available){
    auto& r=State();std::lock_guard guard(r.lock);r.geometryAvailable=available;
    // Installed during plugin startup before clients discover the table.
    if(available)api.capabilities|=PLANCK_CAP_CAPSULE_GEOMETRY;else api.capabilities&=~uint64_t(PLANCK_CAP_CAPSULE_GEOMETRY);
}
void ProcessActorGeometry(uint32_t handle,uintptr_t model,const std::vector<BodySample>& live,
    NativeGeometryExecutor execute,void* context,DeferredEvents* deferred){
    if(!execute)return;
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);std::vector<PlanckToken> pending;
    {
        std::lock_guard guard(r.lock);
        for(auto& [id,res]:r.resources)if(res->kind==Kind::Geometry&&!res->compound&&(res->active||res->nativeCommitted)&&!res->dispatched&&
            res->result.stage==PLANCK_REQUEST_ACCEPTED&&res->geometry.body.actor.handle==handle)pending.push_back(id);
    }
    DeferredEvents events;
    for(auto id:pending){
        std::shared_ptr<Resource> res;BodySample copied{};PlanckStatus status=PLANCK_OK;
        {
            std::lock_guard guard(r.lock);auto entry=r.resources.find(id);
            if(entry==r.resources.end())continue;res=entry->second;
            if((!res->active&&!res->nativeCommitted)||res->result.stage!=PLANCK_REQUEST_ACCEPTED||res->dispatched)continue;
            auto* body=FindBody(r,res->geometry.body);auto actor=r.actors.find(handle);
            if(!body||actor==r.actors.end()||actor->second.model!=model||res->geometry.body.index>=live.size())status=PLANCK_STALE;
            else{
                copied=live[res->geometry.body.index];
                if(copied.identity!=body->identity||copied.lineage!=body->lineage||
                    (body->state.geometryRevision!=res->geometry.expectedRevision&&
                     !(res->nativeCommitted&&body->state.geometryRevision==res->geometry.expectedRevision+1))||!copied.state.capsule||
                    (!res->nativeCommitted&&(copied.state.capsuleRadius!=body->state.capsuleRadius||
                    std::memcmp(&copied.state.capsuleA,&body->state.capsuleA,sizeof(PlanckVec3))||
                    std::memcmp(&copied.state.capsuleB,&body->state.capsuleB,sizeof(PlanckVec3)))))status=PLANCK_STALE;
            }
            res->dispatched=true;++res->inFlight;
        }
        if(status==PLANCK_OK){try{status=execute(id,copied,res->geometry,context);}catch(...){status=PLANCK_LIMIT;}}
        {
            std::lock_guard guard(r.lock);res->dispatched=false;--res->inFlight;
            if(res->result.stage!=PLANCK_REQUEST_ACCEPTED)continue;
            auto* body=FindBody(r,res->geometry.body);
            if(!body||(body->state.geometryRevision!=res->geometry.expectedRevision&&
                !(res->nativeCommitted&&body->state.geometryRevision==res->geometry.expectedRevision+1)))status=PLANCK_STALE;
            if(status==NativeGeometryPending){res->nativeCommitted=true;continue;}
            if(status==PLANCK_BUSY)continue; // Safe phase unavailable; nothing was applied.
            res->nativeCommitted=false;
            res->active=false;res->result.status=status;
            if(status==PLANCK_OK){
                body->state.capsuleA=res->geometry.capsuleA;body->state.capsuleB=res->geometry.capsuleB;
                body->state.capsuleRadius=res->geometry.radius;
                if(body->state.geometryRevision==res->geometry.expectedRevision)++body->state.geometryRevision;
                auto& actor=r.actors.at(handle);const auto sample=++r.sample;actor.state.sample=sample;
                for(auto& b:actor.bodies)b.state.sample=sample;
                res->result.stage=PLANCK_REQUEST_COMPLETED;res->result.geometryRevision=body->state.geometryRevision;
                events.push_back({PLANCK_ACTOR_CHANGED,0,res->geometry.body.actor,PLANCK_PHASE_MAIN});
            }else res->result.stage=PLANCK_REQUEST_REJECTED;
            events.push_back({PLANCK_REQUEST_RESULT,id,res->geometry.body.actor,PLANCK_PHASE_MAIN});
        }
    }
    if(deferred)deferred->insert(deferred->end(),events.begin(),events.end());else DispatchEvents(events);
}
void SetCompoundExecutorAvailable(bool available){
    auto& r=State();std::lock_guard guard(r.lock);r.compoundAvailable=available;
    compoundAPI.capabilities=PLANCK_COMPOUND_COPY|(available?uint64_t(PLANCK_COMPOUND_APPLY):0);
}
void ProcessActorCompoundGeometry(uint32_t handle,uintptr_t model,const std::vector<BodySample>& live,
    NativeCompoundExecutor execute,void* context,DeferredEvents* deferred){
    if(!execute)return;
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);std::vector<PlanckToken> pending;DeferredEvents events;
    {
        std::lock_guard guard(r.lock);
        for(auto& [id,res]:r.resources)if(res->kind==Kind::Geometry&&res->compound&&(res->active||res->nativeCommitted)&&
            !res->dispatched&&res->result.stage==PLANCK_REQUEST_ACCEPTED&&res->geometry.body.actor.handle==handle)pending.push_back(id);
    }
    for(auto id:pending){
        std::shared_ptr<Resource> res;BodySample copied{},applied{};PlanckStatus status=PLANCK_OK;PlanckCompoundProposal proposal{};
        {
            std::lock_guard guard(r.lock);auto entry=r.resources.find(id);if(entry==r.resources.end())continue;res=entry->second;
            if((!res->active&&!res->nativeCommitted)||res->dispatched||res->result.stage!=PLANCK_REQUEST_ACCEPTED)continue;
            auto* body=FindBody(r,res->geometry.body);auto actor=r.actors.find(handle);
            if(!body||actor==r.actors.end()||actor->second.model!=model||res->geometry.body.index>=live.size())status=PLANCK_STALE;
            else{
                copied=live[res->geometry.body.index];
                if(copied.identity!=body->identity||copied.lineage!=body->lineage||!copied.listIdentity||
                    (body->state.geometryRevision!=res->geometry.expectedRevision&&
                    !(res->nativeCommitted&&body->state.geometryRevision==res->geometry.expectedRevision+1))||
                    (!res->nativeCommitted&&!SameChildren(copied,*body)))status=PLANCK_STALE;
            }
            proposal={sizeof(proposal),static_cast<uint32_t>(res->edits.size()),res->geometry.body,res->geometry.expectedRevision,0};
            res->dispatched=true;++res->inFlight;
        }
        if(status==PLANCK_OK){try{status=execute(id,copied,proposal,res->edits,context,&applied);}catch(...){status=PLANCK_LIMIT;}}
        {
            std::lock_guard guard(r.lock);res->dispatched=false;--res->inFlight;
            if(res->result.stage!=PLANCK_REQUEST_ACCEPTED)continue;
            auto* body=FindBody(r,res->geometry.body);
            if(!body||(body->state.geometryRevision!=res->geometry.expectedRevision&&
                !(res->nativeCommitted&&body->state.geometryRevision==res->geometry.expectedRevision+1)))status=PLANCK_STALE;
            if(status==NativeGeometryPending){res->nativeCommitted=true;continue;}
            if(status==PLANCK_BUSY)continue;
            if(status==PLANCK_OK){
                if(applied.identity!=copied.identity||applied.lineage!=copied.lineage||!applied.listIdentity||
                    applied.children.size()!=copied.children.size()||applied.children.empty()||applied.children.size()>256)status=PLANCK_STALE;
                for(const auto& edit:res->edits)if(status==PLANCK_OK){
                    if(edit.index>=applied.children.size())status=PLANCK_STALE;
                    else{
                        const auto& child=applied.children[edit.index].state;
                        if(child.shapeKind!=PLANCK_SHAPE_CAPSULE||child.radius!=edit.radius||
                            std::memcmp(&child.capsuleA,&edit.capsuleA,sizeof(PlanckVec3))||
                            std::memcmp(&child.capsuleB,&edit.capsuleB,sizeof(PlanckVec3)))status=PLANCK_STALE;
                    }
                }
            }
            res->nativeCommitted=false;res->active=false;res->result.status=status;
            if(status==PLANCK_OK){
                body->listIdentity=applied.listIdentity;body->children=std::move(applied.children);
                if(body->state.geometryRevision==res->geometry.expectedRevision)++body->state.geometryRevision;
                auto& actor=r.actors.at(handle);const auto sample=++r.sample;actor.state.sample=sample;
                for(auto& row:actor.bodies)row.state.sample=sample;
                res->result.stage=PLANCK_REQUEST_COMPLETED;res->result.geometryRevision=body->state.geometryRevision;
                events.push_back({PLANCK_ACTOR_CHANGED,0,res->geometry.body.actor,PLANCK_PHASE_MAIN});
            }else res->result.stage=PLANCK_REQUEST_REJECTED;
            events.push_back({PLANCK_REQUEST_RESULT,id,res->geometry.body.actor,PLANCK_PHASE_MAIN});
        }
    }
    if(deferred)deferred->insert(deferred->end(),events.begin(),events.end());else DispatchEvents(events);
}
PlanckStatus VisitResolvedEndpoint(PlanckToken owner,PlanckToken lease,uintptr_t model,
    uintptr_t identity,uintptr_t lineage,const PlanckNativeEndpoint& native,PlanckEndpointVisitor visitor,void* context){
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);PlanckNativeEndpoint copied=native;
    {
        std::lock_guard guard(r.lock);auto status=ClientStatus(r,owner);if(status)return status;
        auto resource=Owned(r,owner,lease);
        if(!resource||resource->kind!=Kind::Endpoint)return PLANCK_INVALID;
        auto* body=FindBody(r,resource->endpoint.body);
        auto actor=r.actors.find(resource->endpoint.body.actor.handle);
        if(!resource->active||!body||actor==r.actors.end()||actor->second.model!=model||
            body->identity!=identity||body->lineage!=lineage||native.rigidBody!=identity||!native.world)
            return PLANCK_STALE;
        copied.size=sizeof(copied);copied.endpoint=resource->endpoint;
        copied.sample=body->state.sample;copied.geometryRevision=body->state.geometryRevision;
    }
    CallbackScope callback;
    return visitor(&copied,context);
}

uint64_t MonotonicNowNs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}
void PublishActor(uint32_t handle,uintptr_t model,bool ragdolled,uint64_t frame,uint64_t tick,double time,const std::vector<BodySample>& bodies,DeferredEvents* deferred,uint64_t observedAtNs) {
    if(!handle)return;
    if(!model||bodies.empty()||bodies.size()>MaxBodies){InvalidateActor(handle,deferred);return;}
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);std::vector<uint64_t> invalidated,cancelled,driveInvalidated;PlanckActorRef old{},current{};bool changed=false;
    {
        std::lock_guard guard(r.lock);
        if(r.clients.empty()||r.suspended||r.exhausted)return;
        if(!r.actors.contains(handle)&&r.actors.size()>=MaxActors)return;
        auto it=r.actors.find(handle);
        bool replaced=it==r.actors.end()||it->second.model!=model||it->second.bodies.size()!=bodies.size();
        if(!replaced)for(size_t i=0;i<bodies.size();++i)if(it->second.bodies[i].identity!=bodies[i].identity||it->second.bodies[i].lineage!=bodies[i].lineage){replaced=true;break;}
        ActorState next;next.model=model;next.bodies=bodies;
        const auto sample=++r.sample;
        if(replaced) {
            changed=true;
            if(it!=r.actors.end()){old=it->second.state.actor;RetireActorLocked(r,old,invalidated,cancelled);RetireDrivePoliciesLocked(r,old,driveInvalidated);}
            const auto generation=r.Next();if(!generation)return;
            current={r.epoch,generation,handle,0};
            for(size_t i=0;i<bodies.size();++i) {
                auto generationBody=r.Next();if(!generationBody)return;
                next.bodies[i].state.body={current,generationBody,static_cast<uint32_t>(i),0};
                next.bodies[i].state.geometryRevision=1;
            }
        }else{
            current=it->second.state.actor;changed=it->second.state.ragdolled!=static_cast<uint32_t>(ragdolled);
            next.drive=it->second.drive;next.driveBodies=it->second.driveBodies;
            for(size_t i=0;i<bodies.size();++i){
                next.bodies[i].state.body=it->second.bodies[i].state.body;
                auto& before=it->second.bodies[i].state;auto& after=next.bodies[i].state;
                const bool geometryChanged=!SameChildren(it->second.bodies[i],next.bodies[i])||before.capsule!=after.capsule||before.capsuleRadius!=after.capsuleRadius||
                    std::memcmp(&before.capsuleA,&after.capsuleA,sizeof(PlanckVec3))||std::memcmp(&before.capsuleB,&after.capsuleB,sizeof(PlanckVec3));
                changed|=geometryChanged;
                after.geometryRevision=before.geometryRevision+(geometryChanged?1:0);
            }
        }
        for(auto& b:next.bodies){b.state.size=sizeof(b.state);b.state.sample=sample;b.state.boneName[95]=0;}
        next.state={sizeof(PlanckActorPhysics),static_cast<uint32_t>(bodies.size()),current,sample,frame,tick,time,PLANCK_PHASE_MAIN,static_cast<uint32_t>(ragdolled),observedAtNs};
        r.actors[handle]=std::move(next);
    }
    DeferredEvents events;
    for(auto id:invalidated)events.push_back({PLANCK_ENDPOINT_INVALIDATED,id,old,PLANCK_PHASE_MAIN});
    for(auto id:cancelled)events.push_back({PLANCK_REQUEST_RESULT,id,old,PLANCK_PHASE_MAIN});
    for(auto id:driveInvalidated)events.push_back({PLANCK_DRIVE_POLICY_INVALIDATED,id,old,PLANCK_PHASE_MAIN});
    if(changed)events.push_back({PLANCK_ACTOR_CHANGED,0,current,PLANCK_PHASE_MAIN});
    if(deferred)deferred->insert(deferred->end(),events.begin(),events.end());else DispatchEvents(events);
}
void DispatchEvents(const DeferredEvents& events){for(const auto& event:events)Emit(event.type,event.subject,event.actor,event.phase);}
void InvalidateActor(uint32_t handle,DeferredEvents* deferred){
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);PlanckActorRef old{};std::vector<uint64_t> endpoints,cancelled,driveInvalidated;
    {std::lock_guard guard(r.lock);auto it=r.actors.find(handle);if(it==r.actors.end())return;old=it->second.state.actor;RetireActorLocked(r,old,endpoints,cancelled);RetireDrivePoliciesLocked(r,old,driveInvalidated);r.actors.erase(it);}
    DeferredEvents events;
    for(auto id:endpoints)events.push_back({PLANCK_ENDPOINT_INVALIDATED,id,old,PLANCK_PHASE_LIFECYCLE});
    for(auto id:cancelled)events.push_back({PLANCK_REQUEST_RESULT,id,old,PLANCK_PHASE_LIFECYCLE});
    for(auto id:driveInvalidated)events.push_back({PLANCK_DRIVE_POLICY_INVALIDATED,id,old,PLANCK_PHASE_LIFECYCLE});
    events.push_back({PLANCK_ACTOR_CHANGED,0,old,PLANCK_PHASE_LIFECYCLE});
    if(deferred)deferred->insert(deferred->end(),events.begin(),events.end());else DispatchEvents(events);
}
void InvalidateBody(uintptr_t identity){
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);std::vector<uint32_t> handles;
    {std::lock_guard guard(r.lock);for(auto& [handle,actor]:r.actors)for(auto& body:actor.bodies)if(body.identity==identity){handles.push_back(handle);break;}}
    for(auto handle:handles)InvalidateActor(handle);
}
void Reset(){
    auto& r=State();std::lock_guard nativeGuard(r.nativeGate);std::vector<std::pair<uint64_t,PlanckActorRef>> endpoints,cancelled,driveInvalidated;
    {std::lock_guard guard(r.lock);
        if(r.epoch==(std::numeric_limits<uint64_t>::max)())r.exhausted=true;else ++r.epoch;
        for(auto& [id,res]:r.resources){
            if(res->kind==Kind::Endpoint&&res->active)endpoints.emplace_back(id,res->endpoint.body.actor);
            if(res->kind==Kind::Endpoint||res->kind==Kind::ActorPolicy)res->active=false;
            if(res->kind==Kind::DrivePolicy&&res->active){res->active=false;driveInvalidated.emplace_back(id,res->drivePolicy.actor);}
            if(res->kind==Kind::Geometry&&res->result.stage==PLANCK_REQUEST_ACCEPTED){
                res->active=false;res->nativeCommitted=false;res->result.stage=PLANCK_REQUEST_CANCELLED;res->result.status=PLANCK_STALE;
                cancelled.emplace_back(id,res->geometry.body.actor);
            }
        }
        r.actors.clear();r.hits.clear();r.hitOrder.clear();
    }
    for(auto [id,actor]:endpoints)Emit(PLANCK_ENDPOINT_INVALIDATED,id,actor,PLANCK_PHASE_LIFECYCLE);
    for(auto [id,actor]:cancelled)Emit(PLANCK_REQUEST_RESULT,id,actor,PLANCK_PHASE_LIFECYCLE);
    for(auto [id,actor]:driveInvalidated)Emit(PLANCK_DRIVE_POLICY_INVALIDATED,id,actor,PLANCK_PHASE_LIFECYCLE);
    Emit(PLANCK_RESET,0,{},PLANCK_PHASE_LIFECYCLE);
}
void SetSuspended(bool suspended){
    {auto& r=State();std::lock_guard guard(r.lock);r.suspended=suspended;}
    if(suspended)Reset();
}
PlanckHitSnapshot BeginHit(uint32_t kind,uint32_t target,uint32_t source,PlanckVec3 position,PlanckVec3 velocity,float damage,uint32_t hand,uint64_t frame,uint64_t tick){
    PlanckHitSnapshot hit{sizeof(PlanckHitSnapshot),kind,0,0,frame,tick,{},source,PLANCK_HIT_CANCELLED,position,velocity,damage,1.f,hand,PLANCK_PHASE_HIT,0};
    if (!HasClients()) {
        hit.outcome = PLANCK_HIT_PENDING; // No policy or receipt requested.
        return hit;
    }
    if(callbackDepth||!target||!(kind==PLANCK_HIT_MELEE||kind==PLANCK_HIT_CLUTTER))return hit;
    auto& r=State();std::vector<PlanckToken> policies;bool denied=false;
    {
        std::lock_guard guard(r.lock);if(r.suspended)return hit;hit.hitId=r.Next();if(!hit.hitId)return hit;hit.threadId=ThreadToken(r);hit.epoch=r.epoch;hit.target={r.epoch,0,target,0};
        auto actor=r.actors.find(target);if(actor!=r.actors.end())hit.target=actor->second.state.actor;
        hit.outcome=PLANCK_HIT_PENDING;
        for(auto& [id,res]:r.resources)if(res->active){
            if(res->kind==Kind::ActorPolicy&&(res->actorPolicy.hitKinds&kind)&&Same(res->actorPolicy.actor,hit.target)){
                denied|=(res->actorPolicy.flags&PLANCK_POLICY_SUPPRESS_HITS)!=0;
                if(kind==PLANCK_HIT_CLUTTER)hit.damageScale=(std::min)(hit.damageScale,res->actorPolicy.damageScale);
            }
            if(res->kind==Kind::HitPolicy&&(res->hitPolicy.hitKinds&kind)&&(!res->hitPolicy.actor.handle||Same(res->hitPolicy.actor,hit.target)))policies.push_back(id);
        }
        // Do not evict an in-flight commitment; nested engine hits are bounded.
        if(r.hits.size()>=MaxHits){
            auto victim=std::find_if(r.hitOrder.begin(),r.hitOrder.end(),[&](auto id){return r.hits.at(id).outcome!=PLANCK_HIT_PENDING;});
            if (victim == r.hitOrder.end()) {
                hit.hitId = 0; // Observational history is full. Still evaluate policies.
            } else {
                r.hits.erase(*victim);
                r.hitOrder.erase(victim);
            }
        }
        if (hit.hitId) {
            r.hits.emplace(hit.hitId,hit);
            r.hitOrder.push_back(hit.hitId);
        }
    }
    if (hit.hitId) Emit(PLANCK_HIT_CANDIDATE,hit.hitId,hit.target,PLANCK_PHASE_HIT);
    const auto start=std::chrono::steady_clock::now();
    for(auto id:policies){
        if(std::chrono::steady_clock::now()-start>std::chrono::milliseconds(2)){denied=true;break;}
        std::shared_ptr<Resource> res;
        {std::lock_guard guard(r.lock);auto it=r.resources.find(id);if(it==r.resources.end()||!it->second->active||r.epoch!=hit.epoch)continue;res=it->second;++res->inFlight;}
        PlanckHitDecision decision{sizeof(decision),0,1.f,0};bool failed=false;
        const auto before=std::chrono::steady_clock::now();
        {CallbackScope scope;auto copy=hit;try{res->hit(&copy,&decision,res->context);}catch(...){failed=true;}}
        failed|=std::chrono::steady_clock::now()-before>std::chrono::milliseconds(1);
        failed|=decision.size!=sizeof(decision)||decision.deny>1||decision.reserved||!Scale(decision.damageScale);
        {std::lock_guard guard(r.lock);--res->inFlight;if(failed)res->active=false;}
        denied|=failed||decision.deny;
        if(!failed&&kind==PLANCK_HIT_CLUTTER)hit.damageScale=(std::min)(hit.damageScale,decision.damageScale);
    }
    bool terminal=false;
    {
        std::lock_guard guard(r.lock);
        if(r.epoch!=hit.epoch||(hit.target.actorGeneration&&!CurrentActor(r,hit.target))||(hit.hitId&&!r.hits.contains(hit.hitId))){
            hit.outcome=PLANCK_HIT_CANCELLED;
            // Target removal can occur during a policy callback. Retire the
            // stored pending snapshot too, even when the caller returns early.
            // Reset may already have evicted it; never recreate that old epoch.
            auto stored=r.hits.find(hit.hitId);
            if(stored!=r.hits.end()&&stored->second.outcome==PLANCK_HIT_PENDING){stored->second=hit;terminal=true;}
        } else {
            // Pending until FinishHit: observers cannot turn this into a commitment.
            hit.outcome=denied?PLANCK_HIT_DENIED:PLANCK_HIT_PENDING;
            if (hit.hitId) {
                r.hits[hit.hitId]=hit;
                terminal=denied;
            }
        }
    }
    if(terminal)Emit(PLANCK_HIT_COMMITTED,hit.hitId,hit.target,PLANCK_PHASE_HIT);
    return hit;
}
void FinishHit(uint64_t id,bool committed){
    if(!id)return;auto& r=State();PlanckActorRef actor{};
    {std::lock_guard guard(r.lock);auto it=r.hits.find(id);if(it==r.hits.end()||it->second.outcome!=PLANCK_HIT_PENDING)return;
        it->second.outcome=committed?PLANCK_HIT_ALLOWED:PLANCK_HIT_CANCELLED;actor=it->second.target;}
    Emit(PLANCK_HIT_COMMITTED,id,actor,PLANCK_PHASE_HIT);
}
}
