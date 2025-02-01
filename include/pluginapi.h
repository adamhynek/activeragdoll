#pragma once

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

#include "planckinterface001.h"


namespace PlanckPluginAPI {
    // Handles skse mod messages requesting to fetch API functions from PLANCK
    void ModMessageHandler(SKSEMessagingInterface::Message *message);

    // This object provides access to PLANCK's mod support API version 1
    struct PlanckInterface001 : IPlanckInterface001
    {
        virtual unsigned int GetBuildNumber();

        virtual bool Deprecated1(const std::string_view & name, double &out);
        virtual bool Deprecated2(const std::string & name, double val);

        virtual bool GetSettingDouble(const char *name, double &out);
        virtual bool SetSettingDouble(const char *name, double val);

        virtual void AddIgnoredActor(Actor *actor);
        virtual void RemoveIgnoredActor(Actor *actor);

        virtual void AddAggressionIgnoredActor(Actor *actor);
        virtual void RemoveAggressionIgnoredActor(Actor *actor);

        virtual void SetAggressionLowTopic(Actor *actor, TESTopic *topic);
        virtual void SetAggressionHighTopic(Actor *actor, TESTopic *topic);

        virtual void AddRagdollCollisionIgnoredActor(Actor *actor);
        virtual void RemoveRagdollCollisionIgnoredActor(Actor *actor);

        virtual PlanckHitData GetLastHitData();
        virtual TESHitEvent *GetCurrentHitEvent();

        bool IsRagdollCollisionIgnored(TESObjectREFR *actor);

        PlanckHitData lastHitData;
        TESHitEvent *currentHitEvent = nullptr;

        std::mutex ignoredActorsLock;
        std::unordered_set<Actor *> ignoredActors;

        std::mutex aggressionIgnoredActorsLock;
        std::unordered_set<Actor *> aggressionIgnoredActors;

        std::mutex aggressionTopicsLock;
        std::unordered_map<Actor *, TESTopic *> lowAggressionTopics;
        std::unordered_map<Actor *, TESTopic *> highAggressionTopics;

        std::mutex ragdollCollisionIgnoredActorsLock;
        std::unordered_set<TESObjectREFR *> ragdollCollisionIgnoredActors;
    };
}

extern PlanckPluginAPI::PlanckInterface001 g_interface001;
