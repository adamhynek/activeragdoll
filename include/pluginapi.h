#pragma once

#include <atomic>
#include <mutex>
#include <unordered_set>

#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

#include "planckinterface001.h"


namespace PlanckPluginAPI {
	// Handles skse mod messages requesting to fetch API functions from PLANCK
	void ModMessageHandler(SKSEMessagingInterface::Message * message);

	// This object provides access to PLANCK's mod support API version 1
	struct PlanckInterface001 : IPlanckInterface001
	{
		virtual unsigned int GetBuildNumber();

		virtual bool GetSettingDouble(const std::string_view &name, double &out);
		virtual bool SetSettingDouble(const std::string &name, double val);

		virtual void AddIgnoredActor(Actor *actor);
		virtual void RemoveIgnoredActor(Actor *actor);

		std::mutex ignoredActorsLock;
		std::unordered_set<Actor *> ignoredActors;
	};
}

extern PlanckPluginAPI::PlanckInterface001 g_interface001;
