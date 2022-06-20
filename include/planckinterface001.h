#pragma once
#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

namespace PlanckPluginAPI {

	// Returns an IPlanckInterface001 object compatible with the API shown below
	// This should only be called after SKSE sends kMessage_PostLoad to your plugin
	struct IPlanckInterface001 * GetPlanckInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface);

	// This object provides access to PLANCK's mod support API
	struct IPlanckInterface001
	{
		// Gets the PLANCK build number
		virtual unsigned int GetBuildNumber() = 0;

		virtual bool GetSettingDouble(const std::string_view &name, double &out) = 0;
		virtual bool SetSettingDouble(const std::string &name, double val) = 0;

		virtual void AddIgnoredActor(Actor *actor) = 0;
		virtual void RemoveIgnoredActor(Actor *actor) = 0;
	};
}

extern PlanckPluginAPI::IPlanckInterface001 * g_planckInterface;
