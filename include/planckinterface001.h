#pragma once
#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

namespace PlanckPluginAPI {

	// Returns an IPlanckInterface001 object compatible with the API shown below
	// This should only be called after SKSE sends kMessage_PostPostLoad to your plugin
	struct IPlanckInterface001 * GetPlanckInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface);

	// This object provides access to PLANCK's mod support API
	struct IPlanckInterface001
	{
		// Gets the PLANCK build number
		virtual unsigned int GetBuildNumber() = 0;

		// Read or modify any of planck's ini settings.
		// Only some settings will have an effect if modified, depending on if they are read at startup, when loading / switching cells, or at the time that they are actually required.
		virtual bool GetSettingDouble(const std::string_view &name, double &out) = 0;
		virtual bool SetSettingDouble(const std::string &name, double val) = 0;

		// These actors will not be physically animated, and pretty much entirely excluded from planck's interaction and hit detection.
		// They will behave the same way as actors that do not have a ragdoll (such as wisps), and will revert to registering hits when their character controller is hit instead of their ragdoll.
		virtual void AddIgnoredActor(Actor *actor) = 0;
		virtual void RemoveIgnoredActor(Actor *actor) = 0;

		// These actors will be ignored by planck's aggression system.
		virtual void AddAggressionIgnoredActor(Actor *actor) = 0;
		virtual void RemoveAggressionIgnoredActor(Actor *actor) = 0;

		// Set the topic used by the given npc when interacted with for less than aggressionRequiredGrabTimeHigh seconds by planck's aggression system.
		// Pass a null actor to replace the default. Pass a null topic to revert a previously set topic.
		virtual void SetAggressionLowTopic(Actor *actor, TESTopic *topic) = 0;
		// Set the topic used by the given npc when interacted with for more than aggressionRequiredGrabTimeHigh seconds by planck's aggression system.
		// Pass a null actor to replace the default. Pass a null topic to revert a previously set topic.
		virtual void SetAggressionHighTopic(Actor *actor, TESTopic *topic) = 0;
	};
}

extern PlanckPluginAPI::IPlanckInterface001 * g_planckInterface;
