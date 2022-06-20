#include "planckinterface001.h"

// A message used to fetch PLANCK's interface
struct PlanckMessage {
	enum { kMessage_GetInterface = 0x92F38745 }; // Randomly generated
	void * (*GetApiFunction)(unsigned int revisionNumber) = nullptr;
};

// Stores the API after it has already been fetched
static PlanckPluginAPI::IPlanckInterface001 * g_planckInterface = nullptr;

// Fetches the interface to use from PLANCK
PlanckPluginAPI::IPlanckInterface001 * PlanckPluginAPI::GetPlanckInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface)
{
	// If the interface has already been fetched, rturn the same object
	if (g_planckInterface) {
		return g_planckInterface;
	}

	// Dispatch a message to get the plugin interface from PLANCK
	PlanckMessage planckMessage;
	messagingInterface->Dispatch(pluginHandle, PlanckMessage::kMessage_GetInterface, (void*)&planckMessage, sizeof(PlanckMessage*), "PLANCK");
	if (!planckMessage.GetApiFunction) {
		return nullptr;
	}

	// Fetch the API for this version of the PLANCK interface
	g_planckInterface = static_cast<IPlanckInterface001*>(planckMessage.GetApiFunction(1));
	return g_planckInterface;
}
