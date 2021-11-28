#include "higgsinterface001.h"

// A message used to fetch HIGGS's interface
struct HiggsMessage {
	enum { kMessage_GetInterface = 0xF9279A57 }; // Randomly generated
	void * (*GetApiFunction)(unsigned int revisionNumber) = nullptr;
};

// Stores the API after it has already been fetched
HiggsPluginAPI::IHiggsInterface001 * g_higgsInterface = nullptr;

// Fetches the interface to use from HIGGS
HiggsPluginAPI::IHiggsInterface001 * HiggsPluginAPI::GetHiggsInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface)
{
	// If the interface has already been fetched, rturn the same object
	if (g_higgsInterface) {
		return g_higgsInterface;
	}

	// Dispatch a message to get the plugin interface from HIGGS
	HiggsMessage higgsMessage;
	messagingInterface->Dispatch(pluginHandle, HiggsMessage::kMessage_GetInterface, (void*)&higgsMessage, sizeof(HiggsMessage*), "HIGGS");
	if (!higgsMessage.GetApiFunction) {
		return nullptr;
	}

	// Fetch the API for this version of the HIGGS interface
	g_higgsInterface = static_cast<IHiggsInterface001*>(higgsMessage.GetApiFunction(1));
	return g_higgsInterface;
}
