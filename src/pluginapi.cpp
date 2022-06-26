#include <vector>

#include "config.h"
#include "pluginapi.h"
#include "version.h"

using namespace PlanckPluginAPI;

// A message used to fetch PLANCK's interface
struct PlanckMessage {
	enum { kMessage_GetInterface = 0x92F38745 }; // Randomly generated
	void * (*GetApiFunction)(unsigned int revisionNumber) = nullptr;
};

// Interface classes are stored statically
PlanckInterface001 g_interface001;

// Constructs and returns an API of the revision number requested
void * GetApi(unsigned int revisionNumber) {
	switch (revisionNumber) {
	case 1:	_MESSAGE("Interface revision 1 requested"); return &g_interface001;
	}
	return nullptr;
}

// Handles skse mod messages requesting to fetch API functions from PLANCK
void PlanckPluginAPI::ModMessageHandler(SKSEMessagingInterface::Message * message) {
	if (message->type == PlanckMessage::kMessage_GetInterface) {
		PlanckMessage * planckMessage = (PlanckMessage*)message->data;
		planckMessage->GetApiFunction = GetApi;
		_MESSAGE("Provided PLANCK plugin interface to \"%s\"", message->sender);
	}
}

// PLANCK build numbers are made up as follows: V01.00.05.00
constexpr int planckBuildNumber = ACTIVERAGDOLL_VERSION_MAJOR * 1000000 + ACTIVERAGDOLL_VERSION_MINOR * 10000 + ACTIVERAGDOLL_VERSION_PATCH * 100 + ACTIVERAGDOLL_VERSION_BETA;

// Fetches the PLANCK version number
unsigned int PlanckInterface001::GetBuildNumber() {
	return planckBuildNumber;
}

bool PlanckInterface001::GetSettingDouble(const std::string_view &name, double &out) {
	return Config::GetSettingDouble(name, out);
}

bool PlanckInterface001::SetSettingDouble(const std::string &name, double val) {
	return Config::SetSettingDouble(name, val);
}

void PlanckInterface001::AddIgnoredActor(Actor *actor) {
	std::scoped_lock lock(ignoredActorsLock);
	ignoredActors.insert(actor);
}

void PlanckInterface001::RemoveIgnoredActor(Actor *actor) {
	std::scoped_lock lock(ignoredActorsLock);
	ignoredActors.erase(actor);
}

void PlanckInterface001::AddAggressionIgnoredActor(Actor *actor) {
	std::scoped_lock lock(aggressionIgnoredActorsLock);
	aggressionIgnoredActors.insert(actor);
}

void PlanckInterface001::RemoveAggressionIgnoredActor(Actor *actor) {
	std::scoped_lock lock(aggressionIgnoredActorsLock);
	aggressionIgnoredActors.erase(actor);
}

void PlanckInterface001::SetAggressionLowTopic(Actor *actor, TESTopic *topic) {
	std::scoped_lock lock(aggressionTopicsLock);
	if (topic) {
		lowAggressionTopics[actor] = topic;
	}
	else {
		lowAggressionTopics.erase(actor);
	}
}

void PlanckInterface001::SetAggressionHighTopic(Actor *actor, TESTopic *topic) {
	std::scoped_lock lock(aggressionTopicsLock);
	if (topic) {
		highAggressionTopics[actor] = topic;
	}
	else {
		highAggressionTopics.erase(actor);
	}
}

PlanckHitData PlanckInterface001::GetLastHitData()
{
	return lastHitData; // deliberate copy
}
