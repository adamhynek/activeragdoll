#include "papyrusapi.h"
#include "pluginapi.h"

namespace PapyrusAPI
{
	float PapyrusGetSetting(StaticFunctionTag *base, BSFixedString name) {
		double out;
		bool success = g_interface001.GetSettingDouble(name.c_str(), out);
		return success ? out : -2.71828f;
	}

	bool PapyrusSetSetting(StaticFunctionTag *base, BSFixedString name, float val) {
		return g_interface001.SetSettingDouble(name.c_str(), val);
	}

	void PapyrusAddIgnoredActor(StaticFunctionTag *base, Actor *actor) {
		g_interface001.AddIgnoredActor(actor);
	}

	void PapyrusRemoveIgnoredActor(StaticFunctionTag *base, Actor *actor) {
		g_interface001.RemoveIgnoredActor(actor);
	}

	void PapyrusAddAggressionIgnoredActor(StaticFunctionTag *base, Actor *actor) {
		g_interface001.AddAggressionIgnoredActor(actor);
	}

	void PapyrusRemoveAggressionIgnoredActor(StaticFunctionTag *base, Actor *actor) {
		g_interface001.RemoveAggressionIgnoredActor(actor);
	}

	void PapyrusSetAggressionLowTopic(StaticFunctionTag *base, Actor *actor, TESTopic *topic) {
		g_interface001.SetAggressionLowTopic(actor, topic);
	}

	void PapyrusSetAggressionHighTopic(StaticFunctionTag *base, Actor *actor, TESTopic *topic) {
		g_interface001.SetAggressionHighTopic(actor, topic);
	}

	void PapyrusAddRagdollCollisionIgnoredActor(StaticFunctionTag *base, Actor *actor) {
		g_interface001.AddRagdollCollisionIgnoredActor(actor);
	}

	void PapyrusRemoveRagdollCollisionIgnoredActor(StaticFunctionTag *base, Actor *actor) {
		g_interface001.RemoveRagdollCollisionIgnoredActor(actor);
	}

	BSFixedString PapyrusGetLastHitNodeName(StaticFunctionTag *base) {
		return g_interface001.GetLastHitData().nodeName;
	}

	bool RegisterPapyrusFuncs(VMClassRegistry* registry) {
		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, float, BSFixedString>("GetSetting", "PLANCK", PapyrusGetSetting, registry));
		registry->RegisterFunction(new NativeFunction2<StaticFunctionTag, bool, BSFixedString, float>("SetSetting", "PLANCK", PapyrusSetSetting, registry));

		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, void, Actor *>("AddIgnoredActor", "PLANCK", PapyrusAddIgnoredActor, registry));
		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, void, Actor *>("RemoveIgnoredActor", "PLANCK", PapyrusRemoveIgnoredActor, registry));

		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, void, Actor *>("AddAggressionIgnoredActor", "PLANCK", PapyrusAddAggressionIgnoredActor, registry));
		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, void, Actor *>("RemoveAggressionIgnoredActor", "PLANCK", PapyrusRemoveAggressionIgnoredActor, registry));

		registry->RegisterFunction(new NativeFunction2<StaticFunctionTag, void, Actor *, TESTopic *>("SetAggressionLowTopic", "PLANCK", PapyrusSetAggressionLowTopic, registry));
		registry->RegisterFunction(new NativeFunction2<StaticFunctionTag, void, Actor *, TESTopic *>("SetAggressionHighTopic", "PLANCK", PapyrusSetAggressionHighTopic, registry));

		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, void, Actor *>("AddRagdollCollisionIgnoredActor", "PLANCK", PapyrusAddRagdollCollisionIgnoredActor, registry));
		registry->RegisterFunction(new NativeFunction1<StaticFunctionTag, void, Actor *>("RemoveRagdollCollisionIgnoredActor", "PLANCK", PapyrusRemoveRagdollCollisionIgnoredActor, registry));

		registry->RegisterFunction(new NativeFunction0<StaticFunctionTag, BSFixedString>("GetLastHitNodeName", "PLANCK", PapyrusGetLastHitNodeName, registry));

		return true;
	}
}
