# PLANCK ([Nexus link](https://www.nexusmods.com/skyrimspecialedition/mods/66025))

This mod is also complicated.

To build from source, it requires object definitions / headers similar to those belonging to havok 2010.2. I will not be providing these, nor do I claim to have them myself.


PLANCK has a C++ API that can be used by other mods.\
To use it, copy `src/planckinterface001.cpp` and `include/planckinterface001.h` into your project.\
Then, do something like this in your SKSE plugin in `PostPostLoad` or later (this is important - if you try and get the interface before `PostPostLoad`, such as in `PostLoad`, it will not work).

```cpp
#include "planckinterface001.h"

...

void OnSKSEMessage(SKSEMessagingInterface::Message* msg)
{
  if (msg) {
    if (msg->type == SKSEMessagingInterface::kMessage_PostPostLoad) {
      // Get the PLANCK plugin API
      PlanckPluginAPI::GetPlanckInterface001(g_pluginHandle, g_messaging);
      if (g_planckInterface) {
        _MESSAGE("Got planck interface!");
        unsigned int planckVersion = g_planckInterface->GetBuildNumber();
      }
    }
  }
}
```
To receive hit events with extended info provided by PLANCK, you do not need to fetch the interface (though there is no harm in it).\
Planck hit events are simply extensions to regular hit events. When receiving a hit event within a regular hit event handler, check if it's a hit from planck and then simply treat it as a planck hit event, like so.
```cpp
class HitEventHandler : public BSTEventSink <TESHitEvent>
{
public:
  virtual EventResult ReceiveEvent(TESHitEvent *evn, EventDispatcher<TESHitEvent> *dispatcher)
  {
    PlayerCharacter *player = *g_thePlayer;
    if (evn->caster == player) {
      if (PlanckPluginAPI::IsPlanckHit(evn)) {
        PlanckPluginAPI::PlanckHitEvent *extendedEvent = (PlanckPluginAPI::PlanckHitEvent *)evn;
        _MESSAGE("%s", extendedEvent->extendedHitData.nodeName);
      }
    }
    return kEvent_Continue;
  }
};
```

### Optional read of a physical bone

A VR mod placing a pocket at an NPC's pelvis can ask PLANCK where that physical bone is. Request `PLANCK_QUERY_ACTOR_POSE` through the existing SKSE message at PostPostLoad and check the returned table's size and version (2). Call from an SKSE main-thread task before taking any engine locks. [C interface](include/planck_actor_pose.h)

```cpp
PlanckBodyPose pose{sizeof(pose)};
if (api->CopyBodyPose(actorHandle, "NPC Pelvis [Pelv]", &pose) == PLANCK_POSE_OK) {
    // Transform the pocket using pose.positionMetres and pose.rotationXyzw.
}
```

Use `observedAtNs` to compare the pose with the hand sample. End an ongoing interaction on failure or a changed `generation`, including removal and re-addition of the same body. A corpse can still supply a pose while PLANCK tracks its physical body. The consumer decides whether its pocket is usable on a corpse. All values are copies. No subscription, native pointer, physics write or configuration change is required.

### Optional actor physics services

The [actor physics interface](docs/synergy-api.md) adds owned endpoints, capsule fitting and hit policies to the same message transport. An NPC mod can request a [scoped drive policy](docs/drive-api.md); a fitting provider can propose [compound capsule edits](docs/compound-geometry-api.md). Native HIGGS adapters can provide hand-part attribution and receive [own-item contact observations](docs/own-item-contacts.md). These proposed contracts implement the PLANCK side of the [shared interaction concept](https://github.com/Vhodnoylogin/body-pouches/blob/main/docs/trinity-synergy/concept-en.md).

Requests 6 and 7 expose a watched body's potential contact, an optional causal link to a PLANCK hit, and the stages that the native hit actually reached. This lets a fitting mod identify a touched capsule and a gameplay mod apply a rule to an eligible melee hit. [Contacts and hit evidence](docs/synergy-api.md#observing-contacts-and-fitting-the-right-capsule) · [Concept used for this review](https://github.com/Vhodnoylogin/body-pouches/blob/7557db2/docs/trinity-synergy/concept-en.md)
