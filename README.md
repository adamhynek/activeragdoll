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
