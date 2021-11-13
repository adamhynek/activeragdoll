#pragma once

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"


typedef bool(*_GetAnimationVariableBool)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* obj, const BSFixedString &asVariableName);
RelocAddr<_GetAnimationVariableBool> GetAnimationVariableBool(0x009CE880);

typedef int(*_GetAnimationVariableInt)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* obj, const BSFixedString &asVariableName);
RelocAddr<_GetAnimationVariableInt> GetAnimationVariableInt(0x009CE950);

typedef void(*_DebugSendAnimationEvent)(VMClassRegistry* registry, UInt32 stackId, void* unk1, TESObjectREFR* objectRefr, const BSFixedString &animEvent);
RelocAddr<_DebugSendAnimationEvent> DebugSendAnimationEvent(0x009A7F40);

typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);
