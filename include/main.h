#pragma once

#include "skse64/GameReferences.h"

#include "blender.h"
#include "RE/offsets.h"


enum class RagdollState : UInt8
{
	Keyframed,
	BlendIn,
	Dynamic,
	BlendOut,
};

struct ActiveRagdoll
{
	Blender blender{};
	std::vector<hkQsTransform> animPose{};
	std::vector<hkQsTransform> ragdollPose{};
	std::vector<float> stress{};
	hkQsTransform hipBoneTransform{};
	float avgStress = 0.f;
	RE::hkRefPtr<hkpEaseConstraintsAction> easeConstraintsAction = nullptr;
	double frameTime = 0.0;
	double stateChangedTime = 0.0;
	RagdollState state = RagdollState::Keyframed;
	bool isOn = false;
	bool wantKeyframe = true;
	bool hasHipBoneTransform = false;
	bool shouldNullOutWorldWhenRemovingFromWorld = false;
};
