#pragma once

#include "skse64/GameReferences.h"

#include "blender.h"
#include "RE/offsets.h"


enum class RagdollState : UInt8
{
	Idle,
	BlendIn,
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
	float deltaTime = 0.f;
	RE::hkRefPtr<hkpEaseConstraintsAction> easeConstraintsAction = nullptr;
	double frameTime = 0.0;
	RagdollState state = RagdollState::Idle;
	bool isOn = false;
	bool hasHipBoneTransform = false;
	bool shouldNullOutWorldWhenRemovingFromWorld = false;
};
