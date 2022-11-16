#pragma once

#include <algorithm>
#include <vector>

#include <Common/Base/hkBase.h>

#include "RE/havok_behavior.h"

struct Blender
{
	enum class BlendType : UInt8
	{
		AnimToRagdoll,
		CurrentAnimToRagdoll,
		RagdollToAnim,
		CurrentRagdollToAnim,
		RagdollToCurrentRagdoll,
	};

	struct Curve
	{
		Curve(double duration);
		virtual float GetBlendValueAtTime(double time);

		double duration = 1.0;
	};

	struct PowerCurve : Curve
	{
		PowerCurve(double duration, double power);
		virtual float GetBlendValueAtTime(double time) override;

		double a;
		double b;
	};

	void StartBlend(BlendType blendType, double currentTime, const Curve &blendCurve, bool ignoreRoot = false);

	inline void StopBlend() { isActive = false; }

	bool Update(const struct ActiveRagdoll &ragdoll, const hkbRagdollDriver &driver, hkbGeneratorOutput &inOut, double frameTime);

	std::vector<hkQsTransform> initialPose{};
	std::vector<hkQsTransform> currentPose{};
	double startTime = 0.0;
	BlendType type = BlendType::AnimToRagdoll;
	Curve curve{ 1.0 };
	bool isFirstBlendFrame = false;
	bool isActive = false;
	bool ignoreRoot = false;
};

void BlendPoses(const hkQsTransform *srcPoses, const hkQsTransform *dstPoses, hkQsTransform *outPoses, float *weights, int numPoses);
