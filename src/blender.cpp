#include "blender.h"
#include "main.h"
#include "RE/offsets.h"


Blender::Curve::Curve(double duration) : duration(duration) {};

float Blender::Curve::GetBlendValueAtTime(double time)
{
	// Linear by default
	// y = (1/d)*t
	return std::clamp(time / duration, 0.0, 1.0);
}

Blender::PowerCurve::PowerCurve(double duration, double power) :
	a(pow(power, -duration)), // a = b^(-d)
	b(power),
	Curve(duration)
{}

float Blender::PowerCurve::GetBlendValueAtTime(double time)
{
	// y = a * t^b
	return std::clamp(a * pow(time, b), 0.0, 1.0);
}

void Blender::StartBlend(BlendType blendType, double currentTime, const Curve &blendCurve)
{
	if (isActive) {
		// We were already blending before, so set the initial pose to the current blend pose
		initialPose = currentPose;
		isFirstBlendFrame = false;
	}
	else {
		isFirstBlendFrame = true;
	}

	startTime = currentTime;
	type = blendType;
	curve = blendCurve;
	isActive = true;
}

bool Blender::Update(const ActiveRagdoll &ragdoll, const hkbRagdollDriver &driver, hkbGeneratorOutput &inOut, double frameTime)
{
	double elapsedTime = (frameTime - startTime) * *g_globalTimeMultiplier;

	float lerpAmount = curve.GetBlendValueAtTime(elapsedTime);

	hkInt32 numTracks = inOut.m_tracks->m_masterHeader.m_numTracks;

	hkbGeneratorOutput::TrackHeader *poseHeader = GetTrackHeader(inOut, hkbGeneratorOutput::StandardTracks::TRACK_POSE);
	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);

		// Save initial pose if necessary
		if ((type == BlendType::AnimToRagdoll || type == BlendType::RagdollToAnim) && isFirstBlendFrame) {
			if (type == BlendType::AnimToRagdoll)
				initialPose = ragdoll.animPose;
			else if (type == BlendType::RagdollToAnim)
				initialPose = ragdoll.ragdollPose;
		}
		isFirstBlendFrame = false;

		// Blend poses
		if (type == BlendType::AnimToRagdoll) {
			hkbBlendPoses(numPoses, initialPose.data(), ragdoll.ragdollPose.data(), lerpAmount, poseOut);
		}
		else if (type == BlendType::RagdollToAnim) {
			hkbBlendPoses(numPoses, initialPose.data(), ragdoll.animPose.data(), lerpAmount, poseOut);
		}
		else if (type == BlendType::CurrentAnimToRagdoll) {
			hkbBlendPoses(numPoses, ragdoll.animPose.data(), ragdoll.ragdollPose.data(), lerpAmount, poseOut);
		}
		else if (type == BlendType::CurrentRagdollToAnim) {
			hkbBlendPoses(numPoses, ragdoll.ragdollPose.data(), ragdoll.animPose.data(), lerpAmount, poseOut);
		}

		currentPose.assign(poseOut, poseOut + numPoses); // save the blended pose in case we need to blend out from here
	}

	if (elapsedTime >= curve.duration) {
		isActive = false;
		return true;
	}

	return false;
}

void BlendPoses(const hkQsTransform *srcPoses, const hkQsTransform *dstPoses, hkQsTransform *outPoses, float *weights, int numPoses)
{
	for (int i = 0; i < numPoses; i++) {
		outPoses[i].setInterpolate4(srcPoses[i], dstPoses[i], weights[i]);
	}
}
