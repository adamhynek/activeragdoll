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

bool Blender::Update(ActiveRagdoll &ragdoll, hkbGeneratorOutput &inOut, double frameTime)
{
	double elapsedTime = (frameTime - startTime) * *g_globalTimeMultiplier;

	float lerpAmount = curve.GetBlendValueAtTime(elapsedTime);

	hkInt32 numTracks = inOut.m_tracks->m_masterHeader.m_numTracks;

	int poseTrackId = (int)hkbGeneratorOutput::StandardTracks::TRACK_POSE;
	hkbGeneratorOutput::TrackHeader *poseHeader = numTracks > poseTrackId ? &(inOut.m_tracks->m_trackHeaders[poseTrackId]) : nullptr;
	if (poseHeader && poseHeader->m_onFraction > 0.f) {
		int numPoses = poseHeader->m_numData;
		hkQsTransform *poseOut = (hkQsTransform *)Track_getData(inOut, *poseHeader);

		// Save initial pose if necessary
		if ((type == BlendType::AnimToRagdoll || type == BlendType::RagdollToAnim) && isFirstBlendFrame) {
			hkQsTransform *fromPose = nullptr;
			if (type == BlendType::AnimToRagdoll)
				fromPose = ragdoll.animPose.data();
			else if (type == BlendType::RagdollToAnim)
				fromPose = poseOut; // ragdoll pose
			initialPose.assign(fromPose, fromPose + numPoses);
		}
		isFirstBlendFrame = false;

		// Blend poses
		//amounts.assign(numPoses, lerpAmount);
		if (type == BlendType::AnimToRagdoll) {
			// Save the pose before we write to it - at this point it is the high-res pose extracted from the ragdoll
			scratchPose.assign(poseOut, poseOut + numPoses);
			//BlendPoses(initialPose.data(), scratchPose.data(), poseOut, amounts.data(), numPoses);
			hkbBlendPoses(numPoses, initialPose.data(), scratchPose.data(), lerpAmount, poseOut);
		}
		else if (type == BlendType::RagdollToAnim) {
			//BlendPoses(initialPose.data(), g_animPose.data(), poseOut, amounts.data(), numPoses);
			hkbBlendPoses(numPoses, initialPose.data(), ragdoll.animPose.data(), lerpAmount, poseOut);
		}
		else if (type == BlendType::CurrentRagdollToAnim) {
			// Save the pose before we write to it - at this point it is the high-res pose extracted from the ragdoll
			scratchPose.assign(poseOut, poseOut + numPoses);
			//BlendPoses(scratchPose.data(), g_animPose.data(), poseOut, amounts.data(), numPoses);
			hkbBlendPoses(numPoses, scratchPose.data(), ragdoll.animPose.data(), lerpAmount, poseOut);
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
