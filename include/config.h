#pragma once

#include <set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		double blendOutDurationMin = 1.0;
		double blendOutDurationMax = 1.0;
		double blendOutDurationStressMin = 5.0;
		double blendOutDurationStressMax = 40.0;
		double blendOutBlendPower = 2.5;
		double blendInTime = 0.06;

		bool doRootMotion = false;
		float rootMotionMinOffset = 0.02f;
		float rootMotionVelocityMultiplier = 0.03f;

		float activeRagdollDistance = 3.f;

		float collideHierarchyGain = 0.6f;
		float collideVelocityGain = 0.6f;
		float collidePositionGain = 0.05f;

		float blendOutHierarchyGain = 0.6f;
		float blendOutVelocityGain = 0.6f;
		float blendOutPositionGain = 0.05f;

		float poweredControllerOnFraction = 0.01f;

		float poweredMaxForce = 1000.f;
		float poweredTau = 0.8f;
		float poweredDaming = 1.0f;
		float poweredProportionalRecoveryVelocity = 5.f;
		float poweredConstantRecoveryVelocity = 0.2f;

		float malleableConstraintStrength = 1.f;

		bool enableBipedCollision = true;
		bool keyframeBones = true;
		bool forceRagdollPose = false;
		bool doBlending = true;
	};
	extern Options options; // global object containing options


	// Fills Options struct from INI file
	bool ReadConfigOptions();

	bool ReloadIfModified();

	const std::string & GetConfigPath();

	std::string GetConfigOption(const char * section, const char * key);

	bool GetConfigOptionDouble(const char *section, const char *key, double *out);
	bool GetConfigOptionFloat(const char *section, const char *key, float *out);
	bool GetConfigOptionInt(const char *section, const char *key, int *out);
	bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}
