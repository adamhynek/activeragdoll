#pragma once

#include <set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		double blendOutDurationMin = 0.2;
		double blendOutDurationMax = 0.5;
		double blendOutDurationStressMin = 0.0;
		double blendOutDurationStressMax = 40.0;
		double blendOutBlendPower = 2.5;
		double blendInTime = 0.15;

		float blendStressWeight = 8.f;

		bool doRootMotion = true;
		float rootMotionMinOffset = 0.02f;
		float rootMotionVelocityMultiplier = 0.03f;

		float activeRagdollDistance = 4.f;

		float collideHierarchyGain = 0.6f;
		float collideVelocityGain = 0.6f;
		float collidePositionGain = 0.05f;

		float blendOutHierarchyGain = 0.6f;
		float blendOutVelocityGain = 0.6f;
		float blendOutPositionGain = 0.05f;

		float malleableConstraintStrength = 1.f;

		bool enableBipedCollision = true;
		bool keyframeBones = true;
		bool forceRagdollPose = false;
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
