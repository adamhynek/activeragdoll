#pragma once

#include <set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		double stopCollideBlendDurationMin = 0.2;
		double stopCollideBlendDurationMax = 0.4;
		double stopCollideBlendStressMin = 0.0;
		double stopCollideBlendStressMax = 40.0;
		double stopCollideBlendPower = 2.5;
		double blendInTime = 0.15;
		double blendOutTime = 0.15;

		bool doRootMotion = true;
		float rootMotionMinOffset = 0.02f;
		float rootMotionVelocityMultiplier = 0.03f;

		float activeRagdollDistance = 4.f;

		float collideHierarchyGain = 0.6f;
		float collideVelocityGain = 0.6f;
		float collidePositionGain = 0.05f;

		float stopCollideHierarchyGain = 0.6f;
		float stopCollideVelocityGain = 0.6f;
		float stopCollidePositionGain = 0.05f;

		float malleableConstraintStrength = 1.f;

		bool enableBipedCollision = true;
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
