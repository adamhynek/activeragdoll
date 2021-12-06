#pragma once

#include <set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		double stopCollideMaxTime = 2.0;
		double blendOutMaxTime = 0.15;

		float stopCollideStressThresholdStart = 8.f;
		float stopCollideStressThresholdEnd = 12.f;
		float blendOutStressThresholdStart = 0.5f;
		float blendOutStressThresholdEnd = 0.5f;

		bool doRootMotion = true;
		float rootMotionMinOffset = 0.02f;
		float rootMotionVelocityMultiplier = 0.03f;

		float activeRagdollDistance = 4.f;

		float hierarchyGain = 0.17f;
		float blendOutVelocityGain = 0.3f;
		float blendOutPositionGain = 0.1f;
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
