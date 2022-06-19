#include <chrono>
#include <filesystem>

#include "config.h"
#include "math_utils.h"
#include "utils.h"


static inline double vlibGetSetting(const char * name) {
	Setting * setting = GetINISetting(name);
	double value;
	if (!setting)
		return -1;
	if (setting->GetDouble(&value))
		return value;
	return -1;
}


namespace Config {
	// Define extern options
	Options options;
	std::map<std::string, float *, std::less<>> floatMap{};
	std::map<std::string, double *, std::less<>> doubleMap{};
	std::map<std::string, int *, std::less<>> intMap{};
	std::map<std::string, bool *, std::less<>> boolMap{};
	bool g_registrationComplete = false;

	bool ReadFloat(const std::string &name, float &val)
	{
		if (!GetConfigOptionFloat("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read float config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadDouble(const std::string &name, double &val)
	{
		if (!GetConfigOptionDouble("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read double config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadBool(const std::string &name, bool &val)
	{
		if (!GetConfigOptionBool("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read bool config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadInt(const std::string &name, int &val)
	{
		if (!GetConfigOptionInt("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read int config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadString(const std::string &name, std::string &val)
	{
		std::string	data = GetConfigOption("Settings", name.c_str());
		if (data.empty()) {
			_WARNING("Failed to read str config option: %s", name.c_str());
			return false;
		}

		val = std::move(data);
		return true;
	}

	bool ReadVector(const std::string &name, NiPoint3 &vec)
	{
		if (!ReadFloat(name + "X", vec.x)) return false;
		if (!ReadFloat(name + "Y", vec.y)) return false;
		if (!ReadFloat(name + "Z", vec.z)) return false;

		return true;
	}

	bool ReadStringSet(const std::string &name, std::set<std::string, std::less<>> &val)
	{
		std::string	data = GetConfigOption("Settings", name.c_str());
		if (data.empty()) {
			_WARNING("Failed to read StringSet config option: %s", name.c_str());
			return false;
		}

		val = SplitStringToSet(data, ',');
		return true;
	}

	bool ReadFormArray(const std::string &name, std::vector<UInt32> &val)
	{
		std::string	data = GetConfigOption("Settings", name.c_str());
		if (data.empty()) {
			_WARNING("Failed to read StringSet config option: %s", name.c_str());
			return false;
		}

		val.clear(); // first empty the set, since we will be reading into it

		std::set<std::string, std::less<>> stringSet = SplitStringToSet(data, ',');
		for (const std::string &str : stringSet) {
			val.push_back(std::stoul(str, nullptr, 16));
		}
		return true;
	}

	bool RegisterFloat(const std::string &name, float &val)
	{
		if (!g_registrationComplete) floatMap[name] = &val;
		return ReadFloat(name, val);
	}

	bool RegisterDouble(const std::string &name, double &val)
	{
		if (!g_registrationComplete) doubleMap[name] = &val;
		return ReadDouble(name, val);
	}

	bool RegisterInt(const std::string &name, int &val)
	{
		if (!g_registrationComplete) intMap[name] = &val;
		return ReadInt(name, val);
	}

	bool RegisterBool(const std::string &name, bool &val)
	{
		if (!g_registrationComplete) boolMap[name] = &val;
		return ReadBool(name, val);
	}

	bool SetSettingDouble(const std::string_view &name, double val)
	{
		if (auto it = doubleMap.find(name); it != doubleMap.end()) {
			*it->second = val;
			return true;
		}
		if (auto it = floatMap.find(name); it != floatMap.end()) {
			*it->second = float(val);
			return true;
		}
		if (auto it = intMap.find(name); it != intMap.end()) {
			*it->second = int(val);
			return true;
		}
		if (auto it = boolMap.find(name); it != boolMap.end()) {
			*it->second = bool(val);
			return true;
		}
		return false;
	}

	bool GetSettingDouble(const std::string_view &name, double &out)
	{
		if (auto it = doubleMap.find(name); it != doubleMap.end()) {
			out = *it->second;
			return true;
		}
		if (auto it = floatMap.find(name); it != floatMap.end()) {
			out = double(*it->second);
			return true;
		}
		if (auto it = intMap.find(name); it != intMap.end()) {
			out = double(*it->second);
			return true;
		}
		if (auto it = boolMap.find(name); it != boolMap.end()) {
			out = double(*it->second);
			return true;
		}
		return false;
	}

	bool ReadConfigOptions()
	{
		bool success = true;

		if (!RegisterFloat("activeRagdollStartDistance", options.activeRagdollStartDistance)) success = false;
		if (!RegisterFloat("activeRagdollEndDistance", options.activeRagdollEndDistance)) success = false;

		if (!RegisterDouble("blendInTime", options.blendInTime)) success = false;

		if (!RegisterBool("enableKeyframes", options.enableKeyframes)) success = false;
		if (!RegisterDouble("blendInKeyframeTime", options.blendInKeyframeTime)) success = false;

		if (!RegisterDouble("hitCooldownTimeStoppedColliding", options.hitCooldownTimeStoppedColliding)) success = false;
		if (!RegisterDouble("hitCooldownTimeFallback", options.hitCooldownTimeFallback)) success = false;
		if (!RegisterDouble("physicsHitRecoveryTime", options.physicsHitRecoveryTime)) success = false;

		if (!RegisterDouble("thrownObjectLingerTime", options.thrownObjectLingerTime)) success = false;

		if (!RegisterDouble("worldChangedWaitTime", options.worldChangedWaitTime)) success = false;

		if (!RegisterBool("enableActorShove", options.enableActorShove)) success = false;
		if (!RegisterBool("disableShoveWhileWeaponsDrawn", options.disableShoveWhileWeaponsDrawn)) success = false;
		if (!RegisterBool("enableShoveFromFurniture", options.enableShoveFromFurniture)) success = false;
		if (!RegisterBool("playShovePhysicsSound", options.playShovePhysicsSound)) success = false;
		if (!RegisterBool("playSoundOnShoveNoStamina", options.playSoundOnShoveNoStamina)) success = false;
		if (!RegisterFloat("shoveStaggerMagnitude", options.shoveStaggerMagnitude)) success = false;
		if (!RegisterFloat("shoveStaminaCost", options.shoveStaminaCost)) success = false;
		if (!RegisterFloat("shoveSpeedThreshold", options.shoveSpeedThreshold)) success = false;
		if (!RegisterFloat("shoveRumbleIntensity", options.shoveRumbleIntensity)) success = false;
		if (!RegisterFloat("shoveRumbleDuration", options.shoveRumbleDuration)) success = false;
		if (!RegisterFloat("shoveAggressionImpact", options.shoveAggressionImpact)) success = false;
		if (!RegisterDouble("shoveCooldown", options.shoveCooldown)) success = false;
		if (!RegisterDouble("collisionCooldownTime", options.collisionCooldownTime)) success = false;
		if (!RegisterDouble("shoveWaitForBumpTimeBeforeStagger", options.shoveWaitForBumpTimeBeforeStagger)) success = false;
		if (!ReadFormArray("shoveTopicInfos", options.shoveTopicInfos)) success = false;


		if (!RegisterBool("enableBump", options.enableBump)) success = false;
		if (!RegisterBool("doAggression", options.doAggression)) success = false;
		if (!RegisterBool("followersSkipAggression", options.followersSkipAggression)) success = false;
		if (!RegisterBool("summonsSkipAggression", options.summonsSkipAggression)) success = false;

		if (!RegisterBool("stopUsingFurnitureOnHighAggression", options.stopUsingFurnitureOnHighAggression)) success = false;
		if (!RegisterBool("calmedActorsDontAccumulateAggression", options.calmedActorsDontAccumulateAggression)) success = false;
		if (!RegisterDouble("aggressionBumpCooldownTime", options.aggressionBumpCooldownTime)) success = false;
		if (!RegisterDouble("aggressionStopDelay", options.aggressionStopDelay)) success = false;
		if (!RegisterDouble("aggressionDialogueInitMaxTime", options.aggressionDialogueInitMaxTime)) success = false;
		if (!RegisterDouble("aggressionDialogueCooldownFallback", options.aggressionDialogueCooldownFallback)) success = false;
		if (!RegisterDouble("aggressionDialogueCooldown", options.aggressionDialogueCooldown)) success = false;
		if (!RegisterFloat("aggressionRequiredGrabTimeLow", options.aggressionRequiredGrabTimeLow)) success = false;
		if (!RegisterFloat("aggressionRequiredGrabTimeHigh", options.aggressionRequiredGrabTimeHigh)) success = false;
		if (!RegisterFloat("aggressionRequiredGrabTimeAssault", options.aggressionRequiredGrabTimeAssault)) success = false;
		if (!RegisterFloat("aggressionMaxAccumulatedGrabTime", options.aggressionMaxAccumulatedGrabTime)) success = false;
		if (!RegisterFloat("aggressionStopCombatAlarmDistance", options.aggressionStopCombatAlarmDistance)) success = false;
		if (!RegisterFloat("aggressionRequiredHandWithinHmdConeHalfAngle", options.aggressionRequiredHandWithinHmdConeHalfAngle)) success = false;
		if (!RegisterInt("aggressionMaxRelationshipRank", options.aggressionMaxRelationshipRank)) success = false;
		if (!ReadFormArray("aggressionLowTopicInfos", options.aggressionLowTopicInfos)) success = false;
		if (!ReadFormArray("aggressionHighTopicInfos", options.aggressionHighTopicInfos)) success = false;

		if (!RegisterBool("doSpeedReduction", options.doSpeedReduction)) success = false;
		if (!RegisterFloat("smallRaceSpeedReduction", options.smallRaceSpeedReduction)) success = false;
		if (!RegisterFloat("mediumRaceSpeedReduction", options.mediumRaceSpeedReduction)) success = false;
		if (!RegisterFloat("largeRaceSpeedReduction", options.largeRaceSpeedReduction)) success = false;
		if (!RegisterFloat("extraLargeRaceSpeedReduction", options.extraLargeRaceSpeedReduction)) success = false;
		if (!RegisterFloat("maxSpeedReduction", options.maxSpeedReduction)) success = false;
		if (!RegisterFloat("speedReductionHealthInfluence", options.speedReductionHealthInfluence)) success = false;
		if (!RegisterFloat("followerSpeedReductionMultiplier", options.followerSpeedReductionMultiplier)) success = false;

		if (!RegisterBool("followersSkipStaminaCost", options.followersSkipStaminaCost)) success = false;
		if (!RegisterBool("playSoundOnGrabStaminaDepletion", options.playSoundOnGrabStaminaDepletion)) success = false;
		if (!RegisterFloat("grabbedActorStaminaCost", options.grabbedActorStaminaCost)) success = false;
		if (!RegisterFloat("grabbedActorHostileStaminaCost", options.grabbedActorHostileStaminaCost)) success = false;
		if (!RegisterFloat("grabbedActorStaminaCostHealthInfluence", options.grabbedActorStaminaCostHealthInfluence)) success = false;
		if (!RegisterInt("grabbedstaminaDrainMaxRelationshipRank", options.grabbedstaminaDrainMaxRelationshipRank)) success = false;

		if (!RegisterBool("ragdollOnGrab", options.ragdollOnGrab)) success = false;
		if (!RegisterBool("ragdollSmallRacesOnGrab", options.ragdollSmallRacesOnGrab)) success = false;
		if (!RegisterFloat("smallRaceHealthThreshold", options.smallRaceHealthThreshold)) success = false;

		if (!RegisterBool("doKeepOffset", options.doKeepOffset)) success = false;
		if (!RegisterBool("bumpActorIfKeepOffsetFails", options.bumpActorIfKeepOffsetFails)) success = false;
		if (!RegisterFloat("keepOffsetMinAngleDifference", options.keepOffsetMinAngleDifference)) success = false;
		if (!RegisterFloat("keepOffsetAngleDifferenceMultiplier", options.keepOffsetAngleDifferenceMultiplier)) success = false;
		if (!RegisterDouble("keepOffsetRetryInterval ", options.keepOffsetRetryInterval)) success = false;

		if (!RegisterFloat("collisionDamageMinSpeed", options.collisionDamageMinSpeed)) success = false;
		if (!RegisterFloat("collisionDamageMinMass", options.collisionDamageMinMass)) success = false;

		if (!RegisterBool("doWarp", options.doWarp)) success = false;
		if (!RegisterFloat("maxAllowedDistBeforeWarp", options.maxAllowedDistBeforeWarp)) success = false;

		if (!RegisterFloat("hierarchyGain", options.hierarchyGain)) success = false;
		if (!RegisterFloat("velocityGain", options.velocityGain)) success = false;
		if (!RegisterFloat("positionGain", options.positionGain)) success = false;

		if (!RegisterFloat("poweredControllerOnFraction", options.poweredControllerOnFraction)) success = false;

		if (!RegisterFloat("poweredMaxForce", options.poweredMaxForce)) success = false;
		if (!RegisterFloat("poweredTau", options.poweredTau)) success = false;
		if (!RegisterFloat("poweredDaming", options.poweredDaming)) success = false;
		if (!RegisterFloat("poweredProportionalRecoveryVelocity", options.poweredProportionalRecoveryVelocity)) success = false;
		if (!RegisterFloat("poweredConstantRecoveryVelocity", options.poweredConstantRecoveryVelocity)) success = false;

		if (!RegisterFloat("ragdollBoneMaxLinearVelocity", options.ragdollBoneMaxLinearVelocity)) success = false;
		if (!RegisterFloat("ragdollBoneMaxAngularVelocity", options.ragdollBoneMaxAngularVelocity)) success = false;

		if (!RegisterBool("overrideSoundVelForRagdollCollisions", options.overrideSoundVelForRagdollCollisions)) success = false;
		if (!RegisterFloat("ragdollSoundVel", options.ragdollSoundVel)) success = false;

		if (!RegisterFloat("playerVsBipedInteractionImpulseMultiplier", options.playerVsBipedInteractionImpulseMultiplier)) success = false;

		if (!RegisterBool("stopRagdollNonSelfCollisionForCloseActors", options.stopRagdollNonSelfCollisionForCloseActors)) success = false;
		if (!RegisterFloat("closeActorMinDistance", options.closeActorMinDistance)) success = false;

		if (!RegisterBool("stopRagdollNonSelfCollisionForActorsWithVehicle", options.stopRagdollNonSelfCollisionForActorsWithVehicle)) success = false;

		if (!RegisterBool("stopAggressionForCloseActors", options.stopAggressionForCloseActors)) success = false;
		if (!RegisterBool("stopAggressionForActorsWithVehicle", options.stopAggressionForActorsWithVehicle)) success = false;

		if (!RegisterBool("enableBipedBipedCollision", options.enableBipedBipedCollision)) success = false;
		if (!RegisterBool("enableBipedBipedCollisionNoCC", options.enableBipedBipedCollisionNoCC)) success = false;
		if (!RegisterBool("doBipedSelfCollision", options.doBipedSelfCollision)) success = false;
		if (!RegisterBool("doBipedSelfCollisionForNPCs", options.doBipedSelfCollisionForNPCs)) success = false;
		if (!RegisterBool("doBipedNonSelfCollision", options.doBipedNonSelfCollision)) success = false;
		if (!RegisterBool("enableBipedDeadBipCollision", options.enableBipedDeadBipCollision)) success = false;
		if (!RegisterBool("enablePlayerBipedCollision", options.enablePlayerBipedCollision)) success = false;
		if (!RegisterBool("disableBipedCollisionWithWorld", options.disableBipedCollisionWithWorld)) success = false;
		if (!RegisterBool("enableBipedClutterCollision", options.enableBipedClutterCollision)) success = false;
		if (!RegisterBool("enableBipedWeaponCollision", options.enableBipedWeaponCollision)) success = false;
		if (!RegisterBool("enableBipedProjectileCollision", options.enableBipedProjectileCollision)) success = false;
		if (!RegisterBool("disableGravityForActiveRagdolls", options.disableGravityForActiveRagdolls)) success = false;
		if (!RegisterBool("loosenRagdollContraintsToMatchPose", options.loosenRagdollContraintsToMatchPose)) success = false;
		if (!RegisterBool("convertHingeConstraintsToRagdollConstraints", options.convertHingeConstraintsToRagdollConstraints)) success = false;
		if (!RegisterBool("copyFootIkToPoseTrack", options.copyFootIkToPoseTrack)) success = false;
		if (!RegisterBool("disableCullingForActiveRagdolls", options.disableCullingForActiveRagdolls)) success = false;
		if (!RegisterBool("forceGenerateForActiveRagdolls", options.forceGenerateForActiveRagdolls)) success = false;
		if (!RegisterBool("forceAnimationUpdateForActiveActors", options.forceAnimationUpdateForActiveActors)) success = false;
		if (!RegisterBool("disableClutterVsCharacterControllerCollisionForActiveActors", options.disableClutterVsCharacterControllerCollisionForActiveActors)) success = false;
		if (!RegisterBool("doClutterVsBipedCollisionDamage", options.doClutterVsBipedCollisionDamage)) success = false;
		if (!RegisterBool("showCollisionDamageHitFx", options.showCollisionDamageHitFx)) success = false;
		if (!RegisterBool("forceAnimPose", options.forceAnimPose)) success = false;
		if (!RegisterBool("forceRagdollPose", options.forceRagdollPose)) success = false;
		if (!RegisterBool("doBlending", options.doBlending)) success = false;
		if (!RegisterBool("applyImpulseOnHit", options.applyImpulseOnHit)) success = false;
		if (!RegisterBool("useHandVelocityForStabHitDirection", options.useHandVelocityForStabHitDirection)) success = false;
		if (!RegisterBool("disableHitIfSheathed", options.disableHitIfSheathed)) success = false;
		if (!RegisterBool("blendWhenGettingUp", options.blendWhenGettingUp)) success = false;
		if (!RegisterBool("seamlessFurnitureTransition", options.seamlessFurnitureTransition)) success = false;

		if (!RegisterFloat("hitImpulseBaseStrength", options.hitImpulseBaseStrength)) success = false;
		if (!RegisterFloat("hitImpulseProportionalStrength", options.hitImpulseProportionalStrength)) success = false;
		if (!RegisterFloat("hitImpulseMassExponent", options.hitImpulseMassExponent)) success = false;

		if (!RegisterFloat("hitImpulseMinStrength", options.hitImpulseMinStrength)) success = false;
		if (!RegisterFloat("hitImpulseMaxStrength", options.hitImpulseMaxStrength)) success = false;
		if (!RegisterFloat("hitImpulseMaxVelocity", options.hitImpulseMaxVelocity)) success = false;

		if (!RegisterFloat("hitImpulseDownwardsMultiplier", options.hitImpulseDownwardsMultiplier)) success = false;

		if (!RegisterFloat("hitSwingSpeedThreshold", options.hitSwingSpeedThreshold)) success = false;
		if (!RegisterFloat("hitSwingImpulseMult", options.hitSwingImpulseMult)) success = false;

		if (!RegisterFloat("hitStabDirectionThreshold", options.hitStabDirectionThreshold)) success = false;
		if (!RegisterFloat("hitStabSpeedThreshold", options.hitStabSpeedThreshold)) success = false;
		if (!RegisterFloat("hitStabImpulseMult", options.hitStabImpulseMult)) success = false;

		if (!RegisterFloat("hitPunchDirectionThreshold", options.hitPunchDirectionThreshold)) success = false;
		if (!RegisterFloat("hitPunchSpeedThreshold", options.hitPunchSpeedThreshold)) success = false;
		if (!RegisterFloat("hitPunchImpulseMult", options.hitPunchImpulseMult)) success = false;

		if (!RegisterFloat("hitRequiredHandSpeedRoomspace", options.hitRequiredHandSpeedRoomspace)) success = false;

		if (!RegisterFloat("hitImpulseDecayMult1", options.hitImpulseDecayMult1)) success = false;
		if (!RegisterFloat("hitImpulseDecayMult2", options.hitImpulseDecayMult2)) success = false;
		if (!RegisterFloat("hitImpulseDecayMult3", options.hitImpulseDecayMult3)) success = false;

		if (!RegisterFloat("meleeSwingLinearVelocityThreshold", options.meleeSwingLinearVelocityThreshold)) success = false;
		if (!RegisterFloat("shieldSwingLinearVelocityThreshold", options.shieldSwingLinearVelocityThreshold)) success = false;

		if (!RegisterBool("resizePlayerCharController", options.resizePlayerCharController)) success = false;
		if (!RegisterBool("adjustPlayerCharControllerBottomRingHeightToMaintainSlope", options.adjustPlayerCharControllerBottomRingHeightToMaintainSlope)) success = false;
		if (!RegisterFloat("playerCharControllerBottomRingMaxHeightAdjustment", options.playerCharControllerBottomRingMaxHeightAdjustment)) success = false;
		if (!RegisterBool("resizePlayerCapsule", options.resizePlayerCapsule)) success = false;
		if (!RegisterBool("centerPlayerCapsule", options.centerPlayerCapsule)) success = false;
		if (!RegisterFloat("playerCharControllerRadius", options.playerCharControllerRadius)) success = false;
		if (!RegisterFloat("playerCapsuleRadius", options.playerCapsuleRadius)) success = false;

		if (!ReadStringSet("additionalSelfCollisionRaces", Config::options.additionalSelfCollisionRaces)) success = false;
		if (!ReadStringSet("excludeRaces", Config::options.excludeRaces)) success = false;
		if (!ReadStringSet("aggressionExcludeRaces", Config::options.aggressionExcludeRaces)) success = false;

		g_registrationComplete = true;

		return success;
	}

	bool ReloadIfModified()
	{
		namespace fs = std::filesystem;

		static long long lastModifiedConfigTime = 0;

		const std::string &path = GetConfigPath();
		auto ftime = fs::last_write_time(path);
		auto time = ftime.time_since_epoch().count();
		if (time > lastModifiedConfigTime) {
			lastModifiedConfigTime = time;

			// Reload config if file has been modified since we last read it
			if (Config::ReadConfigOptions()) {
				_MESSAGE("Successfully reloaded config parameters");
			}
			else {
				_WARNING("[WARNING] Failed to reload config options");
			}

			return true;
		}

		return false;
	}

	const std::string & GetConfigPath()
	{
		static std::string s_configPath;

		if (s_configPath.empty()) {
			std::string	runtimePath = GetRuntimeDirectory();
			if (!runtimePath.empty()) {
				s_configPath = runtimePath + "Data\\SKSE\\Plugins\\activeragdoll.ini";

				_MESSAGE("config path = %s", s_configPath.c_str());
			}
		}

		return s_configPath;
	}

	std::string GetConfigOption(const char *section, const char *key)
	{
		std::string	result;

		const std::string & configPath = GetConfigPath();
		if (!configPath.empty()) {
			static char resultBuf[4096];
			resultBuf[0] = 0;

			UInt32	resultLen = GetPrivateProfileString(section, key, NULL, resultBuf, sizeof(resultBuf), configPath.c_str());

			result = resultBuf;
		}

		return result;
	}

	bool GetConfigOptionDouble(const char *section, const char *key, double *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		*out = std::stod(data);
		return true;
	}

	bool GetConfigOptionFloat(const char *section, const char *key, float *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		*out = std::stof(data);
		return true;
	}

	bool GetConfigOptionInt(const char *section, const char *key, int *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		*out = std::stoi(data);
		return true;
	}

	bool GetConfigOptionBool(const char *section, const char *key, bool *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		int val = std::stoi(data);
		if (val == 1) {
			*out = true;
			return true;
		}
		else if (val == 0) {
			*out = false;
			return true;
		}
		else {
			return false;
		}
	}
}
