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

	bool ReadConfigOptions()
	{
		if (!ReadFloat("activeRagdollStartDistance", options.activeRagdollStartDistance)) return false;
		if (!ReadFloat("activeRagdollEndDistance", options.activeRagdollEndDistance)) return false;

		if (!ReadDouble("blendInTime", options.blendInTime)) return false;

		if (!ReadBool("enableKeyframes", options.enableKeyframes)) return false;
		if (!ReadDouble("blendInKeyframeTime", options.blendInKeyframeTime)) return false;

		if (!ReadDouble("hitCooldownTimeStoppedColliding", options.hitCooldownTimeStoppedColliding)) return false;
		if (!ReadDouble("hitCooldownTimeFallback", options.hitCooldownTimeFallback)) return false;
		if (!ReadDouble("physicsHitRecoveryTime", options.physicsHitRecoveryTime)) return false;

		if (!ReadDouble("thrownObjectLingerTime", options.thrownObjectLingerTime)) return false;

		if (!ReadBool("bumpActorsWhenTouched", options.bumpActorsWhenTouched)) return false;

		if (!ReadBool("dontBumpAnimals", options.dontBumpAnimals)) return false;
		if (!ReadInt("bumpMaxRelationshipRank", options.bumpMaxRelationshipRank)) return false;

		if (!ReadDouble("actorBumpCooldownTime", options.actorBumpCooldownTime)) return false;

		if (!ReadBool("ragdollOnGrab", options.ragdollOnGrab)) return false;
		if (!ReadBool("ragdollSmallRacesOnGrab", options.ragdollSmallRacesOnGrab)) return false;
		if (!ReadFloat("smallRaceHealthThreshold", options.smallRaceHealthThreshold)) return false;

		if (!ReadBool("doKeepOffset", options.doKeepOffset)) return false;

		if (!ReadFloat("collisionDamageMinSpeed", options.collisionDamageMinSpeed)) return false;
		if (!ReadFloat("collisionDamageMinMass", options.collisionDamageMinMass)) return false;

		if (!ReadBool("doRootMotion", options.doRootMotion)) return false;
		if (!ReadFloat("rootMotionMinOffset", options.rootMotionMinOffset)) return false;
		if (!ReadFloat("rootMotionVelocityMultiplier", options.rootMotionVelocityMultiplier)) return false;

		if (!ReadFloat("hierarchyGain", options.hierarchyGain)) return false;
		if (!ReadFloat("velocityGain", options.velocityGain)) return false;
		if (!ReadFloat("positionGain", options.positionGain)) return false;

		if (!ReadFloat("poweredControllerOnFraction", options.poweredControllerOnFraction)) return false;

		if (!ReadFloat("poweredMaxForce", options.poweredMaxForce)) return false;
		if (!ReadFloat("poweredTau", options.poweredTau)) return false;
		if (!ReadFloat("poweredDaming", options.poweredDaming)) return false;
		if (!ReadFloat("poweredProportionalRecoveryVelocity", options.poweredProportionalRecoveryVelocity)) return false;
		if (!ReadFloat("poweredConstantRecoveryVelocity", options.poweredConstantRecoveryVelocity)) return false;

		if (!ReadBool("enableBipedBipedCollision", options.enableBipedBipedCollision)) return false;
		if (!ReadBool("enableBipedBipedCollisionNoCC", options.enableBipedBipedCollisionNoCC)) return false;
		if (!ReadBool("doBipedSelfCollision", options.doBipedSelfCollision)) return false;
		if (!ReadBool("doBipedSelfCollisionForNPCs", options.doBipedSelfCollisionForNPCs)) return false;
		if (!ReadBool("enableBipedDeadBipCollision", options.enableBipedDeadBipCollision)) return false;
		if (!ReadBool("enablePlayerBipedCollision", options.enablePlayerBipedCollision)) return false;
		if (!ReadBool("disableBipedCollisionWithWorld", options.disableBipedCollisionWithWorld)) return false;
		if (!ReadBool("enableBipedClutterCollision", options.enableBipedClutterCollision)) return false;
		if (!ReadBool("enableBipedWeaponCollision", options.enableBipedWeaponCollision)) return false;
		if (!ReadBool("loosenRagdollContraintsToMatchPose", options.loosenRagdollContraintsToMatchPose)) return false;
		if (!ReadBool("convertHingeConstraintsToRagdollConstraints", options.convertHingeConstraintsToRagdollConstraints)) return false;
		if (!ReadBool("copyFootIkToPoseTrack", options.copyFootIkToPoseTrack)) return false;
		if (!ReadBool("disableCullingForActiveRagdolls", options.disableCullingForActiveRagdolls)) return false;
		if (!ReadBool("forceGenerateForActiveRagdolls", options.forceGenerateForActiveRagdolls)) return false;
		if (!ReadBool("disableClutterVsCharacterControllerCollisionForActiveActors", options.disableClutterVsCharacterControllerCollisionForActiveActors)) return false;
		if (!ReadBool("doClutterVsBipedCollisionDamage", options.doClutterVsBipedCollisionDamage)) return false;
		if (!ReadBool("showCollisionDamageHitFx", options.showCollisionDamageHitFx)) return false;
		if (!ReadBool("forceAnimPose", options.forceAnimPose)) return false;
		if (!ReadBool("forceRagdollPose", options.forceRagdollPose)) return false;
		if (!ReadBool("doBlending", options.doBlending)) return false;
		if (!ReadBool("applyImpulseOnHit", options.applyImpulseOnHit)) return false;
		if (!ReadBool("useHandVelocityForStabHitDirection", options.useHandVelocityForStabHitDirection)) return false;
		if (!ReadBool("disableHitIfSheathed", options.disableHitIfSheathed)) return false;
		if (!ReadBool("blendWhenGettingUp", options.blendWhenGettingUp)) return false;

		if (!ReadFloat("hitImpulseBaseStrength", options.hitImpulseBaseStrength)) return false;
		if (!ReadFloat("hitImpulseProportionalStrength", options.hitImpulseProportionalStrength)) return false;
		if (!ReadFloat("hitImpulseMassExponent", options.hitImpulseMassExponent)) return false;

		if (!ReadFloat("hitImpulseMinStrength", options.hitImpulseMinStrength)) return false;
		if (!ReadFloat("hitImpulseMaxStrength", options.hitImpulseMaxStrength)) return false;
		if (!ReadFloat("hitImpulseMaxVelocity", options.hitImpulseMaxVelocity)) return false;

		if (!ReadFloat("hitImpulseDownwardsMultiplier", options.hitImpulseDownwardsMultiplier)) return false;

		if (!ReadFloat("hitSwingSpeedThreshold", options.hitSwingSpeedThreshold)) return false;
		if (!ReadFloat("hitSwingImpulseMult", options.hitSwingImpulseMult)) return false;

		if (!ReadFloat("hitStabDirectionThreshold", options.hitStabDirectionThreshold)) return false;
		if (!ReadFloat("hitStabSpeedThreshold", options.hitStabSpeedThreshold)) return false;
		if (!ReadFloat("hitStabImpulseMult", options.hitStabImpulseMult)) return false;

		if (!ReadFloat("hitPunchDirectionThreshold", options.hitPunchDirectionThreshold)) return false;
		if (!ReadFloat("hitPunchSpeedThreshold", options.hitPunchSpeedThreshold)) return false;
		if (!ReadFloat("hitPunchImpulseMult", options.hitPunchImpulseMult)) return false;

		if (!ReadFloat("hitRequiredHandSpeedRoomspace", options.hitRequiredHandSpeedRoomspace)) return false;

		if (!ReadFloat("hitImpulseDecayMult1", options.hitImpulseDecayMult1)) return false;
		if (!ReadFloat("hitImpulseDecayMult2", options.hitImpulseDecayMult2)) return false;
		if (!ReadFloat("hitImpulseDecayMult3", options.hitImpulseDecayMult3)) return false;

		if (!ReadFloat("meleeSwingLinearVelocityThreshold", options.meleeSwingLinearVelocityThreshold)) return false;
		if (!ReadFloat("shieldSwingLinearVelocityThreshold", options.shieldSwingLinearVelocityThreshold)) return false;

		if (!ReadBool("resizePlayerCharController", options.resizePlayerCharController)) return false;
		if (!ReadBool("adjustPlayerCharControllerBottomRingHeightToMaintainSlope", options.adjustPlayerCharControllerBottomRingHeightToMaintainSlope)) return false;
		if (!ReadBool("resizePlayerCapsule", options.resizePlayerCapsule)) return false;
		if (!ReadBool("centerPlayerCapsule", options.centerPlayerCapsule)) return false;
		if (!ReadFloat("playerCharControllerRadius", options.playerCharControllerRadius)) return false;
		if (!ReadFloat("playerCapsuleRadius", options.playerCapsuleRadius)) return false;

		if (!ReadStringSet("additionalSelfCollisionRaces", Config::options.additionalSelfCollisionRaces)) return false;

		return true;
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
