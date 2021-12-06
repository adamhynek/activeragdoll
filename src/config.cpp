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

	bool ReadConfigOptions()
	{
		if (!ReadDouble("StopCollideMaxTime", options.stopCollideMaxTime)) return false;
		if (!ReadDouble("BlendInTime", options.blendInTime)) return false;
		if (!ReadDouble("BlendOutTime", options.blendOutTime)) return false;

		if (!ReadFloat("StopCollideStressThresholdStart", options.stopCollideStressThresholdStart)) return false;
		if (!ReadFloat("StopCollideStressThresholdEnd", options.stopCollideStressThresholdEnd)) return false;

		if (!ReadFloat("CollideHierarchyGain", options.collideHierarchyGain)) return false;
		if (!ReadFloat("CollideVelocityGain", options.collideVelocityGain)) return false;
		if (!ReadFloat("CollidePositionGain", options.collidePositionGain)) return false;

		if (!ReadFloat("StopCollideHierarchyGain", options.stopCollideHierarchyGain)) return false;
		if (!ReadFloat("StopCollideVelocityGain", options.stopCollideVelocityGain)) return false;
		if (!ReadFloat("StopCollidePositionGain", options.stopCollidePositionGain)) return false;

		if (!ReadBool("DoRootMotion", options.doRootMotion)) return false;
		if (!ReadFloat("RootMotionMinOffset", options.rootMotionMinOffset)) return false;
		if (!ReadFloat("RootMotionVelocityMultiplier", options.rootMotionVelocityMultiplier)) return false;

		if (!ReadFloat("ActiveRagdollDistance", options.activeRagdollDistance)) return false;

		if (!ReadBool("EnableBipedCollision", options.enableBipedCollision)) return false;

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
				s_configPath = runtimePath + "Data\\SKSE\\Plugins\\melee_vr.ini";

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
			char	resultBuf[256];
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
