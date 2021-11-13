namespace DualWieldBlockVR
{
	const std::string & GetConfigPath();
	std::string GetConfigOption(const char * section, const char * key);
	bool GetConfigOptionFloat(const char *section, const char *key, float *out);
	bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}