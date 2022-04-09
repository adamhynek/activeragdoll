#pragma once

#include <set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		float activeRagdollStartDistance = 50.f;
		float activeRagdollEndDistance = 60.f;

		double blendInTime = 0.2;
		double getUpBlendTime = 0.2;

		bool enableKeyframes = true;
		double blendInKeyframeTime = 0.05;

		double hitCooldownTimeStoppedColliding = 0.2;
		double hitCooldownTimeFallback = 1.0;
		double physicsHitRecoveryTime = 0.01;

		double thrownObjectLingerTime = 5.0;

		bool bumpActorsWhenTouched = true;
		bool dontBumpAnimals = true;
		int bumpMaxRelationshipRank = 0; // Acquantaince
		double actorBumpCooldownTime = 3.0;

		bool ragdollOnGrab = false;
		bool ragdollSmallRacesOnGrab = true;
		float smallRaceHealthThreshold = 6.f;

		bool doKeepOffset = true;
		double keepOffsetRetryInterval = 1.0;
		double keepOffsetTimeout = 10.0;

		float collisionDamageMinSpeed = 400.f; // skyrim units
		float collisionDamageMinMass = 6.f;

		bool doRootMotion = false;
		float rootMotionMinOffset = 0.02f;
		float rootMotionVelocityMultiplier = 0.03f;

		float hierarchyGain = 0.6f;
		float velocityGain = 0.6f;
		float positionGain = 0.05f;

		float poweredControllerOnFraction = 0.05f;

		float poweredMaxForce = 500.f;
		float poweredTau = 0.8f;
		float poweredDaming = 1.0f;
		float poweredProportionalRecoveryVelocity = 5.f;
		float poweredConstantRecoveryVelocity = 0.2f;

		bool enableBipedBipedCollision = true;
		bool enableBipedBipedCollisionNoCC = true;
		bool doBipedSelfCollision = true;
		bool doBipedSelfCollisionForNPCs = true;
		bool enableBipedDeadBipCollision = true;
		bool enablePlayerBipedCollision = true;
		bool disableBipedCollisionWithWorld = true;
		bool enableBipedClutterCollision = true;
		bool enableBipedWeaponCollision = true;
		bool loosenRagdollContraintsToMatchPose = true;
		bool convertHingeConstraintsToRagdollConstraints = true;
		bool copyFootIkToPoseTrack = true;
		bool disableCullingForActiveRagdolls = true;
		bool forceGenerateForActiveRagdolls = true;
		bool disableClutterVsCharacterControllerCollisionForActiveActors = true;
		bool doClutterVsBipedCollisionDamage = true;
		bool showCollisionDamageHitFx = false;
		bool forceAnimPose = false;
		bool forceRagdollPose = false;
		bool doBlending = true;
		bool applyImpulseOnHit = true;
		bool useHandVelocityForStabHitDirection = true;
		bool disableHitIfSheathed = false;
		bool blendWhenGettingUp = false;

		float hitImpulseBaseStrength = 1.f;
		float hitImpulseProportionalStrength = -0.15f;
		float hitImpulseMassExponent = 0.5f;

		float hitImpulseMinStrength = 0.2f;
		float hitImpulseMaxStrength = 1.f;
		float hitImpulseMaxVelocity = 1500.f; // skyrim units

		float hitImpulseDownwardsMultiplier = 0.5f;

		float hitSwingSpeedThreshold = 5.f;
		float hitSwingImpulseMult = 1.f;

		float hitStabDirectionThreshold = 0.8f;
		float hitStabSpeedThreshold = 2.f;
		float hitStabImpulseMult = 5.0f;

		float hitPunchDirectionThreshold = 0.7f;
		float hitPunchSpeedThreshold = 2.5f;
		float hitPunchImpulseMult = 2.25f;

		float hitRequiredHandSpeedRoomspace = 1.f;

		float hitImpulseDecayMult1 = 0.225f;
		float hitImpulseDecayMult2 = 0.125f;
		float hitImpulseDecayMult3 = 0.075f;

		float meleeSwingLinearVelocityThreshold = 3.f;
		float shieldSwingLinearVelocityThreshold = 3.f;

		bool resizePlayerCharController = true;
		bool adjustPlayerCharControllerBottomRingHeightToMaintainSlope = true;
		bool resizePlayerCapsule = true;
		bool centerPlayerCapsule = true;
		float playerCharControllerRadius = 0.15f;
		float playerCapsuleRadius = 0.15f;

		std::set<std::string, std::less<>> additionalSelfCollisionRaces;
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
