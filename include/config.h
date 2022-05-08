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

		double worldChangedWaitTime = 0.4;

		bool bumpActorsWhenTouched = true;
		bool exitFurnitureWhenBumped = true;
		float bumpSpeedThreshold = 1.f;
		float largeBumpSpeedThreshold = 2.f;
		bool dontBumpAnimals = true;
		bool dontBumpFollowers = false;
		int bumpMaxRelationshipRank = 0; // Acquantaince
		double actorBumpCooldownTime = 3.0;

		bool doAggression = true;
		bool followersSkipAggression = true;
		bool stopUsingFurnitureOnHighAggression = true;
		double aggressionDialogueCooldownTime = 1.0;
		float aggressionRequiredGrabTimeLow = 0.25f;
		float aggressionRequiredGrabTimeLowFallback = 1.f;
		float aggressionRequiredGrabTimeHigh = 3.5f;
		float aggressionRequiredGrabTimeAssault = 7.f;
		float aggressionMaxAccumulatedGrabTime = 20.f;
		int aggressionMaxRelationshipRank = 0; // Acquantaince
		int aggressionDialogueSubtypeLow = 88; // pickpocketTopic
		int aggressionDialogueSubtypeHigh = 49; // trespass

		bool doSpeedReduction = true;
		float maxSpeedReduction = 80.f;
		float speedReductionHealthInfluence = 0.5f;
		float speedReductionMassProportion = 0.625f;
		float speedReductionMassExponent = 1.f;
		float followerSpeedReductionMultiplier = 0.2f;

		bool followersSkipStaminaCost = true;
		float grabbedActorStaminaCost = 9.f;
		float grabbedActorStaminaCostHealthInfluence = 0.8f;

		bool ragdollOnGrab = false;
		bool ragdollSmallRacesOnGrab = true;
		float smallRaceHealthThreshold = 6.f;

		bool doKeepOffset = true;
		bool bumpActorIfKeepOffsetFails = true;
		double keepOffsetRetryInterval = 1.0;

		float collisionDamageMinSpeed = 400.f; // skyrim units
		float collisionDamageMinMass = 6.f;

		bool doWarp = true;
		float maxAllowedDistBeforeWarp = 15.f;

		float hierarchyGain = 0.6f;
		float velocityGain = 0.6f;
		float positionGain = 0.05f;

		float poweredControllerOnFraction = 0.05f;

		float poweredMaxForce = 500.f;
		float poweredTau = 0.8f;
		float poweredDaming = 1.0f;
		float poweredProportionalRecoveryVelocity = 5.f;
		float poweredConstantRecoveryVelocity = 0.2f;

		float ragdollBoneMaxLinearVelocity = 500.f;
		float ragdollBoneMaxAngularVelocity = 500.f;

		bool overrideSoundVelForRagdollCollisions = true;
		float ragdollSoundVel = 1000.f;

		float playerVsBipedInteractionImpulseMultiplier = 0.f;

		bool stopRagdollNonSelfCollisionForCloseActors = true;
		double closeActorFilterRefreshInterval = 1.0;
		float ragdollNonSelfCollisionActorMinDistance = 2.f;

		bool enableBipedBipedCollision = true;
		bool enableBipedBipedCollisionNoCC = true;
		bool doBipedSelfCollision = true;
		bool doBipedSelfCollisionForNPCs = true;
		bool doBipedNonSelfCollision = true;
		bool enableBipedDeadBipCollision = true;
		bool enablePlayerBipedCollision = true;
		bool disableBipedCollisionWithWorld = true;
		bool enableBipedClutterCollision = true;
		bool enableBipedWeaponCollision = true;
		bool disableGravityForActiveRagdolls = true;
		bool loosenRagdollContraintsToMatchPose = true;
		bool convertHingeConstraintsToRagdollConstraints = true;
		bool copyFootIkToPoseTrack = true;
		bool disableCullingForActiveRagdolls = true;
		bool forceGenerateForActiveRagdolls = true;
		bool forceAnimationUpdateForActiveActors = true;
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
		bool disableConstraints = false;

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
		std::set<std::string, std::less<>> excludeRaces;
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
