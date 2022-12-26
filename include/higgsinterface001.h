#pragma once
#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

namespace HiggsPluginAPI {

	// Returns an IHiggsInterface001 object compatible with the API shown below
	// This should only be called after SKSE sends kMessage_PostLoad to your plugin
	struct IHiggsInterface001;
	IHiggsInterface001 *GetHiggsInterface001(const PluginHandle &pluginHandle, SKSEMessagingInterface *messagingInterface);

	// This object provides access to HIGGS's mod support API
	struct IHiggsInterface001
	{
		// Gets the HIGGS build number
		virtual unsigned int GetBuildNumber() = 0;

		// Callbacks for when an object is pulled, grabbed, or dropped/thrown
		typedef void(*PulledCallback)(bool isLeft, TESObjectREFR *pulledRefr);
		virtual void AddPulledCallback(PulledCallback callback) = 0;

		typedef void(*GrabbedCallback)(bool isLeft, TESObjectREFR *grabbedRefr);
		virtual void AddGrabbedCallback(GrabbedCallback callback) = 0;

		typedef void(*DroppedCallback)(bool isLeft, TESObjectREFR *droppedRefr);
		virtual void AddDroppedCallback(DroppedCallback callback) = 0;

		// Callbacks for when an object is stashed (dropped over the shoulder) or consumed (dropped at the mouth)
		// Only the base form is returned in these cases as the object will no longer exist
		typedef void(*StashedCallback)(bool isLeft, TESForm *stashedForm);
		virtual void AddStashedCallback(StashedCallback callback) = 0;

		typedef void(*ConsumedCallback)(bool isLeft, TESForm *consumedForm);
		virtual void AddConsumedCallback(ConsumedCallback callback) = 0;

		// Callback for when the hands or held objects collide with something
		typedef void(*CollisionCallback)(bool isLeft, float mass, float separatingVelocity);
		virtual void AddCollisionCallback(CollisionCallback callback) = 0;

		// Grab the given object reference. It must have collision, and the given hand must be in a ready state as returned by CanGrabObject.
		virtual void GrabObject(TESObjectREFR *object, bool isLeft) = 0;

		// Get the currently held object reference. Note that some references can have multiple physics objects.
		virtual TESObjectREFR *GetGrabbedObject(bool isLeft) = 0;

		// Returns whether the given hand is in a state that can grab an object
		// (no currently held object, not pulling anything or have an object locked in for pulling (i.e. trigger/grip held on a selected object)
		virtual bool IsHandInGrabbableState(bool isLeft) = 0;

		// Disable and enable grabbing, selecting, pulling, etc. for each hand. Every disable should be accompanied by a later enable.
		// Multiple mods can disable at once, and the hand is only re-enabled once all mods have called enable.
		virtual void DisableHand(bool isLeft) = 0;
		virtual void EnableHand(bool isLeft) = 0;
		virtual bool IsDisabled(bool isLeft) = 0;

		// Disable and enable collision for the weapon held in each hand.
		// Multiple mods can disable at once, and the collision is only re-enabled once all mods have called enable.
		virtual void DisableWeaponCollision(bool isLeft) = 0;
		virtual void EnableWeaponCollision(bool isLeft) = 0;
		virtual bool IsWeaponCollisionDisabled(bool isLeft) = 0;

		// Whether both hands are holding a weapon
		virtual bool IsTwoHanding() = 0;

		// Callbacks for starting / stopping two-handing
		typedef void(*StartTwoHandingCallback)();
		virtual void AddStartTwoHandingCallback(StartTwoHandingCallback callback) = 0;

		typedef void(*StopTwoHandingCallback)();
		virtual void AddStopTwoHandingCallback(StopTwoHandingCallback callback) = 0;

		// Returns whether the given hand can actually grab an object right now.
		// This includes whether it is in a grabbable state, but also whether it is holding a blocking weapon or disabled through the api.
		virtual bool CanGrabObject(bool isLeft) = 0;

		enum class CollisionFilterComparisonResult : UInt8 {
			Continue, // Do not affect whether the two objects should collide
			Collide, // Force the two objects to collide
			Ignore, // Force the two objects to not collide
		};
		// Add a callback for when havok compares collision filter info to determine if two objects should collide. This can be called hundreds of times per frame, so be brief.
		// collisionFilter is really of type bhkCollisionFilter
		typedef CollisionFilterComparisonResult(*CollisionFilterComparisonCallback)(void *collisionFilter, UInt32 filterInfoA, UInt32 filterInfoB);
		virtual void AddCollisionFilterComparisonCallback(CollisionFilterComparisonCallback callback) = 0;

		// Add a callback for right before hkpWorld::stepDeltaTime is called.
		// world is really of type bhkWorld
		typedef void(*PrePhysicsStepCallback)(void *world);
		virtual void AddPrePhysicsStepCallback(PrePhysicsStepCallback callback) = 0;

		// Get/set the collision layer bitfield for the higgs collision layer (used for hands, weapons, and held objects).
		virtual UInt64 GetHiggsLayerBitfield() = 0;
		virtual void SetHiggsLayerBitfield(UInt64 bitfield) = 0;

		// Get the hand and weapon rigidbodies that higgs creates. Both return types are really bhkRigidBody.
		virtual NiObject *GetHandRigidBody(bool isLeft) = 0;
		virtual NiObject *GetWeaponRigidBody(bool isLeft) = 0;

		// Get the currently held rigid body. Return type is actually bhkRigidBody.
		virtual NiObject *GetGrabbedRigidBody(bool isLeft) = 0;

		// Forces the weapon collision to stay enabled, unless sheathed or disabled through the API.
		virtual void ForceWeaponCollisionEnabled(bool isLeft) = 0;

		// Get whether the given hand has an object held
		virtual bool IsHoldingObject(bool isLeft) = 0;

		// Get the current amount that each finger is curled. Values range from 0 to 1, where 1 is fully open and 0 is fully curled. Order is from thumb to pinky.
		virtual void GetFingerValues(bool isLeft, float values[5]) = 0;

		typedef void(*NoArgCallback)();
		virtual void AddPreVrikPreHiggsCallback(NoArgCallback callback) = 0;
		virtual void AddPreVrikPostHiggsCallback(NoArgCallback callback) = 0;
		virtual void AddPostVrikPreHiggsCallback(NoArgCallback callback) = 0;
		virtual void AddPostVrikPostHiggsCallback(NoArgCallback callback) = 0;

		// Read or modify any of higgs's numeric ini settings. Returns true if the option exists and is gotten/set, and false otherwise.
		// Only some settings will have an effect if modified, depending on if they are read at startup, when loading / switching cells, or at the time that they are actually required.
		virtual bool GetSettingDouble(const std::string_view &name, double &out) = 0;
		virtual bool SetSettingDouble(const std::string &name, double val) = 0;

		// Get/set the transform of the current grabbed object in the space of the hand that grabs it, i.e. the hand-to-grabbed-node transform.
		// This is the transform the grabbed node is driven/set to, as well as the transform the hand is set from when in physics-grab mode.
		virtual NiTransform GetGrabTransform(bool isLeft) = 0;
		virtual void SetGrabTransform(bool isLeft, const NiTransform &transform) = 0;
	};
}

extern HiggsPluginAPI::IHiggsInterface001 *g_higgsInterface;
