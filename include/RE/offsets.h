#pragma once
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Utilities/Constraint/Keyframe/hkpKeyFrameUtility.h>
#include <Physics/Utilities/Collide/TriggerVolume/hkpTriggerVolume.h>
#include <Physics/Utilities/Actions/EaseConstraints/hkpEaseConstraintsAction.h>

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"
#include "skse64/NiNodes.h"
#include "skse64/GameVR.h"
#include "skse64/NiGeometry.h"

#include "RE/havok.h"
#include "RE/havok_behavior.h"
#include "RE/misc.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
extern RelocPtr<float> g_havokWorldScale;
extern RelocPtr<float> g_inverseHavokWorldScale;

// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
extern RelocPtr<bhkSimpleShapePhantom *> g_pickSphere;

extern RelocPtr<CrosshairPickData *> g_pickData;

extern RelocPtr<float> g_deltaTime;

extern RelocPtr<float> g_globalTimeMultiplier;

extern RelocPtr<float> fMaxTime;
extern RelocPtr<float> fMaxTimeComplex;

extern RelocPtr<float> g_secondsSinceLastFrame_WorldTime_CheckPaused;
extern RelocPtr<float> g_secondsSinceLastFrame_WorldTime;
extern RelocPtr<float> g_secondsSinceLastFrame_Unmultiplied;

extern RelocPtr<int> g_currentFrameCounter;
extern RelocPtr<int> g_sceneComplexCounter;
extern RelocPtr<int> g_iShadowUpdateFrameDelay;
extern RelocPtr<int> g_nextShadowUpdateFrameCount;

struct BSAudioManager { /* TODO */ };
extern RelocPtr<BSAudioManager *> g_audioManager;

struct ShadowSceneNode : NiNode { /* TODO */ };
extern RelocPtr<ShadowSceneNode *> g_shadowSceneNode;

extern RelocPtr<float> g_minSoundVel;

extern RelocPtr<float> g_fMeleeWeaponHavokScale;

extern RelocPtr<float> g_fMagicHandTranslateX;
extern RelocPtr<float> g_fMagicHandTranslateY;
extern RelocPtr<float> g_fMagicHandTranslateZ;
extern RelocPtr<float> g_fMagicHandRotateX;
extern RelocPtr<float> g_fMagicHandRotateY;
extern RelocPtr<float> g_fMagicHandRotateZ;
extern RelocPtr<float> g_fMagicHandScale;

extern RelocPtr<float> g_fMeleeLinearVelocityThreshold;
extern RelocPtr<float> g_fShieldLinearVelocityThreshold;

extern RelocPtr<DWORD> g_dwTlsIndex;


// Havok / Bethesda havok wrappers
typedef float(*_hkpWorld_getCurrentTime)(hkpWorld *world);
extern RelocAddr<_hkpWorld_getCurrentTime> hkpWorld_getCurrentTime;

typedef void(*_hkpWorld_CastRay)(hkpWorld *world, hkpWorldRayCastInput *input, hkpRayHitCollector *collector);
extern RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay;

typedef void(*_hkpWorld_LinearCast)(hkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector);
extern RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast;

typedef void(*_hkpWorld_GetPenetrations)(hkpWorld *world, const hkpCollidable* collA, const hkpCollisionInput* input, hkpCdBodyPairCollector* collector);
extern RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations;

typedef void(*_hkpWorld_GetClosestPoints)(hkpWorld *world, const hkpCollidable* collA, const hkpCollisionInput* input, hkpCdPointCollector* collector);
extern RelocAddr<_hkpWorld_GetClosestPoints> hkpWorld_GetClosestPoints;

typedef hkpEntity* (*_hkpWorld_AddEntity)(hkpWorld *world, hkpEntity* entity, hkpEntityActivation initialActivationState);
extern RelocAddr<_hkpWorld_AddEntity> hkpWorld_AddEntity;

typedef hkpEntity* (*_hkpWorld_RemoveEntity)(hkpWorld *world, hkBool *ret, hkpEntity* entity);
extern RelocAddr<_hkpWorld_RemoveEntity> hkpWorld_RemoveEntity;

typedef void* (*_hkpWorld_addContactListener)(hkpWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_hkpWorld_addContactListener> hkpWorld_addContactListener;

typedef void* (*_hkpWorld_removeContactListener)(hkpWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_hkpWorld_removeContactListener> hkpWorld_removeContactListener;

typedef void* (*_hkpWorld_addIslandActivationListener)(hkpWorld *world, hkpIslandActivationListener* worldListener);
extern RelocAddr<_hkpWorld_addIslandActivationListener> hkpWorld_addIslandActivationListener;

typedef void* (*_hkpWorld_removeIslandActivationListener)(hkpWorld *world, hkpIslandActivationListener* worldListener);
extern RelocAddr<_hkpWorld_removeIslandActivationListener> hkpWorld_removeIslandActivationListener;

typedef void* (*_bhkWorld_addContactListener)(bhkWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_bhkWorld_addContactListener> bhkWorld_addContactListener;

typedef void(*_hkpWorld_UpdateCollisionFilterOnEntity)(hkpWorld *world, hkpEntity* entity, hkpUpdateCollisionFilterOnEntityMode updateMode, hkpUpdateCollectionFilterMode updateShapeCollectionFilter);
extern RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity;

typedef void(*_bhkWorld_UpdateCollisionFilterOnEntity)(bhkWorld *world, hkpEntity* entity);
extern RelocAddr<_bhkWorld_UpdateCollisionFilterOnEntity> bhkWorld_UpdateCollisionFilterOnEntity;

typedef void(*_ContactListener_PreprocessContactPointEvent)(hkpContactListener *listener, const hkpContactPointEvent &evnt);
extern RelocAddr<_ContactListener_PreprocessContactPointEvent> ContactListener_PreprocessContactPointEvent;

typedef float(*_hkpSimpleContactConstraintUtil_calculateSeparatingVelocity)(const hkpRigidBody* bodyA, const hkpRigidBody* bodyB, const hkVector4& centerOfMassInWorldA, const hkVector4& centerOfMassInWorldB, const hkContactPoint* cp);
extern RelocAddr<_hkpSimpleContactConstraintUtil_calculateSeparatingVelocity> hkpSimpleContactConstraintUtil_calculateSeparatingVelocity;

typedef void(*_hkpEntity_activate)(hkpEntity *entity);
extern RelocAddr<_hkpEntity_activate> hkpEntity_activate;

typedef void(*_bhkRigidBody_setActivated)(bhkRigidBody *rigidBody, bool activate);
extern RelocAddr<_bhkRigidBody_setActivated> bhkRigidBody_setActivated;

typedef void(*_hkpEntity_setPositionAndRotation)(hkpEntity *_this, const hkVector4& position, const hkQuaternion& rotation);
extern RelocAddr<_hkpEntity_setPositionAndRotation> hkpEntity_setPositionAndRotation;

typedef void(*_hkpEntity_setTransform)(hkpEntity *_this, const hkTransform& transform);
extern RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform;

typedef int(*_hkpEntity_getNumConstraints)(hkpEntity *_this);
extern RelocAddr<_hkpEntity_getNumConstraints> hkpEntity_getNumConstraints;

typedef void* (*_hkpEntity_addContactListener)(hkpEntity *_this, hkpContactListener* cl);
extern RelocAddr<_hkpEntity_addContactListener> hkpEntity_addContactListener;

typedef void* (*_hkpEntity_removeContactListener)(hkpEntity *_this, hkpContactListener* cl);
extern RelocAddr<_hkpEntity_removeContactListener> hkpEntity_removeContactListener;

typedef void(*_hkpRigidBody_ctor)(hkpRigidBody *_this, hkpRigidBodyCinfo *info);
extern RelocAddr<_hkpRigidBody_ctor> hkpRigidBody_ctor;

typedef void(*_hkpRigidBodyCinfo_ctor)(hkpRigidBodyCinfo *_this);
extern RelocAddr<_hkpRigidBodyCinfo_ctor> hkpRigidBodyCinfo_ctor;

typedef hkpBoxShape* (*_hkpBoxShape_ctor)(hkpBoxShape *_this, const hkVector4& halfExtents, float radius);
extern RelocAddr<_hkpBoxShape_ctor> hkpBoxShape_ctor;

typedef hkpTriggerVolume* (*_hkpTriggerVolume_ctor)(hkpTriggerVolume *_this, hkpRigidBody* triggerBody);
extern RelocAddr<_hkpTriggerVolume_ctor> hkpTriggerVolume_ctor;

typedef void(*_hkpKeyFrameUtility_applyHardKeyFrame)(const hkVector4& nextPosition, const hkQuaternion& nextOrientation, hkReal invDeltaTime, hkpRigidBody* body);
extern RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrame;
extern RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrameAsynchronously;

typedef void(*_hkpKeyFrameUtility_applySoftKeyFrame)(const hkpKeyFrameUtility::KeyFrameInfo& keyFrameInfo, hkpKeyFrameUtility::AccelerationInfo& accelInfo, hkReal deltaTime, hkReal invDeltaTime, hkpRigidBody* body);
extern RelocAddr<_hkpKeyFrameUtility_applySoftKeyFrame> hkpKeyFrameUtility_applySoftKeyFrame;

typedef void(*_hkpConstraintInstance_setPriority)(hkpConstraintInstance *_this, hkpConstraintInstance::ConstraintPriority priority);
extern RelocAddr<_hkpConstraintInstance_setPriority> hkpConstraintInstance_setPriority;

typedef void(*_hkpMotion_approxTransformAt)(hkpMotion *motion, float time, hkTransform& transformOut);
extern RelocAddr<_hkpMotion_approxTransformAt> hkpMotion_approxTransformAt;

typedef bool(*_bhkCollisionFilter_CompareFilterInfos)(bhkCollisionFilter *filter, UInt32 filterInfoA, UInt32 filterInfoB);
extern RelocAddr<_bhkCollisionFilter_CompareFilterInfos> bhkCollisionFilter_CompareFilterInfos;

// newState HAS to be a UInt64, NOT a hkpMotion::MotionType, as hkpMotion::MotionType is a UInt8 but the actual function in the binary expects a UInt64. Fuck.
typedef void(*_hkpRigidBody_setMotionType)(hkpRigidBody *_this, UInt64 newState, hkpEntityActivation preferredActivationState, hkpUpdateCollisionFilterOnEntityMode collisionFilterUpdateMode);
extern RelocAddr<_hkpRigidBody_setMotionType> hkpRigidBody_setMotionType;

typedef void(*_bhkRigidBody_setMotionType)(bhkRigidBody *_this, UInt64 newState, hkpEntityActivation preferredActivationState, hkpUpdateCollisionFilterOnEntityMode collisionFilterUpdateMode);
extern RelocAddr<_bhkRigidBody_setMotionType> bhkRigidBody_setMotionType;

typedef void(*_bhkRigidBody_MoveToPositionAndRotation)(bhkRigidBody *_this, NiPoint3 &pos, NiQuaternion &rot);
extern RelocAddr<_bhkRigidBody_MoveToPositionAndRotation> bhkRigidBody_MoveToPositionAndRotation;

typedef void(*_bhkCollisionObject_SetNodeTransformsFromWorldTransform)(bhkCollisionObject *_this, NiTransform &worldTransform);
extern RelocAddr<_bhkCollisionObject_SetNodeTransformsFromWorldTransform> bhkCollisionObject_SetNodeTransformsFromWorldTransform;

typedef void(*_bhkEntity_setPositionAndRotation)(bhkEntity *_this, const hkVector4& position, const hkQuaternion& rotation);
extern RelocAddr<_bhkEntity_setPositionAndRotation> bhkEntity_setPositionAndRotation;

typedef void(*_bhkWorldObject_UpdateCollisionFilter)(bhkWorldObject *_this);
extern RelocAddr<_bhkWorldObject_UpdateCollisionFilter> bhkWorldObject_UpdateCollisionFilter;

typedef void(*_bhkRigidBodyCinfo_ctor)(bhkRigidBodyCinfo *_this);
extern RelocAddr<_bhkRigidBodyCinfo_ctor> bhkRigidBodyCinfo_ctor;

typedef void(*_bhkRigidBody_ctor)(bhkRigidBody *_this, bhkRigidBodyCinfo *cInfo);
extern RelocAddr<_bhkRigidBody_ctor> bhkRigidBody_ctor;

typedef void(*_bhkBoxShape_ctor)(bhkBoxShape *_this, hkVector4 *halfExtents);
extern RelocAddr<_bhkBoxShape_ctor> bhkBoxShape_ctor;

typedef void(*_hkReferencedObject_addReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference;

typedef void(*_hkReferencedObject_removeReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference;


// More havok-related
typedef bhkWorld * (*_GetHavokWorldFromCell)(TESObjectCELL *cell);
extern RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell;

typedef NiAVObject * (*_GetNodeFromCollidable)(hkpCollidable * a_collidable);
extern RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable;

typedef TESObjectREFR * (*_GetRefFromCollidable)(hkpCollidable * a_collidable);
extern RelocAddr<_GetRefFromCollidable> GetRefFromCollidable;


typedef NiTransform * (*_BSVRInterface_GetHandTransform)(BSOpenVR *_this, NiTransform *transformOut, BSVRInterface::BSControllerHand handForOpenVRDeviceIndex, BSVRInterface::BSControllerHand handForBSOpenVRTransform);
extern RelocAddr<_BSVRInterface_GetHandTransform> BSOpenVR_GetHandTransform;

typedef void(*_CreateDetectionEvent)(ActorProcessManager *ownerProcess, Actor *owner, NiPoint3 *position, int soundLevel, TESObjectREFR *source);
extern RelocAddr<_CreateDetectionEvent> CreateDetectionEvent;

typedef void(*_ShadowSceneNode_UpdateNodeList)(ShadowSceneNode *sceneNode, NiAVObject *node, bool useOtherList);
extern RelocAddr<_ShadowSceneNode_UpdateNodeList> ShadowSceneNode_UpdateNodeList;

typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
extern RelocAddr<_IsInMenuMode> IsInMenuMode;

typedef bool(*_ObjectReference_SetActorCause)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, Actor *actor);
extern RelocAddr<_ObjectReference_SetActorCause> ObjectReference_SetActorCause;

typedef bool(*_ObjectReference_Activate)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, TESObjectREFR* activator, bool defaultProcessingOnly);
extern RelocAddr<_ObjectReference_Activate> ObjectReference_Activate;

typedef bool(*_TESObjectREFR_Activate)(TESObjectREFR* activatee, TESObjectREFR* activator, UInt32 unk01, UInt32 unk02, UInt32 count, bool defaultProcessingOnly); // unks are 0, 0
extern RelocAddr<_TESObjectREFR_Activate> TESObjectREFR_Activate;

typedef bool(*_TESObjectREFR_SetScale)(TESObjectREFR* refr, float scale);
extern RelocAddr<_TESObjectREFR_SetScale> TESObjectREFR_SetScale;

typedef void(*_EffectShader_Play)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target, float duration);
extern RelocAddr<_EffectShader_Play> EffectShader_Play;

typedef void(*_EffectShader_Stop)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target);
extern RelocAddr<_EffectShader_Stop> EffectShader_Stop;

typedef void(*_VisualEffect_Play)(VMClassRegistry* registry, UInt32 stackId, BGSReferenceEffect *effect, TESObjectREFR *target, float duration, TESObjectREFR *objToFace);
extern RelocAddr<_VisualEffect_Play> VisualEffect_Play;

typedef void(*_VisualEffect_Stop)(VMClassRegistry* registry, UInt32 stackId, BGSReferenceEffect *effect, TESObjectREFR *target);
extern RelocAddr<_VisualEffect_Stop> VisualEffect_Stop;

typedef UInt32(*_Sound_Play)(VMClassRegistry* registry, UInt32 stackId, BGSSoundDescriptorForm *sound, TESObjectREFR *source);
extern RelocAddr<_Sound_Play> Sound_Play;

typedef void(*_BSExtraDataList_RemoveOwnership)(BaseExtraList *_this);
extern RelocAddr<_BSExtraDataList_RemoveOwnership> BSExtraDataList_RemoveOwnership;

typedef void(*_BSExtraDataList_SetOwnerForm)(BaseExtraList *_this, TESForm *form);
extern RelocAddr<_BSExtraDataList_SetOwnerForm> BSExtraDataList_SetOwnerForm;

typedef void(*_TESObjectREFR_SetActorOwner)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR *_this, TESForm *owner);
extern RelocAddr<_TESObjectREFR_SetActorOwner> TESObjectREFR_SetActorOwner;

typedef void(*_NiAVObject_RecalculateWorldTransform)(NiAVObject *_this);
extern RelocAddr<_NiAVObject_RecalculateWorldTransform> NiAVObject_RecalculateWorldTransform;

typedef void(*_ActivatePickRef)(PlayerCharacter *player);
extern RelocAddr<_ActivatePickRef> ActivatePickRef;

typedef float(*_TESObjectREFR_GetMass)(float sum, bool firstPerson, TESObjectREFR *obj);
extern RelocAddr<_TESObjectREFR_GetMass> TESObjectREFR_GetMass;

typedef float(*_NiAVObject_GetMass)(NiAVObject *node, float sum);
extern RelocAddr<_NiAVObject_GetMass> NiAVObject_GetMass;

typedef void(*_StartGrabObject)(PlayerCharacter *_this, UInt64 isRightHand);
extern RelocAddr<_StartGrabObject> StartGrabObject;

typedef void(*_TESObjectREFR_SetPosition)(TESObjectREFR *_this, NiPoint3 &newPos);
extern RelocAddr<_TESObjectREFR_SetPosition> TESObjectREFR_SetPosition;

typedef void(*_TESObjectREFR_SetRotation)(TESObjectREFR *_this, NiPoint3 &newRot);
extern RelocAddr<_TESObjectREFR_SetRotation> TESObjectREFR_SetRotation;

typedef void(*_NiAVObject_UpdateNode)(NiAVObject *_this, NiAVObject::ControllerUpdateContext *ctx);
extern RelocAddr<_NiAVObject_UpdateNode> NiAVObject_UpdateNode;

typedef BGSMaterialType * (*_GetMaterialType)(UInt32 materialId); // materialId is gotten from the bhkShape at offset 0x20
extern RelocAddr<_GetMaterialType> GetMaterialType;

typedef BGSImpactData * (*_BGSImpactDataSet_GetImpactData)(BGSImpactDataSet *_this, BGSMaterialType *material);
extern RelocAddr<_BGSImpactDataSet_GetImpactData> BGSImpactDataSet_GetImpactData;

typedef void(*_BSAudioManager_InitSoundData)(BSAudioManager *audioManager, SoundData *soundData, UInt32 formId, int a4); // just pass 16 in a4
extern RelocAddr<_BSAudioManager_InitSoundData> BSAudioManager_InitSoundData;

typedef bool(*_SoundData_SetPosition)(SoundData *soundData, float x, float y, float z);
extern RelocAddr<_SoundData_SetPosition> SoundData_SetPosition;

typedef void(*_SoundData_SetNode)(SoundData *soundData, NiAVObject *node);
extern RelocAddr<_SoundData_SetNode> SoundData_SetNode;

typedef bool(*_SoundData_Play)(SoundData *SoundData);
extern RelocAddr<_SoundData_Play> SoundData_Play;

typedef UInt32(*_BSExtraList_GetCount)(BaseExtraList *extraList);
extern RelocAddr<_BSExtraList_GetCount> BSExtraList_GetCount;

typedef bool(*_TESObjectBOOK_LearnSpell)(TESObjectBOOK *book, Actor *reader);
extern RelocAddr<_TESObjectBOOK_LearnSpell> TESObjectBOOK_LearnSpell;

typedef BGSSoundDescriptorForm * (*_Actor_GetPickupPutdownSound)(Actor *_this, TESBoundObject *object, bool pickup, bool use);
extern RelocAddr<_Actor_GetPickupPutdownSound> Actor_GetPickupPutdownSound;

typedef void(*_NiMatrixToNiQuaternion)(NiQuaternion &quatOut, const NiMatrix33 &matIn);
extern RelocAddr<_NiMatrixToNiQuaternion> NiMatrixToNiQuaternion;

typedef NiMatrix33 * (*_MatrixFromForwardVector)(NiMatrix33 *matOut, NiPoint3 *forward, NiPoint3 *world);
extern RelocAddr<_MatrixFromForwardVector> MatrixFromForwardVector;

typedef NiMatrix33 & (*_EulerToNiMatrix)(NiMatrix33 &matOut, float x, float y, float z);
extern RelocAddr<_EulerToNiMatrix> EulerToNiMatrix;

typedef void(*_UpdateClavicleToTransformHand)(NiAVObject *clavicle, NiAVObject *hand, NiTransform *desiredHandWorldTransform, NiTransform *additionalLocalTransform);
extern RelocAddr<_UpdateClavicleToTransformHand> UpdateClavicleToTransformHand;

typedef void(*_NiSkinInstance_UpdateBoneMatrices)(NiSkinInstance *_this, NiTransform &rootTransform);
extern RelocAddr<_NiSkinInstance_UpdateBoneMatrices> NiSkinInstance_UpdateBoneMatrices;

typedef NiObject * (*_NiObject_Clone)(NiObject *_this, NiCloningProcess *cloningProcess);
extern RelocAddr<_NiObject_Clone> NiObject_Clone;

typedef NiAVObject * (*_PlayerCharacter_GetOffsetNodeForWeaponIndex)(PlayerCharacter *_this, UInt32 isLeft, UInt32 weaponIndex);
extern RelocAddr<_PlayerCharacter_GetOffsetNodeForWeaponIndex> PlayerCharacter_GetOffsetNodeForWeaponIndex;

typedef void(*_BSFixedString_Copy)(BSFixedString *dst, BSFixedString *src);
extern RelocAddr<_BSFixedString_Copy> BSFixedString_Copy;

typedef void(*_RefreshActivateButtonArt)(void *wsActivateRollover);
extern RelocAddr<_RefreshActivateButtonArt> RefreshActivateButtonArt;


extern RelocPtr<AIProcessManager *> g_aiProcessManager;
extern RelocPtr<float> g_bAlwaysDriveRagdoll;


typedef bool(*_bhkRefObject_ctor)(bhkRefObject *_this);
extern RelocAddr<_bhkRefObject_ctor> bhkRefObject_ctor;

typedef bool(*_hkMalleableConstraintCinfo_Func4)(hkMalleableConstraintCinfo *_this);
extern RelocAddr<_hkMalleableConstraintCinfo_Func4> hkMalleableConstraintCinfo_Func4;

typedef bool(*_hkRagdollConstraintCinfo_Func4)(hkRagdollConstraintCinfo *_this);
extern RelocAddr<_hkRagdollConstraintCinfo_Func4> hkRagdollConstraintCinfo_Func4;

typedef bool(*_hkMalleableConstraintCinfo_setWrappedConstraintData)(hkMalleableConstraintCinfo *_this, hkpConstraintData *data);
extern RelocAddr<_hkMalleableConstraintCinfo_setWrappedConstraintData> hkMalleableConstraintCinfo_setWrappedConstraintData;

typedef bool(*_hkMalleableConstraintCinfo_setStrength)(hkMalleableConstraintCinfo *_this, float strength);
extern RelocAddr<_hkMalleableConstraintCinfo_setStrength> hkMalleableConstraintCinfo_setStrength;

typedef bool(*_Actor_IsInRagdollState)(Actor *_this);
extern RelocAddr<_Actor_IsInRagdollState> Actor_IsInRagdollState;

typedef void(*_BSAnimationGraphManager_HasRagdollInterface)(BSAnimationGraphManager *_this, bool *out);
extern RelocAddr<_BSAnimationGraphManager_HasRagdollInterface> BSAnimationGraphManager_HasRagdollInterface;

typedef void(*_BSAnimationGraphManager_AddRagdollToWorld)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_AddRagdollToWorld> BSAnimationGraphManager_AddRagdollToWorld;

typedef void(*_BSAnimationGraphManager_RemoveRagdollFromWorld)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_RemoveRagdollFromWorld> BSAnimationGraphManager_RemoveRagdollFromWorld;

typedef void(*_NiNode_AddOrRemoveMalleableConstraints)(NiNode *_this, bool a1, bool a2, bool a3);
extern RelocAddr<_NiNode_AddOrRemoveMalleableConstraints> NiNode_AddOrRemoveMalleableConstraints;

typedef void(*_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints> BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints;

typedef hkaRagdollInstance * (*_hkbRagdollDriver_getRagdollInterface)(hkbRagdollDriver *_this);
extern RelocAddr<_hkbRagdollDriver_getRagdollInterface> hkbRagdollDriver_getRagdollInterface;

typedef bhkConstraint * (*_ConstraintToFixedConstraint)(bhkConstraint *constraint, float strength, bool a3);
extern RelocAddr<_ConstraintToFixedConstraint> ConstraintToFixedConstraint;

typedef void(*_hkpConstraintInstance_setEnabled)(hkpConstraintInstance *_this, bool enable);
extern RelocAddr<_hkpConstraintInstance_setEnabled> hkpConstraintInstance_setEnabled;

typedef bool * (*_hkpConstraintInstance_isEnabled)(hkpConstraintInstance *_this, bool *enabled);
extern RelocAddr<_hkpConstraintInstance_isEnabled> hkpConstraintInstance_isEnabled;

typedef bool * (*_hkpConstraintInstance_ctor)(hkpConstraintInstance *_this, hkpEntity* entityA, hkpEntity* entityB, hkpConstraintData* data, hkpConstraintInstance::ConstraintPriority priority);
extern RelocAddr<_hkpConstraintInstance_ctor> hkpConstraintInstance_ctor;

typedef bool * (*_hkpRagdollConstraintData_ctor)(hkpRagdollConstraintData *_this);
extern RelocAddr<_hkpRagdollConstraintData_ctor> hkpRagdollConstraintData_ctor;

typedef bool(*_hkpCollisionCallbackUtil_requireCollisionCallbackUtil)(hkpWorld *world);
extern RelocAddr<_hkpCollisionCallbackUtil_requireCollisionCallbackUtil> hkpCollisionCallbackUtil_requireCollisionCallbackUtil;

typedef bool(*_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil)(hkpWorld *world);
extern RelocAddr<_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil> hkpCollisionCallbackUtil_releaseCollisionCallbackUtil;

typedef hkpWorldExtension * (*_hkpWorld_findWorldExtension)(hkpWorld *world, int id);
extern RelocAddr<_hkpWorld_findWorldExtension> hkpWorld_findWorldExtension;

typedef void(*_ahkpCharacterProxy_setLinearVelocity)(hkpCharacterProxy *_this, const hkVector4& vel);
extern RelocAddr<_ahkpCharacterProxy_setLinearVelocity> ahkpCharacterProxy_setLinearVelocity;

typedef void(*_ahkpCharacterRigidBody_setLinearVelocity)(hkpCharacterRigidBody *_this, const hkVector4& newVel, hkReal timestep);
extern RelocAddr<_ahkpCharacterRigidBody_setLinearVelocity> ahkpCharacterRigidBody_setLinearVelocity;

typedef hkVector4 & (*_ahkpCharacterRigidBody_getLinearVelocity)(hkpCharacterRigidBody *_this);
extern RelocAddr<_ahkpCharacterRigidBody_getLinearVelocity> ahkpCharacterRigidBody_getLinearVelocity;

typedef hkVector4 & (*_hkbBlendPoses)(UInt32 numData, const hkQsTransform *src, const hkQsTransform *dst, float amount, hkQsTransform *out);
extern RelocAddr<_hkbBlendPoses> hkbBlendPoses;

typedef bool(*_hkConstraintCinfo_setConstraintData)(struct hkConstraintCinfo *_this, hkpConstraintData *data);
extern RelocAddr<_hkConstraintCinfo_setConstraintData> hkConstraintCinfo_setConstraintData;

typedef float(*_hkpRagdollConstraintData_setInBodySpace)(hkpRagdollConstraintData *_this,
	const hkVector4& pivotA, const hkVector4& pivotB,
	const hkVector4& planeAxisA, const hkVector4& planeAxisB,
	const hkVector4& twistAxisA, const hkVector4& twistAxisB);
extern RelocAddr<_hkpRagdollConstraintData_setInBodySpace> hkpRagdollConstraintData_setInBodySpace;

typedef bool(*_bhkRagdollConstraint_ctor)(bhkRagdollConstraint *_this, hkRagdollConstraintCinfo *cInfo);
extern RelocAddr<_bhkRagdollConstraint_ctor> bhkRagdollConstraint_ctor;

typedef void(*_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld)(hkbRagdollDriver *_this, const hkQsTransform* highResPoseLocal, const hkQsTransform& worldFromModel, hkQsTransform* lowResPoseWorld);
extern RelocAddr<_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld> hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld;

typedef void(*_hkRotation_setFromQuat)(hkRotation *_this, hkQuaternionParameter q);
extern RelocAddr<_hkRotation_setFromQuat> hkRotation_setFromQuat;

typedef void(*_hkpConstraintUtils_loosenConstraintLimits)(hkpConstraintInstance* constraint);
extern RelocAddr<_hkpConstraintUtils_loosenConstraintLimits> hkpConstraintUtils_loosenConstraintLimits;

typedef bool(*_hkpEaseConstraintsAction_ctor)(hkpEaseConstraintsAction *_this, const hkArray<hkpEntity*>& entities, hkUlong userData);
extern RelocAddr<_hkpEaseConstraintsAction_ctor> hkpEaseConstraintsAction_ctor;

typedef bool(*_hkpEaseConstraintsAction_loosenConstraints)(hkpEaseConstraintsAction *_this);
extern RelocAddr<_hkpEaseConstraintsAction_loosenConstraints> hkpEaseConstraintsAction_loosenConstraints;

typedef bool(*_hkpEaseConstraintsAction_restoreConstraints)(hkpEaseConstraintsAction *_this, hkReal duration);
extern RelocAddr<_hkpEaseConstraintsAction_restoreConstraints> hkpEaseConstraintsAction_restoreConstraints;

typedef hkpConstraintInstance * (*_hkpConstraintUtils_convertToPowered)(const hkpConstraintInstance* originalConstraint, hkpConstraintMotor* constraintMotor, hkBool enableMotors);
extern RelocAddr<_hkpConstraintUtils_convertToPowered> hkpConstraintUtils_convertToPowered;

typedef void(*_hkpWorld_addWorldPostSimulationListener)(hkpWorld *_this, hkpWorldPostSimulationListener* worldListener);
extern RelocAddr<_hkpWorld_addWorldPostSimulationListener> hkpWorld_addWorldPostSimulationListener;

typedef void(*_hkpWorld_removeWorldPostSimulationListener)(hkpWorld *_this, hkpWorldPostSimulationListener* worldListener);
extern RelocAddr<_hkpWorld_removeWorldPostSimulationListener> hkpWorld_removeWorldPostSimulationListener;

typedef bool(*_Actor_IsGhost)(Actor *_this);
extern RelocAddr<_Actor_IsGhost> Actor_IsGhost;

typedef bool(*_Character_CanHit)(Character *_this, Actor *target);
extern RelocAddr<_Character_CanHit> Character_CanHit;

typedef void(*_PlayerCharacter_UpdateAndGetAttackData)(PlayerCharacter *_this, bool isUsingMotionControllers, bool isLeft, bool isPowerAttack, BGSAttackData **attackDataOut);
extern RelocAddr<_PlayerCharacter_UpdateAndGetAttackData> PlayerCharacter_UpdateAndGetAttackData;

typedef bool(*_ActorProcess_IncrementAttackCounter)(ActorProcessManager *_this, int incCount);
extern RelocAddr<_ActorProcess_IncrementAttackCounter> ActorProcess_IncrementAttackCounter;

typedef int(*_TESObjectWEAP_GetSoundAmount)(TESObjectWEAP *_this);
extern RelocAddr<_TESObjectWEAP_GetSoundAmount> TESObjectWEAP_GetSoundAmount;

typedef void(*_Actor_SetActionValue)(Actor *_this, int actionValue);
extern RelocAddr<_Actor_SetActionValue> Actor_SetActionValue;

typedef int(*_TESNPC_GetSoundAmount)(TESNPC *_this);
extern RelocAddr<_TESNPC_GetSoundAmount> TESNPC_GetSoundAmount;

typedef void(*_CombatController_sub_14050DEC0)(void *_this);
extern RelocAddr<_CombatController_sub_14050DEC0> CombatController_sub_14050DEC0;

typedef void(*_Actor_RemoveMagicEffectsDueToAction)(Actor *_this, int action);
extern RelocAddr<_Actor_RemoveMagicEffectsDueToAction> Actor_RemoveMagicEffectsDueToAction;

typedef void(*_Character_HitTarget)(Character *_this, Actor *target, Projectile *projectile, bool isOffhand);
extern RelocAddr<_Character_HitTarget> Character_HitTarget;

typedef void(*_UpdateDialogue)(void *dialogueManager, Character *source, Character *target, int dialogueType, int dialogueSubtype, bool interruptDialogue, void *combatController); // a1 is unused
extern RelocAddr<_UpdateDialogue> UpdateDialogue;
