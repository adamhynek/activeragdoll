#pragma once
#include <Physics/Collide/Shape/Compound/Collection/List/hkpListShape.h>
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>
#include <Physics/Dynamics/Action/hkpUnaryAction.h>
#include <Physics/Utilities/Constraint/Keyframe/hkpKeyFrameUtility.h>
#include <Physics/Utilities/Collide/TriggerVolume/hkpTriggerVolume.h>
#include <Physics/Utilities/Actions/EaseConstraints/hkpEaseConstraintsAction.h>
#include <Common\Base\Memory\Allocator\hkMemoryAllocator.h>
#include <Common\Base\Memory\Allocator\Thread\hkThreadMemory.h>
#include <Animation\Ragdoll\PoseMatching\hkaPoseMatchingUtility.h>

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"
#include "skse64/NiNodes.h"
#include "skse64/GameVR.h"
#include "skse64/NiGeometry.h"
#include "skse64/gamethreads.h"

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

extern RelocPtr<float> g_fMinSoundVel;

extern RelocPtr<int> g_currentFrameCounter;
extern RelocPtr<int> g_sceneComplexCounter;
extern RelocPtr<int> g_iShadowUpdateFrameDelay;
extern RelocPtr<int> g_nextShadowUpdateFrameCount;

struct BSAudioManager { /* TODO */ };
extern RelocPtr<BSAudioManager *> g_audioManager;

struct ShadowSceneNode : NiNode { /* TODO */ };
extern RelocPtr<ShadowSceneNode *> g_shadowSceneNode;

extern RelocPtr<TESObjectWEAP *> g_unarmedWeapon;

extern RelocPtr<float> g_minSoundVel;

extern RelocPtr<float> g_fMeleeWeaponHavokScale;

extern RelocPtr<float> g_fMagicHandTranslateX;
extern RelocPtr<float> g_fMagicHandTranslateY;
extern RelocPtr<float> g_fMagicHandTranslateZ;
extern RelocPtr<float> g_fMagicHandRotateX;
extern RelocPtr<float> g_fMagicHandRotateY;
extern RelocPtr<float> g_fMagicHandRotateZ;
extern RelocPtr<float> g_fMagicHandScale;

extern RelocPtr<float> g_fPhysicsDamage1Mass;
extern RelocPtr<float> g_fPhysicsDamage2Mass;
extern RelocPtr<float> g_fPhysicsDamage3Mass;
extern RelocPtr<float> g_fPhysicsDamage1Damage;
extern RelocPtr<float> g_fPhysicsDamage2Damage;
extern RelocPtr<float> g_fPhysicsDamage3Damage;
extern RelocPtr<float> g_fPhysicsDamageSpeedBase;
extern RelocPtr<float> g_fPhysicsDamageSpeedMult;
extern RelocPtr<float> g_fPhysicsDamageSpeedMin;

extern RelocPtr<float> g_fMeleeLinearVelocityThreshold;
extern RelocPtr<float> g_fShieldLinearVelocityThreshold;

extern RelocPtr<float> g_fExplosionKnockStateExplodeDownTime;

extern RelocPtr<float> g_fQuadrupedPitchMult;

extern RelocPtr<DWORD> g_dwTlsIndex;

extern RelocAddr<void *> PlayerCharacter_vtbl;
extern RelocAddr<void *> hkCharControllerShape_vtbl;
extern RelocAddr<void *> TESActionData_vtbl;

extern RelocPtr<FOCollisionListener *> g_foCollisionListener;

extern RelocPtr<AIProcessManager *> g_aiProcessManager;
extern RelocPtr<float> g_bAlwaysDriveRagdoll;

extern RelocPtr<BGSDefaultObjectManager> g_defaultObjectManager;

extern RelocPtr<UInt32> g_playerHandle;

extern RelocPtr<CharacterCollisionHandler *> g_characterCollisionHandler;

extern RelocPtr<TES *> g_tes;
extern RelocPtr<BGSImpactManager *> g_impactManager;

extern RelocPtr<hkThreadMemory> g_hkThreadMemory;


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

typedef hkBool (*_hkpWorld_RemoveEntity)(hkpWorld *world, hkBool *ret, hkpEntity* entity);
extern RelocAddr<_hkpWorld_RemoveEntity> hkpWorld_RemoveEntity;

typedef void(*_bhkWorld_AddEntity)(bhkWorld *world, hkpEntity *entity);
extern RelocAddr<_bhkWorld_AddEntity> bhkWorld_AddEntity;

typedef bool(*_bhkWorld_RemoveEntity)(bhkWorld *world, hkpEntity *entity);
extern RelocAddr<_bhkWorld_RemoveEntity> bhkWorld_RemoveEntity;

typedef void* (*_hkpWorld_addContactListener)(hkpWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_hkpWorld_addContactListener> hkpWorld_addContactListener;

typedef void* (*_hkpWorld_addIslandActivationListener)(hkpWorld *world, hkpIslandActivationListener* worldListener);
extern RelocAddr<_hkpWorld_addIslandActivationListener> hkpWorld_addIslandActivationListener;

typedef void* (*_hkpWorld_removeIslandActivationListener)(hkpWorld *world, hkpIslandActivationListener* worldListener);
extern RelocAddr<_hkpWorld_removeIslandActivationListener> hkpWorld_removeIslandActivationListener;

typedef void* (*_bhkWorld_addContactListener)(bhkWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_bhkWorld_addContactListener> bhkWorld_addContactListener;

typedef void(*_hkpWorld_UpdateCollisionFilterOnEntity)(hkpWorld *world, hkpEntity* entity, hkpUpdateCollisionFilterOnEntityMode updateMode, hkpUpdateCollectionFilterMode updateShapeCollectionFilter);
extern RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity;

typedef void(*_bhkWorld_UpdateCollisionFilterOnWorldObject)(bhkWorld *world, bhkWorldObject *worldObject);
extern RelocAddr<_bhkWorld_UpdateCollisionFilterOnWorldObject> bhkWorld_UpdateCollisionFilterOnWorldObject;

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

typedef void(*_bhkRigidBody_setMotionType)(bhkRigidBody *_this, UInt64 newState);
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

typedef UInt32(*_bhkShape_GetMaterialId)(bhkShape *_this, hkpShapeKey shapeKey);
extern RelocAddr<_bhkShape_GetMaterialId> bhkShape_GetMaterialId;

typedef void(*_hkReferencedObject_addReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference;

typedef void(*_hkReferencedObject_removeReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference;

typedef void(*_hkVector4_setTransformedPos)(hkVector4 &_this, const hkTransform &transform, const hkVector4 &pos);
extern RelocAddr<_hkVector4_setTransformedPos> hkVector4_setTransformedPos;

typedef void(*_hkVector4_setTransformedInversePos)(hkVector4 &_this, const hkTransform &transform, const hkVector4 &pos);
extern RelocAddr<_hkVector4_setTransformedInversePos> hkVector4_setTransformedInversePos;

typedef void(*_hkaPoseMatchingUtility_computeReferenceFrame)(hkaPoseMatchingUtility *_this, const hkQsTransform *animPoseModelSpace, const hkQsTransform *ragdollPoseWorldSpace, hkQsTransform &animWorldFromModel, hkQsTransform &ragdollWorldFromModel);
extern RelocAddr<_hkaPoseMatchingUtility_computeReferenceFrame> hkaPoseMatchingUtility_computeReferenceFrame;


// More havok-related
typedef bhkWorld * (*_GetHavokWorldFromCell)(TESObjectCELL *cell);
extern RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell;

typedef NiAVObject * (*_GetNodeFromCollidable)(const hkpCollidable * a_collidable);
extern RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable;

typedef TESObjectREFR * (*_GetRefFromCollidable)(const hkpCollidable * a_collidable);
extern RelocAddr<_GetRefFromCollidable> GetRefFromCollidable;

typedef void(*_NiNode_SetMotionTypeDownwards)(NiNode *node, UInt32 motionType, bool a3, bool a4, UInt32 a5);
extern RelocAddr<_NiNode_SetMotionTypeDownwards> NiNode_SetMotionTypeDownwards;


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

typedef void(*_TESObjectREFR_GetTransformIncorporatingScale)(TESObjectREFR* _this, NiTransform &out);
extern RelocAddr<_TESObjectREFR_GetTransformIncorporatingScale> TESObjectREFR_GetTransformIncorporatingScale;

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

typedef TESObjectREFR * (*_NiAVObject_GetOwner)(NiAVObject *_this);
extern RelocAddr<_NiAVObject_GetOwner> NiAVObject_GetOwner;

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

typedef NiMatrix33 * (*_NiMatrixFromForwardVector)(NiMatrix33 *matOut, NiPoint3 *forward, NiPoint3 *world);
extern RelocAddr<_NiMatrixFromForwardVector> NiMatrixFromForwardVector;

typedef NiMatrix33 *(*_NiMatrixToYawPitchRollImpl)(NiMatrix33 *mat, float *yaw, float *pitch, float *roll);
extern RelocAddr<_NiMatrixToYawPitchRollImpl> NiMatrixToYawPitchRollImpl;

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


typedef bool(*_hkRealTohkUFloat8)(hkUFloat8 &out, const hkReal &value);
extern RelocAddr<_hkRealTohkUFloat8> hkRealTohkUFloat8;

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

typedef bool(*_IAnimationGraphManagerHolder_SetAnimationVariableFloat)(IAnimationGraphManagerHolder *_this, const BSFixedString &variableName, float value);
extern RelocAddr<_IAnimationGraphManagerHolder_SetAnimationVariableFloat> IAnimationGraphManagerHolder_SetAnimationVariableFloat;

typedef void(*_BSAnimationGraphManager_HasRagdoll)(BSAnimationGraphManager *_this, bool *out);
extern RelocAddr<_BSAnimationGraphManager_HasRagdoll> BSAnimationGraphManager_HasRagdoll;

typedef void(*_BSAnimationGraphManager_AddRagdollToWorld)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_AddRagdollToWorld> BSAnimationGraphManager_AddRagdollToWorld;

typedef void(*_BSAnimationGraphManager_RemoveRagdollFromWorld)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_RemoveRagdollFromWorld> BSAnimationGraphManager_RemoveRagdollFromWorld;

typedef void(*_BSAnimationGraphManager_DisableOrEnableSyncOnUpdate)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_DisableOrEnableSyncOnUpdate> BSAnimationGraphManager_DisableOrEnableSyncOnUpdate;

typedef void(*_BSAnimationGraphManager_ResetRagdoll)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_ResetRagdoll> BSAnimationGraphManager_ResetRagdoll;

typedef void(*_BSAnimationGraphManager_SetWorld)(BSAnimationGraphManager *_this, UInt64 *a1);
extern RelocAddr<_BSAnimationGraphManager_SetWorld> BSAnimationGraphManager_SetWorld;

typedef void(*_NiNode_AddOrRemoveMalleableConstraints)(NiNode *_this, bool a1, bool a2, bool a3);
extern RelocAddr<_NiNode_AddOrRemoveMalleableConstraints> NiNode_AddOrRemoveMalleableConstraints;

typedef void(*_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints)(BSAnimationGraphManager *_this, bool *a1);
extern RelocAddr<_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints> BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints;

typedef hkaRagdollInstance * (*_hkbRagdollDriver_getRagdoll)(hkbRagdollDriver *_this);
extern RelocAddr<_hkbRagdollDriver_getRagdoll> hkbRagdollDriver_getRagdoll;

typedef void(*_hkbRagdollDriver_reset)(hkbRagdollDriver *_this);
extern RelocAddr<_hkbRagdollDriver_reset> hkbRagdollDriver_reset;

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

typedef bool(*_hkbBehaviorGraph_generate)(hkbBehaviorGraph *_this, const hkbContext& context, hkbGeneratorOutput& output, bool setCharacterPose, hkReal timeOffset, bool doUpdateActiveNodesFirst);
extern RelocAddr<_hkbBehaviorGraph_generate> hkbBehaviorGraph_generate;

typedef bool(*_BShkbAnimationGraph_UpdateAnimation)(BShkbAnimationGraph *_this, BShkbAnimationGraph::UpdateData *updateData, void *a3);
extern RelocAddr<_BShkbAnimationGraph_UpdateAnimation> BShkbAnimationGraph_UpdateAnimation;

typedef bool(*_hkaRagdollRigidBodyController_driveToPose)(hkaRagdollRigidBodyController *_this, hkReal deltaTime, const hkQsTransform* poseLocalSpace, const hkQsTransform& worldFromModel, hkaKeyFrameHierarchyUtility::Output* stressOut);
extern RelocAddr<_hkaRagdollRigidBodyController_driveToPose> hkaRagdollRigidBodyController_driveToPose;

typedef bool(*_hkaRagdollRigidBodyController_ctor)(hkaRagdollRigidBodyController *_this);
extern RelocAddr<_hkaRagdollRigidBodyController_ctor> hkaRagdollRigidBodyController_ctor;

typedef bool(*_hkaRagdollRigidBodyController_dtor)(hkaRagdollRigidBodyController *_this);
extern RelocAddr<_hkaRagdollRigidBodyController_dtor> hkaRagdollRigidBodyController_dtor;

typedef bool(*_hkaRagdollRigidBodyController_reinitialize)(hkaRagdollRigidBodyController *_this);
extern RelocAddr<_hkaRagdollRigidBodyController_reinitialize> hkaRagdollRigidBodyController_reinitialize;

typedef bool(*_hkbRagdollDriver_driveToPose)(hkbRagdollDriver *_this, hkReal deltaTime, const hkbContext& context, hkbGeneratorOutput& generatorOutput);
extern RelocAddr<_hkbRagdollDriver_driveToPose> hkbRagdollDriver_driveToPose;

typedef bool(*_hkbRagdollDriver_postPhysics)(hkbRagdollDriver *_this, const hkbContext& context, hkbGeneratorOutput& inOut);
extern RelocAddr<_hkbRagdollDriver_postPhysics> hkbRagdollDriver_postPhysics;

typedef void(*_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld)(hkbRagdollDriver *_this, const hkQsTransform* highResPoseLocal, const hkQsTransform& worldFromModel, hkQsTransform* lowResPoseWorld);
extern RelocAddr<_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld> hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld;

typedef void(*_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal)(hkbRagdollDriver *_this, const hkQsTransform* highResPoseLocal, hkQsTransform* lowResPoseLocal);
extern RelocAddr<_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal> hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal;

typedef hkQsTransform * (*_hkbCharacter_getPoseLocal)(hkbCharacter *_this);
extern RelocAddr<_hkbCharacter_getPoseLocal> hkbCharacter_getPoseLocal;

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

typedef void(*_hkpWorld_addEntityListener)(hkpWorld *_this, hkpEntityListener *entityListener);
extern RelocAddr<_hkpWorld_addEntityListener> hkpWorld_addEntityListener;

typedef void(*_hkpWorld_removeEntityListener)(hkpWorld *_this, hkpEntityListener *entityListener);
extern RelocAddr<_hkpWorld_removeEntityListener> hkpWorld_removeEntityListener;

typedef void(*_hkpShapeShrinker_shrinkConvexVerticesShape)(hkpConvexVerticesShape* convexShape, hkReal maximumConvexRadius, hkReal relShrinkRadius, hkReal allowedDisplacement, const char* shapeName, hkBool optimize); // shapeName default null, optimize default true
extern RelocAddr<_hkpShapeShrinker_shrinkConvexVerticesShape> hkpShapeShrinker_shrinkConvexVerticesShape;

typedef void(*_hkpConvexVerticesShape_getOriginalVertices)(hkpConvexVerticesShape* _this, hkArray<hkVector4>& vertices);
extern RelocAddr<_hkpConvexVerticesShape_getOriginalVertices> hkpConvexVerticesShape_getOriginalVertices;

typedef void(*_hkpConvexVerticesShape_ctor)(hkpConvexVerticesShape* _this, const hkStridedVertices& vertices, const hkpConvexVerticesShape::BuildConfig& config);
extern RelocAddr<_hkpConvexVerticesShape_ctor> hkpConvexVerticesShape_ctor;

typedef void(*_hkpListShape_disableChild)(hkpListShape *_this, hkpShapeKey index);
extern RelocAddr<_hkpListShape_disableChild> hkpListShape_disableChild;

typedef void(*_hkpListShape_enableChild)(hkpListShape *_this, hkpShapeKey index);
extern RelocAddr<_hkpListShape_enableChild> hkpListShape_enableChild;

typedef void(*_hkpCharacterProxy_addCharacterProxyListener)(hkpCharacterProxy *_this, hkpCharacterProxyListener* listener);
extern RelocAddr<_hkpCharacterProxy_addCharacterProxyListener> hkpCharacterProxy_addCharacterProxyListener;

typedef void(*_hkpCharacterProxy_removeCharacterProxyListener)(hkpCharacterProxy *_this, hkpCharacterProxyListener* listener);
extern RelocAddr<_hkpCharacterProxy_removeCharacterProxyListener> hkpCharacterProxy_removeCharacterProxyListener;

typedef void(*_hkpEntity_updateMovedBodyInfo)(hkpEntity *_this);
extern RelocAddr<_hkpEntity_updateMovedBodyInfo> hkpEntity_updateMovedBodyInfo;

typedef UInt32 * (*_Actor_GetCollisionFilterInfo)(Actor *_this, UInt32 &filterInfoOut);
extern RelocAddr<_Actor_GetCollisionFilterInfo> Actor_GetCollisionFilterInfo;

typedef void(*_Actor_GetBumped)(Actor *_this, Actor *bumper, bool isLargeBump, bool exitFurniture);
extern RelocAddr<_Actor_GetBumped> Actor_GetBumped;

typedef bool(*_Actor_HasLargeMovementDelta)(Actor *_this);
extern RelocAddr<_Actor_HasLargeMovementDelta> Actor_HasLargeMovementDelta;

typedef TESPackage * (*_Actor_GetCurrentPackage)(UInt64 a1, UInt64 a2, Actor *_this);
extern RelocAddr<_Actor_GetCurrentPackage> Actor_GetCurrentPackage;

typedef TESPackage* (*_Actor_DoCombatSpellApply)(Actor *_this, SpellItem *spell, TESObjectREFR *target);
extern RelocAddr<_Actor_DoCombatSpellApply> Actor_DoCombatSpellApply;

typedef void(*_Actor_sub_140600400)(Actor *_this, float a2);
extern RelocAddr<_Actor_sub_140600400> Actor_sub_140600400;

typedef float(*_GetHeadingFromVector)(const NiPoint3 &vector);
extern RelocAddr<_GetHeadingFromVector> GetHeadingFromVector;

typedef void(*_ActorProcess_ResetLipSync)(ActorProcessManager *_this, Actor *actor);
extern RelocAddr<_ActorProcess_ResetLipSync> ActorProcess_ResetLipSync;

typedef void(*_ActorProcess_ClearGreetTopic)(ActorProcessManager *_this);
extern RelocAddr<_ActorProcess_ClearGreetTopic> ActorProcess_ClearGreetTopic;

typedef void(*_ActorProcess_ClearLookAt2)(ActorProcessManager *_this, bool dontValidateTarget);
extern RelocAddr<_ActorProcess_ClearLookAt2> ActorProcess_ClearLookAt2;

typedef void(*_ActorProcess_SetLookAt1)(ActorProcessManager *_this, Actor *actor);
extern RelocAddr<_ActorProcess_SetLookAt1> ActorProcess_SetLookAt1;

typedef void(*_ActorProcess_PlayIdle)(ActorProcessManager *_this, Actor *source, int defaultObject, Actor *target, bool a5, bool a6, TESIdleForm *idle);
extern RelocAddr<_ActorProcess_PlayIdle> ActorProcess_PlayIdle;

typedef void(*_ActorProcess_SetBumpState)(ActorProcessManager *_this, UInt32 bumpState);
extern RelocAddr<_ActorProcess_SetBumpState> ActorProcess_SetBumpState;

typedef void(*_ActorProcess_SetBumpDirection)(ActorProcessManager *_this, float direction);
extern RelocAddr<_ActorProcess_SetBumpDirection> ActorProcess_SetBumpDirection;

typedef void(*_ActorProcess_ResetBumpWaitTimer)(ActorProcessManager *_this);
extern RelocAddr<_ActorProcess_ResetBumpWaitTimer> ActorProcess_ResetBumpWaitTimer;

typedef void(*_ActorProcess_PushActorAway)(ActorProcessManager *_this, Actor *actor, NiPoint3 &from, float force);
extern RelocAddr<_ActorProcess_PushActorAway> ActorProcess_PushActorAway;

typedef void(*_MovementControllerNPC_Update)(MovementControllerNPC *_this);
extern RelocAddr<_MovementControllerNPC_Update> MovementControllerNPC_Update;

typedef void(*_Actor_KeepOffsetFromActor)(Actor *_this, UInt32 &targetHandle, NiPoint3 &offset, NiPoint3 &offsetAngleEulerRadians, float catchUpRadius, float followRadius);
extern RelocAddr<_Actor_KeepOffsetFromActor> Actor_KeepOffsetFromActor;

typedef void(*_Actor_ClearKeepOffsetFromActor)(Actor *_this);
extern RelocAddr<_Actor_ClearKeepOffsetFromActor> Actor_ClearKeepOffsetFromActor;

typedef void(*_MovementControllerNPC_SetKeepOffsetFromActor)(MovementControllerNPC *_this, bool keepOffset);
extern RelocAddr<_MovementControllerNPC_SetKeepOffsetFromActor> MovementControllerNPC_SetKeepOffsetFromActor;

typedef bool(*_Actor_IsGhost)(Actor *_this);
extern RelocAddr<_Actor_IsGhost> Actor_IsGhost;

typedef bool(*_Actor_IsRunning)(Actor *_this);
extern RelocAddr<_Actor_IsRunning> Actor_IsRunning;

typedef bool(*_Character_CanHit)(Character *_this, Actor *target);
extern RelocAddr<_Character_CanHit> Character_CanHit;

typedef void(*_PlayerCharacter_UpdateAndGetAttackData)(PlayerCharacter *_this, bool isLeft, bool isOffhand, bool isPowerAttack, BGSAttackData **attackDataOut);
extern RelocAddr<_PlayerCharacter_UpdateAndGetAttackData> PlayerCharacter_UpdateAndGetAttackData;

typedef bool(*_ActorProcess_IncrementAttackCounter)(ActorProcessManager *_this, int incCount);
extern RelocAddr<_ActorProcess_IncrementAttackCounter> ActorProcess_IncrementAttackCounter;

typedef bool(*_ActorProcess_UnsetAttackData)(ActorProcessManager* _this);
extern RelocAddr<_ActorProcess_UnsetAttackData> ActorProcess_UnsetAttackData;

typedef int(*_TESObjectWEAP_GetSoundAmount)(TESObjectWEAP *_this);
extern RelocAddr<_TESObjectWEAP_GetSoundAmount> TESObjectWEAP_GetSoundAmount;

typedef void(*_Actor_SetActionValue)(Actor *_this, int actionValue);
extern RelocAddr<_Actor_SetActionValue> Actor_SetActionValue;

typedef int(*_TESNPC_GetSoundAmount)(TESNPC *_this);
extern RelocAddr<_TESNPC_GetSoundAmount> TESNPC_GetSoundAmount;

typedef void(*_CombatController_SetLastAttackTimeToNow)(void *_this);
extern RelocAddr<_CombatController_SetLastAttackTimeToNow> CombatController_SetLastAttackTimeToNow;

typedef void(*_Actor_RemoveMagicEffectsDueToAction)(Actor *_this, int action);
extern RelocAddr<_Actor_RemoveMagicEffectsDueToAction> Actor_RemoveMagicEffectsDueToAction;

typedef float(*_ActorValueOwner_GetStaminaCostForAttackData)(ActorValueOwner *_this, BGSAttackData *attackData);
extern RelocAddr<_ActorValueOwner_GetStaminaCostForAttackData> ActorValueOwner_GetStaminaCostForAttackData;

typedef float(*_Actor_GetActorValueRegenRate)(Actor *_this, UInt32 actorValue);
extern RelocAddr<_Actor_GetActorValueRegenRate> Actor_GetActorValueRegenRate;

typedef void(*_ActorProcess_UpdateRegenDelay)(ActorProcessManager *_this, UInt32 actorValue, float delay);
extern RelocAddr<_ActorProcess_UpdateRegenDelay> ActorProcess_UpdateRegenDelay;

typedef void(*_FlashHudMenuMeter)(UInt32 actorValue);
extern RelocAddr<_FlashHudMenuMeter> FlashHudMenuMeter;

typedef bool(*_PlayerControls_IsTriggerHeldMainHand)(PlayerControls *_this);
extern RelocAddr<_PlayerControls_IsTriggerHeldMainHand> PlayerControls_IsTriggerHeldMainHand;

typedef bool(*_PlayerControls_IsTriggerHeldOffHand)(PlayerControls *_this);
extern RelocAddr<_PlayerControls_IsTriggerHeldOffHand> PlayerControls_IsTriggerHeldOffHand;

typedef float(*_PlayerControls_SendAction)(PlayerControls *_this, UInt32 defaultActionObject, UInt32 priority);
extern RelocAddr<_PlayerControls_SendAction> PlayerControls_SendAction;

typedef void(*_Character_HitTarget)(Character *_this, Actor *target, Projectile *projectile, bool isOffhand);
extern RelocAddr<_Character_HitTarget> Character_HitTarget;

typedef void(*_UpdateDialogue)(void *dialogueManager, Character *source, Character *target, int dialogueType, int dialogueSubtype, bool interruptDialogue, void *combatController); // a1 is unused
extern RelocAddr<_UpdateDialogue> UpdateDialogue;

typedef void(*_Actor_TriggerMiscDialogue)(Actor *_this, int dialogueSubtype, bool interruptDialogue);
extern RelocAddr<_Actor_TriggerMiscDialogue> Actor_TriggerMiscDialogue;

typedef bool(*_Actor_IsHostileToActor)(Actor *_this, Actor *actor);
extern RelocAddr<_Actor_IsHostileToActor> Actor_IsHostileToActor;

typedef int(*_Actor_GetDetectionCalculatedValue)(Actor *_this, Actor *actor, int a3);
extern RelocAddr<_Actor_GetDetectionCalculatedValue> Actor_GetDetectionCalculatedValue;

typedef void(*_Actor_SendAssaultAlarm)(UInt64 a1, UInt64 a2, Actor *actor); // a1, a2 unused
extern RelocAddr<_Actor_SendAssaultAlarm> Actor_SendAssaultAlarm;

typedef void(*_Actor_StopCombatAlarm)(UInt64 a1, UInt64 a2, Actor *actor); // a1, a2 unused
extern RelocAddr<_Actor_StopCombatAlarm> Actor_StopCombatAlarm;

typedef bool(*_Actor_IsTalking)(Actor *_this);
extern RelocAddr<_Actor_IsTalking> Actor_IsTalking;

typedef bool(*_Actor_EvaluatePackage)(Actor* _this, bool a2, bool resetAI);
extern RelocAddr<_Actor_EvaluatePackage> Actor_EvaluatePackage;

typedef void(*_BSTaskPool_QueueDestructibleDamageTask)(BSTaskPool *taskPool, TESObjectREFR *target, float damage);
extern RelocAddr<_BSTaskPool_QueueDestructibleDamageTask> BSTaskPool_QueueDestructibleDamageTask;

typedef BGSDestructibleObjectForm * (*_TESForm_GetDestructibleObjectForm)(TESForm *_this);
extern RelocAddr<_TESForm_GetDestructibleObjectForm> TESForm_GetDestructibleObjectForm;

typedef void(*_PlayRumble)(UInt32 isRight, float rumbleIntensity, float rumbleDuration);
extern RelocAddr<_PlayRumble> PlayRumble;

typedef InventoryEntryData * (*_ActorProcess_GetCurrentlyEquippedWeapon)(ActorProcessManager *_this, bool isOffhand);
extern RelocAddr<_ActorProcess_GetCurrentlyEquippedWeapon> ActorProcess_GetCurrentlyEquippedWeapon;

typedef void(*_ActorProcess_TransitionFurnitureState)(ActorProcessManager *_this, Actor *actor, UInt64 newState, UInt32 &furnitureHandle, UInt32 furnitureMarkerID);
extern RelocAddr<_ActorProcess_TransitionFurnitureState> ActorProcess_TransitionFurnitureState;

typedef void(*_ActorProcess_SayTopicInfo)(ActorProcessManager *_this, Actor *actor, TESTopic *topic, TESTopicInfo *topicInfo, bool a5, bool a6, bool a7, bool a8);
extern RelocAddr<_ActorProcess_SayTopicInfo> ActorProcess_SayTopicInfo;

typedef void(*_Actor_SetVehicle)(Actor *_this, UInt32 &handle);
extern RelocAddr<_Actor_SetVehicle> Actor_SetVehicle;

typedef bool(*_Actor_IsBlocking)(Actor *_this);
extern RelocAddr<_Actor_IsBlocking> Actor_IsBlocking;

typedef void(*_HitData_ctor)(HitData *_this);
extern RelocAddr<_HitData_ctor> HitData_ctor;

typedef void(*_HitData_dtor)(HitData *_this);
extern RelocAddr<_HitData_dtor> HitData_dtor;

typedef void(*_HitData_populate)(HitData *_this, Actor *src, Actor *target, InventoryEntryData *weapon, bool isOffhand);
extern RelocAddr<_HitData_populate> HitData_populate;

typedef void(*_HitData_PopulateFromPhysicalHit)(HitData *_this, Actor *src, Actor *target, float damage, bhkCharacterController::CollisionEvent &collisionEvent);
extern RelocAddr<_HitData_PopulateFromPhysicalHit> HitData_PopulateFromPhysicalHit;

typedef void(*_ScriptEventSourceHolder_DispatchHitEvenFromHitData)(EventDispatcherList *_this, Actor **hitTarget, Actor **hitSource, UInt32 sourceFormID, UInt32 projectileFormID, HitData *hitData);
extern RelocAddr<_ScriptEventSourceHolder_DispatchHitEvenFromHitData> ScriptEventSourceHolder_DispatchHitEvenFromHitData;

typedef float(*_CalculatePhysicsDamage)(float mass, float speed);
extern RelocAddr<_CalculatePhysicsDamage> CalculatePhysicsDamage;

typedef void(*_Actor_GetHit)(Actor *_this, HitData &hitData);
extern RelocAddr<_Actor_GetHit> Actor_GetHit;

typedef void(*_Actor_EndHavokHit)(HavokHitJobs *jobs, Actor *_this);
extern RelocAddr<_Actor_EndHavokHit> Actor_EndHavokHit;

typedef void(*_hkpUnaryAction_ctor)(hkpUnaryAction *_this, hkpEntity* entity, hkUlong userData);
extern RelocAddr<_hkpUnaryAction_ctor> hkpUnaryAction_ctor;

typedef void(*_hkpUnaryAction_setEntity)(hkpUnaryAction *_this, hkpEntity* entity);
extern RelocAddr<_hkpUnaryAction_setEntity> hkpUnaryAction_setEntity;

typedef void(*_hkpWorld_addAction)(hkpWorld *world, hkpAction* action);
extern RelocAddr<_hkpWorld_addAction> hkpWorld_addAction;

typedef void(*_hkpWorld_removeAction)(hkpWorld *world, hkpAction* action);
extern RelocAddr<_hkpWorld_removeAction> hkpWorld_removeAction;

typedef void(*_hkbPoseLocalToPoseWorld)(int numBones, const hkInt16 *parentIndices, const hkQsTransform &worldFromModel, hkQsTransform *highResPoseLocal, hkQsTransform *lowResPoseWorldOut);
extern RelocAddr<_hkbPoseLocalToPoseWorld> hkbPoseLocalToPoseWorld;

typedef void(*_CopyAndApplyScaleToPose)(bool scaleByHavokWorldScale, UInt32 numBones, hkQsTransform *poseLowResLocal, hkQsTransform *poseOut, float worldFromModelScale);
extern RelocAddr<_CopyAndApplyScaleToPose> CopyAndApplyScaleToPose;

typedef void(*_CopyAndPotentiallyApplyHavokScaleToTransform)(bool scaleByHavokWorldScale, const hkQsTransform *in, hkQsTransform *out);
extern RelocAddr<_CopyAndPotentiallyApplyHavokScaleToTransform> CopyAndPotentiallyApplyHavokScaleToTransform;

typedef void(*_FOCollisionListener_TryApplyCollisionDamage)(FOCollisionListener *_this, float separatingSpeed, hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, TESObjectREFR *refrA, TESObjectREFR *refrB);
extern RelocAddr<_FOCollisionListener_TryApplyCollisionDamage> FOCollisionListener_TryApplyCollisionDamage;

typedef void(*_BSTaskPool_QueueRemoveCollisionFromWorld)(BSTaskPool *_this, NiAVObject *root);
extern RelocAddr<_BSTaskPool_QueueRemoveCollisionFromWorld> BSTaskPool_QueueRemoveCollisionFromWorld;

typedef void(*_BSTaskPool_QueueAddHavok)(BSTaskPool *_this, NiAVObject *root, bhkWorld *world, UInt32 collisionGroup);
extern RelocAddr<_BSTaskPool_QueueAddHavok> BSTaskPool_QueueAddHavok;

typedef float(*_GetRandomNumberInRange)(float min, float max);
extern RelocAddr<_GetRandomNumberInRange> GetRandomNumberInRange;

typedef TESPackage * (*_CreatePackageByType)(int type);
extern RelocAddr<_CreatePackageByType> CreatePackageByType;

typedef void(*_PackageLocation_CTOR)(PackageLocation *_this);
extern RelocAddr<_PackageLocation_CTOR> PackageLocation_CTOR;

typedef void(*_PackageLocation_SetNearReference)(PackageLocation *_this, TESObjectREFR *refr);
extern RelocAddr<_PackageLocation_SetNearReference> PackageLocation_SetNearReference;

typedef void(*_TESPackage_SetPackageLocation)(TESPackage *_this, PackageLocation *packageLocation);
extern RelocAddr<_TESPackage_SetPackageLocation> TESPackage_SetPackageLocation;

typedef void(*_PackageTarget_CTOR)(PackageTarget *_this);
extern RelocAddr<_PackageTarget_CTOR> PackageTarget_CTOR;

typedef void(*_TESPackage_SetPackageTarget)(TESPackage *_this, PackageTarget *packageTarget);
extern RelocAddr<_TESPackage_SetPackageTarget> TESPackage_SetPackageTarget;

typedef void(*_PackageTarget_ResetValueByTargetType)(PackageTarget *_this, int a2);
extern RelocAddr<_PackageTarget_ResetValueByTargetType> PackageTarget_ResetValueByTargetType;

typedef void(*_PackageTarget_SetFromReference)(PackageTarget *_this, TESObjectREFR *refr);
extern RelocAddr<_PackageTarget_SetFromReference> PackageTarget_SetFromReference;

typedef void(*_TESPackage_sub_140439BE0)(TESPackage *_this, UInt64 a2);
extern RelocAddr<_TESPackage_sub_140439BE0> TESPackage_sub_140439BE0;

typedef void(*_TESPackage_CopyFlagsFromOtherPackage)(TESPackage *_this, TESPackage *other);
extern RelocAddr<_TESPackage_CopyFlagsFromOtherPackage> TESPackage_CopyFlagsFromOtherPackage;

typedef void(*_VRMeleeData_ComputeAngularVelocities)(VRMeleeData *_this, const NiPoint3 &hmdPos, float &outVelocityX, float &outVelocityY);
extern RelocAddr<_VRMeleeData_ComputeAngularVelocities> VRMeleeData_ComputeAngularVelocities;

typedef bool(*_WeaponSwingHandler_Handle)(void* _this, Actor* actor);
extern RelocAddr<_WeaponSwingHandler_Handle> WeaponRightSwingHandler_Handle;

typedef bool(*_WeaponSwingHandler_Handle)(void* _this, Actor* actor);
extern RelocAddr<_WeaponSwingHandler_Handle> WeaponLeftSwingHandler_Handle;

typedef void(*_sub_140654E10)(ActorProcessManager *_this, bool a2);
extern RelocAddr<_sub_140654E10> sub_140654E10;

typedef void(*_ActorProcess_TriggerDialogue)(ActorProcessManager *_this, Actor *actor, int dialogueType, int dialogueSubtype, Actor *target, UInt64 a6, bool a7, bool a8, bool a9, bool a10);
extern RelocAddr<_ActorProcess_TriggerDialogue> ActorProcess_TriggerDialogue;

typedef void(*_sub_140664870)(ActorProcessManager *_this, int a2);
extern RelocAddr<_sub_140664870> sub_140664870;

typedef void * (*_sub_1406EE920)(void);
extern RelocAddr<_sub_1406EE920> sub_1406EE920;

typedef UInt32(*_TES_GetLandMaterialId)(TES *_this, const NiPoint3 &position);
extern RelocAddr<_TES_GetLandMaterialId> TES_GetLandMaterialId;

typedef BGSBlockBashData * (*_TESForm_GetBlockBashData)(TESForm *_this);
extern RelocAddr<_TESForm_GetBlockBashData> TESForm_GetBlockBashData;

typedef bool(*_BGSImpactManager_PlayImpactSound)(BGSImpactManager *_this, ImpactSoundData &impactSoundData);
extern RelocAddr<_BGSImpactManager_PlayImpactSound> BGSImpactManager_PlayImpactSound;

typedef void(*_TESObjectCELL_PlaceParticleEffect_Impl)(TESObjectCELL *_this, float lifetime, const char *modelName, const NiMatrix33 &normal, const NiPoint3 &pos, float scale, UInt32 flags, NiAVObject *target);
extern RelocAddr<_TESObjectCELL_PlaceParticleEffect_Impl> TESObjectCELL_PlaceParticleEffect_Impl;

typedef void(*_TESObjectCELL_PlaceParticleEffect)(TESObjectCELL *_this, float lifetime, const char *modelName, const NiPoint3 &normal, const NiPoint3 &pos, float scale, UInt32 flags, NiAVObject *target);
extern RelocAddr<_TESObjectCELL_PlaceParticleEffect> TESObjectCELL_PlaceParticleEffect;

typedef void(*_DispatchTransformDeltaEvent)(void *sinks, NiTransform *a_delta);
extern RelocAddr<_DispatchTransformDeltaEvent> DispatchTransformDeltaEvent;

typedef void(*_BSAnimationGraphManager_GetWorldFromModel)(BSAnimationGraphManager *_this, NiTransform **worldFromModel);
extern RelocAddr<_BSAnimationGraphManager_GetWorldFromModel> BSAnimationGraphManager_GetWorldFromModel;

typedef bool(*_Actor_IsRagdollMovingSlowEnoughToGetUp)(Actor *_this);
extern RelocAddr<_Actor_IsRagdollMovingSlowEnoughToGetUp> Actor_IsRagdollMovingSlowEnoughToGetUp;


struct UnkSwingData
{
	TESObjectWEAP *weapon; // 00
	UInt64 unk08 = -2;
};

typedef void(*_sub_1406EC5C0)(void *a1, UnkSwingData *a2);
extern RelocAddr<_sub_1406EC5C0> sub_1406EC5C0;
