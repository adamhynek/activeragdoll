#pragma once

#include "RE/offsets.h"
#include "RE/misc.h"


RelocPtr<float> g_havokWorldScale(0x15B78F4);
RelocPtr<float> g_inverseHavokWorldScale(0x15ADFE8);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
RelocPtr<bhkSimpleShapePhantom *> g_pickSphere(0x3000958);

RelocPtr<CrosshairPickData *> g_pickData(0x2FC60C0);

RelocPtr<float> g_deltaTime(0x1EC8278);

RelocPtr<float> g_globalTimeMultiplier(0x1EC5698);

RelocPtr<float> fMaxTime(0x1EC82B0);
RelocPtr<float> fMaxTimeComplex(0x1EC8448);

RelocPtr<float> g_secondsSinceLastFrame_WorldTime_CheckPaused(0x2FEB794); // like the one below, but is 0 if in menu mode (paused)
RelocPtr<float> g_secondsSinceLastFrame_WorldTime(0x30C3A08); // is multiplied by timeMultiplier
RelocPtr<float> g_secondsSinceLastFrame_Unmultiplied(0x30C3A0C); // is not multiplied by timeMultiplier

RelocPtr<float> g_fMinSoundVel(0x1E94F78);

RelocPtr<int> g_currentFrameCounter(0x3186C5C);
RelocPtr<int> g_sceneComplexCounter(0x2FEB76C);
RelocPtr<int> g_iShadowUpdateFrameDelay(0x1ED4130);
RelocPtr<int> g_nextShadowUpdateFrameCount(0x3485798);

RelocPtr<BSAudioManager *> g_audioManager(0x30C1D30);

RelocPtr<ShadowSceneNode *> g_shadowSceneNode(0x3423080);

RelocPtr<TESObjectWEAP *> g_unarmedWeapon(0x2FC4758);

RelocPtr<float> g_minSoundVel(0x1E94F78); // it's an ini setting

RelocPtr<float> g_fMeleeWeaponHavokScale(0x1EAD900); // it's an ini setting

RelocPtr<float> g_fMagicHandTranslateX(0x1EAEA58);
RelocPtr<float> g_fMagicHandTranslateY(0x1EAEA70);
RelocPtr<float> g_fMagicHandTranslateZ(0x1EAEA88);
RelocPtr<float> g_fMagicHandRotateX(0x1EAEAA0);
RelocPtr<float> g_fMagicHandRotateY(0x1EAEAB8);
RelocPtr<float> g_fMagicHandRotateZ(0x1EAEAD0);
RelocPtr<float> g_fMagicHandScale(0x1EAEAE8);

RelocPtr<float> g_fPhysicsDamage1Mass(0x1E9D610);
RelocPtr<float> g_fPhysicsDamage2Mass(0x1E9D628);
RelocPtr<float> g_fPhysicsDamage3Mass(0x1E9D640);
RelocPtr<float> g_fPhysicsDamage1Damage(0x1E9D658);
RelocPtr<float> g_fPhysicsDamage2Damage(0x1E9D670);
RelocPtr<float> g_fPhysicsDamage3Damage(0x1E9D688);
RelocPtr<float> g_fPhysicsDamageSpeedBase(0x1E9D6A0);
RelocPtr<float> g_fPhysicsDamageSpeedMult(0x1E9D6B8);
RelocPtr<float> g_fPhysicsDamageSpeedMin(0x1E9D6D0);

RelocPtr<float> g_fMeleeLinearVelocityThreshold(0x1EAE500);
RelocPtr<float> g_fShieldLinearVelocityThreshold(0x1EAE5A8);

RelocPtr<DWORD> g_dwTlsIndex(0x30A8C04);

RelocAddr<void *> PlayerCharacter_vtbl(0x16E2230);
RelocAddr<void *> hkCharControllerShape_vtbl(0x1838E78);
RelocAddr<void *> TESActionData_vtbl(0x15BF5D8);

RelocPtr<FOCollisionListener *> g_foCollisionListener(0x1F850F0);

RelocPtr<AIProcessManager *> g_aiProcessManager(0x01F831B0);
RelocPtr<float> g_bAlwaysDriveRagdoll(0x1EBE830);

RelocPtr<BGSDefaultObjectManager> g_defaultObjectManager(0x01F81D90);
RelocPtr<UInt32> g_playerHandle(0x2FEB9EC);

RelocPtr<CharacterCollisionHandler *> g_characterCollisionHandler(0x300A6A0);

RelocPtr<TES *> g_tes(0x2FEB6F8);
RelocPtr<BGSImpactManager *> g_impactManager(0x2FC52E0);

// Used by NiCloningProcess...
RelocPtr<UInt64> unk_141E703BC(0x1E703BC);
RelocPtr<UInt64> unk_141E703B8(0x1E703B8);


// Havok / Bethesda havok wrappers
RelocAddr<_hkpWorld_getCurrentTime> hkpWorld_getCurrentTime(0xAB74F0);
RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay(0x00AB5B20);
RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast(0x00AB5EC0);
RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations(0x00AB6AA0);
RelocAddr<_hkpWorld_GetClosestPoints> hkpWorld_GetClosestPoints(0xAB62D0);
RelocAddr<_hkpWorld_AddEntity> hkpWorld_AddEntity(0xAB0CB0);
RelocAddr<_hkpWorld_RemoveEntity> hkpWorld_RemoveEntity(0xAB0E50);
RelocAddr<_hkpWorld_addContactListener> hkpWorld_addContactListener(0xAB5580);
RelocAddr<_bhkWorld_addContactListener> bhkWorld_addContactListener(0xDA5C50);
RelocAddr<_hkpEntity_addContactListener> hkpEntity_addContactListener(0xAA6FE0);
RelocAddr<_hkpWorld_addIslandActivationListener> hkpWorld_addIslandActivationListener(0xAB5100);
RelocAddr<_hkpWorld_removeIslandActivationListener> hkpWorld_removeIslandActivationListener(0xAB5160);
RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity(0xAB3110);
RelocAddr<_bhkWorld_UpdateCollisionFilterOnWorldObject> bhkWorld_UpdateCollisionFilterOnWorldObject(0xDFFE50);
RelocAddr<_ContactListener_PreprocessContactPointEvent> ContactListener_PreprocessContactPointEvent(0xE41AB0); // Checks some shape key stuff and sets disabled on the contact point properties if it wants to
RelocAddr<_hkpSimpleContactConstraintUtil_calculateSeparatingVelocity> hkpSimpleContactConstraintUtil_calculateSeparatingVelocity(0xAAF250);
RelocAddr<_hkpEntity_activate> hkpEntity_activate(0xAA7130);
RelocAddr<_bhkRigidBody_setActivated> bhkRigidBody_setActivated(0xE085D0);
RelocAddr<_hkpEntity_setPositionAndRotation> hkpEntity_setPositionAndRotation(0xAA9030);
RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform(0xAA9060);
RelocAddr<_hkpEntity_getNumConstraints> hkpEntity_getNumConstraints(0xAA73B0);
RelocAddr<_hkpEntity_removeContactListener> hkpEntity_removeContactListener(0xAA7080);
RelocAddr<_hkpRigidBody_ctor> hkpRigidBody_ctor(0xAA89C0);
RelocAddr<_hkpRigidBodyCinfo_ctor> hkpRigidBodyCinfo_ctor(0xAC5FE0);
RelocAddr<_hkpBoxShape_ctor> hkpBoxShape_ctor(0xA93600);
RelocAddr<_hkpTriggerVolume_ctor> hkpTriggerVolume_ctor(0xAFFCE0);
RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrame(0xAF6DD0);
RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrameAsynchronously(0xAF7100);
RelocAddr<_hkpKeyFrameUtility_applySoftKeyFrame> hkpKeyFrameUtility_applySoftKeyFrame(0xAF6AF0);
RelocAddr<_hkpConstraintInstance_setPriority> hkpConstraintInstance_setPriority(0xAC05B0);
RelocAddr<_hkpMotion_approxTransformAt> hkpMotion_approxTransformAt(0xAAB6E0);
RelocAddr<_bhkCollisionFilter_CompareFilterInfos> bhkCollisionFilter_CompareFilterInfos(0xDD6A80);
RelocAddr<_hkpRigidBody_setMotionType> hkpRigidBody_setMotionType(0xAA9530);
RelocAddr<_bhkRigidBody_setMotionType> bhkRigidBody_setMotionType(0xE08040);
RelocAddr<_bhkRigidBody_MoveToPositionAndRotation> bhkRigidBody_MoveToPositionAndRotation(0xE09210);
RelocAddr<_bhkCollisionObject_SetNodeTransformsFromWorldTransform> bhkCollisionObject_SetNodeTransformsFromWorldTransform(0xE1ACB0);
RelocAddr<_bhkEntity_setPositionAndRotation> bhkEntity_setPositionAndRotation(0xE08350);
RelocAddr<_bhkWorldObject_UpdateCollisionFilter> bhkWorldObject_UpdateCollisionFilter(0xDF88D0);
RelocAddr<_bhkRigidBodyCinfo_ctor> bhkRigidBodyCinfo_ctor(0xE06110);
RelocAddr<_bhkRigidBody_ctor> bhkRigidBody_ctor(0x2AEC80);
RelocAddr<_bhkBoxShape_ctor> bhkBoxShape_ctor(0x2AEB70);
RelocAddr<_bhkShape_GetMaterialId> bhkShape_GetMaterialId(0xE30F40);
RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference(0xA01280);
RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference(0xA01340);

// More havok-related
RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell(0x276A90);
RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable(0xE01FE0);
RelocAddr<_GetRefFromCollidable> GetRefFromCollidable(0x003B4940);

RelocAddr<_CreateDetectionEvent> CreateDetectionEvent(0x656140);
RelocAddr<_ShadowSceneNode_UpdateNodeList> ShadowSceneNode_UpdateNodeList(0x12F89E0);
RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);
RelocAddr<_ObjectReference_SetActorCause> ObjectReference_SetActorCause(0x9D1830);
RelocAddr<_ObjectReference_Activate> ObjectReference_Activate(0x009CD750);
RelocAddr<_TESObjectREFR_Activate> TESObjectREFR_Activate(0x2A8300);
RelocAddr<_TESObjectREFR_SetScale> TESObjectREFR_SetScale(0x29E3E0);
RelocAddr<_TESObjectREFR_GetTransformIncorporatingScale> TESObjectREFR_GetTransformIncorporatingScale(0x2A5E30);
RelocAddr<_EffectShader_Play> EffectShader_Play(0x9BCAF0);
RelocAddr<_EffectShader_Stop> EffectShader_Stop(0x9BCC20);
RelocAddr<_VisualEffect_Play> VisualEffect_Play(0x9A4E00);
RelocAddr<_VisualEffect_Stop> VisualEffect_Stop(0x9A4F80);
RelocAddr<_Sound_Play> Sound_Play(0x9EF150);
RelocAddr<_BSExtraDataList_RemoveOwnership> BSExtraDataList_RemoveOwnership(0x1309A0);
RelocAddr<_BSExtraDataList_SetOwnerForm> BSExtraDataList_SetOwnerForm(0x11E0C0);
RelocAddr<_TESObjectREFR_SetActorOwner> TESObjectREFR_SetActorOwner(0x9D18C0);
RelocAddr<_NiAVObject_RecalculateWorldTransform> NiAVObject_RecalculateWorldTransform(0xCA7110);
RelocAddr<_ActivatePickRef> ActivatePickRef(0x6CBCE0);
RelocAddr<_TESObjectREFR_GetMass> TESObjectREFR_GetMass(0x9CED20);
RelocAddr<_NiAVObject_GetMass> NiAVObject_GetMass(0x3B5B50);
RelocAddr<_StartGrabObject> StartGrabObject(0x006CC000);
RelocAddr<_TESObjectREFR_SetPosition> TESObjectREFR_SetPosition(0x2A8010);
RelocAddr<_TESObjectREFR_SetRotation> TESObjectREFR_SetRotation(0x2A7C50);
RelocAddr<_NiAVObject_UpdateNode> NiAVObject_UpdateNode(0xC9BC10);
RelocAddr<_NiAVObject_GetOwner> NiAVObject_GetOwner(0x2A5CF0);
RelocAddr<_GetMaterialType> GetMaterialType(0x2D8B60);
RelocAddr<_BGSImpactDataSet_GetImpactData> BGSImpactDataSet_GetImpactData(0x2D4C00);
RelocAddr<_BSAudioManager_InitSoundData> BSAudioManager_InitSoundData(0xC29D20);
RelocAddr<_SoundData_SetPosition> SoundData_SetPosition(0xC287D0);
RelocAddr<_SoundData_SetNode> SoundData_SetNode(0xC289C0);
RelocAddr<_SoundData_Play> SoundData_Play(0xC283E0);
RelocAddr<_BSExtraList_GetCount> BSExtraList_GetCount(0x123D90);
RelocAddr<_TESObjectBOOK_LearnSpell> TESObjectBOOK_LearnSpell(0x23B240);
RelocAddr<_Actor_GetPickupPutdownSound> Actor_GetPickupPutdownSound(0x5D7F90);
RelocAddr<_NiMatrixToNiQuaternion> NiMatrixToNiQuaternion(0xCB4460);
RelocAddr<_NiMatrixFromForwardVector> NiMatrixFromForwardVector(0xC4C1E0);
RelocAddr<_NiMatrixToEulerImpl> NiMatrixToEulerImpl(0xC9AAA0);
RelocAddr<_EulerToNiMatrix> EulerToNiMatrix(0xC995A0);
RelocAddr<_UpdateClavicleToTransformHand> UpdateClavicleToTransformHand(0xC4C5A0);
RelocAddr<_NiSkinInstance_UpdateBoneMatrices> NiSkinInstance_UpdateBoneMatrices(0xDC7DC0);
RelocAddr<_NiObject_Clone> NiObject_Clone(0xC978E0);
RelocAddr<_PlayerCharacter_GetOffsetNodeForWeaponIndex> PlayerCharacter_GetOffsetNodeForWeaponIndex(0x6AF100);
RelocAddr<_BSFixedString_Copy> BSFixedString_Copy(0xC6DD50);
RelocAddr<_RefreshActivateButtonArt> RefreshActivateButtonArt(0x53EFE0);

// Used by NiCloningProcess...
RelocAddr<_CleanupCloneList> CleanupCloneList1(0x1C8CA0);
RelocAddr<_CleanupCloneList> CleanupCloneList2(0x1C8BE0);

//bhkWorld_Update(0xDFB460);

//BipedAnim_RemoveAllParts(0x1D6530);

// 1st arg: ptr to BipedAnim. 2nd arg: ptr to NiNode
//CreateArmorNode(0x1DB680);

RelocAddr<_hkRealTohkUFloat8> hkRealTohkUFloat8(0xA02B10);
RelocAddr<_bhkRefObject_ctor> bhkRefObject_ctor(0xE306A0);
RelocAddr<_hkMalleableConstraintCinfo_Func4> hkMalleableConstraintCinfo_Func4(0xE3DD20);
RelocAddr<_hkRagdollConstraintCinfo_Func4> hkRagdollConstraintCinfo_Func4(0xE64960);
RelocAddr<_hkMalleableConstraintCinfo_setWrappedConstraintData> hkMalleableConstraintCinfo_setWrappedConstraintData(0xE3DE70);
RelocAddr<_hkMalleableConstraintCinfo_setStrength> hkMalleableConstraintCinfo_setStrength(0xE3DE90);
RelocAddr<_Actor_IsInRagdollState> Actor_IsInRagdollState(0x5EBA50);
RelocAddr<_IAnimationGraphManagerHolder_SetAnimationVariableFloat> IAnimationGraphManagerHolder_SetAnimationVariableFloat(0x500990);
RelocAddr<_BSAnimationGraphManager_HasRagdollInterface> BSAnimationGraphManager_HasRagdollInterface(0x20B320);
RelocAddr<_BSAnimationGraphManager_AddRagdollToWorld> BSAnimationGraphManager_AddRagdollToWorld(0x5D1B50);
RelocAddr<_BSAnimationGraphManager_RemoveRagdollFromWorld> BSAnimationGraphManager_RemoveRagdollFromWorld(0x61A1E0);
RelocAddr<_BSAnimationGraphManager_DisableOrEnableSyncOnUpdate> BSAnimationGraphManager_DisableOrEnableSyncOnUpdate(0x61A2E0);
RelocAddr<_BSAnimationGraphManager_ResetRagdoll> BSAnimationGraphManager_ResetRagdoll(0x6973D0);
RelocAddr<_BSAnimationGraphManager_SetWorld> BSAnimationGraphManager_SetWorld(0x502670);
RelocAddr<_NiNode_AddOrRemoveMalleableConstraints> NiNode_AddOrRemoveMalleableConstraints(0xE09CA0);
RelocAddr<_BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints> BSAnimationGraphManager_SetRagdollConstraintsFromBhkConstraints(0x61A4E0);
RelocAddr<_hkbRagdollDriver_getRagdoll> hkbRagdollDriver_getRagdoll(0xA25860);
RelocAddr<_ConstraintToFixedConstraint> ConstraintToFixedConstraint(0xE63A20);
RelocAddr<_hkpConstraintInstance_setEnabled> hkpConstraintInstance_setEnabled(0xAC06A0);
RelocAddr<_hkpConstraintInstance_isEnabled> hkpConstraintInstance_isEnabled(0xAC06D0);
RelocAddr<_hkpConstraintInstance_ctor> hkpConstraintInstance_ctor(0xABFA90);
RelocAddr<_hkpRagdollConstraintData_ctor> hkpRagdollConstraintData_ctor(0xAC1290);
RelocAddr<_hkpCollisionCallbackUtil_requireCollisionCallbackUtil> hkpCollisionCallbackUtil_requireCollisionCallbackUtil(0xAB8700);
RelocAddr<_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil> hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(0xB00C30);
RelocAddr<_hkpWorld_findWorldExtension> hkpWorld_findWorldExtension(0xAB58F0);
RelocAddr<_ahkpCharacterProxy_setLinearVelocity> ahkpCharacterProxy_setLinearVelocity(0xAFA1F0);
RelocAddr<_ahkpCharacterRigidBody_setLinearVelocity> ahkpCharacterRigidBody_setLinearVelocity(0xAF5B20);
RelocAddr<_ahkpCharacterRigidBody_getLinearVelocity> ahkpCharacterRigidBody_getLinearVelocity(0xAF5BB0);
RelocAddr<_hkbBlendPoses> hkbBlendPoses(0xB4DD80);
RelocAddr<_hkConstraintCinfo_setConstraintData> hkConstraintCinfo_setConstraintData(0xE3E6F0);
RelocAddr<_hkpRagdollConstraintData_setInBodySpace> hkpRagdollConstraintData_setInBodySpace(0xAC16E0);
RelocAddr<_bhkRagdollConstraint_ctor> bhkRagdollConstraint_ctor(0xE4A6C0);
RelocAddr<_hkbBehaviorGraph_generate> hkbBehaviorGraph_generate(0xA2BD80);
RelocAddr<_BShkbAnimationGraph_UpdateAnimation> BShkbAnimationGraph_UpdateAnimation(0xB2A190);
RelocAddr<_hkaRagdollRigidBodyController_driveToPose> hkaRagdollRigidBodyController_driveToPose(0xB4CFF0);
RelocAddr<_hkbRagdollDriver_driveToPose> hkbRagdollDriver_driveToPose(0xA25B60);
RelocAddr<_hkbRagdollDriver_postPhysics> hkbRagdollDriver_postPhysics(0xA27730);
RelocAddr<_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld> hkbRagdollDriver_mapHighResPoseLocalToLowResPoseWorld(0xA280C0);
RelocAddr<_hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal> hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal(0xA27F00);
RelocAddr<_hkbCharacter_getPoseLocal> hkbCharacter_getPoseLocal(0xA368B0);
RelocAddr<_hkRotation_setFromQuat> hkRotation_setFromQuat(0xA02C30);
RelocAddr<_hkpConstraintUtils_loosenConstraintLimits> hkpConstraintUtils_loosenConstraintLimits(0xB09940);
RelocAddr<_hkpEaseConstraintsAction_ctor> hkpEaseConstraintsAction_ctor(0xB9F470);
RelocAddr<_hkpEaseConstraintsAction_loosenConstraints> hkpEaseConstraintsAction_loosenConstraints(0xB9F520);
RelocAddr<_hkpEaseConstraintsAction_restoreConstraints> hkpEaseConstraintsAction_restoreConstraints(0xB9F540);
RelocAddr<_hkpConstraintUtils_convertToPowered> hkpConstraintUtils_convertToPowered(0xB08CA0);
RelocAddr<_hkpWorld_addWorldPostSimulationListener> hkpWorld_addWorldPostSimulationListener(0xAB5280);
RelocAddr<_hkpWorld_removeWorldPostSimulationListener> hkpWorld_removeWorldPostSimulationListener(0xAB52E0);
RelocAddr<_hkpShapeShrinker_shrinkConvexVerticesShape> hkpShapeShrinker_shrinkConvexVerticesShape(0xE6E720);
RelocAddr<_hkpConvexVerticesShape_getOriginalVertices> hkpConvexVerticesShape_getOriginalVertices(0xB98F70);
RelocAddr<_hkpConvexVerticesShape_ctor> hkpConvexVerticesShape_ctor(0xE9F890);
RelocAddr<_hkpListShape_disableChild> hkpListShape_disableChild(0xA9C3F0);
RelocAddr<_hkpListShape_enableChild> hkpListShape_enableChild(0xA9C420);
RelocAddr<_hkpCharacterProxy_addCharacterProxyListener> hkpCharacterProxy_addCharacterProxyListener(0xAFA820);
RelocAddr<_hkpCharacterProxy_removeCharacterProxyListener> hkpCharacterProxy_removeCharacterProxyListener(0xAFA880);
RelocAddr<_hkpEntity_updateMovedBodyInfo> hkpEntity_updateMovedBodyInfo(0xAA9E10);
RelocAddr<_Actor_GetCollisionFilterInfo> Actor_GetCollisionFilterInfo(0x5F44A0);
RelocAddr<_Actor_GetBumped> Actor_GetBumped(0x5E4B70);
RelocAddr<_Actor_HasLargeMovementDelta> Actor_HasLargeMovementDelta(0x6116C0);
RelocAddr<_Actor_GetCurrentPackage> Actor_GetCurrentPackage(0x985060);
RelocAddr<_Actor_DoCombatSpellApply> Actor_DoCombatSpellApply(0x6311B0);
RelocAddr<_Actor_sub_140600400> Actor_sub_140600400(0x608C10);
RelocAddr<_GetHeadingFromVector> GetHeadingFromVector(0xC97030);
RelocAddr<_ActorProcess_ResetLipSync> ActorProcess_ResetLipSync(0x659BE0);
RelocAddr<_ActorProcess_ClearGreetTopic> ActorProcess_ClearGreetTopic(0x6579A0);
RelocAddr<_ActorProcess_ClearLookAt2> ActorProcess_ClearLookAt2(0x6623B0);
RelocAddr<_ActorProcess_SetLookAt1> ActorProcess_SetLookAt1(0x6622A0);
RelocAddr<_ActorProcess_PlayIdle> ActorProcess_PlayIdle(0x654490);
RelocAddr<_ActorProcess_SetBumpState> ActorProcess_SetBumpState(0x661A10);
RelocAddr<_ActorProcess_SetBumpDirection> ActorProcess_SetBumpDirection(0x664C00);
RelocAddr<_ActorProcess_ResetBumpWaitTimer> ActorProcess_ResetBumpWaitTimer(0x661A50);
RelocAddr<_ActorProcess_PushActorAway> ActorProcess_PushActorAway(0x686920);
RelocAddr<_MovementControllerNPC_Update> MovementControllerNPC_Update(0x716120);
RelocAddr<_Actor_KeepOffsetFromActor> Actor_KeepOffsetFromActor(0x60C1A0);
RelocAddr<_Actor_ClearKeepOffsetFromActor> Actor_ClearKeepOffsetFromActor(0x60C2D0);
RelocAddr<_MovementControllerNPC_SetKeepOffsetFromActor> MovementControllerNPC_SetKeepOffsetFromActor(0x716F40);
RelocAddr<_Actor_IsGhost> Actor_IsGhost(0x5DAAE0);
RelocAddr<_Actor_IsRunning> Actor_IsRunning(0x5D9770);
RelocAddr<_Character_CanHit> Character_CanHit(0x5EFC20);
RelocAddr<_PlayerCharacter_UpdateAndGetAttackData> PlayerCharacter_UpdateAndGetAttackData(0x6B9F30);
RelocAddr<_ActorProcess_IncrementAttackCounter> ActorProcess_IncrementAttackCounter(0x65A090);
RelocAddr<_ActorProcess_UnsetAttackData> ActorProcess_UnsetAttackData(0x664710);
RelocAddr<_TESObjectWEAP_GetSoundAmount> TESObjectWEAP_GetSoundAmount(0x246360);
RelocAddr<_Actor_SetActionValue> Actor_SetActionValue(0x602C00);
RelocAddr<_TESNPC_GetSoundAmount> TESNPC_GetSoundAmount(0x375AE0);
RelocAddr<_CombatController_SetLastAttackTimeToNow> CombatController_SetLastAttackTimeToNow(0x50DEC0);
RelocAddr<_Actor_RemoveMagicEffectsDueToAction> Actor_RemoveMagicEffectsDueToAction(0x63DA50);
RelocAddr<_ActorValueOwner_GetStaminaCostForAttackData> ActorValueOwner_GetStaminaCostForAttackData(0x3CE510);
RelocAddr<_Actor_GetActorValueRegenRate> Actor_GetActorValueRegenRate(0x6297C0);
RelocAddr<_ActorProcess_UpdateRegenDelay> ActorProcess_UpdateRegenDelay(0x663C80);
RelocAddr<_FlashHudMenuMeter> FlashHudMenuMeter(0x902F20);
RelocAddr<_PlayerControls_SendAction> PlayerControls_SendAction(0x72C380);
RelocAddr<_Character_HitTarget> Character_HitTarget(0x631AF0);
RelocAddr<_UpdateDialogue> UpdateDialogue(0x7940F0);
RelocAddr<_Actor_IsHostileToActor> Actor_IsHostileToActor(0x5F0560);
RelocAddr<_Actor_GetDetectionCalculatedValue> Actor_GetDetectionCalculatedValue(0x605190);
RelocAddr<_Actor_SendAssaultAlarm> Actor_SendAssaultAlarm(0x986530);
RelocAddr<_Actor_StopCombatAlarm> Actor_StopCombatAlarm(0x987A70);
RelocAddr<_Actor_IsTalking> Actor_IsTalking(0x5DA8F0);
RelocAddr<_Actor_EvaluatePackage> Actor_EvaluatePackage(0x5E3990);
RelocAddr<_BSTaskPool_QueueDestructibleDamageTask> BSTaskPool_QueueDestructibleDamageTask(0x5CB4F0);
RelocAddr<_TESForm_GetDestructibleObjectForm> TESForm_GetDestructibleObjectForm(0x193030);
RelocAddr<_PlayRumble> PlayRumble(0xC59440);
RelocAddr<_ActorProcess_GetCurrentlyEquippedWeapon> ActorProcess_GetCurrentlyEquippedWeapon(0x683850);
RelocAddr<_ActorProcess_TransitionFurnitureState> ActorProcess_TransitionFurnitureState(0x6878D0);
RelocAddr<_ActorProcess_SayTopicInfo> ActorProcess_SayTopicInfo(0x64DB70);
RelocAddr<_Actor_SetVehicle> Actor_SetVehicle(0x60E0C0);
RelocAddr<_Actor_IsBlocking> Actor_IsBlocking(0x611680);
RelocAddr<_HitData_ctor> HitData_ctor(0x76D000);
RelocAddr<_HitData_dtor> HitData_dtor(0x76D0F0);
RelocAddr<_HitData_populate> HitData_populate(0x76D400);
RelocAddr<_HitData_PopulateFromPhysicalHit> HitData_PopulateFromPhysicalHit(0x76DAF0);
RelocAddr<_ScriptEventSourceHolder_DispatchHitEvenFromHitData> ScriptEventSourceHolder_DispatchHitEvenFromHitData(0x6366A0);
RelocAddr<_CalculatePhysicsDamage> CalculatePhysicsDamage(0x3B5540);
RelocAddr<_Actor_GetHit> Actor_GetHit(0x62F270);
RelocAddr<_Actor_EndHavokHit> Actor_EndHavokHit(0x75B390);
RelocAddr<_hkpUnaryAction_ctor> hkpUnaryAction_ctor(0xAAC200);
RelocAddr<_hkpUnaryAction_setEntity> hkpUnaryAction_setEntity(0xAAC350);
RelocAddr<_hkpWorld_addAction> hkpWorld_addAction(0xAB1D40);
RelocAddr<_hkpWorld_removeAction> hkpWorld_removeAction(0xAB1F60);
RelocAddr<_hkbPoseLocalToPoseWorld> hkbPoseLocalToPoseWorld(0xB4D7C0);
RelocAddr<_CopyAndApplyScaleToPose> CopyAndApplyScaleToPose(0xA27BC0);
RelocAddr<_FOCollisionListener_TryApplyCollisionDamage> FOCollisionListener_TryApplyCollisionDamage(0x3AC280);
RelocAddr<_BSTaskPool_QueueRemoveCollisionFromWorld> BSTaskPool_QueueRemoveCollisionFromWorld(0x5CC120);
RelocAddr<_BSTaskPool_QueueAddHavok> BSTaskPool_QueueAddHavok(0x5CBF90);
RelocAddr<_GetRandomNumberInRange> GetRandomNumberInRange(0x196550);
RelocAddr<_CreatePackageByType> CreatePackageByType(0x444410);
RelocAddr<_PackageLocation_CTOR> PackageLocation_CTOR(0x450C80);
RelocAddr<_PackageLocation_SetNearReference> PackageLocation_SetNearReference(0x450FA0);
RelocAddr<_TESPackage_SetPackageLocation> TESPackage_SetPackageLocation(0x445510);
RelocAddr<_PackageTarget_CTOR> PackageTarget_CTOR(0x452E70);
RelocAddr<_TESPackage_SetPackageTarget> TESPackage_SetPackageTarget(0x4459B0);
RelocAddr<_PackageTarget_ResetValueByTargetType> PackageTarget_ResetValueByTargetType(0x4531E0);
RelocAddr<_PackageTarget_SetFromReference> PackageTarget_SetFromReference(0x453250);
RelocAddr<_TESPackage_sub_140439BE0> TESPackage_sub_140439BE0(0x449730);
RelocAddr<_TESPackage_CopyFlagsFromOtherPackage> TESPackage_CopyFlagsFromOtherPackage(0x4447E0);
RelocAddr<_VRMeleeData_ComputeAngularVelocities> VRMeleeData_ComputeAngularVelocities(0x6B45C0);
RelocAddr<_WeaponSwingHandler_Handle> WeaponRightSwingHandler_Handle(0x74B340);
RelocAddr<_WeaponSwingHandler_Handle> WeaponLeftSwingHandler_Handle(0x74B3D0);
RelocAddr<_sub_140654E10> sub_140654E10(0x654E10);
RelocAddr<_ActorProcess_TriggerDialogue> ActorProcess_TriggerDialogue(0x6580B0);
RelocAddr<_sub_140664870> sub_140664870(0x664870);
RelocAddr<_sub_1406EE920> sub_1406EE920(0x6EE920);
RelocAddr<_sub_1406EC5C0> sub_1406EC5C0(0x6EC5C0);
RelocAddr<_TES_GetLandMaterialId> TES_GetLandMaterialId(0x167720);
RelocAddr<_TESForm_GetBlockBashData> TESForm_GetBlockBashData(0x192040);
RelocAddr<_BGSImpactManager_PlayImpactSound> BGSImpactManager_PlayImpactSound(0x5A9FF0);
