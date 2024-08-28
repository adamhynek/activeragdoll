#pragma once

#include "skse64/GameRTTI.h"

#include "RE/havok.h"
#include "RE/offsets.h"
#include "utils.h"
#include "math_utils.h"

hkMemoryRouter &hkGetMemoryRouter()
{
    return *(hkMemoryRouter *)(hkUlong)TlsGetValue(*g_havokMemoryRouterTlsIndex);
}

hkUFloat8 &hkUFloat8::operator=(const float &fv)
{
    hkRealTohkUFloat8(*this, fv);
    return *this;
}

namespace RE
{
    template <typename T>
    void hkArray<T>::pushBack(const T &t)
    {
        if (m_size == getCapacity()) {
            hkArrayUtil__reserveMore(g_hkContainerHeapAllocator, this, sizeof(T));
        }
        m_data[m_size] = t;
        ++m_size;
    }
    template void hkArray<const hkpShape *>::pushBack(const hkpShape *const &t);
}

bhkListShapeCinfo::~bhkListShapeCinfo()
{
    bhkListShapeCinfo_dtor(this);
}

hkConstraintCinfo::~hkConstraintCinfo()
{
    hkConstraintCinfo_setConstraintData(this, nullptr);
}

auto hkMalleableConstraintCinfo_vtbl = RelocAddr<void *>(0x182C5F8);
hkMalleableConstraintCinfo::hkMalleableConstraintCinfo()
{
    set_vtbl(this, hkMalleableConstraintCinfo_vtbl);
}

auto hkRagdollConstraintCinfo_vtbl = RelocAddr<void *>(0x1830F38);
hkRagdollConstraintCinfo::hkRagdollConstraintCinfo()
{
    set_vtbl(this, hkRagdollConstraintCinfo_vtbl);
}

auto bhkMalleableConstraint_vtbl = RelocAddr<void *>(0x182C628);
void bhkMalleableConstraint_ctor(bhkMalleableConstraint *_this, hkMalleableConstraintCinfo *cInfo)
{
    // Construct bhkMalleableConstraint
    bhkRefObject_ctor(_this);
    _this->cinfo = 0;
    *((void **)_this) = ((void *)(bhkMalleableConstraint_vtbl)); // set vtbl
    _this->InitHavokFromCinfo((void *)((UInt64)cInfo + 8)); // need to pass in cInfo + 8 for whatever reason. Within the function it subtracts 8 again...
}

bhkMalleableConstraint *CreateMalleableConstraint(bhkConstraint *constraint, float strength)
{
    if (DYNAMIC_CAST(constraint, bhkConstraint, bhkMalleableConstraint)) return nullptr; // already a malleable constraint

    hkMalleableConstraintCinfo cInfo;
    get_vfunc<_hkConstraintCinfo_CreateConstraintData>(&cInfo, 4)(&cInfo); // Creates constraintData
    hkMalleableConstraintCinfo_setWrappedConstraintData(&cInfo, constraint->constraint->m_data);
    cInfo.rigidBodyA = constraint->constraint->getRigidBodyA();
    cInfo.rigidBodyB = constraint->constraint->getRigidBodyB();
    hkMalleableConstraintCinfo_setStrength(&cInfo, strength);

    bhkMalleableConstraint *malleableConstraint = (bhkMalleableConstraint *)Heap_Allocate(sizeof(bhkMalleableConstraint));
    if (malleableConstraint) {
        bhkMalleableConstraint_ctor(malleableConstraint, &cInfo);
        return malleableConstraint;
    }

    return nullptr;
}

hkpConstraintInstance *LimitedHingeToRagdollConstraint(hkpConstraintInstance *constraint)
{
    // gather data from limited hinge constraint
    hkpLimitedHingeConstraintData *limitedHingeData = static_cast<hkpLimitedHingeConstraintData *>(const_cast<hkpConstraintData *>(constraint->getData()));
    hkVector4 &childPivot = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(3);
    hkVector4 &parentPivot = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(3);
    hkVector4 &childPlane = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(0);
    hkVector4 &parentPlane = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(0);

    // get childTwist axis and compute a parentTwist axis which closely matches the limited hinge's min/max limits.
    hkReal minAng = limitedHingeData->getMinAngularLimit();
    hkReal maxAng = limitedHingeData->getMaxAngularLimit();
    hkReal angExtents = (maxAng - minAng) / 2.0f;

    //hkQuaternion minAngRotation; minAngRotation.setAxisAngle(parentPlane, -angExtents + maxAng);
    hkQuaternion minAngRotation = NiQuatToHkQuat(MatrixToQuaternion(MatrixFromAxisAngle(HkVectorToNiPoint(parentPlane), -angExtents + maxAng)));
    minAngRotation.normalize();

    hkVector4 &childTwist = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(2);
    hkVector4 &parentLimitedAxis = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(2);
    hkVector4 parentTwist; parentTwist.setRotatedDir(minAngRotation, parentLimitedAxis);

    hkpRagdollConstraintData *ragdollData = (hkpRagdollConstraintData *)Heap_Allocate(sizeof(hkpRagdollConstraintData));
    hkpRagdollConstraintData_ctor(ragdollData);
    hkpRagdollConstraintData_setInBodySpace(ragdollData, childPivot, parentPivot, childPlane, parentPlane, childTwist, parentTwist);

    // adjust limits to make it like the hinge constraint
    ragdollData->setConeAngularLimit(angExtents);
    ragdollData->setTwistMinAngularLimit(0.0f);
    ragdollData->setTwistMaxAngularLimit(0.0f);
    ragdollData->m_atoms.m_angFriction.m_maxFrictionTorque = limitedHingeData->getMaxFrictionTorque();
    ragdollData->setAngularLimitsTauFactor(limitedHingeData->getAngularLimitsTauFactor());

    hkpConstraintInstance *newConstraint = (hkpConstraintInstance *)Heap_Allocate(sizeof(hkpConstraintInstance));
    hkpConstraintInstance_ctor(newConstraint, constraint->getEntityA(), constraint->getEntityB(), ragdollData, hkpConstraintInstance::ConstraintPriority::PRIORITY_PSI);
    hkReferencedObject_removeReference(ragdollData);

    return newConstraint;
}

void ConvertLimitedHingeDataToRagdollConstraintData(hkpRagdollConstraintData *ragdollData, hkpLimitedHingeConstraintData *limitedHingeData)
{
    // gather data from limited hinge constraint
    hkVector4 &childPivot = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(3);
    hkVector4 &parentPivot = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(3);
    hkVector4 &childPlane = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(0);
    hkVector4 &parentPlane = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(0);

    // get childTwist axis and compute a parentTwist axis which closely matches the limited hinge's min/max limits.
    hkReal minAng = limitedHingeData->getMinAngularLimit();
    hkReal maxAng = limitedHingeData->getMaxAngularLimit();
    hkReal angExtents = (maxAng - minAng) / 2.0f;

    //hkQuaternion minAngRotation; minAngRotation.setAxisAngle(parentPlane, -angExtents + maxAng);
    hkQuaternion minAngRotation = NiQuatToHkQuat(MatrixToQuaternion(MatrixFromAxisAngle(HkVectorToNiPoint(parentPlane), -angExtents + maxAng)));
    minAngRotation.normalize();

    hkVector4 &childTwist = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(2);
    hkVector4 &parentLimitedAxis = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(2);
    hkVector4 parentTwist; parentTwist.setRotatedDir(minAngRotation, parentLimitedAxis);

    hkpRagdollConstraintData_setInBodySpace(ragdollData, childPivot, parentPivot, childPlane, parentPlane, childTwist, parentTwist);

    // adjust limits to make it like the hinge constraint
    ragdollData->setConeAngularLimit(angExtents);
    ragdollData->setTwistMinAngularLimit(0.0f);
    ragdollData->setTwistMaxAngularLimit(0.0f);
    ragdollData->m_atoms.m_angFriction.m_maxFrictionTorque = limitedHingeData->getMaxFrictionTorque();
    ragdollData->setAngularLimitsTauFactor(limitedHingeData->getAngularLimitsTauFactor());
}

bhkRagdollConstraint *ConvertToRagdollConstraint(bhkConstraint *constraint)
{
    if (DYNAMIC_CAST(constraint, bhkConstraint, bhkRagdollConstraint)) return nullptr; // already a bhkRagdollConstraint

    hkRagdollConstraintCinfo cInfo;
    get_vfunc<_hkConstraintCinfo_CreateConstraintData>(&cInfo, 4)(&cInfo); // Creates constraintData and calls hkpRagdollConstraintData_ctor()
    cInfo.rigidBodyA = constraint->constraint->getRigidBodyA();
    cInfo.rigidBodyB = constraint->constraint->getRigidBodyB();
    ConvertLimitedHingeDataToRagdollConstraintData((hkpRagdollConstraintData *)cInfo.constraintData.val(), (hkpLimitedHingeConstraintData *)constraint->constraint->getData());

    bhkRagdollConstraint *ragdollConstraint = (bhkRagdollConstraint *)Heap_Allocate(sizeof(bhkRagdollConstraint));
    if (ragdollConstraint) {
        bhkRagdollConstraint_ctor(ragdollConstraint, &cInfo);
        return ragdollConstraint;
    }

    return nullptr;
}


NiPointer<bhkCharacterController> GetCharacterController(Actor *actor)
{
    ActorProcessManager *process = actor->processManager;
    if (!process) return nullptr;

    MiddleProcess *middleProcess = process->middleProcess;
    if (!middleProcess) return nullptr;

    return *((NiPointer<bhkCharacterController> *) & middleProcess->unk250);
}

NiPointer<bhkCharRigidBodyController> GetCharRigidBodyController(Actor *actor)
{
    NiPointer<bhkCharacterController> controller = GetCharacterController(actor);
    if (!controller) return nullptr;

    return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharRigidBodyController);
}

NiPointer<bhkCharProxyController> GetCharProxyController(Actor *actor)
{
    NiPointer<bhkCharacterController> controller = GetCharacterController(actor);
    if (!controller) return nullptr;

    return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharProxyController);
}


BShkbAnimationGraph *GetAnimationGraph(hkbCharacter *character)
{
    hkbBehaviorGraph *behaviorGraph = character->behaviorGraph;
    if (!behaviorGraph) return nullptr;

    BShkbAnimationGraph *graph = (BShkbAnimationGraph *)behaviorGraph->userData;
    return graph;
}

Actor *GetActorFromCharacter(hkbCharacter *character)
{
    BShkbAnimationGraph *graph = GetAnimationGraph(character);
    if (!graph) return nullptr;

    return graph->holder;
}

Actor *GetActorFromRagdollDriver(hkbRagdollDriver *driver)
{
    hkbCharacter *character = driver->character;
    if (!character) return nullptr;

    return GetActorFromCharacter(character);
}

void ReSyncLayerBitfields(bhkCollisionFilter *filter, UInt8 layer)
{
    UInt64 bitfield = filter->layerBitfields[layer];
    for (int i = 0; i < 64; i++) { // 56 layers in vanilla
        if ((bitfield >> i) & 1) {
            filter->layerBitfields[i] |= ((UInt64)1 << layer);
        }
        else {
            filter->layerBitfields[i] &= ~((UInt64)1 << layer);
        }
    }
}

void hkpWorld_removeContactListener(hkpWorld *_this, hkpContactListener *worldListener)
{
    hkArray<hkpContactListener *> &listeners = _this->m_contactListeners;

    for (int i = 0; i < listeners.getSize(); i++) {
        hkpContactListener *listener = listeners[i];
        if (listener == worldListener) {
            listeners[i] = nullptr;
            return;
        }
    }
}

bool hkpWorld_hasContactListener(hkpWorld *_this, hkpContactListener *listener)
{
    hkArray<hkpContactListener *> &listeners = _this->m_contactListeners;
    for (int i = 0; i < listeners.getSize(); i++) {
        if (listeners[i] == listener) {
            return true;
        }
    }
    return false;
}

bool hkpWorld_hasWorldPostSimulationListener(hkpWorld *_this, hkpWorldPostSimulationListener *listener)
{
    hkArray<hkpWorldPostSimulationListener *> &listeners = _this->m_worldPostSimulationListeners;
    for (int i = 0; i < listeners.getSize(); i++) {
        if (listeners[i] == listener) {
            return true;
        }
    }
    return false;
}

bool hkpWorld_hasEntityListener(hkpWorld *_this, hkpEntityListener *listener)
{
    hkArray<hkpEntityListener *> &listeners = _this->m_entityListeners;
    for (int i = 0; i < listeners.getSize(); i++) {
        if (listeners[i] == listener) {
            return true;
        }
    }
    return false;
}

int hkpCharacterProxy_findCharacterProxyListener(hkpCharacterProxy *_this, hkpCharacterProxyListener *proxyListener)
{
    hkArray<hkpCharacterProxyListener *> &listeners = _this->m_listeners;

    for (int i = 0; i < listeners.getSize(); i++) {
        hkpCharacterProxyListener *listener = listeners[i];
        if (listener == proxyListener) {
            return i;
        }
    }
    return -1;
}

float hkpContactPointEvent_getSeparatingVelocity(const hkpContactPointEvent &_this)
{
    if (_this.m_separatingVelocity) {
        return *_this.m_separatingVelocity;
    }
    else {
        return hkpSimpleContactConstraintUtil_calculateSeparatingVelocity(_this.m_bodies[0], _this.m_bodies[1], _this.m_bodies[0]->getCenterOfMassInWorld(), _this.m_bodies[1]->getCenterOfMassInWorld(), _this.m_contactPoint);
    }
}

void hkpRagdollConstraintData_setPivotInWorldSpace(hkpRagdollConstraintData *constraint, const hkTransform &bodyATransform, const hkTransform &bodyBTransform, const hkVector4 &pivot)
{
    hkVector4_setTransformedInversePos(constraint->m_atoms.m_transforms.m_transformA.m_translation, bodyATransform, pivot);
    hkVector4_setTransformedInversePos(constraint->m_atoms.m_transforms.m_transformB.m_translation, bodyBTransform, pivot);
}
//
//void __fastcall hkbPoseLocalToPoseWorld(int a_numBones, SInt16 *a_parentIndices, hkQsTransform *a_worldFromModel, hkQsTransform *a_poseLocal, hkQsTransform *a_poseWorldOut)
//{
//    __int64 parentIndex; // rax
//    __m128 parentRotation; // xmm5
//    hkVector4 v11; // xmm2
//    __m128 v12; // xmm4
//    float v14; // xmm0_4
//    __m128 v15; // xmm4
//    __m128 v18; // xmm2
//    __m128 v19; // xmm6
//    __m128 v20; // xmm3
//    __m128 v21; // xmm5
//    __m128 v22; // xmm1
//
//    for (int i = 0; i < a_numBones; i++)
//    {
//        parentIndex = a_parentIndices[i];
//        hkQsTransform *parentPose = parentIndex == -1 ? a_worldFromModel : &a_poseWorldOut[parentIndex];
//        parentRotation = parentPose->m_rotation;
//
//        {
//            v11 = a_poseLocal[i].m_translation;
//            v12 = _mm_mul_ps(parentRotation, v11);
//            __m128 qreal = _mm_shuffle_ps(parentRotation, parentRotation, 255); // w
//            v14 = _mm_shuffle_ps(v12, v12, 85).m128_f32_0[0] + v12.m128_f32_0[0]; // y
//            v15 = _mm_shuffle_ps(v12, v12, 170); // z
//            v15.m128_f32_0[0] = v15.m128_f32_0[0] + v14;
//
//            __m128 ret = _mm_add_ps(
//                _mm_mul_ps(
//                    _mm_sub_ps(
//                        _mm_mul_ps(_mm_shuffle_ps(parentRotation, parentRotation, 201), _mm_shuffle_ps(v11, v11, 210)), // yzxw zxyw
//                        _mm_mul_ps(_mm_shuffle_ps(parentRotation, parentRotation, 210), _mm_shuffle_ps(v11, v11, 201))), // zxyw yzxw
//                    qreal),
//                _mm_add_ps(
//                    _mm_mul_ps(_mm_add_ps(_mm_mul_ps(qreal, qreal), { -0.5f, -0.5f, -0.5f, -0.5f }), v11),
//                    _mm_mul_ps(_mm_shuffle_ps(v15, v15, 0), parentRotation))); // x
//            __m128 rotatedDir = _mm_add_ps(ret, ret);
//            a_poseWorldOut[i].m_translation = _mm_add_ps(rotatedDir, parentPose->m_translation);
//        }
//
//        v18 = _mm_shuffle_ps(a_poseLocal[i].m_rotation, a_poseLocal[i].m_rotation, 255); // w
//        v19 = _mm_mul_ps(a_poseLocal[i].m_rotation, parentRotation);
//        v20 = _mm_shuffle_ps(parentRotation, parentRotation, 255); // w
//        v21 = _mm_add_ps(
//            _mm_add_ps(
//                _mm_sub_ps(
//                    _mm_mul_ps(
//                        _mm_shuffle_ps(a_poseLocal[i].m_rotation, a_poseLocal[i].m_rotation, 210), // zxyw
//                        _mm_shuffle_ps(parentRotation, parentRotation, 201)), // yzxw
//                    _mm_mul_ps(
//                        _mm_shuffle_ps(a_poseLocal[i].m_rotation, a_poseLocal[i].m_rotation, 201), // yzxw
//                        _mm_shuffle_ps(parentRotation, parentRotation, 210))), // zxyw
//                _mm_mul_ps(a_poseLocal[i].m_rotation, _mm_shuffle_ps(v20, v20, 0))), // x
//            _mm_mul_ps(parentRotation, _mm_shuffle_ps(v18, v18, 0))); // x
//        v22 = _mm_shuffle_ps(v19, v19, 170); // z
//        v22.m128_f32_0[0] = v22.m128_f32_0[0] + (_mm_shuffle_ps(v19, v19, 85).m128_f32_0[0] + v19.m128_f32_0[0]); // y
//        a_poseWorldOut[i].m_rotation = _mm_shuffle_ps(
//            v21,
//            _mm_unpackhi_ps(v21, _mm_sub_ps(v19, _mm_shuffle_ps(v22, v22, 0))), // x
//            196); // xyxz
//
//        a_poseWorldOut[i].m_scale = _mm_mul_ps(a_poseLocal[i].m_scale, parentPose->m_scale);
//    }
//}


void MapHighResPoseLocalToLowResPoseWorld(hkbRagdollDriver *driver, const hkQsTransform &worldFromModel, const hkQsTransform *highResPoseLocal, hkQsTransform *lowResPoseWorldOut)
{
    // We need this because hkbRagdollDriver::mapHighResPoseLocalToLowResPoseWorld() does not actually give correct results.
    // This is essentially what hkbRagdollDriver::driveToPose() does when computing what transforms to drive the rigidbodies to.

    int numPosesLow = driver->ragdoll->getNumBones();

    hkStackArray<hkQsTransform> lowResPoseLocal(numPosesLow);

    //const_cast<hkQsTransform *>(highResPoseLocal)[1].m_translation = NiPointToHkVector(HkVectorToNiPoint(highResPoseLocal[1].m_translation) + NiPoint3(0.f, -4.3f, -11.6f)); // TODO: Remove this hack
    //const_cast<hkQsTransform *>(highResPoseLocal)[2].m_translation = NiPointToHkVector(HkVectorToNiPoint(highResPoseLocal[2].m_translation) + NiPoint3(0.f, 4.3f, 11.6f)); // TODO: Remove this hack
    //const_cast<hkQsTransform *>(highResPoseLocal)[3].m_translation = NiPointToHkVector(HkVectorToNiPoint(highResPoseLocal[3].m_translation) + NiPoint3(0.f, 4.3f, 11.6f)); // TODO: Remove this hack
    //const_cast<hkQsTransform *>(highResPoseLocal)[0x18].m_translation = NiPointToHkVector(HkVectorToNiPoint(highResPoseLocal[0x18].m_translation) + NiPoint3(0.f, 4.3f, 11.6f)); // TODO: Remove this hack

    hkbRagdollDriver_mapHighResPoseLocalToLowResPoseLocal(driver, highResPoseLocal, lowResPoseLocal.m_data);

    hkStackArray<hkQsTransform> scaledLowResPoseLocal(numPosesLow);

    CopyAndApplyScaleToPose(true, numPosesLow, lowResPoseLocal.m_data, scaledLowResPoseLocal.m_data, worldFromModel.m_scale(0));

    hkQsTransform worldFromModelWithScaledPositionButScaleIs1;
    CopyAndPotentiallyApplyHavokScaleToTransform(true, &worldFromModel, &worldFromModelWithScaledPositionButScaleIs1);
    worldFromModelWithScaledPositionButScaleIs1.m_scale = hkVector4(1.f, 1.f, 1.f, 1.f);

    ApplyRigidBodyTTransformsToPose(driver->ragdoll, worldFromModelWithScaledPositionButScaleIs1, scaledLowResPoseLocal.m_data, scaledLowResPoseLocal.m_data);

    //hkbPoseLocalToPoseWorld(numPosesLow, driver->ragdoll->m_skeleton->m_parentIndices.begin(), worldFromModelWithScaledPositionButScaleIs1, scaledLowResPoseLocal.m_data, lowResPoseWorldOut);
    NiMathDouble::hkbPoseLocalToPoseWorld_Custom(numPosesLow, driver->ragdoll->m_skeleton->m_parentIndices.begin(), worldFromModelWithScaledPositionButScaleIs1, scaledLowResPoseLocal.m_data, lowResPoseWorldOut);
}

void ApplyRigidBodyTTransformsToPose(const hkaRagdollInstance *ragdoll, const hkQsTransform &worldFromModel, const hkQsTransform *poseLocalIn, hkQsTransform *poseLocalOut)
{
    // First, compute the worldspace transforms of all bones
    hkStackArray<hkQsTransform> boneTransformsWS(ragdoll->getNumBones());

    //hkbPoseLocalToPoseWorld(ragdoll->getNumBones(), ragdoll->m_skeleton->m_parentIndices.begin(), worldFromModel, poseLocalIn, boneTransformsWS.m_data);
    NiMathDouble::hkbPoseLocalToPoseWorld_Custom(ragdoll->getNumBones(), ragdoll->m_skeleton->m_parentIndices.begin(), worldFromModel, poseLocalIn, boneTransformsWS.m_data);

    // Now apply the rigid body T transforms to all the world transforms
    for (int i = 0; i < ragdoll->getNumBones(); i++) {
        if (hkpRigidBody *rigidBody = ragdoll->getRigidBodyOfBone(i)) {
            if (bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData) {
                NiTransform rigidBodyTLocalTransform = GetRigidBodyTLocalTransform(wrapper, false);

                hkQsTransform &transform = boneTransformsWS[i];
                transform = NiTransformTohkQsTransform(hkQsTransformToNiTransform(transform, false) * rigidBodyTLocalTransform, false);
            }
        }
    }

    // Finally, convert the worldspace transforms back to local space
    MapPoseWorldSpaceToPoseLocalSpace(ragdoll->getNumBones(), ragdoll->m_skeleton->m_parentIndices.begin(), &worldFromModel, boneTransformsWS.m_data, poseLocalOut);
}

// This is essentially the default game logic for determining whether 2 filter infos should collide or not
bool bhkCollisionFilter_CompareFilterInfosEx(bhkCollisionFilter *_this, UInt32 filterInfoA, UInt32 filterInfoB, std::optional<UInt64> a_layerBitfield)
{
    UInt32 layerA = filterInfoA & 0x7F;
    __int64 layerBitfield = a_layerBitfield ? *a_layerBitfield : _this->layerBitfields[layerA];

    if (layerA != 44 && ((filterInfoA & 0x4000) != 0 || (filterInfoB & 0x4000) != 0)) // check if nocollision flag is set
        return 0;

    if ((filterInfoA & 0xFFFF0000) == 0 || (filterInfoB & 0xFFFF0000) == 0) // collision group 0 means collide with everything
        return 1;

    UInt32 layerB = filterInfoB & 0x7F;

    if (((filterInfoB ^ filterInfoA) & 0xFFFF0000) == 0) {
        // same collision group

        if (!_bittest64(&layerBitfield, layerB) || (layerA == 30 && (filterInfoA & 0x8000) != 0 || layerB == 30 && (filterInfoB & 0x8000) != 0) && (filterInfoA & 0x80) == 0 && (filterInfoB & 0x80) == 0) {
            // layer bitfield check failed, or something with charcontroller layer and bit 15 / bit 7
            return 0;
        }
        long v10 = filterInfoB & filterInfoA;
        if (_bittest(&v10, 0xFu)) { // bit 15 is set in both collisioninfos
            long v11 = ((filterInfoA >> 8) & 0x1F) - ((filterInfoB >> 8) & 0x1F);
            return std::abs(v11) != 1;
        }
        else {
            if (layerA != 8 && (layerA - 32) > 1 || layerB != 8 && layerB - 32 > 1) // both not biped, biped_no_cc, or deadbip
                return 0;
            long bipedBitfield = _this->bipedBitfields[(filterInfoA >> 8) & 0x1F];
            return _bittest(&bipedBitfield, (filterInfoB >> 8) & 0x1F);
        }
    }

    // different collision group

    if (layerA != layerB && (layerA == 30 && (filterInfoA & 0x1F00) == 7936 || layerB == 30 && (filterInfoB & 0x1F00) == 7936))
        return 0;
    if ((layerA != 30 || (filterInfoA & 0x8000) == 0) && (layerB != 30 || (filterInfoB & 0x8000) == 0)) {
        // Most common case - layer is not charcontroller, or bit 15 is not set, for both infos
        return _bittest64(&layerBitfield, layerB);
    }

    // layer is charcontroller and bit 15 is set, for either info

    if (layerA == 30 && (filterInfoA & 0x8000) != 0 && (filterInfoB & 0x80) != 0 || (filterInfoA & 0x80) != 0 && layerB == 30 && (filterInfoB & 0x8000) != 0)
        return 1;

    __int64 bitfield = _this->triggerBitfield1;
    if ((!_bittest64(&bitfield, layerB) & !_bittest64(&bitfield, layerA)) != 0)
        return 0;

    return _bittest64(&layerBitfield, layerB);
}

NiPoint3 hkpRigidBody_getPointVelocity(const hkpRigidBody *body, const hkVector4 &pos)
{
    hkVector4 pointVelocity; body->getPointVelocity(pos, pointVelocity);
    return HkVectorToNiPoint(pointVelocity);
}

