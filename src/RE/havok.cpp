#pragma once

#include "skse64/GameRTTI.h"

#include "RE/havok.h"
#include "RE/offsets.h"
#include "math_utils.h"

hkMemoryRouter &hkGetMemoryRouter()
{
	return *(hkMemoryRouter *)(hkUlong)TlsGetValue(*g_dwTlsIndex);
}

hkConstraintCinfo::~hkConstraintCinfo()
{
	hkConstraintCinfo_setConstraintData(this, nullptr);
}

auto hkMalleableConstraintCinfo_vtbl = RelocAddr<void *>(0x182C5F8);
hkMalleableConstraintCinfo::hkMalleableConstraintCinfo()
{
	this->vtbl = hkMalleableConstraintCinfo_vtbl;
}

auto hkRagdollConstraintCinfo_vtbl = RelocAddr<void *>(0x1830F38);
hkRagdollConstraintCinfo::hkRagdollConstraintCinfo()
{
	this->vtbl = hkRagdollConstraintCinfo_vtbl;
}

auto bhkMalleableConstraint_vtbl = RelocAddr<void *>(0x182C628);
void bhkMalleableConstraint_ctor(bhkMalleableConstraint *_this, hkMalleableConstraintCinfo *cInfo)
{
	// Construct bhkMalleableConstraint
	bhkRefObject_ctor(_this);
	_this->unk18 = 0;
	*((void **)_this) = ((void *)(bhkMalleableConstraint_vtbl)); // set vtbl
	_this->InitHavokFromCinfo((void *)((UInt64)cInfo + 8)); // need to pass in cInfo + 8 for whatever reason. Within the function it subtracts 8 again...
}

bhkMalleableConstraint * CreateMalleableConstraint(bhkConstraint *constraint, float strength)
{
	if (DYNAMIC_CAST(constraint, bhkConstraint, bhkMalleableConstraint)) return nullptr; // already a malleable constraint

	hkMalleableConstraintCinfo cInfo;
	hkMalleableConstraintCinfo_Func4(&cInfo); // Creates constraintData
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

hkpConstraintInstance * LimitedHingeToRagdollConstraint(hkpConstraintInstance *constraint)
{
	// gather data from limited hinge constraint
	hkpLimitedHingeConstraintData* limitedHingeData = static_cast<hkpLimitedHingeConstraintData*>(const_cast<hkpConstraintData*>(constraint->getData()));
	hkVector4& childPivot = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(3);
	hkVector4& parentPivot = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(3);
	hkVector4& childPlane = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(0);
	hkVector4& parentPlane = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(0);

	// get childTwist axis and compute a parentTwist axis which closely matches the limited hinge's min/max limits.
	hkReal minAng = limitedHingeData->getMinAngularLimit();
	hkReal maxAng = limitedHingeData->getMaxAngularLimit();
	hkReal angExtents = (maxAng - minAng) / 2.0f;

	//hkQuaternion minAngRotation; minAngRotation.setAxisAngle(parentPlane, -angExtents + maxAng);
	hkQuaternion minAngRotation = NiQuatToHkQuat(MatrixToQuaternion(MatrixFromAxisAngle(HkVectorToNiPoint(parentPlane), -angExtents + maxAng)));
	minAngRotation.normalize();

	hkVector4& childTwist = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(2);
	hkVector4& parentLimitedAxis = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(2);
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
	hkVector4& childPivot = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(3);
	hkVector4& parentPivot = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(3);
	hkVector4& childPlane = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(0);
	hkVector4& parentPlane = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(0);

	// get childTwist axis and compute a parentTwist axis which closely matches the limited hinge's min/max limits.
	hkReal minAng = limitedHingeData->getMinAngularLimit();
	hkReal maxAng = limitedHingeData->getMaxAngularLimit();
	hkReal angExtents = (maxAng - minAng) / 2.0f;

	//hkQuaternion minAngRotation; minAngRotation.setAxisAngle(parentPlane, -angExtents + maxAng);
	hkQuaternion minAngRotation = NiQuatToHkQuat(MatrixToQuaternion(MatrixFromAxisAngle(HkVectorToNiPoint(parentPlane), -angExtents + maxAng)));
	minAngRotation.normalize();

	hkVector4& childTwist = limitedHingeData->m_atoms.m_transforms.m_transformA.getColumn(2);
	hkVector4& parentLimitedAxis = limitedHingeData->m_atoms.m_transforms.m_transformB.getColumn(2);
	hkVector4 parentTwist; parentTwist.setRotatedDir(minAngRotation, parentLimitedAxis);

	hkpRagdollConstraintData_setInBodySpace(ragdollData, childPivot, parentPivot, childPlane, parentPlane, childTwist, parentTwist);

	// adjust limits to make it like the hinge constraint
	ragdollData->setConeAngularLimit(angExtents);
	ragdollData->setTwistMinAngularLimit(0.0f);
	ragdollData->setTwistMaxAngularLimit(0.0f);
	ragdollData->m_atoms.m_angFriction.m_maxFrictionTorque = limitedHingeData->getMaxFrictionTorque();
	ragdollData->setAngularLimitsTauFactor(limitedHingeData->getAngularLimitsTauFactor());
}

bhkRagdollConstraint * ConvertToRagdollConstraint(bhkConstraint *constraint)
{
	if (DYNAMIC_CAST(constraint, bhkConstraint, bhkRagdollConstraint)) return nullptr; // already a bhkRagdollConstraint

	hkRagdollConstraintCinfo cInfo;
	hkRagdollConstraintCinfo_Func4(&cInfo); // Creates constraintData and calls hkpRagdollConstraintData_ctor()
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


NiPointer<bhkCharRigidBodyController> GetCharRigidBodyController(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return nullptr;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return nullptr;

	NiPointer<bhkCharacterController> controller = *((NiPointer<bhkCharacterController> *)&middleProcess->unk250);
	if (!controller) return nullptr;

	return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharRigidBodyController);
}

NiPointer<bhkCharProxyController> GetCharProxyController(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return nullptr;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return nullptr;

	NiPointer<bhkCharacterController> controller = *((NiPointer<bhkCharacterController> *)&middleProcess->unk250);
	if (!controller) return nullptr;

	return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharProxyController);
}

Actor * GetActorFromRagdollDriver(hkbRagdollDriver *driver)
{
	hkbCharacter *character = driver->character;
	if (!character) return nullptr;

	hkbBehaviorGraph *behaviorGraph = character->behaviorGraph;
	if (!behaviorGraph) return nullptr;

	BShkbAnimationGraph *graph = (BShkbAnimationGraph *)behaviorGraph->userData;
	if (!graph) return nullptr;

	return graph->holder;
}

void ReSyncLayerBitfields(bhkCollisionFilter *filter, UInt64 bitfield)
{
	for (int i = 0; i < 56; i++) { // 56 layers in vanilla
		if ((bitfield >> i) & 1) {
			filter->layerBitfields[i] |= ((UInt64)1 << 57);
		}
	}
}

float hkpContactPointEvent_getSeparatingVelocity(const hkpContactPointEvent &_this)
{
	if (_this.m_separatingVelocity)
	{
		return *_this.m_separatingVelocity;
	}
	else
	{
		return hkpSimpleContactConstraintUtil_calculateSeparatingVelocity(_this.m_bodies[0], _this.m_bodies[1], _this.m_bodies[0]->getCenterOfMassInWorld(), _this.m_bodies[1]->getCenterOfMassInWorld(), _this.m_contactPoint);
	}
}
