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


NiPointer<bhkCharacterController> GetCharacterController(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return nullptr;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return nullptr;

	return *((NiPointer<bhkCharacterController> *)&middleProcess->unk250);
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


BShkbAnimationGraph * GetAnimationGraph(hkbCharacter *character)
{
	hkbBehaviorGraph *behaviorGraph = character->behaviorGraph;
	if (!behaviorGraph) return nullptr;

	BShkbAnimationGraph *graph = (BShkbAnimationGraph *)behaviorGraph->userData;
	return graph;
}

Actor * GetActorFromCharacter(hkbCharacter *character)
{
	BShkbAnimationGraph *graph = GetAnimationGraph(character);
	if (!graph) return nullptr;

	return graph->holder;
}

Actor * GetActorFromRagdollDriver(hkbRagdollDriver *driver)
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

void hkpWorld_removeContactListener(hkpWorld *_this, hkpContactListener* worldListener)
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

int hkpCharacterProxy_findCharacterProxyListener(hkpCharacterProxy *_this, hkpCharacterProxyListener* proxyListener)
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
	if (_this.m_separatingVelocity)
	{
		return *_this.m_separatingVelocity;
	}
	else
	{
		return hkpSimpleContactConstraintUtil_calculateSeparatingVelocity(_this.m_bodies[0], _this.m_bodies[1], _this.m_bodies[0]->getCenterOfMassInWorld(), _this.m_bodies[1]->getCenterOfMassInWorld(), _this.m_contactPoint);
	}
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
