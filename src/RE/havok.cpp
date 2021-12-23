#pragma once

#include "skse64/GameRTTI.h"

#include "RE/havok.h"
#include "RE/offsets.h"

hkConstraintCinfo::~hkConstraintCinfo()
{
	hkConstraintCinfo_setConstraintData(this, nullptr);
}

auto hkMalleableConstraintCinfo_vtbl = RelocAddr<void *>(0x182C5F8);
hkMalleableConstraintCinfo::hkMalleableConstraintCinfo()
{
	this->vtbl = hkMalleableConstraintCinfo_vtbl;
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
	hkMalleableConstraintCinfo_Func4(&cInfo);
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


bhkCharRigidBodyController * GetCharRigidBodyController(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return nullptr;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return nullptr;

	NiPointer<bhkCharacterController> controller = *((NiPointer<bhkCharacterController> *)&middleProcess->unk250);
	if (!controller) return nullptr;

	return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharRigidBodyController);
}

bhkCharProxyController * GetCharProxyController(Actor *actor)
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
