#include <chrono>
#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <regex>
#include <map>

#include "skse64/GameRTTI.h"
#include "skse64/NiExtraData.h"
#include "skse64/NiGeometry.h"

#include "utils.h"
#include "RE/offsets.h"
#include "config.h"
#include "math_utils.h"


ITimer g_timer;
double g_currentFrameTime;
double GetTime()
{
	return g_timer.GetElapsedTime();
}

NiAVObject * GetHighestParent(NiAVObject *node)
{
	if (!node->m_parent) {
		return node;
	}
	return GetHighestParent(node->m_parent);
}

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower)
{
	return (modInfo->modIndex << 24) | formLower;
}

bool IsAllowedCollidable(const hkpCollidable *collidable)
{
	hkpRigidBody *rb = hkpGetRigidBody(collidable);
	if (!rb)
		return false;

	auto motion = &rb->m_motion;
	return (
		motion->m_type == hkpMotion::MotionType::MOTION_DYNAMIC ||
		motion->m_type == hkpMotion::MotionType::MOTION_SPHERE_INERTIA ||
		motion->m_type == hkpMotion::MotionType::MOTION_BOX_INERTIA ||
		motion->m_type == hkpMotion::MotionType::MOTION_THIN_BOX_INERTIA
		);
}

bool HasGeometryChildren(NiAVObject *obj)
{
	NiNode *node = obj->GetAsNiNode();
	if (!node) {
		return false;
	}

	for (int i = 0; i < node->m_children.m_arrayBufLen; ++i) {
		auto child = node->m_children.m_data[i];
		if (child && child->GetAsBSGeometry()) {
			return true;
		}
	}

	return false;
}

bool DoesEntityHaveConstraint(NiAVObject *root, bhkRigidBody *entity)
{
	auto rigidBody = GetRigidBody(root);
	if (rigidBody) {
		for (int i = 0; i < rigidBody->constraints.count; i++) {
			bhkConstraint *constraint = rigidBody->constraints.entries[i];
			if (constraint->constraint->getEntityA() == entity->hkBody || constraint->constraint->getEntityB() == entity->hkBody) {
				return true;
			}
		}
	}

	NiNode *rootNode = root->GetAsNiNode();
	if (rootNode) {
		for (int i = 0; i < rootNode->m_children.m_emptyRunStart; i++) {
			NiPointer<NiAVObject> child = rootNode->m_children.m_data[i];
			if (child) {
				if (DoesEntityHaveConstraint(child, entity)) {
					return true;
				}
			}
		}
	}

	return false;
}

bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node)
{
	bhkRigidBody *entity = GetRigidBody(node);
	if (!entity) {
		return false;
	}

	if (entity->constraints.count > 0) {
		// Easy case: it's a master entity
		return true;
	}

	return DoesEntityHaveConstraint(rootNode, entity);
}

NiAVObject * GetTorsoNode(Actor *actor)
{
	TESRace *race = actor->race;
	BGSBodyPartData *partData = race->bodyPartData;
	if (partData) {
		auto torsoData = partData->part[0];
		if (torsoData && torsoData->unk08.data) {
			NiAVObject *actorNode = actor->GetNiNode();
			if (actorNode) {
				NiAVObject *torsoNode = actorNode->GetObjectByName(&torsoData->unk08.data);
				if (torsoNode) {
					return torsoNode;
				}
			}
		}
	}
	return nullptr;
}

void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx)
{
	root->UpdateWorldData(ctx);

	auto node = root->GetAsNiNode();

	if (node) {
		for (int i = 0; i < node->m_children.m_arrayBufLen; ++i) {
			auto child = node->m_children.m_data[i];
			if (child) updateTransformTree(child, ctx);
		}
	}
}

void UpdateKeyframedNodeTransform(NiAVObject *node, const NiTransform &transform)
{
	NiTransform inverseParent;
	node->m_parent->m_worldTransform.Invert(inverseParent);

	node->m_localTransform = inverseParent * transform;
	NiAVObject::ControllerUpdateContext ctx;
	ctx.flags = 0x2000; // makes havok sim more stable
	ctx.delta = 0;
	NiAVObject_UpdateNode(node, &ctx);
}

bool IsTwoHanded(const TESObjectWEAP *weap)
{
	switch (weap->gameData.type) {
	case TESObjectWEAP::GameData::kType_2HA:
	case TESObjectWEAP::GameData::kType_2HS:
	case TESObjectWEAP::GameData::kType_CBow:
	case TESObjectWEAP::GameData::kType_CrossBow:
	case TESObjectWEAP::GameData::kType_Staff:
	case TESObjectWEAP::GameData::kType_Staff2:
	case TESObjectWEAP::GameData::kType_TwoHandAxe:
	case TESObjectWEAP::GameData::kType_TwoHandSword:
		return true;
	default:
		return false;
	}
}

bool IsBow(const TESObjectWEAP *weap)
{
	UInt8 type = weap->gameData.type;
	return (type == TESObjectWEAP::GameData::kType_Bow || type == TESObjectWEAP::GameData::kType_Bow2);
}

TESObjectWEAP * GetEquippedWeapon(Actor *actor, bool isOffhand)
{
	TESForm *equippedObject = actor->GetEquippedObject(isOffhand);
	if (equippedObject) {
		return DYNAMIC_CAST(equippedObject, TESForm, TESObjectWEAP);
	}
	return nullptr;
}

bool IsHoldingTwoHandedWeapon(Actor *actor)
{
	if (TESObjectWEAP *weapon = GetEquippedWeapon(actor, false)) {
		return weapon->type() == TESObjectWEAP::GameData::kType_TwoHandAxe || weapon->type() == TESObjectWEAP::GameData::kType_TwoHandSword;
	}
	return false;
}

bool IsOneHandedWeapon(TESObjectWEAP *weapon)
{
	return weapon->type() == TESObjectWEAP::GameData::kType_OneHandSword || weapon->type() == TESObjectWEAP::GameData::kType_OneHandDagger || weapon->type() == TESObjectWEAP::GameData::kType_OneHandAxe || weapon->type() == TESObjectWEAP::GameData::kType_OneHandMace;
}

bool IsUnarmed(TESForm *equippedObject)
{
	if (!equippedObject) return true;

	TESObjectWEAP *equippedWeapon = DYNAMIC_CAST(equippedObject, TESForm, TESObjectWEAP);
	if (equippedWeapon && equippedWeapon->type() == TESObjectWEAP::GameData::kType_HandToHandMelee) {
		return true;
	}

	return false;
}

void PrintVector(const NiPoint3 &p)
{
	_MESSAGE("%.2f, %.2f, %.2f", p.x, p.y, p.z);
}

std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim)
{
	std::set<std::string, std::less<>> result;
	std::stringstream ss(s);
	std::string item;

	while (getline(ss, item, delim)) {
		trim(item);
		result.insert(item);
	}

	return result;
}

bool VisitNodes(NiAVObject  *parent, std::function<bool(NiAVObject*, int)> functor, int depth = 0)
{
	if (!parent) return false;
	NiNode * node = parent->GetAsNiNode();
	if (node) {
		if (functor(parent, depth))
			return true;

		auto children = &node->m_children;
		for (UInt32 i = 0; i < children->m_emptyRunStart; i++) {
			NiAVObject * object = children->m_data[i];
			if (object) {
				if (VisitNodes(object, functor, depth + 1))
					return true;
			}
		}
	}
	else if (functor(parent, depth))
		return true;

	return false;
}

std::string PrintNodeToString(NiAVObject *avObj, int depth)
{
	std::stringstream avInfoStr;
	avInfoStr << std::string(depth, ' ') << avObj->GetRTTI()->name;
	if (avObj->m_name) {
		avInfoStr << " " << avObj->m_name;
	}
	BSGeometry *geom = avObj->GetAsBSGeometry();
	if (geom) {
		if (geom->m_spSkinInstance) {
			avInfoStr << " [skinned]";
		}
	}
	auto rigidBody = GetRigidBody(avObj);
	if (rigidBody) {
		avInfoStr.precision(5);
		avInfoStr << " m=" << (1.0f / rigidBody->hkBody->m_motion.getMassInv().getReal());
	}
	if (avObj->m_extraData && avObj->m_extraDataLen > 0) {
		avInfoStr << " { ";
		for (int i = 0; i < avObj->m_extraDataLen; i++) {
			auto extraData = avObj->m_extraData[i];
			auto boolExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiBooleanExtraData);
			if (boolExtraData) {
				avInfoStr << extraData->m_pcName << " (bool): " << boolExtraData->m_data << "; ";
				continue;
			}
			auto intExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiIntegerExtraData);
			if (intExtraData) {
				avInfoStr << extraData->m_pcName << " (int): " << intExtraData->m_data << "; ";
				continue;
			}
			auto stringExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiStringExtraData);
			if (stringExtraData) {
				avInfoStr << extraData->m_pcName << " (str): " << stringExtraData->m_pString << "; ";
				continue;
			}
			auto floatExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiFloatExtraData);
			if (floatExtraData) {
				avInfoStr << extraData->m_pcName << " (flt): " << floatExtraData->m_data << "; ";
				continue;
			}
			auto binaryExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiBinaryExtraData);
			if (binaryExtraData) {
				avInfoStr << extraData->m_pcName << " (bin): " << binaryExtraData->m_data << "; ";
				continue;
			}
			auto floatsExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiFloatsExtraData);
			if (floatsExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (flts): ";
				for (int j = 0; j < floatsExtraData->m_size; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << floatsExtraData->m_data[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto intsExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiIntegersExtraData);
			if (intsExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (ints): ";
				for (int j = 0; j < intsExtraData->m_size; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << intsExtraData->m_data[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto strsExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiStringsExtraData);
			if (strsExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (strs): ";
				for (int j = 0; j < strsExtraData->m_size; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << strsExtraData->m_data[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto vecExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiVectorExtraData);
			if (vecExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (vec): ";
				for (int j = 0; j < 4; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << vecExtraData->m_vector[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto fgAnimExtraData = DYNAMIC_CAST(extraData, NiExtraData, BSFaceGenAnimationData);
			if (fgAnimExtraData) {
				avInfoStr << extraData->m_pcName << " (facegen anim); ";
				continue;
			}
			auto fgModelExtraData = DYNAMIC_CAST(extraData, NiExtraData, BSFaceGenModelExtraData);
			if (fgModelExtraData) {
				avInfoStr << extraData->m_pcName << " (facegen model); ";
				continue;
			}
			auto fgBaseMorphExtraData = DYNAMIC_CAST(extraData, NiExtraData, BSFaceGenBaseMorphExtraData);
			if (fgBaseMorphExtraData) {
				avInfoStr << extraData->m_pcName << " (facegen basemorph); ";
				continue;
			}
			avInfoStr << extraData->m_pcName << "; ";
		}
		avInfoStr << "}";
	}

	return std::regex_replace(avInfoStr.str(), std::regex("\\n"), " ");
}

bool PrintNodes(NiAVObject *avObj, int depth)
{
	gLog.Message(PrintNodeToString(avObj, depth).c_str());
	return false;
}

std::ofstream _file;
bool DumpNodes(NiAVObject *avObj, int depth)
{
	_file << PrintNodeToString(avObj, depth).c_str() << std::endl;
	return false;
}

void PrintSceneGraph(NiAVObject *node)
{
	_file.open("scenegraph.log");
	//VisitNodes(node, PrintNodes);
	VisitNodes(node, DumpNodes);
	_file.close();
}

void PrintToFile(std::string entry, std::string filename)
{
	std::ofstream file;
	file.open(filename);
	file << entry << std::endl;
	file.close();
}

bhkCollisionObject * GetCollisionObject(NiAVObject *obj)
{
	if (!obj->unk040) return nullptr;

	auto niCollObj = ((NiCollisionObject *)obj->unk040);
	auto collObj = DYNAMIC_CAST(niCollObj, NiCollisionObject, bhkCollisionObject);
	return collObj;
}

NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj)
{
	auto collObj = GetCollisionObject(obj);
	if (collObj) {
		NiPointer<bhkWorldObject> worldObj = collObj->body;
		auto rigidBody = DYNAMIC_CAST(worldObj, bhkWorldObject, bhkRigidBody);
		if (rigidBody) {
			return rigidBody;
		}
	}

	return nullptr;
}

bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target)
{
	if (haystack == target) {
		return true;
	}

	NiNode *node = haystack->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				if (DoesNodeHaveNode(child, target)) {
					return true;
				}
			}
		}
	}
	return false;
}

bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node)
{
	if (!node || !ref || !ref->loadedState || !ref->loadedState->node) {
		return false;
	}

	return DoesNodeHaveNode(ref->loadedState->node, node);
}

bool IsSkinnedToNodes(NiAVObject *skinnedRoot, const std::unordered_set<NiAVObject *> &targets)
{
	// Check if skinnedRoot is skinned to target
	BSGeometry *geom = skinnedRoot->GetAsBSGeometry();
	if (geom) {
		NiSkinInstancePtr skinInstance = geom->m_spSkinInstance;
		if (skinInstance) {
			NiSkinDataPtr skinData = skinInstance->m_spSkinData;
			if (skinData) {
				UInt32 numBones = *(UInt32*)((UInt64)skinData.m_pObject + 0x58);
				for (int i = 0; i < numBones; i++) {
					NiAVObject *bone = skinInstance->m_ppkBones[i];
					if (bone) {
						if (targets.count(bone) != 0) {
							return true;
						}
					}
				}
			}
		}
		return false;
	}
	NiNode *node = skinnedRoot->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				if (IsSkinnedToNodes(child, targets)) {
					return true;
				}
			}
		}
		return false;
	}
	return false;
}

void PopulateTargets(NiAVObject *root, std::unordered_set<NiAVObject *> &targets)
{
	if (!root) return;

	// Populate targets with the entire subtree but stop when we hit a node with collision

	targets.insert(root);

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				if (!GetRigidBody(child)) {
					PopulateTargets(child, targets);
				}
			}
		}
	}
}

std::unordered_set<NiAVObject *> targetNodeSet;
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target)
{
	PopulateTargets(target, targetNodeSet);

	bool result = IsSkinnedToNodes(skinnedRoot, targetNodeSet);

	targetNodeSet.clear();
	return result;
}

void GetAllSkinnedNodes(NiAVObject *root, std::unordered_set<NiAVObject *> &skinnedNodes)
{
	BSGeometry *geom = root->GetAsBSGeometry();
	if (geom) {
		NiSkinInstancePtr skinInstance = geom->m_spSkinInstance;
		if (skinInstance) {
			NiSkinDataPtr skinData = skinInstance->m_spSkinData;
			if (skinData) {
				UInt32 numBones = *(UInt32*)((UInt64)skinData.m_pObject + 0x58);
				for (int i = 0; i < numBones; i++) {
					NiAVObject *bone = skinInstance->m_ppkBones[i];
					if (bone) {
						skinnedNodes.insert(bone);
					}
				}
			}
		}
	}
	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				GetAllSkinnedNodes(child, skinnedNodes);
			}
		}
	}
}

NiPointer<bhkRigidBody> GetFirstRigidBody(NiAVObject *root)
{
	auto rigidBody = GetRigidBody(root);
	if (rigidBody) {
		return rigidBody;
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				return GetFirstRigidBody(child);
			}
		}
	}

	return nullptr;
}

bool FindRigidBody(NiAVObject *root, hkpRigidBody *query)
{
	NiPointer<bhkRigidBody> rigidBody = GetRigidBody(root);
	if (rigidBody && rigidBody->hkBody == query) {
		return true;
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				if (FindRigidBody(child, query)) {
					return true;
				}
			}
		}
	}

	return false;
}

void ForEachRagdollDriver(Actor *actor, std::function<void(hkbRagdollDriver *)> f)
{
	BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
	if (GetAnimationGraphManager(actor, animGraphManager)) {
		BSAnimationGraphManager *manager = animGraphManager.ptr;
		SimpleLocker lock(&manager->updateLock);
		for (int i = 0; i < manager->graphs.size; i++) {
			BSTSmartPointer<BShkbAnimationGraph> graph = manager->graphs.GetData()[i];
			hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver;
			if (driver) {
				f(driver);
			}
		}
	}
}

void ForEachAdjacentBody(hkbRagdollDriver *driver, hkpRigidBody *body, std::function<void(hkpRigidBody *)> f) {
	if (!driver || !driver->ragdoll) return;

	for (hkpConstraintInstance *constraint : driver->ragdoll->m_constraints) {
		if (constraint->getRigidBodyA() == body) {
			f(constraint->getRigidBodyB());
		}
		else if (constraint->getRigidBodyB() == body) {
			f(constraint->getRigidBodyA());
		}
	}
};

UInt32 PlaySoundAtNode(BGSSoundDescriptorForm *sound, NiAVObject *node, const NiPoint3 &location)
{
	UInt32 formId = sound->formID;

	SoundData soundData;
	BSAudioManager_InitSoundData(*g_audioManager, &soundData, formId, 16);
	if (soundData.id == -1) {
		return 0;
	}

	SoundData_SetPosition(&soundData, location.x, location.y, location.z);
	SoundData_SetNode(&soundData, node);
	if (SoundData_Play(&soundData)) {
		return soundData.id;
	}

	return 0;
}

void PlayPhysicsSound(hkpCollidable *collidable, const NiPoint3 &location, bool loud)
{
	BGSSoundDescriptorForm *sound = nullptr;
	// Try and get the sound that plays when the object hits stone first, as the grab sound
	if (collidable->m_shape && collidable->m_shape->m_userData) {
		auto shape = (bhkShape *)collidable->m_shape->m_userData;
		if (shape) {
			UInt32 materialId = shape->materialId;

			// Handle MOPP shape, as it doesn't have a material on the MOPP shape itself, only the child collection shape...
			auto moppShape = DYNAMIC_CAST(shape->shape, hkpShape, hkpMoppBvTreeShape);
			if (moppShape) {
				const hkpShape *childShape = moppShape->getChild();
				if (childShape) {
					auto bChildShape = (bhkShape *)childShape->m_userData;
					if (bChildShape) {
						materialId = bChildShape->materialId;
					}
				}
			}

			BGSMaterialType *material = GetMaterialType(materialId);

			static UInt32 skinMaterialId = 0x233db702;
			BGSMaterialType *skinMaterial = GetMaterialType(skinMaterialId);
			static UInt32 stoneMaterialId = 0xdf02f237;
			BGSMaterialType *stoneMaterial = GetMaterialType(stoneMaterialId);
			if (material) {
				BGSImpactData *impactData = nullptr;
				if (material->impactDataSet) {
					auto impactDataSet = DYNAMIC_CAST(material->impactDataSet, TESForm, BGSImpactDataSet);
					if (impactDataSet) {
						if (skinMaterial) {
							impactData = BGSImpactDataSet_GetImpactData(impactDataSet, skinMaterial);
						}

						if (!impactData) {
							if (stoneMaterial) {
								impactData = BGSImpactDataSet_GetImpactData(impactDataSet, stoneMaterial);
							}
						}
					}
				}
				else {
					// No impact data set for the material on the shape... try a lookup in the other direction
					if (skinMaterial && skinMaterial->impactDataSet) {
						auto impactDataSet = DYNAMIC_CAST(skinMaterial->impactDataSet, TESForm, BGSImpactDataSet);
						if (impactDataSet) {
							impactData = BGSImpactDataSet_GetImpactData(impactDataSet, material);
						}
					}

					if (!impactData) {
						if (stoneMaterial && stoneMaterial->impactDataSet) {
							auto impactDataSet = DYNAMIC_CAST(stoneMaterial->impactDataSet, TESForm, BGSImpactDataSet);
							if (impactDataSet) {
								impactData = BGSImpactDataSet_GetImpactData(impactDataSet, material);
							}
						}
					}
				}

				if (impactData) {
					// [0] is quieter sound, [1] is louder sound
					int desiredIndex = (int)loud;
					int alternateIndex = (int)!loud;
					sound = impactData->sounds[desiredIndex];
					if (!sound) {
						sound = impactData->sounds[alternateIndex];
					}
				}
			}
		}
	}
	if (!sound) {
		// Failed to get the physics sound, just use the generic pickup sound instead
		TESForm *defaultPickupSound = g_defaultObjectManager->objects[113]; // kPickupSoundGeneric
		if (defaultPickupSound) {
			sound = DYNAMIC_CAST(defaultPickupSound, TESForm, BGSSoundDescriptorForm);
		}
	}
	if (sound) {
		PlaySoundAtNode(sound, nullptr, location);
	}
}

void ModSpeedMult(Actor *actor, float amount)
{
	// actor->RestoreActorValue(kTemporary, kSpeedMult, amount)
	get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 1, 30, amount);

	// Need to cycle carry weight to make speed change take effect
	// actor->RestoreActorValue(kTemporary, kCarryWeight, amount)
	get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 1, 32, 0.1f);
	get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 1, 32, -0.1f);
}

// 0 -> right, 1 -> left, 2-> both
void PlayMeleeImpactRumble(int hand)
{
	if (hand > 1) {
		VRMeleeData *meleeData = GetVRMeleeData(false);
		PlayRumble(true, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
		meleeData = GetVRMeleeData(true);
		PlayRumble(false, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
	}
	else {
		bool isLeft = hand == 1;
		VRMeleeData *meleeData = GetVRMeleeData(isLeft);
		PlayRumble(!isLeft, meleeData->impactConfirmRumbleIntensity, meleeData->impactConfirmRumbleDuration);
	}
}

MovementControllerNPC * GetMovementController(Actor *actor)
{
	// TODO: This is actually refcounted
	return (MovementControllerNPC *)actor->unk148;
}

ActorCause * TESObjectREFR_GetActorCause(TESObjectREFR *refr)
{
	UInt64 *vtbl = *((UInt64 **)refr);
	return ((_TESObjectREFR_GetActorCause)(vtbl[0x51]))(refr);
}

void TESObjectREFR_SetActorCause(TESObjectREFR *refr, ActorCause* cause)
{
	UInt64 *vtbl = *((UInt64 **)refr);
	((_TESObjectREFR_SetActorCause)(vtbl[0x50]))(refr, cause);
}

UInt32 GetHorseHandle(TESObjectREFR *actor)
{
	BSReadLocker lock(&actor->extraData.m_lock);
	if (BSExtraData *extraHorse = actor->extraData.GetByType(0x7B)) {
		return *(UInt32 *)((UInt64)extraHorse + 0x10);
	}
	return *g_invalidRefHandle;
}

KnockState GetActorKnockState(Actor *actor)
{
	return KnockState((actor->actorState.flags04 >> 25) & 7);
}

bool IsActorGettingUp(Actor *actor)
{
	return GetActorKnockState(actor) == KnockState::GetUp;
}

float GetAVPercentage(Actor *actor, UInt32 av)
{
	float current = actor->actorValueOwner.GetCurrent(av);
	float max = actor->actorValueOwner.GetMaximum(av);
	return max == 0.f ? current / max : 1.f;
}

bool SendAction(Actor *source, TESObjectREFR *target, BGSAction *action)
{
	TESActionData input;
	set_vtbl(&input, TESActionData_vtbl);

	input.source = source;
	input.target = target;
	input.action = action;
	input.unk20 = 2;

	return get_vfunc<_TESActionData_Process>(&input, 5)(&input);
}

void TriggerDialogue(Character *source, Character *target, int dialogueSubtype, bool interruptDialogue)
{
	int dialogueType = GetDialogueTypeFromSubtype(dialogueSubtype);
	UpdateDialogue(nullptr, source, target, dialogueType, dialogueSubtype, interruptDialogue, nullptr);
}

void ExitFurniture(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return;

	UInt32 furnitureHandle = middleProcess->furnitureHandle;
	if (furnitureHandle == *g_invalidRefHandle) return;

	BGSAction *activateAction = (BGSAction *)g_defaultObjectManager->objects[55];
	if (!activateAction) return;

	NiPointer<TESObjectREFR> furniture;
	if (!LookupREFRByHandle(furnitureHandle, furniture)) return;

	SendAction(actor, furniture, activateAction);
}

bool HasKeepOffsetInterface(Actor * actor)
{
	bool hasInterface = false;
	if (MovementControllerNPC *controller = GetMovementController(actor)) {
		InterlockedIncrement(&controller->m_refCount); // incref

		static BSFixedString keepOffsetFromActorStr("Keep Offset From Actor");
		hasInterface = controller->GetInterfaceByName_2(keepOffsetFromActorStr) != nullptr;

		// decref
		if (InterlockedExchangeSubtract(&controller->m_refCount, (UInt32)1) == 1) {
			get_vfunc<_BSIntrusiveRefCounted_Destruct>(controller, 0x0)(controller, 1);
		}
	}
	return hasInterface;
}

void Actor_GetBumpedEx(Actor *actor, Actor *bumper, bool isLargeBump, bool exitFurniture, bool pauseCurrentDialogue, bool triggerDialogue)
{
	if (Actor_IsGhost(actor)) return;

	ActorProcessManager *process = actor->processManager;
	if (!process) return;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return;

	TESPackage *runOncePackage = middleProcess->unk058.package;
	if (runOncePackage && runOncePackage->type == 32) return; // already bumped

	if (Actor_HasLargeMovementDelta(actor)) {
		ActorProcess_ResetBumpWaitTimer(process);
	}

	Actor_sub_140600400(actor, 1.f);

	if (pauseCurrentDialogue) {
		get_vfunc<_Actor_PauseCurrentDialogue>(actor, 0x4F)(actor);
	}

	TESPackage *package = CreatePackageByType(32);
	package->packageFlags |= 6;

	PackageLocation packageLocation; PackageLocation_CTOR(&packageLocation);
	PackageLocation_SetNearReference(&packageLocation, actor);
	TESPackage_SetPackageLocation(package, &packageLocation);

	PackageTarget packageTarget; PackageTarget_CTOR(&packageTarget);
	TESPackage_SetPackageTarget(package, &packageTarget);
	PackageTarget_ResetValueByTargetType((PackageTarget *)package->unk40, 0);
	PackageTarget_SetFromReference((PackageTarget *)package->unk40, bumper);

	TESPackage_sub_140439BE0(package, 0);

	if (TESPackage *currentPackage = process->unk18.package) {
		TESPackage_CopyFlagsFromOtherPackage(package, currentPackage);
	}

	get_vfunc<_Actor_PutCreatedPackage>(actor, 0xE1)(actor, package, !exitFurniture, 1);

	if (isLargeBump) {
		Actor_sub_140600400(actor, 1.f);
		sub_140654E10(process, 1);
		ActorProcess_PlayIdle(process, actor, 90, 0, 1, 0, nullptr);
	}

	if (triggerDialogue) {
		ActorProcess_TriggerDialogue(process, actor, 7, 98, bumper, 0, 0, 0, 0, 0);
	}

	sub_140664870(process, 0);
}

void Actor_SayToEx(Actor *source, Actor *target, TESTopic *topic, TESTopicInfo *topicInfo)
{
	ActorProcessManager *sourceProcess = source->processManager;
	if (!sourceProcess) return;

	ActorProcessManager *targetProcess = target->processManager;
	if (!targetProcess) return;

	get_vfunc<_Actor_PauseCurrentDialogue>(source, 0x4F)(source);

	ActorProcess_ResetLipSync(sourceProcess, source);
	ActorProcess_ClearGreetTopic(sourceProcess);

	UInt32 sourceHandle = GetOrCreateRefrHandle(source);
	UInt32 targetHandle = GetOrCreateRefrHandle(target);

	ActorProcess_ClearLookAt2(targetProcess, 0);
	ActorProcess_SetLookAt1(targetProcess, source);
	target->unk0F8 = sourceHandle; // dialogueItemTarget
	ActorProcess_ClearLookAt2(sourceProcess, 0);
	ActorProcess_SetLookAt1(sourceProcess, target);

	source->unk0F8 = targetHandle; // dialogueItemTarget
	ActorProcess_SayTopicInfo(sourceProcess, source, topic, topicInfo, 0, 0, 1, 1);
}

TESTopicInfo * GetRandomTopicInfo(std::vector<UInt32> &topicInfoIDs, UInt32 exclude1, UInt32 exclude2)
{
	int numTopics = topicInfoIDs.size();
	UInt32 formID;
	int i = 0;
	do {
		int random = (int)(GetRandomNumberInRange(0.f, 0.9999f) * numTopics);
		if (random < 0) random = 0;
		if (random >= numTopics) random = numTopics - 1;
		formID = topicInfoIDs[random];
		++i;
	} while ((formID == exclude1 || formID == exclude2) && i < 30);

	if (TESForm *form = LookupFormByID(formID)) {
		if (TESTopicInfo *topicInfo = DYNAMIC_CAST(form, TESForm, TESTopicInfo)) {
			return topicInfo;
		}
	}
	return nullptr;
}

class IsInFactionVisitor : public Actor::FactionVisitor
{
public:
	IsInFactionVisitor::IsInFactionVisitor(TESFaction *faction) : m_faction(faction) { }
	virtual bool Accept(TESFaction * faction, SInt8 rank)
	{
		if (faction == m_faction) {
			isInFaction = true;
			return true;
		}
		return false;
	}

	TESFaction *m_faction;
	bool isInFaction = false;
};

bool IsInFaction(Actor *actor, TESFaction *faction)
{
	IsInFactionVisitor visitor(faction);
	actor->VisitFactions(visitor);
	return visitor.isInFaction;
}

class IsCalmEffectVisitor : public MagicTarget::ForEachActiveEffectVisitor
{
public:
	bool m_isCalm = false;

	virtual BSContainer::ForEachResult Visit(ActiveEffect* effect)
	{
		if (MagicItem::EffectItem *effectItem = effect->effect) {
			if (EffectSetting *effectSetting = effectItem->mgef) {
				if (effectSetting->properties.archetype == EffectSetting::Properties::kArchetype_Calm) {
					m_isCalm = true;
					return BSContainer::ForEachResult::kAbort;
				}
			}
		}
		return BSContainer::ForEachResult::kContinue;
	}
};

bool IsCalmed(Actor *actor)
{
	MagicTarget &magicTarget = actor->magicTarget;
	IsCalmEffectVisitor visitor{};
	magicTarget.ForEachActiveEffect(visitor);
	return visitor.m_isCalm;
}

NiPointer<NiAVObject> GetFirstPersonHandNode(bool isLeft)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player->GetNiRootNode(1)) return nullptr;

	return isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftHandBone] : player->unk3F0[PlayerCharacter::Node::kNode_RightHandBone];
}

bool IsHandWithinConeFromHmd(bool isLeft, float halfAngle)
{
	NiPointer<NiAVObject> handNode = GetFirstPersonHandNode(isLeft);
	if (!handNode) return false;

	NiPoint3 handPos = handNode->m_worldTransform.pos;

	PlayerCharacter *player = *g_thePlayer;
	NiPointer<NiAVObject> hmdNode = player->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
	if (!hmdNode) return false;

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;
	// Skyrim coords: +x: right vector, +y: forward vector, +z: up vector
	NiPoint3 hmdForward = ForwardVector(hmdNode->m_worldTransform.rot);

	float requiredDotProduct = cosf(halfAngle * 0.0174533f);
	return DotProduct(VectorNormalized(handPos - hmdPos), hmdForward) >= requiredDotProduct;
}
