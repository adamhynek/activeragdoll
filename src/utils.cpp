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

NiAVObject *GetHighestParent(NiAVObject *node)
{
    if (!node->m_parent) {
        return node;
    }
    return GetHighestParent(node->m_parent);
}

UInt32 GetFullFormID(const ModInfo *modInfo, UInt32 formLower)
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

NiAVObject *GetTorsoNode(Actor *actor)
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

void updateTransformTree(NiAVObject *root, NiAVObject::ControllerUpdateContext *ctx)
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

TESObjectWEAP *GetEquippedWeapon(Actor *actor, bool isOffhand)
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

bool ShouldBashBasedOnWeapon(Actor *actor, bool isOffhand, bool isLeft, bool allowWeaponBash, const NiPoint3 *hitPosition)
{
    PlayerCharacter *player = *g_thePlayer;

    TESForm *equippedObj = actor->GetEquippedObject(isOffhand);
    TESObjectWEAP *weapon = DYNAMIC_CAST(equippedObj, TESForm, TESObjectWEAP);
    bool isBowOrCrossbow = weapon && (weapon->type() == TESObjectWEAP::GameData::kType_Bow || weapon->type() == TESObjectWEAP::GameData::kType_CrossBow);

    TESForm *offhandObj = actor->GetEquippedObject(true);
    TESObjectARMO *equippedShield = (offhandObj && offhandObj->formType == kFormType_Armor) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectARMO) : nullptr;
    bool isShield = isOffhand && equippedShield;

    TESObjectLIGH *equippedLight = (offhandObj && offhandObj->formType == kFormType_Light) ? DYNAMIC_CAST(offhandObj, TESForm, TESObjectLIGH) : nullptr;
    bool isTorch = isOffhand && equippedLight;

    if (isBowOrCrossbow || isShield || isTorch) {
        // We hit with a bow, crossbow, shield, or torch - always bash
        return true;
    }
    else if (allowWeaponBash && !IsUnarmed(weapon)) {
        if (!hitPosition) return false;

        NiPointer<NiAVObject> hmdNode = player->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
        if (!hmdNode) return false;

        NiPointer<NiAVObject> weaponOffsetNode = GetWeaponCollisionOffsetNode(weapon, isLeft);
        if (!weaponOffsetNode) return false;

        // Use last frame's offset node transform because this frame is not over yet and it can still be modified by e.g. higgs two-handing
        NiPoint3 weaponForward = ForwardVector(weaponOffsetNode->m_oldWorldTransform.rot);
        NiPoint3 hmdForward = ForwardVector(hmdNode->m_worldTransform.rot);
        float weaponPointingForwardAmount = DotProduct(weaponForward, hmdForward);

        NiPoint3 weaponPos = weaponOffsetNode->m_worldTransform.pos;
        float handToHitDistance = VectorLength(weaponPos - *hitPosition);

        if (weaponPointingForwardAmount <= Config::options.weaponBashWeaponInHmdDirectionThreshold && handToHitDistance <= Config::options.weaponBashMaxHandToHitDistance) {
            // We hit with a weapon that is not pointing forward too much and the hit position is close enough to the hand (more of a blade hit)
            return true;
        }
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

bool VisitNodes(NiAVObject *parent, std::function<bool(NiAVObject *, int)> functor, int depth)
{
    if (!parent) return false;

    if (functor(parent, depth)) {
        return true;
    }

    if (NiNode *node = parent->GetAsNiNode()) {
        for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
            if (NiAVObject *child = node->m_children.m_data[i]) {
                if (VisitNodes(child, functor, depth + 1)) {
                    return true;
                }
            }
        }
    }

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

NiPointer<bhkCollisionObject> GetCollisionObject(NiAVObject *obj)
{
    if (!obj->unk040) return nullptr;

    if (NiPointer<NiCollisionObject> niCollObj = (NiCollisionObject *)obj->unk040) {
        if (NiPointer<bhkCollisionObject> collObj = DYNAMIC_CAST(niCollObj, NiCollisionObject, bhkCollisionObject)) {
            return collObj;
        }
    }

    return nullptr;
}

NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj)
{
    if (NiPointer<bhkCollisionObject> collObj = GetCollisionObject(obj)) {
        if (NiPointer<bhkWorldObject> worldObj = collObj->body) {
            if (NiPointer<bhkRigidBody> rigidBody = DYNAMIC_CAST(worldObj, bhkWorldObject, bhkRigidBody)) {
                return rigidBody;
            }
        }
    }

    return nullptr;
}

bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target)
{
    if (haystack == target) {
        return true;
    }

    if (NiNode *node = haystack->GetAsNiNode()) {
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
                UInt32 numBones = *(UInt32 *)((UInt64)skinData.m_pObject + 0x58);
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
                UInt32 numBones = *(UInt32 *)((UInt64)skinData.m_pObject + 0x58);
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
    if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(root)) {
        return rigidBody;
    }

    NiNode *node = root->GetAsNiNode();
    if (node) {
        for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
            if (NiAVObject *child = node->m_children.m_data[i]) {
                if (NiPointer<bhkRigidBody> rigidBody = GetFirstRigidBody(child)) {
                    return rigidBody;
                }
            }
        }
    }

    return nullptr;
}

NiPointer<bhkRigidBody> FindRigidBody(NiAVObject *root, bhkRigidBody *query)
{
    NiPointer<bhkRigidBody> rigidBody = GetRigidBody(root);
    if (rigidBody && rigidBody == query) {
        return rigidBody;
    }

    if (NiNode *node = root->GetAsNiNode()) {
        for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
            if (NiAVObject *child = node->m_children.m_data[i]) {
                if (NiPointer<bhkRigidBody> rb = FindRigidBody(child, query)) {
                    return rb;
                }
            }
        }
    }

    return false;
}

NiPointer<NiAVObject> GetClosestParentWithCollision(NiAVObject *node, bool ignoreSelf)
{
    NiPointer<NiAVObject> nodeWithCollision = node;
    while (nodeWithCollision) {
        if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(nodeWithCollision)) {
            if (rigidBody->hkBody->m_world) {
                if (!ignoreSelf || nodeWithCollision != node) {
                    return nodeWithCollision;
                }
            }
        }
        nodeWithCollision = nodeWithCollision->m_parent;
    }
    return nullptr;
}

void ForEachRagdollDriver(BSAnimationGraphManager *graphManager, std::function<void(hkbRagdollDriver *)> f)
{
    SimpleLocker lock(&graphManager->updateLock);
    for (int i = 0; i < graphManager->graphs.size; i++) {
        BSTSmartPointer<BShkbAnimationGraph> graph = graphManager->graphs.GetData()[i];
        if (hkbRagdollDriver *driver = graph.ptr->character.ragdollDriver) {
            f(driver);
        }
    }
}

void ForEachRagdollDriver(Actor *actor, std::function<void(hkbRagdollDriver *)> f)
{
    BSTSmartPointer<BSAnimationGraphManager> animGraphManager{ 0 };
    if (GetAnimationGraphManager(actor, animGraphManager)) {
        ForEachRagdollDriver(animGraphManager.ptr, f);
    }
}

void ForEachAnimationGraph(BSAnimationGraphManager *graphManager, std::function<void(BShkbAnimationGraph *)> f)
{
    SimpleLocker lock(&graphManager->updateLock);
    for (int i = 0; i < graphManager->graphs.size; i++) {
        BSTSmartPointer<BShkbAnimationGraph> graph = graphManager->graphs.GetData()[i];
        if (!graph.ptr) continue;
        f(graph.ptr);
    }
}

void CollectAllConstraints(NiAVObject *root, std::vector<NiPointer<bhkConstraint>> &out)
{
    if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(root)) {
        if (rigidBody->hkBody) {
            for (int i = 0; i < rigidBody->constraints.count; i++) {
                bhkConstraint *constraint = rigidBody->constraints.entries[i];
                out.push_back(constraint);
            }
        }
    }

    if (NiNode *node = root->GetAsNiNode()) {
        for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
            if (NiAVObject *child = node->m_children.m_data[i]) {
                CollectAllConstraints(child, out);
            }
        }
    }
}

void ForEachAdjacentBody(NiAVObject *root, bhkRigidBody *body, std::function<void(bhkRigidBody *, int)> f, int waves) {
    if (!root || !body) return;

    std::vector<NiPointer<bhkConstraint>> constraints{};
    CollectAllConstraints(root, constraints);

    std::set<NiPointer<bhkRigidBody>> connectedComponent{};
    connectedComponent.insert(body);
    f(body, 0);

    for (int i = 1; i <= waves; i++) {
        std::set<NiPointer<bhkRigidBody>> currentConnectedComponent = connectedComponent; // copy

        for (bhkConstraint *constraintWrapper : constraints) {
            hkpConstraintInstance *constraint = constraintWrapper->constraint;

            bool isConstraintEnabled; hkpConstraintInstance_isEnabled(constraint, &isConstraintEnabled);
            if (!isConstraintEnabled) continue;

            bhkRigidBody *rigidBodyA = (bhkRigidBody *)constraint->getEntityA()->m_userData;
            bhkRigidBody *rigidBodyB = (bhkRigidBody *)constraint->getEntityB()->m_userData;

            bool isAInSet = currentConnectedComponent.count(rigidBodyA);
            bool isBInSet = currentConnectedComponent.count(rigidBodyB);

            if (isAInSet || isBInSet) {
                // The constraint is connected to the existing connected component
                if (!isAInSet || !isBInSet) {
                    // Part of the constraint is not part of the connected component yet, so add it
                    connectedComponent.insert(rigidBodyA);
                    connectedComponent.insert(rigidBodyB);
                }

                if (!isAInSet) {
                    f(rigidBodyA, i);
                }
                if (!isBInSet) {
                    f(rigidBodyB, i);
                }
            }
        }
    }
}

NiTransform GetRigidBodyTLocalTransform(bhkRigidBody *rigidBody, bool useHavokScale)
{
    NiTransform rigidBodyLocalTransform{}; // identity
    if (bhkRigidBodyT *rigidBodyT = DYNAMIC_CAST(rigidBody, bhkRigidBody, bhkRigidBodyT)) {
        rigidBodyLocalTransform.pos = HkVectorToNiPoint(rigidBodyT->translation) * (useHavokScale ? *g_inverseHavokWorldScale : 1.f);
        rigidBodyLocalTransform.rot = QuaternionToMatrix(HkQuatToNiQuat(rigidBodyT->rotation));
    }
    return rigidBodyLocalTransform;
}

bool DialogueItem_ResetCurrentResponse(DialogueItem *a1)
{
    a1->CurrentResponse_18 = &a1->Responses_8;
    return a1->Responses_8.Item_0 != nullptr;
}

DialogueResponse *DialogueItem_GetCurrentResponse(DialogueItem *a1)
{
    if (BSSimpleList_DialogueResponse__ *currentResponse = a1->CurrentResponse_18) {
        return currentResponse->Item_0;
    }
    return nullptr;
}

bool EvaluateConditionsEx(Condition *condition, ConditionCheckParams &params, const std::vector<UInt16> &skipConditions)
{
    // Should be the same condition evaluation logic as the base game, but with the ability to skip certain conditions.
    // The way it works is every condition must be true, unless it is in an OR block. An OR block is a consecutive set of conditions that are ORed together, and the result of the OR block must be true. There is no nesting or anything like that.

    bool isORtrue = false;
    bool wasOR = false;

    while (condition) {
        bool isOR = condition->comparisonType & 1; // 1 = OR, 0 = AND

        bool alwaysSucceed = std::find(skipConditions.begin(), skipConditions.end(), condition->functionId) != skipConditions.end();

        if (!wasOR && isOR) {
            // Entering an OR block
            bool isTrue = alwaysSucceed || TESConditionItem_IsTrue(condition, params);
            isORtrue = isTrue;
        }
        else if (wasOR && !isOR) {
            // Exiting an OR block
            // This condition is still part of the OR block (the last part). So should still evaluate it as part of the block.
            if (!isORtrue) {
                // We only need to check the condition if the OR hasn't had a true yet
                bool isTrue = alwaysSucceed || TESConditionItem_IsTrue(condition, params);
                isORtrue = isORtrue || isTrue;
            }

            if (!isORtrue) {
                // OR block was false, so it doesn't pass
                return false;
            }
        }
        else if (wasOR && isOR) {
            // Still in an OR block
            if (!isORtrue) {
                // We only need to check the condition if the OR hasn't had a true yet
                bool isTrue = alwaysSucceed || TESConditionItem_IsTrue(condition, params);
                isORtrue = isORtrue || isTrue;
            }
        }
        else { // !wasOR && !isOR
            bool isTrue = alwaysSucceed || TESConditionItem_IsTrue(condition, params);
            if (!isTrue) {
                // not in an OR block and condition is false, so it doesn't pass
                return false;
            }
        }

        wasOR = isOR;

        condition = condition->next;
    }

    // No more conditions. If we ended in an OR block, need to check the outcome of the block.
    if (wasOR && !isORtrue) {
        // OR block was false, so it doesn't pass
        return false;
    }

    return true;
}

bool TESTopicInfo_EvaluateConditionsEx(TESTopicInfo *topicInfo, Actor *source, TESObjectREFR *target, const std::vector<UInt16> &skipConditions)
{
    if (topicInfo->flags & 0x20) return false; // 0x20 == kDeleted

    ConditionCheckParams params(source, target);
    TESTopic *topic = (TESTopic *)topicInfo->unk14;
    if (topic) params.quest = (TESQuest *)topic->unk40;

    bool result = false;
    if (!params.quest || EvaluateConditionsEx((Condition *)params.quest->unk108, params, skipConditions)) {
        result = EvaluateConditionsEx(topicInfo->conditions, params, skipConditions);
    }

    return result;
}

void PlayTopicInfoWithoutActorChecks(TESTopicInfo *topicInfo, Actor *source, TESObjectREFR *target)
{
    if (TESTopic *topic = (TESTopic *)topicInfo->unk14) {
        if (DialogueItem *dialogueItem = TESTopic_GetDialogueItem(topic, source, target, topicInfo, topic, 0)) {
            _InterlockedIncrement(&dialogueItem->refCount_0);

            if (DialogueItem_ResetCurrentResponse(dialogueItem)) {
                if (DialogueResponse *response = DialogueItem_GetCurrentResponse(dialogueItem)) {

                    NiAVObject *sourceNode = source->GetNiNode();
                    if (ActorProcessManager *actorProcess = source->processManager) {
                        if (NiAVObject *headNode = ActorProcess_GetHeadNode(actorProcess)) {
                            sourceNode = headNode;
                        }
                    }

                    if (BGSSoundDescriptorForm *sound = response->VoiceSound_30) {
                        // easy
                        PlaySoundAtNode(sound, sourceNode, {});
                    }
                    else {
                        const char *soundPath = response->Voice_18.data;
                        char dst[264];
                        const char *soundFileRelativePath = strip_base_path(dst, 260, soundPath, "sound\\");

                        bool isPlayer = source == *g_thePlayer;

                        BSResource__ID resourceID;
                        BSResource__ID_ctor(&resourceID, soundFileRelativePath);

                        // no idea what these flags are
                        UInt32 flags = 0x10;
                        if (!isPlayer) flags |= 0x80;
                        //if (false) flags |= 0x8000; // some condition, not sure what it is

                        SoundData soundHandle;
                        BSAudioManager_CreateSoundHandleFromResource(*g_audioManager, &soundHandle, &resourceID, flags, isPlayer ? 0x40 : 0x80);

                        if (BGSSoundOutput *soundOutput = GetDefaultObject<BGSSoundOutput>(isPlayer ? 148 : 147)) { // kDialogueOutputModel3D for non-player, kDialogueOutputModel2D for player
                            SoundData_SetDialogueOutputModel(&soundHandle, &soundOutput->soundOutputModel);
                        }

                        if (BGSSoundCategory *soundCategory = GetDefaultObject<BGSSoundCategory>(136)) { // kDialogueVoiceCategory
                            SoundData_SetSoundCategory(&soundHandle, &soundCategory->soundCategory, 1000);
                        }

                        SoundData_SetNode(&soundHandle, sourceNode);

                        SoundData_Play(&soundHandle);
                    }
                }
            }

            if (_InterlockedExchangeAdd(&dialogueItem->refCount_0, 0xFFFFFFFF) == 1) {
                DialogueItem_dtor(dialogueItem);
                Heap_Free(dialogueItem);
            }
        }
    }
}

void PlayDialogueWithoutActorChecks(int subType, Actor *source, TESObjectREFR *target)
{
    if (TESTopicInfo *topicInfo = BGSStoryTeller_GetTopicInfoForDialogue(*g_storyTeller, GetDialogueTypeFromSubtype(subType), subType, source, target, 0, 0)) {
        PlayTopicInfoWithoutActorChecks(topicInfo, source, target);
    }
}

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

MovementControllerNPC *GetMovementController(Actor *actor)
{
    // TODO: This is actually refcounted
    return (MovementControllerNPC *)actor->unk148;
}

ActorCause *TESObjectREFR_GetActorCause(TESObjectREFR *refr)
{
    UInt64 *vtbl = *((UInt64 **)refr);
    return ((_TESObjectREFR_GetActorCause)(vtbl[0x51]))(refr);
}

void TESObjectREFR_SetActorCause(TESObjectREFR *refr, ActorCause *cause)
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

void TriggerDialogueByType(Character *source, Character *target, int dialogueSubtype, bool interruptDialogue)
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

bool HasKeepOffsetInterface(Actor *actor)
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

    ActorProcess_SetPlayerActionReaction(process, 0);
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

TESTopicInfo *GetRandomTopicInfo(const std::vector<TESTopicInfo *> &topicInfos, TESTopicInfo *exclude1, TESTopicInfo *exclude2)
{
    int numTopics = topicInfos.size();
    if (numTopics == 0) return nullptr;

    TESTopicInfo * topicInfo = nullptr;
    int i = 0;
    do {
        int random = (int)(GetRandomNumberInRange(0.f, 0.9999f) * numTopics);
        if (random < 0) random = 0;
        if (random >= numTopics) random = numTopics - 1;
        topicInfo = topicInfos[random];
        ++i;
    } while ((topicInfo == exclude1 || topicInfo == exclude2) && i < 30);

    return topicInfo;
}

std::vector<TESTopicInfo *> EvaluateTopicInfoConditions(const std::vector<UInt32> &topicInfoIDs, Actor *source, Actor *target, const std::vector<UInt16> &skipConditions)
{
    std::vector<TESTopicInfo *> validTopicInfos;
    for (UInt32 formID : topicInfoIDs) {
        if (TESForm *form = LookupFormByID(formID)) {
            if (TESTopicInfo *topicInfo = DYNAMIC_CAST(form, TESForm, TESTopicInfo)) {
                if (TESTopicInfo_EvaluateConditionsEx(topicInfo, source, target, skipConditions)) {
                    validTopicInfos.push_back(topicInfo);
                }
            }
        }
    }
    return validTopicInfos;
}

class IsInFactionVisitor : public Actor::FactionVisitor
{
public:
    IsInFactionVisitor::IsInFactionVisitor(TESFaction *faction) : m_faction(faction) { }
    virtual bool Accept(TESFaction *faction, SInt8 rank)
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

    virtual BSContainer::ForEachResult Visit(ActiveEffect *effect)
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

NiPointer<NiAVObject> GetWeaponCollisionOffsetNode(TESObjectWEAP *weapon, bool isLeft)
{
    PlayerCharacter *player = *g_thePlayer;

    if (!weapon || weapon->gameData.type == TESObjectWEAP::GameData::kType_HandToHandMelee) {
        return isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftHandBone] : player->unk3F0[PlayerCharacter::Node::kNode_RightHandBone];
    }
    else if (weapon->gameData.type == TESObjectWEAP::GameData::kType_Bow) {
        return player->unk538[PlayerCharacter::BowNode::kBowNode_BowRotationNode];
    }
    else if (weapon->gameData.type == TESObjectWEAP::GameData::kType_CrossBow) {
        return player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftWeaponOffsetNode : PlayerCharacter::Node::kNode_RightWeaponOffsetNode];
    }
    else if (weapon->gameData.type == TESObjectWEAP::GameData::kType_Staff) {
        return player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftStaffWeaponOffsetNode : PlayerCharacter::Node::kNode_RightStaffWeaponOffsetNode];
    }

    return player->unk3F0[isLeft ? PlayerCharacter::Node::kNode_LeftMeleeWeaponOffsetNode : PlayerCharacter::Node::kNode_RightMeleeWeaponOffsetNode];
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
