#pragma once

#include <optional>

#include "skse64/GameReferences.h"

#include "blender.h"
#include "RE/offsets.h"


enum class RagdollState : UInt8
{
    Idle,
    BlendIn,
    BlendOut,
};

struct ActiveRagdoll
{
    Blender blender{};
    std::vector<hkQsTransform> animPose{};
    std::vector<hkQsTransform> ragdollPose{};
    std::vector<hkQsTransform> lowResAnimPoseWS{};
    std::vector<float> stress{};
    std::optional<hkQsTransform> rootBoneTransform{};
    hkQsTransform worldFromModel{};
    hkQsTransform worldFromModelPostPhysics{};
    hkQsTransform stickyWorldFromModel{};
    NiPoint3 rootOffset{}; // meters
    float rootOffsetAngle = 0.f; // radians
    float avgStress = 0.f;
    float deltaTime = 0.f;
    RE::hkRefPtr<hkpEaseConstraintsAction> easeConstraintsAction = nullptr;
    std::unordered_map<hkpConstraintInstance *, std::pair<hkVector4, hkVector4>> originalConstraintPivots{};
    double stateChangedTime = 0.0;
    RagdollState state = RagdollState::Idle;
    KnockState knockState = KnockState::Normal;
    bool wasRigidBodyOn = true;
    bool wasComputingWorldFromModel = false;
    bool fadeInWorldFromModel = false;
    bool fadeOutWorldFromModel = false;
    double worldFromModelFadeInTime = 0.0;
    double worldFromModelFadeOutTime = 0.0;
    double loosenConstraintsStartTime = 0.0;
    bool isOn = false;
    bool shouldNullOutWorldWhenRemovingFromWorld = false;
    bool disableConstraintMotorsForOneFrame = false;
    bool isInRagdollState = false;
};

