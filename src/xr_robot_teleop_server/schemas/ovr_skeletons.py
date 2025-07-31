"""
Defines Python enumerations for tracking skeleton bones in XR applications.

This module provides standardized integer constants for identifying bones in
various skeleton types, including legacy hand, OpenXR hand, and full-body
skeletons. These enums are essential for interpreting tracking data from XR
devices and APIs.

The definitions, particularly for `XRHandBoneId`, are aligned with the
conventions established by the OpenXR specification. For more details on the
underlying standards, see the official OpenXR documentation:
https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html

Key Enums:
- HandBoneId: For legacy hand tracking systems.
- XRHandBoneId: For hand tracking compliant with the OpenXR standard.
- BodyBoneId: For upper-body tracking.
- FullBodyBoneId: For full-body tracking, including legs.
- SkeletonType: To distinguish between the different kinds of skeletons.
"""

import enum


class HandBoneId(enum.IntEnum):
    """Specifies the bone IDs for a legacy hand skeleton."""

    Hand_Start = 0
    Hand_WristRoot = 0
    Hand_ForearmStub = 1
    Hand_Thumb0 = 2
    Hand_Thumb1 = 3
    Hand_Thumb2 = 4
    Hand_Thumb3 = 5
    Hand_Index1 = 6
    Hand_Index2 = 7
    Hand_Index3 = 8
    Hand_Middle1 = 9
    Hand_Middle2 = 10
    Hand_Middle3 = 11
    Hand_Ring1 = 12
    Hand_Ring2 = 13
    Hand_Ring3 = 14
    Hand_Pinky0 = 15
    Hand_Pinky1 = 16
    Hand_Pinky2 = 17
    Hand_Pinky3 = 18
    Hand_MaxSkinnable = 19
    Hand_ThumbTip = 19
    Hand_IndexTip = 20
    Hand_MiddleTip = 21
    Hand_RingTip = 22
    Hand_PinkyTip = 23
    Hand_End = 24

    @classmethod
    def _missing_(cls, value):
        return "Hand_Unknown"


class XRHandBoneId(enum.IntEnum):
    """Specifies the bone IDs for an XR hand skeleton, following OpenXR standards."""

    XRHand_Start = 0
    XRHand_Palm = 0
    XRHand_Wrist = 1
    XRHand_ThumbMetacarpal = 2
    XRHand_ThumbProximal = 3
    XRHand_ThumbDistal = 4
    XRHand_ThumbTip = 5
    XRHand_IndexMetacarpal = 6
    XRHand_IndexProximal = 7
    XRHand_IndexIntermediate = 8
    XRHand_IndexDistal = 9
    XRHand_IndexTip = 10
    XRHand_MiddleMetacarpal = 11
    XRHand_MiddleProximal = 12
    XRHand_MiddleIntermediate = 13
    XRHand_MiddleDistal = 14
    XRHand_MiddleTip = 15
    XRHand_RingMetacarpal = 16
    XRHand_RingProximal = 17
    XRHand_RingIntermediate = 18
    XRHand_RingDistal = 19
    XRHand_RingTip = 20
    XRHand_LittleMetacarpal = 21
    XRHand_LittleProximal = 22
    XRHand_LittleIntermediate = 23
    XRHand_LittleDistal = 24
    XRHand_LittleTip = 25
    XRHand_Max = 26
    XRHand_End = 26

    @classmethod
    def _missing_(cls, value):
        return "XRHand_Unknown"


class BodyBoneId(enum.IntEnum):
    """Specifies the bone IDs for a body skeleton."""

    Body_Start = 0
    Body_Root = 0
    Body_Hips = 1
    Body_SpineLower = 2
    Body_SpineMiddle = 3
    Body_SpineUpper = 4
    Body_Chest = 5
    Body_Neck = 6
    Body_Head = 7
    Body_LeftShoulder = 8
    Body_LeftScapula = 9
    Body_LeftArmUpper = 10
    Body_LeftArmLower = 11
    Body_LeftHandWristTwist = 12
    Body_RightShoulder = 13
    Body_RightScapula = 14
    Body_RightArmUpper = 15
    Body_RightArmLower = 16
    Body_RightHandWristTwist = 17
    Body_LeftHandPalm = 18
    Body_LeftHandWrist = 19
    Body_LeftHandThumbMetacarpal = 20
    Body_LeftHandThumbProximal = 21
    Body_LeftHandThumbDistal = 22
    Body_LeftHandThumbTip = 23
    Body_LeftHandIndexMetacarpal = 24
    Body_LeftHandIndexProximal = 25
    Body_LeftHandIndexIntermediate = 26
    Body_LeftHandIndexDistal = 27
    Body_LeftHandIndexTip = 28
    Body_LeftHandMiddleMetacarpal = 29
    Body_LeftHandMiddleProximal = 30
    Body_LeftHandMiddleIntermediate = 31
    Body_LeftHandMiddleDistal = 32
    Body_LeftHandMiddleTip = 33
    Body_LeftHandRingMetacarpal = 34
    Body_LeftHandRingProximal = 35
    Body_LeftHandRingIntermediate = 36
    Body_LeftHandRingDistal = 37
    Body_LeftHandRingTip = 38
    Body_LeftHandLittleMetacarpal = 39
    Body_LeftHandLittleProximal = 40
    Body_LeftHandLittleIntermediate = 41
    Body_LeftHandLittleDistal = 42
    Body_LeftHandLittleTip = 43
    Body_RightHandPalm = 44
    Body_RightHandWrist = 45
    Body_RightHandThumbMetacarpal = 46
    Body_RightHandThumbProximal = 47
    Body_RightHandThumbDistal = 48
    Body_RightHandThumbTip = 49
    Body_RightHandIndexMetacarpal = 50
    Body_RightHandIndexProximal = 51
    Body_RightHandIndexIntermediate = 52
    Body_RightHandIndexDistal = 53
    Body_RightHandIndexTip = 54
    Body_RightHandMiddleMetacarpal = 55
    Body_RightHandMiddleProximal = 56
    Body_RightHandMiddleIntermediate = 57
    Body_RightHandMiddleDistal = 58
    Body_RightHandMiddleTip = 59
    Body_RightHandRingMetacarpal = 60
    Body_RightHandRingProximal = 61
    Body_RightHandRingIntermediate = 62
    Body_RightHandRingDistal = 63
    Body_RightHandRingTip = 64
    Body_RightHandLittleMetacarpal = 65
    Body_RightHandLittleProximal = 66
    Body_RightHandLittleIntermediate = 67
    Body_RightHandLittleDistal = 68
    Body_RightHandLittleTip = 69
    Body_End = 70

    @classmethod
    def _missing_(cls, value):
        return "Body_Unknown"


class FullBodyBoneId(enum.IntEnum):
    """Specifies the bone IDs for a full body skeleton, including legs and feet."""

    FullBody_Start = 0
    FullBody_Root = 0
    FullBody_Hips = 1
    FullBody_SpineLower = 2
    FullBody_SpineMiddle = 3
    FullBody_SpineUpper = 4
    FullBody_Chest = 5
    FullBody_Neck = 6
    FullBody_Head = 7
    FullBody_LeftShoulder = 8
    FullBody_LeftScapula = 9
    FullBody_LeftArmUpper = 10
    FullBody_LeftArmLower = 11
    FullBody_LeftHandWristTwist = 12
    FullBody_RightShoulder = 13
    FullBody_RightScapula = 14
    FullBody_RightArmUpper = 15
    FullBody_RightArmLower = 16
    FullBody_RightHandWristTwist = 17
    FullBody_LeftHandPalm = 18
    FullBody_LeftHandWrist = 19
    FullBody_LeftHandThumbMetacarpal = 20
    FullBody_LeftHandThumbProximal = 21
    FullBody_LeftHandThumbDistal = 22
    FullBody_LeftHandThumbTip = 23
    FullBody_LeftHandIndexMetacarpal = 24
    FullBody_LeftHandIndexProximal = 25
    FullBody_LeftHandIndexIntermediate = 26
    FullBody_LeftHandIndexDistal = 27
    FullBody_LeftHandIndexTip = 28
    FullBody_LeftHandMiddleMetacarpal = 29
    FullBody_LeftHandMiddleProximal = 30
    FullBody_LeftHandMiddleIntermediate = 31
    FullBody_LeftHandMiddleDistal = 32
    FullBody_LeftHandMiddleTip = 33
    FullBody_LeftHandRingMetacarpal = 34
    FullBody_LeftHandRingProximal = 35
    FullBody_LeftHandRingIntermediate = 36
    FullBody_LeftHandRingDistal = 37
    FullBody_LeftHandRingTip = 38
    FullBody_LeftHandLittleMetacarpal = 39
    FullBody_LeftHandLittleProximal = 40
    FullBody_LeftHandLittleIntermediate = 41
    FullBody_LeftHandLittleDistal = 42
    FullBody_LeftHandLittleTip = 43
    FullBody_RightHandPalm = 44
    FullBody_RightHandWrist = 45
    FullBody_RightHandThumbMetacarpal = 46
    FullBody_RightHandThumbProximal = 47
    FullBody_RightHandThumbDistal = 48
    FullBody_RightHandThumbTip = 49
    FullBody_RightHandIndexMetacarpal = 50
    FullBody_RightHandIndexProximal = 51
    FullBody_RightHandIndexIntermediate = 52
    FullBody_RightHandIndexDistal = 53
    FullBody_RightHandIndexTip = 54
    FullBody_RightHandMiddleMetacarpal = 55
    FullBody_RightHandMiddleProximal = 56
    FullBody_RightHandMiddleIntermediate = 57
    FullBody_RightHandMiddleDistal = 58
    FullBody_RightHandMiddleTip = 59
    FullBody_RightHandRingMetacarpal = 60
    FullBody_RightHandRingProximal = 61
    FullBody_RightHandRingIntermediate = 62
    FullBody_RightHandRingDistal = 63
    FullBody_RightHandRingTip = 64
    FullBody_RightHandLittleMetacarpal = 65
    FullBody_RightHandLittleProximal = 66
    FullBody_RightHandLittleIntermediate = 67
    FullBody_RightHandLittleDistal = 68
    FullBody_RightHandLittleTip = 69
    FullBody_LeftUpperLeg = 70
    FullBody_LeftLowerLeg = 71
    FullBody_LeftFootAnkleTwist = 72
    FullBody_LeftFootAnkle = 73
    FullBody_LeftFootSubtalar = 74
    FullBody_LeftFootTransverse = 75
    FullBody_LeftFootBall = 76
    FullBody_RightUpperLeg = 77
    FullBody_RightLowerLeg = 78
    FullBody_RightFootAnkleTwist = 79
    FullBody_RightFootAnkle = 80
    FullBody_RightFootSubtalar = 81
    FullBody_RightFootTransverse = 82
    FullBody_RightFootBall = 83
    FullBody_End = 84

    @classmethod
    def _missing_(cls, value):
        return "FullBody_Unknown"


# A type hint for any of the bone ID enums
AnyBoneId = HandBoneId | XRHandBoneId | BodyBoneId | FullBodyBoneId

# Aliases for brevity
HB = HandBoneId
BB = BodyBoneId
XHB = XRHandBoneId
FB = FullBodyBoneId


class SkeletonType(enum.Enum):
    """Corresponds to OVRSkeleton.SkeletonType, indicating the skeleton's nature."""

    None_ = -1
    HandLeft = 0
    HandRight = 1
    Body = 2
    FullBody = 3
    XRHandLeft = 4
    XRHandRight = 5


def get_bone_label(skeleton_type: SkeletonType, bone_id: int) -> str:
    """
    Returns the string name of any bone from the specific BoneId enums.

    Args:
        skeleton_type: The type of skeleton.
        bone_id: The integer ID of the bone.

    Returns:
        The official string name of the bone.
    """
    enum_class = None
    if skeleton_type in (SkeletonType.HandLeft, SkeletonType.HandRight):
        enum_class = HandBoneId
    elif skeleton_type in (SkeletonType.XRHandLeft, SkeletonType.XRHandRight):
        enum_class = XRHandBoneId
    elif skeleton_type == SkeletonType.Body:
        enum_class = BodyBoneId
    elif skeleton_type == SkeletonType.FullBody:
        enum_class = FullBodyBoneId
    else:
        return "Skeleton_Unknown"

    bone = enum_class(bone_id)
    # The _missing_ method returns a string for unknown bone IDs.
    if not isinstance(bone, enum.Enum):
        return str(bone)
    return bone.name


# --- Example Usage ---
if __name__ == "__main__":
    print("--- Refactored Skeleton Enums Example ---")

    # Example 1: Accessing a specific bone and its value
    wrist_bone = HB.Hand_WristRoot
    print(f"Hand bone: {wrist_bone.name}, Value: {wrist_bone.value}")

    # Example 2: Looking up a bone by its value in a specific enum
    # Note: Hand_ThumbTip and Hand_MaxSkinnable share value 19
    thumb_tip_bone = HB(19)
    print(f"Hand bone with value 19: {thumb_tip_bone.name} (Canonical is Hand_MaxSkinnable)")

    # Example 3: Demonstrating type safety and clarity
    spine_bone = BB.Body_SpineUpper
    print(
        "Body bone: "
        f"{get_bone_label(SkeletonType.Body, spine_bone.value)}, "
        f"Value: {spine_bone.value}"
    )

    xr_thumb_tip = XHB.XRHand_ThumbTip
    # Note: XRHand can be for left or right, the enum is the same.
    print(
        "XR Hand bone: "
        f"{get_bone_label(SkeletonType.XRHandLeft, xr_thumb_tip.value)}, "
        f"Value: {xr_thumb_tip.value}"
    )

    # Aliasing is now contained within each enum
    print("-" * 20)
    print("Demonstrating aliasing within a single enum:")
    print(f"Is Hand_WristRoot the same as Hand_Start? {HB.Hand_WristRoot is HB.Hand_Start}")

    print("\nDemonstrating that different enums are distinct:")
    try:
        # This comparison across different enum types is not meaningful
        # and highlights the improved type safety.
        print("Comparing HandBoneId.Hand_Start to BodyBoneId.Body_Start...")
        are_equal = HandBoneId.Hand_Start == BodyBoneId.Body_Start
        print(f"Are they equal? {are_equal}")  # This will likely be False

    except Exception as e:
        print(f"An error occurred: {e}")

    # Example of how to robustly check for a bone's identity
    selected_bone = BodyBoneId.Body_Root
    if selected_bone is BodyBoneId.Body_Root:
        print("\nCorrectly identified Body_Root using 'is'.")

FULL_BODY_SKELETON_CONNECTIONS = [
    # Spine
    (FB.FullBody_Hips, FB.FullBody_SpineLower),
    (FB.FullBody_SpineLower, FB.FullBody_SpineMiddle),
    (FB.FullBody_SpineMiddle, FB.FullBody_SpineUpper),
    (FB.FullBody_SpineUpper, FB.FullBody_Chest),
    # Head
    (FB.FullBody_Chest, FB.FullBody_Neck),
    (FB.FullBody_Neck, FB.FullBody_Head),
    # Left arm
    (FB.FullBody_Chest, FB.FullBody_LeftShoulder),
    (FB.FullBody_LeftShoulder, FB.FullBody_LeftArmUpper),
    (FB.FullBody_LeftArmUpper, FB.FullBody_LeftArmLower),
    (FB.FullBody_LeftArmLower, FB.FullBody_LeftHandWrist),
    # Right arm
    (FB.FullBody_Chest, FB.FullBody_RightShoulder),
    (FB.FullBody_RightShoulder, FB.FullBody_RightArmUpper),
    (FB.FullBody_RightArmUpper, FB.FullBody_RightArmLower),
    (FB.FullBody_RightArmLower, FB.FullBody_RightHandWrist),
    # Left leg
    (FB.FullBody_Hips, FB.FullBody_LeftUpperLeg),
    (FB.FullBody_LeftUpperLeg, FB.FullBody_LeftLowerLeg),
    (FB.FullBody_LeftLowerLeg, FB.FullBody_LeftFootAnkle),
    (FB.FullBody_LeftFootAnkle, FB.FullBody_LeftFootBall),
    # Right leg
    (FB.FullBody_Hips, FB.FullBody_RightUpperLeg),
    (FB.FullBody_RightUpperLeg, FB.FullBody_RightLowerLeg),
    (FB.FullBody_RightLowerLeg, FB.FullBody_RightFootAnkle),
    (FB.FullBody_RightFootAnkle, FB.FullBody_RightFootBall),
    # Left Hand
    (FB.FullBody_LeftHandWrist, FB.FullBody_LeftHandThumbMetacarpal),
    (FB.FullBody_LeftHandThumbMetacarpal, FB.FullBody_LeftHandThumbProximal),
    (FB.FullBody_LeftHandThumbProximal, FB.FullBody_LeftHandThumbDistal),
    (FB.FullBody_LeftHandThumbDistal, FB.FullBody_LeftHandThumbTip),
    (FB.FullBody_LeftHandWrist, FB.FullBody_LeftHandIndexMetacarpal),
    (FB.FullBody_LeftHandIndexMetacarpal, FB.FullBody_LeftHandIndexProximal),
    (FB.FullBody_LeftHandIndexProximal, FB.FullBody_LeftHandIndexIntermediate),
    (FB.FullBody_LeftHandIndexIntermediate, FB.FullBody_LeftHandIndexDistal),
    (FB.FullBody_LeftHandIndexDistal, FB.FullBody_LeftHandIndexTip),
    (FB.FullBody_LeftHandWrist, FB.FullBody_LeftHandMiddleMetacarpal),
    (FB.FullBody_LeftHandMiddleMetacarpal, FB.FullBody_LeftHandMiddleProximal),
    (FB.FullBody_LeftHandMiddleProximal, FB.FullBody_LeftHandMiddleIntermediate),
    (FB.FullBody_LeftHandMiddleIntermediate, FB.FullBody_LeftHandMiddleDistal),
    (FB.FullBody_LeftHandMiddleDistal, FB.FullBody_LeftHandMiddleTip),
    (FB.FullBody_LeftHandWrist, FB.FullBody_LeftHandRingMetacarpal),
    (FB.FullBody_LeftHandRingMetacarpal, FB.FullBody_LeftHandRingProximal),
    (FB.FullBody_LeftHandRingProximal, FB.FullBody_LeftHandRingIntermediate),
    (FB.FullBody_LeftHandRingIntermediate, FB.FullBody_LeftHandRingDistal),
    (FB.FullBody_LeftHandRingDistal, FB.FullBody_LeftHandRingTip),
    (FB.FullBody_LeftHandWrist, FB.FullBody_LeftHandLittleMetacarpal),
    (FB.FullBody_LeftHandLittleMetacarpal, FB.FullBody_LeftHandLittleProximal),
    (FB.FullBody_LeftHandLittleProximal, FB.FullBody_LeftHandLittleIntermediate),
    (FB.FullBody_LeftHandLittleIntermediate, FB.FullBody_LeftHandLittleDistal),
    (FB.FullBody_LeftHandLittleDistal, FB.FullBody_LeftHandLittleTip),
    # Right Hand
    (FB.FullBody_RightHandWrist, FB.FullBody_RightHandThumbMetacarpal),
    (FB.FullBody_RightHandThumbMetacarpal, FB.FullBody_RightHandThumbProximal),
    (FB.FullBody_RightHandThumbProximal, FB.FullBody_RightHandThumbDistal),
    (FB.FullBody_RightHandThumbDistal, FB.FullBody_RightHandThumbTip),
    (FB.FullBody_RightHandWrist, FB.FullBody_RightHandIndexMetacarpal),
    (FB.FullBody_RightHandIndexMetacarpal, FB.FullBody_RightHandIndexProximal),
    (FB.FullBody_RightHandIndexProximal, FB.FullBody_RightHandIndexIntermediate),
    (FB.FullBody_RightHandIndexIntermediate, FB.FullBody_RightHandIndexDistal),
    (FB.FullBody_RightHandIndexDistal, FB.FullBody_RightHandIndexTip),
    (FB.FullBody_RightHandWrist, FB.FullBody_RightHandMiddleMetacarpal),
    (FB.FullBody_RightHandMiddleMetacarpal, FB.FullBody_RightHandMiddleProximal),
    (FB.FullBody_RightHandMiddleProximal, FB.FullBody_RightHandMiddleIntermediate),
    (FB.FullBody_RightHandMiddleIntermediate, FB.FullBody_RightHandMiddleDistal),
    (FB.FullBody_RightHandMiddleDistal, FB.FullBody_RightHandMiddleTip),
    (FB.FullBody_RightHandWrist, FB.FullBody_RightHandRingMetacarpal),
    (FB.FullBody_RightHandRingMetacarpal, FB.FullBody_RightHandRingProximal),
    (FB.FullBody_RightHandRingProximal, FB.FullBody_RightHandRingIntermediate),
    (FB.FullBody_RightHandRingIntermediate, FB.FullBody_RightHandRingDistal),
    (FB.FullBody_RightHandRingDistal, FB.FullBody_RightHandRingTip),
    (FB.FullBody_RightHandWrist, FB.FullBody_RightHandLittleMetacarpal),
    (FB.FullBody_RightHandLittleMetacarpal, FB.FullBody_RightHandLittleProximal),
    (FB.FullBody_RightHandLittleProximal, FB.FullBody_RightHandLittleIntermediate),
    (FB.FullBody_RightHandLittleIntermediate, FB.FullBody_RightHandLittleDistal),
    (FB.FullBody_RightHandLittleDistal, FB.FullBody_RightHandLittleTip),
]
