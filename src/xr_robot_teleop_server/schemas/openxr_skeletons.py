"""
Defines Python enumerations for tracking skeleton bones in XR applications.

This module provides standardized integer constants for identifying bones in
various skeleton types, including legacy OVR hand, OpenXR hand, and full-body
skeletons. These enums are essential for interpreting tracking data from XR
devices and APIs.

The definitions, particularly for `HandBoneId`, are aligned with the
conventions established by the OpenXR specification. For more details on the
underlying standards, see the official OpenXR documentation:
https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html

Key Enums:
- OVRHandBoneId: For legacy OVR hand tracking systems.
- HandBoneId: For hand tracking compliant with the OpenXR standard.
- BodyBoneId: For upper-body tracking.
- FullBodyBoneId: For full-body tracking, including legs.
- SkeletonType: To distinguish between the different kinds of skeletons.
"""

import enum


class OVRHandBoneId(enum.IntEnum):
    """Specifies the bone IDs for a legacy OVR hand skeleton."""

    OVRHand_Start = 0
    OVRHand_WristRoot = 0
    OVRHand_ForearmStub = 1
    OVRHand_Thumb0 = 2
    OVRHand_Thumb1 = 3
    OVRHand_Thumb2 = 4
    OVRHand_Thumb3 = 5
    OVRHand_Index1 = 6
    OVRHand_Index2 = 7
    OVRHand_Index3 = 8
    OVRHand_Middle1 = 9
    OVRHand_Middle2 = 10
    OVRHand_Middle3 = 11
    OVRHand_Ring1 = 12
    OVRHand_Ring2 = 13
    OVRHand_Ring3 = 14
    OVRHand_Pinky0 = 15
    OVRHand_Pinky1 = 16
    OVRHand_Pinky2 = 17
    OVRHand_Pinky3 = 18
    OVRHand_MaxSkinnable = 19
    OVRHand_ThumbTip = 19
    OVRHand_IndexTip = 20
    OVRHand_MiddleTip = 21
    OVRHand_RingTip = 22
    OVRHand_PinkyTip = 23
    OVRHand_End = 24


class HandBoneId(enum.IntEnum):
    """Specifies the bone IDs for a hand skeleton, following OpenXR standards."""

    Hand_Start = 0
    Hand_Palm = 0
    Hand_Wrist = 1
    Hand_ThumbMetacarpal = 2
    Hand_ThumbProximal = 3
    Hand_ThumbDistal = 4
    Hand_ThumbTip = 5
    Hand_IndexMetacarpal = 6
    Hand_IndexProximal = 7
    Hand_IndexIntermediate = 8
    Hand_IndexDistal = 9
    Hand_IndexTip = 10
    Hand_MiddleMetacarpal = 11
    Hand_MiddleProximal = 12
    Hand_MiddleIntermediate = 13
    Hand_MiddleDistal = 14
    Hand_MiddleTip = 15
    Hand_RingMetacarpal = 16
    Hand_RingProximal = 17
    Hand_RingIntermediate = 18
    Hand_RingDistal = 19
    Hand_RingTip = 20
    Hand_LittleMetacarpal = 21
    Hand_LittleProximal = 22
    Hand_LittleIntermediate = 23
    Hand_LittleDistal = 24
    Hand_LittleTip = 25
    Hand_Max = 26
    Hand_End = 26


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


# A type hint for any of the bone ID enums
AnyBoneId = OVRHandBoneId | HandBoneId | BodyBoneId | FullBodyBoneId

# Aliases for brevity
OVRHB = OVRHandBoneId
BB = BodyBoneId
HB = HandBoneId
FB = FullBodyBoneId


class SkeletonType(enum.Enum):
    """Corresponds to OVRSkeleton.SkeletonType, indicating the skeleton's nature."""

    None_ = -1
    OVRHandLeft = 0
    OVRHandRight = 1
    Body = 2
    FullBody = 3
    HandLeft = 4
    HandRight = 5


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
    if skeleton_type in (SkeletonType.OVRHandLeft, SkeletonType.OVRHandRight):
        enum_class = OVRHandBoneId
    elif skeleton_type in (SkeletonType.HandLeft, SkeletonType.HandRight):
        enum_class = HandBoneId
    elif skeleton_type == SkeletonType.Body:
        enum_class = BodyBoneId
    elif skeleton_type == SkeletonType.FullBody:
        enum_class = FullBodyBoneId
    else:
        return "Skeleton_Unknown"

    try:
        bone = enum_class(bone_id)
    except ValueError:
        prefix = enum_class.__name__
        if prefix.endswith("BoneId"):
            prefix = prefix[: -len("BoneId")]
        return f"{prefix}_Unknown"
    return bone.name


# Parent-child relationships for each bone in the skeleton.
OVR_HAND_SKELETON_CONNECTIONS = [
    # Fingers
    (OVRHB.OVRHand_WristRoot, OVRHB.OVRHand_Thumb0),
    (OVRHB.OVRHand_Thumb0, OVRHB.OVRHand_Thumb1),
    (OVRHB.OVRHand_Thumb1, OVRHB.OVRHand_Thumb2),
    (OVRHB.OVRHand_Thumb2, OVRHB.OVRHand_Thumb3),
    (OVRHB.OVRHand_Thumb3, OVRHB.OVRHand_ThumbTip),
    (OVRHB.OVRHand_WristRoot, OVRHB.OVRHand_Index1),
    (OVRHB.OVRHand_Index1, OVRHB.OVRHand_Index2),
    (OVRHB.OVRHand_Index2, OVRHB.OVRHand_Index3),
    (OVRHB.OVRHand_Index3, OVRHB.OVRHand_IndexTip),
    (OVRHB.OVRHand_WristRoot, OVRHB.OVRHand_Middle1),
    (OVRHB.OVRHand_Middle1, OVRHB.OVRHand_Middle2),
    (OVRHB.OVRHand_Middle2, OVRHB.OVRHand_Middle3),
    (OVRHB.OVRHand_Middle3, OVRHB.OVRHand_MiddleTip),
    (OVRHB.OVRHand_WristRoot, OVRHB.OVRHand_Ring1),
    (OVRHB.OVRHand_Ring1, OVRHB.OVRHand_Ring2),
    (OVRHB.OVRHand_Ring2, OVRHB.OVRHand_Ring3),
    (OVRHB.OVRHand_Ring3, OVRHB.OVRHand_RingTip),
    (OVRHB.OVRHand_WristRoot, OVRHB.OVRHand_Pinky0),
    (OVRHB.OVRHand_Pinky0, OVRHB.OVRHand_Pinky1),
    (OVRHB.OVRHand_Pinky1, OVRHB.OVRHand_Pinky2),
    (OVRHB.OVRHand_Pinky2, OVRHB.OVRHand_Pinky3),
    (OVRHB.OVRHand_Pinky3, OVRHB.OVRHand_PinkyTip),
    # Arm
    (OVRHB.OVRHand_WristRoot, OVRHB.OVRHand_ForearmStub),
]

HAND_SKELETON_CONNECTIONS = [
    # Thumb
    (HB.Hand_Wrist, HB.Hand_ThumbMetacarpal),
    (HB.Hand_ThumbMetacarpal, HB.Hand_ThumbProximal),
    (HB.Hand_ThumbProximal, HB.Hand_ThumbDistal),
    (HB.Hand_ThumbDistal, HB.Hand_ThumbTip),
    # Index
    (HB.Hand_Wrist, HB.Hand_IndexMetacarpal),
    (HB.Hand_IndexMetacarpal, HB.Hand_IndexProximal),
    (HB.Hand_IndexProximal, HB.Hand_IndexIntermediate),
    (HB.Hand_IndexIntermediate, HB.Hand_IndexDistal),
    (HB.Hand_IndexDistal, HB.Hand_IndexTip),
    # Middle
    (HB.Hand_Wrist, HB.Hand_MiddleMetacarpal),
    (HB.Hand_MiddleMetacarpal, HB.Hand_MiddleProximal),
    (HB.Hand_MiddleProximal, HB.Hand_MiddleIntermediate),
    (HB.Hand_MiddleIntermediate, HB.Hand_MiddleDistal),
    (HB.Hand_MiddleDistal, HB.Hand_MiddleTip),
    # Ring
    (HB.Hand_Wrist, HB.Hand_RingMetacarpal),
    (HB.Hand_RingMetacarpal, HB.Hand_RingProximal),
    (HB.Hand_RingProximal, HB.Hand_RingIntermediate),
    (HB.Hand_RingIntermediate, HB.Hand_RingDistal),
    (HB.Hand_RingDistal, HB.Hand_RingTip),
    # Little
    (HB.Hand_Wrist, HB.Hand_LittleMetacarpal),
    (HB.Hand_LittleMetacarpal, HB.Hand_LittleProximal),
    (HB.Hand_LittleProximal, HB.Hand_LittleIntermediate),
    (HB.Hand_LittleIntermediate, HB.Hand_LittleDistal),
    (HB.Hand_LittleDistal, HB.Hand_LittleTip),
    # Palm
    (HB.Hand_Wrist, HB.Hand_Palm),
]

BODY_SKELETON_CONNECTIONS = [
    # Spine
    (BB.Body_Root, BB.Body_Hips),
    (BB.Body_Hips, BB.Body_SpineLower),
    (BB.Body_SpineLower, BB.Body_SpineMiddle),
    (BB.Body_SpineMiddle, BB.Body_SpineUpper),
    (BB.Body_SpineUpper, BB.Body_Chest),
    # Head
    (BB.Body_Chest, BB.Body_Neck),
    (BB.Body_Neck, BB.Body_Head),
    # Left arm
    (BB.Body_Chest, BB.Body_LeftShoulder),
    (BB.Body_LeftShoulder, BB.Body_LeftScapula),
    (BB.Body_LeftScapula, BB.Body_LeftArmUpper),
    (BB.Body_LeftArmUpper, BB.Body_LeftArmLower),
    (BB.Body_LeftArmLower, BB.Body_LeftHandWristTwist),
    (BB.Body_LeftHandWristTwist, BB.Body_LeftHandWrist),
    # Right arm
    (BB.Body_Chest, BB.Body_RightShoulder),
    (BB.Body_RightShoulder, BB.Body_RightScapula),
    (BB.Body_RightScapula, BB.Body_RightArmUpper),
    (BB.Body_RightArmUpper, BB.Body_RightArmLower),
    (BB.Body_RightArmLower, BB.Body_RightHandWristTwist),
    (BB.Body_RightHandWristTwist, BB.Body_RightHandWrist),
    # Left Hand
    (BB.Body_LeftHandWrist, BB.Body_LeftHandPalm),
    (BB.Body_LeftHandWrist, BB.Body_LeftHandThumbMetacarpal),
    (BB.Body_LeftHandThumbMetacarpal, BB.Body_LeftHandThumbProximal),
    (BB.Body_LeftHandThumbProximal, BB.Body_LeftHandThumbDistal),
    (BB.Body_LeftHandThumbDistal, BB.Body_LeftHandThumbTip),
    (BB.Body_LeftHandWrist, BB.Body_LeftHandIndexMetacarpal),
    (BB.Body_LeftHandIndexMetacarpal, BB.Body_LeftHandIndexProximal),
    (BB.Body_LeftHandIndexProximal, BB.Body_LeftHandIndexIntermediate),
    (BB.Body_LeftHandIndexIntermediate, BB.Body_LeftHandIndexDistal),
    (BB.Body_LeftHandIndexDistal, BB.Body_LeftHandIndexTip),
    (BB.Body_LeftHandWrist, BB.Body_LeftHandMiddleMetacarpal),
    (BB.Body_LeftHandMiddleMetacarpal, BB.Body_LeftHandMiddleProximal),
    (BB.Body_LeftHandMiddleProximal, BB.Body_LeftHandMiddleIntermediate),
    (BB.Body_LeftHandMiddleIntermediate, BB.Body_LeftHandMiddleDistal),
    (BB.Body_LeftHandMiddleDistal, BB.Body_LeftHandMiddleTip),
    (BB.Body_LeftHandWrist, BB.Body_LeftHandRingMetacarpal),
    (BB.Body_LeftHandRingMetacarpal, BB.Body_LeftHandRingProximal),
    (BB.Body_LeftHandRingProximal, BB.Body_LeftHandRingIntermediate),
    (BB.Body_LeftHandRingIntermediate, BB.Body_LeftHandRingDistal),
    (BB.Body_LeftHandRingDistal, BB.Body_LeftHandRingTip),
    (BB.Body_LeftHandWrist, BB.Body_LeftHandLittleMetacarpal),
    (BB.Body_LeftHandLittleMetacarpal, BB.Body_LeftHandLittleProximal),
    (BB.Body_LeftHandLittleProximal, BB.Body_LeftHandLittleIntermediate),
    (BB.Body_LeftHandLittleIntermediate, BB.Body_LeftHandLittleDistal),
    (BB.Body_LeftHandLittleDistal, BB.Body_LeftHandLittleTip),
    # Right Hand
    (BB.Body_RightHandWrist, BB.Body_RightHandPalm),
    (BB.Body_RightHandWrist, BB.Body_RightHandThumbMetacarpal),
    (BB.Body_RightHandThumbMetacarpal, BB.Body_RightHandThumbProximal),
    (BB.Body_RightHandThumbProximal, BB.Body_RightHandThumbDistal),
    (BB.Body_RightHandThumbDistal, BB.Body_RightHandThumbTip),
    (BB.Body_RightHandWrist, BB.Body_RightHandIndexMetacarpal),
    (BB.Body_RightHandIndexMetacarpal, BB.Body_RightHandIndexProximal),
    (BB.Body_RightHandIndexProximal, BB.Body_RightHandIndexIntermediate),
    (BB.Body_RightHandIndexIntermediate, BB.Body_RightHandIndexDistal),
    (BB.Body_RightHandIndexDistal, BB.Body_RightHandIndexTip),
    (BB.Body_RightHandWrist, BB.Body_RightHandMiddleMetacarpal),
    (BB.Body_RightHandMiddleMetacarpal, BB.Body_RightHandMiddleProximal),
    (BB.Body_RightHandMiddleProximal, BB.Body_RightHandMiddleIntermediate),
    (BB.Body_RightHandMiddleIntermediate, BB.Body_RightHandMiddleDistal),
    (BB.Body_RightHandMiddleDistal, BB.Body_RightHandMiddleTip),
    (BB.Body_RightHandWrist, BB.Body_RightHandRingMetacarpal),
    (BB.Body_RightHandRingMetacarpal, BB.Body_RightHandRingProximal),
    (BB.Body_RightHandRingProximal, BB.Body_RightHandRingIntermediate),
    (BB.Body_RightHandRingIntermediate, BB.Body_RightHandRingDistal),
    (BB.Body_RightHandRingDistal, BB.Body_RightHandRingTip),
    (BB.Body_RightHandWrist, BB.Body_RightHandLittleMetacarpal),
    (BB.Body_RightHandLittleMetacarpal, BB.Body_RightHandLittleProximal),
    (BB.Body_RightHandLittleProximal, BB.Body_RightHandLittleIntermediate),
    (BB.Body_RightHandLittleIntermediate, BB.Body_RightHandLittleDistal),
    (BB.Body_RightHandLittleDistal, BB.Body_RightHandLittleTip),
]

FULL_BODY_SKELETON_CONNECTIONS = [
    # Spine
    (FB.FullBody_Root, FB.FullBody_Hips),
    (FB.FullBody_Hips, FB.FullBody_SpineLower),
    (FB.FullBody_SpineLower, FB.FullBody_SpineMiddle),
    (FB.FullBody_SpineMiddle, FB.FullBody_SpineUpper),
    (FB.FullBody_SpineUpper, FB.FullBody_Chest),
    # Head
    (FB.FullBody_Chest, FB.FullBody_Neck),
    (FB.FullBody_Neck, FB.FullBody_Head),
    # Left arm
    (FB.FullBody_Chest, FB.FullBody_LeftShoulder),
    (FB.FullBody_LeftShoulder, FB.FullBody_LeftScapula),
    (FB.FullBody_LeftScapula, FB.FullBody_LeftArmUpper),
    (FB.FullBody_LeftArmUpper, FB.FullBody_LeftArmLower),
    (FB.FullBody_LeftArmLower, FB.FullBody_LeftHandWristTwist),
    (FB.FullBody_LeftHandWristTwist, FB.FullBody_LeftHandWrist),
    # Right arm
    (FB.FullBody_Chest, FB.FullBody_RightShoulder),
    (FB.FullBody_RightShoulder, FB.FullBody_RightScapula),
    (FB.FullBody_RightScapula, FB.FullBody_RightArmUpper),
    (FB.FullBody_RightArmUpper, FB.FullBody_RightArmLower),
    (FB.FullBody_RightArmLower, FB.FullBody_RightHandWristTwist),
    (FB.FullBody_RightHandWristTwist, FB.FullBody_RightHandWrist),
    # Left leg
    (FB.FullBody_Hips, FB.FullBody_LeftUpperLeg),
    (FB.FullBody_LeftUpperLeg, FB.FullBody_LeftLowerLeg),
    (FB.FullBody_LeftLowerLeg, FB.FullBody_LeftFootAnkleTwist),
    (FB.FullBody_LeftFootAnkleTwist, FB.FullBody_LeftFootAnkle),
    (FB.FullBody_LeftFootAnkle, FB.FullBody_LeftFootSubtalar),
    (FB.FullBody_LeftFootSubtalar, FB.FullBody_LeftFootTransverse),
    (FB.FullBody_LeftFootTransverse, FB.FullBody_LeftFootBall),
    # Right leg
    (FB.FullBody_Hips, FB.FullBody_RightUpperLeg),
    (FB.FullBody_RightUpperLeg, FB.FullBody_RightLowerLeg),
    (FB.FullBody_RightLowerLeg, FB.FullBody_RightFootAnkleTwist),
    (FB.FullBody_RightFootAnkleTwist, FB.FullBody_RightFootAnkle),
    (FB.FullBody_RightFootAnkle, FB.FullBody_RightFootSubtalar),
    (FB.FullBody_RightFootSubtalar, FB.FullBody_RightFootTransverse),
    (FB.FullBody_RightFootTransverse, FB.FullBody_RightFootBall),
    # Left Hand
    (FB.FullBody_LeftHandWrist, FB.FullBody_LeftHandPalm),
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
    (FB.FullBody_RightHandWrist, FB.FullBody_RightHandPalm),
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


def get_skeleton_connections(skeleton_type: SkeletonType):
    """Returns the bone connections for a given skeleton type."""
    if skeleton_type in (SkeletonType.OVRHandLeft, SkeletonType.OVRHandRight):
        return OVR_HAND_SKELETON_CONNECTIONS
    if skeleton_type in (SkeletonType.HandLeft, SkeletonType.HandRight):
        return HAND_SKELETON_CONNECTIONS
    if skeleton_type == SkeletonType.Body:
        return BODY_SKELETON_CONNECTIONS
    if skeleton_type == SkeletonType.FullBody:
        return FULL_BODY_SKELETON_CONNECTIONS
    return []


# --- Example Usage ---
if __name__ == "__main__":
    print("--- Refactored Skeleton Enums Example ---")

    # Example 1: Accessing a specific bone and its value
    wrist_bone = OVRHB.OVRHand_WristRoot
    print(f"OVR Hand bone: {wrist_bone.name}, Value: {wrist_bone.value}")

    # Example 2: Looking up a bone by its value in a specific enum
    # Note: OVRHand_ThumbTip and OVRHand_MaxSkinnable share value 19
    thumb_tip_bone = OVRHB(19)
    print(f"OVR Hand bone with value 19: {thumb_tip_bone.name} (Canonical is OVRHand_MaxSkinnable)")

    # Example 3: Demonstrating type safety and clarity
    spine_bone = BB.Body_SpineUpper
    print(
        "Body bone: "
        f"{get_bone_label(SkeletonType.Body, spine_bone.value)}, "
        f"Value: {spine_bone.value}"
    )

    thumb_tip = HB.Hand_ThumbTip
    # Note: Hand can be for left or right, the enum is the same.
    print(
        "Hand bone: "
        f"{get_bone_label(SkeletonType.HandLeft, thumb_tip.value)}, "
        f"Value: {thumb_tip.value}"
    )

    # Aliasing is now contained within each enum
    print("-" * 20)
    print("Demonstrating aliasing within a single enum:")
    print(
        "Is OVRHand_WristRoot the same as OVRHand_Start? "
        f"{OVRHB.OVRHand_WristRoot is OVRHB.OVRHand_Start}"
    )

    print("\nDemonstrating that different enums are distinct:")
    try:
        # This comparison across different enum types is not meaningful
        # and highlights the improved type safety.
        print("Comparing OVRHandBoneId.OVRHand_Start to BodyBoneId.Body_Start...")
        are_equal = OVRHandBoneId.OVRHand_Start == BodyBoneId.Body_Start
        print(f"Are they equal? {are_equal}")  # This will likely be False

    except Exception as e:
        print(f"An error occurred: {e}")

    # Example of how to robustly check for a bone's identity
    selected_bone = BodyBoneId.Body_Root
    if selected_bone is BodyBoneId.Body_Root:
        print("\nCorrectly identified Body_Root using 'is'.")

    # Example 4: Get skeleton connections
    print("\n--- Skeleton Connections Example ---")
    full_body_connections = get_skeleton_connections(SkeletonType.FullBody)
    print(f"Number of connections in FullBody skeleton: {len(full_body_connections)}")
    print("First 5 connections:")
    for parent, child in full_body_connections[:5]:
        print(f"  {parent.name} -> {child.name}")

    hand_connections = get_skeleton_connections(SkeletonType.HandLeft)
    print(f"\nNumber of connections in Hand skeleton: {len(hand_connections)}")
    print("First 5 connections:")
    for parent, child in hand_connections[:5]:
        print(f"  {parent.name} -> {child.name}")
