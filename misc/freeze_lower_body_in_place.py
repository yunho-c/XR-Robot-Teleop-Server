"""
Freeze selected lower-body bones to a static pose directly inside Blender.

Run this from Blender's scripting workspace. The script optionally duplicates
the active action before editing so that the original animation stays intact.
"""

import bpy

# The name of armature object in Blender
ARMATURE_NAME = "CHANGE THIS"

# When True, the script copies the current action before modifying it.
# Disable if you intentionally want to override keyframes on the existing action.
DUPLICATE_ACTION = True

# Set USE_STATIC_LOWER_BODY to True to freeze the lower body at a single frame
# (default scene.frame_start), allowing only the upper body to animate.
USE_STATIC_LOWER_BODY = True

# Names of the lower-body bones whose motion should be frozen. These defaults
# include both MMD-style rigs and BVH human pose dataset rigs.
LOWER_BODY_ROOT_BONES = [
    "下半身",
    "左足",
    "右足",
    "左ひざ",
    "右ひざ",
    "左足首",
    "右足首",
    "左つま先",
    "右つま先",
    # BVH human pose dataset
    "LeftUpLeg",
    "RightUpLeg",
    "LeftLeg",
    "RightLeg",
    "LeftFootMod",
    "RightFootMod",
]

# Additional bones to freeze even if they are not descendants of the root list.
LOWER_BODY_ADDITIONAL_BONES = [
    "センター",
    "上半身",
    "上半身2",
    "首",
    "肩.L",
    "肩.R",
    "腕.L",
    "腕.R",
    # BVH human pose dataset
    "Hips",
    "Spine2",
    # "LeftArm",
    # "RightArm",
    # "LeftForeArm",
    # "RightForeArm",
    # "LeftHand",
    # "RightHand",
]

# When True, every descendant of each entry in LOWER_BODY_ROOT_BONES is frozen.
LOWER_BODY_INCLUDE_CHILDREN = True

# Override the frame used to capture the static lower-body pose. Use None to
# fall back to the scene's start frame.
LOWER_BODY_FREEZE_FRAME = None

# When True, existing keyframes on the frozen bones are removed before
# inserting the static pose keys.
CLEAR_EXISTING_KEYFRAMES = True


def gather_lower_body_bone_names(armature_obj):
    """
    Build the set of bone names whose transforms should be frozen for the
    lower body option.
    """
    pose_bones = armature_obj.pose.bones
    lower_body_names = set()
    missing_bones = set()

    for bone_name in LOWER_BODY_ROOT_BONES:
        pbone = pose_bones.get(bone_name)
        if not pbone:
            missing_bones.add(bone_name)
            continue

        stack = [pbone]
        while stack:
            current = stack.pop()
            if current.name in lower_body_names:
                continue
            lower_body_names.add(current.name)
            if LOWER_BODY_INCLUDE_CHILDREN:
                stack.extend(current.children)

    for bone_name in LOWER_BODY_ADDITIONAL_BONES:
        pbone = pose_bones.get(bone_name)
        if not pbone:
            missing_bones.add(bone_name)
            continue
        lower_body_names.add(pbone.name)

    if missing_bones:
        missing_list = ", ".join(sorted(missing_bones))
        print(f"Warning: Lower body bone(s) not found on armature: {missing_list}")

    return lower_body_names


def remove_keyframes_for_bones(action, bone_transforms):
    """Remove fcurves for the specified bones based on captured data."""
    if not action:
        return
    data_path_prefixes = []
    for bone_name, data in bone_transforms.items():
        bone_prefix = f'pose.bones["{bone_name}"].'
        data_path_prefixes.append(f"{bone_prefix}location")
        data_path_prefixes.append(f"{bone_prefix}{data['rotation_path']}")
        data_path_prefixes.append(f"{bone_prefix}scale")
    fcurves_to_remove = [
        fcurve
        for fcurve in list(action.fcurves)
        for prefix in data_path_prefixes
        if fcurve.data_path == prefix
    ]
    for fcurve in fcurves_to_remove:
        action.fcurves.remove(fcurve)


def capture_pose_transforms(armature_obj, bone_names):
    """Capture location, rotation (respecting mode), and scale for the given bones."""
    transforms = {}
    for bone_name in bone_names:
        pbone = armature_obj.pose.bones.get(bone_name)
        if not pbone:
            continue

        rotation_mode = pbone.rotation_mode
        if rotation_mode == "QUATERNION":
            rotation_attr = "rotation_quaternion"
            rotation_values = tuple(pbone.rotation_quaternion)
        elif rotation_mode == "AXIS_ANGLE":
            rotation_attr = "rotation_axis_angle"
            rotation_values = tuple(pbone.rotation_axis_angle)
        else:
            rotation_attr = "rotation_euler"
            rotation_values = tuple(pbone.rotation_euler)

        transforms[bone_name] = {
            "location": tuple(pbone.location),
            "rotation": rotation_values,
            "rotation_mode": rotation_mode,
            "rotation_attr": rotation_attr,
            "rotation_path": rotation_attr,
            "scale": tuple(pbone.scale),
        }
    return transforms


def freeze_lower_body_in_place(
    armature_name=ARMATURE_NAME,
    duplicate_action=DUPLICATE_ACTION,
    use_static_lower_body=USE_STATIC_LOWER_BODY,
    lower_body_freeze_frame=LOWER_BODY_FREEZE_FRAME,
    clear_existing_keyframes=CLEAR_EXISTING_KEYFRAMES,
):
    """Freeze configured lower-body bones across the animation."""
    if not use_static_lower_body:
        print("USE_STATIC_LOWER_BODY is False; nothing to do.")
        return

    armature_obj = bpy.data.objects.get(armature_name)
    if not armature_obj or armature_obj.type != "ARMATURE":
        print(f"Error: Armature object '{armature_name}' not found.")
        return

    if not armature_obj.animation_data or not armature_obj.animation_data.action:
        print(f"Error: Armature '{armature_name}' has no animation data.")
        return

    action = armature_obj.animation_data.action
    if duplicate_action:
        action = action.copy()
        action.name = f"{action.name}_static_lower_body"
        armature_obj.animation_data.action = action
        print(f"Created duplicate action: {action.name}")
    else:
        print(f"Modifying existing action: {action.name}")

    scene = bpy.context.scene
    start_frame = int(scene.frame_start)
    end_frame = int(scene.frame_end)
    original_frame = scene.frame_current

    lower_body_bone_names = gather_lower_body_bone_names(armature_obj)
    if not lower_body_bone_names:
        print("Warning: No lower body bones resolved; aborting.")
        return

    freeze_frame = lower_body_freeze_frame
    if freeze_frame is None:
        freeze_frame = start_frame
    else:
        freeze_frame = int(freeze_frame)
        if freeze_frame < start_frame or freeze_frame > end_frame:
            print(
                f"Warning: LOWER_BODY_FREEZE_FRAME ({freeze_frame}) is outside "
                f"the animation range. Using start frame {start_frame} instead."
            )
            freeze_frame = start_frame

    scene.frame_set(freeze_frame)
    bpy.context.view_layer.update()
    lower_body_transforms = capture_pose_transforms(armature_obj, lower_body_bone_names)
    if not lower_body_transforms:
        print("Warning: No matching lower body bones found; aborting.")
        scene.frame_set(original_frame)
        bpy.context.view_layer.update()
        return

    if clear_existing_keyframes:
        remove_keyframes_for_bones(action, lower_body_transforms)

    for frame in range(start_frame, end_frame + 1):
        scene.frame_set(frame)
        bpy.context.view_layer.update()

        for bone_name, transform in lower_body_transforms.items():
            pbone = armature_obj.pose.bones.get(bone_name)
            if not pbone:
                continue
            pbone.rotation_mode = transform["rotation_mode"]
            pbone.location = transform["location"]
            setattr(pbone, transform["rotation_attr"], transform["rotation"])
            pbone.scale = transform["scale"]
            pbone.keyframe_insert(data_path="location", frame=frame)
            pbone.keyframe_insert(data_path=transform["rotation_path"], frame=frame)
            pbone.keyframe_insert(data_path="scale", frame=frame)

        if frame % 20 == 0:
            print(f"Processed frame {frame}/{end_frame}...")

    scene.frame_set(original_frame)
    bpy.context.view_layer.update()
    print(
        f"Lower body frozen using frame {freeze_frame}. "
        f"Keyframes inserted on {len(lower_body_transforms)} bones."
    )


if __name__ == "__main__":
    freeze_lower_body_in_place()
