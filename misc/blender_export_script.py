"""
NOTE: This is meant to be invoked from inside of Blender, using its `Scripting` workspace — for now.
"""

import bpy
import csv
import os

# --- YOU ONLY NEED TO EDIT THESE TWO LINES ---

# 1. The exact name of your armature object in Blender
ARMATURE_NAME = "Nacks mmd after jiko_arm"

# 2. The full path where you want to save the CSV file.
#    On macOS/Linux, it looks like: "/Users/yourname/Desktop/pose_data.csv"
#    On Windows, it looks like: "C:/Users/yourname/Desktop/pose_data.csv"
#    (NOTE: Use forward slashes `/` even on Windows for Python)
CSV_FILEPATH = "/Users/yunhocho/Downloads/vmd_pose_data.csv"

# --- END OF EDITABLE SECTION ---

# --- OPTIONAL MOTION FILTERING ---
# Set USE_STATIC_LOWER_BODY to True to freeze the lower body at a single frame
# (default scene.frame_start), allowing only the upper body to animate.
USE_STATIC_LOWER_BODY = True

# Names of the lower-body bones whose motion should be frozen. These defaults
# match common MMD-style rigs but you can customize them for your armature.
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
]

# Additional bones to freeze even if they are not descendants of the root list.
LOWER_BODY_ADDITIONAL_BONES = [
    "センター",
]

# When True, every descendant of each entry in LOWER_BODY_ROOT_BONES is frozen.
LOWER_BODY_INCLUDE_CHILDREN = True

# Override the frame used to capture the static lower-body pose. Use None to
# fall back to the scene's start frame.
LOWER_BODY_FREEZE_FRAME = None


def extract_pose_transform(pbone):
    """Return location, rotation, and scale tuples for the given pose bone."""
    loc, rot, scale = pbone.matrix.decompose()
    return (
        (loc.x, loc.y, loc.z),
        (rot.w, rot.x, rot.y, rot.z),
        (scale.x, scale.y, scale.z),
    )


def vector_to_tuple(vec):
    """Convert a Blender mathutils vector into a plain tuple."""
    return (vec.x, vec.y, vec.z)


def gather_lower_body_bone_names(armature_obj):
    """
    Build the set of bone names whose transforms should be frozen for the
    lower body export option.
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


def build_frozen_ancestor_lookup(armature_obj, frozen_names):
    """
    Build a mapping from bone name to the nearest ancestor that is frozen.
    Only bones not in the frozen set will appear in the result.
    """
    frozen_set = set(frozen_names)
    ancestor_lookup = {}

    for pbone in armature_obj.pose.bones:
        if pbone.name in frozen_set:
            continue

        current = pbone.parent
        while current:
            if current.name in frozen_set:
                ancestor_lookup[pbone.name] = current.name
                break
            current = current.parent

    return ancestor_lookup


def export_armature_pose_data(with_tips=False):
    """
    Exports the pose data of a specified armature to a CSV file.
    It iterates through each frame of the animation and records the
    location, rotation (quaternion), and scale for each pose bone.

    For any bone that has no children (i.e., is a terminal bone
    like a fingertip), it also exports a "fake" bone representing the tip's
    location with an identity rotation and scale.
    """
    # Find the armature object in the scene
    armature_obj = bpy.data.objects.get(ARMATURE_NAME)

    # --- VALIDATION ---
    if not armature_obj or armature_obj.type != 'ARMATURE':
        print(f"Error: Armature object '{ARMATURE_NAME}' not found.")
        return

    if not armature_obj.animation_data or not armature_obj.animation_data.action:
        print(f"Error: Armature '{ARMATURE_NAME}' has no animation data.")
        return

    # Check if the output directory exists
    output_dir = os.path.dirname(CSV_FILEPATH)
    if not os.path.exists(output_dir):
        print(f"Error: Output directory does not exist: {output_dir}")
        print("Please create the folder or correct the CSV_FILEPATH.")
        return

    print(f"Starting export for armature: '{ARMATURE_NAME}'")
    print(f"Output file will be: '{CSV_FILEPATH}'")

    # --- CSV SETUP ---
    # Define the header for our CSV file
    header = [
        'frame', 'bone_name',
        'loc_x', 'loc_y', 'loc_z',
        'rot_w', 'rot_x', 'rot_y', 'rot_z',
        'scale_x', 'scale_y', 'scale_z'
    ]

    # Get the frame range from the scene's timeline
    scene = bpy.context.scene
    start_frame = int(scene.frame_start)
    end_frame = int(scene.frame_end)

    # Store the original frame to return to it later
    original_frame = scene.frame_current

    try:
        # Open the file for writing
        with open(CSV_FILEPATH, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(header)

            freeze_lower_body = USE_STATIC_LOWER_BODY
            lower_body_transforms = {}
            lower_body_tip_locations = {}
            frozen_ancestor_lookup = {}

            if freeze_lower_body:
                lower_body_bone_names = gather_lower_body_bone_names(armature_obj)

                if lower_body_bone_names:
                    frozen_list_str = ", ".join(sorted(lower_body_bone_names))
                    print(f"Lower body bones to freeze: {frozen_list_str}")

                    freeze_frame = LOWER_BODY_FREEZE_FRAME
                    if freeze_frame is None:
                        freeze_frame = start_frame
                    else:
                        freeze_frame = int(freeze_frame)
                        if freeze_frame < start_frame or freeze_frame > end_frame:
                            print(
                                f"Warning: LOWER_BODY_FREEZE_FRAME ({freeze_frame}) is "
                                f"outside the animation range. "
                                f"Using start frame {start_frame} instead."
                            )
                            freeze_frame = start_frame

                    scene.frame_set(freeze_frame)
                    bpy.context.view_layer.update()

                    for bone_name in lower_body_bone_names:
                        pbone = armature_obj.pose.bones.get(bone_name)
                        if not pbone:
                            continue

                        lower_body_transforms[bone_name] = extract_pose_transform(pbone)
                        if with_tips:
                            lower_body_tip_locations[bone_name] = vector_to_tuple(pbone.tail)

                    if lower_body_transforms:
                        print(
                            f"Lower body bones will use a static pose from frame {freeze_frame} "
                            f"({len(lower_body_transforms)} bones)."
                        )
                        frozen_ancestor_lookup = build_frozen_ancestor_lookup(
                            armature_obj,
                            lower_body_transforms.keys()
                        )
                    else:
                        print("Warning: No matching lower body bones found; exporting full motion.")
                        freeze_lower_body = False

                    scene.frame_set(start_frame)
                    bpy.context.view_layer.update()
                else:
                    print("Warning: No lower body bones resolved; exporting full motion.")
                    freeze_lower_body = False

            # --- DATA EXPORT LOOP ---
            # Loop through every frame in the scene's range
            for frame in range(start_frame, end_frame + 1):
                # Go to the specific frame
                scene.frame_set(frame)

                # Update the dependency graph for the new frame
                bpy.context.view_layer.update()

                # Gather the current (possibly unfrozen) transforms for all bones
                current_transforms = {}
                current_tip_locations = {}
                for pbone in armature_obj.pose.bones:
                    current_transforms[pbone.name] = extract_pose_transform(pbone)
                    if with_tips:
                        current_tip_locations[pbone.name] = vector_to_tuple(pbone.tail)

                frame_offsets = {}
                if freeze_lower_body:
                    for frozen_name, frozen_transform in lower_body_transforms.items():
                        current_transform = current_transforms.get(frozen_name)
                        if not current_transform:
                            continue
                        current_loc = current_transform[0]
                        frozen_loc = frozen_transform[0]
                        frame_offsets[frozen_name] = tuple(
                            current_loc[i] - frozen_loc[i] for i in range(3)
                        )

                # Loop through each pose bone in the armature
                for pbone in armature_obj.pose.bones:
                    use_static_pose = freeze_lower_body and (pbone.name in lower_body_transforms)

                    current_loc_vals, current_rot_vals, current_scale_vals = current_transforms[pbone.name]
                    if use_static_pose:
                        loc_vals, rot_vals, scale_vals = lower_body_transforms[pbone.name]
                    else:
                        loc_vals, rot_vals, scale_vals = (
                            current_loc_vals,
                            current_rot_vals,
                            current_scale_vals,
                        )
                        if freeze_lower_body:
                            ancestor_name = frozen_ancestor_lookup.get(pbone.name)
                            ancestor_offset = frame_offsets.get(ancestor_name) if ancestor_name else None
                            if ancestor_offset:
                                loc_vals = tuple(
                                    loc_vals[i] - ancestor_offset[i] for i in range(3)
                                )

                    # Create a row with the real bone's data
                    row = [
                        frame, pbone.name,
                        *loc_vals,
                        *rot_vals,
                        *scale_vals
                    ]
                    # Write the real bone's row to the CSV file
                    csv_writer.writerow(row)

                    # Check for and create fake tip bone
                    if with_tips:
                        # If the bone has no children, it's a terminal bone.
                        if not pbone.children:
                            # Define the "fake" tip bone's properties
                            tip_name = f"{pbone.name}_tip"
                            if use_static_pose and pbone.name in lower_body_tip_locations:
                                tip_loc_vals = lower_body_tip_locations[pbone.name]
                            else:
                                tip_loc_vals = current_tip_locations[pbone.name]
                                if freeze_lower_body:
                                    ancestor_name = frozen_ancestor_lookup.get(pbone.name)
                                    ancestor_offset = frame_offsets.get(ancestor_name) if ancestor_name else None
                                    if ancestor_offset:
                                        tip_loc_vals = tuple(
                                            tip_loc_vals[i] - ancestor_offset[i] for i in range(3)
                                        )

                            # Create the row for the fake tip bone
                            tip_row = [
                                frame, tip_name,
                                *tip_loc_vals,
                                1.0, 0.0, 0.0, 0.0,  # Identity rotation (w, x, y, z)
                                1.0, 1.0, 1.0         # Identity scale
                            ]
                            # Write the fake tip bone's row to the CSV file
                            csv_writer.writerow(tip_row)

                        # Special case: explicitly save センター bone's tail position
                        if pbone.name == "センター":
                            center_tip_name = f"{pbone.name}_tip"
                            if use_static_pose and pbone.name in lower_body_tip_locations:
                                center_tip_loc_vals = lower_body_tip_locations[pbone.name]
                            else:
                                center_tip_loc_vals = current_tip_locations[pbone.name]
                                if freeze_lower_body:
                                    ancestor_name = frozen_ancestor_lookup.get(pbone.name)
                                    ancestor_offset = frame_offsets.get(ancestor_name) if ancestor_name else None
                                    if ancestor_offset:
                                        center_tip_loc_vals = tuple(
                                            center_tip_loc_vals[i] - ancestor_offset[i] for i in range(3)
                                        )

                            center_tip_row = [
                                frame, center_tip_name,
                                *center_tip_loc_vals,
                                1.0, 0.0, 0.0, 0.0,  # Identity rotation
                                1.0, 1.0, 1.0         # Identity scale
                            ]
                            csv_writer.writerow(center_tip_row)

                # Optional: print progress to the system console
                if frame % 20 == 0:
                    print(f"Processed frame {frame}/{end_frame}...")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Return to the original frame when the script is done or fails
        scene.frame_set(original_frame)

    print(f"\nExport finished successfully!")
    print(f"Data for frames {start_frame} to {end_frame} saved.")


# --- SCRIPT EXECUTION ---
if __name__ == "__main__":
    export_armature_pose_data(with_tips=True)
