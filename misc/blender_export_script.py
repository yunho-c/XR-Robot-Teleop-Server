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

            # --- DATA EXPORT LOOP ---
            # Loop through every frame in the scene's range
            for frame in range(start_frame, end_frame + 1):
                # Go to the specific frame
                scene.frame_set(frame)

                # Update the dependency graph for the new frame
                bpy.context.view_layer.update()

                # Loop through each pose bone in the armature
                for pbone in armature_obj.pose.bones:
                    # Get the final transformation matrix in object space
                    final_matrix = pbone.matrix
                    loc, rot, scale = final_matrix.decompose()

                    # Create a row with the real bone's data
                    row = [
                        frame, pbone.name,
                        loc.x, loc.y, loc.z,
                        rot.w, rot.x, rot.y, rot.z,
                        scale.x, scale.y, scale.z
                    ]
                    # Write the real bone's row to the CSV file
                    csv_writer.writerow(row)

                    # Check for and create fake tip bone
                    if with_tips:
                        # If the bone has no children, it's a terminal bone.
                        if not pbone.children:
                            # Define the "fake" tip bone's properties
                            tip_name = f"{pbone.name}_tip"
                            tip_loc = pbone.tail # The location is the tail of the parent bone

                            # Create the row for the fake tip bone
                            tip_row = [
                                frame, tip_name,
                                tip_loc.x, tip_loc.y, tip_loc.z,
                                1.0, 0.0, 0.0, 0.0,  # Identity rotation (w, x, y, z)
                                1.0, 1.0, 1.0         # Identity scale
                            ]
                            # Write the fake tip bone's row to the CSV file
                            csv_writer.writerow(tip_row)

                        # Special case: explicitly save センター bone's tail position
                        if pbone.name == "センター":
                            center_tip_name = f"{pbone.name}_tip"
                            center_tip_loc = pbone.tail

                            center_tip_row = [
                                frame, center_tip_name,
                                center_tip_loc.x, center_tip_loc.y, center_tip_loc.z,
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

