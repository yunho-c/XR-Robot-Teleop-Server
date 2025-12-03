import argparse
from pathlib import Path
import bpy

from freeze_lower_body_in_place import freeze_lower_body_in_place
from bpy_addon_utils import install_addon

ADDON_PATH = Path(__file__).resolve().parent.parent / "assets" / "io_anim_bvh_enhanced.zip"


def process_file(file_path):
    file_path = Path(file_path)
    print(f"Processing: {file_path}")
    
    # Reset Scene
    bpy.ops.wm.read_homefile(use_empty=True)
    
    # Import BVH
    try:
        bpy.ops.import_anim.bvh(filepath=str(file_path), update_scene_fps=True)
    except Exception as e:
        print(f"Failed to import {file_path}: {e}")
        return

    # Find Armature
    armature_obj = None
    for obj in bpy.data.objects:
        if obj.type == 'ARMATURE':
            armature_obj = obj
            break
    
    if not armature_obj:
        print(f"No armature found in {file_path}")
        return

    # Ensure armature is active and selected
    bpy.context.view_layer.objects.active = armature_obj
    armature_obj.select_set(True)

    # Update scene frame range to match the action
    if armature_obj.animation_data and armature_obj.animation_data.action:
        action = armature_obj.animation_data.action
        frame_start, frame_end = action.frame_range
        bpy.context.scene.frame_start = int(frame_start)
        bpy.context.scene.frame_end = int(frame_end)
        print(f"Updated scene frame range to: {int(frame_start)} - {int(frame_end)}")

    # Apply Freezing
    freeze_lower_body_in_place(
        armature_name=armature_obj.name,
        duplicate_action=False, 
        use_static_lower_body=True,
        clear_existing_keyframes=True
    )

    # Export
    output_path = file_path.with_name(f"{file_path.stem}_upper.bvh")
    
    # Ensure the armature is selected again just in case
    bpy.ops.object.select_all(action='DESELECT')
    armature_obj.select_set(True)
    bpy.context.view_layer.objects.active = armature_obj

    try:
        if not hasattr(bpy.ops.export_anim, "bvh_enhanced"):
             print("Error: 'export_anim.bvh_enhanced' operator not found. Is the addon installed?")
             return

        bpy.ops.export_anim.bvh_enhanced(
            filepath=str(output_path),
            axis_up='Y',
            axis_forward='-Z',
        )
        print(f"Exported to: {output_path}")
    except Exception as e:
        print(f"Failed to export {output_path}: {e}")
        import traceback
        traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description="Batch freeze lower body for BVH files.")
    parser.add_argument("input", help="Input file or directory")
    
    args = parser.parse_args()

    install_addon(str(ADDON_PATH), module_name="io_anim_bvh_enhanced")

    input_path = Path(args.input)
    
    if input_path.is_file():
        if input_path.suffix.lower() == ".bvh":
            process_file(input_path)
        else:
            print("Input file is not a .bvh file.")
    elif input_path.is_dir():
        for file_path in input_path.rglob("*"):
            if file_path.is_file() and file_path.suffix.lower() == ".bvh" and not file_path.name.endswith("_upper.bvh"):
                process_file(file_path)
    else:
        print(f"Input path not found: {input_path}")


if __name__ == "__main__":
    main()  
