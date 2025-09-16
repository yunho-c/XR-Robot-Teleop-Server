"""
This script demonstrates how to perform dextrous hand control by retargeting human hand poses
from FBX-exported CSV data to a robot hand in the MuJoCo simulator.

This is an adaptation of examples/dextrous_hand_control.py that reads hand pose data from
a static CSV file (exported from FBX via Blender) instead of a live WebRTC stream.

You need to have the dex-retargeting library and its dependencies installed.
You can install the extra dependencies for this example by running:
pip install -e .[example]
in the `References/dex-retargeting` directory. You will also need to install mujoco.

Usage:
    python dextrous_hand_control_fbx.py --file path/to/animation.csv
    python dextrous_hand_control_fbx.py --file path/to/animation.csv --frame 10
    python dextrous_hand_control_fbx.py --file path/to/animation.csv --animate
"""

import argparse
import csv
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rerun as rr
from dex_retargeting.constants import (
    HandType,
    RetargetingType,
    RobotName,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting
from robot_descriptions.loaders.mujoco import load_robot_description

from xr_robot_teleop_server import configure_logging, logger
from xr_robot_teleop_server.schemas.openxr_skeletons import FullBodyBoneId

# Params
DEBUG = False
ENABLE_DYNAMICS = False

# NOTE: see RobotName enum for possible values
DEFAULT_ROBOT: str = "shadow"

# Mixamo to FullBody bone name mapping (hand bones only)
MIXAMO_TO_FULLBODY_HAND_MAPPING = {
    # Left hand
    "mixamorig:LeftHand": FullBodyBoneId.FullBody_LeftHandWrist,
    "mixamorig:LeftHandThumb1": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "mixamorig:LeftHandThumb2": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "mixamorig:LeftHandThumb3": FullBodyBoneId.FullBody_LeftHandThumbDistal,
    "mixamorig:LeftHandThumb4": FullBodyBoneId.FullBody_LeftHandThumbTip,
    "mixamorig:LeftHandIndex1": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "mixamorig:LeftHandIndex2": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "mixamorig:LeftHandIndex3": FullBodyBoneId.FullBody_LeftHandIndexDistal,
    "mixamorig:LeftHandIndex4": FullBodyBoneId.FullBody_LeftHandIndexTip,
    "mixamorig:LeftHandMiddle1": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "mixamorig:LeftHandMiddle2": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "mixamorig:LeftHandMiddle3": FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    "mixamorig:LeftHandMiddle4": FullBodyBoneId.FullBody_LeftHandMiddleTip,
    "mixamorig:LeftHandRing1": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "mixamorig:LeftHandRing2": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "mixamorig:LeftHandRing3": FullBodyBoneId.FullBody_LeftHandRingDistal,
    "mixamorig:LeftHandRing4": FullBodyBoneId.FullBody_LeftHandRingTip,
    "mixamorig:LeftHandPinky1": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "mixamorig:LeftHandPinky2": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    "mixamorig:LeftHandPinky3": FullBodyBoneId.FullBody_LeftHandLittleDistal,
    "mixamorig:LeftHandPinky4": FullBodyBoneId.FullBody_LeftHandLittleTip,

    # Right hand
    "mixamorig:RightHand": FullBodyBoneId.FullBody_RightHandWrist,
    "mixamorig:RightHandThumb1": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "mixamorig:RightHandThumb2": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "mixamorig:RightHandThumb3": FullBodyBoneId.FullBody_RightHandThumbDistal,
    "mixamorig:RightHandThumb4": FullBodyBoneId.FullBody_RightHandThumbTip,
    "mixamorig:RightHandIndex1": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "mixamorig:RightHandIndex2": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "mixamorig:RightHandIndex3": FullBodyBoneId.FullBody_RightHandIndexDistal,
    "mixamorig:RightHandIndex4": FullBodyBoneId.FullBody_RightHandIndexTip,
    "mixamorig:RightHandMiddle1": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "mixamorig:RightHandMiddle2": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "mixamorig:RightHandMiddle3": FullBodyBoneId.FullBody_RightHandMiddleDistal,
    "mixamorig:RightHandMiddle4": FullBodyBoneId.FullBody_RightHandMiddleTip,
    "mixamorig:RightHandRing1": FullBodyBoneId.FullBody_RightHandRingProximal,
    "mixamorig:RightHandRing2": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "mixamorig:RightHandRing3": FullBodyBoneId.FullBody_RightHandRingDistal,
    "mixamorig:RightHandRing4": FullBodyBoneId.FullBody_RightHandRingTip,
    "mixamorig:RightHandPinky1": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "mixamorig:RightHandPinky2": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
    "mixamorig:RightHandPinky3": FullBodyBoneId.FullBody_RightHandLittleDistal,
    "mixamorig:RightHandPinky4": FullBodyBoneId.FullBody_RightHandLittleTip,
}

# These mappings convert the 21-point MANO hand keypoint structure into
# OpenXR FullBodyBoneId enum for the left/right hands.
# NOTE: We are skipping the "intermediate" phalanx bones from OpenXR
#       to match MANO's structure.
MANO_TO_OPENXR_RIGHT_HAND: dict[int, FullBodyBoneId] = {
    0: FullBodyBoneId.FullBody_RightHandWrist,
    1: FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    2: FullBodyBoneId.FullBody_RightHandThumbProximal,
    3: FullBodyBoneId.FullBody_RightHandThumbDistal,
    4: FullBodyBoneId.FullBody_RightHandThumbTip,
    5: FullBodyBoneId.FullBody_RightHandIndexMetacarpal,
    6: FullBodyBoneId.FullBody_RightHandIndexProximal,
    7: FullBodyBoneId.FullBody_RightHandIndexDistal,
    8: FullBodyBoneId.FullBody_RightHandIndexTip,
    9: FullBodyBoneId.FullBody_RightHandMiddleMetacarpal,
    10: FullBodyBoneId.FullBody_RightHandMiddleProximal,
    11: FullBodyBoneId.FullBody_RightHandMiddleDistal,
    12: FullBodyBoneId.FullBody_RightHandMiddleTip,
    13: FullBodyBoneId.FullBody_RightHandRingMetacarpal,
    14: FullBodyBoneId.FullBody_RightHandRingProximal,
    15: FullBodyBoneId.FullBody_RightHandRingDistal,
    16: FullBodyBoneId.FullBody_RightHandRingTip,
    17: FullBodyBoneId.FullBody_RightHandLittleMetacarpal,
    18: FullBodyBoneId.FullBody_RightHandLittleProximal,
    19: FullBodyBoneId.FullBody_RightHandLittleDistal,
    20: FullBodyBoneId.FullBody_RightHandLittleTip,
}

MANO_TO_OPENXR_LEFT_HAND: dict[int, FullBodyBoneId] = {
    0: FullBodyBoneId.FullBody_LeftHandWrist,
    1: FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    2: FullBodyBoneId.FullBody_LeftHandThumbProximal,
    3: FullBodyBoneId.FullBody_LeftHandThumbDistal,
    4: FullBodyBoneId.FullBody_LeftHandThumbTip,
    5: FullBodyBoneId.FullBody_LeftHandIndexMetacarpal,
    6: FullBodyBoneId.FullBody_LeftHandIndexProximal,
    7: FullBodyBoneId.FullBody_LeftHandIndexDistal,
    8: FullBodyBoneId.FullBody_LeftHandIndexTip,
    9: FullBodyBoneId.FullBody_LeftHandMiddleMetacarpal,
    10: FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    11: FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    12: FullBodyBoneId.FullBody_LeftHandMiddleTip,
    13: FullBodyBoneId.FullBody_LeftHandRingMetacarpal,
    14: FullBodyBoneId.FullBody_LeftHandRingProximal,
    15: FullBodyBoneId.FullBody_LeftHandRingDistal,
    16: FullBodyBoneId.FullBody_LeftHandRingTip,
    17: FullBodyBoneId.FullBody_LeftHandLittleMetacarpal,
    18: FullBodyBoneId.FullBody_LeftHandLittleProximal,
    19: FullBodyBoneId.FullBody_LeftHandLittleDistal,
    20: FullBodyBoneId.FullBody_LeftHandLittleTip,
}

MANO_HAND_CONNECTIVITY = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 4],
    [0, 5],
    [5, 6],
    [6, 7],
    [7, 8],
    [0, 9],
    [9, 10],
    [10, 11],
    [11, 12],
    [0, 13],
    [13, 14],
    [14, 15],
    [15, 16],
    [0, 17],
    [17, 18],
    [18, 19],
    [19, 20],
]

ROBOTS_WITH_DIFFERENT_JOINT_NAMES = [
    "allegro",
    "shadow",
    "ability",
]

ROBODESC_VARIANT = {
    "allegro": "<SIDE>_hand",
    "shadow": "<SIDE>_hand",
    "ability": "<SIDE>_hand_large",
}

DEX_RETARGETING_TO_MUJOCO_JOINT_NAMES: dict[str, dict[str, str]] = {
    "allegro": {
        "joint_0.0": "ffj0",
        "joint_1.0": "ffj1",
        "joint_2.0": "ffj2",
        "joint_3.0": "ffj3",
        "joint_4.0": "mfj0",
        "joint_5.0": "mfj1",
        "joint_6.0": "mfj2",
        "joint_7.0": "mfj3",
        "joint_8.0": "rfj0",
        "joint_9.0": "rfj1",
        "joint_10.0": "rfj2",
        "joint_11.0": "rfj3",
        "joint_12.0": "thj0",
        "joint_13.0": "thj1",
        "joint_14.0": "thj2",
        "joint_15.0": "thj3",
    },
    "shadow": {
        # Wrist
        "WRJ1": "rh_WRJ1",
        "WRJ2": "rh_WRJ2",
        # First Finger (FF)
        "FFJ1": "rh_FFJ1",
        "FFJ2": "rh_FFJ2",
        "FFJ3": "rh_FFJ3",
        "FFJ4": "rh_FFJ4",
        # Middle Finger (MF)
        "MFJ1": "rh_MFJ1",
        "MFJ2": "rh_MFJ2",
        "MFJ3": "rh_MFJ3",
        "MFJ4": "rh_MFJ4",
        # Ring Finger (RF)
        "RFJ1": "rh_RFJ1",
        "RFJ2": "rh_RFJ2",
        "RFJ3": "rh_RFJ3",
        "RFJ4": "rh_RFJ4",
        # Little Finger (LF)
        "LFJ1": "rh_LFJ1",
        "LFJ2": "rh_LFJ2",
        "LFJ3": "rh_LFJ3",
        "LFJ4": "rh_LFJ4",
        "LFJ5": "rh_LFJ5",
        # Thumb (TH)
        "THJ1": "rh_THJ1",
        "THJ2": "rh_THJ2",
        "THJ3": "rh_THJ3",
        "THJ4": "rh_THJ4",
        "THJ5": "rh_THJ5",
    },
    "ability": {
        "index_q1": "index_mcp",
        "index_q2": "index_pip",
        "middle_q1": "middle_mcp",
        "middle_q2": "middle_pip",
        "pinky_q1": "pinky_mcp",
        "pinky_q2": "pinky_pip",
        "ring_q1": "ring_mcp",
        "ring_q2": "ring_pip",
        "thumb_q1": "thumb_cmc",
        "thumb_q2": "thumb_mcp",
    },
}


class BoneData:
    """Represents bone position and rotation data."""
    def __init__(self, bone_id: FullBodyBoneId, position: tuple[float, float, float]):
        self.id = bone_id
        self.position = position


class AppState:
    """State object to hold retargeting and MuJoCo objects."""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.retargeting: SeqRetargeting | None = None
        self.retargeting_config: RetargetingConfig | None = None
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.viewer: mujoco.viewer.Viewer | None = None
        self.retargeting_to_mujoco_map: list | None = None
        self.is_initialized = False

        # Setup retargeting
        config_path = get_default_config_path(
            RobotName[args.robot_name],
            RetargetingType[args.retargeting_type],
            HandType[args.hand_type],
        )
        if not config_path.exists():
            logger.error(f"Retargeting config not found: {config_path}")
            return

        # Set the default URDF directory. Assumes this script is run from the project root.
        urdf_dir = Path("References/dex-retargeting/assets/robots/hands")
        if not urdf_dir.exists():
            logger.error(
                f"URDF directory not found: {urdf_dir}."
                "Please make sure the git submodules are initialized: "
                "`git submodule update --init --recursive`"
            )
            return
        RetargetingConfig.set_default_urdf_dir(urdf_dir)
        self.retargeting = RetargetingConfig.load_from_file(config_path).build()
        logger.info(f"Retargeting for {args.robot_name} initialized.")

        # Setup MuJoCo
        self.setup_mujoco(args.robot_name)
        if self.viewer is None:
            logger.warning(f"Setup failed for {args.robot_name}")
            return

        self.is_initialized = True

    def setup_mujoco(self, robot_name):
        """Initializes the MuJoCo model, data, and viewer."""
        try:
            variant = ROBODESC_VARIANT[robot_name].replace("<SIDE>", self.args.hand_type)
            model = load_robot_description(f"{robot_name}_hand_mj_description", variant=variant)
        except Exception as e:
            logger.error(f"Failed to load model for {robot_name} from `robot_descriptions`: {e}")
            return

        data = mujoco.MjData(model)

        if DEBUG:  # Iterate through all joints in the model
            for i in range(model.njnt):
                joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                qpos_address = model.jnt_qposadr[i]
                print(f"Joint '{joint_name}' (ID: {i}) starts at qpos index: {qpos_address}")

        # Create joint mapping
        if robot_name in ROBOTS_WITH_DIFFERENT_JOINT_NAMES:
            D2M = DEX_RETARGETING_TO_MUJOCO_JOINT_NAMES[robot_name]  # alias
            retargeting_joint_names = [D2M[name] for name in self.retargeting.joint_names]
        else:
            retargeting_joint_names = self.retargeting.joint_names
        if DEBUG:
            print(f"{retargeting_joint_names=}")

        mujoco_joint_ids = []
        for name in retargeting_joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid == -1:
                logger.warning(f"Joint {name} not found in MuJoCo model.")
            mujoco_joint_ids.append(jid)

        # Create a mapping between retargeted pose and qpos array
        self.retargeting_to_mujoco_map = []
        for i, jid in enumerate(mujoco_joint_ids):
            if jid != -1:
                # NOTE: (index_in_retargeting_qpos, index_in_mujoco_qpos)
                self.retargeting_to_mujoco_map.append((i, model.jnt_qposadr[jid]))

        self.model = model
        self.data = data
        self.viewer = mujoco.viewer.launch_passive(model, data)
        logger.info("MuJoCo scene initialized.")


def parse_csv_data(csv_file: str, hand_type: str) -> dict[int, list[BoneData]]:
    """
    Parse CSV data and return frame-indexed hand bone data.

    Args:
        csv_file: Path to the CSV file
        hand_type: "left" or "right" hand

    Returns:
        Dictionary mapping frame numbers to lists of BoneData
    """
    frame_data = {}

    try:
        with open(csv_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = int(row['frame'])
                bone_name = row['bone_name']

                # Skip non-hand bones or bones for the wrong hand
                if bone_name not in MIXAMO_TO_FULLBODY_HAND_MAPPING:
                    continue

                bone_id = MIXAMO_TO_FULLBODY_HAND_MAPPING[bone_name]

                # Filter by hand type
                if hand_type == "left" and "Right" in bone_id.name:
                    continue
                if hand_type == "right" and "Left" in bone_id.name:
                    continue

                # Parse position (convert to meters and adjust coordinate system)
                # Blender uses Z-up, right-handed. Convert to the expected coordinate system.
                loc_x = float(row['loc_x']) / 100.0  # Convert cm to meters
                loc_y = float(row['loc_y']) / 100.0
                loc_z = float(row['loc_z']) / 100.0

                # Blender uses Z-up. Good!
                position = (loc_x, loc_y, loc_z)
                bone_data = BoneData(bone_id, position)

                if frame not in frame_data:
                    frame_data[frame] = []
                frame_data[frame].append(bone_data)

    except FileNotFoundError:
        print(f"Error: Could not find file {csv_file}")
        return {}
    except Exception as e:
        print(f"Error parsing CSV file: {e}")
        return {}

    return frame_data


def process_hand_data(hand_data: list[BoneData], state: AppState) -> bool:
    """Process hand data and update MuJoCo simulation."""
    if not state.is_initialized:
        logger.warning("App state not initialized, skipping frame.")
        return False

    try:
        bone_map = {b.id: b for b in hand_data}
        mapping = (
            MANO_TO_OPENXR_RIGHT_HAND
            if state.args.hand_type == "right"
            else MANO_TO_OPENXR_LEFT_HAND
        )

        # Extract 21 hand keypoints
        hand_keypoints = np.zeros((21, 3))
        missing_bones = []
        hand_detected = True
        for i in range(21):
            bone_id = mapping[i]
            print(f"{bone_map=``}")
            if bone_id in bone_map:
                hand_keypoints[i] = bone_map[bone_id].position
            else:
                missing_bones.append(bone_id.name)
                hand_detected = False

        if not hand_detected:
            logger.warning(f"Missing hand bones: {missing_bones}")
            return False

        if hand_detected:
            # Visualize human hand
            rr.log(
                "world/human_hand/points",
                rr.Points3D(positions=hand_keypoints, radii=0.005),
            )
            connections = [
                (hand_keypoints[start], hand_keypoints[end])
                for start, end in MANO_HAND_CONNECTIVITY
            ]
            rr.log(
                "world/human_hand/skeleton",
                rr.LineStrips3D(strips=connections, radii=0.002),
            )

            # Get target vectors for retargeting
            optimizer = state.retargeting.optimizer
            indices = optimizer.target_link_human_indices
            origin_indices = indices[0, :]
            task_indices = indices[1, :]
            origin_points = hand_keypoints[origin_indices, :]
            task_points = hand_keypoints[task_indices, :]
            target_vectors = task_points - origin_points

            # Retarget
            qpos = state.retargeting.retarget(ref_value=target_vectors)
            if DEBUG:
                print(f"{qpos=}")

            # Visualize retargeting results
            robot = state.retargeting.optimizer.robot
            robot.compute_forward_kinematics(qpos)

            if optimizer.retargeting_type in ["VECTOR", "DEXPILOT"]:
                origin_link_names = optimizer.origin_link_names
                task_link_names = optimizer.task_link_names
                all_link_names = sorted(set(origin_link_names + task_link_names))
                link_name_to_idx = {name: i for i, name in enumerate(all_link_names)}

                rr.log(
                    "world/dex_retargeting/target_vectors",
                    rr.Arrows3D(
                        origins=origin_points,
                        vectors=target_vectors,
                    ),
                )

                positions = np.array(
                    [
                        robot.get_link_pose(robot.get_link_index(name))[:3, 3]
                        for name in all_link_names
                    ]
                )

                rr.log(
                    "world/robot_hand/points",
                    rr.Points3D(positions=positions, radii=0.005),
                )

                connections = [
                    (
                        positions[link_name_to_idx[origin]],
                        positions[link_name_to_idx[task]],
                    )
                    for origin, task in zip(
                        origin_link_names, task_link_names, strict=False
                    )
                ]
                rr.log(
                    "world/robot_hand/skeleton",
                    rr.LineStrips3D(strips=connections, radii=0.002),
                )

            # Apply to MuJoCo model
            for retarget_idx, mujoco_idx in state.retargeting_to_mujoco_map:
                state.data.qpos[mujoco_idx] = qpos[retarget_idx]

        # Step the simulation and render
        if ENABLE_DYNAMICS:
            mujoco.mj_step(state.model, state.data)
        else:
            mujoco.mj_forward(state.model, state.data)  # update kinematics only
        state.viewer.sync()

        return hand_detected

    except Exception as e:
        logger.error(f"Could not process hand pose data: {e}", exc_info=True)
        if state.viewer and state.viewer.is_running():
            state.viewer.close()
        return False


def main():
    parser = argparse.ArgumentParser(description="Dextrous Hand Control from FBX CSV data")
    parser.add_argument(
        "--file",
        type=str,
        required=True,
        help="Path to the CSV file containing pose data"
    )
    parser.add_argument(
        "--frame",
        type=int,
        help="Specific frame to process (if not provided, shows frame 1)"
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Animate through all frames"
    )
    parser.add_argument(
        "--robot-name",
        type=str,
        default=DEFAULT_ROBOT,
        choices=[r.name for r in RobotName],
        help="Name of the robot hand to use.",
    )
    parser.add_argument(
        "--retargeting-type",
        type=str,
        default="vector",
        choices=[t.name for t in RetargetingType],
        help="Type of retargeting to use.",
    )
    parser.add_argument(
        "--hand-type",
        type=str,
        default="right",
        choices=["left", "right"],
        help="Which hand to track.",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    args = parser.parse_args()

    configure_logging(level=args.log_level.upper())

    # Parse CSV data
    print(f"Loading hand pose data from {args.file}...")
    frame_data = parse_csv_data(args.file, args.hand_type)

    if not frame_data:
        print("No valid hand pose data found!")
        return

    print(f"Loaded {len(frame_data)} frames")

    # Initialize rerun
    rr.init("dextrous_hand_control_fbx", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Log world frame axes
    rr.log(
        "world/axes",
        rr.Arrows3D(
            origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=["X", "Y", "Z"],
        ),
        static=True,
    )

    # Initialize MuJoCo and retargeting
    state = AppState(args)
    if not state.is_initialized:
        print("Failed to initialize MuJoCo and retargeting system!")
        return

    # Process data
    if args.animate:
        print("Animating through frames... Press Ctrl+C to stop")
        try:
            sorted_frames = sorted(frame_data.keys())
            while True:
                for frame_num in sorted_frames:
                    if not state.viewer.is_running():
                        print("Viewer closed, stopping animation")
                        return

                    rr.set_time(timeline="main", sequence=frame_num)
                    success = process_hand_data(frame_data[frame_num], state)
                    if not success:
                        print(f"Failed to process frame {frame_num}")
                    time.sleep(0.1)  # 10 FPS
        except KeyboardInterrupt:
            print("\nAnimation stopped")
    else:
        # Show specific frame or frame 1
        target_frame = args.frame if args.frame is not None else 1
        if target_frame in frame_data:
            print(f"Processing frame {target_frame}")
            rr.set_time(timeline="main", sequence=target_frame)
            success = process_hand_data(frame_data[target_frame], state)
            if success:
                print(
                    "Hand retargeting successful! Check the MuJoCo viewer and "
                    "rerun visualization."
                )
                # Keep the viewer open
                try:
                    while state.viewer.is_running():
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("Exiting...")
            else:
                print("Hand retargeting failed!")
        else:
            print(f"Frame {target_frame} not found. Available frames: {sorted(frame_data.keys())}")

    # Cleanup
    if state.viewer:
        state.viewer.close()


if __name__ == "__main__":
    main()
