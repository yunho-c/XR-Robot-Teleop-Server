"""
Dextrous hand control using VMD-exported CSV animation data.

This script demonstrates how to perform dextrous hand control by retargeting human hand poses
from VMD CSV files to a robot hand in the MuJoCo simulator.

You need to have the dex-retargeting library and its dependencies installed.
You can install the extra dependencies for this example by running:
pip install -e .[example]
in the `References/dex-retargeting` directory. You will also need to install mujoco.

This script reads bone position and rotation data from a VMD CSV file and extracts
hand pose data, retargets it to a specified robot hand, and visualizes the
result in MuJoCo with optional animation playback.

Usage:
    python misc/dextrous_hand_control_vmd.py --file animation.csv
    python misc/dextrous_hand_control_vmd.py --file animation.csv --animate --fps 30
    python misc/dextrous_hand_control_vmd.py --file animation.csv --robot-name shadow --hand-type left
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

# Import VMD bone mapping
from visualize_vmd_body_pose import VMD_TO_FULLBODY_MAPPING

# Params
DEBUG = False
ENABLE_DYNAMICS = False
DEFAULT_ROBOT: str = "shadow"

# MANO hand keypoint mappings (same as in examples/dextrous_hand_control.py)
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
    [0, 1], [1, 2], [2, 3], [3, 4],
    [0, 5], [5, 6], [6, 7], [7, 8],
    [0, 9], [9, 10], [10, 11], [11, 12],
    [0, 13], [13, 14], [14, 15], [15, 16],
    [0, 17], [17, 18], [18, 19], [19, 20],
]

# Robot configuration (same as in examples/dextrous_hand_control.py)
ROBOTS_WITH_DIFFERENT_JOINT_NAMES = ["allegro", "shadow", "ability"]

ROBODESC_VARIANT = {
    "allegro": "<SIDE>_hand",
    "shadow": "<SIDE>_hand",
    "ability": "<SIDE>_hand_large",
}

DEX_RETARGETING_TO_MUJOCO_JOINT_NAMES: dict[str, dict[str, str]] = {
    "allegro": {
        "joint_0.0": "ffj0", "joint_1.0": "ffj1", "joint_2.0": "ffj2", "joint_3.0": "ffj3",
        "joint_4.0": "mfj0", "joint_5.0": "mfj1", "joint_6.0": "mfj2", "joint_7.0": "mfj3",
        "joint_8.0": "rfj0", "joint_9.0": "rfj1", "joint_10.0": "rfj2", "joint_11.0": "rfj3",
        "joint_12.0": "thj0", "joint_13.0": "thj1", "joint_14.0": "thj2", "joint_15.0": "thj3",
    },
    "shadow": {
        "WRJ1": "rh_WRJ1", "WRJ2": "rh_WRJ2",
        "FFJ1": "rh_FFJ1", "FFJ2": "rh_FFJ2", "FFJ3": "rh_FFJ3", "FFJ4": "rh_FFJ4",
        "MFJ1": "rh_MFJ1", "MFJ2": "rh_MFJ2", "MFJ3": "rh_MFJ3", "MFJ4": "rh_MFJ4",
        "RFJ1": "rh_RFJ1", "RFJ2": "rh_RFJ2", "RFJ3": "rh_RFJ3", "RFJ4": "rh_RFJ4",
        "LFJ1": "rh_LFJ1", "LFJ2": "rh_LFJ2", "LFJ3": "rh_LFJ3", "LFJ4": "rh_LFJ4", "LFJ5": "rh_LFJ5",
        "THJ1": "rh_THJ1", "THJ2": "rh_THJ2", "THJ3": "rh_THJ3", "THJ4": "rh_THJ4", "THJ5": "rh_THJ5",
    },
    "ability": {
        "index_q1": "index_mcp", "index_q2": "index_pip",
        "middle_q1": "middle_mcp", "middle_q2": "middle_pip",
        "pinky_q1": "pinky_mcp", "pinky_q2": "pinky_pip",
        "ring_q1": "ring_mcp", "ring_q2": "ring_pip",
        "thumb_q1": "thumb_cmc", "thumb_q2": "thumb_mcp",
    },
}


class BoneData:
    """Represents bone position and rotation data."""
    def __init__(self, bone_id: FullBodyBoneId, position: tuple[float, float, float]):
        self.id = bone_id
        self.position = position


def parse_csv_data(csv_file: str, convert_coords: bool = True) -> dict[int, list[BoneData]]:
    """Parse VMD CSV data and return frame-indexed bone data."""
    frame_data = {}
    
    try:
        with open(csv_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = int(row["frame"])
                bone_name = row["bone_name"]
                
                # Skip unmapped bones or bones mapped to None
                if (
                    bone_name not in VMD_TO_FULLBODY_MAPPING
                    or VMD_TO_FULLBODY_MAPPING[bone_name] is None
                ):
                    continue
                
                bone_id = VMD_TO_FULLBODY_MAPPING[bone_name]
                
                # Parse position (VMD data appears to be in meters already)
                loc_x = float(row["loc_x"])
                loc_y = float(row["loc_y"])
                loc_z = float(row["loc_z"])
                
                if convert_coords:
                    # Convert Y-up to Z-up: (X, Y, Z) -> (X, -Z, Y)
                    position = (loc_x, -loc_z, loc_y)
                else:
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


class HandControlSystem:
    """Manages hand control retargeting and MuJoCo simulation."""
    
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.retargeting: SeqRetargeting | None = None
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.viewer: mujoco.viewer.Viewer | None = None
        self.retargeting_to_mujoco_map: list[tuple[int, int]] = []
        self.is_initialized = False
        
        self.setup_retargeting()
        self.setup_mujoco()
        
    def setup_retargeting(self):
        """Initialize the retargeting system."""
        config_path = get_default_config_path(
            RobotName[self.args.robot_name],
            RetargetingType[self.args.retargeting_type],
            HandType[self.args.hand_type],
        )
        if not config_path.exists():
            logger.error(f"Retargeting config not found: {config_path}")
            return
        
        urdf_dir = Path("References/dex-retargeting/assets/robots/hands")
        if not urdf_dir.exists():
            logger.error(f"URDF directory not found: {urdf_dir}")
            return
        
        RetargetingConfig.set_default_urdf_dir(urdf_dir)
        self.retargeting = RetargetingConfig.load_from_file(config_path).build()
        logger.info(f"Retargeting for {self.args.robot_name} initialized.")
        
    def setup_mujoco(self):
        """Initialize MuJoCo model, data, and viewer."""
        if not self.retargeting:
            return
            
        try:
            variant = ROBODESC_VARIANT[self.args.robot_name].replace("<SIDE>", self.args.hand_type)
            model = load_robot_description(f"{self.args.robot_name}_hand_mj_description", variant=variant)
        except Exception as e:
            logger.error(f"Failed to load model for {self.args.robot_name}: {e}")
            return
        
        data = mujoco.MjData(model)
        
        # Create joint mapping
        if self.args.robot_name in ROBOTS_WITH_DIFFERENT_JOINT_NAMES:
            D2M = DEX_RETARGETING_TO_MUJOCO_JOINT_NAMES[self.args.robot_name]
            retargeting_joint_names = [D2M[name] for name in self.retargeting.joint_names]
        else:
            retargeting_joint_names = self.retargeting.joint_names
        
        mujoco_joint_ids = []
        for name in retargeting_joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid == -1:
                logger.warning(f"Joint {name} not found in MuJoCo model.")
            mujoco_joint_ids.append(jid)
        
        # Create mapping between retargeted pose and qpos array
        self.retargeting_to_mujoco_map = []
        for i, jid in enumerate(mujoco_joint_ids):
            if jid != -1:
                self.retargeting_to_mujoco_map.append((i, model.jnt_qposadr[jid]))
        
        self.model = model
        self.data = data
        self.viewer = mujoco.viewer.launch_passive(model, data)
        self.is_initialized = True
        logger.info("MuJoCo scene initialized.")
        
    def extract_hand_keypoints(self, bone_data: list[BoneData]) -> tuple[np.ndarray, bool]:
        """Extract 21 MANO hand keypoints from bone data."""
        bone_map = {b.id.value: b for b in bone_data}
        mapping = (
            MANO_TO_OPENXR_RIGHT_HAND if self.args.hand_type == "right" 
            else MANO_TO_OPENXR_LEFT_HAND
        )
        
        hand_keypoints = np.zeros((21, 3))
        hand_detected = True
        
        for i in range(21):
            bone_id = mapping[i].value
            if bone_id in bone_map:
                hand_keypoints[i] = bone_map[bone_id].position
            else:
                hand_detected = False
                break
        
        return hand_keypoints, hand_detected
        
    def process_frame(self, bone_data: list[BoneData], frame_number: int):
        """Process a single frame of bone data."""
        if not self.is_initialized or not self.viewer.is_running():
            return
            
        hand_keypoints, hand_detected = self.extract_hand_keypoints(bone_data)
        
        if not hand_detected:
            logger.warning(f"Hand not detected in frame {frame_number}")
            return
            
        # Visualize hand keypoints
        rr.set_time_sequence("frame", frame_number)
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
        optimizer = self.retargeting.optimizer
        indices = optimizer.target_link_human_indices
        origin_indices = indices[0, :]
        task_indices = indices[1, :]
        origin_points = hand_keypoints[origin_indices, :]
        task_points = hand_keypoints[task_indices, :]
        target_vectors = task_points - origin_points
        
        # Retarget
        qpos = self.retargeting.retarget(ref_value=target_vectors)
        
        # Visualize retargeting
        robot = self.retargeting.optimizer.robot
        robot.compute_forward_kinematics(qpos)
        
        if optimizer.retargeting_type in ["VECTOR", "DEXPILOT"]:
            origin_link_names = optimizer.origin_link_names
            task_link_names = optimizer.task_link_names
            all_link_names = sorted(set(origin_link_names + task_link_names))
            
            rr.log(
                "world/dex_retargeting/target_vectors",
                rr.Arrows3D(origins=origin_points, vectors=target_vectors),
            )
            
            positions = np.array([
                robot.get_link_pose(robot.get_link_index(name))[:3, 3]
                for name in all_link_names
            ])
            
            rr.log(
                "world/robot_hand/points",
                rr.Points3D(positions=positions, radii=0.005),
            )
            
            link_name_to_idx = {name: i for i, name in enumerate(all_link_names)}
            connections = [
                (positions[link_name_to_idx[origin]], positions[link_name_to_idx[task]])
                for origin, task in zip(origin_link_names, task_link_names, strict=False)
            ]
            rr.log(
                "world/robot_hand/skeleton",
                rr.LineStrips3D(strips=connections, radii=0.002),
            )
        
        # Apply to MuJoCo model
        for retarget_idx, mujoco_idx in self.retargeting_to_mujoco_map:
            self.data.qpos[mujoco_idx] = qpos[retarget_idx]
        
        # Step simulation
        if ENABLE_DYNAMICS:
            mujoco.mj_step(self.model, self.data)
        else:
            mujoco.mj_forward(self.model, self.data)
        self.viewer.sync()


def main():
    parser = argparse.ArgumentParser(description="Dextrous Hand Control from VMD CSV files")
    parser.add_argument("--file", type=str, required=True, help="Path to VMD CSV file containing pose data")
    parser.add_argument("--robot-name", type=str, default=DEFAULT_ROBOT, 
                      choices=[r.name for r in RobotName], help="Robot hand to use")
    parser.add_argument("--retargeting-type", type=str, default="vector", 
                      choices=[t.name for t in RetargetingType], help="Retargeting method")
    parser.add_argument("--hand-type", type=str, default="right", choices=["left", "right"], 
                      help="Which hand to track")
    parser.add_argument("--frame", type=int, help="Specific frame to visualize")
    parser.add_argument("--animate", action="store_true", help="Animate through all frames")
    parser.add_argument("--fps", type=float, default=30.0, help="Animation FPS (default: 30)")
    parser.add_argument("--no-coord-conversion", action="store_true", 
                      help="Skip coordinate system conversion from Y-up to Z-up")
    parser.add_argument("--log-level", type=str, default="INFO", 
                      help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)")
    
    args = parser.parse_args()
    
    configure_logging(level=args.log_level.upper())
    
    # Parse CSV data
    print(f"Loading VMD pose data from {args.file}...")
    frame_data = parse_csv_data(args.file, convert_coords=not args.no_coord_conversion)
    
    if not frame_data:
        print("No valid pose data found!")
        return
    
    print(f"Loaded {len(frame_data)} frames")
    
    # Initialize rerun
    rr.init("dextrous_hand_control_vmd", spawn=True)
    
    # Set coordinate system based on conversion option
    if args.no_coord_conversion:
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
    else:
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
    
    # Initialize hand control system
    hand_system = HandControlSystem(args)
    if not hand_system.is_initialized:
        print("Failed to initialize hand control system")
        return
    
    # Process frames
    sorted_frames = sorted(frame_data.keys())
    
    if args.animate:
        print("Animating through frames... Press Ctrl+C to stop")
        try:
            sleep_time = 1.0 / args.fps
            while hand_system.viewer.is_running():
                for frame_num in sorted_frames:
                    if not hand_system.viewer.is_running():
                        break
                    hand_system.process_frame(frame_data[frame_num], frame_num)
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nAnimation stopped")
    else:
        # Show specific frame or frame 1
        target_frame = args.frame if args.frame is not None else sorted_frames[0]
        if target_frame in frame_data:
            print(f"Processing frame {target_frame}")
            hand_system.process_frame(frame_data[target_frame], target_frame)
            
            # Keep viewer open
            print("Press Ctrl+C to exit...")
            try:
                while hand_system.viewer.is_running():
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        else:
            print(f"Frame {target_frame} not found. Available frames: {sorted_frames[:10]}...")
    
    if hand_system.viewer:
        hand_system.viewer.close()


if __name__ == "__main__":
    main()