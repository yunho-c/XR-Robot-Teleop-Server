"""
This script demonstrates how to perform dextrous hand control by retargeting human hand poses
from a WebRTC stream to a robot hand in the MuJoCo simulator.

You need to have the dex-retargeting library and its dependencies installed.
You can install the extra dependencies for this example by running:
pip install -e .[example]
in the `References/dex-retargeting` directory. You will also need to install mujoco.

This script will start a WebRTC server. You can then connect to it from the client
(e.g., XR Robot Teleop Client Unity application) that sends full-body pose data.
The script will extract the hand pose, retarget it to a specified robot hand,
and visualize the result in MuJoCo.
"""

# TODO: update instructions in docstring
# TODO: make visualization pretty (& intuitive)
# TODO: think whether data should move to a separate file or into core library

import argparse
import time
from functools import partial
from pathlib import Path
from typing import Any

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
from scipy.spatial.transform import Rotation as R

from xr_robot_teleop_server import configure_logging, logger
from xr_robot_teleop_server.schemas.body_pose import deserialize_pose_data
from xr_robot_teleop_server.schemas.openxr_skeletons import FullBodyBoneId
from xr_robot_teleop_server.streaming import WebRTCServer
from xr_robot_teleop_server.utils.frame import get_hand_centric_coordinates, rotate_bones

# Params
# DEBUG = True
DEBUG = False
ENABLE_DYNAMICS = False

# NOTE: see RobotName enum for possible values
# DEFAULT_ROBOT: str = "allegro"
DEFAULT_ROBOT: str = "shadow"
# DEFAULT_ROBOT: str = "leap"
# DEFAULT_ROBOT: str = "ability"  # TODO: confirm joint name mapping using LLMs
# DEFAULT_ROBOT: str = "panda"
# DEFAULT_ROBOT: str = "svh"  # WARNING: unsupported
# DEFAULT_ROBOT: str = "inspire"  # WARNING: unsupported


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


class AppState:
    """State object to hold retargeting and MuJoCo objects."""

    def __init__(self, args: argparse.Namespace, visualizer: Any | None = None):
        self.args = args
        self.visualizer = visualizer
        self.retargeting: SeqRetargeting | None = None
        self.retargeting_config: RetargetingConfig | None = None
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.viewer: mujoco.viewer.Viewer | None = None
        self.retargeting_to_mujoco_map: np.ndarray | None = None
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


def on_body_pose_message(message: bytes, state: AppState):
    """Callback for handling body pose messages from the WebRTC client."""
    if not state.is_initialized:
        logger.warning("App state not initialized, skipping message.")
        return

    try:
        if isinstance(message, bytes) and state.viewer.is_running():
            if state.visualizer:
                rr = state.visualizer
                rr.set_time("body_pose_timestamp", sequence=int(time.time()))

            pose_data = deserialize_pose_data(message, z_up=True)
            if not pose_data:
                logger.warning("Body pose data is empty.")
                return

            hand_data = get_hand_centric_coordinates(pose_data, state.args.hand_type)
            hand_data = rotate_bones(hand_data, R.from_rotvec(np.array([0, 0, 1]) * np.pi))

            bone_map = {b.id: b for b in hand_data}
            mapping = (
                MANO_TO_OPENXR_RIGHT_HAND
                if state.args.hand_type == "right"
                else MANO_TO_OPENXR_LEFT_HAND
            )

            # Extract 21 hand keypoints
            hand_keypoints = np.zeros((21, 3))
            hand_detected = True
            for i in range(21):
                bone_id = mapping[i].value
                if bone_id in bone_map:
                    hand_keypoints[i] = bone_map[bone_id].position
                else:
                    hand_detected = False
                    break

            if hand_detected:
                if state.visualizer:
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

                if state.visualizer:
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
                                # TODO: remainders (colors, radii, labels)
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

    except Exception as e:
        logger.error(f"Could not process body pose data: {e}", exc_info=True)
        if state.viewer and state.viewer.is_running():
            state.viewer.close()


def main():
    parser = argparse.ArgumentParser(description="Dextrous Hand Control Example using MuJoCo")
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
        # default="position",
        default="vector",
        # default="dexpilot",
        # choices=[t.name for t in RetargetingType if t != RetargetingType.position],
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

    rr.init("dextrous_hand_control", spawn=True)
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

    state_factory = partial(AppState, args=args, visualizer=rr)
    data_handlers = {
        "body_pose": on_body_pose_message,
    }

    server = WebRTCServer(
        datachannel_handlers=data_handlers,
        state_factory=state_factory,
    )

    logger.info("Starting WebRTC server. Open the client to begin streaming.")
    try:
        server.run()
    except KeyboardInterrupt:
        logger.info("Server shutting down.")
    finally:
        # In-process cleanup if needed, though server.run() should handle its own threads
        pass


if __name__ == "__main__":
    main()
