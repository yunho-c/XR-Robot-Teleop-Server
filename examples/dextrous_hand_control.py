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

import argparse
import logging
from functools import partial
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from dex_retargeting.constants import (
    HandType,
    RetargetingType,
    RobotName,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting
from robot_descriptions.loaders.mujoco import load_robot_description

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.body_pose import deserialize_pose_data
from xr_robot_teleop_server.schemas.openxr_skeletons import FullBodyBoneId
from xr_robot_teleop_server.streaming import WebRTCServer
from xr_robot_teleop_server.utils.frame import get_hand_centric_coordinates

# Logger
logger = logging.getLogger(__name__)

# This mapping converts the 21-point MANO hand keypoint structure to the
# OpenXR FullBodyBoneId enum for the right hand. We are skipping the
# "intermediate" phalanx bones from OpenXR to match MANO's structure.
MANO_TO_OPENXR_RIGHT_HAND: dict[int, FullBodyBoneId] = {
    0: FullBodyBoneId.FullBody_RightHandWrist,
    1: FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    2: FullBodyBoneId.FullBody_RightHandThumbProximal,
    3: FullBodyBoneId.FullBody_RightHandThumbDistal,
    4: FullBodyBoneId.FullBody_RightHandThumbTip,
    5: FullBodyBoneId.FullBody_RightHandIndexMetacarpal,
    6: FullBodyBoneId.FullBody_RightHandIndexProximal,
    7: FullBodyBoneId.FullBody_RightHandIndexDistal,  # Skip Intermediate
    8: FullBodyBoneId.FullBody_RightHandIndexTip,
    9: FullBodyBoneId.FullBody_RightHandMiddleMetacarpal,
    10: FullBodyBoneId.FullBody_RightHandMiddleProximal,
    11: FullBodyBoneId.FullBody_RightHandMiddleDistal,  # Skip Intermediate
    12: FullBodyBoneId.FullBody_RightHandMiddleTip,
    13: FullBodyBoneId.FullBody_RightHandRingMetacarpal,
    14: FullBodyBoneId.FullBody_RightHandRingProximal,
    15: FullBodyBoneId.FullBody_RightHandRingDistal,  # Skip Intermediate
    16: FullBodyBoneId.FullBody_RightHandRingTip,
    17: FullBodyBoneId.FullBody_RightHandLittleMetacarpal,
    18: FullBodyBoneId.FullBody_RightHandLittleProximal,
    19: FullBodyBoneId.FullBody_RightHandLittleDistal,  # Skip Intermediate
    20: FullBodyBoneId.FullBody_RightHandLittleTip,
}

# Mapping for the left hand
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

DEX_RETARGETING_TO_MUJOCO_JOINT_NAMES: dict[str, dict[str, str]] = {
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

    def __init__(self, args: argparse.Namespace):
        self.args = args
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
                f"URDF directory not found: {urdf_dir}. "
                "Please make sure the git submodule is initialized: "
                "`git submodule update --init --recursive`"
            )
            return
        RetargetingConfig.set_default_urdf_dir(urdf_dir)
        self.retargeting_config = RetargetingConfig.load_from_file(config_path)
        self.retargeting = self.retargeting_config.build()
        logger.info(f"Retargeting for {args.robot_name} initialized.")

        # Setup MuJoCo
        self._setup_mujoco(args.robot_name)
        if self.viewer is None:  # Setup failed
            return

        self.is_initialized = True

    def _setup_mujoco(self, robot_name):
        """Initializes the MuJoCo model, data, and viewer."""
        try:
            model = load_robot_description(f"{robot_name}_hand_mj_description")
        except Exception as e:
            logger.error(f"Failed to load model for {robot_name} from `robot_descriptions`: {e}")
            return

        data = mujoco.MjData(model)

        # # DEBUG: Iterate through all joints in the model
        # for i in range(model.njnt):
        #     joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        #     qpos_address = model.jnt_qposadr[i]
        #     print(f"Joint '{joint_name}' (ID: {i}) starts at qpos index: {qpos_address}")

        D2M = DEX_RETARGETING_TO_MUJOCO_JOINT_NAMES[robot_name]  # alias

        # Create joint mapping
        retargeting_joint_names = self.retargeting.joint_names
        mujoco_joint_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, D2M[name])
            for name in retargeting_joint_names
        ]

        # Filter out joints not found in the MuJoCo model (-1)
        # and create a mapping for the qpos array
        self.retargeting_to_mujoco_map = []
        for i, jid in enumerate(mujoco_joint_ids):
            if jid != -1:
                # (index_in_retargeting_qpos, index_in_mujoco_qpos)
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
            pose_data = deserialize_pose_data(message, z_up=True)
            if not pose_data:
                return

            hand_data = get_hand_centric_coordinates(pose_data, state.args.hand_type)

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
                # print(f"{qpos=}")  # DEBUG

                # Apply to MuJoCo model
                for retarget_idx, mujoco_idx in state.retargeting_to_mujoco_map:
                    state.data.qpos[mujoco_idx] = qpos[retarget_idx]

            # Step the simulation and render
            # mujoco.mj_step(state.model, state.data)  # original
            mujoco.mj_forward(state.model, state.data)  # DEBUG: update kinematics only
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
        default="ability",
        choices=[r.name for r in RobotName],
        help="Name of the robot hand to use.",
    )
    parser.add_argument(
        "--retargeting-type",
        type=str,
        default="dexpilot",
        choices=[t.name for t in RetargetingType if t != RetargetingType.position],
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

    state_factory = partial(AppState, args=args)
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
