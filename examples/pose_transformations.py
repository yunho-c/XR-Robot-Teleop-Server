import argparse
import time
from functools import partial
from typing import Any

import numpy as np
import rerun as rr

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.body_pose import (
    Bone,
    deserialize_pose_data,
)
from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
)
from xr_robot_teleop_server.streaming import WebRTCServer
from xr_robot_teleop_server.utils.frame import (
    get_body_centric_coordinates,
    get_body_frame,
    get_hand_centric_coordinates,
    get_hand_frame,
)

# Params
# body pose visualization
VIZ_POINT_RADIUS = 0.01

# Coordinate system conversion for Unity data
CONVERT_UNITY_COORDS = True


# Define a state object to pass visualizer into aiortc
class AppState:
    def __init__(self, visualizer: Any | None = None):
        self.visualizer = visualizer

    def __repr__(self):
        return f"<AppState visualizer={self.visualizer}>"


def log_skeleton(entity_path: str, bones: list[Bone], rr_instance: rr, keypoint_radius: float):
    """Logs a skeleton to rerun."""
    if not bones:
        return

    positions = [b.position for b in bones]
    keypoint_ids = [b.id for b in bones]
    rr_instance.log(
        entity_path,
        rr_instance.Points3D(
            positions=positions,
            keypoint_ids=keypoint_ids,
            class_ids=SkeletonType.FullBody.value,
            radii=keypoint_radius,
        ),
    )


def on_body_pose_message(message: bytes, state: AppState):
    try:
        if isinstance(message, bytes) and state.visualizer:
            rr = state.visualizer
            pose_data = deserialize_pose_data(message, z_up=CONVERT_UNITY_COORDS)

            # Arbitrary timestamp for visualization timeline
            rr.set_time_sequence("body_pose_timestamp", int(time.time() * 1000))

            # 1. Log original skeleton
            log_skeleton("world/skeleton", pose_data, rr, VIZ_POINT_RADIUS)

            # 2. Test and visualize body-centric coordinates
            body_centric_bones = get_body_centric_coordinates(pose_data)
            log_skeleton("body_centric/skeleton", body_centric_bones, rr, VIZ_POINT_RADIUS)

            bone_positions = {b.id: np.array(b.position) for b in pose_data}
            body_origin, body_rot = get_body_frame(bone_positions)
            if body_origin is not None and body_rot is not None:
                rr.log(
                    "world/body_frame",
                    rr.Transform3D(translation=body_origin, mat3x3=body_rot),
                )

            # 3. Test and visualize hand-centric coordinates for left hand
            left_hand_centric_bones = get_hand_centric_coordinates(pose_data, "left")
            log_skeleton(
                "hand_centric_left/skeleton",
                left_hand_centric_bones,
                rr,
                VIZ_POINT_RADIUS,
            )

            # Visualize the hand frame
            left_hand_origin, left_hand_rot = get_hand_frame("left", bone_positions)
            if left_hand_origin is not None and left_hand_rot is not None:
                rr.log(
                    "world/left_hand_frame",
                    rr.Transform3D(translation=left_hand_origin, mat3x3=left_hand_rot),
                )

            # 4. Test and visualize hand-centric coordinates for right hand
            right_hand_centric_bones = get_hand_centric_coordinates(pose_data, "right")
            log_skeleton(
                "hand_centric_right/skeleton",
                right_hand_centric_bones,
                rr,
                VIZ_POINT_RADIUS,
            )

            # Visualize the hand frame
            right_hand_origin, right_hand_rot = get_hand_frame("right", bone_positions)
            if right_hand_origin is not None and right_hand_rot is not None:
                rr.log(
                    "world/right_hand_frame",
                    rr.Transform3D(translation=right_hand_origin, mat3x3=right_hand_rot),
                )

    except Exception as e:
        print(f"Could not process body pose data: {e}")


# Start server
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XR Pose Transformation Tester")
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    args = parser.parse_args()

    # Configure logging
    configure_logging(level=args.log_level)

    rr.init("xr_robot_teleop_server_pose_test", spawn=True)
    if CONVERT_UNITY_COORDS:
        # Set coordinate system to right-handed, Z-up
        rr.log("hand_centric_left", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log("hand_centric_right", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log("body_centric", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    else:
        rr.log("world", rr.ViewCoordinates.LEFT_HAND_Y_UP, static=True)
        print("Warning: rerun currently does not support left-handed coordinate systems.")

    # Create a ClassDescription for the full body skeleton.
    from matplotlib import colormaps as cm

    colormap = cm.get_cmap("jet")
    keypoint_annotations = [
        rr.AnnotationInfo(
            id=member.value,
            label=member.name,
            color=(np.array(colormap(member.value / FullBodyBoneId.FullBody_End)) * 255).astype(
                np.uint8
            ),
        )
        for member in FullBodyBoneId
    ]

    rr.log(
        "/",  # Log to the root path
        rr.AnnotationContext(
            rr.ClassDescription(
                info=rr.AnnotationInfo(
                    id=SkeletonType.FullBody.value,
                    label="SkeletonType.FullBody",
                    color=np.array([251, 251, 251, 251], dtype=np.uint8),
                ),
                keypoint_annotations=keypoint_annotations,
                keypoint_connections=FULL_BODY_SKELETON_CONNECTIONS,
            )
        ),
        static=True,
    )

    state_factory = partial(AppState, visualizer=rr)

    data_handlers = {
        "body_pose": on_body_pose_message,
    }

    server = WebRTCServer(
        datachannel_handlers=data_handlers,
        state_factory=state_factory,
    )

    server.run()
