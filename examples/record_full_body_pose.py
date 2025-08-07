import argparse  # noqa: I001
import time
from functools import partial
from typing import Any

import numpy as np

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.body_pose import (
    deserialize_pose_data,
)
from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
    get_bone_label,
)
from xr_robot_teleop_server.streaming import WebRTCServer

# Params
# body pose visualization
VISUALIZE = True
VIZ_POINT_RADIUS = 0.01

# Coordinate system conversion for Unity data
CONVERT_UNITY_COORDS = True


# Define a state object to pass visualizer into aiortc
class AppState:
    def __init__(self, visualizer: Any | None = None):
        self.visualizer = visualizer

    def __repr__(self):
        return f"<AppState visualizer={self.visualizer}>"


def on_body_pose_message(message: bytes, state: AppState):
    try:
        if isinstance(message, bytes):
            pose_data = deserialize_pose_data(message, z_up=CONVERT_UNITY_COORDS)
            # print(f"Received {len(pose_data)} bones")

            # Log to rerun
            if state.visualizer:
                rr = state.visualizer
                # Arbitrary timestamp for visualization timeline
                rr.set_time_sequence("body_pose_timestamp", int(time.time() * 1000))

                positions = []
                keypoint_ids = []
                for bone in pose_data:
                    # NOTE: not all bones are being tracked, so we need to filter
                    bone_label = get_bone_label(SkeletonType.FullBody, bone.id)
                    if bone_label and "Unknown" not in bone_label:
                        positions.append(bone.position)
                        keypoint_ids.append(bone.id)

                rr.log(
                    "world/user/bones",
                    rr.Points3D(
                        positions=positions,
                        keypoint_ids=keypoint_ids,
                        class_ids=SkeletonType.FullBody.value,
                        radii=VIZ_POINT_RADIUS,
                    ),
                )

    except Exception as e:
        print(f"Could not process body pose data: {e}")


# Start server
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XR 360 Camera Streamer")
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    parser.add_argument(
        "--visualize", action="store_true", help="Enable 3D visualization with rerun."
    )
    args = parser.parse_args()

    # Configure logging
    configure_logging(level=args.log_level)

    rr = None
    state_factory = AppState
    if VISUALIZE or args.visualize:
        try:
            import rerun as rr
            from matplotlib import colormaps as cm
        except ImportError:
            print("Please install OpenCV, rerun SDK and matplotlib: pip install -e .[viz]")
            exit(1)

        rr.init("xr-robot-teleop-server", spawn=True)
        if CONVERT_UNITY_COORDS:
            # Set coordinate system to right-handed, Z-up
            rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)  # NOTE: same as FLU
        else:
            rr.log("world", rr.ViewCoordinates.LEFT_HAND_Y_UP, static=True)  # Set Y as the up axis
            print("Warning: rerun currently does not support left-handed coordinate systems.")

        # Create a ClassDescription for the full body skeleton.
        # This provides the mapping from Id to Label for the rerun viewer.
        # The keypoint_connections will be used to draw the skeleton.
        # colormap = cm.get_cmap("viridis")
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
