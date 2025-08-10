import argparse
from functools import partial
from typing import Any, Literal

import numpy as np

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.body_pose import (
    Bone,
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
ARROW_LENGTH = 0.05

# Choose the reference frame for the bone rotations.
# "world" - rotation with respect to the world frame
# "parent" - rotation with respect to the parent bone
ROTATION_REFERENCE: Literal["world", "parent"] = "parent"
# NOTE (yunho-c): I am pretty sure(?) that Unity-MetaXR body pose
#      output is in the world frame... but I'm not an expert on this.


# Coordinate system conversion for Unity data
CONVERT_UNITY_COORDS = True


LABELS = ["x", "y", "z"]


# Quaternion utilities
def q_conjugate(q: np.ndarray) -> np.ndarray:
    """Calculate the conjugate of a quaternion."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])


def qv_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by a quaternion."""
    q_vec = np.array([v[0], v[1], v[2], 0.0])
    q_rotated = q_multiply(q_multiply(q, q_vec), q_conjugate(q))
    return q_rotated[:3]


# Define a state object to pass visualizer into aiortc
class AppState:
    def __init__(self, visualizer: Any | None = None):
        self.visualizer = visualizer
        self.parent_map = {
            child.value: parent.value for parent, child in FULL_BODY_SKELETON_CONNECTIONS
        }

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
                # rr.set_time_sequence("body_pose_timestamp", int(time.time() * 1000))

                positions = []
                keypoint_ids = []
                bone_map: dict[int, Bone] = {}

                for bone in pose_data:
                    # NOTE: not all bones are being tracked, so we need to filter
                    bone_label = get_bone_label(SkeletonType.FullBody, bone.id)
                    if bone_label and "Unknown" not in bone_label:
                        positions.append(bone.position)
                        keypoint_ids.append(bone.id)
                        bone_map[bone.id] = bone

                rr.log(
                    "world/user/bones",
                    rr.Points3D(
                        positions=positions,
                        keypoint_ids=keypoint_ids,
                        class_ids=SkeletonType.FullBody.value,
                        radii=VIZ_POINT_RADIUS,
                    ),
                )

                # Visualize local frames
                origins = []
                vectors = []
                colors = []
                labels = []

                for bone_id, bone in bone_map.items():
                    rotation = np.array(bone.rotation)

                    if ROTATION_REFERENCE == "parent":
                        parent_id = state.parent_map.get(bone_id)
                        if parent_id in bone_map:
                            parent_bone = bone_map[parent_id]
                            parent_rotation = np.array(parent_bone.rotation)
                            # rotation = relative rotation
                            rotation = q_multiply(q_conjugate(parent_rotation), rotation)

                    # Basis vectors
                    x_axis = qv_rotate(rotation, np.array([ARROW_LENGTH, 0, 0]))
                    y_axis = qv_rotate(rotation, np.array([0, ARROW_LENGTH, 0]))
                    z_axis = qv_rotate(rotation, np.array([0, 0, ARROW_LENGTH]))

                    # Add arrows for visualization
                    origins.extend([bone.position] * 3)
                    vectors.extend([x_axis, y_axis, z_axis])
                    colors.extend([[255, 0, 0], [0, 255, 0], [0, 0, 255]])

                    labels.extend(LABELS)

                if origins:
                    rr.log(
                        "world/user/bone_frames",
                        rr.Arrows3D(
                            origins=origins,
                            vectors=vectors,
                            colors=colors,
                            radii=0.001,
                            labels=labels,
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
        colormap = cm.get_cmap("viridis")
        # colormap = cm.get_cmap("jet")
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
