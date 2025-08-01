import argparse  # noqa: I001
import json
import os
import struct
import time
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
from aiortc import MediaStreamTrack
from av import VideoFrame

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
    get_bone_label,
)
from xr_robot_teleop_server.sources import FFmpegFileSource
from xr_robot_teleop_server.streaming import WebRTCServer
from xr_robot_teleop_server.transforms import EquilibEqui2Pers

# Params
# video source library
VIDEO_SOURCE = FFmpegFileSource
# VIDEO_SOURCE = OpenCVFileSource

# body pose visualization
VISUALIZE = True
# VISUALIZE = False

VIZ_POINT_RADIUS = 0.01

# Coordinate system conversion for Unity data
CONVERT_UNITY_COORDS = True


def convert_unity_to_right_handed_z_up(
    position: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Converts position and rotation from Unity's left-handed, Y-up coordinate system
    to a right-handed, Z-up coordinate system (+X Forward, +Y Left, +Z Up).
    """
    # Position conversion: Unity (x,y,z) -> (z, -x, y)
    new_position = (position[2], -position[0], position[1])

    # Rotation quaternion conversion: Unity (qx,qy,qz,qw) -> (-qz, qx, -qy, qw)
    qx, qy, qz, qw = rotation
    new_rotation = (-qz, qx, -qy, qw)

    return new_position, new_rotation


# Define a state object for orientation
class AppState:
    def __init__(self, visualizer: Any | None = None):
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.fov_x = 90.0  # Horizontal FOV in degrees
        self.visualizer = visualizer

    def __repr__(self):
        return (
            f"<AppState pitch={self.pitch}, yaw={self.yaw}, roll={self.roll}, fov_x={self.fov_x}>"
        )

    def get_rot(self) -> dict[str, float]:
        return {"pitch": self.pitch, "yaw": self.yaw, "roll": self.roll}


# Define a simple data structure to hold the bone data
class Bone:
    def __init__(
        self,
        id: int,
        position: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ):
        self.id = id
        self.position = position
        self.rotation = rotation

    def __repr__(self):
        return f"Bone(pos={self.position}, rot={self.rotation})"


def deserialize_pose_data(data: bytes) -> list[Bone]:
    """
    Deserializes the binary pose data stream from the Unity client.

    Args:
        data: The raw byte string received from the data channel.

    Returns:
        A list of Bone objects.
    """
    bones = []
    offset = 0

    # The C# BinaryWriter is little-endian by default. The format string '<' specifies this.
    # Format: 1 int (id) + 7 floats (pos/rot) = 8 values
    # '<i' = little-endian integer (4 bytes)
    # '<7f' = 7 little-endian floats (7 * 4 = 28 bytes)
    # Total size per bone: 32 bytes

    try:
        # 1. Read the number of bones (an integer)
        (bone_count,) = struct.unpack_from("<i", data, offset)
        offset += 4

        # 2. Loop for each bone to read its data
        for _ in range(bone_count):
            # Ensure there is enough data left in the buffer
            if offset + 32 > len(data):
                print("Error: Incomplete data buffer for a bone.")
                break

            # 3. Unpack 8 values: id, pos(x,y,z), rot(x,y,z,w)
            bone_data = struct.unpack_from("<i7f", data, offset)
            offset += 32

            bone_id = bone_data[0]
            position = (bone_data[1], bone_data[2], bone_data[3])
            rotation = (bone_data[4], bone_data[5], bone_data[6], bone_data[7])

            bones.append(Bone(bone_id, position, rotation))

    except struct.error as e:
        print(f"Error deserializing pose data: {e}")

    return bones


# Define a custom video track that applies reprojection
class ReprojectionTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, state: AppState, source: VIDEO_SOURCE, transform: EquilibEqui2Pers):
        super().__init__()
        self.state = state
        self.source = source
        self.transform = transform
        self._timestamp = 0

    async def recv(self):
        equi_frame_rgb = next(self.source)  # ALT

        # Get current orientation from the shared state
        rot = self.state.get_rot()

        # # Apply the equirectangular-to-perspective transform
        perspective_frame = self.transform.transform(frame=equi_frame_rgb, rot=rot)

        # Create a VideoFrame for aiortc
        frame = VideoFrame.from_ndarray(perspective_frame, format="rgb24")

        # Set timestamp
        time_base = 90000
        frame.pts = self._timestamp
        frame.time_base = time_base
        self._timestamp += int(time_base / self.source.fps)

        return frame


# Data channel handler to update orientation state
def on_camera_message(message: str, state: AppState):
    try:
        data = json.loads(message)
        # print(f"Received camera data: {data}")
        state.pitch = np.deg2rad(float(data.get("pitch", np.rad2deg(state.pitch))))
        state.yaw = np.deg2rad(float(data.get("yaw", np.rad2deg(state.yaw))))
        state.roll = np.deg2rad(float(data.get("roll", np.rad2deg(state.roll))))
        state.fov_x = float(data.get("fov_x", state.fov_x))
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        print(f"Could not process camera data: {e}")


def on_body_pose_message(message: bytes, state: AppState):
    try:
        if isinstance(message, bytes):
            pose_data = deserialize_pose_data(message)
            # print(f"Received {len(pose_data)} bones")

            if CONVERT_UNITY_COORDS:
                for bone in pose_data:
                    bone.position, bone.rotation = convert_unity_to_right_handed_z_up(
                        bone.position, bone.rotation
                    )

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


# Factory for creating the video track
def create_video_track(state: AppState):
    video_path = os.path.join(
        Path(__file__).parents[2],
        "xr-robot-teleop-server-assets",
        "videos",
        "test_video.mp4",
    )

    if not os.path.exists(video_path):
        raise FileNotFoundError(
            f"Video asset not found at {video_path}. "
            "Please download the assets from the repository "
            "and place them in `xr-robot-teleop-server-assets` at the project root."
        )

    # Initialize the video source and transform
    video_source = VIDEO_SOURCE(video_path)
    video_transform = EquilibEqui2Pers(output_width=1280, output_height=720, fov_x=state.fov_x)

    return ReprojectionTrack(state, video_source, video_transform)


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
        "camera": on_camera_message,
        "body_pose": on_body_pose_message,
    }

    server = WebRTCServer(
        video_track_factory=create_video_track,
        datachannel_handlers=data_handlers,
        state_factory=state_factory,
    )

    server.run()
