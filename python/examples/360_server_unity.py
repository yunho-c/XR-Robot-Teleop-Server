import argparse
import asyncio
import json
import os
import struct
import time
from functools import partial
from pathlib import Path
from typing import Any, Optional

import numpy as np
from aiortc import MediaStreamTrack
from av import VideoFrame

from xr_360_camera_streamer.sources import FFmpegFileSource, OpenCVFileSource
from xr_360_camera_streamer.streaming import WebRTCServer
from xr_360_camera_streamer.transforms import EquilibEqui2Pers

# Params
# video source library
VIDEO_SOURCE = FFmpegFileSource
# VIDEO_SOURCE = OpenCVFileSource

# body pose visualization
VISUALIZE = True
# VISUALIZE = False


# Define a state object for orientation
class AppState:
    def __init__(self, visualizer: Optional[Any] = None):
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

            # Log to rerun
            if state.visualizer:
                rr = state.visualizer
                # Arbitrary timestamp for visualization timeline
                rr.set_time_sequence("body_pose_timestamp", int(time.time() * 1000))
                for bone in pose_data:
                    rr.log(
                        f"world/user/bones/{bone.id}",
                        rr.Points3D(positions=[bone.position]),
                    )

    except Exception as e:
        print(f"Could not process body pose data: {e}")


# Factory for creating the video track
def create_video_track(state: AppState):
    video_path = os.path.join(
        Path(__file__).parents[2],
        "xr-360-streamer-assets",
        "videos",
        "test_video.mp4",
    )

    if not os.path.exists(video_path):
        raise FileNotFoundError(
            f"Video asset not found at {video_path}. "
            "Please download the assets from the repository "
            "and place them in `xr-360-streamer-assets` at the project root."
        )

    # Initialize the video source and transform
    video_source = VIDEO_SOURCE(video_path)
    video_transform = EquilibEqui2Pers(output_width=1280, output_height=720, fov_x=state.fov_x)

    return ReprojectionTrack(state, video_source, video_transform)


# Start server
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XR 360 Camera Streamer")
    parser.add_argument(
        "--visualize", action="store_true", help="Enable 3D visualization with rerun."
    )
    args = parser.parse_args()

    rr = None
    state_factory = AppState
    if VISUALIZE or args.visualize:
        try:
            import rerun as rr
        except ImportError:
            print("Please install rerun SDK: pip install -e .[viz]")
            exit(1)

        rr.init("xr-360-camera-streamer", spawn=True)
        rr.log("world", rr.ViewCoordinates.LEFT_HAND_Y_UP, static=True)  # Set Y as the up axis
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
