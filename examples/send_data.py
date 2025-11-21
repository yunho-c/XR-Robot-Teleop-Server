"""
Visualize body pose data and demonstrate server-initiated data channel sends.

This example is based on `examples/visualize_body_pose.py` but also:
  * keeps track of connected data channels
  * exposes an HTTP endpoint to send a message on-demand
  * optionally emits periodic "ping" messages on a control channel

Expected data channels from the client:
  - "body_pose": bytes payloads produced by `serialize_pose_data`
  - "control": any string/bytes channel that can receive commands
"""

import argparse  # noqa: I001
import asyncio
import json
import time
import uuid
import logging
from contextlib import asynccontextmanager, suppress
from functools import partial
from typing import Any

import numpy as np
from fastapi import Body
from pydantic import BaseModel

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.body_pose import deserialize_pose_data
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
CONTROL_CHANNEL_LABEL = "control"
# DEFAULT_PING_INTERVAL = 10.0  # seconds; set to 0 to disable
DEFAULT_PING_INTERVAL = 1.0  # seconds; set to 0 to disable

logger = logging.getLogger(__name__)


class AppState:
    """
    Holds per-peer state, including an optional rerun visualizer and a peer_id
    used to target sends through the HTTP helper.
    """

    def __init__(self, visualizer: Any | None = None):
        self.visualizer = visualizer
        self.peer_id = str(uuid.uuid4())

    def __repr__(self):
        return f"<AppState peer_id={self.peer_id} visualizer={self.visualizer}>"


def on_body_pose_message(message: bytes, state: AppState, channel=None):
    try:
        if isinstance(message, bytes):
            pose_data = deserialize_pose_data(message, z_up=CONVERT_UNITY_COORDS)
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


def on_control_message(message: bytes | str, state: AppState, channel=None):
    logger.info("Received control message from peer %s: %s", state.peer_id, message)


async def periodic_ping(server: WebRTCServer, interval: float):
    """Periodically send a ping message on the control channel."""
    while True:
        payload = json.dumps({"type": "ping", "sent_at": time.time()})
        logger.debug("Sending ping on '%s': %s", CONTROL_CHANNEL_LABEL, payload)
        deliveries = server.send_to_datachannel(CONTROL_CHANNEL_LABEL, payload)
        if deliveries:
            print(f"Sent ping to {deliveries} peer(s) on '{CONTROL_CHANNEL_LABEL}'")
        await asyncio.sleep(interval)


def _find_state_by_peer_id(server: WebRTCServer, peer_id: str) -> Any | None:
    """Helper to look up a state by peer_id (stored on AppState)."""
    for ctx in server._peer_context.values():  # noqa: SLF001
        state = ctx.get("state")
        if getattr(state, "peer_id", None) == peer_id:
            return state
    return None


class ControlRequest(BaseModel):
    message: str
    label: str = CONTROL_CHANNEL_LABEL
    target_peer_id: str | None = None


# Start server
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XR Body Pose Visualizer with Control Channel")
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    parser.add_argument(
        "--visualize", action="store_true", help="Enable 3D visualization with rerun."
    )
    parser.add_argument(
        "--ping-interval",
        type=float,
        default=DEFAULT_PING_INTERVAL,
        help="Seconds between server-initiated control pings (0 to disable).",
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
        CONTROL_CHANNEL_LABEL: on_control_message,
    }

    server = WebRTCServer(
        datachannel_handlers=data_handlers,
        state_factory=state_factory,
        server_data_channels=[CONTROL_CHANNEL_LABEL],
    )

    # FastAPI lifespan is already provided by WebRTCServer, so wrap it to add ping task management.
    original_lifespan = server.app.router.lifespan_context

    @asynccontextmanager
    async def lifespan(app):
        task = None
        # Use original lifespan if present; otherwise provide a no-op wrapper
        if original_lifespan:
            base_ctx = original_lifespan(app)
        else:
            @asynccontextmanager
            async def _noop(_app):
                yield

            base_ctx = _noop(app)
        async with base_ctx:
            if args.ping_interval and args.ping_interval > 0:
                task = asyncio.create_task(periodic_ping(server, args.ping_interval))
                app.state.ping_task = task
            try:
                yield
            finally:
                if task:
                    task.cancel()
                    with suppress(asyncio.CancelledError):
                        await task

    server.app.router.lifespan_context = lifespan

    @server.app.post("/control/send")
    async def send_control(req: ControlRequest = Body(...)):
        target_state = None
        if req.target_peer_id:
            target_state = _find_state_by_peer_id(server, req.target_peer_id)
            if target_state is None:
                return {"delivered": 0, "error": f"Peer {req.target_peer_id} not found"}
        deliveries = server.send_to_datachannel(req.label, req.message, state=target_state)
        return {"delivered": deliveries}

    server.run()
