"""
Send fake haptic feedback data over a server-initiated WebRTC data channel.

Based on `examples/send_data.py`, but simplified to emit periodic haptic
intensities for each finger and the palm for both left and right hands.
"""

import argparse  # noqa: I001
import asyncio
import json
import logging
import time
import uuid
from contextlib import asynccontextmanager, suppress
from functools import partial
from typing import Any

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.streaming import WebRTCServer

HAPTIC_CHANNEL_LABEL = "haptics"
DEFAULT_INTERVAL = 1.0
HAPTIC_KEYS = ["thumb", "index", "middle", "ring", "little", "palm"]
STEP_SIZE = 0.2
RAMP_CYCLE_SECONDS = 11.0  # 0->1 (6 steps) then 1->0 (5 steps) = 11 steps total

logger = logging.getLogger(__name__)
_start_time = None


class AppState:
    """Minimal per-peer state with a peer identifier."""

    def __init__(self):
        self.peer_id = str(uuid.uuid4())

    def __repr__(self):
        return f"<AppState peer_id={self.peer_id}>"


def on_haptics_message(message: bytes | str, state: AppState, channel=None):
    logger.info("Received message on haptics channel from %s: %s", state.peer_id, message)


def make_fake_haptics():
    """Return a dict of fake haptic intensities ramping up and down from 0 to 1 in steps of 0.2."""
    global _start_time
    now = time.time()
    
    # Initialize start time on first call
    if _start_time is None:
        _start_time = now
    
    # Calculate which step we're on (0-10 for full cycle: 0->1->0)
    elapsed = now - _start_time
    step = int(elapsed) % int(RAMP_CYCLE_SECONDS)
    
    # Ramp up: steps 0-5 (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
    # Ramp down: steps 6-10 (0.8, 0.6, 0.4, 0.2, 0.0)
    if step <= 5:
        intensity = step * STEP_SIZE
    else:
        # Ramp down: step 6->0.8, 7->0.6, 8->0.4, 9->0.2, 10->0.0
        intensity = (10 - step) * STEP_SIZE
    
    intensity = round(intensity, 2)
    
    # Apply same intensity to all fingers for both hands
    left_intensities = {key: intensity for key in HAPTIC_KEYS}
    right_intensities = {key: intensity for key in HAPTIC_KEYS}
    
    return {
        "type": "haptics",
        "timestamp": now,
        "left": left_intensities,
        "right": right_intensities
    }


async def periodic_haptics(server: WebRTCServer, interval: float):
    """Periodically send fake haptic payloads."""
    while True:
        payload = json.dumps(make_fake_haptics())
        sends = server.send_to_datachannel(HAPTIC_CHANNEL_LABEL, payload)
        if sends:
            logger.debug("Sent haptics payload to %d peer(s): %s", sends, payload)
        await asyncio.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description="Send fake haptics over WebRTC datachannel.")
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL,
        help="Seconds between haptic payloads.",
    )
    args = parser.parse_args()

    configure_logging(level=args.log_level)

    data_handlers = {
        HAPTIC_CHANNEL_LABEL: on_haptics_message,
    }

    server = WebRTCServer(
        datachannel_handlers=data_handlers,
        state_factory=AppState,
        server_data_channels=[HAPTIC_CHANNEL_LABEL],
    )

    @asynccontextmanager
    async def haptics_lifespan(app):
        task = None
        if args.interval and args.interval > 0:
            task = asyncio.create_task(periodic_haptics(server, args.interval))
            app.state.haptics_task = task
        try:
            yield
        finally:
            if task:
                task.cancel()
                with suppress(asyncio.CancelledError):
                    await task

    server.add_lifespan_context(haptics_lifespan)
    server.run()


if __name__ == "__main__":
    main()
