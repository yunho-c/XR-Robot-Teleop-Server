"""
Send fake haptic feedback data over a server-initiated WebRTC data channel.

Based on `examples/send_data.py`, but simplified to emit periodic haptic
intensities for each finger and the palm.
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

logger = logging.getLogger(__name__)


class AppState:
    """Minimal per-peer state with a peer identifier."""

    def __init__(self):
        self.peer_id = str(uuid.uuid4())

    def __repr__(self):
        return f"<AppState peer_id={self.peer_id}>"


def on_haptics_message(message: bytes | str, state: AppState, channel=None):
    logger.info("Received message on haptics channel from %s: %s", state.peer_id, message)


def make_fake_haptics():
    """Return a dict of fake haptic intensities between 0 and 1."""
    now = time.time()
    base = (now % 1.0)  # simple changing value
    intensities = {key: round((base + idx * 0.1) % 1.0, 2) for idx, key in enumerate(HAPTIC_KEYS)}
    return {"type": "haptics", "timestamp": now, "intensity": intensities}


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
