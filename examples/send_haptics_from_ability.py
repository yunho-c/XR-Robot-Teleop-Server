"""
Send live haptic feedback data derived from Ability Hand FSR readings over a
server-initiated WebRTC data channel.

This example mirrors `send_haptics.py` but samples the 30 touch sensors
published by the Ability Hand API and condenses them into per-finger intensity
values for the haptics data channel.
"""

import argparse  # noqa: I001
import asyncio
import json
import logging
import time
import uuid
from contextlib import asynccontextmanager, suppress

from ah_wrapper.ah_serial_client import AHSerialClient  # type: ignore

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.streaming import WebRTCServer

HAPTIC_CHANNEL_LABEL = "haptics"
DEFAULT_INTERVAL = 0.05  # seconds between haptic payloads
DEFAULT_FSR_SCALE = 8.0  # value mapped to intensity 1.0 (touch plots cap around 8)

logger = logging.getLogger(__name__)


class AppState:
    """Minimal per-peer state with a peer identifier."""

    def __init__(self):
        self.peer_id = str(uuid.uuid4())

    def __repr__(self):
        return f"<AppState peer_id={self.peer_id}>"


class AbilityHandReader:
    """Owns the Ability Hand serial client and exposes latest FSR readings."""

    def __init__(
        self,
        port: str | None,
        baud_rate: int | None,
        reply_mode: int,
        rate_hz: int,
        simulated: bool,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.reply_mode = reply_mode
        self.rate_hz = rate_hz
        self.simulated = simulated
        self.client: AHSerialClient | None = None

    def start(self):
        logger.info(
            "Connecting to Ability Hand (port=%s, baud=%s, reply_mode=%s, rate_hz=%s, simulated=%s)",
            self.port,
            self.baud_rate,
            self.reply_mode,
            self.rate_hz,
            self.simulated,
        )
        self.client = AHSerialClient(
            port=self.port,
            baud_rate=self.baud_rate,
            reply_mode=self.reply_mode,
            rate_hz=self.rate_hz,
            simulated=self.simulated,
        )

    def close(self):
        if self.client:
            logger.info("Closing Ability Hand connection")
            self.client.close()
            self.client = None

    def latest_fsr(self) -> list[float] | None:
        if not self.client:
            return None
        fsr_values = self.client.hand.get_fsr()
        if fsr_values is None:
            return None
        return list(fsr_values)


def on_haptics_message(message: bytes | str, state: AppState, channel=None):
    logger.info("Received message on haptics channel from %s: %s", state.peer_id, message)


def fsr_tip_to_haptic_intensities(fsr_values: list[float], scale: float) -> dict[str, float]:
    """Convert tip FSR readings (sensor indices 1, 7, 13, 19, 25) to per-finger intensities."""

    finger_names = ["index", "middle", "ring", "little", "thumb"]
    tip_indices = [1 + 6 * i for i in range(len(finger_names))]

    intensities = {}
    for name, idx in zip(finger_names, tip_indices):
        value = fsr_values[idx] if idx < len(fsr_values) else 0.0
        if scale and scale > 0:
            value = min(max(value / scale, 0.0), 1.0)
        intensities[name] = round(value, 3)

    return intensities


async def periodic_haptics(
    server: WebRTCServer,
    reader: AbilityHandReader,
    interval: float,
    fsr_scale: float,
    include_raw_fsr: bool,
):
    """Poll Ability Hand FSR data and push haptics payloads to peers."""

    warned_missing = False
    while True:
        fsr_values = reader.latest_fsr()
        if fsr_values is None:
            if not warned_missing:
                logger.info("Waiting for FSR data from Ability Hand...")
                warned_missing = True
            await asyncio.sleep(interval)
            continue

        warned_missing = False
        payload = {
            "type": "haptics",
            "timestamp": time.time(),
            "intensity": fsr_tip_to_haptic_intensities(fsr_values, fsr_scale),
        }
        if include_raw_fsr:
            payload["raw_fsr"] = fsr_values

        deliveries = server.send_to_datachannel(HAPTIC_CHANNEL_LABEL, json.dumps(payload))
        if deliveries:
            logger.debug("Sent haptics payload to %d peer(s): %s", deliveries, payload)

        await asyncio.sleep(interval)


def main():
    parser = argparse.ArgumentParser(
        description="Send Ability Hand FSR-based haptics over WebRTC datachannel."
    )
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
    parser.add_argument(
        "--fsr-scale",
        type=float,
        default=DEFAULT_FSR_SCALE,
        help="FSR value that maps to intensity 1.0 (values are clamped)",
    )
    parser.add_argument("--port", type=str, default=None, help="Serial port for Ability Hand")
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=None,
        help="Serial baud rate (falls back to API default if omitted)",
    )
    parser.add_argument(
        "--reply-mode",
        type=int,
        default=0,
        help="Ability Hand reply mode (0/1 include touch sensors)",
    )
    parser.add_argument(
        "--rate-hz",
        type=int,
        default=500,
        help="Write/read rate passed to AHSerialClient",
    )
    parser.add_argument(
        "--simulated",
        action="store_true",
        help="Use simulated serial connection from the Ability Hand SDK",
    )
    parser.add_argument(
        "--include-raw-fsr",
        action="store_true",
        help="Include all 30 raw FSR readings in each payload for debugging",
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

    hand_reader = AbilityHandReader(
        port=args.port,
        baud_rate=args.baud_rate,
        reply_mode=args.reply_mode,
        rate_hz=args.rate_hz,
        simulated=args.simulated,
    )

    @asynccontextmanager
    async def haptics_lifespan(app):
        task = None
        try:
            hand_reader.start()
        except Exception:
            logger.exception("Failed to initialize Ability Hand client")
            raise

        if args.interval and args.interval > 0:
            task = asyncio.create_task(
                periodic_haptics(
                    server=server,
                    reader=hand_reader,
                    interval=args.interval,
                    fsr_scale=args.fsr_scale,
                    include_raw_fsr=args.include_raw_fsr,
                )
            )
            app.state.haptics_task = task

        try:
            yield
        finally:
            if task:
                task.cancel()
                with suppress(asyncio.CancelledError):
                    await task
            hand_reader.close()

    server.add_lifespan_context(haptics_lifespan)
    server.run()


if __name__ == "__main__":
    main()
