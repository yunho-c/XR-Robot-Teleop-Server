import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

from .. import logger
from .base import VideoSource

_available_hw_accels = None


def get_ffmpeg_hw_accels():
    """
    Checks for available hardware acceleration methods in FFmpeg.
    Caches the result to avoid repeated calls.
    Returns:
        A set of available hardware acceleration method names.
    """
    global _available_hw_accels
    if _available_hw_accels is not None:
        return _available_hw_accels

    try:
        result = subprocess.run(
            ["ffmpeg", "-hwaccels"], capture_output=True, text=True, check=True, encoding="utf-8"
        )
        lines = result.stdout.strip().split("\n")
        # The first line is "Hardware acceleration methods:"
        accels = {line.strip() for line in lines[1:] if line.strip()}
        _available_hw_accels = accels
        logger.info(f"Available FFmpeg HW accels: {_available_hw_accels}")
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        logger.warning(f"Could not get FFmpeg HW accels. Is ffmpeg in PATH? Error: {e}")
        _available_hw_accels = set()
    return _available_hw_accels


def get_best_hw_accel() -> str | None:
    """
    Determines the best available hardware acceleration method for the current platform.
    """
    available = get_ffmpeg_hw_accels()
    if not available:
        return None

    # Preference order by platform
    if sys.platform == "darwin":  # macOS
        preferred_order = ["videotoolbox"]
    elif sys.platform == "win32":  # Windows
        preferred_order = ["d3d11va", "nvdec", "qsv", "cuda"]
    else:  # Linux and other Unix-likes
        preferred_order = ["vaapi", "nvdec", "vdpau", "qsv", "cuda"]

    for method in preferred_order:
        if method in available:
            return method

    return None


class FFmpegFileSource(VideoSource):
    """
    A video source that reads from a file using a direct FFmpeg subprocess pipe.

    This can be faster for some high-resolution or high-framerate videos
    as it avoids some of the overhead of OpenCV's wrapper.

    It can use hardware acceleration if `ffmpeg` was compiled with support for it,
    which can significantly reduce CPU usage for high-resolution videos (e.g., 4K).

    Requires `ffmpeg` to be installed and accessible in the system's PATH.

    Args:
        filepath (str): The path to the video file.
        hw_accel_enabled (bool): If True, attempts to use the best available
            hardware acceleration method for the current platform. Defaults to True.
    """

    def __init__(self, filepath: str, hw_accel_enabled: bool = True):
        self.filepath = Path(filepath)  # NOTE: avoid using `filepath` directly (?)
        if not self.filepath.is_file():
            raise FileNotFoundError(f"Video file not found at: {str(self.filepath)}")

        # Use OpenCV once to get reliable metadata.
        # This is simpler than parsing ffprobe's output.
        cap = cv2.VideoCapture(str(self.filepath))
        if cap.isOpened():
            logger.info(f"Successfully inspected video file: {str(self.filepath)}")
        else:
            raise ValueError(f"Failed to inspect video file: {str(self.filepath)}")

        self._width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._fps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()

        # Calculate the size of a single frame in bytes (Width x Height x 3 channels for RGB)
        self.frame_size = self._width * self._height * 3

        # Construct the FFmpeg command
        command = [
            "ffmpeg",
        ]

        if hw_accel_enabled:
            hw_accel_method = get_best_hw_accel()
            if hw_accel_method:
                logger.info(f"Using '{hw_accel_method}' for hardware acceleration.")
                command.extend(["-hwaccel", hw_accel_method])
            else:
                logger.warning(
                    "Hardware acceleration was requested, but no suitable method "
                    "was found. Falling back to software decoding."
                )

        # fmt: off
        command.extend([
            '-i', str(self.filepath),  # Input file
            '-loglevel', 'error',      # Suppress verbose output
            '-f', 'rawvideo',          # Output format: raw video frames
            '-pix_fmt', 'rgb24',       # Pixel format: 24-bit RGB
            '-'                        # Output to stdout
        ])
        self.ffmpeg_command = command
        # fmt: on

        # Start the FFmpeg subprocess
        self.process = subprocess.Popen(
            self.ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def fps(self) -> float:
        return self._fps

    def __iter__(self):
        return self

    def __next__(self) -> np.ndarray:
        """Reads a raw frame from the stdout pipe and reshapes it."""
        # Read the exact number of bytes for one frame
        raw_frame = self.process.stdout.read(self.frame_size)

        if len(raw_frame) != self.frame_size:
            # End of stream or error
            self.release()
            raise StopIteration

        # Reshape the raw byte buffer into a NumPy array (H, W, C)
        frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3))
        return frame

    def release(self):
        """Terminates the FFmpeg subprocess and closes pipes."""
        if hasattr(self, "process") and self.process.poll() is None:
            self.process.terminate()
            # Wait for the process to terminate to avoid zombie processes
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
