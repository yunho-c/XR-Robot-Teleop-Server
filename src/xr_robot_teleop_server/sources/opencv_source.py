from pathlib import Path

import cv2
import numpy as np

from .base import VideoSource


class OpenCVFileSource(VideoSource):
    """
    A video source that reads from a video file using OpenCV.

    NOTE: OpenCV reads frames in BGR format by default.
          To grab RGB frames, pass `use_rgb`=True.

    Args:
        filepath (str): The path to the video file.
        use_rgb (bool): Whether to convert the frames to RGB. Defaults to True.
    """

    def __init__(self, filepath: str, use_rgb=True):
        self.filepath = Path(filepath)
        if not self.filepath.is_file():
            raise FileNotFoundError(f"Video file not found at: {filepath}")

        self.cap = cv2.VideoCapture(str(self.filepath))
        if not self.cap.isOpened():
            raise ValueError(f"Failed to open video file with OpenCV: {filepath}")
        self.use_rgb = use_rgb

        self._width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._fps = self.cap.get(cv2.CAP_PROP_FPS)

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
        """Reads the next frame. Raises StopIteration when the video ends."""
        ret, frame = self.cap.read()
        if not ret:
            self.release()
            raise StopIteration
        return frame if not self.use_rgb else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def release(self):
        """Releases the video capture object."""
        if self.cap.isOpened():
            self.cap.release()
