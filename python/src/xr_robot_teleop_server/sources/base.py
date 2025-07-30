import abc

import numpy as np


class VideoSource(abc.ABC):
    """
    Abstract base class for video sources.

    This defines a common interface for any class that provides video frames,
    allowing for interchangeable sources (e.g., file, camera, network stream).

    Args:
        abc (_type_): _description_
    """

    @abc.abstractmethod
    def __iter__(self):
        """Allows the class to be used as an interator."""
        return self

    @abc.abstractmethod
    def __next__(self) -> np.ndarray:
        """Returns the next frame as a NumPy array."""
        pass

    @abc.abstractmethod
    def release(self):
        """Releases the video source and cleans up resources."""
        pass

    @property
    @abc.abstractmethod
    def width(self) -> int:
        """Width of the video frames."""
        pass

    @property
    @abc.abstractmethod
    def height(self) -> int:
        """Height of the video frames."""
        pass

    @property
    @abc.abstractmethod
    def fps(self) -> float:
        """Frames per second of the video."""
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
