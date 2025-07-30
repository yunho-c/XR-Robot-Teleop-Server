import abc

import numpy as np


class VideoTransform(abc.ABC):
    """
    Abstract base class for video transformations.

    This defines a common interface for any class that processes or transforms
    video frames, such as changing projections or applying effects. This allows
    for a modular pipeline where different transformations can be easily swapped.
    """

    @abc.abstractmethod
    def transform(self, frame: np.ndarray, **kwargs) -> np.ndarray:
        """
        Applies the transformation to a single video frame.

        Args:
            frame: The input video frame as a NumPy array.
            **kwargs: Dynamic parameters for the transformation, such as
                      camera orientation (e.g., yaw, pitch, roll) which may
                      change per frame based on client input.

        Returns:
            The transformed video frame as a NumPy array.
        """
        pass

    @property
    @abc.abstractmethod
    def output_width(self) -> int:
        """The width of the output frames after transformation."""
        pass

    @property
    @abc.abstractmethod
    def output_height(self) -> int:
        """The height of the output frames after transformation."""
        pass

    def __call__(self, frame: np.ndarray, **kwargs) -> np.ndarray:
        """Provides a convenient, callable interface for the transform."""
        return self.transform(frame, **kwargs)
