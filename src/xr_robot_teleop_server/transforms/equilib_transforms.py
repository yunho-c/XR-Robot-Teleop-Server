import numpy as np
from equilib import Equi2Pers

from .base import VideoTransform


class EquilibEqui2Pers(VideoTransform):
    """
    A transform that projects a frame from an equirectangular (360°) source
    to a standard perspective view.

    Args:
        output_width (int): The width of the output perspective video.
        output_height (int): The height of the output perspective video.
    """

    def __init__(self, output_width: int, output_height: int, fov_x: float) -> None:
        """
        Initializes the EquilibReprojection.

        Args:
            output_width (int): The width of the output perspective video.
            output_height (int): The height of the output perspective video.
            fov_x (float): The horizontal field of view in degrees.
        """
        self._output_width = output_width
        self._output_height = output_height
        self._equi2pers = Equi2Pers(width=output_width, height=output_height, fov_x=fov_x)

    @property
    def output_width(self) -> int:
        return self._output_width

    @property
    def output_height(self) -> int:
        return self._output_height

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        """
        Preprocesses image

        (from equilib/scripts/equi2pers_numpy.py)
        """
        assert len(img.shape) == 3, "input must be dim=3"
        assert img.shape[-1] == 3, "input must be HWC"
        img = np.transpose(img, (2, 0, 1))  # ?!
        return img

    def postprocess(self, img: np.ndarray) -> np.ndarray:
        return np.transpose(img, (1, 2, 0))  # ?!

    def transform(self, frame: np.ndarray, rot: dict[str, float]) -> np.ndarray:
        """
        Re-projects an equirectangular frame to a perspective frame.

        Args:
            frame (np.ndarray): The equirectangular frame (H, W, C).
            rot (dict[str, float]): A dictionary with rotation parameters:
                - "pitch": rotation in degrees around the x-axis.
                - "yaw": rotation in degrees around the y-axis.
                - "roll": rotation in degrees around the z-axis.

        Returns:
            np.ndarray: The perspective frame.
        """
        # NOTE: `_equi2pers()` *silently hangs* when non-CHW images are provided.

        equi = self.preprocess(frame)
        pers = self._equi2pers(equi=equi, rots=rot)
        perspective_frame = self.postprocess(pers)
        return perspective_frame
