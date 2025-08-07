"""A Python project that streams 360 panoramic videos to XR headsets."""

from loguru import logger

from . import __about__
from .logging import configure_logging
from .schemas.body_pose import (
    Bone,
    deserialize_pose_data,
)
from .schemas.openxr_skeletons import (
    BODY_SKELETON_CONNECTIONS,
    FULL_BODY_SKELETON_CONNECTIONS,
    HAND_SKELETON_CONNECTIONS,
    OVR_HAND_SKELETON_CONNECTIONS,
    BodyBoneId,
    FullBodyBoneId,
    HandBoneId,
    OVRHandBoneId,
    SkeletonType,
    get_bone_label,
    get_skeleton_connections,
)
from .sources.base import VideoSource
from .transforms.base import VideoTransform
from .utils.coordinate import convert_unity_to_right_handed_z_up
from .utils.frame import (
    get_body_centric_coordinates,
    get_body_frame,
    get_hand_centric_coordinates,
    get_hand_frame,
)

__all__ = [
    "__version__",
    "logger",
    "configure_logging",
    "Bone",
    "deserialize_pose_data",
    "BODY_SKELETON_CONNECTIONS",
    "FULL_BODY_SKELETON_CONNECTIONS",
    "HAND_SKELETON_CONNECTIONS",
    "OVR_HAND_SKELETON_CONNECTIONS",
    "BodyBoneId",
    "FullBodyBoneId",
    "HandBoneId",
    "OVRHandBoneId",
    "SkeletonType",
    "get_bone_label",
    "get_skeleton_connections",
    "VideoSource",
    "VideoTransform",
    "convert_unity_to_right_handed_z_up",
    "get_body_centric_coordinates",
    "get_body_frame",
    "get_hand_centric_coordinates",
    "get_hand_frame",
]
__version__ = __about__.version

# Set up default logging for the library.
# Users can easily override this by calling `configure_logging()` with their
# preferred settings.
configure_logging()
