"""A Python project that streams 360 panoramic videos to XR headsets."""

from loguru import logger

from . import __about__
from .logging import configure_logging
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

__all__ = [
    "__version__",
    "configure_logging",
    "VideoSource",
    "VideoTransform",
    "logger",
    "HandBoneId",
    "OVRHandBoneId",
    "BodyBoneId",
    "FullBodyBoneId",
    "SkeletonType",
    "get_bone_label",
    "get_skeleton_connections",
    "BODY_SKELETON_CONNECTIONS",
    "HAND_SKELETON_CONNECTIONS",
    "OVR_HAND_SKELETON_CONNECTIONS",
    "FULL_BODY_SKELETON_CONNECTIONS",
]
__version__ = __about__.version

# Set up default logging for the library.
# Users can easily override this by calling `configure_logging()` with their
# preferred settings.
configure_logging()
