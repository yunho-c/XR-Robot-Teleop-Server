"""A Python project that streams 360 panoramic videos to XR headsets."""

import sys

from loguru import logger

from . import __about__
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

# Set up logging for the library.
#
# This library uses Loguru for logging. By default, Loguru is configured to output
# logs to stderr. Library users can customize this behavior by using the `logger`
# object.
#
# For example, to redirect logs to a file:
#
# from loguru import logger
# logger.add("my_app.log")
#
# To disable logging from the library, the default handler can be removed:
#
# logger.remove()
#
# For more advanced configuration, please refer to the Loguru documentation.
logger.remove()
logger.add(sys.stderr, format="{level: <9} {message}", level="INFO")
