"""A Python project that streams 360 panoramic videos to XR headsets."""

import sys

from loguru import logger

from . import __about__
from .sources.base import VideoSource
from .transforms.base import VideoTransform

__all__ = ["__version__", "VideoSource", "VideoTransform", "logger"]
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
