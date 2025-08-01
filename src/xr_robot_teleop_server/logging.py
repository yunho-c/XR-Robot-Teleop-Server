"""Logging configuration for the xr_robot_teleop_server library."""

import sys

from loguru import logger


def configure_logging(level="INFO", sink=sys.stderr, format="{level: <9} {message}"):
    """
    Configures the Loguru logger for the library.

    This function removes the default Loguru handler and adds a new one with
    the specified parameters. It provides a simple, one-line way for library
    users to set up logging.

    Args:
        level (str, optional): The minimum logging level to output.
            Defaults to "INFO".
        sink (file-like object, optional): The destination for logs.
            Defaults to `sys.stderr`.
        format (str, optional): The Loguru format string for the log messages.
            Defaults to "{level: <9} {message}".
    """
    logger.remove()
    logger.add(sink, format=format, level=level)
