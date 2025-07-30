"""Command-line interface for xr-robot-teleop-server."""

import sys

from . import logger


def main() -> int:
    """The main function of the command-line interface."""
    logger.info("Starting xr-robot-teleop-server...")
    # Your application logic will go here
    return 0


if __name__ == "__main__":
    sys.exit(main())
