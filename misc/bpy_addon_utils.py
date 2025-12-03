import bpy
import addon_utils
import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

def install_addon(zip_path: str, module_name: Optional[str] = None) -> bool:
    """
    Install and enable a Blender addon from a zip file.

    Args:
        zip_path: Full path to the addon .zip file.
        module_name: The internal name of the addon (e.g., folder name inside the zip). 
                     If None, attempts to guess from the zip filename.

    Returns:
        True if the addon is active (installed & enabled), False otherwise.
    """
    addon_file = Path(zip_path)
    
    # Guess module name from file stem if not provided
    if not module_name:
        module_name = addon_file.stem

    # Check if already enabled
    is_default, is_loaded = addon_utils.check(module_name)
    if is_loaded:
        logger.info(f"✓ {module_name} is already enabled.")
        return True

    # Check if zip file exists
    if not addon_file.exists():
        logger.warning(f"Addon file not found at: {addon_file}")
        return False

    try:
        # Install addon
        bpy.ops.preferences.addon_install(filepath=str(addon_file))
        logger.info(f"✓ Installed {module_name} from {addon_file.name}")

        # Refresh and Enable
        addon_utils.modules_refresh()
        bpy.ops.preferences.addon_enable(module=module_name)
        
        # Save preferences so it stays enabled
        bpy.ops.wm.save_userpref()
        
        logger.info(f"✓ Enabled {module_name} successfully.")
        return True

    except Exception as e:
        logger.error(f"Failed to install/enable {module_name}: {e}")
        return False
