import os
from tesseract_robotics.tesseract_common import GeneralResourceLocator

def TesseractSupportResourceLocator():
    """Create a resource locator for tesseract_support package.

    Uses GeneralResourceLocator which resolves package:// URLs using
    the TESSERACT_RESOURCE_PATH environment variable.

    Make sure TESSERACT_RESOURCE_PATH is set to the directory containing
    tesseract_support (e.g., /path/to/ws/src/tesseract).
    """
    # Set TESSERACT_RESOURCE_PATH if not already set and TESSERACT_SUPPORT_DIR is available
    if "TESSERACT_RESOURCE_PATH" not in os.environ and "TESSERACT_SUPPORT_DIR" in os.environ:
        # TESSERACT_SUPPORT_DIR points to .../tesseract_support
        # TESSERACT_RESOURCE_PATH should point to parent (.../tesseract)
        support_dir = os.environ["TESSERACT_SUPPORT_DIR"]
        parent_dir = os.path.dirname(support_dir)
        os.environ["TESSERACT_RESOURCE_PATH"] = parent_dir

    return GeneralResourceLocator()