import os
from tesseract_robotics.tesseract_common import GeneralResourceLocator
import tesseract_robotics  # noqa: F401 - triggers env var setup


def TesseractSupportResourceLocator():
    """Create a resource locator for tesseract_support package.

    Uses GeneralResourceLocator which resolves package:// URLs using
    the TESSERACT_RESOURCE_PATH environment variable.

    The environment variable is automatically set by tesseract_robotics
    when using bundled data. Can be overridden by setting env var manually.
    """
    # Legacy fallback for custom TESSERACT_SUPPORT_DIR without TESSERACT_RESOURCE_PATH
    if "TESSERACT_RESOURCE_PATH" not in os.environ and "TESSERACT_SUPPORT_DIR" in os.environ:
        support_dir = os.environ["TESSERACT_SUPPORT_DIR"]
        parent_dir = os.path.dirname(support_dir)
        os.environ["TESSERACT_RESOURCE_PATH"] = parent_dir

    return GeneralResourceLocator()
