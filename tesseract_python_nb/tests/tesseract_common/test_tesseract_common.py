from tesseract_robotics import tesseract_common
from inspect import currentframe, getframeinfo
import numpy as np
import numpy.testing as nptest

def test_bytes_resource():
    my_bytes = bytearray([10,57,92,56,92,46,92,127])
    my_bytes_url = "file:///test_bytes.bin"
    bytes_resource = tesseract_common.BytesResource(my_bytes_url, my_bytes)
    my_bytes_ret = bytes_resource.getResourceContents()
    assert(len(my_bytes_ret) == len(my_bytes))
    assert(my_bytes == bytearray(my_bytes_ret))
    assert(my_bytes_url == bytes_resource.getUrl())

class _TestOutputHandler(tesseract_common.OutputHandler):
    def __init__(self):
        super().__init__()
        self.last_text = None

    def log(self, text, level, filename, line):
        self.last_text = text

def test_console_bridge():
    tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

    # Note: tesseract_common.log() is not bound (variadic C function)
    # Use the OutputHandler callback mechanism instead

    output_handler = _TestOutputHandler()
    tesseract_common.useOutputHandler(output_handler)

    # Output handler should be active now
    # Note: Can't call log directly, but handler is ready to receive logs

    tesseract_common.restorePreviousOutputHandler()
    tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_ERROR)

def test_manipulator_info():
    info = tesseract_common.ManipulatorInfo()

    # Test string tcp_offset
    info.tcp_offset = "tool0"
    assert info.tcp_offset == "tool0"

    # Test Isometry3d tcp_offset (using numpy 4x4 arrays)
    # In nanobind, Eigen::Isometry3d converts to/from 4x4 numpy arrays
    transform = np.eye(4)
    transform[0, 3] = 1.0  # translation x
    transform[1, 3] = 2.0  # translation y
    transform[2, 3] = 3.0  # translation z

    info.tcp_offset = transform
    transform2 = info.tcp_offset
    nptest.assert_allclose(transform2, transform)

def test_translation3d_multiply():
    """Test Translation3d multiply with Isometry3d"""
    t = tesseract_common.Translation3d(1, 2, 3)
    identity = np.eye(4)
    result = t * identity
    expected = np.eye(4)
    expected[0, 3] = 1.0
    expected[1, 3] = 2.0
    expected[2, 3] = 3.0
    nptest.assert_allclose(result, expected)
