from tesseract_robotics import tesseract_common
from inspect import currentframe, getframeinfo

import numpy.testing as nptest

def test_status_code():
    # Test that status codes can be created

    status_code = tesseract_common.StatusCode(100, tesseract_common.GeneralStatusCategory())

    print(status_code)

def test_bytes_resource():

    my_bytes = bytearray([10,57,92,56,92,46,92,127])
    my_bytes_url = "file:///test_bytes.bin"
    bytes_resource = tesseract_common.BytesResource(my_bytes_url,my_bytes)
    my_bytes_ret = bytes_resource.getResourceContents()
    assert(len(my_bytes_ret) == len(my_bytes))
    assert(my_bytes == bytearray(my_bytes_ret))
    assert(my_bytes_url == bytes_resource.getUrl())

class _TestOutputHandler(tesseract_common.OutputHandler):
    def __init__(self):
        super().__init__()
        self.last_text=None

    def log(self, text, level, filename, line):
        self.last_text = text

def test_console_bridge():
    tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

    frameinfo = getframeinfo(currentframe())
    tesseract_common.log(frameinfo.filename,frameinfo.lineno,
        tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG, "This is a test message")

    output_handler = _TestOutputHandler()
    tesseract_common.useOutputHandler(output_handler)

    tesseract_common.log(frameinfo.filename,frameinfo.lineno,
        tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG, "This is a test message 2")
    tesseract_common.restorePreviousOutputHandler()

    assert output_handler.last_text == "This is a test message 2"

def test_manipulator_info():

    info = tesseract_common.ManipulatorInfo()
    info.tcp_offset="tool0"
    assert info.tcp_offset=="tool0"
    
    transform = tesseract_common.Isometry3d() * tesseract_common.Translation3d(1,2,3)
    info.tcp_offset = transform
    transform2=info.tcp_offset
    nptest.assert_allclose(transform2.matrix(), transform.matrix())