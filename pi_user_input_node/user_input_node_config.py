from enum import IntEnum
from dataclasses import dataclass

# Typedefs for configuration-related stuff
class PiButtonType(IntEnum):
    BUTTON_TYPE_PERCENTAGE = 0
    BUTTON_TYPE_SWITCH = 1

@dataclass(frozen = True)
class PiButtonInfo:
    pin: int
    description: str
    button_type: PiButtonType
    
@dataclass(frozen=True)
class PiEncoderInfo:
    a: int
    b: int
    
@dataclass(frozen=True)
class PiRGBInfo:
    r: int
    g: int
    b: int
    
# Configuration
BUTTON_DEFAULT_DEBOUNCE_DURATION_SEC = 0.1
BUTTON_ARRAY_CONFIGURATION: tuple[PiButtonInfo] = (
    PiButtonInfo(12, "0", PiButtonType.BUTTON_TYPE_PERCENTAGE),
    PiButtonInfo(20, "25", PiButtonType.BUTTON_TYPE_PERCENTAGE),
    PiButtonInfo(23, "50", PiButtonType.BUTTON_TYPE_PERCENTAGE),
    PiButtonInfo(16, "75", PiButtonType.BUTTON_TYPE_PERCENTAGE),
    PiButtonInfo(21, "100", PiButtonType.BUTTON_TYPE_PERCENTAGE),
    PiButtonInfo(18, "START/STOP", PiButtonType.BUTTON_TYPE_SWITCH)
)

ENCODER_CONFIGURATION: PiEncoderInfo = PiEncoderInfo(
    a = 17,
    b = 27
)

DISPLAY_IDLE_OUTPUT = ""
SEVEN_SEGMENT_LED_PINS = (
    2, # A
    3, # B
    5, # C
    6, # D
    13, # E
    19, # F
    26  # G
    # ignore DP, not required for this application...
)

SEVEN_SEGMENT_MUX_PINS = [
    22, # LED 1
    10, # LED 2
    9   # LED 3
]

RGB_LED_PINS: PiRGBInfo = PiRGBInfo(
    r = 4,
    g = 24,
    b = 25
)

# ROS configuration
USER_INPUT_NODE_NAME = "user_input_node"
USER_INPUT_TOPIC_NAME = "user_input_node/fatigue_info"
USER_INPUT_TOPIC_PUBLISH_RATE = 0.01 # 10 Hz
USER_INPUT_QOS_PROFILE = 0