from dataclasses import dataclass

@dataclass(frozen = True)
class PiButtonInfo:
    pin: int
    description: str
    
    
@dataclass(frozen=True)
class PiEncoderInfo:
    a: int
    b: int
    
@dataclass(frozen=True)
class PiRGBInfo:
    r: int
    g: int
    b: int
    
BUTTON_ARRAY_PINS: tuple[PiButtonInfo] = (
    PiButtonInfo(12, "0%"),
    PiButtonInfo(20, "25%"),
    PiButtonInfo(23, "50%"),
    PiButtonInfo(16, "75%"),
    PiButtonInfo(21, "100%"),
    PiButtonInfo(18, "START/STOP")
)

ENCODER_CONFIGURATION: PiEncoderInfo = PiEncoderInfo(
    a = 17,
    b = 27
)

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
    r = 25,
    g = 8,
    b = 7
)

