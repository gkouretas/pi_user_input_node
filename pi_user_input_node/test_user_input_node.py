"""Test script that checks basic functionality for all peripherals being used for the user input node"""
import time
from gpiozero.input_devices import (
    Button, RotaryEncoder
)

from gpiozero.output_devices import RGBLED

from gpiozero.boards import (
    LEDCharDisplay, LEDMultiCharDisplay
)

def test_button_callback():
    print("button pressed")
    
def test_encoder_callback():
    print(f"encoder value: {test_encoder.steps}")
    
test_button = Button(
    pin = 4,
    pull_up = True,
    bounce_time = 0.1 # 100 ms debounce
)

test_encoder = RotaryEncoder(
    a = 5,
    b = 6,
    bounce_time = 0.1, # 100 ms debounce
    max_steps = 20
)

test_rgba_led = RGBLED(
    red = 9,
    green = 10,
    blue = 11,
    active_high = True,
    pwm = True
)

test_rgba_led.color = (1, 1, 0)

_seven_segment_led_pins = [
    12, # A
    13, # B
    16, # C
    18, # D
    20, # E
    21, # F
    22  # G
    # ignore DP, not required for this application...
]

_seven_segment_mux_pins = [
    23, # 1 
    24, # 2
    25  # 3
]

test_seven_segment = LEDMultiCharDisplay(
    LEDCharDisplay(*_seven_segment_led_pins, active_high = True),
    *_seven_segment_mux_pins,
    active_high = True
)

def main():
    test_button.when_activated = test_button_callback
    test_encoder.when_activated = test_encoder_callback
    test_rgba_led.color = (1, 1, 0) # yellow
    test_seven_segment.source = "123"
    
    while True: 
        time.sleep(0.1)

