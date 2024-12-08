"""Test script that checks basic functionality for all peripherals being used for the user input node"""
import time
from gpiozero.input_devices import (
    Button, RotaryEncoder
)

from gpiozero.output_devices import RGBLED

from gpiozero.boards import (
    LEDCharDisplay, LEDMultiCharDisplay
)

from collections import deque
from itertools import cycle

from functools import partial

from user_input_node_config import *

def test_button_callback(button_description: str):
    print(f"button {button_description} pressed")
    
def test_encoder_callback(encoder: RotaryEncoder):
    print(f"encoder value: {encoder.steps}")
    encoder.steps = 0 # reset steps
    # set seven segment...
 

def main():
    button_objs: list[Button] = []
    for button in BUTTON_ARRAY_PINS:
        # Initialize configured buttons
        button_objs.append(
            Button(
                pin = button.pin,
                pull_up = True,
                bounce_time = 0.1 # 100ms debounce
            )
        )
        
        button_objs[-1].when_activated = partial(test_button_callback, button.description)
        
    encoder_obj = RotaryEncoder(
        a = ENCODER_CONFIGURATION.a,
        b = ENCODER_CONFIGURATION.b,
        max_steps = 100 # clamp to 100
    )
    
    encoder_obj.when_rotated = partial(test_encoder_callback, encoder_obj)
    
    seven_segment_obj = LEDMultiCharDisplay(
        LEDCharDisplay(*SEVEN_SEGMENT_LED_PINS, active_high = True),
        *SEVEN_SEGMENT_MUX_PINS,
        active_high = True
    )
    
    rgba_led_obj = RGBLED(
        red = RGB_LED_PINS.r,
        green = RGB_LED_PINS.g,
        blue = RGB_LED_PINS.b,
        active_high = True,
        pwm = True
    )
    
    print("Setup complete")

    # Test outputs
    rgba_led_obj.color = (1, 0, 0) # red
    seven_segment_obj.value = "123"

    # Loop
    while True: 
        time.sleep(1.0)

