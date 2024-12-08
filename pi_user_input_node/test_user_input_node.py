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

start_stop_state = False

def test_button_callback(button_description: str, encoder: RotaryEncoder, display: LEDMultiCharDisplay):
    # TODO: improve this
    if button_description == "0%":
        encoder.steps = 0
    elif button_description == "25%":
        encoder.steps = 25
    elif button_description == "50%":
        encoder.steps = 50
    elif button_description == "75%":
        encoder.steps = 75
    elif button_description == "100%":
        encoder.steps = 100
    else:
        # start/stop button
        global start_stop_state
        start_stop_state = not start_stop_state
        encoder.steps = 0
        if start_stop_state:
            pass
        else:
            display.value = ""
            return
    
    display.value = str(encoder.steps)
    
def test_encoder_callback(encoder: RotaryEncoder, display: LEDMultiCharDisplay):
    display.value = str(encoder.steps)

def main():
    button_objs: list[Button] = []
        
    encoder_obj = RotaryEncoder(
        a = ENCODER_CONFIGURATION.a,
        b = ENCODER_CONFIGURATION.b,
        max_steps = 100 # clamp to 100
    )
    
    seven_segment_obj = LEDMultiCharDisplay(
        LEDCharDisplay(*SEVEN_SEGMENT_LED_PINS, active_high = True),
        *SEVEN_SEGMENT_MUX_PINS,
        active_high = True
    )

    encoder_obj.when_rotated = partial(test_encoder_callback, encoder_obj, seven_segment_obj)
        
    for button in BUTTON_ARRAY_PINS:
        # Initialize configured buttons
        button_objs.append(
            Button(
                pin = button.pin,
                pull_up = True,
                bounce_time = 0.1 # 100ms debounce
            )
        )
        
        button_objs[-1].when_activated = partial(test_button_callback, button.description, encoder_obj, seven_segment_obj)
        
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
    seven_segment_obj.value = ""

    # Loop
    while True: 
        time.sleep(1.0)

