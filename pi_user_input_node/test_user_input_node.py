"""Test script that checks basic functionality for all peripherals being used for the user input node"""
import time
from gpiozero.input_devices import (
    Button, RotaryEncoder
)

from gpiozero.output_devices import RGBLED

from gpiozero.boards import (
    LEDCharDisplay, LEDMultiCharDisplay
)

from functools import partial

from user_input_node_config import *

def test_button_callback(button_description: str):
    print(f"button {button_description} pressed")
    
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
        bounce_time = 0.1 # 100 ms
    )
    
    encoder_obj.when_activated = test_encoder_callback
    
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

    # Test outputs
    rgba_led_obj.color = (1, 1, 0) # yellow
    seven_segment_obj.source = "123"
    
    # Loop
    while True: 
        time.sleep(0.1)

