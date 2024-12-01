import time
from gpiozero.input_devices import (
    Button, RotaryEncoder
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

def main():
    test_button.when_activated = test_button_callback
    test_encoder.when_activated = test_encoder_callback
    
    while True: 
        time.sleep(0.1)

