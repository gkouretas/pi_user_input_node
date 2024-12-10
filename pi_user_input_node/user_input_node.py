import threading

import rclpy
from rclpy.node import Node
from idl_definitions.msg import UserInputMsg
from user_input_node_config import *

from gpiozero.input_devices import (
    Button, RotaryEncoder
)

from gpiozero.output_devices import RGBLED

from gpiozero.boards import (
    LEDCharDisplay, LEDMultiCharDisplay
)

from functools import partial

class UserInputNode:
    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.RLock()

        self._is_active = False

        # TODO: handle negative counts
        self._encoder = RotaryEncoder(
            ENCODER_CONFIGURATION.a,
            ENCODER_CONFIGURATION.b,
            max_steps = 100 # Cap value at 100
        )

        self._encoder.when_rotated = self.encoder_callback

        self._buttons: list[Button] = []

        for button_info in BUTTON_ARRAY_CONFIGURATION:
            _button = Button(
                pin = button_info.pin,
                pull_up = True,
                bounce_time = BUTTON_DEFAULT_DEBOUNCE_DURATION_SEC
            ) 

            _button.when_activated = partial(self.button_callback, button_info)

            self._buttons.append(_button)
        
        self._display = LEDMultiCharDisplay(
            LEDCharDisplay(*SEVEN_SEGMENT_LED_PINS, active_high = True),
            *SEVEN_SEGMENT_MUX_PINS,
            active_high = True
        )

        self._display.value = DISPLAY_IDLE_OUTPUT

        self._publisher = self._node.create_publisher(
            msg_type = UserInputMsg,
            topic = USER_INPUT_TOPIC_NAME,
            qos_profile = USER_INPUT_QOS_PROFILE
        )

        self._pub_timer = self._node.create_timer(
            USER_INPUT_TOPIC_PUBLISH_RATE, 
            self.publish_info_callback
        )

        self._node.get_logger().info("Initialized user input class")
            
    @property
    def fatigue_percentage(self):
        with self._lock:
            # Clamp to (0, max_steps)
            # RotaryEncoder class handles max output, but can go negative.
            # We handle this in the callback, but also here in case there is
            # an edge-case where this method is called after the value changes
            # and before the event is triggered (is this possible? who knows, but
            # may as well guarantee safety by doing it like this)
            _percent = max(0, self._encoder.steps)
        
        return _percent

    def button_callback(self, button: PiButtonInfo):
        self._node.get_logger().info(f"Button {button.description} press triggered")

        with self._lock:
            if button.button_type == PiButtonType.BUTTON_TYPE_PERCENTAGE:
                self._encoder.steps = int(button.description)
            else:
                self._is_active = not self._is_active

                # Always reset the encoder step count on a start/stop change
                self._encoder.steps = 0

            if self._is_active:
                # Start displaying the encoder steps again
                self._display.value = str(self._encoder.steps)
            else:
                # Show idle display
                self._display.value = DISPLAY_IDLE_OUTPUT

    def encoder_callback(self):
        self._node.get_logger().info("Encoder rotation event triggered")

        with self._lock:
            # Clamp value if it goes negative
            self._encoder.steps = max(0, self._encoder.steps)

            # Update the display whenever the encoder counts update
            self._display.value = str(self._encoder.steps)

    def publish_info_callback(self):
        msg = UserInputMsg()

        with self._lock:
            msg.is_active = self._is_active
            msg.fatigue_percentage = self.fatigue_percentage if self._is_active else 0

        self._publisher.publish(msg)

def main():
    rclpy.init()
    
    node = Node(USER_INPUT_NODE_NAME)
    input_device_node = UserInputNode(node)

    rclpy.spin(node)

if __name__ == "__main__":
    main()