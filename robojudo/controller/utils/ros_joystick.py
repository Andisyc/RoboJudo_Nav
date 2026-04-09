import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from threading import Thread
import time

# A default mapping from standard Joy message to named axes and buttons
# This can be customized as needed.
DEFAULT_AXIS_MAP = {
    0: "lx",  # Left stick horizontal
    1: "ly",  # Left stick vertical
    2: "lt",  # Left trigger
    3: "rx",  # Right stick horizontal
    4: "ry",  # Right stick vertical
    5: "rt",  # Right trigger
}

DEFAULT_BUTTON_MAP = {
    0: "A",
    1: "B",
    2: "X",
    3: "Y",
    4: "LB",
    5: "RB",
    6: "Back",
    7: "Start",
    8: "Power",
    9: "L_Stick_Button",
    10: "R_Stick_Button",
}


class ROS2JoystickNode(Node):
    def __init__(self, state_queue, event_queue):
        super().__init__("ros2_joystick_subscriber")
        self.state_queue = state_queue
        self.event_queue = event_queue
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        self.get_logger().info("ROS2 Joystick Subscriber node started, listening to /joy topic.")

        # Store the last button state to detect changes
        self.last_buttons = None

    def listener_callback(self, msg):
        timestamp = time.time()

        # --- Process Axes ---
        axes_state = {name: 0.0 for name in DEFAULT_AXIS_MAP.values()}
        for i, value in enumerate(msg.axes):
            if i in DEFAULT_AXIS_MAP:
                axes_state[DEFAULT_AXIS_MAP[i]] = value

        state_data = {
            "type": "axes",
            "axes": axes_state,
            "timestamp": timestamp,
        }
        if self.state_queue.full():
            self.state_queue.get_nowait()  # Remove old message
        self.state_queue.put_nowait(state_data)

        # --- Process Buttons ---
        if self.last_buttons is None:
            self.last_buttons = [0] * len(msg.buttons)

        for i, pressed in enumerate(msg.buttons):
            if i < len(self.last_buttons) and pressed != self.last_buttons[i]:
                if i in DEFAULT_BUTTON_MAP:
                    event = {
                        "type": "button",
                        "name": DEFAULT_BUTTON_MAP[i],
                        "pressed": bool(pressed),
                        "timestamp": timestamp,
                    }
                    if self.event_queue.full():
                        self.event_queue.get_nowait() # Remove old message
                    self.event_queue.put_nowait(event)
        
        self.last_buttons = list(msg.buttons)


class ROS2JoystickThread(Thread):
    def __init__(self, state_queue, event_queue):
        super().__init__(name="ROS2JoystickThread", daemon=True)
        self.state_queue = state_queue
        self.event_queue = event_queue

    def run(self):
        # rclpy may already be initialized by the parent process; only init if not yet done
        if not rclpy.ok():
            rclpy.init()
        ros_node = ROS2JoystickNode(self.state_queue, self.event_queue)
        try:
            rclpy.spin(ros_node)
        finally:
            ros_node.destroy_node()
