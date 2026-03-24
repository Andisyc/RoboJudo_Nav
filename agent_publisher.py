import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading

# --- Configuration ---
MAX_SPEED = 0.8  # Speed limit for joystick axes, value between 0.0 and 1.0

# Keyboard to Joystick mapping
KEY_AXIS_MAP = {
    'w': ('LeftY', 1.0),   # Forward
    's': ('LeftY', -1.0),  # Backward
    'a': ('LeftX', -1.0),  # Left
    'd': ('LeftX', 1.0),   # Right
    'i': ('RightY', 1.0),   # Cam Up
    'k': ('RightY', -1.0),  # Cam Down
    'j': ('RightX', -1.0),  # Cam Left
    'l': ('RightX', 1.0),   # Cam Right
    'u': ('LT', 1.0),   # Left Trigger
    'o': ('RT', 1.0),   # Right Trigger
}

KEY_BUTTON_MAP = {
    'h': 'A',
    # Note: j, k, l are used for both axes and buttons. This might not be ideal but can work.
    'j': 'B',
    'k': 'X',
    'l': 'Y',
    ' ': 'Start',
    'b': 'Back',
    'q': 'LB',
    'e': 'RB',
    # D-Pad mapping
    'up': 'Up',
    'down': 'Down',
    'left': 'Left',
    'right': 'Right',
}
# --- End Configuration ---

try:
    from pynput import keyboard
except ImportError:
    print("\n[ERROR] pynput library not found.")
    print("Please install it using: pip install pynput\n")
    exit(1)


class AgentPublisher(Node):
    def __init__(self):
        super().__init__('agent_publisher')
        self.publisher_ = self.create_publisher(String, '/agent/joy_cmd_json', 10)
        
        # State management
        self.axes_state = {
            "LeftX": 0.0, "LeftY": 0.0, "LT": 0.0,
            "RightX": 0.0, "RightY": 0.0, "RT": 0.0
        }
        self.button_events = [] # A queue for button press/release events
        self.active_keys = set() # To track currently pressed keys for axes

        self.timer = self.create_timer(0.05, self.publish_command)  # Publish at 20Hz
        self.get_logger().info('Agent Publisher started. Use your keyboard to send commands.')
        self.print_instructions()

    def print_instructions(self):
        print("------------------------------------------")
        print("Keyboard Control for Agent Publisher:")
        print("  - W/A/S/D: Left Stick (Move)")
        print("  - Arrow Keys: D-Pad (Up/Down/Left/Right)")
        print("  - I/J/K/L: Right Stick (Camera)")
        print("  - U/O: Left/Right Triggers")
        print("  - Q/E: LB/RB")
        print("  - H, J, K, L: A, B, X, Y buttons")
        print("  - Space/B: Start/Back")
        print("  - Press 'Esc' to exit.")
        print(f"  - Max Speed: {MAX_SPEED}")
        print("------------------------------------------")

    def on_press(self, key):
        self.get_logger().info(f"Key pressed: {key}")
        try:
            key_char = key.char
        except AttributeError:
            key_char = key.name # For special keys like 'space' or 'esc'

        if key_char in self.active_keys:
            return # Avoid repeat events for held-down keys
        self.active_keys.add(key_char)

        # Handle Axes
        if key_char in KEY_AXIS_MAP:
            axis_name, direction = KEY_AXIS_MAP[key_char]
            self.axes_state[axis_name] = direction * MAX_SPEED

        # Handle Buttons
        if key_char in KEY_BUTTON_MAP:
            button_name = KEY_BUTTON_MAP[key_char]
            event = {"type": "button", "name": button_name, "pressed": True, "timestamp": time.time()}
            self.button_events.append(event)


    def on_release(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = key.name

        if key_char not in self.active_keys:
            return
        self.active_keys.remove(key_char)

        # Handle Axes
        if key_char in KEY_AXIS_MAP:
            axis_name, _ = KEY_AXIS_MAP[key_char]
            # Check if another key affecting the same axis is still pressed
            is_other_key_active = False
            for active_key in self.active_keys:
                if active_key in KEY_AXIS_MAP and KEY_AXIS_MAP[active_key][0] == axis_name:
                    is_other_key_active = True
                    break
            if not is_other_key_active:
                self.axes_state[axis_name] = 0.0

        # Handle Buttons
        if key_char in KEY_BUTTON_MAP:
            button_name = KEY_BUTTON_MAP[key_char]
            event = {"type": "button", "name": button_name, "pressed": False, "timestamp": time.time()}
            self.button_events.append(event)
        
        # Exit condition
        if key == keyboard.Key.esc:
            print("Escape key pressed. Shutting down...")
            # A more robust solution might use threading.Event to signal shutdown
            rclpy.shutdown()
            return False # Stop the listener thread

    def publish_command(self):
        # Grab all button events that have occurred since last publish
        events_to_send = self.button_events
        self.button_events = [] # Clear the queue for the next cycle

        # Construct the final command and publish it as a JSON string
        command = {
            "axes": self.axes_state.copy(),
            "button_event": events_to_send,
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    agent_publisher = AgentPublisher()

    # Start keyboard listener in a non-blocking way
    listener = keyboard.Listener(
        on_press=agent_publisher.on_press,
        on_release=agent_publisher.on_release)
    listener.start()

    # Spin the ROS node. This will block until rclpy.shutdown() is called.
    try:
        rclpy.spin(agent_publisher)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass # Expected exceptions on shutdown
    finally:
        # Cleanup
        print("Cleaning up and shutting down.")
        listener.stop()
        agent_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
