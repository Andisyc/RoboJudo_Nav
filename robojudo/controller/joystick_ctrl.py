import time
from queue import Empty, Queue
import json
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robojudo.controller import Controller, ctrl_registry
from robojudo.controller.ctrl_cfgs import JoystickCtrlCfg
from robojudo.controller.utils.joystick import JoystickThread


@ctrl_registry.register
class JoystickCtrl(Controller):
    cfg_ctrl: JoystickCtrlCfg

    def __init__(self, cfg_ctrl: JoystickCtrlCfg, env=None, device="cpu"):
        super().__init__(cfg_ctrl=cfg_ctrl, env=env, device=device)

        self.state_queue = Queue(maxsize=2)  # for axes
        self.event_queue = Queue(maxsize=100)  # for button/dpad events
        self.joystick_thread = JoystickThread(self.state_queue, self.event_queue)
        self.joystick_thread.start()

        self.axes_names = self.joystick_thread.config["axis_config"]["axis_map"].keys()

        self.control_mode = 'local'
        
        # ========= ROS2 Subscriber =========

        # ROS2 and mode switching setup
        self.control_mode = 'local'  # 'local' or 'ros'
        self.last_ros_cmd_time = 0
        self.last_ros_cmd = None
        self.toggle_buttons = {'Back', 'Start'}  # Use Back+Start to toggle
        self.active_toggle_buttons = set()
        self.toggle_debounce = False  # True if combo is pressed, to prevent rapid switching

        self.init_ros()

        # ========= ROS2 Subscriber =========
        
        self.reset()

    def init_ros(self):
        """Initializes the ROS2 node and subscriber in a separate thread."""
        try:
            rclpy.init()
            self.ros_node = Node('joystick_ctrl_subscriber')
            self.ros_sub = self.ros_node.create_subscription(
                String,
                '/agent/joy_cmd_json',
                self._ros_cmd_callback,
                10)
            
            self.ros_thread = Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
            self.ros_thread.start()
            print("[JoystickCtrl] ROS2 subscriber initialized for /agent/joy_cmd_json.")
        except Exception as e:
            # This can happen if rclpy.init() is called elsewhere. Assume it's handled.
            print(f"[JoystickCtrl] ROS2 initialization skipped or failed: {e}")

    def _ros_cmd_callback(self, msg):
        """Callback for receiving ROS2 commands."""
        try:
            self.last_ros_cmd = json.loads(msg.data)
            self.last_ros_cmd_time = time.time()
        except json.JSONDecodeError:
            self.ros_node.get_logger().error("Failed to decode JSON from /agent/joy_cmd_json")

    def reset(self):
        self.combination_init_buttons = self.cfg_ctrl.combination_init_buttons
        self.onhold_buttons = set()
        while not self.state_queue.empty():
            try:
                self.state_queue.get_nowait()
            except Empty:
                break

        while not self.event_queue.empty():
            try:
                self.event_queue.get_nowait()
            except Empty:
                break

        self.last_state = {
            "type": "axes",
            "axes": {name: 0.0 for name in self.axes_names},
            "timestamp": time.time(),
        }

    def get_state(self):
        try:
            state = self.state_queue.get_nowait()
            self.last_state = state.copy()
        except Empty:
            state = self.last_state

        return state

    def get_events(self):
        events = []
        while not self.event_queue.empty():
            try:
                event = self.event_queue.get_nowait()
                events.append(event)
            except Empty:
                break
        return events

    def _update_control_mode(self, events):
        """Checks for button combination to toggle control mode."""
        for event in events:
            if event['type'] == 'button' and event['name'] in self.toggle_buttons:
                if event['pressed']:
                    self.active_toggle_buttons.add(event['name'])
                else:
                    self.active_toggle_buttons.discard(event['name'])
        
        # Check if the toggle condition is met
        if self.active_toggle_buttons == self.toggle_buttons:
            if not self.toggle_debounce:
                if self.control_mode == 'local':
                    self.control_mode = 'ros'
                    print("\n[JoystickCtrl] Switched to ROS control mode.")
                else:
                    self.control_mode = 'local'
                    print("\n[JoystickCtrl] Switched to Local Joystick control mode.")
                self.toggle_debounce = True  # Prevent re-toggling until buttons are released
        else:
            self.toggle_debounce = False  # Reset debounce when buttons are no longer held

    def get_data(self):
        # Always get physical events to check for mode switch
        events = self.get_events()

        # ========= ROS2 Subscriber =========

        self._update_control_mode(events)

        # ========= ROS2 Subscriber =========

        if self.control_mode == 'ros':
            # Use ROS command if available and recent, otherwise return empty/default
            if self.last_ros_cmd and (time.time() - self.last_ros_cmd_time < 0.5):
                # We still want to pass through the physical button events so we can toggle back
                ros_cmd = self.last_ros_cmd.copy()
                ros_cmd['button_event'] = events  # Use physical events for toggling
                return ros_cmd
            else:
                # ROS is the mode, but no data is coming in. Return a safe, neutral state.
                return {
                    "axes": {name: 0.0 for name in self.axes_names},
                    "button_event": events,
                }
        else:  # 'local' control
            state = self.get_state()
            return {
                "axes": state["axes"],
                "button_event": events,
            }


    def process_triggers(self, ctrl_data):
        commands = []
        if len(self.triggers) == 0:
            return ctrl_data, commands

        # Create a copy to avoid modifying list while iterating
        for event in list(ctrl_data["button_event"]):
            if event["type"] == "button":
                if event["name"] in self.combination_init_buttons:
                    if event["pressed"]:
                        self.onhold_buttons.add(event["name"])
                    else:
                        self.onhold_buttons.discard(event["name"])
                else:
                    if event["pressed"]:
                        command = None
                        if len(self.onhold_buttons) == 0:
                            command = self.triggers.get(event["name"], None)
                        else:
                            event_combination = "+".join(sorted(list(self.onhold_buttons)) + [event["name"]])
                            command = self.triggers.get(event_combination, None)
                        if command is not None:
                            commands.append(command)
                            # remove event after triggered
                            if event in ctrl_data["button_event"]:
                                ctrl_data["button_event"].remove(event)

        return ctrl_data, commands


if __name__ == "__main__":
    # Note: This main block is for basic testing and does not initialize rclpy.
    # Full functionality requires running within a ROS2 environment.
    joystick_ctrl = JoystickCtrl(
        cfg_ctrl=JoystickCtrlCfg(
            triggers={
                "A": "[TEST_A]",
                "B": "[TEST_B]",
                "LB+Left": "[TEST_LB_Left]",
                "RB+Right": "[TEST_RB_Right]",
                "LB+RB+A": "[TEST_LB_RB_A]",
            },
        )
    )
    for _ in range(10000):
        ctrl_data = joystick_ctrl.get_data()
        ctrl_data, commands = joystick_ctrl.process_triggers(ctrl_data)
        print(f"Mode: {joystick_ctrl.control_mode}, Data: {ctrl_data}, Commands: {commands}")
        print("================================")
        time.sleep(0.3)
    exit()
