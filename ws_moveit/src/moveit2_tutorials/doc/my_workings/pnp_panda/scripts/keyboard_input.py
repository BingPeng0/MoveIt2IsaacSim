#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import keyboard
import signal

# Constants for key mappings
KEY_MAPPING = {
    'w': ('right', 'linear', (0, 1, 0)),   # Right arm forward
    's': ('right', 'linear', (0, -1, 0)),  # Right arm backward
    'a': ('right', 'linear', (-1, 0, 0)),  # Right arm left
    'd': ('right', 'linear', (1, 0, 0)),   # Right arm right
    'r': ('right', 'linear', (0, 0, 1)),   # Right arm up
    'f': ('right', 'linear', (0, 0, -1)),  # Right arm down
    'q': ('right', 'angular', (0, 0, -5)), # Right arm rotate Z CCW
    'e': ('right', 'angular', (0, 0, 5)),  # Right arm rotate Z CW
    'z': ('right', 'angular', (-1, 0, 0)), # Right arm rotate Y CCW
    'c': ('right', 'angular', (1, 0, 0)),  # Right arm rotate Y CW
    'x': ('right', 'angular', (0, 1, 0)),   # Right arm rotate X CW
    'v': ('right', 'angular', (0, -1, 0)),  # Right arm rotate X CCW
    '8': ('left', 'linear', (0, 1, 0)),    # Left arm forward
    '5': ('left', 'linear', (0, -1, 0)),   # Left arm backward
    '4': ('left', 'linear', (-1, 0, 0)),   # Left arm left
    '6': ('left', 'linear', (1, 0, 0)),    # Left arm right
    '-': ('left', 'linear', (0, 0, 1)),    # Left arm up
    '+': ('left', 'linear', (0, 0, -1)),   # Left arm down
    '7': ('left', 'angular', (0, 0, -5)),  # Left arm rotate Z CCW
    '9': ('left', 'angular', (0, 0, 5)),   # Left arm rotate Z CW
    '1': ('left', 'angular', (-1, 0, 0)),  # Left arm rotate Y CCW
    '3': ('left', 'angular', (1, 0, 0)),   # Left arm rotate Y CW
    '2': ('left', 'angular', (0, 1, 0)),  # Left arm rotate X CW
    '0': ('left', 'angular', (0, -1, 0))    # Left arm rotate X CCW
}

LINEAR_SCALE = 0.05
ROTATION_SCALE = 0.2
ROS_QUEUE_SIZE = 10

class KeyboardServo(Node):
    def __init__(self):
        super().__init__('servo_keyboard_input')

        # Publishers for commands
        self.twist_pub_left = self.create_publisher(TwistStamped, "/servo_node_left/delta_twist_cmds", ROS_QUEUE_SIZE)
        self.twist_pub_right = self.create_publisher(TwistStamped, "/servo_node_right/delta_twist_cmds", ROS_QUEUE_SIZE)

        # Servo clients
        self.switch_input_left = self.create_client(ServoCommandType, 'servo_node_left/switch_command_type')
        self.switch_input_right = self.create_client(ServoCommandType, 'servo_node_right/switch_command_type')

        # Gripper Control
        self.openleftgripper = self.create_publisher(Bool,"OpenLeftGripper",10)
        self.openrightgripper = self.create_publisher(Bool,"OpenRightGripper",10)
        self.closeleftgripper = self.create_publisher(Bool,"CloseLeftGripper",10)
        self.closerightgripper = self.create_publisher(Bool,"CloseRightGripper",10)

    def publish_twist(self, arm, motion_type, values):
        pub = self.twist_pub_left if arm == 'left' else self.twist_pub_right
        twist_msg = TwistStamped()
        if motion_type == 'linear':
            twist_msg.header.frame_id = "world"
            twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z = [LINEAR_SCALE * v for v in values]
        elif motion_type == 'angular':
            twist_msg.header.frame_id = "left_panda_hand" if arm == 'left' else "right_panda_hand"
            twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z = [ROTATION_SCALE * v for v in values]
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(twist_msg)

    def switch_input_mode(self, mode):
        """Switch the servo input mode for both arms."""
        request = ServoCommandType.Request(command_type=mode)
        for client in [self.switch_input_left, self.switch_input_right]:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().success:
                    self.get_logger().info(f"Switched to mode {mode}")
                else:
                    self.get_logger().warn("Failed to switch input mode")

    def get_opposite_msg(self,current_msg):
         return Bool(data=not current_msg.data)   

    def key_loop(self):
        msg = Bool(data=False)
        msgleftopen = msg
        msgleftclose = msg
        msgrightopen = msg
        msgrightclose = msg
        print("Reading from keyboard")
        print("---------------------------")
        print("Commands:")
        print("  - Use WASD to control the right arm (linear movement).")
        print("  - Use RF to move the right arm up/down.")
        print("  - Use QA, ZC, XV for right arm rotations.")
        print("  - Use TG for right gripper control.")
        print("  - Use 8465 on the keypad to control the left arm (linear movement).")
        print("  - Use -/+ to move the left arm up/down.")
        print("  - Use 79, 13, 20 for left arm rotations.")
        print("  - Use .enter for left gripper control.")
        print("Switch Input Modes:")
        print("  - H: Switch to Twist mode.")
        print("  - J: Switch to Joint Jog mode.")
        print("  - K: Switch to Pose mode.")
        print("Press 'Esc' to quit.")

        while rclpy.ok():
            try:
                linear_keys = []
                angular_keys = []

                for key, (arm, motion_type, values) in KEY_MAPPING.items():
                    if keyboard.is_pressed(key):
                        if motion_type == 'linear':
                            linear_keys.append((arm, values))
                        elif motion_type == 'angular':
                            angular_keys.append((arm, values))

                # Check for conflicting inputs
                if linear_keys and angular_keys:
                    self.get_logger().warn("Linear and angular keys cannot be pressed together. Ignoring input.")
                    continue

                # Combine linear inputs
                for arm in ['left', 'right']:
                    combined_linear = [0.0, 0.0, 0.0]
                    combined_angular = [0.0, 0.0, 0.0]

                    for input_arm, values in linear_keys:
                        if input_arm == arm:
                            combined_linear = [sum(x) for x in zip(combined_linear, values)]

                    for input_arm, values in angular_keys:
                        if input_arm == arm:
                            combined_angular = [sum(x) for x in zip(combined_angular, values)]

                    if any(combined_linear):
                        self.publish_twist(arm, 'linear', combined_linear)
                    if any(combined_angular):
                        self.publish_twist(arm, 'angular', combined_angular)

                if keyboard.is_pressed('h'):
                    self.switch_input_mode(ServoCommandType.Request.TWIST)
                elif keyboard.is_pressed('j'):
                    self.switch_input_mode(ServoCommandType.Request.JOINT_JOG)
                elif keyboard.is_pressed('k'):
                    self.switch_input_mode(ServoCommandType.Request.POSE)
                elif keyboard.is_pressed('t'):
                    msgrightopen=self.get_opposite_msg(msgrightopen)
                    self.openrightgripper.publish(msgrightopen)
                elif keyboard.is_pressed('g'):
                    msgrightclose=self.get_opposite_msg(msgrightclose)
                    self.closerightgripper.publish(msgrightclose)
                elif keyboard.is_pressed('.'):
                    msgleftopen=self.get_opposite_msg(msgleftopen)
                    self.openleftgripper.publish(msgleftopen)
                elif keyboard.is_pressed('enter'):
                    msgleftclose=self.get_opposite_msg(msgleftclose)
                    self.closeleftgripper.publish(msgleftclose)
                elif keyboard.is_pressed('esc'):
                    print("Exiting key loop.")
                    break

            except Exception as e:
                self.get_logger().error(f"Error in key loop: {e}")
                break


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServo()

    signal.signal(signal.SIGINT, lambda s, f: node.get_logger().info("SIGINT received. Shutting down."))
    try:
        node.key_loop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from rclpy.publisher import Publisher
# from rclpy.client import Client
# from std_msgs.msg import String, Bool
# from geometry_msgs.msg import TwistStamped, PoseStamped
# from control_msgs.msg import JointJog
# from moveit_msgs.srv import ServoCommandType
# import termios
# import sys
# import os
# import signal
# import select

# # Constants for key mappings
# KEY_MAPPING = {
#     'W': ('right', 'linear', (0, 1, 0)),   # Right arm forward
#     'S': ('right', 'linear', (0, -1, 0)),  # Right arm backward
#     'A': ('right', 'linear', (-1, 0, 0)),  # Right arm left
#     'D': ('right', 'linear', (1, 0, 0)),   # Right arm right
#     'R': ('right', 'linear', (0, 0, 1)),   # Right arm up
#     'F': ('right', 'linear', (0, 0, -1)),  # Right arm down
#     'Q': ('right', 'angular', (0, 0, -1)), # Right arm rotate Z CCW
#     'E': ('right', 'angular', (0, 0, 1)),  # Right arm rotate Z CW
#     'Z': ('right', 'angular', (-1, 0, 0)), # Right arm rotate Y CCW
#     'C': ('right', 'angular', (1, 0, 0)),  # Right arm rotate Y CW
#     'X': ('right', 'angular', (0, -1, 0)), # Right arm rotate X CCW
#     'V': ('right', 'angular', (0, 1, 0)),  # Right arm rotate X CW
#     '8': ('left', 'linear', (0, 1, 0)),    # Left arm forward
#     '2': ('left', 'linear', (0, -1, 0)),   # Left arm backward
#     '4': ('left', 'linear', (-1, 0, 0)),   # Left arm left
#     '6': ('left', 'linear', (1, 0, 0)),    # Left arm right
#     '-': ('left', 'linear', (0, 0, 1)),    # Left arm up
#     '+': ('left', 'linear', (0, 0, -1)),   # Left arm down
#     '7': ('left', 'angular', (0, 0, -1)),  # Left arm rotate Z CCW
#     '9': ('left', 'angular', (0, 0, 1)),   # Left arm rotate Z CW
#     '1': ('left', 'angular', (-1, 0, 0)),  # Left arm rotate Y CCW
#     '3': ('left', 'angular', (1, 0, 0)),   # Left arm rotate Y CW
#     '0': ('left', 'angular', (0, -1, 0)),  # Left arm rotate X CCW
#     '.': ('left', 'angular', (0, 1, 0))    # Left arm rotate X CW
# }

# LINEAR_SCALE = 0.1
# ROTATION_SCALE = 1.0
# ROS_QUEUE_SIZE = 10

# class KeyboardReader:
#     """Utility class for non-blocking keyboard input."""

#     def __init__(self):
#         self.file_descriptor = sys.stdin.fileno()
#         self.cooked = termios.tcgetattr(self.file_descriptor)
#         self.raw = self.cooked[:]
#         self.raw[3] &= ~(termios.ICANON | termios.ECHO)
#         termios.tcsetattr(self.file_descriptor, termios.TCSANOW, self.raw)

#     def read_one(self):
#         """Reads a single key press."""
#         dr, _, _ = select.select([sys.stdin], [], [], 0)
#         return os.read(self.file_descriptor, 1).decode() if dr else ""

#     def shutdown(self):
#         termios.tcsetattr(self.file_descriptor, termios.TCSANOW, self.cooked)


# class KeyboardServo(Node):
#     def __init__(self):
#         super().__init__('servo_keyboard_input')

#         # Publishers for commands
#         self.twist_pub_left = self.create_publisher(TwistStamped, "/servo_node_left/delta_twist_cmds", ROS_QUEUE_SIZE)
#         self.twist_pub_right = self.create_publisher(TwistStamped, "/servo_node_right/delta_twist_cmds", ROS_QUEUE_SIZE)

#         # Servo clients
#         self.switch_input_left = self.create_client(ServoCommandType, 'servo_node_left/switch_command_type')
#         self.switch_input_right = self.create_client(ServoCommandType, 'servo_node_right/switch_command_type')

#     def publish_twist(self, arm, motion_type, values):
#         pub = self.twist_pub_left if arm == 'left' else self.twist_pub_right
#         twist_msg = TwistStamped()
#         if motion_type == 'linear':
#             twist_msg.header.frame_id = "world"
#             twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z = [LINEAR_SCALE * v for v in values]
#         elif motion_type == 'angular':
#             twist_msg.header.frame_id = "left_panda_hand" if arm == 'left' else "right_panda_hand"
#             twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z = [ROTATION_SCALE * v for v in values]
#         twist_msg.header.stamp = self.get_clock().now().to_msg()
#         pub.publish(twist_msg)

#     def switch_input_mode(self, mode):
#         """Switch the servo input mode for both arms."""
#         request = ServoCommandType.Request(command_type=mode)
#         for client in [self.switch_input_left, self.switch_input_right]:
#             if client.wait_for_service(timeout_sec=1.0):
#                 future = client.call_async(request)
#                 rclpy.spin_until_future_complete(self, future)
#                 if future.result() and future.result().success:
#                     self.get_logger().info(f"Switched to mode {mode}")
#                 else:
#                     self.get_logger().warn("Failed to switch input mode")

#     def key_loop(self):
#         keyboard_reader = KeyboardReader()

#         print("Reading from keyboard")
#         print("---------------------------")
#         print("Commands:")
#         print("  - Use WASD to control the right arm (linear movement).")
#         print("  - Use RF to move the right arm up/down.")
#         print("  - Use QA, ZC, XV for right arm rotations.")
#         print("  - Use 8462 on the keypad to control the left arm (linear movement).")
#         print("  - Use -/+ to move the left arm up/down.")
#         print("  - Use 79, 13, 0. for left arm rotations.")
#         print("Switch Input Modes:")
#         print("  - H: Switch to Twist mode.")
#         print("  - J: Switch to Joint Jog mode.")
#         print("  - K: Switch to Pose mode.")
#         print("Press '`' to quit.")

#         while rclpy.ok():
#             try:
#                 key = keyboard_reader.read_one().upper()

#                 if key == '`':
#                     break

#                 if key in KEY_MAPPING:
#                     arm, motion_type, values = KEY_MAPPING[key]
#                     self.publish_twist(arm, motion_type, values)

#                 elif key == 'H':
#                     self.switch_input_mode(ServoCommandType.Request.TWIST)
#                 elif key == 'J':
#                     self.switch_input_mode(ServoCommandType.Request.JOINT_JOG)
#                 elif key == 'K':
#                     self.switch_input_mode(ServoCommandType.Request.POSE)

#             except Exception as e:
#                 self.get_logger().error(f"Error in key loop: {e}")
#                 break

#         keyboard_reader.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardServo()

#     signal.signal(signal.SIGINT, lambda s, f: node.get_logger().info("SIGINT received. Shutting down."))
#     try:
#         node.key_loop()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
