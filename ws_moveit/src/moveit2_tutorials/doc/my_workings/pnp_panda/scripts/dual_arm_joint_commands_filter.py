#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class IsaacJointCommands(Node):
    def __init__(self):
        super().__init__('isaac_joint_commands')
        self.get_logger().info("IsaacJointCommands Node has started")

        # Subscribers for left and right arm joint commands
        self.left_joint_commands_sub = self.create_subscription(
            JointState,
            '/left_isaac_joint_commands',
            self.left_joint_commands_callback,
           10
        )

        self.right_joint_commands_sub = self.create_subscription(
            JointState,
            '/right_isaac_joint_commands',
            self.right_joint_commands_callback,
            10
        )

        # Publishers for filtered joint commands
        self.left_joint_commands_pub = self.create_publisher(
            JointState,
            '/left_isaac_joint_commands_no_gripper',
            10
        )

        self.right_joint_commands_pub = self.create_publisher(
            JointState,
            '/right_isaac_joint_commands_no_gripper',
            10
        )

    def filter_finger_joints(self, msg):
        """Helper function to filter out finger joints from joint command messages"""
        filtered_msg = JointState()
        filtered_msg.header = msg.header
        filtered_msg.name = [
            joint for joint in msg.name if not joint.endswith('finger_joint1') and not joint.endswith('finger_joint2')
        ]
        filtered_msg.position = [
            pos for joint, pos in zip(msg.name, msg.position) if not joint.endswith('finger_joint1') and not joint.endswith('finger_joint2')
        ] if msg.position else []
        filtered_msg.velocity = [
            vel for joint, vel in zip(msg.name, msg.velocity) if not joint.endswith('finger_joint1') and not joint.endswith('finger_joint2')
        ] if msg.velocity else []
        filtered_msg.effort = [
            eff for joint, eff in zip(msg.name, msg.effort) if not joint.endswith('finger_joint1') and not joint.endswith('finger_joint2')
        ] if msg.effort else []
        return filtered_msg

    def publish_filtered_message(self, publisher, msg):
        """Publish the message only if it contains valid data"""
        if msg.name:  # Only publish if the message has at least one joint name
            publisher.publish(msg)

    def left_joint_commands_callback(self, msg):
        # Filter out finger joints for the left arm and publish the result
        filtered_msg = self.filter_finger_joints(msg)
        self.publish_filtered_message(self.left_joint_commands_pub, filtered_msg)

    def right_joint_commands_callback(self, msg):
        # Filter out finger joints for the right arm and publish the result
        filtered_msg = self.filter_finger_joints(msg)
        self.publish_filtered_message(self.right_joint_commands_pub, filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacJointCommands()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
