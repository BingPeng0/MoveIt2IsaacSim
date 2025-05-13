#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from moveit_msgs.srv import ServoCommandType


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')

        # Publishers for left and right arms
        self.twist_pub_left = self.create_publisher(TwistStamped, "/servo_node_left/delta_twist_cmds", 10)
        self.twist_pub_right = self.create_publisher(TwistStamped, "/servo_node_right/delta_twist_cmds", 10)

        # Service clients to switch servo mode to Twist
        self.switch_service_left = self.create_client(ServoCommandType, '/servo_node_left/switch_command_type')
        self.switch_service_right = self.create_client(ServoCommandType, '/servo_node_right/switch_command_type')

        # Initialize CvBridge for ROS <-> OpenCV conversions
        self.bridge = CvBridge()

        # Load the gesture recognizer model with num_hands=2
        model_path = '/root/ws_moveit/src/moveit2_tutorials/doc/my_workings/pnp_panda/scripts/gesture_recognizer.task'
        base_options = python.BaseOptions(model_asset_path=model_path)
        gesture_options = vision.GestureRecognizerOptions(
            base_options=base_options,
            num_hands=2  # Allow detection of two hands
        )
        self.gesture_recognizer = vision.GestureRecognizer.create_from_options(gesture_options)

        # Visualization settings
        self.hand_connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),  # Thumb
            (0, 5), (5, 6), (6, 7), (7, 8),  # Index finger
            (0, 9), (9, 10), (10, 11), (11, 12),  # Middle finger
            (0, 13), (13, 14), (14, 15), (15, 16),  # Ring finger
            (0, 17), (17, 18), (18, 19), (19, 20)  # Pinky
        ]
        self.linear_scale = 0.05  # Scale for linear movement
        self.get_logger().info("Gesture recognition node started.")

    def on_start(self):
        """Called on node startup to set the initial state."""
        self.switch_to_twist_mode()  # Ensure both servo nodes are in Twist mode

        # Subscribe to the ZED camera topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            QoSProfile(depth=10)
        )

    def switch_to_twist_mode(self):
        """Switch the control mode to Twist for both arms."""
        for service, arm in [(self.switch_service_left, "Left"), (self.switch_service_right, "Right")]:
            if service.wait_for_service(timeout_sec=2.0):
                request = ServoCommandType.Request()
                request.command_type = ServoCommandType.Request.TWIST
                future = service.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().success:
                    self.get_logger().info(f"{arm} arm switched to Twist mode.")
                else:
                    self.get_logger().warn(f"Failed to switch {arm} arm to Twist mode.")
            else:
                self.get_logger().error(f"{arm} arm service not available.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert OpenCV Image to RGB (MediaPipe requires RGB)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Create an Image object for Gesture Recognition
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

            # Perform gesture recognition
            recognition_result = self.gesture_recognizer.recognize(mp_image)

            # Draw landmarks and process gestures for all detected hands
            for i, hand_landmarks in enumerate(recognition_result.hand_landmarks):
                if recognition_result.gestures[i]:
                    top_gesture = recognition_result.gestures[i][0]
                    hand_label = recognition_result.handedness[i][0].category_name  # "Left" or "Right"
                    self.process_gesture(hand_label, top_gesture.category_name)
                    self.draw_hand_landmarks(cv_image, hand_landmarks, hand_label, top_gesture)

            # Flip the image horizontally for display
            flipped_image = cv2.flip(cv_image, 1)

            # Draw text on flipped image
            for i, hand_landmarks in enumerate(recognition_result.hand_landmarks):
                if recognition_result.gestures[i]:
                    top_gesture = recognition_result.gestures[i][0]
                    hand_label = recognition_result.handedness[i][0].category_name
                    gesture_label = f"{hand_label} Hand: {top_gesture.category_name} ({top_gesture.score:.2f})"

                    # Dynamic text positions for each hand
                    text_position = (10, 30 if hand_label == "Left" else 60)
                    text_color = (0, 255, 0) if hand_label == "Left" else (0, 0, 255)  # Green for left, Red for right
                    cv2.putText(flipped_image, gesture_label, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

            # Show the flipped image
            cv2.imshow("Gesture Recognition", flipped_image)

            # Wait for a short period to process window events (1 ms)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_gesture(self, hand_label, gesture_name):
        """Processes the detected gesture and publishes to the correct arm."""
        twist_msg = TwistStamped()

        # Map gesture to linear movement
        if gesture_name == "Forward":
            twist_msg.twist.linear.y = self.linear_scale
        elif gesture_name == "Backward":
            twist_msg.twist.linear.y = -self.linear_scale
        elif gesture_name == "Left":
            twist_msg.twist.linear.x = -self.linear_scale
        elif gesture_name == "Right":
            twist_msg.twist.linear.x = self.linear_scale
        elif gesture_name == "Up":
            twist_msg.twist.linear.z = self.linear_scale
        elif gesture_name == "Down":
            twist_msg.twist.linear.z = -self.linear_scale
        elif gesture_name == "Stop":
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0

        # Publish the twist message to the appropriate arm
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"

        if hand_label == "Left":
            self.twist_pub_right.publish(twist_msg)  # Left hand controls right arm
        elif hand_label == "Right":
            self.twist_pub_left.publish(twist_msg)  # Right hand controls left arm

    def draw_hand_landmarks(self, image, hand_landmarks, hand_label, top_gesture):
        """Draws hand landmarks, connections, and gesture labels."""
        # Determine colors based on the hand label
        if hand_label == "Left":
            link_color = (0, 255, 0)  # Green for left hand
            joint_color = (0, 0, 255)  # Red for left hand
        else:
            link_color = (0, 0, 255)  # Red for right hand
            joint_color = (0, 255, 0)  # Green for right hand

        # Draw landmarks and connections
        height, width, _ = image.shape
        for connection in self.hand_connections:
            start_idx, end_idx = connection
            start = hand_landmarks[start_idx]
            end = hand_landmarks[end_idx]
            start_px = (int(start.x * width), int(start.y * height))
            end_px = (int(end.x * width), int(end.y * height))
            cv2.line(image, start_px, end_px, link_color, 2)

        for landmark in hand_landmarks:
            landmark_px = (int(landmark.x * width), int(landmark.y * height))
            cv2.circle(image, landmark_px, 3, joint_color, -1)  # Inner circle
            cv2.circle(image, landmark_px, 5, link_color, 2)    # Outer circle


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    node.on_start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
