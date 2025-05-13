#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')

        # Subscribe to the ZED camera topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )

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

        # Define hand connections for drawing
        self.hand_connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),  # Thumb
            (0, 5), (5, 6), (6, 7), (7, 8),  # Index finger
            (0, 9), (9, 10), (10, 11), (11, 12),  # Middle finger
            (0, 13), (13, 14), (14, 15), (15, 16),  # Ring finger
            (0, 17), (17, 18), (18, 19), (19, 20)  # Pinky
        ]

        self.get_logger().info("Gesture recognition node started.")

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

            # Draw hand landmarks for all detected hands
            for i, hand_landmarks in enumerate(recognition_result.hand_landmarks):
                if recognition_result.gestures[i]:
                    hand_label = recognition_result.handedness[i][0].category_name  # "Left" or "Right"
                
                # Draw landmarks using the recognition result
                self.draw_hand_landmarks(cv_image, hand_landmarks,hand_label)

            # Flip the image horizontally for display only
            flipped_image = cv2.flip(cv_image, 1)

            # Display gestures for all detected hands
            for i, hand_landmarks in enumerate(recognition_result.hand_landmarks): 
                # Display the recognized gesture for this hand
                if recognition_result.gestures[i]:
                    top_gesture = recognition_result.gestures[i][0]
                    hand_label = recognition_result.handedness[i][0].category_name  # "Left" or "Right"
                    gesture_label = f"{hand_label} Hand: {top_gesture.category_name} ({top_gesture.score:.2f})"

                    if hand_label == "Left":
                        text_color = (0,255,0)
                        text_position = (10,30)
                    elif hand_label == "Right":
                        text_color = (0,0,255)
                        text_position = (10,70)

                    cv2.putText(
                        flipped_image,
                        gesture_label,
                        text_position,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        text_color,
                        2
                    )

            # Show the flipped image in a single window
            cv2.imshow("Gesture Recognition", flipped_image)

            # Wait for a short period to process window events (1 ms)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()  # Stop the ROS node when 'q' is pressed

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def draw_hand_landmarks(self, image, hand_landmarks,hand_label):
        if hand_label == "Left":
            link_color = (0,255,0)
            joint_color = (0,0,255)
        elif hand_label == "Right":
            link_color = (0,0,255)
            joint_color = (0,255,0)
        """Draws hand landmarks and connections on the image."""
        # Convert normalized coordinates to pixel coordinates
        height, width, _ = image.shape
        for connection in self.hand_connections:
            start_idx, end_idx = connection

            # Get start and end landmarks
            start = hand_landmarks[start_idx]
            end = hand_landmarks[end_idx]

            # Convert to pixel coordinates
            start_px = (int(start.x * width), int(start.y * height))
            end_px = (int(end.x * width), int(end.y * height))

            # Draw connection
            cv2.line(image, start_px, end_px, link_color, 2)

        # Draw landmarks
        for landmark in hand_landmarks:
            landmark_px = (int(landmark.x * width), int(landmark.y * height))
            cv2.circle(image, landmark_px, 3, joint_color, -1)
            cv2.circle(image, landmark_px, 5, link_color, 2)

def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
