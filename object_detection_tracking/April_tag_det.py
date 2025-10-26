#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import numpy as np


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.get_logger().info("Starting AprilTag Detector Node (subscribing to /camera/image_raw)...")

        # ROS 2 parameters
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('quad_decimate', 2.0)
        self.declare_parameter('display', True)  # whether to show the image window

        family = self.get_parameter('family').get_parameter_value().string_value
        quad_decimate = self.get_parameter('quad_decimate').get_parameter_value().double_value
        self.display = self.get_parameter('display').get_parameter_value().bool_value

        # Initialize detector
        self.detector = Detector(
            families=family,
            nthreads=4,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("Subscribed to /camera/image_raw")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

        self.get_logger().info_once("Detecting AprilTags... Press Ctrl+C to stop.")

        cv2.putText(frame, f"Tags detected: {len(tags)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        for tag in tags:
            cx, cy = tag.center
            corners = np.array(tag.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [corners], True, (0, 255, 0), 3)
            cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)

            label = f"ID: {tag.tag_id}"
            cv2.putText(frame, label, (int(cx) + 10, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if self.display:
            cv2.imshow("AprilTag Detection", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                self.get_logger().info("ESC pressed. Exiting display.")
                self.display = False
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
