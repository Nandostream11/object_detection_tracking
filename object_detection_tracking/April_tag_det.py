#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import numpy as np
import json


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.get_logger().info("Starting AprilTag Detector Node (subscribing to /camera/image_raw)...")

        # Parameters
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('quad_decimate', 2.0)
        self.declare_parameter('display', True)
        self.declare_parameter('max_traj_length', 100)

        family = self.get_parameter('family').get_parameter_value().string_value
        quad_decimate = self.get_parameter('quad_decimate').get_parameter_value().double_value
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        self.max_traj_length = self.get_parameter('max_traj_length').get_parameter_value().integer_value

        # AprilTag detector
        self.detector = Detector(
            families=family,
            nthreads=4,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

        # Subscribers / Publishers
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/apriltag/centers', 10)

        # Store trajectories: {tag_id: [(x1, y1), (x2, y2), ...]}
        self.trajectories = {}

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

        detections_msg = {}

        for tag in tags:
            tag_id = tag.tag_id
            cx, cy = map(int, tag.center)
            # --- Approximate pixel → world coordinate mapping ---
            # These constants can be tuned to align with your reference Bezier curve
            SCALE_X = 0.0004     # meters per pixel in X
            SCALE_Y = -0.0004    # negative flips Y-axis (image Y increases downward)
            OFFSET_X = -0.10     # where your world origin starts in meters
            OFFSET_Y = -0.30     # offset to align with Bezier Y region

            wx = cx * SCALE_X + OFFSET_X
            wy = cy * SCALE_Y + OFFSET_Y

            detections_msg[tag_id] = {'center': [wx, wy]}


            # Update trajectory
            if tag_id not in self.trajectories:
                self.trajectories[tag_id] = []
            self.trajectories[tag_id].append((cx, cy))
            if len(self.trajectories[tag_id]) > self.max_traj_length:
                self.trajectories[tag_id].pop(0)

            # Draw marker and trajectory
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            pts = np.array(self.trajectories[tag_id], np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], False, (255, 255, 0), 2)

            # Label
            label = f"ID:{tag_id}"
            cv2.putText(frame, label, (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            detections_msg[tag_id] = {'center': [cx, cy]}

        # Publish detections as JSON
        if detections_msg:
            msg_out = String()
            msg_out.data = json.dumps(detections_msg)
            self.publisher.publish(msg_out)

        # Display
        if self.display:
            cv2.imshow("AprilTag Detection with Trajectories", frame)
            if cv2.waitKey(1) & 0xFF == 27:
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
