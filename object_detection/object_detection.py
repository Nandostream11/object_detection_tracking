import rclpy
from rclpy.node import Node
import cv2
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publisher for detected objects
        self.publisher = self.create_publisher(String, '/detected_objects', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO inference
        results = self.model(frame)
        detections = results.pandas().xyxy[0]

        detected_objects = []
        for _, row in detections.iterrows():
            label = row['name']
            confidence = row['confidence']
            detected_objects.append(f"{label} ({confidence:.2f})")

        # Publish detected objects
        if detected_objects:
            self.publisher.publish(String(data=', '.join(detected_objects)))

        # Display results
        results.render()
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
