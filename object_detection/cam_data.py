import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_=self.create_publisher(Image,'/camera/image_raw',10)

        #opencv video capture
        self.cap=cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Failed to open camera.")
            return
        
        self.bridge=CvBridge()

        #Timer to grab frames at ~30Hz
        timer_period=1.0/30.0
        self.timer= self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame=self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return
        
        #convert opencv image to ROS2 Image msg
        msg=self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info("Published frame")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()
def main():
    rclpy.init()
    node=CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()  
