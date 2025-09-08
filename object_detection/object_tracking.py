import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            String, '/detected_objects', self.tracking_callback, 10)

    def tracking_callback(self, msg):
        self.get_logger().info(f"Tracking: {msg.data}")

def main():
    rclpy.init()
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
