from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	Node(package='object_detection_tracking', executable='camera_publisher'),
        Node(package='object_detection_tracking', executable='apriltag_detector'),
        Node(package='object_detection_tracking', executable='apriltag_plotter')
    ])
