# Object Detection &amp; Tracking in ROS 2 (Using OpenCV/YOLO)

- Set up object detection and tracking in ROS 2 using OpenCV, YOLO, or another
model.
- Capture real-time camera data (either from a physical camera or a Gazebo simulation).
- Publish detected object data as ROS 2 topics for use in robotic applications.

## Task Outline
1. Repository & Environment Setup
● Clone and build the necessary repositories (e.g., OpenCV, YOLO, ROS 2 perception
packages).
● Set up a ROS 2 package to handle real-time image processing.
● Ensure a camera feed is available from a physical camera or a simulated Gazebo
camera.
2. Object Detection & Tracking
● Implement an object detection pipeline using OpenCV, YOLO, or TensorFlow.
● Detect objects from a camera feed and classify them (e.g., bottle, box, person).
● Track the detected object over time and publish its position as a ROS 2 topic.
3. ROS 2 Integration
● Publish the detected object’s class and bounding box coordinates as a ROS 2 topic.
● Implement a ROS 2 subscriber that reads the object data and displays it in RViz or
prints it in the terminal.

## Contents:
● ROS 2 workspace with:
○ Object detection and tracking nodes.
○ Launch files to start the detection pipeline.
● Documentation:
○ A README.md with setup instructions and dependencies.
○ Steps to build and run the object detection system.
● Demonstration Video or GIF:

## Steps to run:
1. Make a workspace(or get inside one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
2. Clone the package inside a workspace:
   ```bash
   git clone https://github.com/Nandostream11/object_detection_tracking.git
   ```
3. Build the package and source the build using source install/setup.bash:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select object_detection_tracking
   ```
4. Run the camera
   ```bash
   ros2 run object_detection_tracking camera_publisher
   ```
5. Run the launch file for detection
   ```bash
   ros2 launch object_detection_tracking detection_launch.py
   ```
