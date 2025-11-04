#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import json
import numpy as np
import os
from datetime import datetime

class AprilTagPlotter(Node):
    def __init__(self):
        super().__init__('apriltag_plotter')
        self.get_logger().info("AprilTag Plotter Node started (subscribing to /apriltag/centers)...")

        # Subscribe to AprilTag centers
        self.subscription = self.create_subscription(String, '/apriltag/centers', self.callback, 10)

        # Store trajectory points
        self.traj_points = []
        self.last_tag_id = None

        # Static reference matrix (blue points)
        self.points = np.array([
            [0.025, -0.19, -0.054],
            [0.037921, -0.198695, -0.054],
            [0.0626794, -0.205769, -0.054],
            [0.0883857, -0.212786, -0.054],
            [0.107712, -0.220283, -0.054],
            [0.116417, -0.228098, -0.054],
            [0.11287, -0.235662, -0.054],
            [0.0975779, -0.242233, -0.054],
            [0.0727096, -0.24709, -0.054],
            [0.0416208, -0.24967, -0.054],
            [0.00837923, -0.24967, -0.054],
            [-0.0227096, -0.24709, -0.054],
            [-0.0475779, -0.242233, -0.054],
            [-0.0628698, -0.235662, -0.054],
            [-0.0664167, -0.228098, -0.054],
            [-0.057712, -0.220283, -0.054],
            [-0.0383857, -0.212786, -0.054],
            [-0.0126794, -0.205769, -0.054],
            [0.012079, -0.198695, -0.054],
            [0.025, -0.19, -0.054]
        ])

        # Timer to plot every 15 seconds
        self.timer = self.create_timer(15.0, self.plot_timer_callback)
        self.get_logger().info("Will plot all static points and trajectory every 15 seconds.")

    def callback(self, msg: String):
        """Receive AprilTag detections and store trajectory points."""
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON decode error: {e}")
            return

        for tag_id, data in detections.items():
            cx, cy = data['center']
            point = np.array([cx, cy, 0.0])

            # Track only one tag (first seen)
            if self.last_tag_id is None:
                self.last_tag_id = tag_id

            if tag_id != self.last_tag_id:
                continue

            self.traj_points.append(point)

    def plot_timer_callback(self):
        """Plot both static points and trajectory every 15 seconds."""
        num_traj = len(self.traj_points)
        self.get_logger().info(f"Plotting all static points and {num_traj} tracked points...")

        self.plot_trajectory()
        self.traj_points = []  # Clear after plotting

    def plot_trajectory(self):
        """Plot both trajectories on the Z=0 plane (top-down 2D view) and save the plot."""
        fig, ax = plt.subplots()

        # --- Plot static reference (blue) ---
        ax.plot(self.points[:, 0], self.points[:, 1],
                'bo-', label='Static Reference Points')

        if self.traj_points:
            traj_points = np.array(self.traj_points)

            # === Convert pixel coordinates to approximate world frame ===
            SCALE_X = 0.0008      # meters per pixel in X
            SCALE_Y = -0.0008     # meters per pixel in Y (flip image axis)
            OFFSET_X = -0.10
            OFFSET_Y = -0.30

            traj_points[:, 0] = traj_points[:, 0] * SCALE_X + OFFSET_X
            traj_points[:, 1] = traj_points[:, 1] * SCALE_Y + OFFSET_Y

            # --- Debug info ---
            print("=== DEBUG: Trajectory Ranges (after scaling) ===")
            print(f"X range: {traj_points[:,0].min():.4f} → {traj_points[:,0].max():.4f}")
            print(f"Y range: {traj_points[:,1].min():.4f} → {traj_points[:,1].max():.4f}")
            print("================================================")

            # --- Translation-only alignment ---
            ref_center = np.mean(self.points[:, :2], axis=0)
            traj_center = np.mean(traj_points[:, :2], axis=0)
            translation = ref_center - traj_center
            traj_points[:, 0] += translation[0]
            traj_points[:, 1] += translation[1]

            # --- Plot trajectory (red) ---
            ax.plot(traj_points[:, 0], traj_points[:, 1],
                    'ro-', label='AprilTag Trajectory (Translated)')

        # --- Axis labels & limits ---
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_aspect('equal', 'box')
        ax.set_xlim(-0.1, 0.15)
        ax.set_ylim(-0.3, -0.15)
        ax.legend()
        plt.title("AprilTag Trajectory vs Static Reference Points (Z=0 Plane)")
        plt.grid(True)

        # === SAVE PLOT ===
        os.makedirs("leg_plots", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"leg_plots/trajectory_plot_{timestamp}.png"
        plt.savefig(filename, dpi=300)
        self.get_logger().info(f"Saved plot to: {filename}")

        plt.close(fig)  # close figure to free memory

    # def plot_trajectory(self):
    #     """Plot both trajectories on the Z=0 plane (top-down 2D view)."""
    #     fig, ax = plt.subplots()

    #     # --- Plot static reference (blue) ---
    #     ax.plot(self.points[:, 0], self.points[:, 1],
    #             'bo-', label='Static Reference Points')

    #     if self.traj_points:
    #         traj_points = np.array(self.traj_points)

    #         # === Convert pixel coordinates to approximate world frame ===
    #         # Adjust these values experimentally to match your setup
    #         SCALE_X = 0.0008      # meters per pixel in X
    #         SCALE_Y = -0.0008     # meters per pixel in Y (flip image axis)
    #         OFFSET_X = -0.10      # move near your blue trajectory
    #         OFFSET_Y = -0.30

    #         traj_points[:, 0] = traj_points[:, 0] * SCALE_X + OFFSET_X
    #         traj_points[:, 1] = traj_points[:, 1] * SCALE_Y + OFFSET_Y
    #         # ============================================================

    #         # --- Debug info ---
    #         print("=== DEBUG: Trajectory Ranges (after scaling) ===")
    #         print(f"X range: {traj_points[:,0].min():.4f} → {traj_points[:,0].max():.4f}")
    #         print(f"Y range: {traj_points[:,1].min():.4f} → {traj_points[:,1].max():.4f}")
    #         print("================================================")

    #         # --- Translation-only alignment (if desired) ---
    #         ref_center = np.mean(self.points[:, :2], axis=0)
    #         traj_center = np.mean(traj_points[:, :2], axis=0)
    #         translation = ref_center - traj_center
    #         traj_points[:, 0] += translation[0]
    #         traj_points[:, 1] += translation[1]
    #         # ------------------------------------------------

    #         # --- Plot trajectory (red) ---
    #         ax.plot(traj_points[:, 0], traj_points[:, 1],
    #                 'ro-', label='AprilTag Trajectory (Translated)')

    #     # --- Axis labels & limits ---
    #     ax.set_xlabel('X')
    #     ax.set_ylabel('Y')
    #     ax.set_aspect('equal', 'box')
    #     ax.set_xlim(-0.1, 0.15)
    #     ax.set_ylim(-0.3, -0.15)
    #     ax.legend()
    #     plt.title("AprilTag Trajectory vs Static Reference Points (Z=0 Plane)")
    #     plt.grid(True)
    #     plt.show()



def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
