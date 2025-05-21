###################################################################################
# MIT License
#
# Copyright (c) 2024 Hannibal Paul
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################

__author__ = "Hannibal Paul"

import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # Parameters loaded from YAML
        self.declare_parameter('cam_pos', [0.15, 0.0, -0.02])
        self.declare_parameter('arm1_pos', [-0.1500, 0.0000])
        self.declare_parameter('arm2_pos', [0.0750, -0.1299])
        self.declare_parameter('arm3_pos', [0.0750, 0.1299])
        self.declare_parameter('arm_diameter', 0.01)

        # Retrieve parameters
        self.cam_pos = self.get_parameter('cam_pos').value
        arm1 = self.get_parameter('arm1_pos').value
        arm2 = self.get_parameter('arm2_pos').value
        arm3 = self.get_parameter('arm3_pos').value
        self.target_points = np.array([arm1, arm2, arm3], dtype=float)
        self.search_radius = self.get_parameter('arm_diameter').value

        # Subscriber
        self.subs_pc = self.create_subscription(PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)

        # Publisher
        self.pub_z = self.create_publisher(Float64MultiArray, '/tams2/arm_heights', 10)

        # Logger
        self.get_logger().info(f"Camera position: {self.cam_pos}")
        self.get_logger().info(f"Target points: {self.target_points}")
        self.get_logger().info(f"Search radius: {self.search_radius}")
        self.get_logger().info("Point cloud processor started!")

    def pointcloud_callback(self, msg):
        self.get_logger().info("----------------------------------------------------")
        cloud_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        if cloud_data.size == 0:  # Check if point cloud is empty
            self.get_logger().warn("Received empty point cloud!")
            return

        # X and Z needs to be swapped for correcting frame and camera offset corrections. 
        x =  cloud_data['z'] + self.cam_pos[0] # PX4 x
        y = -cloud_data['y'] + self.cam_pos[1] # PX4 y
        z =  cloud_data['x'] + self.cam_pos[2] # PX4 z

        # Stack them into an (N,3) matrix
        points = np.column_stack((x, y, z)).astype(np.float32)

        # Filter out points where z < z_threshold
        z_threshold = 0.4  # Set your threshold
        valid_indices = points[:, 2] >= z_threshold
        filtered_points = points[valid_indices]

        xy_points = filtered_points[:, :2]  # Extract only x, y
        z_values = filtered_points[:, 2]    # Extract only z

        if np.any(np.isnan(xy_points)) or np.any(np.isinf(xy_points)):
            self.get_logger().error("xy_points contains NaN or Inf!")
            return
        if np.any(np.isnan(self.target_points)) or np.any(np.isinf(self.target_points)):
            self.get_logger().error("target_points contains NaN or Inf!")
            return


        # KDTree for fast nearest-neighbor lookup
        tree = cKDTree(xy_points)
        z_results = []
        cannot_land = False

        # Find average height in each arm region
        for i, target in enumerate(self.target_points):
            indices = tree.query_ball_point(target, self.search_radius, p=np.inf)

            if indices:
                avg_z = np.mean(z_values[indices])
                z_results.append(avg_z)
                self.get_logger().info(f"Point {i+1} ({target[0]}, {target[1]}): Avg Z = {avg_z:.3f}")
            else:
                z_results.append(float('nan'))
                self.get_logger().warn(f"Point {i+1} ({target[0]}, {target[1]}): No nearby points found!")
                cannot_land = True

        if cannot_land:
            self.get_logger().warn("Camera too close or too far, change location/height!")
            return

        # Compute the central height of the surface
        z_center = (max(z_results) - min(z_results)) / 2.0
        land_dist = min(z_results) + z_center
        self.get_logger().info(f"Computed surface center: {land_dist:.3f}")

        # Compute arm relative heights from its center
        target_heights = np.array(z_results) - land_dist
        self.get_logger().info(f"Target heights: {np.array2string(target_heights, precision=3)}")

        # Publish all three Z values
        msg = Float64MultiArray()
        msg.data =  [float(z) for z in target_heights]
        self.pub_z.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
