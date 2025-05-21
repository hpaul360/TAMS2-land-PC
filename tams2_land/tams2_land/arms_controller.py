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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import time, math
import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Subscriber
        self.height_sub = self.create_subscription(Float64MultiArray, '/tams2/arm_heights', self.height_callback, 10)

        # Publishers
        self.cam_servo_pub = self.create_publisher(Float64, '/model/tams2_0/servo_0', 10)
        self.a1_rotate_pub = self.create_publisher(Float64, '/model/tams2_0/servo_1', 10)
        self.a1_slider_pub = self.create_publisher(Float64, '/model/tams2_0/servo_2', 10)
        self.a2_rotate_pub = self.create_publisher(Float64, '/model/tams2_0/servo_3', 10)
        self.a2_slider_pub = self.create_publisher(Float64, '/model/tams2_0/servo_4', 10)
        self.a3_rotate_pub = self.create_publisher(Float64, '/model/tams2_0/servo_5', 10)
        self.a3_slider_pub = self.create_publisher(Float64, '/model/tams2_0/servo_6', 10)

        self.target_heights = [0.0, 0.0, 0.0]  # Store target heights
        self.step_size = 0.005  # Step size for smooth movement
        self.timer = self.create_timer(0.1, self.smooth_adjustment)

        time.sleep(1)
        rot = Float64()
        rot.data = 1.57
        self.cam_servo_pub.publish(rot)


        # Logger
        self.get_logger().info("Arm contol started!")

    def height_callback(self, msg):
        rot = Float64()
        rot.data = 1.57
        self.cam_servo_pub.publish(rot)

        if len(msg.data) < 3:
            self.get_logger().warn("Received incorrect displacement value array!")
            return

        #self.get_logger().info(f"Received Heights: {msg.data}")

        self.target_heights = -np.array(msg.data)

        if math.isnan(self.target_heights[0]) or math.isnan(self.target_heights[1]) or math.isnan(self.target_heights[2]):
            self.get_logger().warn("Received NaN values, skipping arm position update!")
            return

        self.get_logger().info(f"Received Z values: {self.target_heights[0]:.4f}, {self.target_heights[1]:.4f}, {self.target_heights[2]:.4f}")

    def smooth_adjustment(self):
         # Lower all arms together while keeping their offsets
        #self.target_heights = [max(h - self.step_size, 0.0) for h in self.target_heights]
        
        heights = np.array(self.target_heights)

        # Perform the comparison
        if np.any(heights > 0.16):
            self.get_logger().warn("Height exceceds 0.16m. Cannot land!")
        else:
            # Publish to servos
            msg1 = Float64()
            msg1.data = self.target_heights[0]
            self.a1_slider_pub.publish(msg1)

            msg2 = Float64()
            msg2.data = self.target_heights[1]
            self.a2_slider_pub.publish(msg2)

            msg3 = Float64()
            msg3.data = self.target_heights[2]
            self.a3_slider_pub.publish(msg3)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
