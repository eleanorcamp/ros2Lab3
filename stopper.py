# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Eleanor Camp
# Feb 11, 2025
# Lab 2 - making a robot stop .5m away from an object

import rclpy
from rclpy.node import Node

# the velcoity command message
from geometry_msgs.msg import Twist
# the base_scan data
from sensor_msgs.msg import LaserScan


class Stopper(Node):
    def __init__(self):
        super().__init__('stopper')

        # Publisher for the move data
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to the laser scan data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'base_scan',
            self.laser_callback,
            10
        )

        self.declare_parameter('stop_distance', .5)
        self.stop_distance = self.get_parameter('stop_distance').value

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.obstacle_detected = False  # Flag to track if an obstacle is detected

    def laser_callback(self, msg):
        # Laser scan data is a list of range readings at different angles
        # We'll check the front-facing value of the laser scan data
        front_value = msg.ranges[len(msg.ranges)//2]

        # If reading is below 0.5 meters (50 cm), stop the robot
        if front_value < self.stop_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        
        self.get_logger().info(f"Minimum distance from obstacle: {front_value} meters")

    def timer_callback(self):
        msg = Twist()

        if self.obstacle_detected:
            # If an obstacle is detected, stop the robot
            msg.linear.x = 0.0
        else:
            # Otherwise, drive forward at a constant speed
            msg.linear.x = 0.25

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    mover = Stopper()

    rclpy.spin(mover)

    # Destroy the node explicitly (optional)
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
