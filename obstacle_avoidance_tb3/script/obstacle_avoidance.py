#!/usr/bin/env python3

#
# Copyright (c) 2024, Vinay Bukka, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random


class Obstacle_Avoidance(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self._subscriber = self.create_subscription(LaserScan, "/scan", self.laser_scan_process, 10)
        self._timer = self.create_timer(1, self.publish_action)
        self._velocity_msg = Twist()
        self._move_forward_velocity = 0.0
        self._rotate_angle_velocity = 0.0

    def laser_scan_process(self,msg):
        message_range = msg.ranges
        field_range = 60
        initial_angle = 330
        obstacle_detected = False

        for i in range(initial_angle,initial_angle+field_range):
            if(message_range[i%360] < 0.75):
                obstacle_detected= True
                break
        if (obstacle_detected):
            self._rotate_angle_velocity = -0.3
            self._move_forward_velocity = 0.0
        else:
            self._rotate_angle_velocity = 0.0
            self._move_forward_velocity = 0.4

    
    def publish_action(self):
        self._velocity_msg.linear.x =  self._move_forward_velocity
        self._velocity_msg.angular.z = self._rotate_angle_velocity
        self._publisher.publish(self._velocity_msg)


def main(args=None):
    rclpy.init(args=args)  
    node = Obstacle_Avoidance("turtlebot_obstacle") 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()  


