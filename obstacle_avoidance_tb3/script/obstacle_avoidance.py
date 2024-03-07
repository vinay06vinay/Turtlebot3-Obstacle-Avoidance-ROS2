#!/usr/bin/env python3

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


