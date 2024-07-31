#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Robot(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Robot node started")

def main(args=None):
    rclpy.init()
    robot = Robot("robot")
    rclpy.spin(robot)
    rclpy.shutdown()

if __name__ == "__main__":
    main()