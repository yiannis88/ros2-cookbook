#!/usr/bin/env python3

'''
    A minimal ROS2 python node that logs a message every second.

    Basic structure for ROS2 node:
    - Initialise rclpy
    - Create a node
    - Keep it alive with spin
    - Shutdown

    Author: Ioannis Selinis <selinis.g@gmail.com> 2025
'''


import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """The HelloNode class that inherits from rclpy.node.Node.
    Initialises with a counter and creates a timer that triggers
    once per second.
    """
    def __init__(self):
        super().__init__(node_name='hello_node')
        # vars
        self.counter = 0
        self.timer = self.create_timer(timer_period_sec=1.0,
                                       callback=self.timer_callback)
        self.get_logger().info(f'Node has started with counter: {self.counter}')


    def timer_callback(self):
        """Increment and log the counter once per second."""
        self.get_logger().info(f'Hello counter: {self.counter}')
        self.counter += 1


def main(args=None) -> None:
    """Entry Point for our node"""
    rclpy.init(args=args)   # Initialise rclpy (ROS communications)
    node = HelloNode()
    rclpy.spin(node=node)   # Keep node alive
    node.destroy_node()     # Destroy node
    rclpy.shutdown()        # Shutdown the executor



if __name__ == '__main__':
    main()
