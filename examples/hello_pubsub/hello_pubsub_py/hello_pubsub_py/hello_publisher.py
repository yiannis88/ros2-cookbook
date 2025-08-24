#!/usr/bin/env python3

'''
    A minimal ROS2 python node that publishes a string every second.

    Basic structure for ROS2 node publisher:
    - Initialise rclpy
    - Create a node
    - Create the publisher
    - Create a timer to publish at specific intervals
    - Keep it alive with spin
    - Shutdown

    BEST_EFFORT and VOLATILE have been selected for the QOS_PROFILE, as the
    node keeps publishing at fixed intervals and losing messages it's fine.

    * If you want reliability change BEST_EFFORT to RELIABLE.
    * If you want persisting samples for late-joining subscriptions,
      change VOLATILE to TRANSIENT_LOCAL.

    Author: Ioannis Selinis <selinis.g@gmail.com> 2025
'''


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from example_interfaces.msg import String


# https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,      # RELIABLE or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,           # VOLATILE or TRANSIENT_LOCAL
    history=HistoryPolicy.KEEP_LAST,                # KEEP_ALL or KEEP_LAST
    depth=1                                         # Used only with KEEP_LAST
)


class HelloPublisher(Node):
    """The HelloPublisher class that inherits from rclpy.node.Node.
    Initialises with a counter and a publisher and creates a timer
    that triggers once per second to publish a message.
    """
    def __init__(self):
        super().__init__(node_name='hello_publisher')

        self.counter = 0
        # ~/hello_world would publish it as /hello_publisher/hello_world
        self.publisher = self.create_publisher(msg_type=String,
                                               topic='hello_world',
                                               qos_profile=QOS_PROFILE)
        self.timer = self.create_timer(timer_period_sec=1.0,
                                       callback=self.publisher_callback)


    def publisher_callback(self):
        """Publish the message once per second."""
        msg = String()
        msg.data = f'Hello counter: {self.counter}'
        self.publisher.publish(msg=msg)
        self.counter += 1


def main(args=None) -> None:
    """Entry Point for our node"""
    rclpy.init(args=args)   # Initialise rclpy (ROS communications)
    node = HelloPublisher()
    rclpy.spin(node=node)   # Keep node alive
    node.destroy_node()     # Destroy node
    rclpy.shutdown()        # Shutdown the executor


if __name__ == '__main__':
    main()
