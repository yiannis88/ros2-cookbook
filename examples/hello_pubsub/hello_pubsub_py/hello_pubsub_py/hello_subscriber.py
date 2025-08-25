#!/usr/bin/env python3

'''
    A minimal ROS2 python node that subscribes to a topic.

    Basic structure for ROS2 node publisher:
    - Initialise rclpy
    - Create a node
    - Create the subscriber
    - Keep it alive with spin
    - Shutdown

    Since the publisher node uses BEST_EFFORT and VOLATILE, the subscriber
    has to use the same policies.

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


class HelloSubscriber(Node):
    """The HelloSubscriber class that inherits from rclpy.node.Node.
    Initialises with a subscriber.
    """
    def __init__(self):
        super().__init__(node_name='hello_subscriber')

        self.subscriber = self.create_subscription(msg_type=String,
                                                   topic='hello_world',
                                                   callback=self.hello_world_callback,
                                                   qos_profile=QOS_PROFILE)


    def hello_world_callback(self, msg: String) -> None:
        """The callback of the hello_world topic

        Keyword arguments:
        msg (String) -- The ROS2 message
        """
        self.get_logger().info(f'Received message: {msg.data}')


def main(args=None) -> None:
    """Entry Point for our node"""
    rclpy.init(args=args)   # Initialise rclpy (ROS communications)
    node = HelloSubscriber()
    rclpy.spin(node=node)   # Keep node alive
    node.destroy_node()     # Destroy node
    rclpy.shutdown()        # Shutdown the executor


if __name__ == '__main__':
    main()
