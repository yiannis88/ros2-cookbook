#!/usr/bin/env python3

'''
    A minimal ROS2 python node that subscribes to a service.

    Basic structure for ROS2 node publisher:
    - Initialise rclpy
    - Create a node
    - Create the client for the service
    - Keep it alive with spin
    - Shutdown

    RELIABLE and VOLATILE have been selected for the QOS_PROFILE to match with
    the ones defined in the service server.

    Author: Ioannis Selinis <selinis.g@gmail.com> 2025
'''

import random
from functools import partial  # partial to pass args in add_done_callback
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from example_interfaces.srv import AddTwoInts


# https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,         # RELIABLE or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,           # VOLATILE or TRANSIENT_LOCAL
    history=HistoryPolicy.KEEP_LAST,                # KEEP_ALL or KEEP_LAST
    depth=1                                         # Used only with KEEP_LAST
)


class HelloClient(Node):
    """The HelloClient class that inherits from rclpy.node.Node.
    Initialises with the client for the service AddTwoInts.
    """
    def __init__(self):
        super().__init__(node_name='hello_client')
        self.client = self.create_client(srv_type=AddTwoInts,
                                         srv_name='add_two_ints',
                                         qos_profile=QOS_PROFILE)
        # Calling a non existing service doesn't throw an error,
        # but it's good to check if it's up or not.
        while not self.client.wait_for_service(1.0):
            self.get_logger().warning('Waiting for the service add_two_ints...')
        self.timer = self.create_timer(timer_period_sec=1.0,
                                       callback=self.call_service)


    def call_service(self) -> None:
        """Triggered every second and calls the service"""
        request = AddTwoInts.Request()
        request.a = random.randint(5, 35)
        request.b = random.randint(2, 102)
        future = self.client.call_async(request=request)
        future.add_done_callback(partial(self.service_callback, request=request))


    def service_callback(self, future: Future, request: AddTwoInts.Request) -> None:
        """Callback for the service call"""
        self.get_logger().info(f'Result: {future.result()} for {request.a} + {request.b}')


def main(args=None) -> None:
    """Entry Point for our node"""
    rclpy.init(args=args)   # Initialise rclpy (ROS communications)
    node = HelloClient()
    rclpy.spin(node=node)   # Keep node alive
    node.destroy_node()     # Destroy node
    rclpy.shutdown()        # Shutdown the executor


if __name__ == '__main__':
    main()
