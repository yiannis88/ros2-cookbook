#!/usr/bin/env python3

'''
    A minimal ROS2 python node that creates a service.

    Basic structure for ROS2 node publisher:
    - Initialise rclpy
    - Create a node
    - Create the server
    - Keep it alive with spin
    - Shutdown

    RELIABLE and VOLATILE have been selected for the QOS_PROFILE. The reason is with BEST_EFFORT
    is that we won't be able to call the service from the terminal (CLI) and with TRANSIENT_LOCAL
    that can cause the service servers to receive requests from clients that have since terminated.

    Author: Ioannis Selinis <selinis.g@gmail.com> 2025
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from example_interfaces.srv import AddTwoInts


# https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,         # RELIABLE or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,           # VOLATILE or TRANSIENT_LOCAL
    history=HistoryPolicy.KEEP_LAST,                # KEEP_ALL or KEEP_LAST
    depth=1                                         # Used only with KEEP_LAST
)


class HelloServer(Node):
    """The HelloServer class that inherits from rclpy.node.Node.
    Initialises with server/service that returns the sum of two ints.
    """
    def __init__(self):
        super().__init__(node_name='hello_server')

        self.server = self.create_service(srv_type=AddTwoInts,
                                          srv_name='add_two_ints',
                                          callback=self.service_callback,
                                          qos_profile=QOS_PROFILE)


    def service_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        """The callback service to return the sum of two integers.

        Keyword arguments:
        request (AddTwoInts.Request) -- The request with the two integers
        response (AddTwoInts.Response) -- The response with the sum of the integers

        Return:
        AddTwoInts.Response -- The sum of the integers
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received with {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None) -> None:
    """Entry Point for our node"""
    rclpy.init(args=args)   # Initialise rclpy (ROS communications)
    node = HelloServer()
    rclpy.spin(node=node)   # Keep node alive
    node.destroy_node()     # Destroy node
    rclpy.shutdown()        # Shutdown the executor


if __name__ == '__main__':
    main()
