#!/usr/bin/env python3

'''
    A minimal ROS2 python node to get/set params (using the self.set_parameters)

    Basic structure for ROS2 node publisher:
    - Initialise rclpy
    - Create a node
    - Declare the params
    - Keep it alive with spin
    - Shutdown

    Author: Ioannis Selinis <selinis.g@gmail.com> 2025
'''


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult


class HelloParams(Node):
    """The HelloParams class that inherits from rclpy.node.Node.
    Initialises with the params.
    """
    def __init__(self):
        super().__init__(node_name='hello_params')

        # declare_parameters to declare multiple ones
        # use declare_parameter to declare a single
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('timer_interval', 1.0, ParameterDescriptor(description='Set the interval for the timer',
                                                                                floating_point_range=[FloatingPointRange(from_value=1.0,
                                                                                                                         to_value=13.0,
                                                                                                                         step=2.0)],
                                                                                additional_constraints='Range [1.0, 13.0] Step 2.0')),
                                    ('hello_msg', 'Orcheas', ParameterDescriptor(description='The msg name to print'))
                                ])

        # add a callback to update on the fly
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.timer = self.create_timer(timer_period_sec=self.get_parameter('timer_interval').value, callback=self.print_message)


    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Callback for setting params on the fly"""
        for param in params:
            if param.name == 'timer_interval':
                if self.timer:
                    self.timer.cancel()
                self.timer = self.create_timer(timer_period_sec=param.value, callback=self.print_message)
        return SetParametersResult(successful=True)


    def print_message(self):
        """Print a message based on the timer"""
        self.get_logger().info(f'Hello from {self.get_parameter("hello_msg").value} (interval={self.get_parameter("timer_interval").value})')
        self.set_parameters([Parameter('hello_msg', Parameter.Type.STRING, 'zeus')])


def main(args=None) -> None:
    """Entry Point for our node"""
    rclpy.init(args=args)   # Initialise rclpy (ROS communications)
    node = HelloParams()
    rclpy.spin(node=node)   # Keep node alive
    node.destroy_node()     # Destroy node
    rclpy.shutdown()        # Shutdown the executor


if __name__ == '__main__':
    main()
