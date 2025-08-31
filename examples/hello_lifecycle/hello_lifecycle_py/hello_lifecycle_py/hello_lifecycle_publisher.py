#!/usr/bin/env python3

'''
    A minimal ROS2 python node that publishes a string every second.

    Basic structure for ROS2 node publisher:
    - Initialise rclpy
    - Create a node
    - Create the publisher when in inactive state
    - Create a timer to publish at specific intervals when in active state
    - Keep it alive with spin
    - Shutdown

    From inactive to active, call the service lifecycle_activate with SetBool.Request true
    From active to inactive, call the service lifecycle_activate with SetBool.Request false

    BEST_EFFORT and VOLATILE have been selected for the QOS_PROFILE, as the
    node keeps publishing at fixed intervals and losing messages it's fine.

    * If you want reliability change BEST_EFFORT to RELIABLE.
    * If you want persisting samples for late-joining subscriptions,
      change VOLATILE to TRANSIENT_LOCAL.

    Author: Ioannis Selinis <selinis.g@gmail.com> 2025
'''

import sys
import asyncio

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from example_interfaces.msg import String
from example_interfaces.srv import SetBool


# https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,      # RELIABLE or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,           # VOLATILE or TRANSIENT_LOCAL
    history=HistoryPolicy.KEEP_LAST,                # KEEP_ALL or KEEP_LAST
    depth=1                                         # Used only with KEEP_LAST
)
QOS_PROFILE_SRV = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,         # RELIABLE or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,           # VOLATILE or TRANSIENT_LOCAL
    history=HistoryPolicy.KEEP_LAST,                # KEEP_ALL or KEEP_LAST
    depth=1                                         # Used only with KEEP_LAST
)
NODE_NAME = 'hello_lifecycle_publisher'


class HelloLifecyclePublisher(LifecycleNode):
    """The HelloLifecyclePublisher class that inherits from rclpy.node.Node.
    Initialises with a counter and a publisher and creates a timer
    that triggers once per second to publish a message.
    """

    def __init__(self):
        super().__init__(node_name=NODE_NAME)

        self.counter = 0
        self.cancel = False
        self.publisher = None
        self.timer = None
        self.server = None
        self.current_state = 'Unconfigured'

    def publisher_callback(self):
        """Publish the message once per second."""
        msg = String()
        msg.data = f'Lifecycle counter: {self.counter}'
        self.publisher.publish(msg=msg)
        self.counter += 1

    def service_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Put the node in activate or deactivate mode based on the request flag."""
        self.get_logger().info(f'Service Callback is triggered with {request.data}')

        response.success = True
        self.get_logger().info(f'Current state is {self.current_state}')
        if self.current_state.lower() == 'unconfigured':
            # state is unconfigured:
            # i) go to inactive if request.data is True
            # ii) go to finalized if request.data is False
            if request.data:
                response.message = 'Node configured'
                self.trigger_configure()
            else:
                response.message = 'Node finalized (shutdown)'
                self.trigger_shutdown()
        elif self.current_state.lower() == 'inactive':
            # state is inactive:
            # i) go to active if request.data is True
            # ii) go to unconfigured if request.data is False
            # iii) go to finalized (we don't consider it here due to SetBool states)
            if request.data:
                self.trigger_activate()
                response.message = 'Node activated'
            else:
                self.trigger_cleanup()
                response.message = 'Node unconfigured (cleaned up)'
        elif self.current_state.lower() == 'active':
            # state is active:
            # i) go to shutdown if request.data is True
            # ii) go to inactive if request.data is False
            if request.data:
                response.message = 'Node finalized (shutdown)'
                self.trigger_shutdown()
            else:
                response.message = 'Node deactivated'
                self.trigger_deactivate()
        return response

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Set the node state to configure (create the publisher)."""
        self.current_state = 'Inactive'
        self.get_logger().info(f'{NODE_NAME} transitions from {state.label} ({state.state_id}) to {self.current_state} - create the publisher & service')
        self.publisher = self.create_publisher(msg_type=String,
                                               topic='hello_lifecycle',
                                               qos_profile=QOS_PROFILE)
        self.server = self.create_service(srv_type=SetBool,
                                          srv_name='lifecycle_activate',
                                          callback=self.service_callback,
                                          qos_profile=QOS_PROFILE_SRV)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Set the node state to activate (create the timer for the publisher)."""
        self.current_state = 'Active'
        self.get_logger().info(f'{NODE_NAME} transitions from {state.label} ({state.state_id}) to {self.current_state} - create the timer')
        self.timer = self.create_timer(timer_period_sec=1.0,
                                       callback=self.publisher_callback)
        return super().on_activate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup state"""
        self.current_state = 'Unconfigured'
        self.get_logger().info(f'{NODE_NAME} transitions from {state.label} ({state.state_id}) to {self.current_state} -- cleanup')
        self.destroy_lifecycle_publisher(self.publisher)
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate node."""
        self.current_state = 'Inactive'
        self.get_logger().info(f'{NODE_NAME} transitions from {state.label} ({state.state_id}) to {self.current_state} -- deactivate timer')
        if self.timer:
            self.timer.cancel()
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown node."""
        self.current_state = 'Finalized'
        self.get_logger().info(f'{NODE_NAME} transitions from {state.label} ({state.state_id}) to {self.current_state} -- shutdown')
        self.destroy_lifecycle_publisher(self.publisher)
        self.destroy_timer(self.timer)
        self.cancel = True
        return super().on_shutdown(state)


async def spin(executor: SingleThreadedExecutor, node: HelloLifecyclePublisher):
    """Async spin."""
    while not node.cancel:
        executor.spin_once(timeout_sec=0.1)
        # Setting the delay to 0 provides an optimized path to allow other tasks to run.
        await asyncio.sleep(0)


def main(args=None) -> None:
    """Entry Point for our node"""
    try:
        rclpy.init(args=args)
        executor = SingleThreadedExecutor()
        node = HelloLifecyclePublisher()
        executor.add_node(node=node)
        # Node state: UNCONFIGURED --> INACTIVE
        node.trigger_configure()

        loop = asyncio.get_event_loop()
        loop.run_until_complete(spin(executor=executor, node=node))
    except Exception as err:  # pylint: disable=broad-except
        node.get_logger().error(f'Main loop crashed: {err}')
    finally:
        node.get_logger().error(f'Shutting down {NODE_NAME}')
        node.destroy_node()
        rclpy.shutdown()
        loop.close()
        sys.exit(0)


if __name__ == '__main__':
    main()
