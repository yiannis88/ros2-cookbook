# hello_lifecycle_py
A minimal ROS2 cpp lifecycle publisher. We set the states for the **lifecycle** node from the `/lifecycle_activate` service.

**BEST_EFFORT** and **VOLATILE** have been selected for the QOS_PROFILE, as the node keeps publishing at fixed intervals and losing messages it's fine. See [qos_profile](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

* If you want reliability change **BEST_EFFORT** to **RELIABLE**.
* If you want persisting samples for late-joining subscriptions, change **VOLATILE** to **TRANSIENT_LOCAL**.

* Package created with `ros2 pkg create hello_lifecycle_cpp --build-type ament_cmake --dependencies rclcpp example_interfaces rclcpp_lifecycle`.
* The main script is `src/hello_lifecycle_publisher.cpp`.
* Update the `CMakeLists.txt` to create the executable with its dependencies with
```
add_executable(hello_lifecycle_c_publisher src/hello_lifecycle_publisher.cpp)
ament_target_dependencies(hello_lifecycle_c_publisher rclcpp example_interfaces)

install(TARGETS
  hello_lifecycle_c_publisher
  DESTINATION lib/${PROJECT_NAME}
)
```
* Build the package from the workspace directory with `colcon build --packages-select hello_lifecycle_cpp`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_lifecycle_cpp hello_c_publisher`.


## Scenario A
Transition node state **unconfigured >> inactive >> active >> finalized**
* Run the node with `ros2 run hello_lifecycle_cpp hello_lifecycle_c_publisher`.
* Run `ros2 topic list`, the topic `hello_lifecycle` has been created but not publishing as the node is in **inactive** state.
* Run `ros2 service call /lifecycle_activate example_interfaces/srv/SetBool "{data: true}"`, the node is now in **active** state and you will see the messages coming through the topic.
* Run again the `ros2 service call /lifecycle_activate example_interfaces/srv/SetBool "{data: true}"` to shutdown the node (**finalized** state).

## Scenario B
Transition node state **unconfigured >> inactive >> active >> inactive >> unconfigured >> finalized**
* Run the node with `ros2 run hello_lifecycle_cpp hello_lifecycle_c_publisher`.
* Run `ros2 topic list`, the topic `hello_lifecycle` has been created but not publishing as the node is in **inactive** state.
* Run `ros2 service call /lifecycle_activate example_interfaces/srv/SetBool "{data: true}"`, the node is now in **active** state and you will see the messages coming through the topic.
* Run again the `ros2 service call /lifecycle_activate example_interfaces/srv/SetBool "{data: false}"` to set the node to **inactive** state.
* Run again the `ros2 service call /lifecycle_activate example_interfaces/srv/SetBool "{data: false}"` to set the node to **unconfigured** state.
* Run again the `ros2 service call /lifecycle_activate example_interfaces/srv/SetBool "{data: false}"` to set the node to **finalized** state.

**Note**
You can use the `<node>/transition_event` topic to track the state changes or to create the `~/lifecycle_state` and publish this information according to [managed_nodes](https://design.ros2.org/articles/node_lifecycle.html).
