# hello_pubsub
A minimal ROS2 cpp publisher and subscriber. The message published is an `example_interfaces/msg/String`, hence the dependency on the `example_interfaces`.

**BEST_EFFORT** and **VOLATILE** have been selected for the QOS_PROFILE, as the node keeps publishing at fixed intervals and losing messages it's fine. See [qos_profile](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

* If you want reliability change **BEST_EFFORT** to **RELIABLE**.
* If you want persisting samples for late-joining subscriptions, change **VOLATILE** to **TRANSIENT_LOCAL**.

* Package created with `ros2 pkg create hello_pubsub_cpp --build-type ament_cmake --dependencies rclcpp example_interfaces`.
* The main scripts are `src/hello_publisher.cpp` and `src/hello_subscriber.cpp`.
* Update the `CMakeLists.txt` to create the executable with its dependencies with
```
add_executable(hello_c_publisher src/hello_publisher.cpp)
ament_target_dependencies(hello_c_publisher rclcpp example_interfaces)

add_executable(hello_c_subscriber src/hello_subscriber.cpp)
ament_target_dependencies(hello_c_subscriber rclcpp example_interfaces)

install(TARGETS
  hello_c_publisher
  hello_c_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
```
* Build the package from the workspace directory with `colcon build --packages-select hello_pubsub_cpp`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_pubsub_cpp hello_c_subscriber` and `ros2 run hello_pubsub_cpp hello_c_publisher`.
