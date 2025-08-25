# hello_service_cpp
A minimal ROS2 cpp server/client (service) example.
**RELIABLE** and **VOLATILE** have been selected for the QOS_PROFILE, as with **BEST_EFFORT** we won't be able to call the service from a terminal and with **TRANSIENT_LOCAL** can cause a service to receive requests from clients that have since terminated. See [qos_profile](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

* Package created with `ros2 pkg create hello_service_cpp --build-type ament_cmake --dependencies rclcpp example_interfaces`.
* The main scripts are `src/hello_server.cpp` and `src/hello_client.cpp`.
* Update the `CMakeLists.txt` to create the executable with its dependencies with
```
add_executable(hello_c_server src/hello_server.cpp)
ament_target_dependencies(hello_c_server rclcpp example_interfaces)

add_executable(hello_c_client src/hello_client.cpp)
ament_target_dependencies(hello_c_client rclcpp example_interfaces)

install(TARGETS
  hello_c_server
  hello_c_client
  DESTINATION lib/${PROJECT_NAME}
)
```
* Build the package from the workspace directory with `colcon build --packages-select hello_service_cpp`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_service_cpp hello_c_client` and `ros2 run hello_service_cpp hello_c_server`.