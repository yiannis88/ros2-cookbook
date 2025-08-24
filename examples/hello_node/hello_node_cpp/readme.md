# hello_node_cpp
A minimal ROS2 cpp node that logs a message every second.

* Package created with `ros2 pkg create hello_node_cpp --build-type ament_cmake --dependencies rclcpp`.
* The main script is `src/hello_node.cpp`.
* Update the `CMakeLists.txt` to create the executable with its dependencies with
```
add_executable(hello_c_node src/hello_node.cpp)
ament_target_dependencies(hello_c_node rclcpp)
install(TARGETS
  hello_c_node
  DESTINATION lib/${PROJECT_NAME}
)
```
* Build the package from the workspace directory with `colcon build --packages-select hello_node_cpp`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_node_cpp hello_c_node`.