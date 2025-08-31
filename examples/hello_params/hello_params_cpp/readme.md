# hello_params_cpp
A minimal ROS2 cpp params example. Two parameters are defined in the node, allowing the user to change their values on the fly.

* Package created with `ros2 pkg create hello_params_cpp --build-type ament_cmake --dependencies rclcpp`.
* The main script is `src/hello_params.cpp` and `src/hello_param.cpp` (using the `set_parameters()`).
* Update the `CMakeLists.txt` to create the executable with its dependencies with
```
add_executable(hello_c_params src/hello_params.cpp)
ament_target_dependencies(hello_c_params rclcpp)
install(TARGETS
  hello_c_params
  DESTINATION lib/${PROJECT_NAME}
)
```
* Build the package from the workspace directory with `colcon build --packages-select hello_params_cpp`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_params_cpp hello_c_params`.
* List the parameters on the terminal with `ros2 param list`.
* Update a parameter from the terminal with `ro2 param set <node> <param> <value>`.
* Info about a parameter `ros2 param describe <node> <param>`.