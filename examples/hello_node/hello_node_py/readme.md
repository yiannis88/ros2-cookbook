# hello_node_py
A minimal ROS2 python node that logs a message every second.

* Package created with `ros2 pkg create hello_node_py --build-type ament_python --dependencies rclpy`.
* The main script is `hello_node_py/hello_node.py`.
* Update the `entry_points >> console_scripts` in the `setup.py` file with `"hello_node = hello_node_py.hello_node:main"`.
* Build the package from the workspace directory with `colcon build --packages-select hello_node_py --symlink-install`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_node_py hello_node`.