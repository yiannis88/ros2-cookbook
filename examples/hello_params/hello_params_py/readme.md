# hello_params_py
A minimal ROS2 python params example. Two parameters are defined in the node, allowing the user to change their values on the fly.

* Package created with `ros2 pkg create hello_params_py --build-type ament_python --dependencies rclpy`.
* The main script is `hello_params_py/hello_params.py`.
* Update the `entry_points >> console_scripts` in the `setup.py` file with
```
"hello_params = hello_params_py.hello_params:main"
```
* Build the package from the workspace directory with `colcon build --packages-select hello_params_py --symlink-install`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_params_py hello_params`.
* List the parameters on the terminal with `ros2 param list`.
* Update a parameter from the terminal with `ro2 param set <node> <param> <value>`.
* Info about a parameter `ros2 param describe <node> <param>`.