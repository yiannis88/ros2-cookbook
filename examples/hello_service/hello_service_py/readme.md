# hello_service_py
A minimal ROS2 python server/client (service) example.
**RELIABLE** and **VOLATILE** have been selected for the QOS_PROFILE, as with **BEST_EFFORT** we won't be able to call the service from a terminal and with **TRANSIENT_LOCAL** can cause a service to receive requests from clients that have since terminated. See [qos_profile](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

* Package created with `ros2 pkg create hello_service_py --build-type ament_python --dependencies rclpy example_interfaces`.
* The main scripts are `hello_service_py/hello_server.py` and `hello_service_py/hello_client.py`.
* Update the `entry_points >> console_scripts` in the `setup.py` file with
```
"hello_server = hello_service_py.hello_server:main",
"hello_client = hello_service_py.hello_client:main"
```
* Build the package from the workspace directory with `colcon build --packages-select hello_service_py --symlink-install`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_service_py hello_client` and `ros2 run hello_service_py hello_server`.