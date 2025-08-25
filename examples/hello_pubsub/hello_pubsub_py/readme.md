# hello_pubsub_py
A minimal ROS2 python publisher and subscriber. The message published is an `example_interfaces/msg/String`, hence the dependency on the `example_interfaces`.

**BEST_EFFORT** and **VOLATILE** have been selected for the QOS_PROFILE, as the node keeps publishing at fixed intervals and losing messages it's fine. See [qos_profile](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

* If you want reliability change **BEST_EFFORT** to **RELIABLE**.
* If you want persisting samples for late-joining subscriptions, change **VOLATILE** to **TRANSIENT_LOCAL**.

* Package created with `ros2 pkg create hello_pubsub_py --build-type ament_python --dependencies rclpy example_interfaces`.
* The main scripts are `hello_pubsub_py/hello_publisher.py` and `hello_pubsub_py/hello_subscriber.py`.
* Update the `entry_points >> console_scripts` in the `setup.py` file with
```
"hello_publisher = hello_pubsub_py.hello_publisher:main",
"hello_subscriber = hello_pubsub_py.hello_subscriber:main"
```
* Build the package from the workspace directory with `colcon build --packages-select hello_pubsub_py --symlink-install`.
* Source with `source ~/.zshrc` or `source /.bashrc`.
* Run the node with `ros2 run hello_pubsub_py hello_subscriber` and `ros2 run hello_pubsub_py hello_publisher`.