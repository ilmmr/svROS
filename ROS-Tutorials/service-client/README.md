## HOW TO RUN

* 1 - Create a package on your environment default directory. (The --dependencies argument will automatically add the necessary dependency lines to package.xml) `ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces`
* 2 - Update your package.xml and setup.py as well.
* 3 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 4 - Run the talker and the listener.

---

### Summary

You created two nodes to request and respond to data over a service.
You added their dependencies and executables to the package configuration files so that you could build and run them, allowing you to see a service/client system at work.

