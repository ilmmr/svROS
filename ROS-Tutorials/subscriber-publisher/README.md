## HOW TO RUN

* 1 - Create a package on your environment default directory. `ros2 pkg create --build-type ament_python py_pubsub`
* 2 - Add dependecies and entry points (initial configuration) to your package.xml. Update setup.py as well.
* 3 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 4 - Run the talker and the listener.

---

### Summary

You created two nodes to publish and subscribe to data over a topic.
Before running them, you added their dependencies and entry points to the package configuration files.
