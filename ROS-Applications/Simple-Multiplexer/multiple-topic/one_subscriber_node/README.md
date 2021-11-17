## HOW TO RUN

* 1 - Create a package on your environment default directory. `ros2 pkg create --build-type ament_python py_simple_multiplexer`
* 2 - Add dependecies and entry points (initial configuration) to your package.xml. Update setup.py as well.
* 3 - Write 2 publishers and 1 subscriber that keeps only 1 subscriber node, that keeps changing the subscribing topic, by destroying the connectio.
* 4 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 5 - Run each publisher and the subscriber.

* OPTIONAL - If you want to set different timers for each publisher, check the commented code on the `subscriber` node related to the timers.
---

### Summary

You created two nodes to publish and one node to subscribe to data over two different topics. (Check each topic: `ros2 topic list`)
Before running them, you added their dependencies and entry points to the package configuration files.
