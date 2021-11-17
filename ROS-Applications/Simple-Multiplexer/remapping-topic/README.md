## HOW TO RUN

* 1 - Create a package on your environment default directory. `ros2 pkg create --build-type ament_python py_simple_multiplexer`
* 2 - Add dependecies and entry points (initial configuration) to your package.xml. Update setup.py as well.
* 3 - Write 2 publishers, 1 subscriber and 1 multiplexer that subscribes 2 different topics. When the publisher is publishing, a remap within topics is made, so that the multiplexer that is subscried to a 'low\_priority' topic and a 'high\_priority' topic, can receive information from the publishers. Then it publish through a new topic that the subscriber has to subscribe and remap.
* 4 - In order to manage these remaps, a `launch file` must be created. Check the file in `launch/pub_sub_remap.py`.
* 4 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 5 - Run each publisher and the subscriber.

