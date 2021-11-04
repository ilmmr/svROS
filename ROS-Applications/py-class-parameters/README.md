## HOW TO RUN

* 1 - Create a package on your environment default directory. (The --dependencies argument will automatically add the necessary dependency lines to package.xml) `ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy`
* 2 - Update your package.xml and setup.py as well.
* 3 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 4 - Run the parameter talker.
* 5 (Optional) - To change the parameter simply run the following line in the console: `ros2 param set /minimal_param_node my_parameter earth`
* 6 (Optional) - If you do declare a launch file, its needed that you create a directory for that purpose and define your Python code there related to the launch file. Then add this to your `setup.py` file.
`data_files=[
  # ...
  (os.path.join('share', package_name), glob('launch/*_launch.py')),
]`

---

### Background

When making your own nodes you will sometimes need to add parameters that can be set from the launch file.

---

### Summary

You created a node with a custom parameter, that can be set either from the launch file or the command line.
You wrote the code of a parameter talker: a Python node that declares, and then loops getting and setting a string parameter.
You added the entry point so that you could build and run it, and used ros2 param to interact with the parameter talker.
