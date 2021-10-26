## HOW TO RUN

* 1 - Create a package on your environment default directory. `ros2 pkg create --build-type ament_cmake tutorial_interfaces`
* 2 - Create directories related to the interfaces you want to build (`mkdir msg`), and create interface files in those directories with the same extension of the parent dir `.msg`.
* 3 - Add dependecies to `CMakeLists.txt` and `package.xml`.
* 4 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 5 - Test the new interface created by adding dependecies to your `simple-multiplexer` package.

---

### Summary

In this tutorial, you learned how to create custom interfaces in their own package and how to utilize those interfaces from within other packages.
