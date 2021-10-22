## HOW TO RUN

* 1 - You will need the `action_tutorials_interfaces` package and the `Fibonacci.action` interface.
* 2 - Create a package on your environment default directory. (The --dependencies argument will automatically add the necessary dependency lines to package.xml). You can also write a single Python file instead.
* 3 - Update your package.xml and setup.py as well.
* 4 - Build and run using colcon - `colcon build --packages-select <package name>` and source the setup files on your bash/zsh config file.
* 5 - Run the action server and the client.

---

### Summary

In this tutorial, you put together a Python action server and action client line by line, and configured them to exchange goals, feedback, and results.
