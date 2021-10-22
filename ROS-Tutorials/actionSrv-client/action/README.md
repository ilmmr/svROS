## Creating an Action

* 1 - Set up a workspace and create a package named `action_tutorials_interfaces`
```
mkdir -p action_ws/src
cd action_ws/src
ros2 pkg create action_tutorials_interfaces
```

* 2 - Actions are defined in `.action` files of the form:
```
# Request - A request message is sent from an action client to an action server initiating a new goal.
---
# Result - A result message is sent from an action server to an action client when a goal is done.
---
# Feedback - Feedback messages are periodically sent from an action server to an action client with updates about a goal.
```

* 3 - Create an action directory in our ROS 2 package `action_tutorials_interfaces`

* 4 - Before we can use the new Fibonacci action type in our code, we must pass the definition to the `rosidl` code generation pipeline.
This is accomplished by adding the following lines to our `CMakeLists.txt` before the `ament_package()` line, in the `action_tutorials_interfaces`:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

* 5 - We should also add the required dependencies to our `package.xml`:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

* 6 - We should now be able to build the package containing the Fibonacci action definition:
```
cd ~/action_ws
colcon build
```
