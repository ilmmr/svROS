## Summary

The idea behind this mini-project is to consider the turtlesim example with a multiplexer that manages 2 different topics that the turtle must be synchronized with. Timers are used to give priority alternately to each topic.
These topics will be called 'low\_priority' and 'high\_priority' respectively, and the low priority one will get less time to execute than the high priority one, thanks to the multiplexer.

ROS2 provides a useful way to launch multiple nodes via a launch file. This file is specified in `turtle_launch.py`, however has some flaws, since this method doesn't furnish a way of launching nodes in a new terminal. This technique is needed so the keyboard node is initialized in a seperated terminal, so that the input of the user is read.
For that, a shell script has been created to fulfill this need. To run, execute the following command in your terminal:
```
./turtle_launch
```

For understanding reasons, it was created 3 packages, one for the turtlesim node, one for the multiplexer and other for the random controller (untrusted).
