## Summary

The idea behind this mini-project is to consider the turtlesim example with a multiplexer that manages 2 different toppics that the turtle must be synchronized with. Timers are used to give priority alternately to each topic.
These topics will be called 'low\_priority' and 'high\_priority' respectively, and the low priority one will get less time to execute than the high priority one, thanks to the multiplexer.

For understanding reasons, it was created 3 packages, one for the turtlesim node, one for the multiplexer and other for the random controller (untrusted).
