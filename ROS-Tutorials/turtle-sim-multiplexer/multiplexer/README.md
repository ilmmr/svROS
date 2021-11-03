**This folder contains:**
* Multiplexer node that interpolates to different publishers, the **keyboard** node and the **random controller** node, by assigning timmers to each one and rulling them over a boolean. Then it remaps the topic to the **turtle sim** node.
-- The boolean that is initially set to False, becomes True when the keyboard sends an input through a topic that the multiplexer is subscribed to, and assings a timer to listen to its topic. After the timer is finished, the boolean is set to False and random controller takes charge.
