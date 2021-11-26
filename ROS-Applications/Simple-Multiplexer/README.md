## Folder Description

This folder has useful ROS-Applications using a simple multiplexer node over a topic subscriber-publisher technique:
* **multiple-topic:** This project considers the existence of different topic, and the multiplexer subscribes to each one of them.
* **remapping-topic:** This project uses the technique of **remapping** to map the topic that should be considered at each time.
* **same-topic**: Here, the multiplexer only subscribes to a topic namespace, where multiple publishers are publishing to. The multiplexer must address which and how many publishers are in the network.
