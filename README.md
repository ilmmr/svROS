## Formalizing ROS2 security configuration with Alloy - svROS

<p align="center">
   <img width="200" height="182" src="https://upload.wikimedia.org/wikipedia/commons/9/93/EEUMLOGO.png">
</p>

This dissertation reports on academic work that can be used by third parties as long as the internationally accepted standards and good practices are respected concerning copyright and related rights. This work can thereafter be used under the terms established in the license below. Readers needing authorization conditions not provided for in the indicated licensing should contact the author through the RepositóriUM of the University of Minho.

---

### Introduction

The intended goal of this work is to propose a technique based on the software verification perspective, to automatically verify system-wide properties related to the security configuration of ROS2-based applications. This tool is later regarded as *Security Verification in ROS* (**[svROS](https://luis1ribeiro.github.io/svROS/)**).

To that purpose it will model the ROS architecture, as well as the network communication behaviour, in Alloy, a formal specification language and analysis tool supported by a model-finder, with which system-wide properties will subsequently model-checked.

---

### Content

* [Dissertation's Work Plan](./workplan.pdf)
* **January's Checkpoint:**
   * [RPD - Dissertation Checkpoint](./rpd-checkpoint.pdf)
   * [RPD - Presentation](./rpd-presentation.pdf)

---

### Contextualization

Industrial manufacturing is becoming highly reliant on automation developments, as they bring more efficient and accurate processes with less associated cost. Consequently, robots are increasingly being deployed in a wide range of scenarios, especially where safety is demanded. In such cases, it is critical to employ appropriate procedures to verify both the system's quality and safety.

Following the current growth of cyber-physical system, as well as their usage in various technology domains, the development of software applications is becoming more demanding due to the complexity behind the integration of needed services, beyond those provided by the operating system. Therefore, software middleware is increasingly used, since it offers services that support application development and delivery.

One of the most popular open-source software platforms for building robotic systems is the Robot Operating System (ROS) middleware, where highly configurable robots are usually built by composing third-party modules. A major factor behind its popularity and widespread adoption is its flexibility and interoperability. One drawback of this flexibility, however, lies in the increased security risks that ROS applications face. The emergence of performance and scalability challenges connected to the ROS middleware standard, in addition to security concerns, prompted the creation of ROS2.

Robot Operating System 2 (ROS2), which continues to provide a simple, uniform message passing interface, to allow components to communicate with each other, is implemented using the Data Distribution Service (DDS) communication protocol, where security guarantees are ensured by the DDS-Security specification. Using DDS-Security, it is possible to configure ROS2 to run with security guarantees using the SROS2 toolset.

This dissertation will propose a technique, based on the software verification perspective, to automatically verify system-wide properties related to the security configuration of ROS2-based applications. To that purpose it will model the ROS architecture, as well as the network communication behaviour, in Alloy, a formal specification language and analysis tool supported by a model-finder, with which system-wide properties will subsequently model-checked.

