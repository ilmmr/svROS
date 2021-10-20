# Thesis

<p align="center">
   <img width="200" height="182" src="https://upload.wikimedia.org/wikipedia/commons/9/93/EEUMLOGO.png">
</p>

---

## Thesis Contextualization

One of the most popular open-source software platforms for building robotic systems is the Robot
Operating System (ROS) [1]. A major factor behind its popularity and widespread adoption is its
flexibility and interoperability.

One drawback of this flexibility, however, lies in the increased security risks that ROS applications
face. The low barrier to entry and open nature of the ROS ecosystem means a malicious actor could
potentially inject code or vulnerabilities into a library, which could then be reused by another
unsuspecting developer.

The main reason to this lack of security that ROS faces is the fact that they originally provided their
own specified middleware, which also didn’t scale well, making it unsuitable for safety-critical and
real-time systems, and that lead to the creation of ROS2.

ROS2 is deployed without security mechanisms by default, but it uses the Data Distribution Service
(DDS) [3] communication protocol, which can provide security guarantees such as authentication
and access control with a variant called DDS-Security. Using DDS-Security it is possible to
configure ROS2 to run with security guarantees using the SROS2 toolset [4].

ROS2 continues to provide a simple, uniform message passing interface to allow components to
communicate with each other, meaning it’s relatively straightforward for a developer to add and
integrate a new component into an existing system.
