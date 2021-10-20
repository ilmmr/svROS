# Formalizing ROS2 security configuration with Alloy

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

---

## References

* [1] https://www.ros.org

* [2] Nicholas DeMarinis, Stefanie Tellex, Vasileios P. Kemerlis, George Dimitri Konidaris,
Rodrigo Fonseca: Scanning the Internet for ROS: A View of Security in Robotics Research. ICRA
2019: 8514-8521

* [3] Object Management Group. Data Distribution Service (DDS). https://www.omg.org/omg-dds-
portal/

* [4] ROS 2 DDS-Security integration https://design.ros2.org/articles/ros2_dds_security.html

* [5] Renato Carvalho, Alcino Cunha, Nuno Macedo, André Santos: Verification of system-wide
safety properties of ROS applications. IROS 2020: 7249-7254
https://www.autoware.auto

* [6] Shinpei Kato, Shota Tokunaga, Yuya Maruyama, Seiya Maeda, Manato Hirabayashi, Yuki
Kitsukawa, Abraham Monrroy, Tomohito Ando, Yusuke Fujii, Takuya Azumi: Autoware on
board: enabling autonomous vehicles with embedded systems. ICCPS 2018: 287-296
https://www.autoware.auto

* [7] Daniel Jackson: Alloy: a language and tool for exploring software designs. Commun. ACM
62(9): 66-76 (2019), http://alloytools.org/

* [8] Nuno Macedo, Julien Brunel, David Chemouil, Alcino Cunha, Denis Kuperberg: Lightweight
specification and analysis of dynamic systems with rich configurations. SIGSOFT FSE 2016: 373-
383, https://haslab.github.io/formal-software-design/
