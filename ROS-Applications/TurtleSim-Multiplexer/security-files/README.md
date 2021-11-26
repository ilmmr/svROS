## Folder Description

This folder contains files to enable SROS2 features.
* The script provides exports of SROS2 environment variables, that enables security matters on the ROS2 network.
* The folder `turtlesim_keys` contains all the SROS2 artifacts that RCL uses as source to enable security on the network.

## ROS Security Enclaves

### Introduction
n summary, all secure processes must use an enclave that contains the runtime security artifacts unique to that enclave, yet each process may not necessarily have a unique enclave. Multiple enclaves can be encapsulated in a single security policy to accurately model the information flow control.

### Concepts

**Participant** is the object representing a single entity on the network. In the case of DDS, the Participant is a DDS DomainParticipant, which has both access control permissions and a security identity.
