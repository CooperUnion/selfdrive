## https://github.com/TRECVT/ros_kvh1750.git (indigo) - 1.0.0-0

The packages in the `https://github.com/TRECVT/ros_kvh1750.git` repository were released into the `indigo` distro by running `/usr/bin/bloom-release --ros-distro indigo --track indigo --edit https://github.com/TRECVT/ros_kvh1750.git` on `Tue, 15 Mar 2016 14:00:14 -0000`

The `kvh1750` package was released.

Version of package(s) in repository `https://github.com/TRECVT/ros_kvh1750.git`:

- upstream repository: https://github.com/jasedit/ros_kvh1750.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.0.0-0`

Versions of tools used:

- bloom version: `0.5.21`
- catkin_pkg version: `0.2.10`
- rosdep version: `0.11.4`
- rosdistro version: `0.4.5`
- vcstools version: `0.1.38`


# Introduction

This ROS package provides an interface to interface with the [KVH 1750 IMU](http://www.kvh.com/Military-and-Government/Gyros-and-Inertial-Systems-and-Compasses/Gyros-and-IMUs-and-INS/IMUs/1750-IMU.aspx), with support for a hardware time-of-validity signal using serial, and a plugin interface for supporting alternate publishing schemes. These are used to support the message types for MIT's [Pronto](https://github.com/ipab-slmc/pronto-distro) state estimation library.

# Configuration

1. `link_name` - link IMU is associated with
2. `processor_type` - string representing name of a [pluginlib](http://wiki.ros.org/pluginlib) plugin to load for processing processing IMU data. This should be based on the MessageProcessorBase class in the repository.
3. `rate` - Frequency to set the IMU reporting data.
4. `use_delta_angles` - sets whether the IMU reports delta angles (change in angles between readings) or angular rates.
5. `priority` - Priority for this node to be inserted into the scheduler.
6. `use_rt` - Flag to indicate if the node should be inserted into the round-robin scheduler.
7. `orientation_covariance` - Array of orientation covariances inserted into each IMU message.
8. `angular_covariance` - Array of angular covariances inserted into each IMU message.
9. `linear_covariance` - Array of linear covariances inserted into each IMU message.
10. `address` - The file to open for reading the IMU.
11. `tov_address` - File to open for reading the time-of-validity signal. This uses the [serial](http://wiki.ros.org/serial) library to perform the reading.
12. `baudrate` - Baud rate at which to read the IMU. *NOTE:* This must match the setting on the IMU, or else communication cannot be established.
13. `max_temp` - temperature at which to stop reading the IMU, as a safety feature.

# Time of Validity

In order to receive time-of-validity, this package requires that a serial message is sent on the `tov_address` file descriptor. The contents of the message is irrelevant - an empty message is preferred for accuracy and reducing system latency.

# Contributing

1. Fork this repository, make suggested changes.
2. If the change is minor (e.g. can be read and understood in a short period of time, does not require much discussion) issue a pull request.
3. If the change is more significant or requires discussion, open an issue explaining the issue and current thoughts. Open a pull request demonstrating a proposed solution, if one exists.
4. Pull requests should be reviewed by a maintainer, and handled as appropriate.