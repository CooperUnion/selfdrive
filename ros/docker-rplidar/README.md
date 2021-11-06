```
n1mda@nyquist ~/Documents/igvc/selfdrive/ros/docker-rplidar (TECH-15?) $ docker logs master 
... logging to /root/.ros/log/258cd2cc-3eaa-11ec-8631-0242ac150002/roslaunch-3f2ef5c2d7bf-1.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

invalid ROS_IP (must be a valid IPv4 or IPv6 address)
invalid ROS_IP (must be a valid IPv4 or IPv6 address)
invalid ROS_IP (must be a valid IPv4 or IPv6 address)
started roslaunch server http://master:44619/
ros_comm version 1.15.13


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.13

NODES

auto-starting new master
process[master]: started with pid [32]
invalid ROS_IP (must be a valid IPv4 or IPv6 address)
invalid ROS_IP (must be a valid IPv4 or IPv6 address)
ROS_MASTER_URI=http://master:11311/

setting /run_id to 258cd2cc-3eaa-11ec-8631-0242ac150002
process[rosout-1]: started with pid [42]
started core service [/rosout]

```
```
n1mda@nyquist ~/Documents/igvc/selfdrive/ros/docker-rplidar (TECH-15?) $ docker logs rplidar
... logging to /root/.ros/log/258cd2cc-3eaa-11ec-8631-0242ac150002/roslaunch-47e75703b74e-1.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

invalid ROS_IP (must be a valid IPv4 or IPv6 address)
invalid ROS_IP (must be a valid IPv4 or IPv6 address)
started roslaunch server http://rplidar:38089/

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.13
 * /rplidarNode/angle_compensate: True
 * /rplidarNode/frame_id: laser
 * /rplidarNode/inverted: False
 * /rplidarNode/serial_baudrate: 115200
 * /rplidarNode/serial_port: /dev/ttyUSB0

NODES
  /
    rplidarNode (rplidar_ros/rplidarNode)

ROS_MASTER_URI=http://master:11311

process[rplidarNode-1]: started with pid [50]
[ INFO] [1636166109.996683941]: RPLIDAR running on ROS package rplidar_ros, SDK Version:2.0.0
[ INFO] [1636166110.064593101]: RPLIDAR S/N: B5D699F6C9E59AD4C5E59CF749043414
[ INFO] [1636166110.064687838]: Firmware Ver: 1.29
[ INFO] [1636166110.064713827]: Hardware Rev: 7
[ INFO] [1636166110.116009102]: RPLidar health status : OK.
[ INFO] [1636166110.351687798]: current scan mode: Sensitivity, sample rate: 8 Khz, max_distance: 12.0 m, scan frequency:10.0 Hz,
```