# ROS Environment
installing docker
https://www.stereolabs.com/docs/docker/install-guide-linux/#nvidia-docker

libegl issue
https://www.stereolabs.com/docs/ros/

rviz gazebo
https://answers.ros.org/question/300113/docker-how-to-use-rviz-and-gazebo-from-a-container/

hardware accel docker
http://wiki.ros.org/action/login/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2

nvidia driver
https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html

fixing nvidia driver shit
https://askubuntu.com/questions/1280205/problem-while-installing-cuda-toolkit-in-ubuntu-18-04

libgl error
https://unix.stackexchange.com/questions/589236/libgl-error-no-matching-fbconfigs-or-visuals-found-glxgears-error-docker-cu

## Allow X server
```bash
xhost +si:localuser:root
```

## Install nvidia docker
```bash
# https://www.stereolabs.com/docs/docker/install-guide-linux/#nvidia-docker
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Test RPLidar
```
docker-compose up -d master rplidar
```

### ROS environment install

ROS environment install:

http://wiki.ros.org/noetic/Installation/Ubuntu
- make sure you source ROS environment
```bash
source /opt/ros/noetic/setup.zsh # or setup.bash
```
verify it worked by running 
```
printenv | grep ROS
```

## ROS environment install pt2
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
- make sure you source catkin workspace
```bash
catkin_make
```
```
# inside catkin_ws
source devel/setup.zsh # or setup.bash
```
verify it worked by running 
```
echo $ROS_PACKAGE_PATH
# you should get something like {your-catkin-path}/catkin_ws/src:/opt/ros/noetic/share
```

### CARLA ROS Bridge install
https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/
use the source repository


### Gazebo install
http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install


## Gazebo + ROS integration
http://gazebosim.org/tutorials?tut=ros_installing


### TurtleBot3
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

### Dependencies
[ISSUE]: TurtleBot3 dependency installation command is faulty  
[FIX]: Add single quotes around ros-noetic-rqt*  

```
$ sudo apt update
$ sudo apt upgrade
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport 'ros-noetic-rqt*' ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

### Gazebo Integration 
[ISSUE]: Missing `zed_nodelets` dependency in `./zed-ros-wrapper/zed_wrapper/package.xml`; comes out when running `catkin_make`   
[FIX]: Add `<depend>zed_nodelets</depend>` in `./zed-ros-wrapper/zed_wrapper/package.xml`  

**Notes:** When exporting the turtlebot3 models and executing `roslaunch`, make sure  
&nbsp;&nbsp;&nbsp;&nbsp; to terminate the **Gazebo** instance to generate a separate world when testing each turtlebot3 model
```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make && cd ~/catkin_ws/src
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```


## Reference Repos
https://github.com/robustify/igvc_self_drive_sim
https://github.com/westpoint-robotics/AY21_IGVC/tree/master/catkin_ws
