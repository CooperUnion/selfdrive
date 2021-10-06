# catkin workspace

## turtlebot gazebo simulation
```bash
$ sudo apt install -y ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
```


#### 1. trying out turtlebot with keyboard input
```bash
# terminal 1
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

# terminal 2
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```