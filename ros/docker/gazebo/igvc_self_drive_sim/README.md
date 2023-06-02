# IGVC Self-Drive Gazebo Simulator
This repository provides a Gazebo model of a Polaris GEM, as well as worlds and models to simulate the tasks of the competition.

**If using ROS Noetic**, you need to clone the [hector_models](https://github.com/tu-darmstadt-ros-pkg/hector_models.git) and [hector_gazebo](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git) repositories into the same workspace as this repositoriy.

After cloning into a ROS workspace along with `hector_gazebo` and `hector_models` if necessary, run the following the command from the root of the workspace to install additional ROS package dependencies to make everything work correctly:

`rosdep install --from-paths src --ignore-src -r`

### Starting the Simulator

`roslaunch gazebo_ros empty_world.launch`

To load a custom Gazebo world file, pass its file path to the `world_name` argument:

`roslaunch gazebo_ros empty_world.launch world_name:=<complete path to world file here\>`

Spawn the GEM model in Gazebo with `spawn_gem.launch` in the `igvc_self_drive_gazebo` package:

`roslaunch igvc_self_drive_gazebo spawn_gem.launch`

The `spawn_gem.launch` file has arguments that allow the user to control some aspects of the spawned GEM:

* `start_x`, `start_y`, `start_z`: The `x`, `y`, `z` position of the vehicle in Gazebo

* `start_yaw`: The direction the vehicle faces relative to the Gazebo world frame

* `twist_mode`: If true, the GEM subscribes to a `geometry_msgs/Twist` topic to move around the world. If false, it subscribes to individual actuator command topics. See the discussion of these modes below.

* `pub_tf`: If true, a ground truth TF transform from `world` frame to `base_footprint` frame is published

### Simulating IGVC Tasks

The `self_drive_tasks.world` file in the `igvc_self_drive_gazebo` package simulates the specific tasks as described in the official rules document, which can be found [[here](http://www.igvc.org/2018selfdriverules.pdf)].

To load the world and spawn the vehicle at the starting point for a particular task, run the corresponding launch file in the `igvc_self_drive_gazebo` package. For example, to run task F3, the right turn test where the vehicle has to stop at the intersection:

`roslaunch igvc_self_drive_gazebo f3_gazebo.launch`

At the moment, the F9 task is not implemented in the `self_drive_tasks.world` Gazebo world, even though the launch file for it exists.

### Sensor Data Topics
The following topics are published by Gazebo:

* `/fix`: GPS position given in latitude and longitude in the form of a `sensor_msgs/NavSatFix` message

* `/camera_front/image_raw/*`: Standard group of image topics from `image_proc`

* `/scan`: LIDAR scan data in the form of a `sensor_msgs/LaserScan` message

* `/twist`: The current measurement of the vehicle's speed and yaw rate in a `geometry_msgs/TwistStamped` message. The `linear.x` field contains the speed in m/s and `angular.z` contains the yaw rate in rad/s

* `/gear_state`: The current gear of the vehicle in a `std_msgs/UInt8` message:
    * 0 = Forward
    * 1 = Reverse

* `/sonar/*`: Group of 10 sonar sensor range measurement topics in separate `sensor_msgs/Range` messages. There are three sensors on the front of the vehicle, three on the rear, and two on either side.

### Controlling the Simulated Vehicle
The Gazebo GEM model can be controlled in two different modes:

* **Twist Mode:** Send a `geometry_msgs/Twist` message on the `/cmd_vel` topic with desired speed and yaw rate, and let the simulation generate the appropriate actuator commands.

* **Actuator Mode:** Send throttle, brake, steering, and gear actuator commands directly to the simulator.

#### Twist Mode (default)
The control mode is specified by setting the `twist_mode` argument to the `spawn_gem.launch` file to true or false. This argument defaults to true, so if you don't have to change anything to use Twist Mode.

The `linear.x` field of the user's `geometry_msgs/Twist` message should contain the desired speed in m/s, and the `angular.z` field should contain the desired yaw rate in rad/s.

#### Actuator Mode
By setting the `twist_mode` argument to false, the Gazebo plugin instead subscribes to four separate actuator command topics so the user can control them directly:

* `/throttle_cmd`: `std_msgs/Float64` topic containing commanded throttle percentage (0 to 1)

* `/brake_cmd`: `std_msgs/Float64` topic containing commanded brake torque in Newton-meters (0 to 1000)

* `/steering_cmd`: `std_msgs/Float64` topic containing commanded steering wheel angle in radians (-9.5 to +9.5)

* `/gear_cmd`: `std_msgs/UInt8` topic containing commanded gear, as controlled by the switch on the dashboard of the real vehicle:
    * 0 = Forward
    * 1 = Reverse

### Some Useful Kinematics Parameters

* Gear ratio between steering wheel and equivalent bicycle steer angle = 17 : 1

* Wheelbase = 2.4 meters

* Track width = 1.2 meters

* Wheel radius = 0.36 meters
