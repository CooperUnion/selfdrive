# Yostlab IMU driver

Yostlab IMU Driver for ROS see: https://yostlabs.com/3-space-sensors/

## How to run
There is a launch file to run driver node

    roslaunch yostlab_imu yost_imu_test.launch

You can visualize orientation of imu via rviz

    rosrun rviz rviz -d rviz/imu.rviz

## Topics
Published topics: /imu <sensor_msgs::Imu>
Subscribed topics: None
