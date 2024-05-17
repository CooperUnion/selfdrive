# Stuff that needs to be done to run the robot_localization properly

# Finish Setting up Sensors
1) set up the IMU in accordance with REP-103 such that:
    - X is forward
    - Y is left
    - Z is up

# Sensor Data Messages
1) odom data should be published such that:
    - message type: nav_msgs/Odometry

2) IMU data should be published such that:
    - message type is sensor_msgs/Imu

# Software Setup
1) Navigate to snesor_fusion/src/move/params/ekf.yaml

2) Most values are fine by default, ensure the following parameters are as follows:
    - two_d_mode: true
    - odom0: car_odom_topic_name
    - odom0_config: [true,  true,  false,
                    false, false, false,
                    false, false, false,
                    false, false, true,
                    false, false, false]
    - imu0: car_imu_topic_name
    - imu0_config: [false, false, false,
                   true,  true,  true,
                   false, false, false,
                   true,  true,  true,
                   true,  true,  true]

3) build and source install
4) ros2 launch robot_localization ekf.launch.py
5) filteded data should be published to /odometry/filtered as a nav_msgs/Odometry message

# Current Problems
1) Covariance is currently just kinda guessed
- IMU covariance may be calculable from the spec sheet on Yostlabs.com, but I would need help on that. spec sheet for similar device (https://yostlabs.com/product/3-space-watertight-usbrs232/)

- Odom covariance is a complete guess but may be calculable based on unit tests. Check the thread comments here for more info: (https://answers.ros.org/question/318445/imu-covariance-matrix-setting-for-robot_localization/)

2) Brightside
- Even with inaccurate covariance values, good enough results should still be possible

