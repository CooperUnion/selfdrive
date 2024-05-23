## How to Run Object Detection Stack

The object detection stack is a ros2 package. It can be built and ran like any other standard ros2 package. 

Run the following: 

```
colcon build
```

```
source install/setup.bash
```

```
ros2 run dist_to_obj_package dist_to_obj_node
```

## Errors that might occur on launch 

If the ZED camera is not plugged in the following error will pop up: 

```
Camera Open : CAMERA NOT DETECTED. Exit program.
```

Sometimes this error will pop up when trying to start object detection: 

```
NO GPU DETECTED
```

While it doesn't happen often in the even that it does starting the computer fixes it. 
