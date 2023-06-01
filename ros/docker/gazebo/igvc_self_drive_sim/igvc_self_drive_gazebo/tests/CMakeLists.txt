# Check all the launch/*.launch files
find_package(roslaunch REQUIRED)
roslaunch_add_file_check(../launch)
