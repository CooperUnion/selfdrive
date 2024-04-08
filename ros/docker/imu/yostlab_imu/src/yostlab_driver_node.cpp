#include "../include/yostlab_imu/YostLabDriver.h"


int main(int argc, char **argv)
{
  // ros::init(argc, argv, "yostlab_driver_node");
  // ros::NodeHandle nh;
  // ros::NodeHandle priv_nh("~");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yostlab_driver_node");
  auto node_ptr = shared_from_this("yostlab_driver_node");  // source: https://answers.ros.org/question/330920/ros-2-composable-node-pass-nodehandler/
  shared_ptr = rclcpp::Node::make_shared<YostLabDriver>(node_ptr);

  YostLabDriver imu_drv(shared_ptr);
  imu_drv.run();
}
