#include <yostlab_imu/YostLabDriver.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "yostlab_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  YostLabDriver imu_drv(nh, priv_nh);
  imu_drv.run();
}
