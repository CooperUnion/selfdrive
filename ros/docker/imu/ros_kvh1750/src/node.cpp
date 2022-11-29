/*
 * KVH 1750 IMU
 * Eric L. Hahn <erichahn@vt.edu>
 * 12/5/2014
 * Copyright 2014. All Rights Reserved.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <ros/ros.h>
#include <tf/tf.h>
#include <pluginlib/class_loader.h>
#pragma GCC diagnostic pop

#include "kvh1750/tov_file.h"
#include "kvh1750/kvh_plugin.h"
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>
#include <sched.h>

namespace
{
  const std::string DefaultImuLink = "torso";
  const std::string DefaultAddress = "/dev/ttyS4";
  int Rate;
  bool IsDA = true;
  double Ahrs_gyro_x = 0;
  double Ahrs_gyro_y = 0;
  double Ahrs_gyro_z = 0;
  double Prev_stamp = 0;
  boost::shared_ptr<kvh::MessageProcessorBase> Plugin;
}

/**
 * Converts KVH1750Message into the standard ROS messages corresponding
 * to the same set of data, namely an Imu and Temperature message.
 */
void to_ros(const kvh::Message& msg, sensor_msgs::Imu& imu,
  sensor_msgs::Temperature& temp)
{
  msg.time(imu.header.stamp.sec, imu.header.stamp.nsec);

  imu.angular_velocity.x = msg.gyro_x();
  imu.angular_velocity.y = msg.gyro_y();
  imu.angular_velocity.z = msg.gyro_z();
  imu.linear_acceleration.x = msg.accel_x();
  imu.linear_acceleration.y = msg.accel_y();
  imu.linear_acceleration.z = msg.accel_z();

  //scale for ROS if delta angles are enabled
  if(IsDA)
  {
    Ahrs_gyro_x += msg.gyro_x();
    Ahrs_gyro_y += msg.gyro_y();
    Ahrs_gyro_z += msg.gyro_z();

    imu.angular_velocity.x *= Rate;
    imu.angular_velocity.y *= Rate;
    imu.angular_velocity.z *= Rate;
  }
  else
  {
    double current_stamp = imu.header.stamp.sec + imu.header.stamp.nsec * 1E-9;
    double deltatime;
    if (Prev_stamp)
    {
      deltatime = current_stamp - Prev_stamp;
    }
    else
    {
      deltatime = 1/Rate;
    }
    Ahrs_gyro_x += msg.gyro_x()*deltatime;
    Ahrs_gyro_y += msg.gyro_y()*deltatime;
    Ahrs_gyro_z += msg.gyro_z()*deltatime;
    Prev_stamp = current_stamp;
  }

  imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(Ahrs_gyro_x,
    Ahrs_gyro_y, Ahrs_gyro_z);
  temp.header.stamp = imu.header.stamp;
  temp.temperature = msg.temp();
}

int main(int argc, char **argv)
{
  //Name of node
  ros::init(argc, argv, "kvh_1750_imu");
  //Node handle
  ros::NodeHandle nh("~");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Publisher temp_pub = nh.advertise<sensor_msgs::Temperature>("temp", 1);

  std::string imu_link_name = DefaultImuLink;
  nh.getParam("link_name", imu_link_name);

  std::string plugin_name = "";
  nh.getParam("processor_type", plugin_name);
  if(!plugin_name.empty())
  {
    pluginlib::ClassLoader<kvh::MessageProcessorBase> plugin_loader("kvh1750", "kvh::KVHMessageProcessorBase");
    Plugin = plugin_loader.createInstance(plugin_name);
    Plugin->set_link_name(imu_link_name);
  }

  nh.param("rate", Rate, 100);

  bool use_delta_angles = true;

  nh.getParam("use_delta_angles", use_delta_angles);

  sensor_msgs::Imu current_imu;
  sensor_msgs::Temperature current_temp;

  int priority = 99;
  bool use_rt = true;

  nh.getParam("priority", priority);
  nh.getParam("use_rt", use_rt);

  int policy = (use_rt ? SCHED_RR : SCHED_OTHER);

  priority = std::min(sched_get_priority_max(policy),
    std::max(sched_get_priority_min(policy), priority));

  struct sched_param params;
  params.sched_priority = (use_rt ? static_cast<int>(priority) : 0);

  int rc = sched_setscheduler(0, policy, &params);
  if(rc != 0)
  {
    ROS_ERROR("Setting schedule priority produced error: \"%s\"", strerror(errno));
    return 1;
  }

  std::vector<double> ahrs_cov;
  std::vector<double> ang_cov;
  std::vector<double> lin_cov;

  nh.param<std::vector<double>>("orientation_covariance",
    ahrs_cov, {1, 0, 0, 0, 1, 0, 0, 0, 1});
  std::copy(ahrs_cov.begin(), ahrs_cov.end(),
    current_imu.orientation_covariance.begin());

  if(nh.getParam("angular_covariance", ang_cov))
  {
    std::copy(ang_cov.begin(), ang_cov.end(),
      current_imu.angular_velocity_covariance.begin());
  }
  else
  {
    current_imu.angular_velocity_covariance[0] = 1;
    current_imu.angular_velocity_covariance[4] = 1;
    current_imu.angular_velocity_covariance[8] = 1;
  }

  if(nh.getParam("linear_covariance", lin_cov))
  {
    std::copy(lin_cov.begin(), lin_cov.end(),
      current_imu.linear_acceleration_covariance.begin());
  }
  else
  {
    current_imu.linear_acceleration_covariance[0] = 1;
    current_imu.linear_acceleration_covariance[4] = 1;
    current_imu.linear_acceleration_covariance[8] = 1;
  }

  //IMU link locations
  current_temp.header.frame_id = imu_link_name;
  current_imu.header.frame_id = imu_link_name;
  std::string addr = DefaultAddress;
  nh.getParam("address", addr);

  std::string tov_addr = "";
  nh.getParam("tov_address", tov_addr);

  uint32_t baud = 921600;
  int read_baud; //Because rosparam can't provide unsigned ints
  nh.getParam("baudrate", read_baud);
  baud = static_cast<uint32_t>(read_baud);

  int max_temp = kvh::MaxTemp_C;

  nh.getParam("max_temp", max_temp);

  uint32_t wait = 100;
  std::shared_ptr<kvh::IOModule> mod(new kvh::TOVFile(addr, baud, wait,
    tov_addr));
  kvh::IMU1750 imu(mod);

  imu.set_temp_limit(max_temp);
  if(!imu.set_angle_units(use_delta_angles))
  {
    ROS_ERROR("Could not set angle units.");
  }
  if(Rate > 0)
  {
    if(!imu.set_data_rate(Rate))
    {
      ROS_ERROR("Could not set data rate to %d", Rate);
    }
  }

  imu.query_data_rate(Rate);
  imu.query_angle_units(IsDA);

  bool keep_reading = true;
  while(ros::ok() && keep_reading)
  {
    kvh::Message msg;
    switch(imu.read(msg))
    {
      case kvh::IMU1750::VALID:
        to_ros(msg, current_imu, current_temp);
        if(Plugin)
        {
          Plugin->process_message(msg);
        }
        imu_pub.publish(current_imu);
        temp_pub.publish(current_temp);
        break;
      case kvh::IMU1750::BAD_READ:
      case kvh::IMU1750::BAD_CRC:
        ROS_ERROR("Bad data from KVH, ignoring.");
        break;
      case kvh::IMU1750::FATAL_ERROR:
        ROS_FATAL("Lost connection to IMU!"); //should reconnect
        keep_reading = false;
        break;
      case kvh::IMU1750::OVER_TEMP:
        ROS_FATAL("IMU is overheating!");
        keep_reading = false;
        break;
      case kvh::IMU1750::PARTIAL_READ:
      default:
      break;
    }
    ros::spinOnce();
  }

  return 0;
}
