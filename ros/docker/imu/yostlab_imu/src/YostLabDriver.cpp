#include <yostlab_imu/YostLabDriver.h>
#include <tf/tf.h>


YostLabDriver::YostLabDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_):
  SerialInterface(priv_nh_),
  yostlab_priv_nh_(priv_nh_),
  yostlab_nh_(nh_)
{
  this->SerialConnect();
  this->imu_pub_ = this->yostlab_nh_.advertise<sensor_msgs::Imu>("/imu", 10);
}


//! Destructor
YostLabDriver::~YostLabDriver()
{
}
void YostLabDriver::restoreFactorySettings()
{
  this->SerialWriteString(RESTORE_FACTORY_SETTINGS);
}
const std::string YostLabDriver::getSoftwareVersion()
{
  this->SerialWriteString(GET_FIRMWARE_VERSION_STRING);
  const std::string buf = this->SerialReadLine();
  ROS_INFO_STREAM(this->logger << "Software version: " << buf);
  return buf;
}

const std::string YostLabDriver::getEulerDecomp()
{
  this->SerialWriteString(GET_EULER_DECOMPOSTION_ORDER);
  const std::string buf = this->SerialReadLine();
  const std::string ret_buf = [&]()
  {
  if(buf == "0\r\n")
    return "XYZ";
  else if ( buf == "1\r\n")
    return "YZX";
  else if ( buf == "2\r\n")
    return "ZXY";
  else if (buf == "3\r\n")
    return "ZYX";
  else if( buf == "4\r\n")
    return "XZY";
  else if( buf == "5\r\n")
    return "YXZ";
  else
    return "Unknown";
  }();
  ROS_INFO_STREAM(this->logger << "Euler Decomposition: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

const std::string YostLabDriver::getAxisDirection()
{
  this->SerialWriteString(GET_AXIS_DIRECTION);
  const std::string buf = this->SerialReadLine();
  const std::string ret_buf = [&]()
  {
  if(buf == "0\r\n")
    return "X: Right, Y: Up, Z: Forward";
  else if ( buf == "1\r\n")
    return "X: Right, Y: Forward, Z: Up";
  else if ( buf == "2\r\n")
    return "X: Up, Y: Right, Z: Forward";
  else if (buf == "3\r\n")
    return "X: Forward, Y: Right, Z: Up";
  else if( buf == "4\r\n")
    return "X: Up, Y: Forward, Z: Right";
  else if( buf == "5\r\n")
    return "X: Forward, Y: Up, Z: Right";
  else
    return "Unknown";
  }();
  ROS_INFO_STREAM(this->logger << "Axis Direction: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

void YostLabDriver::startGyroCalibration(void)
{
  ROS_INFO_STREAM(this->logger << "Starting Auto Gyro Calibration");
  this->SerialWriteString(BEGIN_GYRO_AUTO_CALIB);
  ros::Duration(5.0).sleep();
}

void YostLabDriver::setMIMode(bool on)
{
  if(on)
    this->SerialWriteString(SET_MI_MODE_ENABLED);
  else
    this->SerialWriteString(SET_MI_MODE_DISABLED);
}

const std::string YostLabDriver::getCalibMode()
{
  this->SerialWriteString(GET_CALIB_MODE);
  const std::string buf = this->SerialReadLine();
  const std::string ret_buf = [&]()
  {
  if(buf == "0\r\n")
    return "Bias";
  else if ( buf == "1\r\n")
    return "Scale and Bias";
  else
    return "Unknown";
  }();
  ROS_INFO_STREAM(this->logger << "Calibration Mode: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}

const std::string YostLabDriver::getMIMode()
{
  this->SerialWriteString(GET_MI_MODE_ENABLED);
  const std::string buf = this->SerialReadLine();
  const std::string ret_buf = [&]()
  {
  if(buf == "0\r\n")
    return "Disabled";
  else if ( buf == "1\r\n")
    return "Enabled";
  else
    return "Unknown";
  }();
  ROS_INFO_STREAM(this->logger << "MI Mode: " << ret_buf << ", buf is: " << buf);
  return ret_buf;
}


//! Run the serial sync
void YostLabDriver::run()
{
  this->startGyroCalibration();
  this->getSoftwareVersion();
  this->getAxisDirection();
  this->getEulerDecomp();
  this->getCalibMode();
  this->getMIMode();
  this->SerialWriteString(SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR_IN_GLOBAL);
  this->SerialWriteString(TARE_WITH_CURRENT_ORIENTATION);
  this->SerialWriteString(TARE_WITH_CURRENT_QUATERNION);
  this->SerialWriteString(SET_STREAMING_TIMING_100_MS);
  this->SerialWriteString(START_STREAMING);
  ros::Rate loop_rate(20);
  int line_num_ = 0;
  sensor_msgs::Imu imu_msg_;
  std::vector<double> parsed_val_;
  while(ros::ok())
  {
    while( this->Available() > 0 )
    {
      line_num_ += 1;
      std::string buf = this->SerialReadLine();
      std::string parse_buf_;
      std::stringstream ss(buf);
      double i;
      while (ss >> i)
      {
        parsed_val_.push_back(i);
        if (ss.peek() == ',')
          ss.ignore();
      }

      if(line_num_ == 3)
      {
        line_num_ = 0;
        imu_msg_.header.stamp    = ros::Time::now();
        imu_msg_.header.frame_id = "imu_link";
        imu_msg_.orientation.x = parsed_val_[0];
        imu_msg_.orientation.y = parsed_val_[1];
        imu_msg_.orientation.z = parsed_val_[2];
        imu_msg_.orientation.w = parsed_val_[3];
        imu_msg_.angular_velocity.x = parsed_val_[4];
        imu_msg_.angular_velocity.y = parsed_val_[5];
        imu_msg_.angular_velocity.z = parsed_val_[6];
        imu_msg_.linear_acceleration.x = parsed_val_[7];
        imu_msg_.linear_acceleration.y = parsed_val_[8];
        imu_msg_.linear_acceleration.z = parsed_val_[9];
        parsed_val_.clear();
        this->imu_pub_.publish(imu_msg_);
        tf::Quaternion q( imu_msg_.orientation.x,
                          imu_msg_.orientation.y,
                          imu_msg_.orientation.z,
                          imu_msg_.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO_THROTTLE(1.0, "[ YostLabImuDriver ] roll: %f, pitch: %f, yaw: %f ", roll,pitch,yaw);
      }

    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}

