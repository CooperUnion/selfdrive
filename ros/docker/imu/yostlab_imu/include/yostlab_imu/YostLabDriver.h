#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <yostlab_imu/SerialInterface.h>
// This is the  basic ros-based device driver of IMU
class YostLabDriver: SerialInterface
{
public:
  //! constructor and destructor
  YostLabDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_);
  ~YostLabDriver();
  //!
  //! \brief run: runs system
  //!
  void run(void);
  //!
  //! \brief getSoftwareVersion
  //! \return returns software string version
  //!
  const std::string getSoftwareVersion( void );
  //!
  //! \brief restoreFactorySettings resets everything
  //!
  void restoreFactorySettings( void );
  //!
  //! \brief getAxisDirection
  //! \return returns axis directions
  //!
  const std::string getAxisDirection( void );
  //!
  //! \brief getEulerDecomp
  //! \return decompisition string
  //!
  const std::string getEulerDecomp( void );
  //!
  //! \brief getCalibMode
  //! \return 0 for bias 1 for scale and bias
  //!
  const std::string getCalibMode(void);
  //!
  //! \brief getMIMode
  //! \return 1 if enabled 0 if disabled
  //!
  const std::string getMIMode(void);
  //!
  //! \brief startGyroCalibration
  //!
  void startGyroCalibration(void);
  //!
  //! \brief setMIMode
  //! \param on True to set , False to Unset
  //!
  void setMIMode(bool on);
private:
  //! Node Handlers
  ros::NodeHandle yostlab_nh_;
  ros::NodeHandle yostlab_priv_nh_;
  ros::Publisher imu_pub_;
  //! Orientation Commands
  static constexpr auto GET_TARED_ORIENTATION_AS_QUATERNION        = ":0\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_EULER_ANGLES      = ":1\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_ROTATION_MATRIX   = ":2\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_AXIS_ANGLE        = ":3\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_TWO_VECTOR        = ":4\n";
  static constexpr auto GET_DIFFERENCE_QUATERNION                  = ":5\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_QUATERNION      = ":6\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_EULER_ANGLES    = ":7\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = ":8\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE      = ":9\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_TWO_VECTOR      = ":10\n";
  static constexpr auto GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME       = ":11\n";
  static constexpr auto GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME     = ":12\n";
  //! Corrected Data Commands
  static constexpr auto GET_ALL_CORRECTED_COMPONENT_SENSOR = ":37\n";
  static constexpr auto GET_CORRECTED_GYRO_RATE            = ":38\n";
  static constexpr auto GET_CORRECTED_ACCELEROMETER_VECTOR = ":39\n";
  static constexpr auto GET_CORRECTED_COMPASS_VECTOR       = ":40\n";
  static constexpr auto GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = ":41\n";
  static constexpr auto CORRECT_RAW_GYRO_DATA    = ":48\n";
  static constexpr auto CORRECT_RAW_ACCEL_DATA   = ":49\n";
  static constexpr auto CORRECT_RAW_COMPASS_DATA = ":50\n";
  //! Other Data Commands
  static constexpr auto GET_TEMPERATURE_C     = ":43\n";
  static constexpr auto GET_TEMPERATURE_F     = ":44\n";
  static constexpr auto GET_CONFIDENCE_FACTOR = ":45\n";
  //! RAW Data Commands
  static constexpr auto GET_ALL_RAW_COMPONENT_SENSOR_DATA = ":64\n";
  static constexpr auto GET_RAW_GYRO_RATE                 = ":65\n";
  static constexpr auto GET_RAW_ACCEL_DATA                = ":66\n";
  static constexpr auto GET_RAW_COMPASS_DATA              = ":67\n";
  //! Streaming Commands
  static constexpr auto SET_STREAMING_SLOTS_EULER_TEMP        = ":80,1,43,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_EULER_QUATERNION  = ":80,1,0,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_EULER  = ":80,0,1,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_EULER             = ":80,1,255,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION        = ":80,0,255,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR_IN_GLOBAL
  = ":80,0,38,41,255,255,255,255,255\n";
  static constexpr auto GET_STREAMING_SLOTS                   = ":81\n";
  static constexpr auto SET_STREAMING_TIMING_100_MS           = ":82,100000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_1000_MS          = ":82,1000000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_5000_MS          = ":82,5000000,0,0\n";
  static constexpr auto GET_STREAMING_TIMING                  = ":83\n";
  static constexpr auto GET_STREAMING_BATCH                   = ":84\n";
  static constexpr auto START_STREAMING                       = ":85\n";
  static constexpr auto STOP_STREAMING                        = ":86\n";
  static constexpr auto UPDATE_CURRENT_TIMESTAMP              = ":95\n";
  //! Configuration Read Commands
  static constexpr auto GET_AXIS_DIRECTION           = ":143\n";
  static constexpr auto GET_FILTER_MODE              = ":152\n";
  static constexpr auto GET_EULER_DECOMPOSTION_ORDER = ":156\n";
  static constexpr auto GET_MI_MODE_ENABLED          = ":136\n";
  //! Configuration Write Commands
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_XYZ = ":16,0\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_YZX = ":16,1\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_ZXY = ":16,2\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_ZYX = ":16,3\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_XZY = ":16,4\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_YXZ = ":16,5\n";
  static constexpr auto OFFSET_WITH_CURRENT_ORIENTATION  = ":19\n";
  static constexpr auto TARE_WITH_CURRENT_ORIENTATION    = ":96\n";
  static constexpr auto TARE_WITH_CURRENT_QUATERNION     = ":97\n";
  static constexpr auto SET_MI_MODE_ENABLED = ":112,1\n";
  static constexpr auto SET_MI_MODE_DISABLED = ":112,0\n";
  static constexpr auto BEGIN_MI_MODE_FIELD_CALIBRATION = ":114\n";
  static constexpr auto SET_AXIS_DIRECTIONS_ENU     = ":116,8\n";
  static constexpr auto SET_AXIS_DIRECTIONS_DEFAULT = ":116,5\n";
  //! Calibration Commands
  static constexpr auto BEGIN_GYRO_AUTO_CALIB       = ":165\n";
  static constexpr auto SET_CALIB_MODE_BIAS         = ":169,0\n";
  static constexpr auto SET_CALIB_MODE_SCALE_BIAS   = ":169,1\n";
  static constexpr auto GET_CALIB_MODE              = ":170\n";
  //! System Commands
  static constexpr auto GET_FIRMWARE_VERSION_STRING = ":223\n";
  static constexpr auto RESTORE_FACTORY_SETTINGS    = ":224\n";
  static constexpr auto SOFTWARE_RESET              = ":226\n";
  //! logger space
  static constexpr auto logger = "[ YostImuDriver ] ";
}; // YostLabDriver

