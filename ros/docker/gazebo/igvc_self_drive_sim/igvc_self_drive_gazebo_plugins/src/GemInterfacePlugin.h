#ifndef GEMINTERFACEPLUGIN_H_
#define GEMINTERFACEPLUGIN_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{

#define STEERING_RATIO            17.0
#define WHEELBASE                 2.4
#define TRACK_WIDTH               1.2
#define MAX_BRAKE_TORQUE          2000.0 // N-m
#define MAX_STEERING_ANGLE        0.5617 // min turning radius = 3.81 m
#define MASS                      584.0
#define G                         9.81
#define ROLLING_RESISTANCE_COEFF  0.01

enum {GEAR_FORWARD=0, GEAR_REVERSE};

class GemInterfacePlugin : public ModelPlugin
{
  public:
    GemInterfacePlugin();
    virtual ~GemInterfacePlugin();

  protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void Reset();

  private:
    void twistTimerCallback(const ros::TimerEvent& event);
    void tfTimerCallback(const ros::TimerEvent& event);
    void OnUpdate(const common::UpdateInfo& info);
    void recvSteeringCmd(const std_msgs::Float64ConstPtr& msg);
    void recvThrottleCmd(const std_msgs::Float64ConstPtr& msg);
    void recvBrakeCmd(const std_msgs::Float64ConstPtr& msg);
    void recvGearCmd(const std_msgs::UInt8ConstPtr& msg);
    void recvCmdVel(const geometry_msgs::TwistConstPtr& msg);
    void steeringUpdate(const common::UpdateInfo& info);
    void driveUpdate();
    void twistControlUpdate();
    void setWheelTorque(double torque);

    ros::NodeHandle* n_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_gear_state_;
    ros::Subscriber sub_steering_cmd_;
    ros::Subscriber sub_throttle_cmd_;
    ros::Subscriber sub_brake_cmd_;
    ros::Subscriber sub_gear_shift_cmd_;
    ros::Subscriber sub_cmd_vel_;
    ros::Timer twist_timer_;
    ros::Timer tf_timer_;
    tf::TransformBroadcaster tf_broadcaster_;

    physics::ModelPtr model_;
    geometry_msgs::Twist twist_;
    bool rollover_;
    bool pub_tf_;
    bool twist_control_mode_;
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d world_pose_;
#else
    math::Pose world_pose_;
#endif
    common::Time last_update_time_;
    ros::Time last_model_update_stamp_;
    event::ConnectionPtr update_connection_;
    physics::JointPtr steer_fl_joint_;
    physics::JointPtr steer_fr_joint_;
    physics::JointPtr wheel_rl_joint_;
    physics::JointPtr wheel_rr_joint_;
    physics::JointPtr wheel_fl_joint_;
    physics::JointPtr wheel_fr_joint_;
    physics::LinkPtr footprint_link_;

    // Steering values
    double current_steer_angle_;
    double target_angle_;
    ros::Time steering_stamp_;

    // Brakes
    double brake_cmd_;
    ros::Time brake_stamp_;

    // Throttle
    double throttle_cmd_;
    ros::Time throttle_stamp_;

    // Gear
    std_msgs::UInt8 gear_state_;

    // Twist control
    geometry_msgs::Twist cmd_vel_;
    ros::Time cmd_vel_stamp_;
    double int_throttle_;
};

GZ_REGISTER_MODEL_PLUGIN(GemInterfacePlugin)

}

#endif // GEMINTERFACEPLUGIN_H_
