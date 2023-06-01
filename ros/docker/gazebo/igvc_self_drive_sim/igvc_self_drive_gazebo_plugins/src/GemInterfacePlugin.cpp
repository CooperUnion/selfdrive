#include "GemInterfacePlugin.h"

namespace gazebo
{

GemInterfacePlugin::GemInterfacePlugin()
{
  target_angle_ = 0.0;
  current_steer_angle_ = 0.0;
  brake_cmd_ = 0.0;
  throttle_cmd_ = 0.0;
  rollover_ = false;
  gear_state_.data = GEAR_FORWARD;
  pub_tf_ = false;
  twist_control_mode_ = false;
}

void GemInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  ROS_INFO("Loading GEM interface plugin");

  // Gazebo initialization
  steer_fl_joint_ = model->GetJoint("steer_fl_joint");
  steer_fr_joint_ = model->GetJoint("steer_fr_joint");
  wheel_rl_joint_ = model->GetJoint("wheel_rl_joint");
  wheel_rr_joint_ = model->GetJoint("wheel_rr_joint");
  wheel_fl_joint_ = model->GetJoint("wheel_fl_joint");
  wheel_fr_joint_ = model->GetJoint("wheel_fr_joint");
  model_ = model;
  footprint_link_ = model->GetLink("base_footprint");

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GemInterfacePlugin::OnUpdate, this, _1));

  steer_fr_joint_->SetParam("fmax", 0, 99999.0);
  steer_fl_joint_->SetParam("fmax", 0, 99999.0);

  if (sdf->HasElement("pubTf")) {
    sdf->GetElement("pubTf")->GetValue()->Get(pub_tf_);
  }

  if (sdf->HasElement("twistMode")) {
    sdf->GetElement("twistMode")->GetValue()->Get(twist_control_mode_);
  }

  // ROS initialization
  n_ = new ros::NodeHandle();

  if (twist_control_mode_) {
    sub_cmd_vel_ = n_->subscribe("cmd_vel", 1, &GemInterfacePlugin::recvCmdVel, this);
  } else {
    sub_steering_cmd_ = n_->subscribe("steering_cmd", 1, &GemInterfacePlugin::recvSteeringCmd, this);
    sub_brake_cmd_ = n_->subscribe("brake_cmd", 1, &GemInterfacePlugin::recvBrakeCmd, this);
    sub_throttle_cmd_ = n_->subscribe("throttle_cmd", 1, &GemInterfacePlugin::recvThrottleCmd, this);
    sub_gear_shift_cmd_ = n_->subscribe("gear_cmd", 1, &GemInterfacePlugin::recvGearCmd, this);
  }

  pub_twist_ = n_->advertise<geometry_msgs::TwistStamped>("twist", 1);
  pub_gear_state_ = n_->advertise<std_msgs::UInt8>("gear_state", 1, true);
  pub_gear_state_.publish(gear_state_);

  twist_timer_ = n_->createTimer(ros::Duration(0.02), &GemInterfacePlugin::twistTimerCallback, this);
  if (pub_tf_) {
    tf_timer_ = n_->createTimer(ros::Duration(0.01), &GemInterfacePlugin::tfTimerCallback, this);
  }
}

void GemInterfacePlugin::OnUpdate(const common::UpdateInfo& info)
{
  if (twist_control_mode_) {
    twistControlUpdate();
  }
  steeringUpdate(info);
  driveUpdate();

#if GAZEBO_MAJOR_VERSION >= 9
  twist_.linear.x = footprint_link_->RelativeLinearVel().X();
  twist_.angular.z = footprint_link_->RelativeAngularVel().Z();

  // Detect rollovers
  world_pose_ = footprint_link_->WorldPose();
  rollover_ = (fabs(world_pose_.Rot().X()) > 0.2 || fabs(world_pose_.Rot().Y()) > 0.2);
#else
  twist_.linear.x = footprint_link_->GetRelativeLinearVel().x;
  twist_.angular.z = footprint_link_->GetRelativeAngularVel().z;

  // Detect rollovers
  world_pose_ = footprint_link_->GetWorldPose();
  rollover_ = (fabs(world_pose_.rot.x) > 0.2 || fabs(world_pose_.rot.y) > 0.2);
#endif
}

void GemInterfacePlugin::twistControlUpdate()
{
  // Timeout after 0.25 seconds
  if ((ros::Time::now() - cmd_vel_stamp_).toSec() > 0.25) {
    std_msgs::Float64 zero;
    throttle_cmd_ = 0;
    brake_cmd_ = 0;
    target_angle_ = 0;
    int_throttle_ = 0;
    return;
  }

  // Automatically shift gears to support both positive and negative command speeds
  double speed_target = cmd_vel_.linear.x;
  switch (gear_state_.data) {
    case GEAR_FORWARD:
      if (cmd_vel_.linear.x < 0) {
        speed_target = 0;
        if (twist_.linear.x <= 0.01) {
          gear_state_.data = GEAR_REVERSE;
          pub_gear_state_.publish(gear_state_);
        }
      }
      break;
    case GEAR_REVERSE:
      if (cmd_vel_.linear.x > 0) {
        speed_target = 0;
        if (twist_.linear.x >= -0.01) {
          gear_state_.data = GEAR_FORWARD;
          pub_gear_state_.publish(gear_state_);
        }
      }
      break;
  }

  // Enable control of brake, throttle, and steering by syncing command time stamps
  brake_stamp_ = cmd_vel_stamp_;
  throttle_stamp_ = cmd_vel_stamp_;
  steering_stamp_ = cmd_vel_stamp_;

  // Regulate speed with throttle and brake
  const double throttle_kp = 0.5;
  const double throttle_ki = 0.15;
  const double throttle_int_sat = 0.5;
  const double brake_kp = 120.0;

  double speed_error = speed_target - twist_.linear.x;
  if (gear_state_.data == GEAR_REVERSE) {
    speed_error *= -1;
  }
  throttle_cmd_ = throttle_kp * speed_error + throttle_ki * int_throttle_;

  if (throttle_cmd_ >= 1.0) {
    throttle_cmd_ = 1.0;
    brake_cmd_ = 0;
  } else if (throttle_cmd_ <= 0) {
    throttle_cmd_ = 0;
    brake_cmd_ = -brake_kp * speed_error;
    if (brake_cmd_ > MAX_BRAKE_TORQUE) {
      brake_cmd_ = MAX_BRAKE_TORQUE;
    } else if (brake_cmd_ < 0) {
      brake_cmd_ = 0;
    }
  } else {
    int_throttle_ += 0.02 *  speed_error;
    if (int_throttle_ > throttle_int_sat) {
      int_throttle_ = throttle_int_sat;
    } else if (int_throttle_ < -throttle_int_sat) {
      int_throttle_ = -throttle_int_sat;
    }
    brake_cmd_ = 0;
  }

  // Open loop kinematic control of steering
  if (fabs(twist_.linear.x) < 0.05) {
    target_angle_ = 0.0;
  } else {
    target_angle_ = atan(WHEELBASE * cmd_vel_.angular.z / twist_.linear.x);
  }
}

void GemInterfacePlugin::driveUpdate()
{
  // Stop wheels if vehicle is rolled over
  if (rollover_) {
    wheel_rl_joint_->SetForce(0, -1000.0 * wheel_rl_joint_->GetVelocity(0));
    wheel_rr_joint_->SetForce(0, -1000.0 * wheel_rr_joint_->GetVelocity(0));
    wheel_fl_joint_->SetForce(0, -1000.0 * wheel_fl_joint_->GetVelocity(0));
    wheel_fr_joint_->SetForce(0, -1000.0 * wheel_fr_joint_->GetVelocity(0));
    return;
  }

  // Brakes have precedence over throttle
  ros::Time current_stamp = ros::Time::now();
  if ((brake_cmd_ > 0) && ((current_stamp - brake_stamp_).toSec() < 0.25)) {
    double brake_adjust_factor = 1.0;
    if (twist_.linear.x < -0.1) {
      brake_adjust_factor = -1.0;
    } else if (twist_.linear.x < 0.1) {
      brake_adjust_factor = 1.0 + (twist_.linear.x - 0.1) / 0.1;
    }

    setWheelTorque(-brake_adjust_factor * brake_cmd_);
  } else {
    if ((current_stamp - throttle_stamp_).toSec() < 0.25) {
      // Sigmoid function approximating torque dropoff as max speed is reached
      double throttle_torque = throttle_cmd_ * 500.0 * (1 - 1 / (1 + exp(-1.5 * (fabs(twist_.linear.x) - 9.7))));
      if (throttle_torque < 0.0) {
        throttle_torque = 0.0;
      }

      switch (gear_state_.data) {
        case GEAR_FORWARD:
          setWheelTorque(throttle_torque);
          break;
        case GEAR_REVERSE:
          setWheelTorque(-throttle_torque);
          break;
      }
    }
  }

  // Rolling resistance
  double rolling_resistance_torque = ROLLING_RESISTANCE_COEFF * MASS * G;
  if (twist_.linear.x > 0.0) {
    setWheelTorque(-rolling_resistance_torque);
  } else {
    setWheelTorque(rolling_resistance_torque);
  }
}

void GemInterfacePlugin::setWheelTorque(double torque)
{
  wheel_rl_joint_->SetForce(0, 0.25 * torque);
  wheel_rr_joint_->SetForce(0, 0.25 * torque);
  wheel_fl_joint_->SetForce(0, 0.25 * torque);
  wheel_fr_joint_->SetForce(0, 0.25 * torque);
}

void GemInterfacePlugin::steeringUpdate(const common::UpdateInfo& info)
{
  if ((ros::Time::now() - steering_stamp_).toSec() > 0.25) {
    target_angle_ = 0.0;
  }

  double time_step = (info.simTime - last_update_time_).Double();
  last_update_time_ = info.simTime;

  // Arbitrarily set maximum steering rate to 800 deg/s
  const double max_rate = 800.0 * M_PI / 180.0 / STEERING_RATIO;
  double max_inc = time_step * max_rate;

  if ((target_angle_ - current_steer_angle_) > max_inc) {
    current_steer_angle_ += max_inc;
  } else if ((target_angle_ - current_steer_angle_) < -max_inc) {
    current_steer_angle_ -= max_inc;
  }

  // Compute Ackermann steering angles for each wheel
  double t_alph = tan(current_steer_angle_);
  double left_steer = atan(WHEELBASE * t_alph / (WHEELBASE - 0.5 * TRACK_WIDTH * t_alph));
  double right_steer = atan(WHEELBASE * t_alph / (WHEELBASE + 0.5 * TRACK_WIDTH * t_alph));

#if GAZEBO_MAJOR_VERSION >= 9
  steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->Position(0)));
  steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->Position(0)));
#else
  steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->GetAngle(0).Radian()));
  steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->GetAngle(0).Radian()));
#endif

}

void GemInterfacePlugin::recvSteeringCmd(const std_msgs::Float64ConstPtr& msg)
{
  steering_stamp_ = ros::Time::now();
  if (!std::isfinite(msg->data)) {
    ROS_WARN_THROTTLE(0.5, "Steering command is NaN!");
    target_angle_ = 0.0;
    return;
  }

  target_angle_ = msg->data / STEERING_RATIO;
  if (target_angle_ > MAX_STEERING_ANGLE) {
    target_angle_ = MAX_STEERING_ANGLE;
  } else if (target_angle_ < -MAX_STEERING_ANGLE) {
    target_angle_ = -MAX_STEERING_ANGLE;
  }
}

void GemInterfacePlugin::recvBrakeCmd(const std_msgs::Float64ConstPtr& msg)
{
  if (msg->data < 0) {
    brake_cmd_ = 0;
  } else if (msg->data > MAX_BRAKE_TORQUE) {
    brake_cmd_ = MAX_BRAKE_TORQUE;
  } else {
    brake_cmd_ = msg->data;
  }
  brake_stamp_ = ros::Time::now();
}

void GemInterfacePlugin::recvThrottleCmd(const std_msgs::Float64ConstPtr& msg)
{
  throttle_cmd_ = msg->data;
  if (throttle_cmd_ < 0.0) {
    throttle_cmd_ = 0.0;
  } else if (throttle_cmd_ > 1.0) {
    throttle_cmd_ = 1.0;
  }
  throttle_stamp_ = ros::Time::now();
}

void GemInterfacePlugin::recvGearCmd(const std_msgs::UInt8ConstPtr& msg)
{
  if (msg->data == GEAR_FORWARD && gear_state_.data != GEAR_FORWARD) {
    gear_state_.data = GEAR_FORWARD;
    pub_gear_state_.publish(gear_state_);
  } else if (msg->data == GEAR_REVERSE && gear_state_.data != GEAR_REVERSE) {
    gear_state_.data = GEAR_REVERSE;
    pub_gear_state_.publish(gear_state_);
  }
}

void GemInterfacePlugin::recvCmdVel(const geometry_msgs::TwistConstPtr& msg)
{
  cmd_vel_ = *msg;
  cmd_vel_stamp_ = ros::Time::now();
}

void GemInterfacePlugin::twistTimerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = "base_footprint";
  twist_msg.header.stamp = event.current_real;
  twist_msg.twist.linear.x = twist_.linear.x;
  twist_msg.twist.angular.z = twist_.angular.z;
  pub_twist_.publish(twist_msg);
}

void GemInterfacePlugin::tfTimerCallback(const ros::TimerEvent& event)
{
  tf::StampedTransform transform;
  transform.stamp_ = event.current_real;
  transform.frame_id_ = "world";
  transform.child_frame_id_ = footprint_link_->GetName();
#if GAZEBO_MAJOR_VERSION >= 9
  transform.setOrigin(tf::Vector3(world_pose_.Pos().X(), world_pose_.Pos().Y(), world_pose_.Pos().Z()));
  transform.setRotation(tf::Quaternion(world_pose_.Rot().X(), world_pose_.Rot().Y(), world_pose_.Rot().Z(), world_pose_.Rot().W()));
#else
  transform.setOrigin(tf::Vector3(world_pose_.pos.x, world_pose_.pos.y, world_pose_.pos.z));
  transform.setRotation(tf::Quaternion(world_pose_.rot.x, world_pose_.rot.y, world_pose_.rot.z, world_pose_.rot.w));
#endif
  tf_broadcaster_.sendTransform(transform);
}

void GemInterfacePlugin::Reset()
{
}

GemInterfacePlugin::~GemInterfacePlugin()
{
  n_->shutdown();
  delete n_;
}


}
